import logging
import time
from queue import PriorityQueue

from colorama import Fore, Style
from gurobipy import GRB, quicksum

from GeneralHelper import *
from NodeInfo import NodeInfo
from PSP import PSP
from RMP import RMP

node_id_counter = 0
node_infos = {}
route_dict = {}  # key (truck_route, drone_route)
route_key_id_pairs = {}  # key: route key. value: route id


class BranchAndPrice:
    def __init__(self, trans_net, tolerance=1e-6, max_runtime=float('inf')):
        self.net = trans_net
        self.tolerance = tolerance
        self.max_runtime = max_runtime
        self.branch_queue = PriorityQueue()
        self.global_upper_bound = float('inf')  # the cost for incumbent solution
        self.best_solution = None
        # add initial routes
        self.initial_route = []
        for route_key, route in find_initial_routes(trans_net):
            route_dict[route_key] = route
            route_key_id_pairs[route_key] = route['id']
            self.initial_route.append(route_key)
        self.master_problem = RMP()

    def solve(self):
        """Main branch-and-price loop"""
        start_time = time.time()
        global node_id_counter
        # add root node
        node = self.master_problem.model
        node_id_counter += 1
        node_id = node_id_counter
        node_infos[node_id] = NodeInfo(node, -1)
        node_infos[node_id].columns = self.initial_route
        self.branch_queue.put(node_id)

        # branch
        while not self.branch_queue.empty():
            if time.time() - start_time > self.max_runtime:
                logging.info("Terminated due to reaching maximum runtime.")
                break

            node_id = self.branch_queue.get()
            self.master_problem.model = node_infos[node_id].model
            # solve the current node
            need_branch = self.solve_node(node_id)
            # update the node info

            if need_branch:
                self.branch(node_id)

        # terminated
        final_solution, cost = self.construct_final_route()
        sdas = 0

    def add_column_to_master(self, route_key, node_id):
        """Add new column (route) to the master problem"""
        route = route_dict[route_key]
        self.master_problem.z_keys.append(route_key)
        # generate a new variable and assign the coefficient to objective function
        var = self.master_problem.model.addVar(obj=route['cost'], name=f"z_{route['id']}", vtype=GRB.CONTINUOUS)
        self.master_problem.z_list.append(var)

        # customer must be served once
        for i, n_name in enumerate(self.net.customers):
            theta = int(n_name in route['truck'] or n_name in route['drone'])
            self.master_problem.model.chgCoeff(self.master_problem.constraints[i], var, theta)

        # truck fleet constraints
        self.master_problem.model.chgCoeff(self.master_problem.constraints[-1], var, 1)
        # update model
        self.master_problem.model.update()
        # update node_infos
        node_infos[node_id].columns.append(route_key) if route_key not in node_infos[node_id].columns else None

    def branch(self, parent_id):
        global node_id_counter
        parent_node = node_infos[parent_id]
        fraction_rhs = self.master_problem.sum_z_val
        # node 1: Add constraint sum_z <= floor(sum_z_val)
        node_left = self.master_problem.model.copy()
        node_id_counter += 1
        node_id = node_id_counter
        node_infos[node_id] = NodeInfo(node_left, parent_id)
        node_infos[node_id].columns, node_infos[
            node_id].branches = parent_node.columns.copy(), parent_node.branches.copy()
        # add branching constraint
        z_var_names = [var.VarName for var in self.master_problem.z_vars]
        cons_name = f"branch_{node_id}"
        node_left.addConstr(quicksum(node_left.getVarByName(var.VarName) for var in self.master_problem.z_vars) <= int(
            fraction_rhs), cons_name)
        node_infos[node_id].branches.append((z_var_names, int(fraction_rhs), cons_name))
        self.branch_queue.put(node_id)

        # node 2: Add constraint var >= ceil(fractional_var.X)
        node_right = self.master_problem.model.copy()
        node_id_counter += 1
        node_id = node_id_counter
        node_infos[node_id] = NodeInfo(node_right, parent_id)
        node_infos[node_id].columns, node_infos[
            node_id].branches = parent_node.columns.copy(), parent_node.branches.copy()
        # add branching constraint
        cons_name = f"branch_{node_id}"
        node_right.addConstr(
            quicksum(node_right.getVarByName(var.VarName) for var in self.master_problem.z_vars) >= int(
                fraction_rhs) + 1, cons_name)
        node_infos[node_id].branches.append((z_var_names, int(fraction_rhs) + 1, cons_name))
        self.branch_queue.put(node_id)

    def is_integer_solution(self):
        """Check if the RMP current solution is integer"""
        if self.master_problem.model.Status == GRB.OPTIMAL:
            for var in self.master_problem.model.getVars():
                if abs(var.X - round(var.X)) > close_tolerance:
                    return False, var
        return True, None

    def solve_node(self, node_id):
        """
        iteratively solve the RMP, then solve pricing problem using Benders decomposition (BMP, BSP), until no new
        columns found :return: if_branch
        """
        print(Fore.RED + f"Solving node: {node_id}, remaining:{list(self.branch_queue.queue)}" + Style.RESET_ALL)

        if node_id > 1:
            # with the branch constraint, solve the RMP once, remove the columns with positive reduced cost
            RMP_Status, duals = self.master_problem.solve(node_id)  # solve RMP
            remove_columns = []
            for route_key in node_infos[node_id].columns:
                # do not remove initial routes
                if route_key in self.initial_route:
                    continue
                reduced_cost = self.cal_reduced_cost(route_key, duals, node_id)
                if reduced_cost > 0:
                    remove_columns.append(route_key)
            for route_key in remove_columns:
                route_id = route_key_id_pairs[route_key]
                var_name = f"z_{route_id}"
                model = node_infos[node_id].model
                # remove it from the model
                model.remove(model.getVarByName(var_name))
                # remove the column from node info
                node_infos[node_id].remove(route_key)

            # examine the route pool, add the columns that have already been found with negative reduced cost
            while True:
                RMP_Status, duals = self.master_problem.solve(node_id)  # solve RMP
                if RMP_Status != GRB.OPTIMAL:
                    print("infeasible, pruned")
                    return False  # infeasible node, prune
                # routes that are not added to the node
                alternative_routes = [key for key in route_key_id_pairs.keys() if
                                      key not in node_infos[node_id].columns]
                promising_routes = []  # routes in alternative_routes with negative reduced cost
                for route_key in alternative_routes:
                    reduced_cost = self.cal_reduced_cost(route_key, duals, node_id)
                    if reduced_cost + close_tolerance < 0:
                        promising_routes.append((route_key, reduced_cost))
                # sort the promising routes
                promising_routes = sorted(promising_routes, key=lambda elem: elem[1])
                if len(promising_routes) > 0:
                    # add the most promising (lowest reduced cost) route to the RMP
                    route_key = route_dict[promising_routes[0][0]]
                    self.add_column_to_master(route_key, node_id)
                    node_infos[node_id].columns.append(route_key)
                else:
                    break
                # calculate the reduced cost

        # column generation, break if no columns are found
        while True:
            # print()
            # print(Fore.RED + f"Solving RMP, node: {node_id}" + Style.RESET_ALL)
            RMP_Status, duals = self.master_problem.solve(node_id)  # solve RMP

            # check feasibility of RMP
            if RMP_Status != GRB.OPTIMAL:
                # infeasible RMP, prune the current node
                print("infeasible, pruned")
                return False

            # solving the pricing problem to find route with negative reduced cost
            # route [truck_route, drone_route, cost,id]
            new_route, find_new_route = self.pricing_problem.solve(self.net, duals)
            # print()
            # no column is found
            if not find_new_route:
                break
            # find a new column with negative reduced cost
            else:
                new_route['id'] = len(route_key_id_pairs)
                route_key = (tuple(new_route['truck']), tuple(new_route['drone']))
                # an unexplored route
                if route_key not in route_dict.keys():
                    route_dict[route_key] = new_route
                    route_key_id_pairs[route_key] = new_route['id']
                # add the column to RMP
                if route_key not in node_infos[node_id].columns:
                    node_infos[node_id].columns.append(route_key)
                    self.add_column_to_master(route_key, node_id)

        # here the node is solved, update the bounds and decides whether to branch
        # check integrality of RMP
        is_integer, _ = self.is_integer_solution()
        RMP_obj_val = self.master_problem.model.ObjVal
        if is_integer:  # update incumbent
            if RMP_obj_val < self.global_upper_bound:
                self.global_upper_bound = RMP_obj_val
            return False
        else:  # fractional solution
            return True

    def construct_final_route(self):
        solution = []
        cost = 0
        for i in range(len(self.master_problem.z_list)):
            if is_close(self.master_problem.z_list[i].X, 1):
                key = self.master_problem.z_keys[i]
                route = route_dict[key]
                solution.append(route)
                cost += route['cost']
        return solution, cost

    def cal_reduced_cost(self, route_key, duals, node_id):
        route = route_dict[route_key]
        route_cost = route['cost']
        truck_route = route['truck']
        drone_route = route['drone']
        reduced_cost = route_cost  # c_r
        for n_name in self.net.customers:
            n_idx = self.net.customers.index(n_name)
            dual = duals['origin'][n_idx]  # mu
            if n_name in truck_route + drone_route:
                reduced_cost -= dual
        reduced_cost -= duals['origin'][-1]  # nu
        return reduced_cost
