import logging
import time
from queue import PriorityQueue

from colorama import Fore, Style
from gurobipy import GRB

from GeneralHelper import *
from PSP import PSP
from RMP import RMP


class BranchAndPrice:
    def __init__(self, net, tolerance=1e-6, max_runtime=float('inf')):
        self.net = net
        self.tolerance = tolerance
        self.max_runtime = max_runtime
        self.branch_queue = PriorityQueue()
        self.global_upper_bound = float('inf')  # the cost for incumbent solution
        self.best_solution = None
        self.node_id_counter = 0
        self.node_info = {}
        self.route_dict = {}  # key (truck_route, drone_route)
        self.route_keys = []  # route id - route key
        # add initial routes
        for route in self.find_initial_routes(net):
            route_key = (tuple(route['truck']), tuple(route['drone']))
            route['id'] = len(self.route_keys)
            self.route_dict[route_key] = route
            self.route_keys.append(route_key)
        self.master_problem = RMP(net, self.route_dict)
        self.pricing_problem = PSP(net)

    def solve(self):
        """Main branch-and-price loop"""
        start_time = time.time()

        # add root node
        node = self.master_problem.model
        self.node_id_counter += 1
        node_id = self.node_id_counter
        self.node_info[node_id] = {'model': node, 'parent': -1, 'columns': []}
        self.branch_queue.put(node_id)

        # branch
        while not self.branch_queue.empty():
            if time.time() - start_time > self.max_runtime:
                logging.info("Terminated due to reaching maximum runtime.")
                break

            node_id = self.branch_queue.get()
            self.master_problem.model = self.node_info[node_id]['model']
            # solve the current node
            need_branch = self.solve_node(node_id)

            if need_branch:
                self.branch()

        # terminated
        final_solution, cost = self.construct_final_route()
        sdas = 0

    def add_column_to_master(self, route):
        """Add new column (route) to the master problem"""
        route_key = (tuple(route['truck']), tuple(route['drone']))
        self.master_problem.z_keys.append(route_key)
        var = self.master_problem.model.addVar(obj=route['cost'], name=f"z_{route['id']}", vtype=GRB.CONTINUOUS)
        self.master_problem.z_list.append(var)

        for i, n_name in enumerate(self.net.customers):
            theta = int(n_name in route['truck'] or n_name in route['drone'])
            self.master_problem.model.chgCoeff(self.master_problem.constraints[i], var, theta)

        self.master_problem.model.chgCoeff(self.master_problem.constraints[-1], var, 1)
        self.master_problem.model.update()

    def branch(self, parent_id=None):
        parent_node = self.node_info[parent_id]
        # node 1: Add constraint sum_z <= floor(sum_z_val)
        node_left = self.master_problem.model.copy()
        cons_name = self.master_problem.branch_cons_UB.ConstrName
        node_left.getConstrByName(cons_name).setAttr("RHS", int(self.master_problem.sum_z_val))
        self.node_id_counter += 1
        node_id = self.node_id_counter
        self.node_info[node_id] = {'model': node_left, 'parent': parent_id, 'columns': parent_node['columns'].copy()}
        self.branch_queue.put(node_id)

        # node 2: Add constraint var >= ceil(fractional_var.X)
        node_right = self.master_problem.model.copy()
        cons_name = self.master_problem.branch_cons_LB.ConstrName
        node_right.getConstrByName(cons_name).setAttr("RHS", int(self.master_problem.sum_z_val) + 1)
        self.node_id_counter += 1
        node_id = self.node_id_counter
        self.node_info[node_id] = {'model': node_right, 'parent': parent_id, 'columns': parent_node['columns'].copy()}
        self.branch_queue.put(node_id)

    def is_integer_solution(self):
        """Check if the RMP current solution is integer"""
        if self.master_problem.model.Status == GRB.OPTIMAL:
            for var in self.master_problem.model.getVars():
                if var.X > 0 and not var.X.is_integer():
                    return False, var
        return True, None

    def solve_node(self, node_id):
        """
        iteratively solve the RMP, then solve pricing problem using Benders decomposition (BMP, BSP), until no new
        columns found :return: if_branch
        """

        # Column generation, break if no columns are found
        while True:
            print()
            print(Fore.RED + f"Solving RMP, node: {node_id}" + Style.RESET_ALL)
            RMP_Status, duals = self.master_problem.solve()  # solve RMP

            # check feasibility of RMP
            if RMP_Status != GRB.OPTIMAL:
                # infeasible RMP, prune the current node
                return False

            # solving the pricing problem to find route with negative reduced cost
            # route [truck_route, drone_route, cost,id]
            new_route, find_new_route = self.pricing_problem.solve(self.net, duals)
            print()
            # no column is found
            if not find_new_route:
                break
            # find a new column with negative reduced cost
            else:
                new_route['id'] = len(self.route_keys)
                route_key = (tuple(new_route['truck']), tuple(new_route['drone']))
                # an unexplored route
                if route_key not in self.route_dict.keys():
                    self.route_dict[route_key] = new_route
                    self.route_keys.append(route_key)
                # add the column to RMP
                if route_key not in self.node_info[node_id]['columns']:
                    self.node_info[node_id]['columns'].append(route_key)
                    self.add_column_to_master(new_route)

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

    def find_initial_routes(self, net):
        routes = []
        # the route that visits all nodes
        truck_route = [net.depot_source, net.customers[0]]
        arrive_time = net.truck_travel_times[(net.depot_source, net.customers[0])]
        # cost = (arrive_time - net.a_lb[net.customers[0]]) ** 2
        cost = cost_scale * (arrive_time - net.a_lb[net.customers[0]])
        return_time = arrive_time  # the time it returns to depot sink
        for i in range(len(net.customers) - 1):
            n = net.customers[i]
            n_next = net.customers[i + 1]
            arrive_time += net.truck_travel_times[(n, n_next)]
            # cost += (arrive_time - net.a_lb[n_next]) ** 2
            cost += cost_scale * (arrive_time - net.a_lb[n_next])
            return_time += net.truck_travel_times[(n, n_next)]
            truck_route.append(n_next)
        return_time += net.truck_travel_times[(net.customers[-1], net.depot_sink)]
        cost += return_time
        truck_route.append(net.depot_sink)
        drone_route = []
        route = {'truck': truck_route, 'drone': drone_route, 'cost': cost, 'drone links': []}
        routes.append(route)

        # the route that visit only a node
        for n_name in net.customers + net.hubs:
            truck_route = [net.depot_source, n_name, net.depot_sink]
            arrive_time = net.truck_travel_times[(net.depot_source, n_name)]
            # cost = (arrive_time - net.a_lb[net.customers[0]]) ** 2
            cost = cost_scale * (arrive_time - net.a_lb[n_name])
            return_time = arrive_time + net.truck_travel_times[(n_name, net.depot_sink)]
            cost += return_time
            drone_route = []
            route = {'truck': truck_route, 'drone': drone_route, 'cost': cost, 'drone links': []}
            routes.append(route)

        return routes

    def construct_final_route(self):
        solution = []
        cost = 0
        for i in range(len(self.master_problem.z_list)):
            if is_close(self.master_problem.z_list[i].X, 1):
                key = self.master_problem.z_keys[i]
                route = self.route_dict[key]
                solution.append(route)
                cost += route['cost']
        return solution, cost
