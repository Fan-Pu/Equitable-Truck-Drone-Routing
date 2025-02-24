from pyscipopt import Pricer, SCIP_RESULT

import RMP
from BMP import BMP
from BSP import BSP
from LSA import *
from NodeInfo import NodeInfo


class MyPricer(Pricer):

    def pricerredcost(self):
        """
        The reduced cost function for the variable pricer
        """
        # set up the node infos
        current_node = self.model.getCurrentNode()
        parent_node = current_node.getParent()
        node_id = current_node.getNumber()
        # current node is the root node
        if not parent_node:
            RMP.node_infos[node_id] = NodeInfo(-1)
            RMP.node_infos[node_id].columns = RMP.initial_route

        # retrieve the dual solutions
        duals = {"mu": [], "nu": -1}
        for c in RMP.constraints[:-1]:
            duals["mu"].append(self.model.getDualsolLinear(c))
        duals["nu"] = self.model.getDualsolLinear(RMP.constraints[-1])

        var_vals = []
        for var in RMP.z_list:
            var_vals.append(self.model.getVal(var))

        # not the root node, check the column pool to add promising columns
        if parent_node:
            self.model.writeProblem("RMP.lp")
            # routes that are not added to the node
            alternative_routes = [key for key in RMP.route_key_id_pairs.keys() if
                                  key not in RMP.node_infos[node_id].columns]
            promising_routes = []  # routes in alternative_routes with negative reduced cost
            for route_key in alternative_routes:
                reduced_cost = self.cal_reduced_cost(route_key, duals)
                if reduced_cost + close_tolerance < 0:
                    promising_routes.append((route_key, reduced_cost))
            # sort the promising routes
            promising_routes = sorted(promising_routes, key=lambda elem: elem[1])
            if len(promising_routes) > 0:
                route_key = promising_routes[0][0]
                # add the most promising route to the RMP
                self.add_column_to_master(route_key, node_id)
                RMP.node_infos[node_id].columns.append(route_key)

        # bi-directional label setting
        label_setting = BiDirectionalLabelSetting(duals)
        label_setting.solve(farkas=False)  # Call solve() directly

        # solve the pricing problem (Benders loop)
        iter_num = 0
        # remove the cuts
        self.BMP.remove_cuts()
        new_route = None
        while True:
            if len(RMP.route_dict) >= 7:
                break
            iter_num += 1
            self.BMP.update_objective_and_BSP_Lb(duals)
            BMP_obj_val, x_vals, phi_value = self.BMP.solve()
            self.BSP.update_objective(duals)
            self.BSP.update_binding_cons(x_vals)
            # BSP_LP_obj_val, BSP_duals = self.BSP.solveLP()
            # # add subgradient cut
            # if phi_value + close_tolerance < BSP_LP_obj_val and not is_close(phi_value, BSP_LP_obj_val):
            #     self.BMP.separate_subgradient_cut(BSP_LP_obj_val, BSP_duals, x_vals)
            # needs to solve IP
            # else:
            BSP_IP_obj_val, y_vals, route_cost = self.BSP.solveIP()
            # optimal solution found
            if abs(BSP_IP_obj_val - phi_value) < close_tolerance:
                # negative reduced cost, return the new column
                if self.model.isLT(BMP_obj_val, 0):
                    # if BMP_obj_val + close_tolerance < 0:
                    new_route = self.get_route(x_vals, y_vals, route_cost)
                break
            elif phi_value + close_tolerance < BSP_IP_obj_val:
                self.BMP.separate_L_shaped_cut(BSP_IP_obj_val, x_vals, GeneralHelper.BSP_LB)
            else:
                raise Exception("Errors in BSP objective value")

        # find a new route
        if new_route is not None:
            # add the new route to route pool
            new_route['id'] = len(RMP.route_key_id_pairs)
            route_key = (tuple(new_route['truck']), tuple(new_route['drone']))
            # an unexplored route
            if route_key not in RMP.route_dict.keys():
                RMP.route_dict[route_key] = new_route
                RMP.route_key_id_pairs[route_key] = new_route['id']
            # add the column to RMP
            if route_key not in RMP.node_infos[node_id].columns:
                RMP.node_infos[node_id].columns.append(route_key)
                self.add_column_to_master(route_key, node_id)

        return {'result': SCIP_RESULT.SUCCESS}

    def pricerinit(self):
        """
        The initialisation function for the variable pricer to retrieve the transformed constraints of the problem
        """
        for i, c in enumerate(RMP.constraints):
            RMP.constraints[i] = self.model.getTransformedCons(c)

        # initialize the BMP and BSP
        self.BMP = BMP()
        self.BSP = BSP()

    def pricerfarkas(self):
        """Farkas pricing: Add columns for infeasible LP relaxation."""
        print("Executing Farkas pricing...")

        # Get dual values (Farkas multipliers) of constraints
        duals = {"mu": [], "nu": -1}
        for c in RMP.constraints[:-1]:
            duals["mu"].append(self.model.getDualfarkasLinear(c))
        duals["nu"] = self.model.getDualfarkasLinear(RMP.constraints[-1])

        label_setting = BiDirectionalLabelSetting(duals)
        label_setting.solve(farkas=True)

        return SCIP_RESULT.SUCCESS

    def get_node_id(self):
        """Get the unique ID of the current branch-and-bound node."""
        current_node = self.model.getCurrentNode()
        if current_node:
            return current_node.getNumber()
        return None  # Root node or no current node

    def cal_reduced_cost(self, route_key, duals):
        route = RMP.route_dict[route_key]
        route_cost = route['cost']
        truck_route = route['truck']
        drone_route = route['drone']
        reduced_cost = route_cost  # c_r
        for n_idx, n_name in enumerate(GeneralHelper.net.customers):
            dual = duals[n_idx]  # mu
            if n_name in truck_route + drone_route:
                reduced_cost -= dual
        reduced_cost -= duals[-1]  # nu
        return reduced_cost

    def add_column_to_master(self, route_key, node_id):
        """Add new column (route) to the master problem"""
        route = RMP.route_dict[route_key]
        # generate a new variable and assign the coefficient to objective function
        newVar = self.model.addVar(f"z_{route['id']}", vtype="C", obj=route['cost'], pricedVar=True)
        # customer must be served once
        for i, n_name in enumerate(GeneralHelper.net.customers):
            theta = int(n_name in route['truck'] or n_name in route['drone'])
            self.model.addConsCoeff(RMP.constraints[i], newVar, theta)

        # truck fleet constraints
        self.model.addConsCoeff(RMP.constraints[-1], newVar, 1)

        # update node_infos
        RMP.node_infos[node_id].columns.append(route_key) if route_key not in RMP.node_infos[node_id].columns else None

        RMP.z_list.append(newVar)
        RMP.z_keys.append(route_key)

    def get_route(self, x_vals, y_vals, route_cost):
        """
        given the values of variables, output the route solution
        """
        # for truck
        active_links = {key for key, value in x_vals.items() if is_close(1, value)}
        # Initialize path
        start_node = GeneralHelper.net.depot_source
        end_node = GeneralHelper.net.depot_sink
        truck_route = [start_node]
        # Follow the links to construct the path
        current_node = GeneralHelper.net.all_nodes_indices[start_node]
        while current_node != GeneralHelper.net.all_nodes_indices[end_node]:
            # Find the next node connected to the current node
            next_node = next((j for i, j in active_links if i == current_node), None)
            if next_node is None:
                raise ValueError(f"No valid path found from node {current_node}.")
            truck_route.append(GeneralHelper.net.all_nodes[next_node])
            current_node = next_node

        # for drone
        drone_route = [GeneralHelper.net.all_nodes[j] for (i, j, d), value in y_vals.items() if is_close(1, value)]
        launches = [GeneralHelper.net.all_nodes[i] for (i, j, d), value in y_vals.items() if is_close(1, value)]
        drone_links = [(GeneralHelper.net.all_nodes[i], GeneralHelper.net.all_nodes[j]) for (i, j, d), value in
                       y_vals.items() if
                       abs(1 - value) < close_tolerance]
        route = {'truck': truck_route, 'drone': drone_route, 'launches': launches, 'cost': route_cost,
                 'drone links': drone_links}

        return route
