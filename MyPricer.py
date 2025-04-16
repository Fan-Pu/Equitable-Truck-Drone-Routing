from pyscipopt import Pricer, SCIP_RESULT, Model

import GeneralHelper
import RMP
from LSA import *
from NodeInfo import NodeInfo
import cProfile
import pstats


class MyPricer(Pricer):

    def pricerinit(self):
        """
        The initialisation function for the variable pricer to retrieve the transformed constraints of the problem
        """
        for i, c in enumerate(RMP.constraints):
            RMP.constraints[i] = self.model.getTransformedCons(c)

    def pricerredcost(self):
        """
        The reduced cost function for the variable pricer
        """

        # set up the node infos
        current_node = self.model.getCurrentNode()
        # self.model.constructLP()
        parent_node = current_node.getParent()
        if parent_node is not None:
            parent_id = parent_node.getNumber()
        node_id = current_node.getNumber()
        if node_id == 3:
            sdsa = 0
        self.model.writeLP(f"RMP_{node_id}.lp")
        node_dep = current_node.getDepth()
        node_ids.add(node_id)
        node_depth.add(node_dep)
        node_visit_list.append(node_id)
        # current node is the root node
        if not parent_node:
            RMP.node_infos[node_id] = NodeInfo(-1)
            RMP.node_infos[node_id].columns = RMP.initial_route

        # retrieve the dual solutions
        duals = {"mu": []}
        for c in RMP.constraints[:-1]:
            duals["mu"].append(self.model.getDualsolLinear(c))

        nu = self.model.getDualsolLinear(RMP.constraints[-1])

        xi = 0
        if RMP.node_infos[node_id].vehicle_fleet_branch_lb_cons is not None:
            xi = self.model.getDualsolLinear(RMP.node_infos[node_id].vehicle_fleet_branch_lb_cons)

        kappa = 0
        if RMP.node_infos[node_id].vehicle_fleet_branch_ub_cons is not None:
            kappa = self.model.getDualsolLinear(RMP.node_infos[node_id].vehicle_fleet_branch_ub_cons)

        duals["constant_term"] = -nu + xi - kappa

        duals["gamma"] = {}
        for i, j, frac_val, cons in RMP.node_infos[node_id].arc_flow_ub_cons_list:
            dual = self.model.getDualsolLinear(cons)
            duals["gamma"][(i, j, frac_val)] = dual

        duals["pi"] = {}
        for i, j, frac_val, cons in RMP.node_infos[node_id].arc_flow_lb_cons_list:
            dual = self.model.getDualsolLinear(cons)
            duals["pi"][(i, j, frac_val)] = dual

        result = self.run_pricer(parent_node, node_id, duals, False)
        return result

    def pricerfarkas(self):
        """Farkas pricing: Add columns for infeasible LP relaxation."""
        # set up the node infos
        current_node = self.model.getCurrentNode()
        # self.model.constructLP()
        parent_node = current_node.getParent()
        if parent_node is not None:
            parent_id = parent_node.getNumber()
        node_id = current_node.getNumber()
        if node_id == 3:
            row_data = self.model.getLPRowsData()
            node_added_rows = current_node.getAddedConss()
            dsa = 0
        self.model.writeLP(f"RMP_{node_id}.lp")
        node_dep = current_node.getDepth()
        node_ids.add(node_id)
        node_depth.add(node_dep)
        node_visit_list.append(node_id)
        # current node is the root node
        if not parent_node:
            RMP.node_infos[node_id] = NodeInfo(-1)
            RMP.node_infos[node_id].columns = RMP.initial_route

        # Get dual values (Farkas multipliers) of constraints
        duals = {"mu": []}
        for c in RMP.constraints[:-1]:
            duals["mu"].append(self.model.getDualfarkasLinear(c))

        nu = self.model.getDualfarkasLinear(RMP.constraints[-1])

        xi = 0
        if RMP.node_infos[node_id].vehicle_fleet_branch_lb_cons is not None:
            xi = self.model.getDualfarkasLinear(RMP.node_infos[node_id].vehicle_fleet_branch_lb_cons)

        kappa = 0
        if RMP.node_infos[node_id].vehicle_fleet_branch_ub_cons is not None:
            kappa = self.model.getDualfarkasLinear(RMP.node_infos[node_id].vehicle_fleet_branch_ub_cons)

        duals["constant_term"] = -nu + xi - kappa

        duals["gamma"] = {}
        for i, j, frac_val, cons in RMP.node_infos[node_id].arc_flow_ub_cons_list:
            dual = self.model.getDualfarkasLinear(cons)
            duals["gamma"][(i, j, frac_val)] = dual

        duals["pi"] = {}
        for i, j, frac_val, cons in RMP.node_infos[node_id].arc_flow_lb_cons_list:
            dual = self.model.getDualfarkasLinear(cons)
            duals["pi"][(i, j, frac_val)] = dual

        result = self.run_pricer(parent_node, node_id, duals, True)
        return result

    def run_pricer(self, parent_node, node_id, duals, farkas):
        # not the root node, check the column pool to add promising columns
        # if parent_node:
        #     # routes that are not added to the node
        #     alternative_routes = [key for key in RMP.route_key_id_pairs.keys() if
        #                           key not in RMP.node_infos[node_id].columns]
        #     promising_routes = []  # routes in alternative_routes with negative reduced cost
        #     for route_key in alternative_routes:
        #         route = RMP.route_dict[route_key]
        #         reduced_cost = cal_reduced_cost(route, duals, GeneralHelper.net)
        #         if reduced_cost + close_tolerance < 0:
        #             promising_routes.append((route_key, reduced_cost))
        #     # sort the promising routes
        #     promising_routes = sorted(promising_routes, key=lambda elem: elem[1])
        #     if len(promising_routes) > 0:
        #         route_key = promising_routes[0][0]
        #         # add the most promising route to the RMP
        #         self.add_column_to_master(route_key, node_id)
        #         RMP.node_infos[node_id].columns.append(route_key)

        # bi-directional label setting
        label_setting = BiDirectionalLabelSetting(duals)

        reduced_cost, hashable_path, where = label_setting.solve(farkas, RMP.node_infos[node_id])

        if where == 0:
            GeneralHelper.label_merge_num += 1
        elif where == 1:
            GeneralHelper.label_forward_num += 1
        else:
            GeneralHelper.label_backward_num += 1

        GeneralHelper.test_sols.append((farkas, hashable_path, where))

        # find a new route
        if reduced_cost + close_tolerance < 0:
            # route_key, new_route = elementary_path_to_route(path, len(RMP.route_key_id_pairs),
            #                                                 GeneralHelper.net, GeneralHelper.transformed_net)
            route_key, new_route = hashable_path_to_route(hashable_path, len(RMP.route_key_id_pairs),
                                                          GeneralHelper.transformed_net)
            if hashable_path == (tuple(['Source', 'H1', 'Sink']), frozenset(
                    {
                        'H1': frozenset({'C4_prime', 'C8_prime'})
                    }.items()
            )):
                sdas = 0

            if route_key in GeneralHelper.test_path_list:
                sdas = 0

            # an unexplored route
            if route_key not in RMP.route_dict.keys():
                RMP.route_dict[route_key] = new_route
                RMP.route_key_id_pairs[route_key] = new_route['id']
            # add the column to RMP
            if route_key not in RMP.node_infos[node_id].columns:
                RMP.node_infos[node_id].columns.append(route_key)
                self.add_column_to_master(route_key, node_id)
        else:
            node_info = RMP.node_infos[node_id]
            check_list = []
            for temp_path in test_path_list:
                check_list.append(temp_path in node_info.columns)
            sdas = 0

        return {'result': SCIP_RESULT.SUCCESS}

    def add_column_to_master(self, route_key, node_id):
        """Add new column (route) to the master problem"""
        route = RMP.route_dict[route_key]
        GeneralHelper.columns_list.append(route['cost'])
        # generate a new variable and assign the coefficient to objective function
        newVar = self.model.addVar(f"z_{route['id']}", vtype="B", obj=route['cost'], pricedVar=True)
        # customer must be served once
        for i, n_name in enumerate(GeneralHelper.net.customers):
            theta = 0
            if n_name in route['truck']:
                theta = 1
            else:
                for key, node_set in route['drone'].items():
                    if n_name + "_prime" in node_set:
                        theta = 1
                        break
            self.model.addConsCoeff(RMP.constraints[i], newVar, theta)

        # truck fleet constraints
        self.model.addConsCoeff(RMP.constraints[-1], newVar, 1)

        # for vehicle fleet branching constraints
        node_info = RMP.node_infos[node_id]
        for cons in node_info.vehicle_fleet_branch_lb_cons_list + node_info.vehicle_fleet_branch_ub_cons_list:
            self.model.addConsCoeff(cons, newVar, 1)

        # for arc flow branching constraints
        for i, j, _, cons in node_info.arc_flow_lb_cons_list + node_info.arc_flow_ub_cons_list:
            # the truck travel the arc
            if (i, j) in zip(route['truck'], route['truck'][1:]):
                self.model.addConsCoeff(cons, newVar, 1)
                continue
            elif i in GeneralHelper.net.hubs and i in route['drone'].keys() and j + "_prime" in route['drone'][i]:
                self.model.addConsCoeff(cons, newVar, 1)

        # update node_infos
        RMP.node_infos[node_id].columns.append(route_key) if route_key not in RMP.node_infos[node_id].columns else None

        RMP.z_list.append(newVar)
        RMP.z_keys.append(route_key)
