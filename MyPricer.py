from pyscipopt import Pricer, SCIP_RESULT, Model

import GeneralHelper
import RMP
from LSA import *
from NodeInfo import NodeInfo


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
        self.model.constructLP()
        self.model.writeLP("RMP.lp")
        parent_node = current_node.getParent()
        node_id = current_node.getNumber()
        node_dep = current_node.getDepth()
        node_ids.add(node_id)
        node_depth.add(node_dep)
        node_visit_list.append(node_id)
        # current node is the root node
        if not parent_node:
            RMP.node_infos[node_id] = NodeInfo(-1)
            RMP.node_infos[node_id].columns = RMP.initial_route

        # retrieve the dual solutions
        duals = {"mu": [], "nu": -1}
        for c in RMP.constraints[:-1]:
            duals["mu"].append(self.model.getDualsolLinear(c))
        duals["nu"] = self.model.getDualsolLinear(RMP.constraints[-1])

        result = self.run_pricer(parent_node, node_id, duals, False)
        return result

    def pricerfarkas(self):
        """Farkas pricing: Add columns for infeasible LP relaxation."""
        # set up the node infos
        current_node = self.model.getCurrentNode()
        self.model.constructLP()
        self.model.writeLP("RMP.lp")
        parent_node = current_node.getParent()
        node_id = current_node.getNumber()
        node_dep = current_node.getDepth()
        node_ids.add(node_id)
        node_depth.add(node_dep)
        node_visit_list.append(node_id)
        # current node is the root node
        if not parent_node:
            RMP.node_infos[node_id] = NodeInfo(-1)
            RMP.node_infos[node_id].columns = RMP.initial_route

        # Get dual values (Farkas multipliers) of constraints
        duals = {"mu": [], "nu": -1}
        for c in RMP.constraints[:-1]:
            duals["mu"].append(self.model.getDualfarkasLinear(c))
        duals["nu"] = self.model.getDualfarkasLinear(RMP.constraints[-1])
        result = self.run_pricer(parent_node, node_id, duals, True)

        return result

    def run_pricer(self, parent_node, node_id, duals, farkas):
        # not the root node, check the column pool to add promising columns
        if parent_node:
            # routes that are not added to the node
            alternative_routes = [key for key in RMP.route_key_id_pairs.keys() if
                                  key not in RMP.node_infos[node_id].columns]
            promising_routes = []  # routes in alternative_routes with negative reduced cost
            for route_key in alternative_routes:
                route = RMP.route_dict[route_key]
                reduced_cost = cal_reduced_cost(route, duals, GeneralHelper.net)
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

        reduced_cost, hashable_path, where = label_setting.solve(farkas, RMP.node_infos[node_id])

        GeneralHelper.test_sols.append((farkas, hashable_path, where))

        # find a new route
        if reduced_cost + close_tolerance < 0:
            # route_key, new_route = elementary_path_to_route(path, len(RMP.route_key_id_pairs),
            #                                                 GeneralHelper.net, GeneralHelper.transformed_net)
            route_key, new_route = hashable_path_to_route(hashable_path, len(RMP.route_key_id_pairs),
                                                          GeneralHelper.transformed_net)

            # an unexplored route
            if route_key not in RMP.route_dict.keys():
                RMP.route_dict[route_key] = new_route
                RMP.route_key_id_pairs[route_key] = new_route['id']
            # add the column to RMP
            if route_key not in RMP.node_infos[node_id].columns:
                RMP.node_infos[node_id].columns.append(route_key)
                self.add_column_to_master(route_key, node_id)
        else:
            sdas = 0

        return {'result': SCIP_RESULT.SUCCESS}

    def add_column_to_master(self, route_key, node_id):
        """Add new column (route) to the master problem"""
        route = RMP.route_dict[route_key]
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

        # update node_infos
        RMP.node_infos[node_id].columns.append(route_key) if route_key not in RMP.node_infos[node_id].columns else None

        RMP.z_list.append(newVar)
        RMP.z_keys.append(route_key)
