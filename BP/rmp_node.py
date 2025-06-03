from NodeInfo import NodeInfo
import gurobipy as gp
import GeneralHelper
from gurobipy import GRB
from LSA import *
import BP.branch_and_price as bp
import cProfile, pstats, io


class RMPNode:
    def __init__(self, node_info: NodeInfo):
        self.node_info = node_info
        self.obj_val = None
        self.status = None
        self.model = gp.Model("RMP")
        self.model.ModelSense = GRB.MINIMIZE
        self.z_dict = {}
        self.z_vals = {}
        self.duals = None
        self.constraints = []
        self.vehicle_fleet_branch_lb_cons = None
        self.vehicle_fleet_branch_ub_cons = None
        self.SR_inequalities = {}  # key: customer triple
        self.arc_flow_lb_cons_dict = {}  # key (i,j,frac_val), val: cons
        self.arc_flow_ub_cons_dict = {}  # key (i,j,frac_val), val: cons
        # add decision variables
        for route_key in node_info.columns:
            route = bp.route_dict[route_key]
            idx, cost = route['id'], route['cost']
            var = self.model.addVar(name=f"z_{idx}", vtype=GRB.CONTINUOUS, obj=cost, lb=0)
            self.z_dict[route_key] = var
        self.model.update()
        # basic cons
        for i in range(len(GeneralHelper.net.customers)):
            n_name = GeneralHelper.net.customers[i]
            z_vars = [self.z_dict[route_key] for route_key in self.z_dict.keys() if
                      if_route_visit_node(bp.route_dict[route_key], n_name)]
            # set covering
            if node_info.id == 1:
                self.constraints.append(
                    self.model.addConstr(gp.quicksum(z_vars) >= 1, name=f"visit_customer_{i}")
                )
            else:
                self.constraints.append(
                    self.model.addConstr(gp.quicksum(z_vars) == 1, name=f"visit_customer_{i}")
                )
            # self.constraints.append(
            #     self.model.addConstr(gp.quicksum(z_vars) == 1, name=f"visit_customer_{i}")
            # )
        # vehicle fleet branching
        self.vehicle_fleet_branch_ub_cons = self.model.addConstr(
            gp.quicksum(self.z_dict.values()) <= node_info.vehicle_fleet_branch_ub, name=f"truck_fleet_ub")
        self.vehicle_fleet_branch_lb_cons = self.model.addConstr(
            gp.quicksum(self.z_dict.values()) >= node_info.vehicle_fleet_branch_lb, name=f"truck_fleet_lb")
        # SR inequalities
        for triple in node_info.added_SR_keys:
            route_keys = node_info.SR_infos[triple]
            if len(self.SR_inequalities) >= SR_num:
                break
            lhs = gp.quicksum([self.z_dict[key] for key in route_keys])
            self.SR_inequalities[triple] = self.model.addConstr(lhs <= 1, name=f"SR_{triple}")

    def solve(self):
        self.model.setParam('OutputFlag', 0)
        self.model.setParam('InfUnbdInfo', 1)
        self.model.setParam('DualReductions', 0)
        self.model.setParam('Presolve', 0)
        self.model.update()
        self.model.write("node.lp")
        self.model.optimize()
        self.status = self.model.status
        if self.status == GRB.OPTIMAL:
            self.z_vals = {key: var.X for key, var in self.z_dict.items()}
            self.obj_val = self.model.objVal
            self._get_duals()
        elif self.status == GRB.INFEASIBLE:
            self._get_duals()
        else:
            raise Exception("invalid RMP status")

    def _get_duals(self):
        digit_num = 4
        # get dual
        if self.status == GRB.OPTIMAL:
            self.duals = {"mu": []}
            for cons in self.constraints:
                self.duals["mu"].append(round(cons.Pi, digit_num))

            xi = round(self.vehicle_fleet_branch_lb_cons.Pi, digit_num)
            kappa = round(self.vehicle_fleet_branch_ub_cons.Pi, digit_num)
            self.duals["constant_term"] = -xi - kappa

            # for SR inequalities
            for triple, cons in self.SR_inequalities.items():
                self.duals[triple] = round(cons.Pi, digit_num)
        # get farkas dual
        elif self.status == GRB.INFEASIBLE:
            self.duals = {"mu": []}
            for cons in self.constraints:
                self.duals["mu"].append(round(-cons.FarkasDual, digit_num))

            xi = round(-self.vehicle_fleet_branch_lb_cons.FarkasDual, digit_num)
            kappa = round(-self.vehicle_fleet_branch_ub_cons.FarkasDual, digit_num)
            self.duals["constant_term"] = -xi - kappa

            # for SR inequalities
            for triple, cons in self.SR_inequalities.items():
                self.duals[triple] = round(-cons.FarkasDual, digit_num)

    def run_pricer(self, node_info: NodeInfo):
        """
        run the pricer once
        """
        add_new_column = False
        node_id = node_info.id
        farkas = True if self.status == GRB.INFEASIBLE else False
        label_setting = LabelSetting(self.duals)
        reduced_cost, hashable_path, element_path = label_setting.solve(farkas, node_info)
        if node_info.id == 1:
            test_route_list.append(element_path)

        # reduced_cost, hashable_path, element_path, where = label_setting.solve(farkas, node_info)

        # if element_path is not None:
        #     route_key, new_route = hashable_path_to_route(hashable_path, len(bp.route_key_id_pairs),
        #                                                   GeneralHelper.transformed_net)
        #     # check feasibility
        #     for i, j in node_info.disabled_arcs_trucks:
        #         if if_route_travel_arc(new_route, i, j, GeneralHelper.net):
        #             sdas = 0
        #     for i, j in node_info.disabled_arcs_drones:
        #         if if_route_travel_arc(new_route, i, j + "_prime", GeneralHelper.net):
        #             sdas = 0
        #     for i, j in (node_info.must_visit_arcs_trucks | node_info.must_visit_arcs_drones
        #                  | node_info.must_visit_arcs_trans):
        #         if not if_route_travel_arc(new_route, i, j, GeneralHelper.net):
        #             sda = 0

        # find a new route
        if reduced_cost + close_tolerance < 0:
            route_key, new_route = hashable_path_to_route(hashable_path, len(bp.route_key_id_pairs),
                                                          GeneralHelper.transformed_net)
            # an unexplored route
            if route_key not in bp.route_dict.keys():
                bp.route_dict[route_key] = new_route
                bp.route_key_id_pairs[route_key] = new_route['id']
            # add the column to RMP
            if route_key not in bp.node_infos[node_id].columns:
                bp.node_infos[node_id].columns.append(route_key)
                bp.node_infos[node_id].column_elementary_paths[route_key] = element_path
                bp.node_infos[node_id].column_customer_visits[route_key].update(
                    get_route_customer_visits(bp.route_dict[route_key]))

                if route_key in node_info.removed_columns_keys:
                    sdsa = 0

                self._add_column(route_key)
                add_new_column = True

        return add_new_column

    def _add_column(self, route_key):
        """Add new column (route) to the master problem"""
        route = bp.route_dict[route_key]
        # for SR inequalities
        self.node_info.column_customer_visits[route_key].update(get_route_customer_visits(route))

        GeneralHelper.columns_list.append(route['cost'])
        # generate a new variable and assign the coefficient to objective function
        c = gp.Column()
        # customer visit constraints
        for i, n_name in enumerate(GeneralHelper.net.customers):
            if if_route_visit_node(route, n_name):
                c.addTerms(1.0, self.constraints[i])
        # truck fleet constraints
        c.addTerms(1.0, self.vehicle_fleet_branch_lb_cons)
        c.addTerms(1.0, self.vehicle_fleet_branch_ub_cons)
        # for arc flow branching constraints
        for cons_dict in [self.arc_flow_lb_cons_dict, self.arc_flow_ub_cons_dict]:
            for (i, j, frac_val), cons in cons_dict.items():
                if if_route_travel_arc(route, i, j, GeneralHelper.net):
                    c.addTerms(1.0, cons)

        # for existing SR inequalities
        for triple, cons in self.SR_inequalities.items():
            covered_customers = self.node_info.column_customer_visits[route_key]
            if len(covered_customers & set(triple)) >= 2:
                c.addTerms(1.0, cons)
                self.node_info.SR_infos[triple].append(route_key)
                self.node_info.column_in_SR_triples[route_key].append(triple)

        # add new variable
        newVar = self.model.addVar(name=f"z_{route['id']}", vtype=GRB.CONTINUOUS, obj=route['cost'], lb=0, column=c)

        self.model.update()

        # for new SR inequalities
        # new_add_cons = 0
        # if len(self.SR_inequalities) < SR_num:
        #     potential_triples = sorted(set(GeneralHelper.transformed_net.PI) - set(self.SR_inequalities.keys()))
        #     for triple in potential_triples:
        #         covered_customers = self.node_info.column_customer_visits[route_key]
        #         if len(covered_customers & set(triple)) >= 2:
        #             # add a new constraint
        #             new_add_cons += 1
        #             self.SR_inequalities[triple] = self.model.addConstr(newVar <= 1, name=f"SR_{triple}")
        #             self.node_info.SR_infos[triple].append(route_key)
        #             self.node_info.added_SR_keys.add(triple)
        #             self.node_info.column_in_SR_triples[route_key].append(triple)
        #         if len(self.SR_inequalities) >= SR_num:
        #             break

        new_add_cons = 0
        potential_triples = sorted(set(GeneralHelper.transformed_net.PI) - set(self.SR_inequalities.keys()))
        for triple in potential_triples:
            if new_add_cons >= SR_num_each_run or len(self.node_info.added_SR_keys) >= GeneralHelper.SR_num:
                break
            covered_customers = self.node_info.column_customer_visits[route_key]
            if len(covered_customers & set(triple)) >= 2:
                # add a new constraint
                new_add_cons += 1
                self.SR_inequalities[triple] = self.model.addConstr(newVar <= 1, name=f"SR_{triple}")
                self.node_info.SR_infos[triple].append(route_key)
                self.node_info.added_SR_keys.add(triple)
                self.node_info.column_in_SR_triples[route_key].append(triple)

        # update node_infos
        self.z_dict[route_key] = newVar

    def is_integer(self):
        result = True
        fractions = {}
        for key, val in self.z_vals.items():
            if not is_integer(val):
                result = False
                fractions[key] = val
        return result, fractions
