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
            self.constraints.append(
                self.model.addConstr(gp.quicksum(z_vars) == 1, name=f"visit_customer_{i}")
            )
        # vehicle fleet branching
        self.vehicle_fleet_branch_ub_cons = self.model.addConstr(
            gp.quicksum(self.z_dict.values()) <= node_info.vehicle_fleet_branch_ub, name=f"truck_fleet_ub")
        self.vehicle_fleet_branch_lb_cons = self.model.addConstr(
            gp.quicksum(self.z_dict.values()) >= node_info.vehicle_fleet_branch_lb, name=f"truck_fleet_lb")
        # SR inequalities
        for triple, route_keys in node_info.SR_infos.items():
            if len(route_keys) == 0:
                continue
            if len(self.SR_inequalities) >= SR_num:
                break
            lhs = gp.quicksum([self.z_dict[key] for key in route_keys])
            self.SR_inequalities[triple] = self.model.addConstr(lhs <= 1, name=f"SR_{triple}")

    def solve(self):
        self.model.setParam('OutputFlag', 0)
        self.model.setParam('InfUnbdInfo', 1)
        self.model.setParam('DualReductions', 0)
        # self.model.setParam('Presolve', 0)
        self.model.update()
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
        # get dual
        if self.status == GRB.OPTIMAL:
            self.duals = {"mu": []}
            for cons in self.constraints:
                self.duals["mu"].append(round(cons.Pi, 2))

            xi = round(self.vehicle_fleet_branch_lb_cons.Pi, 2)
            kappa = round(self.vehicle_fleet_branch_ub_cons.Pi, 2)
            self.duals["constant_term"] = -xi - kappa

            # for SR inequalities
            for triple, cons in self.SR_inequalities.items():
                self.duals[triple] = round(cons.Pi, 2)
        # get farkas dual
        elif self.status == GRB.INFEASIBLE:
            self.duals = {"mu": []}
            for cons in self.constraints:
                self.duals["mu"].append(round(-cons.FarkasDual, 2))

            xi = round(-self.vehicle_fleet_branch_lb_cons.FarkasDual, 2)
            kappa = round(-self.vehicle_fleet_branch_ub_cons.FarkasDual, 2)
            self.duals["constant_term"] = -xi - kappa

            # for SR inequalities
            for triple, cons in self.SR_inequalities.items():
                self.duals[triple] = round(cons.FarkasDual, 2)

    def run_pricer(self, node_info: NodeInfo):
        """
        run the pricer once
        """
        find_new_column = False
        node_id = node_info.id
        farkas = True if self.status == GRB.INFEASIBLE else False
        # bi-directional label setting
        label_setting = BiDirectionalLabelSetting(self.duals)

        deb = False
        if len(node_info.columns) == 9999:
            try:
                pr = cProfile.Profile()
                pr.enable()
                deb = True
                reduced_cost, hashable_path, element_path, where = label_setting.solve(farkas, node_info)
            finally:
                if deb:
                    pr.disable()
                    s = io.StringIO()
                    stats = pstats.Stats(pr, stream=s).sort_stats('cumtime')

                    stats.print_stats(10)
                    # now dump everything at once
                    print(s.getvalue())

                    dsads = 0
        else:
            reduced_cost, hashable_path, element_path, where = label_setting.solve(farkas, node_info)

        # reduced_cost, hashable_path, element_path, where = label_setting.solve(farkas, node_info)
        GeneralHelper.label_forward_num += 1

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
                self._add_column(route_key)

                find_new_column = True
            else:
                raise Exception("column revisited")
        return find_new_column

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
        if len(self.SR_inequalities) < SR_num:
            potential_triples = sorted(set(GeneralHelper.transformed_net.PI) - set(self.SR_inequalities.keys()))
            for triple in potential_triples:
                covered_customers = self.node_info.column_customer_visits[route_key]
                if len(covered_customers & set(triple)) >= 2:
                    # add a new constraint
                    self.SR_inequalities[triple] = self.model.addConstr(newVar <= 1, name=f"SR_{triple}")
                    self.node_info.SR_infos[triple].append(route_key)
                    self.node_info.column_in_SR_triples[route_key].append(triple)
                if len(self.SR_inequalities) >= SR_num:
                    break

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
