import math
from concurrent.futures import ProcessPoolExecutor, as_completed
from multiprocessing import Manager

import gurobipy as gp
from gurobipy import GRB

import BP.branch_and_price as bp
from LabelSetting import *

_pool = None
_stop_event = None


def worker_init(seed, num_trucks, num_customers, custom_dist, drone_num, cw):
    CommonHelper.update_inputs(seed, num_trucks, num_customers, custom_dist, drone_num, cw)
    CommonHelper.create_original_network()
    CommonHelper.transform_network()


def get_pricer_pool(num_threads, seed, num_trucks, num_customers, custom_dist, drone_num, cw):
    global _pool, _stop_event
    if _pool is None:
        mgr = Manager()
        _stop_event = mgr.Event()
        _pool = ProcessPoolExecutor(
            max_workers=num_threads,
            initializer=worker_init,
            initargs=(seed, num_trucks, num_customers, custom_dist, drone_num, cw)
        )
    return _pool, _stop_event


def pricer_task(i: int, duals: dict, farkas: bool, node_info_dict: dict, stop_event):
    if stop_event.is_set():
        return i, (None, (None, None))
    # rebuild NodeInfo from a plain dict
    ni = node_info_dict
    # ni = NodeInfo.deserialize(node_info_dict)
    ls = LabelSetting(duals, i)
    return i, ls.solve(farkas, ni, stop_event)


class RMPNode:
    def __init__(self, node_info: NodeInfo):
        self.node_info = node_info
        self.obj_val = None
        self.primal_obj_val = None
        self.status = None
        self.model = gp.Model("RMP")
        self.model.ModelSense = GRB.MINIMIZE
        self.z_dict = {}
        self.z_vals = {}
        self.primal_z_vals = {}
        self.find_primal_solution = False
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
        for i in range(len(CommonHelper.net.customers)):
            n_name = CommonHelper.net.customers[i]
            z_vars = [self.z_dict[route_key] for route_key in self.z_dict.keys() if
                      CommonHelper.if_route_visit_node(bp.route_dict[route_key], n_name)]
            self.constraints.append(
                self.model.addConstr(gp.quicksum(z_vars) == 1, name=f"visit_customer_{i}")
            )
        # vehicle fleet branching
        self.vehicle_fleet_branch_ub_cons = self.model.addConstr(
            gp.quicksum(self.z_dict.values()) <= node_info.vehicle_fleet_branch_ub, name=f"truck_fleet_ub")
        self.vehicle_fleet_branch_lb_cons = self.model.addConstr(
            gp.quicksum(self.z_dict.values()) >= node_info.vehicle_fleet_branch_lb, name=f"truck_fleet_lb")
        # SR inequalities
        for triple in node_info.added_SR_keys:
            route_keys = node_info.SR_infos[triple]
            if len(self.SR_inequalities) >= CommonHelper.SR_num:
                break
            lhs = gp.quicksum([self.z_dict[key] for key in route_keys])
            self.SR_inequalities[triple] = self.model.addConstr(lhs <= 1, name=f"SR_{triple}")

    def solve(self):
        self.model.setParam('OutputFlag', 0)
        self.model.setParam('Method', 0)
        self.model.setParam('InfUnbdInfo', 1)
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

    def solve_IP(self):
        for var in self.model.getVars():
            var.vtype = gp.GRB.BINARY
        self.model.setParam(GRB.Param.TimeLimit, CommonHelper.primal_heuristic_time)
        self.model.update()
        self.model.write("node_IP.lp")
        print("solving node IP...")
        self.model.optimize()
        if self.model.SolCount > 0:
            self.primal_z_vals = {key: var.X for key, var in self.z_dict.items()}
            self.primal_obj_val = self.model.objVal
            self.find_primal_solution = True
            return True
        else:
            return False

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
        run the pricer once, solve the pricing problem
        """
        add_new_column = False
        node_id = node_info.id
        farkas = True if self.status == GRB.INFEASIBLE else False

        executor, stop_evt = get_pricer_pool(CommonHelper.num_threads, CommonHelper.seed, CommonHelper.num_trucks,
                                             CommonHelper.num_customers, CommonHelper.custom_dist,
                                             CommonHelper.num_drones_per_truck, CommonHelper.cw)
        stop_evt.clear()  # reset from any previous run

        futures = [
            executor.submit(
                pricer_task,
                i,
                self.duals,
                farkas,
                node_info,
                stop_evt
            )
            for i in range(CommonHelper.num_threads)
        ]

        reduced_cost = math.inf
        element_path = None
        for future in as_completed(futures):
            thread_id, (print_text, (temp_rc, temp_path)) = future.result()
            if temp_rc + CommonHelper.close_tolerance < 0:
                reduced_cost = temp_rc
                element_path = temp_path
                stop_evt.set()
                for f in futures:
                    if not f.done():
                        f.cancel()
                print(f"\r{print_text}", end="", flush=True)
                break

        # find a new route
        if reduced_cost + CommonHelper.close_tolerance < 0:
            route_key = tuple(element_path)
            new_route = CommonHelper.elem_path_to_route(element_path, len(bp.route_dict), CommonHelper.net,
                                                        CommonHelper.transformed_net)

            bp.route_dict[route_key] = new_route
            # add the column to RMP
            if route_key not in bp.node_infos[node_id].columns:
                bp.node_infos[node_id].columns.append(route_key)
                bp.node_infos[node_id].column_customer_visits[route_key].update(
                    CommonHelper.get_route_customer_visits(bp.route_dict[route_key]))

                self._add_column(route_key)
                add_new_column = True

        return add_new_column

    def _add_column(self, route_key):
        """Add new column (route) to the master problem"""
        route = bp.route_dict[route_key]
        # for SR inequalities
        self.node_info.column_customer_visits[route_key].update(CommonHelper.get_route_customer_visits(route))

        # generate a new variable and assign the coefficient to objective function
        c = gp.Column()
        # customer visit constraints
        for i, n_name in enumerate(CommonHelper.net.customers):
            if CommonHelper.if_route_visit_node(route, n_name):
                c.addTerms(1.0, self.constraints[i])
        # truck fleet constraints
        c.addTerms(1.0, self.vehicle_fleet_branch_lb_cons)
        c.addTerms(1.0, self.vehicle_fleet_branch_ub_cons)
        # for arc flow branching constraints
        for cons_dict in [self.arc_flow_lb_cons_dict, self.arc_flow_ub_cons_dict]:
            for (i, j, frac_val), cons in cons_dict.items():
                if CommonHelper.if_route_travel_arc(route, i, j, CommonHelper.net):
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
        new_add_cons = 0
        if len(self.SR_inequalities) < CommonHelper.SR_num:
            potential_triples = sorted(set(CommonHelper.transformed_net.PI) - set(self.SR_inequalities.keys()))
            for triple in potential_triples:
                covered_customers = self.node_info.column_customer_visits[route_key]
                if len(covered_customers & set(triple)) >= 2:
                    # add a new constraint
                    new_add_cons += 1
                    self.SR_inequalities[triple] = self.model.addConstr(newVar <= 1, name=f"SR_{triple}")
                    self.node_info.SR_infos[triple].append(route_key)
                    self.node_info.added_SR_keys.add(triple)
                    self.node_info.column_in_SR_triples[route_key].append(triple)
                if len(self.SR_inequalities) >= CommonHelper.SR_num:
                    break

        new_add_cons = 0
        potential_triples = sorted(set(CommonHelper.transformed_net.PI) - set(self.SR_inequalities.keys()))
        for triple in potential_triples:
            if new_add_cons >= CommonHelper.SR_num_each_run or len(
                    self.node_info.added_SR_keys) >= CommonHelper.SR_num:
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
            if not CommonHelper.is_integer(val):
                result = False
                fractions[key] = val
        return result, fractions
