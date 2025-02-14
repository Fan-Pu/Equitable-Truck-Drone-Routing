from pyscipopt import Model, quicksum, scip

import GeneralHelper
from GeneralHelper import *


class BSP:
    """
    Benders subproblem for pricing problem
    """

    def __init__(self):
        self.route_cost = None
        self.model = Model("BSP")
        # disable presolve
        self.model.setPresolve(scip.PY_SCIP_PARAMSETTING.OFF)
        self.model.setHeuristics(scip.PY_SCIP_PARAMSETTING.OFF)
        self.model.disablePropagation()
        self.model.setParam("presolving/maxrounds", 0)
        self.model.setParam("display/verblevel", 0)  # for logging
        # add decision variables
        self.y_dict = {}
        for n in GeneralHelper.net.all_nodes:
            for j in GeneralHelper.net.drone_out_arcs[n]:
                i = GeneralHelper.net.all_nodes_indices[n]
                j = GeneralHelper.net.all_nodes_indices[j]
                for d in range(GeneralHelper.net.total_drone_num):
                    self.y_dict[(i, j, d)] = self.model.addVar(name=f"y_{(i, j, d)}", vtype="BINARY")
        # auxiliary variables
        self.ad_dict = {}
        self.ak_dict = {}
        self.a_dict = {}
        self.t_dict = {}
        for n_name in GeneralHelper.net.all_nodes:
            n = GeneralHelper.net.all_nodes_indices[n_name]
            if n_name in GeneralHelper.net.customers:
                self.a_dict[n] = self.model.addVar(name=f"a_{n}", vtype="CONTINUOUS", lb=0)
            self.t_dict[n] = self.model.addVar(name=f"t_{n}", vtype="CONTINUOUS", lb=0)
            self.ak_dict[n] = self.model.addVar(name=f"ak_{n}", vtype="CONTINUOUS", lb=GeneralHelper.net.a_lb[n_name])
            for d in range(num_drones_per_truck):
                self.ad_dict[(n, d)] = self.model.addVar(name=f"ad_{(n, d)}", vtype="CONTINUOUS",
                                                         lb=GeneralHelper.net.a_lb[n_name])

        # the variables for binding the x values
        self.x_dict = {}
        for n_name in GeneralHelper.net.all_nodes:
            for j_name in GeneralHelper.net.truck_out_arcs[n_name]:
                i = GeneralHelper.net.all_nodes_indices[n_name]
                j = GeneralHelper.net.all_nodes_indices[j_name]
                self.x_dict[(i, j)] = self.model.addVar(name=f"x_{(i, j)}", vtype="CONTINUOUS")

        # add constraints
        self.constraints = []
        self.binding_constraints = {}

        # customer serve once ************************************************************
        for n_name in GeneralHelper.net.customers:
            n = GeneralHelper.net.all_nodes_indices[n_name]
            lhs = quicksum(
                self.x_dict[(GeneralHelper.net.all_nodes_indices[i_name], n)]
                for i_name in GeneralHelper.net.truck_in_arcs[n_name]
            ) + quicksum(
                self.y_dict[(GeneralHelper.net.all_nodes_indices[i_name], n, d)]
                for i_name in GeneralHelper.net.drone_in_arcs[n_name]
                for d in range(num_drones_per_truck)
            )
            self.constraints.append(self.model.addCons(lhs <= 1, name=f"servonce_{n}"))

        # drone launch ************************************************************
        for n_name in GeneralHelper.net.hubs:
            n = GeneralHelper.net.all_nodes_indices[n_name]
            lhs = quicksum(
                self.y_dict[(n, GeneralHelper.net.all_nodes_indices[j_name], d)]
                for d in range(num_drones_per_truck)
                for j_name in GeneralHelper.net.drone_out_arcs[n_name]
            )
            rhs = num_drones_per_truck * quicksum(
                self.x_dict[(GeneralHelper.net.all_nodes_indices[i_name], n)]
                for i_name in GeneralHelper.net.truck_in_arcs[n_name]
            )
            self.constraints.append(self.model.addCons(lhs <= rhs, name=f"drone_launch_{n}"))

        # truck waiting times ************************************************************
        # cons 1
        for n_name in GeneralHelper.net.hubs:
            n = GeneralHelper.net.all_nodes_indices[n_name]
            for j_name in GeneralHelper.net.drone_out_arcs[n_name]:
                j = GeneralHelper.net.all_nodes_indices[j_name]
                travel_time = GeneralHelper.net.drone_travel_times[(n_name, j_name)]
                rhs = quicksum(
                    2 * travel_time * self.y_dict[(n, j, d)]
                    for d in range(num_drones_per_truck)
                )
                self.constraints.append(self.model.addCons(self.t_dict[n] >= rhs, name=f"wait1_{n}_{j}"))
        # cons 2
        for n_name in (n for n in GeneralHelper.net.all_nodes if n not in GeneralHelper.net.hubs):
            n = GeneralHelper.net.all_nodes_indices[n_name]
            self.constraints.append(self.model.addCons(self.t_dict[n] <= 0, name=f"wait2_{n}"))

        # Realized service times ************************************************************
        # cons 1
        lhs = self.ak_dict[GeneralHelper.net.all_nodes_indices[GeneralHelper.net.depot_source]] + quicksum(
            self.ad_dict[(GeneralHelper.net.all_nodes_indices[GeneralHelper.net.depot_source], d)] for d in
            range(num_drones_per_truck)
        )
        self.constraints.append(self.model.addCons(lhs <= 0, "realized1"))
        # cons 2
        for n_name in GeneralHelper.net.customers:
            n = GeneralHelper.net.all_nodes_indices[n_name]
            self.constraints.append(self.model.addCons(self.a_dict[n] >= self.ak_dict[n], name=f"realized2_{n}"))
        # cons 3
        for n_name in GeneralHelper.net.customers:
            n = GeneralHelper.net.all_nodes_indices[n_name]
            for d in range(num_drones_per_truck):
                self.constraints.append(
                    self.model.addCons(self.a_dict[n] >= self.ad_dict[(n, d)], name=f"realized3_{n}_{d}")
                )
        # cons 4
        for n_name in GeneralHelper.net.all_nodes:
            if n_name != GeneralHelper.net.depot_source:
                n = GeneralHelper.net.all_nodes_indices[n_name]
                for i_name in GeneralHelper.net.truck_in_arcs[n_name]:
                    i = GeneralHelper.net.all_nodes_indices[i_name]
                    travel_time = GeneralHelper.net.truck_travel_times[(i_name, n_name)]
                    rhs = (self.ak_dict[i] + self.t_dict[i] + travel_time * self.x_dict[(i, n)] +
                           M * (self.x_dict[(i, n)] - 1))
                    self.constraints.append(self.model.addCons(self.ak_dict[n] >= rhs, name=f"realized4_{i}_{n}"))

        # cons 5
        for n_name in GeneralHelper.net.hubs:
            n = GeneralHelper.net.all_nodes_indices[n_name]
            for j_name in GeneralHelper.net.drone_out_arcs[n_name]:
                j = GeneralHelper.net.all_nodes_indices[j_name]
                travel_time = GeneralHelper.net.drone_travel_times[(n_name, j_name)]
                for d in range(num_drones_per_truck):
                    rhs = (self.ak_dict[n] + travel_time * self.y_dict[(n, j, d)] + M * (self.y_dict[(n, j, d)] - 1))
                    self.constraints.append(
                        self.model.addCons(self.ad_dict[(j, d)] >= rhs, name=f"realized5_{n}_{j}_{d}")
                    )

        # binding constraints ************************************************************
        for key, var in self.x_dict.items():
            # lower bound
            cons_lb = self.model.addCons(var >= 0, name=f"bind_lb_{key}")
            self.constraints.append(cons_lb)
            # upper bound
            cons_ub = self.model.addCons(var <= 0, name=f"bind_ub_{key}")
            self.constraints.append(cons_ub)
            self.binding_constraints[key] = (cons_lb, cons_ub)

        # self.model.writeProblem("BSP.lp")

        # transform constraints
        # for i in range(len(self.constraints)):
        #     self.constraints[i] = self.model.getTransformedCons(self.constraints[i])
        # for key in self.binding_constraints.keys():
        #     self.binding_constraints[key] = self.model.getTransformedCons(self.binding_constraints[key])

    def update_objective(self, duals):
        self.model.freeTransform()
        # add objective
        obj_expr = 0
        for i in range(len(duals) - 1):
            dual = duals[i]
            n_name = GeneralHelper.net.customers[i]
            for i_name in GeneralHelper.net.drone_in_arcs[n_name]:
                for d in range(num_drones_per_truck):
                    obj_expr += -dual * self.y_dict[
                        (GeneralHelper.net.all_nodes_indices[i_name], GeneralHelper.net.all_nodes_indices[n_name], d)]

        self.route_cost = quicksum(
            cost_scale * (self.a_dict[GeneralHelper.net.all_nodes_indices[n_name]] - GeneralHelper.net.a_lb[n_name])
            for n_name in GeneralHelper.net.customers)
        self.route_cost += self.ak_dict[GeneralHelper.net.all_nodes_indices[GeneralHelper.net.depot_sink]]
        obj_expr += self.route_cost
        self.model.setObjective(obj_expr, "minimize")

    def update_binding_cons(self, x_vals):
        self.model.freeTransform()
        """ Update the right-hand side of the binding constraints with given x values. """
        for key, var in self.x_dict.items():
            old_cons_lb, old_cons_ub = self.binding_constraints[key]
            # remove the old constraints
            self.model.delCons(old_cons_lb)
            self.model.delCons(old_cons_ub)

            val = x_vals[key]
            # lower bound
            cons_lb = self.model.addCons(var >= val, name=f"bind_lb_{key}")
            # upper bound
            cons_ub = self.model.addCons(var <= val, name=f"bind_ub_{key}")
            self.binding_constraints[key] = (cons_lb, cons_ub)

    def solveIP(self):
        """ Solve the BSP as an Integer Program (IP). """
        self.model.freeTransform()  # back to problem creation stage
        # retrieve the IP
        for var in self.y_dict.values():
            self.model.chgVarType(var, "BINARY")
        # self.model.writeProblem("BSP.lp")

        self.model.optimize()
        if self.model.getStatus() == "optimal":
            y_vals = {key: self.model.getVal(var) for key, var in self.y_dict.items()}
            ak_vals = {key: self.model.getVal(var) for key, var in self.ak_dict.items()}
            return self.model.getObjVal(), y_vals, self.model.getVal(self.route_cost)
        else:
            raise Exception("BSP did not converge")

    def solveLP(self):
        """ Solve the BSP as a Linear Program (LP). """
        self.model.relax()
        self.model.writeProblem("BSP.lp")
        self.model.optimize()
        if self.model.getStatus() == "optimal":
            duals = {
                key: self.model.getDualsolLinear(self.model.getTransformedCons(cons_lb)) - self.model.getDualsolLinear(
                    self.model.getTransformedCons(cons_ub)) for key, (cons_lb, cons_ub) in
                self.binding_constraints.items()}
            return self.model.getObjVal(), duals
        else:
            raise Exception("BSP did not converge")
