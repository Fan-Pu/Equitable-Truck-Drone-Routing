from pyscipopt import Model, quicksum

import GeneralHelper
from GeneralHelper import *


class BMP:
    """
    Benders master problem for pricing subproblem
    """

    def __init__(self):
        self.model = Model("BMP")
        self.model.setParam("display/verblevel", 0)
        self.subgradient_cuts = []
        self.Lshaped_cuts = []
        # add decision variables
        self.x_dict = {}
        for n_name in GeneralHelper.net.all_nodes:
            for j_name in GeneralHelper.net.truck_out_arcs[n_name]:
                i = GeneralHelper.net.all_nodes_indices[n_name]
                j = GeneralHelper.net.all_nodes_indices[j_name]
                self.x_dict[(i, j)] = self.model.addVar(name=f"x_{(i, j)}", vtype="BINARY")
        self.phi = self.model.addVar(name="phi", vtype="CONTINUOUS")
        # auxiliary variables
        self.wk_dict = {}
        for n in range(len(GeneralHelper.net.all_nodes_indices)):
            self.wk_dict[n] = self.model.addVar(name=f"wk_{n}", vtype="CONTINUOUS", lb=0, ub=truck_max_weight)

        # add constraints
        self.constraints = []
        # flow conservation ************************************************************
        # cons 1
        lhs = quicksum(
            self.x_dict[(GeneralHelper.net.all_nodes_indices[GeneralHelper.net.depot_source],
                         GeneralHelper.net.all_nodes_indices[j_name])]
            for j_name in GeneralHelper.net.truck_out_arcs[GeneralHelper.net.depot_source]
        )
        rhs = quicksum(
            self.x_dict[(GeneralHelper.net.all_nodes_indices[i_name],
                         GeneralHelper.net.all_nodes_indices[GeneralHelper.net.depot_sink])]
            for i_name in GeneralHelper.net.truck_in_arcs[GeneralHelper.net.depot_sink]
        )
        self.constraints.append(self.model.addCons(lhs == rhs, name="conserv1"))
        # cons 2
        for idx, n_name in enumerate(GeneralHelper.net.all_nodes):
            if n_name in {GeneralHelper.net.depot_source, GeneralHelper.net.depot_sink}:
                continue
            n = GeneralHelper.net.all_nodes_indices[n_name]
            lhs = quicksum(
                self.x_dict[(n, GeneralHelper.net.all_nodes_indices[j_name])]
                for j_name in GeneralHelper.net.truck_out_arcs[n_name]
            )
            rhs = quicksum(
                self.x_dict[(GeneralHelper.net.all_nodes_indices[i_name], n)]
                for i_name in GeneralHelper.net.truck_in_arcs[n_name]
            )
            self.constraints.append(self.model.addCons(lhs == rhs, name=f"conserv2_{idx}"))
        # cons 3
        lhs = quicksum(
            self.x_dict[(GeneralHelper.net.all_nodes_indices[GeneralHelper.net.depot_source],
                         GeneralHelper.net.all_nodes_indices[j_name])]
            for j_name in GeneralHelper.net.truck_out_arcs[GeneralHelper.net.depot_source]
        )
        self.constraints.append(self.model.addCons(lhs == 1, name="conserv3"))

        # payload ************************************************************
        for n_name in GeneralHelper.net.customers + GeneralHelper.net.hubs:
            n = GeneralHelper.net.all_nodes_indices[n_name]
            for i_name in GeneralHelper.net.truck_in_arcs[n_name]:
                i = GeneralHelper.net.all_nodes_indices[i_name]
                self.constraints.append(self.model.addCons(
                    self.wk_dict[n] >= self.wk_dict[i] + GeneralHelper.net.demand_weights[n] + M * (
                            self.x_dict[(i, n)] - 1),
                    name=f"payload_{n}"
                ))

        # initialize variables for BSP acceleration
        self.y_dict = {}
        for n in GeneralHelper.net.all_nodes:
            for j in GeneralHelper.net.drone_out_arcs[n]:
                i = GeneralHelper.net.all_nodes_indices[n]
                j = GeneralHelper.net.all_nodes_indices[j]
                for d in range(GeneralHelper.net.total_drone_num):
                    self.y_dict[(i, j, d)] = self.model.addVar(name=f"y_{(i, j, d)}", vtype="CONTINUOUS")
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
            self.constraints.append(self.model.addCons(lhs <= rhs, f"drone_launch_{n}"))

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
                self.constraints.append(
                    self.model.addCons(self.t_dict[n] >= rhs, f"wait1_{n, j}")
                )
        # cons 2
        for n_name in (n for n in GeneralHelper.net.all_nodes if n not in GeneralHelper.net.hubs):
            n = GeneralHelper.net.all_nodes_indices[n_name]
            self.constraints.append(
                self.model.addCons(self.t_dict[n] <= 0, f"wait2_{n}"))

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
            self.constraints.append(self.model.addCons(self.a_dict[n] >= self.ak_dict[n], f"realized2_{n}"))
        # cons 3
        for n_name in GeneralHelper.net.customers:
            n = GeneralHelper.net.all_nodes_indices[n_name]
            for d in range(num_drones_per_truck):
                self.constraints.append(
                    self.model.addCons(self.a_dict[n] >= self.ad_dict[(n, d)], f"realized3_{n, d}")
                )
        # cons 4
        for n_name in GeneralHelper.net.all_nodes:
            if n_name != GeneralHelper.net.depot_source:
                n = GeneralHelper.net.all_nodes_indices[n_name]
                for i_name in GeneralHelper.net.truck_in_arcs[n_name]:
                    i = GeneralHelper.net.all_nodes_indices[i_name]
                    travel_time = GeneralHelper.net.truck_travel_times[(i_name, n_name)]
                    rhs = (self.ak_dict[i] + self.t_dict[i] + travel_time * self.x_dict[(i, n)] + M * (
                            self.x_dict[(i, n)] - 1)
                           )
                    self.constraints.append(self.model.addCons(self.ak_dict[n] >= rhs, f"realized4_{i, n}"))
        # cons 5
        for n_name in GeneralHelper.net.hubs:
            n = GeneralHelper.net.all_nodes_indices[n_name]
            for j_name in GeneralHelper.net.drone_out_arcs[n_name]:
                j = GeneralHelper.net.all_nodes_indices[j_name]
                travel_time = GeneralHelper.net.drone_travel_times[(n_name, j_name)]
                for d in range(num_drones_per_truck):
                    rhs = (self.ak_dict[n] + travel_time * self.y_dict[(n, j, d)] + M * (self.y_dict[(n, j, d)] - 1)
                           )
                    self.constraints.append(
                        self.model.addCons(self.ad_dict[(j, d)] >= rhs, f"realized5_{n, j, d}"))

        # add phi cons
        self.phi_cons = None

    def update_objective_and_BSP_Lb(self, duals):
        self.model.freeTransform()
        # update the BSP lower bound
        GeneralHelper.BSP_LB = 0
        for i in range(len(duals) - 1):
            dual = duals[i]
            if dual < 0:
                continue
            n_name = GeneralHelper.net.customers[i]
            GeneralHelper.BSP_LB += -dual * len(GeneralHelper.net.drone_in_arcs[n_name]) * num_drones_per_truck
        self.model.chgVarLb(self.phi, GeneralHelper.BSP_LB)
        # Update the objective function
        obj_expr = self.phi + duals[-1]
        for i in range(len(duals) - 1):
            dual = duals[i]
            n_name = GeneralHelper.net.customers[i]
            for i_name in GeneralHelper.net.truck_in_arcs[n_name]:
                i, n = GeneralHelper.net.all_nodes_indices[i_name], GeneralHelper.net.all_nodes_indices[n_name]
                obj_expr += -dual * self.x_dict[(i, n)]
        self.model.setObjective(obj_expr, "minimize")

        # Update the phi constraint
        BSP_obj_expr = 0
        for i in range(len(duals) - 1):
            dual = duals[i]
            n_name = GeneralHelper.net.customers[i]
            for i_name in GeneralHelper.net.drone_in_arcs[n_name]:
                for d in range(num_drones_per_truck):
                    BSP_obj_expr += -dual * self.y_dict[
                        (GeneralHelper.net.all_nodes_indices[i_name], GeneralHelper.net.all_nodes_indices[n_name], d)]

        # get the route cost
        route_cost = quicksum(
            cost_scale * (self.a_dict[GeneralHelper.net.all_nodes_indices[n_name]] - GeneralHelper.net.a_lb[n_name])
            for n_name in GeneralHelper.net.customers
        )
        route_cost += self.ak_dict[GeneralHelper.net.all_nodes_indices[GeneralHelper.net.depot_sink]]

        BSP_obj_expr += route_cost

        # Update the phi constraint
        if hasattr(self, "phi_cons") and self.phi_cons is not None:
            self.model.delCons(self.phi_cons)  # Remove the old phi constraint
        self.phi_cons = self.model.addCons(self.phi >= BSP_obj_expr, name="phi_cons")

    def solve(self):
        """Solve the BMP."""
        # self.model.writeProblem("BMP.lp")
        self.model.optimize()
        if self.model.getStatus() == "optimal":
            x_vals = {key: self.model.getVal(var) for key, var in self.x_dict.items()}
            phi_value = self.model.getVal(self.phi)
            return self.model.getObjVal(), x_vals, phi_value
        else:
            raise Exception("BMP did not converge")

    def remove_cuts(self):
        """Remove all Benders cuts."""
        self.model.freeTransform()
        for cut in self.subgradient_cuts + self.Lshaped_cuts:
            self.model.delCons(cut)
        self.subgradient_cuts.clear()
        self.Lshaped_cuts.clear()

    def separate_subgradient_cut(self, LP_obj_val, duals, x_vals):
        self.model.freeTransform()
        rhs = LP_obj_val
        for key, val in duals.items():
            rhs += val * (self.x_dict[key] - x_vals[key])
        cons = self.model.addCons(self.phi >= rhs, f"subgrad_{len(self.subgradient_cuts)}")
        self.subgradient_cuts.append(cons)
        self.constraints.append(cons)
        # check the validness of this constraint
        rhs_val = self.model.getVal(rhs)
        phi_val = self.model.getVal(self.phi)
        # the phi_val is wrong
        if phi_val > LP_obj_val:
            sdsa = 0
        # the node is not cut
        if phi_val >= rhs_val:
            sdas = 0

    def separate_L_shaped_cut(self, obj_val, x_vals, L):
        # Identify the set where x values are close to 1
        S_set = [key for key, val in x_vals.items() if val + close_tolerance >= 1]

        # Construct the RHS of the L-shaped cut
        rhs_expr = obj_val + (obj_val - L) * (
                -len(S_set) +
                quicksum(self.x_dict[key] for key in S_set) -
                quicksum(var for key, var in self.x_dict.items() if key not in S_set)
        )

        # Check the correctness of the constraint
        rhs_val = self.model.getVal(rhs_expr)  # Evaluate the RHS expression
        phi_val = self.model.getVal(self.phi)  # Get the current value of phi

        self.model.freeTransform()
        # Add the L-shaped cut to the BMP model
        cons = self.model.addCons(self.phi >= rhs_expr, name=f"Lshaped_{len(self.Lshaped_cuts)}")
        self.Lshaped_cuts.append(cons)
        self.constraints.append(cons)
