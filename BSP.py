import gurobipy as gp
from gurobipy import GRB

from GeneralHelper import *


class BSP:
    """
    Benders subproblem for pricing subproblem
    """

    def __init__(self, net):
        self.route_cost = None
        self.model = gp.Model("model")
        # add decision variables
        self.y_dict = {}
        for n in net.all_nodes:
            for j in net.drone_out_arcs[n]:
                i = net.all_nodes_indices[n]
                j = net.all_nodes_indices[j]
                for d in range(net.total_drone_num):
                    self.y_dict[(i, j, d)] = self.model.addVar(name=f"y_{(i, j, d)}", vtype=GRB.BINARY)
        # auxiliary variables
        self.ad_dict = {}
        self.ak_dict = {}
        self.a_dict = {}
        self.t_dict = {}
        for n_name in net.all_nodes:
            n = net.all_nodes_indices[n_name]
            if n_name in net.customers:
                self.a_dict[n] = self.model.addVar(name=f"a_{n}", vtype=GRB.CONTINUOUS, lb=0)
            # waiting time
            self.t_dict[n] = self.model.addVar(name=f"t_{n}", vtype=GRB.CONTINUOUS, lb=0)
            self.ak_dict[n] = self.model.addVar(name=f"ak_{n}", vtype=GRB.CONTINUOUS, lb=net.a_lb[n_name])
            for d in range(num_drones_per_truck):
                self.ad_dict[(n, d)] = self.model.addVar(name=f"ad_{(n, d)}", vtype=GRB.CONTINUOUS, lb=net.a_lb[n_name])
        # the variables for binding the x values
        self.x_dict = {}
        for n_name in net.all_nodes:
            for j_name in net.truck_out_arcs[n_name]:
                i = net.all_nodes_indices[n_name]
                j = net.all_nodes_indices[j_name]
                self.x_dict[(i, j)] = self.model.addVar(name=f"x_{(i, j)}", vtype=GRB.CONTINUOUS)

        # add constraints
        self.constraints = []
        self.binding_constraints = {}

        # customer serve once ************************************************************
        for n_name in net.customers:
            n = net.all_nodes_indices[n_name]
            lhs = gp.quicksum(
                self.x_dict[(net.all_nodes_indices[i_name], n)]
                for i_name in net.truck_in_arcs[n_name]
            ) + gp.quicksum(
                self.y_dict[(net.all_nodes_indices[i_name], n, d)]
                for i_name in net.drone_in_arcs[n_name]
                for d in range(num_drones_per_truck)
            )
            self.constraints.append(self.model.addConstr(lhs <= 1, f"servonce_{n}"))

        # drone launch ************************************************************
        for n_name in net.hubs:
            n = net.all_nodes_indices[n_name]
            lhs = gp.quicksum(
                self.y_dict[(n, net.all_nodes_indices[j_name], d)]
                for d in range(num_drones_per_truck)
                for j_name in net.drone_out_arcs[n_name]
            )
            rhs = num_drones_per_truck * gp.quicksum(
                self.x_dict[(net.all_nodes_indices[i_name], n)]
                for i_name in net.truck_in_arcs[n_name]
            )
            self.constraints.append(self.model.addConstr(lhs <= rhs, f"drone_launch_{n}"))

        # truck waiting times ************************************************************
        # cons 1
        for n_name in net.hubs:
            n = net.all_nodes_indices[n_name]
            for j_name in net.drone_out_arcs[n_name]:
                j = net.all_nodes_indices[j_name]
                travel_time = net.drone_travel_times[(n_name, j_name)]
                rhs = gp.quicksum(
                    2 * travel_time * self.y_dict[(n, j, d)]
                    for d in range(num_drones_per_truck)
                )
                self.constraints.append(
                    self.model.addConstr(self.t_dict[n] >= rhs, f"wait1_{n, j}")
                )
        # cons 2
        for n_name in (n for n in net.all_nodes if n not in net.hubs):
            n = net.all_nodes_indices[n_name]
            self.constraints.append(
                self.model.addConstr(self.t_dict[n] <= 0, f"wait2_{n}"))

        # Realized service times ************************************************************
        # cons 1
        lhs = self.ak_dict[net.all_nodes_indices[net.depot_source]] + gp.quicksum(
            self.ad_dict[(net.all_nodes_indices[net.depot_source], d)] for d in range(num_drones_per_truck)
        )
        self.constraints.append(self.model.addConstr(lhs <= 0, "realized1"))
        # cons 2
        for n_name in net.customers:
            n = net.all_nodes_indices[n_name]
            self.constraints.append(self.model.addConstr(self.a_dict[n] >= self.ak_dict[n], f"realized2_{n}"))
        # cons 3
        for n_name in net.customers:
            n = net.all_nodes_indices[n_name]
            for d in range(num_drones_per_truck):
                self.constraints.append(
                    self.model.addConstr(self.a_dict[n] >= self.ad_dict[(n, d)], f"realized3_{n, d}")
                )
        # cons 4
        for n_name in net.all_nodes:
            if n_name != net.depot_source:
                n = net.all_nodes_indices[n_name]
                for i_name in net.truck_in_arcs[n_name]:
                    i = net.all_nodes_indices[i_name]
                    travel_time = net.truck_travel_times[(i_name, n_name)]
                    rhs = (
                            self.ak_dict[i] + self.t_dict[i] + travel_time * self.x_dict[(i, n)] + M * (
                            self.x_dict[(i, n)] - 1)
                    )
                    self.constraints.append(self.model.addConstr(self.ak_dict[n] >= rhs, f"realized4_{i, n}"))
        # cons 5
        for n_name in net.hubs:
            n = net.all_nodes_indices[n_name]
            for j_name in net.drone_out_arcs[n_name]:
                j = net.all_nodes_indices[j_name]
                travel_time = net.drone_travel_times[(n_name, j_name)]
                for d in range(num_drones_per_truck):
                    rhs = (
                            self.ak_dict[n] + travel_time * self.y_dict[(n, j, d)] + M * (self.y_dict[(n, j, d)] - 1)
                    )
                    self.constraints.append(self.model.addConstr(self.ad_dict[(j, d)] >= rhs, f"realized5_{n, j, d}"))

        # binding constraints ************************************************************
        for key, var in self.x_dict.items():
            cons = self.model.addConstr(var == 0, f"bind_{key}")
            self.constraints.append(cons)
            self.binding_constraints[key] = cons

    def update_objective(self, net, duals):
        # add objective
        obj_expr = 0
        for i in range(len(duals['origin']) - 1):
            dual = duals['origin'][i]
            n_name = net.customers[i]
            for i_name in net.drone_in_arcs[n_name]:
                for d in range(num_drones_per_truck):
                    obj_expr += -dual * self.y_dict[
                        (net.all_nodes_indices[i_name], net.all_nodes_indices[n_name], d)]

        # obj_expr = gp.quicksum(
        #     -1000 * self.y_dict[
        #         (net.all_nodes_indices[i_name], net.all_nodes_indices[n_name], d)]
        #     for n_name in net.customers
        #     for i_name in net.drone_in_arcs[n_name]
        #     for d in range(num_drones_per_truck)
        # )
        self.route_cost = gp.quicksum(
            cost_scale * (self.a_dict[net.all_nodes_indices[n_name]] - net.a_lb[n_name])
            for n_name in net.customers
        )
        self.route_cost += self.ak_dict[net.all_nodes_indices[net.depot_sink]]
        obj_expr += self.route_cost
        self.model.setObjective(obj_expr, GRB.MINIMIZE)

    def update_binding_cons(self, x_vals):
        for key, val in x_vals.items():
            self.binding_constraints[key].rhs = val

    def solveIP(self):
        self.model.setParam("OutputFlag", 0)
        # change the model to IP
        for var in self.y_dict.values():
            var.setAttr("VType", GRB.BINARY)
        self.model.update()
        # self.model.write('BSP.lp')
        self.model.optimize()
        if self.model.Status == GRB.OPTIMAL:
            # print(f"BSP IP obj: {self.model.objVal:.4f}", end="")
            y_vals = {key: var.X for key, var in self.y_dict.items()}
            ak_vals = {key: var.X for key, var in self.ak_dict.items()}
            if abs(self.model.ObjVal) <= 0.001:
                sdas = 0
            return self.model.ObjVal, y_vals, self.route_cost.getValue()
        else:
            raise Exception("BMP did not converge")

    def solveLP(self):
        self.model.setParam("OutputFlag", 0)
        # self.model.setParam("NumericFocus", 3)  # Maximize numerical robustness
        self.model.setParam("FeasibilityTol", 1e-9)
        self.model.setParam("OptimalityTol", 1e-9)
        # change the model to LP
        for var in self.y_dict.values():
            var.setAttr("VType", GRB.CONTINUOUS)
            var.setAttr("LB", 0)
            var.setAttr("UB", 1)
        # self.model.setParam("Method", 0)  # Enforce simplex method
        self.model.update()
        self.model.write('BSP.lp')
        self.model.optimize()
        if self.model.Status == GRB.OPTIMAL:
            # print(f"BSP LP obj: {self.model.objVal:.4f}", end="    ")
            # vals = {key: var.X for key, var in self.y_dict.items()}
            duals = {key: c.Pi for key, c in self.binding_constraints.items()}
            return self.model.ObjVal, duals
        else:
            raise Exception("BMP did not converge")
