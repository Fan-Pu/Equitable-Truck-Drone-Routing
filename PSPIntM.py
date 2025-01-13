import gurobipy as gp
from gurobipy import GRB

from GeneralHelper import *


class PSPIntM:
    def __init__(self, net):
        self.model = gp.Model("model")
        # variables
        self.x_dict = {
            (net.all_nodes_indices[n_name], net.all_nodes_indices[j_name]): self.model.addVar(
                name=f"x_{(net.all_nodes_indices[n_name], net.all_nodes_indices[j_name])}",
                vtype=GRB.BINARY
            )
            for n_name in net.all_nodes
            for j_name in net.truck_out_arcs[n_name]
        }

        self.wk_dict = {
            n: self.model.addVar(name=f"wk_{n}", vtype=GRB.CONTINUOUS, lb=0, ub=truck_max_weight)
            for n in range(len(net.all_nodes_indices))
        }

        self.y_dict = {
            (net.all_nodes_indices[n], net.all_nodes_indices[j], d): self.model.addVar(
                name=f"y_{(net.all_nodes_indices[n], net.all_nodes_indices[j], d)}",
                vtype=GRB.BINARY
            )
            for n in net.all_nodes
            for j in net.drone_out_arcs[n]
            for d in range(net.total_drone_num)
        }

        self.a_dict = {}
        self.t_dict = {}
        self.ak_dict = {}
        self.ad_dict = {
            (net.all_nodes_indices[n_name], d): self.model.addVar(
                name=f"ad_{(net.all_nodes_indices[n_name], d)}",
                vtype=GRB.CONTINUOUS,
                lb=net.a_lb[n_name]
            )
            for n_name in net.all_nodes
            for d in range(num_drones_per_truck)
        }

        for n_name in net.all_nodes:
            n = net.all_nodes_indices[n_name]
            if n_name in net.customers:
                self.a_dict[n] = self.model.addVar(name=f"a_{n}", vtype=GRB.CONTINUOUS, lb=0)
            self.t_dict[n] = self.model.addVar(name=f"t_{n}", vtype=GRB.CONTINUOUS, lb=0)
            self.ak_dict[n] = self.model.addVar(name=f"ak_{n}", vtype=GRB.CONTINUOUS, lb=net.a_lb[n_name])

        self.constraints = []
        # flow conservation ************************************************************
        # cons 1
        lhs = gp.quicksum(
            self.x_dict[(net.all_nodes_indices[net.depot_source], net.all_nodes_indices[j_name])]
            for j_name in net.truck_out_arcs[net.depot_source]
        )
        rhs = gp.quicksum(
            self.x_dict[(net.all_nodes_indices[i_name], net.all_nodes_indices[net.depot_sink])]
            for i_name in net.truck_in_arcs[net.depot_sink]
        )
        self.constraints.append(self.model.addConstr(lhs == rhs, f"conserv1"))
        # cons 2
        for idx, n_name in enumerate(net.all_nodes):
            if n_name in {net.depot_source, net.depot_sink}:
                continue
            n = net.all_nodes_indices[n_name]
            lhs = gp.quicksum(
                self.x_dict[(n, net.all_nodes_indices[j_name])]
                for j_name in net.truck_out_arcs[n_name]
            )
            rhs = gp.quicksum(
                self.x_dict[(net.all_nodes_indices[i_name], n)]
                for i_name in net.truck_in_arcs[n_name]
            )
            self.constraints.append(self.model.addConstr(lhs == rhs, f"conserv2_{idx}"))
        # cons 3
        lhs = gp.quicksum(
            self.x_dict[(net.all_nodes_indices[net.depot_source], net.all_nodes_indices[j_name])]
            for j_name in net.truck_out_arcs[net.depot_source]
        )
        self.constraints.append(self.model.addConstr(lhs == 1, f"conserv3"))

        # payload ************************************************************
        for n_name in net.customers + net.hubs:
            n = net.all_nodes_indices[n_name]
            for i_name in net.truck_in_arcs[n_name]:
                i = net.all_nodes_indices[i_name]
                self.constraints.append(self.model.addConstr(
                    self.wk_dict[n] >= self.wk_dict[i] + net.demand_weights[n] + M * (self.x_dict[(i, n)] - 1),
                    f"payload_{n}"))

        # ****************************************** BSP **************************************************
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
                            self.ak_dict[n] + travel_time * self.y_dict[(n, j, d)] + M * (
                            self.y_dict[(n, j, d)] - 1)
                    )
                    self.constraints.append(
                        self.model.addConstr(self.ad_dict[(j, d)] >= rhs, f"realized5_{n, j, d}"))

    def update_objective(self, net, duals):
        # add objective
        obj_expr = duals[-2] - duals[-1]
        for n_name in net.customers:
            n_idx = net.customers.index(n_name)
            dual = duals[n_idx]
            for i_name in net.truck_in_arcs[n_name]:
                i, n = net.all_nodes_indices[i_name], net.all_nodes_indices[n_name]
                obj_expr -= dual * self.x_dict[(i, n)]

        obj_expr += gp.quicksum(
            -duals[net.customers.index(n_name)] * self.y_dict[
                (net.all_nodes_indices[i_name], net.all_nodes_indices[n_name], d)]
            for n_name in net.customers
            for i_name in net.drone_in_arcs[n_name]
            for d in range(num_drones_per_truck)
        )

        self.route_cost = gp.quicksum(
            cost_scale * (self.a_dict[net.all_nodes_indices[n_name]] - net.a_lb[n_name])
            for n_name in net.customers
        )
        obj_expr += self.route_cost

        self.model.setObjective(obj_expr, GRB.MINIMIZE)

    def solve(self):
        self.model.setParam("OutputFlag", 0)
        self.model.update()
        # self.model.write('IntM.lp')
        self.model.optimize()
        if self.model.Status == GRB.OPTIMAL:
            return self.model.ObjVal
        else:
            raise Exception("IntM did not converge")
