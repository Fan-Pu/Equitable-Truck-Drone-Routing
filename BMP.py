import gurobipy as gp
from gurobipy import GRB

from GeneralHelper import *


class BMP:
    """
    Benders master problem for pricing subproblem
    """

    def __init__(self, net):
        self.model = gp.Model("model")
        # add decision variables
        self.x_dict = {}
        for n_name in net.all_nodes:
            for j_name in net.truck_out_arcs[n_name]:
                i = net.all_nodes_indices[n_name]
                j = net.all_nodes_indices[j_name]
                self.x_dict[(i, j)] = self.model.addVar(name=f"x_{(i, j)}", vtype=GRB.BINARY)
        self.phi = self.model.addVar(name="phi", vtype=GRB.CONTINUOUS, lb=0)
        # auxiliary variables
        wk_dict = {}
        for n in range(len(net.all_nodes_indices)):
            wk_dict[n] = self.model.addVar(name=f"wk_{n}", vtype=GRB.CONTINUOUS, lb=0, ub=truck_max_weight)

        # add constraints
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
                    wk_dict[n] >= wk_dict[i] + net.demand_weights[n] + M * (self.x_dict[(i, n)] - 1), f"payload_{n}"))

    def update_objective(self, net, duals):
        # add objective
        obj_expr = self.phi - duals[-1]
        for n_name in net.customers:
            n_idx = net.customers.index(n_name)
            dual = duals[n_idx]
            for i_name in net.truck_in_arcs[n_name]:
                i, n = net.all_nodes_indices[i_name], net.all_nodes_indices[n_name]
                obj_expr -= dual * self.x_dict[(i, n)]
        self.model.setObjective(obj_expr, GRB.MINIMIZE)

    def solve(self):
        self.model.update()
        self.model.write('BMP.lp')
        self.model.optimize()
        if self.model.Status == GRB.OPTIMAL:
            x_vals = {key: var.X for key, var in self.x_dict.items()}
            return self.model.ObjVal, x_vals
        else:
            raise Exception("BMP did not converge")
