import re

import gurobipy as gp
from gurobipy import GRB

import BranchAndPrice


class RMP:
    def __init__(self, net):
        route_dict = BranchAndPrice.route_dict
        self.z_vars = None
        self.sum_z_val = None  # branching term
        self.model = gp.Model("model")
        self.duals = None

        # add decision variables
        self.z_list = []  # no updates
        self.z_keys = []  # value: key of z variable
        obj_expr = 0
        for route_key, route in route_dict.items():
            idx, cost = route['id'], route['cost']
            var = self.model.addVar(name=f"z_{idx}", vtype=GRB.CONTINUOUS, lb=0)
            obj_expr += var * cost
            self.z_list.append(var)
            self.z_keys.append(route_key)

        # set objective
        self.model.setObjective(obj_expr, GRB.MINIMIZE)

        self.constraints = []
        # cons 1
        for i in range(len(net.customers)):
            n_name = net.customers[i]
            visit_route_ids = []
            for idx in range(len(self.z_keys)):
                if n_name in route_dict[self.z_keys[idx]]['truck']:
                    visit_route_ids.append(idx)

            cons = self.model.addConstr(
                gp.quicksum(self.z_list[j] for j in visit_route_ids) == 1,
                name=f"visit_customer_{i}"
            )
            self.constraints.append(cons)
        # cons 2 (truck fleet UB)
        cons = self.model.addConstr(
            gp.quicksum(self.z_list) <= net.num_trucks,
            name=f"truck_fleet"
        )
        self.constraints.append(cons)

    def solve(self, node_id):
        """Solve the master problem"""
        self.model.setParam("OutputFlag", 0)
        # self.model.setParam("NumericFocus", 3)  # Maximize numerical robustness
        self.model.setParam("FeasibilityTol", 1e-9)
        self.model.setParam("OptimalityTol", 1e-9)
        self.model.update()
        self.model.write("RMP.lp")

        self.model.optimize()
        if self.model.Status == GRB.OPTIMAL:
            # print(f"RMP obj value: {self.model.objVal:.4f}")
            node = BranchAndPrice.node_infos[node_id]
            self.duals = {'origin': [c.Pi for c in self.constraints],
                          'branch': [self.model.getConstrByName(branch_info[-1]).Pi for branch_info in node.branches]}
            self.z_vars = [var for var in self.model.getVars() if re.match(r"z_\d+", var.VarName)]
            self.sum_z_val = sum([var.X for var in self.z_vars])
            return self.model.Status, self.duals
        else:
            return self.model.Status, None
