import gurobipy as gp
from gurobipy import GRB


class RMP:
    def __init__(self, net, route_dict):
        self.sum_z_val = None  # branching term
        self.model = gp.Model("model")
        self.duals = None
        self.sum_z = None
        self.branch_cons_LB = None
        self.branch_cons_UB = None

        # add decision variables
        self.z_list = []
        self.z_keys = []  # value: key of z variable
        obj_expr = 0
        for route_key, route in route_dict.items():
            idx, cost = route['id'], route['cost']
            var = self.model.addVar(name=f"z_{idx}", vtype=GRB.CONTINUOUS, lb=0)
            obj_expr += var * cost
            self.z_list.append(var)
            self.z_keys.append(route_key)

        self.sum_z = gp.quicksum(self.z_list)

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
        # cons 2 (truck fleet LB)
        cons = self.model.addConstr(
            gp.quicksum(self.z_list) >= 0,
            name=f"truck_fleet1"
        )
        self.constraints.append(cons)
        self.branch_cons_LB = cons
        # cons 2 (truck fleet UB)
        cons = self.model.addConstr(
            gp.quicksum(self.z_list) <= net.num_trucks,
            name=f"truck_fleet2"
        )
        self.constraints.append(cons)
        self.branch_cons_UB = cons

    def solve(self):
        """Solve the master problem"""
        self.model.setParam("OutputFlag", 0)
        # self.model.setParam("NumericFocus", 3)  # Maximize numerical robustness
        self.model.setParam("FeasibilityTol", 1e-9)
        self.model.setParam("OptimalityTol", 1e-9)
        self.model.update()
        self.model.write("RMP.lp")

        self.model.optimize()
        if self.model.Status == GRB.OPTIMAL:
            print(f"RMP obj value: {self.model.objVal:.4f}")
            self.duals = [c.Pi for c in self.constraints]
            self.sum_z_val = self.sum_z.getValue()
            return self.model.Status, self.duals
        else:
            return self.model.Status, None
