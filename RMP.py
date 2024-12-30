import gurobipy as gp
from gurobipy import GRB

cost_scale = 0.1


def find_initial_routes(net):
    routes = []
    route = [net.depot_source, net.customers[0]]
    arrive_time = net.truck_travel_times[(net.depot_source, net.customers[0])]
    cost = (arrive_time - net.a_lb[net.customers[0]]) ** 2
    for i in range(len(net.customers) - 1):
        n = net.customers[i]
        n_next = net.customers[i + 1]
        arrive_time += net.truck_travel_times[(n, n_next)]
        cost += (arrive_time - net.a_lb[n_next]) ** 2
        route.append(n_next)
    route.append(net.depot_sink)
    routes.append((route, cost))

    return routes


class RMP:
    def __init__(self, net):
        self.model = gp.Model("model")
        self.duals = None
        # dummy route, cost is 0
        self.routes = find_initial_routes(net)

        # add decision variables
        self.z_list = []
        obj_expr = 0
        for idx, (route, cost) in enumerate(self.routes):
            var = self.model.addVar(name=f"z_{idx}", vtype=GRB.CONTINUOUS, lb=0)
            obj_expr += var * cost * cost_scale
            self.z_list.append(var)

        # set objective
        self.model.setObjective(obj_expr, GRB.MINIMIZE)

        self.constraints = []
        # cons 1
        for i in range(len(net.customers)):
            n_name = net.customers[i]
            visit_route_ids = [i for i in range(len(self.routes)) if n_name in self.routes[i][0]]
            cons = self.model.addConstr(
                gp.quicksum(self.z_list[j] for j in visit_route_ids) == 1,
                name=f"visit_customer_{i}"
            )
            self.constraints.append(cons)
        # cons 2
        cons = self.model.addConstr(
            gp.quicksum(self.z_list) <= net.num_trucks,
            name=f"truck_fleet"
        )
        self.constraints.append(cons)

        self.model.update()
        self.model.write("RMP.lp")

    def solve(self):
        """Solve the master problem"""
        self.model.optimize()
        if self.model.Status == GRB.OPTIMAL:
            self.duals = [c.Pi for c in self.constraints]
            return self.model.ObjVal, self.duals
        else:
            raise Exception("Master problem did not converge")
