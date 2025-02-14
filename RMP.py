from pyscipopt import Model, SCIP_PARAMSETTING

import GeneralHelper
from GeneralHelper import *
from MyBranchingRule import MyBranchingRule
from MyPricer import MyPricer

node_infos = {}
route_dict = {}  # key (truck_route, drone_route)
route_key_id_pairs = {}  # key: route key. value: route id
initial_route = []
constraints = []
sum_z_val = None  # branching term
z_list = []
z_keys = []  # value: key of z variable


class RMP:
    def __init__(self):
        self.model = Model("RMP")
        self.init_master_problem()

    def init_master_problem(self):
        # add initial routes
        for route in self.find_initial_routes():
            route_key = (tuple(route['truck']), tuple(route['drone']))
            route['id'] = len(route_key_id_pairs)
            route_dict[route_key] = route
            route_key_id_pairs[route_key] = route['id']
            initial_route.append(route_key)

        # Add variables
        for route_key, route in route_dict.items():
            idx, cost = route['id'], route['cost']
            var = self.model.addVar(name=f"z_{idx}", vtype="B", obj=cost)
            z_list.append(var)
            z_keys.append(route_key)

        # cons 1
        for i in range(len(GeneralHelper.net.customers)):
            n_name = GeneralHelper.net.customers[i]
            visit_route_ids = []  # the routes that visits the node
            for idx in range(len(z_keys)):
                if n_name in route_dict[z_keys[idx]]['truck']:
                    visit_route_ids.append(idx)
            cons = self.model.addCons(sum(z_list[j] for j in visit_route_ids) == 1, name=f"visit_customer_{i}",
                                      modifiable=True)
            constraints.append(cons)
        # cons 2 (truck fleet UB)
        cons = self.model.addCons(sum(z_list) <= GeneralHelper.net.num_trucks, name=f"truck_fleet", modifiable=True)
        constraints.append(cons)

    def branch_and_price(self):
        self.model.setPresolve(SCIP_PARAMSETTING.OFF)
        self.model.setSeparating(SCIP_PARAMSETTING.OFF)
        self.model.setHeuristics(SCIP_PARAMSETTING.OFF)

        pricer = MyPricer()
        self.model.includePricer(pricer, "MyPricer", "Column generation pricing", priority=5000000)

        branch_rule = MyBranchingRule()
        self.model.includeBranchrule(branch_rule, "sum_z", "branch on the sum of z",
                                     priority=10000000, maxdepth=-1, maxbounddist=1)

        # Solve the model
        self.model.writeProblem("RMP.lp")
        self.model.optimize()

        # Print the solution
        if self.model.getStatus() == "optimal":
            print("Optimal solution found!")
            for var in self.model.getVars():
                if self.model.getVal(var) > 0.5:
                    print(f"{var.name} = {self.model.getVal(var)}")
        else:
            print("No optimal solution found.")

    def find_initial_routes(self):
        routes = []
        # the route that visits all nodes
        truck_route = [GeneralHelper.net.depot_source, GeneralHelper.net.customers[0]]
        arrive_time = GeneralHelper.net.truck_travel_times[
            (GeneralHelper.net.depot_source, GeneralHelper.net.customers[0])]
        # cost = (arrive_time - GeneralHelper.net.a_lb[GeneralHelper.net.customers[0]]) ** 2
        cost = cost_scale * (arrive_time - GeneralHelper.net.a_lb[GeneralHelper.net.customers[0]])
        return_time = arrive_time  # the time it returns to depot sink
        for i in range(len(GeneralHelper.net.customers) - 1):
            n = GeneralHelper.net.customers[i]
            n_next = GeneralHelper.net.customers[i + 1]
            arrive_time += GeneralHelper.net.truck_travel_times[(n, n_next)]
            # cost += (arrive_time - GeneralHelper.net.a_lb[n_next]) ** 2
            cost += cost_scale * (arrive_time - GeneralHelper.net.a_lb[n_next])
            return_time += GeneralHelper.net.truck_travel_times[(n, n_next)]
            truck_route.append(n_next)
        return_time += GeneralHelper.net.truck_travel_times[
            (GeneralHelper.net.customers[-1], GeneralHelper.net.depot_sink)]
        cost += return_time
        truck_route.append(GeneralHelper.net.depot_sink)
        route = {'truck': truck_route, 'drone': [], 'launches': [], 'cost': cost, 'drone links': []}
        routes.append(route)

        # the route that visit only a node
        for n_name in GeneralHelper.net.customers + GeneralHelper.net.hubs:
            truck_route = [GeneralHelper.net.depot_source, n_name, GeneralHelper.net.depot_sink]
            arrive_time = GeneralHelper.net.truck_travel_times[(GeneralHelper.net.depot_source, n_name)]
            # cost = (arrive_time - GeneralHelper.net.a_lb[GeneralHelper.net.customers[0]]) ** 2
            cost = cost_scale * (arrive_time - GeneralHelper.net.a_lb[n_name])
            return_time = arrive_time + GeneralHelper.net.truck_travel_times[(n_name, GeneralHelper.net.depot_sink)]
            cost += return_time
            drone_route = []
            route = {'truck': truck_route, 'drone': drone_route, 'cost': cost, 'drone links': []}
            routes.append(route)

        return routes

    def construct_final_route(self):
        solution = []
        cost = 0
        z_vals = [self.model.getVal(var) for var in z_list]
        self.model.writeProblem("RMP.lp")
        for i, var in enumerate(z_list):
            # Check if the variable is close to 1 in the solution
            if self.model.getVal(var) + close_tolerance >= 1:
                key = z_keys[i]
                route = route_dict[key]
                solution.append(route)
                cost += route['cost']

        return solution, cost
