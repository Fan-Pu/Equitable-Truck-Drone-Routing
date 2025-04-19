import GeneralHelper
from GeneralHelper import *
import gurobipy as gp
from gurobipy import GRB
from NodeInfo import NodeInfo

node_infos = {}
route_dict = {}  # key (truck_route, drone_route)
route_key_id_pairs = {}  # key: route key. value: route id
initial_route = []
constraints = []
sum_z_val = None  # branching term
z_list = []
z_keys = []  # value: key of z variable
infeasible_nodes = []


class RMP:
    def __init__(self, node_info: NodeInfo):
        self.model = gp.Model("RMP")
        # add decision variables
        for route_key in node_info.columns:
            route = route_dict[route_key]

    def init_master_problem(self):
        # add initial routes
        for route_key, route in find_initial_routes(GeneralHelper.net):
            route_dict[route_key] = route
            route_key_id_pairs[route_key] = route['id']
            initial_route.append(route_key)

        # add variables
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

        # self.model.setParam("lp/solvefreq", 1)  # Solve LP at every iteration
        # self.model.setParam("lp/iterlim", 10000)  # Allow sufficient LP iterations
        # self.model.setParam("limits/nodes", 10000)  # Ensure enough nodes are processed

        pricer = MyPricer()
        self.model.includePricer(pricer, "MyPricer", "Column generation pricing", priority=5000000)

        branch_rule = MyBranchingRule()
        self.model.includeBranchrule(branch_rule, "sum_z", "branch on the sum of z",
                                     priority=10000000, maxdepth=-1, maxbounddist=1)

        # Solve the model
        self.model.writeProblem("RMP.lp")
        self.model.setParam("display/verblevel", 5)  # Highest verbosity level
        self.model.optimize()

        # Print the solution
        if self.model.getStatus() == "optimal":
            print("Optimal solution found!")
            for i in range(len(z_list)):
                var = z_list[i]
                if self.model.getVal(var) > 0.5:
                    print(f"{var.name} = {self.model.getVal(var)}")
        else:
            print("No optimal solution found.")

        return self.model.getSolvingTime()

    def construct_final_route(self):
        solution = []
        cost = 0
        self.model.writeProblem("RMP.lp")
        for i, var in enumerate(z_list):
            # Check if the variable is close to 1 in the solution
            if self.model.getVal(var) + close_tolerance >= 1:
                key = z_keys[i]
                route = route_dict[key]
                solution.append(route)
                cost += route['cost']

        return solution, cost
