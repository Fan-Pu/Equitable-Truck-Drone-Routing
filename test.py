from queue import PriorityQueue

import gurobipy as gp
from gurobipy import GRB


class VRPBranchAndPrice:
    def __init__(self, distance_matrix, demand, vehicle_capacity, depot=0, max_iter=100, tolerance=1e-6):
        self.distance_matrix = distance_matrix
        self.demand = demand
        self.vehicle_capacity = vehicle_capacity
        self.depot = depot
        self.tolerance = tolerance
        self.num_customers = len(demand)
        self.master_problem = None
        self.routes = []  # Stores generated routes
        self.duals = None
        self.branch_queue = PriorityQueue()  # To manage subproblems
        self.best_solution = None
        self.best_cost = float('inf')

    def initialize_master(self):
        """Initialize the restricted master problem with dummy routes"""
        self.master_problem = gp.Model("MasterProblem")
        self.routes = [
            (
                [self.depot, i, self.depot],
                sum(self.distance_matrix[self.depot][i] + self.distance_matrix[i][self.depot]))
            for i in range(1, self.num_customers)
        ]
        self.variables = []
        for idx, (route, cost) in enumerate(self.routes):
            var = self.master_problem.addVar(obj=cost, name=f"route_{idx}", vtype=GRB.CONTINUOUS)
            self.variables.append(var)
        # Constraints: Each customer must be visited exactly once
        self.constraints = []
        for i in range(1, self.num_customers):
            cons = self.master_problem.addConstr(
                gp.quicksum(var for var, (route, _) in zip(self.variables, self.routes) if i in route) == 1,
                name=f"visit_customer_{i}"
            )
            self.constraints.append(cons)
        self.master_problem.update()

    def solve_master(self):
        """Solve the master problem"""
        self.master_problem.optimize()
        if self.master_problem.Status == GRB.OPTIMAL:
            self.duals = [c.Pi for c in self.constraints]
            return self.master_problem.ObjVal, self.duals
        else:
            raise Exception("Master problem did not converge")

    def solve_pricing(self, duals):
        """Solve the pricing problem (subproblem) using dual variables"""
        best_cost = float('inf')
        best_route = None
        for start in range(1, self.num_customers):
            visited = [False] * self.num_customers
            visited[self.depot] = True
            visited[start] = True
            route = [self.depot, start]
            current_cost = -duals[start - 1]  # Dual adjustment
            while True:
                # Find the next customer to visit
                min_cost, next_customer = float('inf'), -1
                for i in range(1, self.num_customers):
                    if not visited[i] and current_cost + self.distance_matrix[route[-1]][i] <= self.vehicle_capacity:
                        cost = self.distance_matrix[route[-1]][i] - duals[i - 1]
                        if cost < min_cost:
                            min_cost = cost
                            next_customer = i
                if next_customer == -1:
                    break
                visited[next_customer] = True
                route.append(next_customer)
                current_cost += min_cost
            # Return to depot
            route.append(self.depot)
            current_cost += self.distance_matrix[route[-2]][self.depot]
            if current_cost < best_cost:
                best_cost = current_cost
                best_route = route
        return best_cost, best_route

    def add_column_to_master(self, route, cost):
        """Add new column (route) to the master problem"""
        var = self.master_problem.addVar(obj=cost, name=f"route_{len(self.routes)}", vtype=GRB.CONTINUOUS)
        self.routes.append((route, cost))
        self.variables.append(var)
        for i in range(1, self.num_customers):
            if i in route:
                self.master_problem.chgCoeff(self.constraints[i - 1], var, 1)
        self.master_problem.update()

    def is_integer_solution(self):
        """Check if the current solution is integer"""
        self.master_problem.optimize()
        if self.master_problem.Status == GRB.OPTIMAL:
            for var in self.master_problem.getVars():
                if var.X > 0 and not var.X.is_integer():
                    return False, var
        return True, None

    def branch(self):
        """Branching logic to enforce integer solutions"""
        is_integer, fractional_var = self.is_integer_solution()
        if is_integer:
            solution_cost = self.master_problem.ObjVal
            if solution_cost < self.best_cost:
                self.best_cost = solution_cost
                self.best_solution = [var.X for var in self.master_problem.getVars()]
                print(f"New best solution with cost: {solution_cost}")
        else:
            # Branching on fractional variable
            fractional_var_name = fractional_var.VarName
            print(f"Branching on variable: {fractional_var_name} with value {fractional_var.X}")

            # Subproblem 1: Add constraint var <= floor(fractional_var.X)
            subproblem1 = self.master_problem.copy()
            subproblem1.addConstr(fractional_var <= int(fractional_var.X))
            self.branch_queue.put((subproblem1.ObjVal, subproblem1))

            # Subproblem 2: Add constraint var >= ceil(fractional_var.X)
            subproblem2 = self.master_problem.copy()
            subproblem2.addConstr(fractional_var >= int(fractional_var.X) + 1)
            self.branch_queue.put((subproblem2.ObjVal, subproblem2))

    def solve(self):
        """Main branch-and-price loop"""
        self.initialize_master()
        while iteration < self.max_iter:
            print(f"Iteration {iteration}: Solving Master Problem")
            obj_val, duals = self.solve_master()
            print(f"Objective value: {obj_val}")

            print("Solving Pricing Problem")
            reduced_cost, new_route = self.solve_pricing(duals)
            if reduced_cost >= -self.tolerance:
                print("No new columns with negative reduced cost. Stopping.")
                break

            print("Adding new route to Master Problem")
            self.add_column_to_master(new_route, reduced_cost)

            iteration += 1

        print("Branching Phase")
        while not self.branch_queue.empty():
            _, subproblem = self.branch_queue.get()
            self.master_problem = subproblem
            self.branch()


# Example usage
distance_matrix = [
    [0, 10, 15, 20],
    [10, 0, 35, 25],
    [15, 35, 0, 30],
    [20, 25, 30, 0]
]
demand = [0, 1, 1, 1]  # Demand for each customer
vehicle_capacity = 2
vrp_solver = VRPBranchAndPrice(distance_matrix, demand, vehicle_capacity)
vrp_solver.solve()
