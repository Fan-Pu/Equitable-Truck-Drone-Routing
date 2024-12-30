import gurobipy as gp
import numpy as np
from gurobipy import GRB

# Define data for the VRP
n_customers = 5
vehicle_capacity = 10
demand = [0, 2, 3, 4, 5]  # Including the depot as node 0
distance_matrix = np.array([
    [0, 10, 15, 20, 25],
    [10, 0, 35, 25, 30],
    [15, 35, 0, 30, 20],
    [20, 25, 30, 0, 15],
    [25, 30, 20, 15, 0]
])


# Step 1: Define the Master Problem (Set Partitioning)
def create_master_model(columns):
    model = gp.Model("MasterProblem")
    lambda_vars = model.addVars(len(columns), vtype=GRB.CONTINUOUS, name="lambda")
    model.setObjective(gp.quicksum(lambda_vars[i] for i in range(len(columns))), GRB.MINIMIZE)

    # One route per customer (except depot)
    for j in range(1, n_customers + 1):
        model.addConstr(gp.quicksum(lambda_vars[i] * (j in columns[i]) for i in range(len(columns))) == 1)

    return model, lambda_vars


# Step 2: Solve the Pricing Problem (Shortest Path with Capacity Constraints)
def solve_pricing_problem(master_dual, demand, vehicle_capacity, distance_matrix):
    n = len(demand)
    model = gp.Model("PricingProblem")
    x = model.addVars(n, n, vtype=GRB.BINARY, name="x")
    u = model.addVars(n, vtype=GRB.CONTINUOUS, name="u")

    # Objective: Reduced cost
    obj = gp.quicksum(distance_matrix[i, j] * x[i, j] for i in range(n) for j in range(n) if i != j)
    obj -= gp.quicksum(master_dual[j - 1] * gp.quicksum(x[i, j] for i in range(n) if i != j) for j in range(1, n))
    model.setObjective(obj, GRB.MINIMIZE)

    # Constraints
    for i in range(n):
        model.addConstr(gp.quicksum(x[i, j] for j in range(n) if i != j) <= 1)
        model.addConstr(gp.quicksum(x[j, i] for j in range(n) if i != j) <= 1)

    model.addConstrs(u[i] - u[j] + vehicle_capacity * x[i, j] <= vehicle_capacity - demand[j]
                     for i in range(1, n) for j in range(1, n) if i != j)

    model.addConstr(u[0] == 0)

    model.optimize()

    # Extract new route if solution found
    if model.objVal < -1e-6:  # If reduced cost is negative
        new_route = []
        for i in range(n):
            for j in range(n):
                if x[i, j].x > 0.5:
                    new_route.append((i, j))
        return new_route
    return None


# Step 3: Branch-and-Price
def branch_and_price(demand, vehicle_capacity, distance_matrix):
    columns = [[0, i, 0] for i in range(1, n_customers + 1)]  # Initial routes (one customer per route)
    master_model, lambda_vars = create_master_model(columns)

    while True:
        master_model.optimize()
        if master_model.status != GRB.OPTIMAL:
            break

        master_dual = [c.Pi for c in master_model.getConstrs()]
        new_route = solve_pricing_problem(master_dual, demand, vehicle_capacity, distance_matrix)

        if new_route is None:  # No new column found
            break

        columns.append(new_route)
        master_model, lambda_vars = create_master_model(columns)

    return master_model, columns


# Solve the VRP
final_model, routes = branch_and_price(demand, vehicle_capacity, distance_matrix)

# Output results
if final_model.status == GRB.OPTIMAL:
    print("Optimal cost:", final_model.objVal)
    print("Routes:")
    for route in routes:
        print(route)
