from PSP import PSP
from RMP import RMP


class BranchAndPrice:
    def __init__(self, net, max_iter=100, tolerance=1e-6):
        self.net = net
        self.master_problem = RMP(net)
        self.pricing_problem = PSP(net)
        self.max_iter = max_iter
        self.tolerance = tolerance

    def solve(self):
        """Main branch-and-price loop"""
        iteration = 0
        while iteration < self.max_iter:
            print(f"Iteration {iteration}: Solving Master Problem")
            obj_val, duals = self.master_problem.solve()
            print(f"Objective value: {obj_val}")

            print("Solving Pricing Problem")
            reduced_cost, new_route = self.pricing_problem.solve(self.net, duals)
            if reduced_cost >= -self.tolerance:
                print("No new columns with negative reduced cost. Stopping.")
                break

            # print("Adding new route to Master Problem")
            # self.add_column_to_master(new_route, reduced_cost)

            iteration += 1

        print("Branching Phase")
        # self.branch()
