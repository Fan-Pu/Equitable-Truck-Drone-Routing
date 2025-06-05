# from BranchAndPrice import *
import matplotlib.pyplot as plt

import GeneralHelper
from RMP import *
from FullModel import FullModel
from BP.branch_and_price import BranchAndPrice

if __name__ == '__main__':
    # data preparation
    create_original_network()
    transform_network()
    visualize_network()

    test_case = FullModel()

    full_obj_val, full_solve_time, gap = test_case.solve()

    BP = BranchAndPrice()
    BP_solve_time, BP_solution, BP_cost = BP.solve()

    BP_costs = []
    for solution in BP_solution:
        cost = route_get_cost(solution, GeneralHelper.net, GeneralHelper.transformed_net)
        BP_costs.append(cost)
    sum_BP_cost = sum(BP_costs)

    if gap is not None:
        print(f"full model solved in {full_solve_time:.4f} s with obj val: {full_obj_val} and Gap: {gap:.2f}%")
    print(f"LSA solved in {BP_solve_time:.4f} s with cost: {BP_cost}")
    for item in GeneralHelper.node_lp_trace:
        print(item)
    sdas = 0

    # test_case = ToyTest(net)
    # test_case.solve()

    # bc_solver = BranchAndPrice(net)
    # bc_solver.solve()
