# from BranchAndPrice import *
import matplotlib.pyplot as plt

import GeneralHelper
from RMP import *
from FullModel import FullModel
from BP.branch_and_price import BranchAndPrice

if __name__ == '__main__':
    # data preparation
    create_random_truck_drone_network()
    transform_network()

    test_case = FullModel()
    test_case.visualize()

    # plt.show()
    full_obj_val, full_solve_time, gap = test_case.solve()

    # test_case.visualize_routes()
    # plt.show()

    # rmp = RMP()
    # BP_solve_time = rmp.BP()
    # solution, cost = rmp.construct_final_route()

    BP = BranchAndPrice()
    BP_solve_time, BP_solution, BP_cost = BP.solve()

    print(f"full model solved in {full_solve_time:.4f} s with obj val: {full_obj_val} and Gap: {gap:.2f}%")
    print(f"LSA solved in {BP_solve_time:.4f} s with cost: {BP_cost}")
    sdas = 0

    # test_case = ToyTest(net)
    # test_case.solve()

    # bc_solver = BranchAndPrice(net)
    # bc_solver.solve()
