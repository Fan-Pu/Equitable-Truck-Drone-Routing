# from BranchAndPrice import *
import matplotlib.pyplot as plt

import GeneralHelper
from RMP import *
from FullModel import FullModel

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

    rmp = RMP()
    BP_solve_time = rmp.branch_and_price()
    solution, cost = rmp.construct_final_route()

    print(f"full model solved in {full_solve_time:.4f} s with obj val: {full_obj_val} and Gap: {gap:.2f}%")
    print(f"LSA solved in {BP_solve_time:.4f} s with cost: {cost}")
    test = GeneralHelper.test_sols
    sdsad = GeneralHelper.node_ids
    node_types = {}
    for idx, node in GeneralHelper.node_dict.items():
        cur_type = node.getType()
        node_types[idx] = cur_type
    sdas = 0

    # test_case = ToyTest(net)
    # test_case.solve()

    # bc_solver = BranchAndPrice(net)
    # bc_solver.solve()
