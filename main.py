# from BranchAndPrice import *
import matplotlib.pyplot as plt
from RMP import *
from FullModel import FullModel

if __name__ == '__main__':
    # data preparation
    create_random_truck_drone_network()
    transform_network()

    # test_case = FullModel()
    # test_case.visualize()
    # # plt.show()
    # test_case.solve()
    # test_case.visualize_routes()
    # plt.show()

    rmp = RMP()
    rmp.branch_and_price()
    solution, cost = rmp.construct_final_route()
    sdas = 0

    # test_case = ToyTest(net)
    # test_case.solve()

    # bc_solver = BranchAndPrice(net)
    # bc_solver.solve()
