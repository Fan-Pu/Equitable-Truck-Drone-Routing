# from BranchAndPrice import *
from RMP import *

if __name__ == '__main__':
    # test_case = ToyTest(8, 2, 3, 2)
    # test_case.visualize()
    # # plt.show()
    # test_case.solve()
    # test_case.visualize_routes()
    # plt.show()
    # ss = 0

    # test_cuttingstock()

    create_random_truck_drone_network()
    transform_network()

    rmp = RMP()
    rmp.branch_and_price()
    solution, cost = rmp.construct_final_route()
    sdas = 0

    # test_case = ToyTest(net)
    # test_case.solve()

    # bc_solver = BranchAndPrice(net)
    # bc_solver.solve()
