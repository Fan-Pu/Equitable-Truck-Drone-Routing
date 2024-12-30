from BranchAndPrice import *
from GeneralHelper import *

if __name__ == '__main__':
    # test_case = ToyTest(8, 2, 3, 2)
    # test_case.visualize()
    # # plt.show()
    # test_case.solve()
    # test_case.visualize_routes()
    # plt.show()
    # ss = 0
    net = create_random_truck_drone_network()
    bc_solver = BranchAndPrice(net)
    bc_solver.solve()
