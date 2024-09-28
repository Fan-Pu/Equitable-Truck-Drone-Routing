from toy_test import *

if __name__ == '__main__':
    test_case = ToyTest(3, 1, 0, 2)
    test_case.visualize()
    # plt.show()
    test_case.solve()
    test_case.visualize_routes()
    plt.show()
    ss = 0
