from toy_test import *

if __name__ == '__main__':
    test_case = ToyTest(5, 2, 1, 1)
    test_case.solve()
    test_case.visualize()
    test_case.visualize_routes()
    plt.show()
    ss = 0
