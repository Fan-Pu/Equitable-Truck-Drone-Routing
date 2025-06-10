import GeneralHelper
from FullModel import FullModel
from BP.branch_and_price import BranchAndPrice

if __name__ == '__main__':
    # data preparation
    GeneralHelper.create_original_network()
    GeneralHelper.transform_network()
    # GeneralHelper.visualize_network()

    # the integrated model
    full_model = FullModel()
    full_obj_val, full_solve_time, gap, _ = full_model.solve(time_limit=5)

    # branch and price
    BP = BranchAndPrice()
    # get the warm start solution
    full_model.reset()
    warm_obj_val, _, _, warm_start_sols = full_model.solve(time_limit=60)
    print(f"\nBP warm start: {warm_obj_val:.4f}")
    BP.add_warm_start_solution(warm_start_sols)
    BP_solve_time, BP_solution, BP_cost = BP.solve()

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
