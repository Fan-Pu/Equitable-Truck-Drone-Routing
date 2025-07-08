import os
from pathlib import Path

import CommonHelper
from BP.branch_and_price import BranchAndPrice
from FullModel import FullModel

os.environ["OMP_NUM_THREADS"] = "1"

if __name__ == '__main__':
    root_dir = "results/"
    ins_dir = root_dir + (f"{CommonHelper.num_trucks}-{CommonHelper.num_customers}-"
                          f"{CommonHelper.custom_dist}-{CommonHelper.seed}/")
    Path(ins_dir).mkdir(parents=True, exist_ok=True)
    param_file = ins_dir + f"param-seed={CommonHelper.seed}.json"
    BP_result_file = ins_dir + f"BP-seed={CommonHelper.seed}.json"

    # data preparation
    CommonHelper.create_original_network()
    CommonHelper.transform_network()
    CommonHelper.num_of_arcs = len(CommonHelper.transformed_net.arcs)
    # CommonHelper.visualize_network()

    # CommonHelper.visualize_solution()
    # CommonHelper.plot_set_style()
    # CommonHelper.plot_solution_waiting_times(CommonHelper.net, CommonHelper.transformed_net)

    # CommonHelper.read_sol_get_delay_times()

    # the integrated model
    full_model = FullModel()
    full_obj_val, full_solve_time, gap, full_sol = full_model.solve(time_limit=1, use_cb=False,
                                                                    log_file=ins_dir + "full_solve.log")
    CommonHelper.solver_sol = full_sol
    full_model.model.dispose()

    # branch and price
    bp = BranchAndPrice()
    # get the warm start solution
    if CommonHelper.enable_warm_start:
        full_model = FullModel()
        warm_obj_val, _, _, warm_start_sols = full_model.solve(use_cb=True)
        full_model.model.dispose()
        print(f"\nBP warm start: {warm_obj_val:.4f}")
        bp.add_warm_start_solution(warm_start_sols)
    BP_solve_time, BP_solution, BP_cost = bp.solve()
    CommonHelper.BP_runtime = BP_solve_time
    CommonHelper.BP_sol = BP_solution

    if gap is not None:
        text = f"full model solved in {full_solve_time:.4f} s with obj val: {full_obj_val} and Gap: {gap:.2f}%"
        print(text)
        CommonHelper.sol_summary.append(text)
    text = f"LSA solved in {BP_solve_time:.4f} s with cost: {BP_cost}"
    print(text)

    CommonHelper.get_SA_infos(CommonHelper.BP_sol, CommonHelper.transformed_net)

    CommonHelper.sol_summary.append(text)
    CommonHelper.export_class_attributes(CommonHelper, param_file)
    CommonHelper.save_complex_dict(bp.node_solutions, BP_result_file)
    sdas = 0
