import argparse
import json
import os
from pathlib import Path

import CommonHelper
from BP.branch_and_price import BranchAndPrice
from FullModel import FullModel

os.environ["OMP_NUM_THREADS"] = "1"


def main(seed, num_trucks, num_customers, custom_dist, drone_num, cw, enable_sens):
    CommonHelper.seed = seed
    CommonHelper.num_trucks = num_trucks
    CommonHelper.num_customers = num_customers
    CommonHelper.custom_dist = custom_dist
    CommonHelper.num_drones_per_truck = drone_num
    CommonHelper.cw = cw
    root_dir = "results/"
    if enable_sens == "True":
        ins_dir = root_dir + (f"{CommonHelper.num_trucks}-{CommonHelper.num_customers}-"
                              f"{CommonHelper.custom_dist}-{CommonHelper.seed}-{CommonHelper.cw}/")
    else:
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
    if drone_num > 0:
        full_model = FullModel()
        full_obj_val, full_solve_time, gap, full_sol = full_model.solve(time_limit=1800, use_cb=False,
                                                                        log_file=ins_dir + "full_solve.log")
        CommonHelper.solver_sol = full_sol
        full_model.model.dispose()
        if gap is not None:
            text = f"full model solved in {full_solve_time:.4f} s with obj val: {full_obj_val} and Gap: {gap:.2f}%"
            print(text)
            CommonHelper.sol_summary.append(text)

    # branch and price
    bp = BranchAndPrice()
    # get the warm start solution
    if CommonHelper.enable_warm_start and drone_num > 0:
        full_model = FullModel()
        warm_obj_val, _, _, warm_start_sols = full_model.solve(use_cb=True)
        full_model.model.dispose()
        text = f"\nBP warm start: {warm_obj_val:.4f}"
        print(text)
        bp.add_warm_start_solution(warm_start_sols)
        CommonHelper.sol_summary.append(text)
    BP_solve_time, BP_solution, BP_cost = bp.solve()
    CommonHelper.BP_runtime = BP_solve_time
    CommonHelper.BP_sol = BP_solution
    _BP_sol_info = CommonHelper.get_SA_infos(CommonHelper.BP_sol, CommonHelper.transformed_net)

    text = f"LSA solved in {BP_solve_time:.4f} s with cost: {BP_cost}"
    print(text)

    CommonHelper.sol_summary.append(text)
    CommonHelper.export_class_attributes(CommonHelper, param_file)
    CommonHelper.save_complex_dict(bp.node_solutions, BP_result_file)

    with open("BP_sol_info.json", "w") as f:
        json.dump(_BP_sol_info, f)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--num_trucks", type=int, required=True)
    parser.add_argument("--num_customers", type=int, required=True)
    parser.add_argument("--custom_dist", type=str, required=True)
    parser.add_argument("--drone_num", type=int, required=True)
    parser.add_argument("--cw", type=float, required=True)
    parser.add_argument("--enable_sens", type=str, required=True)
    args = parser.parse_args()
    main(args.seed, args.num_trucks, args.num_customers, args.custom_dist, args.drone_num, args.cw, args.enable_sens)
