import json
import subprocess
from collections import defaultdict
from pathlib import Path

import numpy as np

import CommonHelper

net_profile = (5, 15)
# custom_dists = ["PS", "PC", "mixed"]
custom_dists = ["mixed"]
seed_dict = {
    "PS": [3, 4, 5, 7],
    "PC": [1, 6, 7, 8],
    "mixed": [1, 2, 4, 5]
}

# seed_dict = {
#     "PC": [1, 6, 7, 8],
#     "mixed": [1, 2, 4, 5]
# }

# custom_dists = ["PS", "PC"]
# seed_dict = {
#     "PS": [3, 4],
#     "PC": [1, 3],
#     "mixed": [1, 2]
# }

alpha_values = np.arange(0.1, 1.1, 0.1)
# alpha_values = np.arange(0.1, 1.1, 0.5)

result_dict = defaultdict()

root_dir = "sens_results/"


def flatten_key(key):
    parts = []
    for part in key:
        if isinstance(part, tuple):
            # unpack the inner tuple
            parts.extend(part)
        else:
            # convert numpy float to Python float or str
            parts.append(float(part) if hasattr(part, "item") else part)
    # join all parts with hyphens
    return "-".join(str(x) for x in parts)


# loop over every combination
for custom_dist in custom_dists:
    (num_trucks, num_customers) = net_profile
    seeds = seed_dict[custom_dist]

    ins_dir = root_dir + f"{custom_dist}/"
    Path(ins_dir).mkdir(parents=True, exist_ok=True)

    for alpha in alpha_values:
        # for stability
        if abs(alpha - 1) <= 0.01:
            alpha = 0.99
        alpha_dir = ins_dir + f"{alpha}/"
        Path(alpha_dir).mkdir(parents=True, exist_ok=True)
        cw = (1 - alpha) / alpha

        ins_num_trucks = []
        ins_truck_customer_visit_nums = []
        ins_drone_flight_nums = []
        ins_drone_ratios = []
        ins_f_times = []
        ins_f_costs = []
        ins_delay_times = []

        for seed in seeds:
            subprocess.run([
                "python", "main.py",
                "--seed", str(seed),
                "--num_trucks", str(num_trucks),
                "--num_customers", str(num_customers),
                "--custom_dist", str(custom_dist),
                "--drone_num", str(CommonHelper.num_drones_per_truck),
                "--cw", str(cw),
                "--enable_sens", str(True),
            ], check=True)

            # the solution is saved in this file
            with open("BP_sol_info.json", "r") as f:
                bp_sol_info = json.load(f)
                num_truck_dispatched, truck_customer_visit_num, drone_flight_num, drone_ratio, \
                    f_time, f_cost, delay_times = bp_sol_info
                ins_num_trucks.append(num_truck_dispatched)
                ins_truck_customer_visit_nums.append(truck_customer_visit_num)
                ins_drone_flight_nums.append(drone_flight_num)
                ins_drone_ratios.append(drone_ratio)
                ins_f_times.append(f_time)
                ins_f_costs.append(f_cost)
                ins_delay_times.extend(list(delay_times.values()))
                key = flatten_key((custom_dist, alpha, seed))
                result_dict[key] = bp_sol_info

        with open(f"{alpha_dir}num_trucks.txt", "w") as f:
            f.writelines(f"{item}\n" for item in ins_num_trucks)
        with open(f"{alpha_dir}truck_customer_visit_nums.txt", "w") as f:
            f.writelines(f"{item}\n" for item in ins_truck_customer_visit_nums)
        with open(f"{alpha_dir}drone_flight_nums.txt", "w") as f:
            f.writelines(f"{item}\n" for item in ins_drone_flight_nums)
        with open(f"{alpha_dir}drone_ratios.txt", "w") as f:
            f.writelines(f"{item}\n" for item in ins_drone_ratios)
        with open(f"{alpha_dir}f_times.txt", "w") as f:
            f.writelines(f"{item}\n" for item in ins_f_times)
        with open(f"{alpha_dir}f_costs.txt", "w") as f:
            f.writelines(f"{item}\n" for item in ins_f_costs)
        with open(f"{alpha_dir}delay_times.txt", "w") as f:
            f.writelines(f"{item}\n" for item in ins_delay_times)

with open(f"{root_dir}summary.json", "w") as f:
    json.dump(result_dict, f, indent=2)
