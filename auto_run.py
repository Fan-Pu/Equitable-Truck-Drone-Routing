import itertools
import subprocess

import CommonHelper

seed_dict = {
    "PS": [1, 2, 3],
    "PC": [1, 3, 4],
    "mixed": [1, 2, 3]
}
# net_profiles = [(5, 15), (8, 25)]
net_profiles = [(8, 25)]
# custom_dists = ["PS", "PC", "mixed"]
custom_dists = ["PS"]

# loop over every combination
for net_profile, custom_dist in itertools.product(net_profiles, custom_dists):
    (num_trucks, num_customers) = net_profile
    for seed in seed_dict[custom_dist]:
        seed = 3
        subprocess.run([
            "python", "main.py",
            "--seed", str(seed),
            "--num_trucks", str(num_trucks),
            "--num_customers", str(num_customers),
            "--custom_dist", str(custom_dist),
            "--drone_num", str(CommonHelper.num_drones_per_truck),
            "--cw", str(0.25),
            "--enable_sens", str(False)
        ], check=True)
        dsa = 0
