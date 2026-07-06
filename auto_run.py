from __future__ import annotations

import itertools
import subprocess


seed_dict = {
    "PS": [1, 2, 3],
    "PC": [1, 2, 3],
    "mixed": [1, 2, 3],
}
net_profiles = [(2, 5)]
custom_dists = ["PS", "PC", "mixed"]

for (num_trucks, num_customers), distribution in itertools.product(net_profiles, custom_dists):
    for seed in seed_dict[distribution]:
        subprocess.run(
            [
                "python",
                "-m",
                "thvrpd.solve",
                "--seed",
                str(seed),
                "--num-trucks",
                str(num_trucks),
                "--num-customers",
                str(num_customers),
                "--distribution",
                distribution,
                "--drones-per-truck",
                "3",
                "--weights",
                "0.4",
                "0.3",
                "0.3",
                "--threads",
                "1",
                "--time-limit",
                "1800",
                "--output-dir",
                "runs",
            ],
            check=True,
        )
