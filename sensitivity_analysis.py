from __future__ import annotations

import subprocess


profiles = [
    ("PS", [3, 4, 5, 7]),
    ("PC", [1, 6, 7, 8]),
    ("mixed", [1, 2, 4, 5]),
]
alphas = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 0.99]

for distribution, seeds in profiles:
    for alpha in alphas:
        delay = alpha
        remaining = 1.0 - alpha
        for seed in seeds:
            subprocess.run(
                [
                    "python",
                    "-m",
                    "thvrpd.solve",
                    "--seed",
                    str(seed),
                    "--num-trucks",
                    "5",
                    "--num-customers",
                    "15",
                    "--distribution",
                    distribution,
                    "--drones-per-truck",
                    "3",
                    "--weights",
                    str(delay),
                    str(remaining / 2.0),
                    str(remaining / 2.0),
                    "--threads",
                    "1",
                    "--time-limit",
                    "1800",
                    "--output-dir",
                    f"runs/{distribution}/alpha={alpha}",
                ],
                check=True,
            )
