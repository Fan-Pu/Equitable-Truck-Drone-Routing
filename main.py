from __future__ import annotations

import argparse
import sys

from thvrpd.solve import main as package_main


def main() -> None:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--seed")
    parser.add_argument("--num_trucks")
    parser.add_argument("--num_customers")
    parser.add_argument("--custom_dist")
    parser.add_argument("--drone_num")
    parser.add_argument("--cw")
    parser.add_argument("--enable_sens")
    known, _ = parser.parse_known_args()
    if known.num_trucks is not None:
        delay_weight = float(known.cw) / (1.0 + float(known.cw))
        remaining = 1.0 - delay_weight
        sys.argv = [
            sys.argv[0],
            "--seed",
            known.seed,
            "--num-trucks",
            known.num_trucks,
            "--num-customers",
            known.num_customers,
            "--distribution",
            known.custom_dist,
            "--drones-per-truck",
            known.drone_num,
            "--weights",
            str(delay_weight),
            str(remaining / 2.0),
            str(remaining / 2.0),
            "--output-dir",
            "runs",
        ]
    package_main()


if __name__ == "__main__":
    main()
