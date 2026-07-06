from __future__ import annotations

import argparse

from .config import InstanceConfig, ObjectiveWeights, SolverConfig
from .instance import generate_instance, tiny_instance


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", choices=["tiny", "small", "medium"], required=True)
    args = parser.parse_args()
    weights = ObjectiveWeights(0.4, 0.3, 0.3)
    instances = []
    if args.profile == "tiny":
        instances = [tiny_instance()]
    elif args.profile == "small":
        instances = [
            generate_instance(InstanceConfig(seed=s, num_trucks=2, num_customers=4, distribution=d, drones_per_truck=2))
            for d in ("PS", "PC", "mixed")
            for s in (1, 2)
        ]
    else:
        instances = [
            generate_instance(InstanceConfig(seed=s, num_trucks=3, num_customers=6, distribution=d, drones_per_truck=2))
            for d in ("PS", "PC", "mixed")
            for s in (1, 2, 3)
        ]
    from .bpc import solve_branch_price_cut
    from .compact import solve_compact_miqp

    for instance in instances:
        bpc = solve_branch_price_cut(instance, weights, SolverConfig(time_limit=1800.0))
        compact = solve_compact_miqp(instance, weights, time_limit=1800.0)
        if abs(bpc.objective_full - compact) > 1e-5:
            raise AssertionError((bpc.objective_full, compact, instance.config))
        print(f"validated seed={instance.config.seed} dist={instance.config.distribution} obj={bpc.objective_full:.8f}")


if __name__ == "__main__":
    main()
