from __future__ import annotations

import argparse
import csv
import json
import sys
from dataclasses import asdict
from pathlib import Path
import time

from .config import InstanceConfig, ObjectiveWeights, SolverConfig, case_defaults
from .instance import generate_instance, instance_generation_metadata
from .metrics import solution_service_metrics


SCALES = {
    "small": {"num_customers": 5, "num_trucks": 2, "num_hubs": 2, "drones_per_truck": 4},
    "medium": {"num_customers": 10, "num_trucks": 3, "num_hubs": 2, "drones_per_truck": 4},
    "large": {"num_customers": 30, "num_trucks": 6, "num_hubs": 3, "drones_per_truck": 4},
}


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--seeds", nargs="+", type=int, default=[1, 2, 3])
    parser.add_argument("--distributions", nargs="+", choices=["PS", "PC", "mixed"], default=["PS"])
    parser.add_argument("--scales", nargs="+", choices=sorted(SCALES), default=["small", "medium", "large"])
    parser.add_argument("--weights", nargs=3, type=float, default=[0.4, 0.3, 0.3])
    parser.add_argument("--threads", type=int, default=0)
    parser.add_argument("--case-time-limit", type=float, default=1800.0)
    parser.add_argument("--pricing-tolerance", type=float)
    parser.add_argument("--pricing-parallel-workers", type=int)
    parser.add_argument("--pricing-worker-backend", choices=["thread", "process"])
    parser.add_argument("--pricing-workload-customer-weight", type=float, default=1.0)
    parser.add_argument("--pricing-workload-out-degree-weight", type=float, default=0.25)
    parser.add_argument("--pricing-workload-drone-pad-weight", type=float, default=0.5)
    parser.add_argument("--pricing-workload-deadline-weight", type=float, default=0.5)
    parser.add_argument("--pricing-split-open-labels-min", type=int, default=500)
    parser.add_argument("--pricing-split-gap-factor", type=float, default=1.0)
    parser.add_argument("--pricing-split-elapsed-min", type=float, default=5.0)
    parser.add_argument("--pricing-split-work-min", type=int, default=2000)
    parser.add_argument("--pricing-dynamic-refinement-depth", type=int, default=2)
    parser.add_argument("--pricing-checkpoint-extension-period", type=int, default=1000)
    parser.add_argument("--pricing-batch-size", type=int, default=64)
    parser.add_argument("--farkas-batch-size", type=int, default=16)
    parser.add_argument("--service-deadline-mode", choices=["none", "random_absolute"], default=None)
    parser.add_argument("--service-deadline-offset-min", type=float)
    parser.add_argument("--service-deadline-offset-max", type=float)
    parser.add_argument("--truck-speed", type=float, default=40.0)
    parser.add_argument("--drone-speed", type=float, default=40.0)
    parser.add_argument("--truck-payload", type=float, default=50.0)
    parser.add_argument("--drone-payload", type=float, default=2.3)
    parser.add_argument("--drone-endurance", type=float, default=30.0)
    parser.add_argument("--truck-cost", type=float, default=20.0)
    parser.add_argument("--drone-cost", type=float, default=6.0)
    parser.add_argument("--disable-root-compact-warm-start", action="store_true")
    parser.add_argument("--root-compact-solve-time-limit", type=float, default=60.0)
    parser.add_argument("--logging-mode", choices=["audit", "light"], default="audit")
    parser.add_argument("--gurobi-python-path", type=Path)
    return parser


def main() -> None:
    args = _parser().parse_args()
    if args.gurobi_python_path is not None:
        sys.path.append(str(args.gurobi_python_path))
    from .bpc import BPCTimeLimitNoIncumbent, solve_branch_price_cut

    args.output_dir.mkdir(parents=True, exist_ok=False)
    weights = ObjectiveWeights(*args.weights)
    summary: list[dict] = []
    for scale in args.scales:
        dimensions = SCALES[scale]
        defaults = case_defaults(**dimensions)
        for distribution in args.distributions:
            for seed in args.seeds:
                instance_config = InstanceConfig(
                    seed=seed,
                    distribution=distribution,
                    **dimensions,
                    truck_arc_probability=float(defaults["truck_arc_probability"]),
                    hub_arc_probability=float(defaults["hub_arc_probability"]),
                    mandatory_drone_customer_fraction=float(defaults["mandatory_drone_customer_fraction"]),
                    truck_speed=args.truck_speed,
                    drone_speed=args.drone_speed,
                    truck_payload=args.truck_payload,
                    drone_payload=args.drone_payload,
                    drone_endurance=args.drone_endurance,
                    truck_cost=args.truck_cost,
                    drone_cost=args.drone_cost,
                    service_deadline_mode=args.service_deadline_mode or str(defaults["service_deadline_mode"]),
                    service_deadline_offset_min=(
                        args.service_deadline_offset_min
                        if args.service_deadline_offset_min is not None
                        else float(defaults["service_deadline_offset_min"])
                    ),
                    service_deadline_offset_max=(
                        args.service_deadline_offset_max
                        if args.service_deadline_offset_max is not None
                        else float(defaults["service_deadline_offset_max"])
                    ),
                )
                case_dir = args.output_dir / f"{scale}_{distribution}_seed{seed}"
                case_dir.mkdir()
                solver_config = SolverConfig(
                    threads=args.threads,
                    time_limit=args.case_time_limit,
                    pricing_tolerance=(
                        args.pricing_tolerance
                        if args.pricing_tolerance is not None
                        else float(defaults["pricing_tolerance"])
                    ),
                    pricing_batch_size=args.pricing_batch_size,
                    farkas_batch_size=args.farkas_batch_size,
                    enable_root_compact_warm_start=not args.disable_root_compact_warm_start,
                    root_compact_solve_time_limit=args.root_compact_solve_time_limit,
                    pricing_parallel_workers=(
                        args.pricing_parallel_workers
                        if args.pricing_parallel_workers is not None
                        else int(defaults["pricing_parallel_workers"])
                    ),
                    pricing_worker_backend=args.pricing_worker_backend or str(defaults["pricing_worker_backend"]),
                    pricing_workload_customer_weight=args.pricing_workload_customer_weight,
                    pricing_workload_out_degree_weight=args.pricing_workload_out_degree_weight,
                    pricing_workload_drone_pad_weight=args.pricing_workload_drone_pad_weight,
                    pricing_workload_deadline_weight=args.pricing_workload_deadline_weight,
                    pricing_split_open_labels_min=args.pricing_split_open_labels_min,
                    pricing_split_gap_factor=args.pricing_split_gap_factor,
                    pricing_split_elapsed_min=args.pricing_split_elapsed_min,
                    pricing_split_work_min=args.pricing_split_work_min,
                    pricing_dynamic_refinement_depth=args.pricing_dynamic_refinement_depth,
                    pricing_checkpoint_extension_period=args.pricing_checkpoint_extension_period,
                    logging_mode=args.logging_mode,
                    gurobi_log_dir=str(case_dir / "gurobi_logs"),
                )
                instance = generate_instance(instance_config)
                start = time.time()
                try:
                    result = solve_branch_price_cut(instance, weights, solver_config)
                    record = {
                        "scale": scale,
                        "instance_config": asdict(instance_config),
                        "solver_config": asdict(solver_config),
                        **instance_generation_metadata(instance),
                        **result.to_record(),
                        "service_metrics": solution_service_metrics(instance, result),
                    }
                except BPCTimeLimitNoIncumbent as exc:
                    record = {
                        "scale": scale,
                        "status": "time_limit_no_incumbent",
                        "runtime": exc.runtime,
                        "nodes_processed": exc.nodes_processed,
                        "lower_bound_shifted": exc.lower_bound_shifted,
                        "lower_bound_full": exc.objective.full_value_from_route_sum(exc.lower_bound_shifted),
                        "instance_config": asdict(instance_config),
                        "solver_config": asdict(solver_config),
                        **instance_generation_metadata(instance),
                        "bpc_stats": asdict(exc.stats),
                        "wall_runtime": time.time() - start,
                    }
                output = case_dir / "result.json"
                output.write_text(json.dumps(record, indent=2), encoding="utf-8")
                summary.append({
                    "scale": scale,
                    "distribution": distribution,
                    "seed": seed,
                    "status": record["status"],
                    "objective_full": record.get("objective_full"),
                    "lower_bound_full": record.get("lower_bound_full"),
                    "gap_full": record.get("gap_full"),
                    "runtime": record.get("runtime"),
                    "nodes_processed": record.get("nodes_processed"),
                    "result_file": str(output),
                })
    (args.output_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    with (args.output_dir / "summary.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(summary[0]))
        writer.writeheader()
        writer.writerows(summary)


if __name__ == "__main__":
    main()
