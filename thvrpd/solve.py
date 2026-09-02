from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict
from pathlib import Path
import time

from .config import InstanceConfig, ObjectiveWeights, SolverConfig, case_defaults
from .instance import (
    generate_instance,
    instance_generation_metadata,
    read_instance_snapshot,
    validate_instance_case,
)
from .metrics import solution_service_metrics
from .service_windows import load_manual_service_deadline_bounds


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--num-trucks", type=int, required=True)
    parser.add_argument("--num-customers", type=int, required=True)
    parser.add_argument("--num-hubs", type=int, default=2)
    parser.add_argument("--distribution", choices=["PS", "PC", "mixed"], required=True)
    parser.add_argument("--drones-per-truck", type=int, required=True)
    parser.add_argument("--truck-arc-probability", type=float)
    parser.add_argument("--hub-arc-probability", type=float)
    parser.add_argument("--truck-speed", type=float, default=40.0)
    parser.add_argument("--drone-speed", type=float, default=40.0)
    parser.add_argument("--truck-payload", type=float, default=50.0)
    parser.add_argument("--drone-payload", type=float, default=2.3)
    parser.add_argument("--drone-endurance", type=float, default=30.0)
    parser.add_argument("--truck-cost", type=float, default=20.0)
    parser.add_argument("--drone-cost", type=float, default=6.0)
    parser.add_argument("--mandatory-drone-customer-fraction", type=float)
    parser.add_argument("--service-deadline-mode", choices=["none", "manual", "random_absolute"])
    parser.add_argument("--service-deadline-file", type=Path)
    parser.add_argument("--service-deadline-offset-min", type=float)
    parser.add_argument("--service-deadline-offset-max", type=float)
    parser.add_argument("--service-deadline-random-seed", type=int)
    parser.add_argument("--weights", nargs=3, type=float, metavar=("DELAY", "RETURN", "COST"), required=True)
    parser.add_argument("--threads", type=int, default=0)
    parser.add_argument("--time-limit", type=float, default=1800.0)
    parser.add_argument("--pricing-tolerance", type=float)
    parser.add_argument("--pricing-batch-size", type=int, default=64)
    parser.add_argument("--farkas-batch-size", type=int, default=16)
    parser.add_argument("--route-pool-time-limit", type=float, default=2.0)
    parser.add_argument("--disable-root-compact-warm-start", action="store_true")
    parser.add_argument("--root-compact-solve-time-limit", type=float, default=60.0)
    parser.add_argument("--disable-incremental-rmp", action="store_true")
    parser.add_argument("--disable-active-coefficient-cache", action="store_true")
    parser.add_argument("--disable-row-local-sr-coeff-cache", action="store_true")
    parser.add_argument("--sr-cut-add-batch-size", type=int, default=32)
    parser.add_argument("--disable-global-branch-route-index", action="store_true")
    parser.add_argument("--disable-pricing-pruning", action="store_true")
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
    parser.add_argument("--logging-mode", choices=["audit", "light"], default="audit")
    parser.add_argument("--progress-snapshot-period", type=int, default=1)
    parser.add_argument("--gurobi-python-path", type=Path)
    parser.add_argument("--instance-file", type=Path)
    parser.add_argument("--instance-sha256")
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser


def main() -> None:
    args = _parser().parse_args()
    if args.gurobi_python_path is not None:
        sys.path.insert(0, str(args.gurobi_python_path))
    defaults = case_defaults(
        num_customers=args.num_customers,
        num_trucks=args.num_trucks,
        num_hubs=args.num_hubs,
        drones_per_truck=args.drones_per_truck,
    )
    for name, value in defaults.items():
        if hasattr(args, name) and getattr(args, name) is None:
            setattr(args, name, value)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    manual_deadlines = load_manual_service_deadline_bounds(args.service_deadline_file) if args.service_deadline_file else None
    instance_config = InstanceConfig(
        seed=args.seed,
        num_trucks=args.num_trucks,
        num_customers=args.num_customers,
        num_hubs=args.num_hubs,
        distribution=args.distribution,
        drones_per_truck=args.drones_per_truck,
        truck_arc_probability=args.truck_arc_probability,
        hub_arc_probability=args.hub_arc_probability,
        truck_speed=args.truck_speed,
        drone_speed=args.drone_speed,
        truck_payload=args.truck_payload,
        drone_payload=args.drone_payload,
        drone_endurance=args.drone_endurance,
        truck_cost=args.truck_cost,
        drone_cost=args.drone_cost,
        mandatory_drone_customer_fraction=args.mandatory_drone_customer_fraction,
        service_deadline_mode=args.service_deadline_mode,
        service_deadline_manual_bounds=manual_deadlines,
        service_deadline_offset_min=args.service_deadline_offset_min,
        service_deadline_offset_max=args.service_deadline_offset_max,
        service_deadline_random_seed=args.service_deadline_random_seed,
    )
    weights = ObjectiveWeights(*args.weights)
    solver_config = SolverConfig(
        threads=args.threads,
        time_limit=args.time_limit,
        pricing_tolerance=args.pricing_tolerance,
        route_pool_time_limit=args.route_pool_time_limit,
        pricing_batch_size=args.pricing_batch_size,
        farkas_batch_size=args.farkas_batch_size,
        enable_root_compact_warm_start=not args.disable_root_compact_warm_start,
        root_compact_solve_time_limit=args.root_compact_solve_time_limit,
        enable_incremental_rmp=not args.disable_incremental_rmp,
        enable_active_coefficient_cache=not args.disable_active_coefficient_cache,
        use_row_local_sr_coeff_cache=not args.disable_row_local_sr_coeff_cache,
        sr_cut_add_batch_size=args.sr_cut_add_batch_size,
        enable_global_branch_route_index=not args.disable_global_branch_route_index,
        enable_pricing_pruning=not args.disable_pricing_pruning,
        pricing_parallel_workers=args.pricing_parallel_workers,
        pricing_worker_backend=args.pricing_worker_backend,
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
        progress_snapshot_period=args.progress_snapshot_period,
        gurobi_log_dir=str(args.output_dir / "gurobi_logs"),
    )

    from .bpc import BPCTimeLimitNoIncumbent, solve_branch_price_cut

    if args.instance_sha256 is not None and args.instance_file is None:
        raise ValueError("--instance-sha256 requires --instance-file")
    instance_snapshot_sha256 = None
    if args.instance_file is None:
        instance = generate_instance(instance_config)
    else:
        instance, instance_snapshot_sha256 = read_instance_snapshot(
            args.instance_file,
            expected_sha256=args.instance_sha256,
        )
        validate_instance_case(
            instance,
            requested_seed=args.seed,
            num_trucks=args.num_trucks,
            num_customers=args.num_customers,
            num_hubs=args.num_hubs,
            distribution=args.distribution,
            drones_per_truck=args.drones_per_truck,
            expected_config=instance_config,
        )
    start = time.time()
    try:
        result = solve_branch_price_cut(instance, weights, solver_config)
        record = {
            "requested_instance_config": asdict(instance_config),
            "instance_config": asdict(instance.config),
            "instance_snapshot_file": None if args.instance_file is None else str(args.instance_file.resolve()),
            "instance_snapshot_sha256": instance_snapshot_sha256,
            "solver_config": asdict(solver_config),
            "total_drones": instance.num_trucks * instance.drones_per_truck,
            "locations": instance.locations,
            "demand": instance.demand,
            **instance_generation_metadata(instance),
            **result.to_record(),
            "service_metrics": solution_service_metrics(instance, result),
        }
    except BPCTimeLimitNoIncumbent as exc:
        record = {
            "status": "time_limit_no_incumbent",
            "runtime": exc.runtime,
            "nodes_processed": exc.nodes_processed,
            "lower_bound_shifted": exc.lower_bound_shifted,
            "lower_bound_full": exc.objective.full_value_from_route_sum(exc.lower_bound_shifted),
            "requested_instance_config": asdict(instance_config),
            "instance_config": asdict(instance.config),
            "instance_snapshot_file": None if args.instance_file is None else str(args.instance_file.resolve()),
            "instance_snapshot_sha256": instance_snapshot_sha256,
            "solver_config": asdict(solver_config),
            "total_drones": instance.num_trucks * instance.drones_per_truck,
            **instance_generation_metadata(instance),
            "bpc_stats": asdict(exc.stats),
            "wall_runtime": time.time() - start,
        }
    output_file = args.output_dir / f"thvrpd-seed={args.seed}.json"
    output_file.write_text(json.dumps(record, indent=2), encoding="utf-8")
    print(output_file)


if __name__ == "__main__":
    main()
