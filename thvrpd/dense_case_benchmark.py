from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys
import time


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--gurobi-python-path", type=Path, required=True)
    parser.add_argument("--time-limit", type=float, default=1800.0)
    parser.add_argument("--threads", type=int, default=0)
    parser.add_argument("--truck-arc-probability", type=float, default=0.10)
    parser.add_argument("--hub-arc-probability", type=float, default=0.30)
    return parser


def main() -> None:
    args = _parser().parse_args()
    if args.time_limit <= 0.0 or args.threads < 0:
        raise ValueError("time limit must be positive and threads nonnegative")
    sys.path.insert(0, str(args.gurobi_python_path.resolve()))

    from .campaign import CampaignCase, _run_monitored, _solver_command, _summary_row
    from .config import InstanceConfig, ObjectiveWeights
    from .experiments import SCALES
    from .instance import (
        generate_instance,
        instance_physical_fingerprint,
        validate_instance_case,
        write_instance_snapshot,
    )
    from .transform import build_transformed_graph

    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=False)
    case = CampaignCase("large", "PS", 1, dict(SCALES["large"]))
    config = InstanceConfig(
        seed=case.seed,
        distribution=case.distribution,
        truck_arc_probability=args.truck_arc_probability,
        hub_arc_probability=args.hub_arc_probability,
        **case.dimensions,
    )
    generation_start = time.time()
    instance = generate_instance(config)
    generation_wall = time.time() - generation_start
    validate_instance_case(
        instance,
        requested_seed=case.seed,
        num_trucks=case.dimensions["num_trucks"],
        num_customers=case.dimensions["num_customers"],
        num_hubs=case.dimensions["num_hubs"],
        distribution=case.distribution,
        drones_per_truck=case.dimensions["drones_per_truck"],
        expected_config=config,
    )
    instances_dir = output_dir / "instances"
    instances_dir.mkdir()
    snapshot_path = instances_dir / f"{case.case_id}.json"
    snapshot_hash = write_instance_snapshot(instance, snapshot_path)
    transformed = build_transformed_graph(instance)
    case_record = {
        "case_id": case.case_id,
        "scale": case.scale,
        "distribution": case.distribution,
        "seed": case.seed,
        "dimensions": case.dimensions,
        "instance_config": asdict(instance.config),
        "requested_seed": instance.requested_seed,
        "realized_seed": instance.config.seed,
        "generation_attempt": instance.generation_attempt,
        "generation_feasibility_time": instance.generation_feasibility_time,
        "generation_wall_time": generation_wall,
        "instance_snapshot_file": str(snapshot_path),
        "instance_snapshot_sha256": snapshot_hash,
        "instance_physical_fingerprint": instance_physical_fingerprint(instance),
        "truck_arc_count": len(instance.truck_arcs),
        "drone_arc_count": len(instance.drone_arcs),
        "transformed_node_count": len(transformed.nodes),
        "transformed_arc_count": len(transformed.arcs),
        "truck_arc_probability": args.truck_arc_probability,
        "hub_arc_probability": args.hub_arc_probability,
        "compact_time_limit": args.time_limit,
        "compact_threads": args.threads,
    }
    manifest = {
        "schema_version": 1,
        "case_count": 1,
        "solver_run_count": 2,
        "case_order": [case.case_id],
        "solvers": ["bpc", "compact"],
        "time_limit_seconds_per_run": args.time_limit,
        "threads": args.threads,
        "objective_weights": asdict(ObjectiveWeights(0.4, 0.3, 0.3)),
        "gurobi_python_path": str(args.gurobi_python_path.resolve()),
        "python_executable": sys.executable,
        "cases": [case_record],
    }
    _write_json(output_dir / "campaign_manifest.json", manifest)
    _write_json(output_dir / "dense_instance_manifest.json", case_record)

    rows = []
    for solver in ("bpc", "compact"):
        attempt_dir = output_dir / "cases" / case.case_id / solver / "attempt_001"
        attempt_dir.mkdir(parents=True)
        command = _solver_command(
            case,
            solver,
            attempt_dir,
            args.time_limit,
            args.threads,
            args.gurobi_python_path,
            snapshot_path,
            snapshot_hash,
        )
        command.extend(
            [
                "--truck-arc-probability",
                str(args.truck_arc_probability),
                "--hub-arc-probability",
                str(args.hub_arc_probability),
            ]
        )
        print(f"START {case.case_id} {solver} attempt_001", flush=True)
        _run_monitored(command, attempt_dir, 1.0, case, solver, snapshot_hash)
        status = json.loads((attempt_dir / "run_status.json").read_text(encoding="utf-8"))
        if not status["completed"]:
            raise RuntimeError(f"{solver} attempt did not complete: {status}")
        rows.append(_summary_row(case_record, solver, attempt_dir))
        print(f"END   {case.case_id} {solver} attempt_001", flush=True)
    _write_json(output_dir / "campaign_summary.json", rows)
    _write_json(
        output_dir / "campaign_status.json",
        {
            "expected_runs": 2,
            "completed_runs": 2,
            "pending_runs": 0,
            "failed_attempts": 0,
            "updated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        },
    )


def _write_json(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
