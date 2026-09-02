from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys
import time

from .dense_large_campaign import _artifact_inventory, _write_csv


PILOT_CONFIGS = (
    {"num_customers": 15, "num_trucks": 3, "num_hubs": 2, "drones_per_truck": 4},
    {"num_customers": 20, "num_trucks": 4, "num_hubs": 3, "drones_per_truck": 4},
    {"num_customers": 25, "num_trucks": 5, "num_hubs": 3, "drones_per_truck": 4},
)
PILOT_SCENARIOS = (("PS", (101, 102)), ("PC", (105, 106)))
FINAL_SCENARIOS = (("PS", (1, 2, 3, 4)), ("PC", (5, 6, 7, 8)))


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--gurobi-python-path", type=Path, required=True)
    parser.add_argument("--pilot-time-limit", type=float, default=600.0)
    parser.add_argument("--final-time-limit", type=float, default=1800.0)
    parser.add_argument("--threads", type=int, default=0)
    parser.add_argument("--truck-arc-probability", type=float, default=0.10)
    parser.add_argument("--hub-arc-probability", type=float, default=0.30)
    return parser


def main() -> None:
    args = _parser().parse_args()
    if min(args.pilot_time_limit, args.final_time_limit) <= 0.0 or args.threads < 0:
        raise ValueError("time limits must be positive and threads nonnegative")
    sys.path.insert(0, str(args.gurobi_python_path.resolve()))

    from .campaign import CampaignCase

    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    guard_path = output_dir / "preserved_existing_results_sha256.json"
    existing_hashes = _existing_result_hashes(output_dir.parent, output_dir)
    if guard_path.exists():
        if _read_json(guard_path)["artifacts"] != existing_hashes:
            raise RuntimeError("pre-existing numerical results changed")
    else:
        _write_json(
            guard_path,
            {
                "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                "artifacts": existing_hashes,
            },
        )

    pilot_summaries = []
    for dimensions in PILOT_CONFIGS:
        customer_count = dimensions["num_customers"]
        cases = tuple(
            CampaignCase(f"pilot{customer_count}", distribution, seed, dict(dimensions))
            for distribution, seeds in PILOT_SCENARIOS
            for seed in seeds
        )
        stage_dir = output_dir / f"pilot_n{customer_count}"
        _run_stage(stage_dir, cases, args.pilot_time_limit, args, output_dir, f"pilot_n{customer_count}")
        summary = _pilot_metrics(stage_dir, dimensions)
        pilot_summaries.append(summary)
        _write_json(output_dir / "pilot_selection.json", {"pilots": pilot_summaries})

    selected = next((summary for summary in pilot_summaries if summary["qualifies"]), pilot_summaries[-1])
    selected_dimensions = selected["dimensions"]
    final_cases = tuple(
        CampaignCase("medium", distribution, seed, dict(selected_dimensions))
        for distribution, seeds in FINAL_SCENARIOS
        for seed in seeds
    )
    final_dir = output_dir / "final"
    _write_json(
        output_dir / "selected_scale.json",
        {
            "selected_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "selection_rule": (
                "smallest pilot with lower BPC median runtime, at least as many BPC optimal solves, "
                "and no worse BPC median gap; default to n=25 if none qualifies"
            ),
            "selected": selected,
            "held_out_case_order": [case.case_id for case in final_cases],
        },
    )
    _run_stage(final_dir, final_cases, args.final_time_limit, args, output_dir, "final")
    _write_crossover_report(output_dir, pilot_summaries, selected, final_dir)
    if _existing_result_hashes(output_dir.parent, output_dir) != existing_hashes:
        raise RuntimeError("pre-existing numerical results changed during crossover campaign")
    guard = _read_json(guard_path)
    guard["verified"] = True
    guard["verified_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
    _write_json(guard_path, guard)
    _write_csv(output_dir / "artifact_inventory.csv", _artifact_inventory(output_dir))
    _write_root_status(output_dir, "complete")


def _run_stage(stage_dir, cases, time_limit, args, root_dir, phase) -> None:
    from .campaign import (
        _new_attempt_dir,
        _run_monitored,
        _solver_command,
        _successful_attempt,
        _summary_row,
    )
    from .config import InstanceConfig, ObjectiveWeights
    from .dense_large_campaign import _refresh_outputs, _validate_dense_instance, _write_case_report
    from .instance import (
        generate_instance,
        instance_physical_fingerprint,
        read_instance_snapshot,
        write_instance_snapshot,
    )
    from .transform import build_transformed_graph

    stage_dir.mkdir(parents=True, exist_ok=True)
    manifest_path = stage_dir / "campaign_manifest.json"
    if manifest_path.exists():
        manifest = _read_json(manifest_path)
        _validate_stage_manifest(manifest, cases, time_limit, args)
        records = {record["case_id"]: record for record in manifest["cases"]}
        for case in cases:
            record = records[case.case_id]
            instance, _ = read_instance_snapshot(
                Path(record["instance_snapshot_file"]),
                expected_sha256=record["instance_snapshot_sha256"],
            )
            _validate_dense_instance(instance, case, args)
    else:
        preparation_path = stage_dir / "snapshot_preparation.json"
        prepared_records = _read_json(preparation_path)["cases"] if preparation_path.exists() else []
        records = {record["case_id"]: record for record in prepared_records}
        instances_dir = stage_dir / "instances"
        instances_dir.mkdir(exist_ok=True)
        for case in cases:
            if case.case_id in records:
                continue
            config = InstanceConfig(
                seed=case.seed,
                distribution=case.distribution,
                truck_arc_probability=args.truck_arc_probability,
                hub_arc_probability=args.hub_arc_probability,
                **case.dimensions,
            )
            start = time.time()
            instance = generate_instance(config)
            generation_wall = time.time() - start
            _validate_dense_instance(instance, case, args)
            snapshot_path = instances_dir / f"{case.case_id}.json"
            digest = write_instance_snapshot(instance, snapshot_path)
            transformed = build_transformed_graph(instance)
            record = {
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
                "instance_snapshot_sha256": digest,
                "instance_physical_fingerprint": instance_physical_fingerprint(instance),
                "truck_arc_count": len(instance.truck_arcs),
                "drone_arc_count": len(instance.drone_arcs),
                "transformed_node_count": len(transformed.nodes),
                "transformed_arc_count": len(transformed.arcs),
                "truck_arc_probability": args.truck_arc_probability,
                "hub_arc_probability": args.hub_arc_probability,
            }
            records[case.case_id] = record
            _write_json(preparation_path, {"cases": list(records.values())})
            print(f"SNAPSHOT {case.case_id} arcs={len(transformed.arcs)} sha256={digest}", flush=True)
        manifest = {
            "schema_version": 1,
            "case_count": len(cases),
            "solver_run_count": 2 * len(cases),
            "case_order": [case.case_id for case in cases],
            "solvers": ["bpc", "compact"],
            "time_limit_seconds_per_run": time_limit,
            "threads": args.threads,
            "objective_weights": asdict(ObjectiveWeights(0.4, 0.3, 0.3)),
            "gurobi_python_path": str(args.gurobi_python_path.resolve()),
            "truck_arc_probability": args.truck_arc_probability,
            "hub_arc_probability": args.hub_arc_probability,
            "cases": [records[case.case_id] for case in cases],
        }
        _write_json(manifest_path, manifest)
        _refresh_outputs(stage_dir, manifest, cases, _successful_attempt, _summary_row)

    for case in cases:
        record = records[case.case_id]
        for solver in ("bpc", "compact"):
            solver_dir = stage_dir / "cases" / case.case_id / solver
            if _successful_attempt(solver_dir, solver, case.seed) is not None:
                print(f"SKIP  {case.case_id} {solver}", flush=True)
                continue
            attempt_dir = _new_attempt_dir(solver_dir)
            command = _solver_command(
                case,
                solver,
                attempt_dir,
                time_limit,
                args.threads,
                args.gurobi_python_path,
                Path(record["instance_snapshot_file"]),
                record["instance_snapshot_sha256"],
            )
            command.extend(
                [
                    "--truck-arc-probability",
                    str(args.truck_arc_probability),
                    "--hub-arc-probability",
                    str(args.hub_arc_probability),
                ]
            )
            print(f"START {case.case_id} {solver} {attempt_dir.name}", flush=True)
            _run_monitored(command, attempt_dir, 1.0, case, solver, record["instance_snapshot_sha256"])
            run_status = _read_json(attempt_dir / "run_status.json")
            if not run_status["completed"]:
                _refresh_outputs(stage_dir, manifest, cases, _successful_attempt, _summary_row)
                _write_root_status(root_dir, phase)
                raise RuntimeError(f"{case.case_id} {solver} failed: {run_status}")
            print(f"END   {case.case_id} {solver} {attempt_dir.name}", flush=True)
            _refresh_outputs(stage_dir, manifest, cases, _successful_attempt, _summary_row)
            _write_root_status(root_dir, phase)
        _write_case_report(stage_dir, case.case_id)
    _refresh_outputs(stage_dir, manifest, cases, _successful_attempt, _summary_row)
    _write_stage_aggregate(stage_dir, manifest)
    _write_root_status(root_dir, phase)


def _pilot_metrics(stage_dir, dimensions) -> dict[str, object]:
    records = _read_json(stage_dir / "stage_comparison.json")["cases"]
    bpc_times = [record["bpc_runtime"] for record in records]
    mip_times = [record["compact_runtime"] for record in records]
    bpc_gaps = [_gap_for_selection(record["bpc_gap"]) for record in records]
    mip_gaps = [_gap_for_selection(record["compact_gap"]) for record in records]
    bpc_median_gap_score = statistics.median(bpc_gaps)
    mip_median_gap_score = statistics.median(mip_gaps)
    bpc_optimal = sum(record["bpc_status"] == "optimal" for record in records)
    mip_optimal = sum(record["compact_status_code"] == 2 for record in records)
    metrics = {
        "dimensions": dimensions,
        "case_count": len(records),
        "bpc_median_runtime": statistics.median(bpc_times),
        "mip_median_runtime": statistics.median(mip_times),
        "bpc_optimal_count": bpc_optimal,
        "mip_optimal_count": mip_optimal,
        "bpc_median_gap": _finite_or_none(bpc_median_gap_score),
        "mip_median_gap": _finite_or_none(mip_median_gap_score),
    }
    metrics["qualifies"] = (
        metrics["bpc_median_runtime"] < metrics["mip_median_runtime"]
        and bpc_optimal >= mip_optimal
        and bpc_median_gap_score <= mip_median_gap_score
    )
    return metrics


def _write_stage_aggregate(stage_dir, manifest) -> None:
    records = []
    for case in manifest["cases"]:
        report = _read_json(stage_dir / f"{case['case_id']}_solution_process_report.json")
        comparison = report["comparison"]
        stats = report["raw_results"]["bpc"].get("bpc_stats", {})
        compact = report["raw_results"]["compact"]
        records.append(
            {
                "case_id": case["case_id"],
                "distribution": case["distribution"],
                "seed": case["seed"],
                "transformed_arcs": case["transformed_arc_count"],
                "snapshot_sha256": case["instance_snapshot_sha256"],
                "bpc_status": comparison["bpc_status"],
                "bpc_objective": comparison["bpc_objective"],
                "bpc_bound": comparison["bpc_bound"],
                "bpc_gap": comparison["bpc_gap"],
                "bpc_runtime": comparison["bpc_runtime"],
                "bpc_nodes": comparison["bpc_nodes"],
                "bpc_root_validated": stats.get("root_compact_incumbent_validated"),
                "compact_status": comparison["compact_termination"],
                "compact_status_code": compact.get("status_code"),
                "compact_objective": comparison["compact_objective"],
                "compact_bound": comparison["compact_bound"],
                "compact_gap": comparison["compact_gap"],
                "compact_runtime": comparison["compact_runtime"],
                "compact_nodes": comparison["compact_nodes"],
                "bpc_valid": report["independent_validation"]["bpc"]["valid"],
                "compact_valid": report["independent_validation"]["compact"]["valid"],
                "bpc_gurobi_version": report["bpc_root_log_summary"]["gurobi_version"],
                "compact_gurobi_version": report["compact_log_summary"]["gurobi_version"],
            }
        )
    _write_json(stage_dir / "stage_comparison.json", {"cases": records})
    _write_csv(stage_dir / "stage_comparison.csv", records)
    _write_csv(stage_dir / "artifact_inventory.csv", _artifact_inventory(stage_dir))


def _write_crossover_report(output_dir, pilots, selected, final_dir) -> None:
    final_records = _read_json(final_dir / "stage_comparison.json")["cases"]
    report = {
        "selection_rule": (
            "smallest scale with lower BPC median runtime, at least as many BPC optimal solves, "
            "and no worse BPC median gap"
        ),
        "pilots": pilots,
        "selected": selected,
        "final_cases": final_records,
    }
    _write_json(output_dir / "crossover_report.json", report)
    lines = [
        "# Medium-Scale Crossover Study",
        "",
        "| Customers | BPC median time | MIQP median time | BPC optimal | MIQP optimal | BPC gap | MIQP gap | Qualifies |",
        "|---:|---:|---:|---:|---:|---:|---:|---|",
    ]
    for pilot in pilots:
        lines.append(
            f"| {pilot['dimensions']['num_customers']} | {pilot['bpc_median_runtime']:.3f} | "
            f"{pilot['mip_median_runtime']:.3f} | {pilot['bpc_optimal_count']} | {pilot['mip_optimal_count']} | "
            f"{_format(pilot['bpc_median_gap'])} | {_format(pilot['mip_median_gap'])} | "
            f"{pilot['qualifies']} |"
        )
    lines.extend(
        [
            "",
            f"Selected configuration: `{selected['dimensions']}`.",
            "",
            f"Held-out final cases: `{len(final_records)}`.",
        ]
    )
    (output_dir / "crossover_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def _validate_stage_manifest(manifest, cases, time_limit, args) -> None:
    if manifest.get("case_order") != [case.case_id for case in cases]:
        raise ValueError("stage case order mismatch")
    if manifest.get("time_limit_seconds_per_run") != time_limit:
        raise ValueError("stage time limit mismatch")
    if manifest.get("threads") != args.threads:
        raise ValueError("stage thread count mismatch")
    if manifest.get("truck_arc_probability") != args.truck_arc_probability:
        raise ValueError("stage truck probability mismatch")
    if manifest.get("hub_arc_probability") != args.hub_arc_probability:
        raise ValueError("stage hub probability mismatch")


def _write_root_status(output_dir, phase) -> None:
    run_statuses = list(output_dir.glob("**/run_status.json"))
    completed = 0
    failed = 0
    for path in run_statuses:
        status = _read_json(path)
        if status.get("completed"):
            completed += 1
        else:
            failed += 1
    _write_json(
        output_dir / "campaign_status.json",
        {
            "expected_runs": 40,
            "completed_runs": completed,
            "pending_runs": 40 - completed,
            "failed_attempts": failed,
            "phase": phase,
            "updated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        },
    )


def _existing_result_hashes(numerical_root, output_dir) -> dict[str, str]:
    records = {}
    for directory in sorted(path for path in numerical_root.iterdir() if path.is_dir() and path != output_dir):
        for path in sorted(item for item in directory.rglob("*") if item.is_file()):
            records[str(path.relative_to(numerical_root))] = hashlib.sha256(path.read_bytes()).hexdigest()
    return records


def _gap_for_selection(value) -> float:
    return float("inf") if value is None else float(value)


def _finite_or_none(value):
    return value if math.isfinite(value) else None


def _format(value) -> str:
    return "unresolved" if value is None else f"{value:.6f}"


def _read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _write_json(path, value) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, allow_nan=False), encoding="utf-8")


if __name__ == "__main__":
    main()
