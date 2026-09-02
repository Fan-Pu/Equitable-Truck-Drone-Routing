from __future__ import annotations

import argparse
import csv
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

from .instance import validate_instance_case

DENSE_SCENARIOS = (("PS", (2, 3, 4)), ("PC", (5, 6, 7, 8)))


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

    from .campaign import (
        CampaignCase,
        _new_attempt_dir,
        _run_monitored,
        _solver_command,
        _successful_attempt,
        _summary_row,
    )
    from .config import InstanceConfig, ObjectiveWeights
    from .experiments import SCALES
    from .instance import (
        generate_instance,
        instance_physical_fingerprint,
        read_instance_snapshot,
        write_instance_snapshot,
    )
    from .transform import build_transformed_graph

    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    cases = tuple(
        CampaignCase("large", distribution, seed, dict(SCALES["large"]))
        for distribution, seeds in DENSE_SCENARIOS
        for seed in seeds
    )
    manifest_path = output_dir / "campaign_manifest.json"
    if manifest_path.exists():
        manifest = _read_json(manifest_path)
        _validate_manifest(manifest, args, cases)
        case_records = {record["case_id"]: record for record in manifest["cases"]}
        for case in cases:
            instance, digest = read_instance_snapshot(
                Path(case_records[case.case_id]["instance_snapshot_file"]),
                expected_sha256=case_records[case.case_id]["instance_snapshot_sha256"],
            )
            _validate_dense_instance(instance, case, args)
            if digest != case_records[case.case_id]["instance_snapshot_sha256"]:
                raise RuntimeError(f"snapshot digest changed for {case.case_id}")
    else:
        instances_dir = output_dir / "instances"
        instances_dir.mkdir(exist_ok=True)
        records = []
        for case in cases:
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
            _validate_dense_instance(instance, case, args)
            snapshot_path = instances_dir / f"{case.case_id}.json"
            snapshot_hash = write_instance_snapshot(instance, snapshot_path)
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
            records.append(record)
            print(
                f"SNAPSHOT {case.case_id} arcs={record['transformed_arc_count']} "
                f"sha256={snapshot_hash}",
                flush=True,
            )
        manifest = {
            "schema_version": 1,
            "case_count": len(cases),
            "solver_run_count": 2 * len(cases),
            "case_order": [case.case_id for case in cases],
            "solvers": ["bpc", "compact"],
            "time_limit_seconds_per_run": args.time_limit,
            "threads": args.threads,
            "objective_weights": asdict(ObjectiveWeights(0.4, 0.3, 0.3)),
            "gurobi_python_path": str(args.gurobi_python_path.resolve()),
            "python_executable": sys.executable,
            "truck_arc_probability": args.truck_arc_probability,
            "hub_arc_probability": args.hub_arc_probability,
            "cases": records,
        }
        _write_json(manifest_path, manifest)
        case_records = {record["case_id"]: record for record in records}
        _refresh_outputs(output_dir, manifest, cases, _successful_attempt, _summary_row)

    for case in cases:
        case_record = case_records[case.case_id]
        for solver in ("bpc", "compact"):
            solver_dir = output_dir / "cases" / case.case_id / solver
            existing = _successful_attempt(solver_dir, solver, case.seed)
            if existing is not None:
                print(f"SKIP  {case.case_id} {solver} {existing.name}", flush=True)
                continue
            attempt_dir = _new_attempt_dir(solver_dir)
            command = _solver_command(
                case,
                solver,
                attempt_dir,
                args.time_limit,
                args.threads,
                args.gurobi_python_path,
                Path(case_record["instance_snapshot_file"]),
                case_record["instance_snapshot_sha256"],
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
            _run_monitored(
                command,
                attempt_dir,
                1.0,
                case,
                solver,
                case_record["instance_snapshot_sha256"],
            )
            run_status = _read_json(attempt_dir / "run_status.json")
            if not run_status["completed"]:
                _refresh_outputs(output_dir, manifest, cases, _successful_attempt, _summary_row)
                raise RuntimeError(f"{case.case_id} {solver} failed: {run_status}")
            print(f"END   {case.case_id} {solver} {attempt_dir.name}", flush=True)
            _refresh_outputs(output_dir, manifest, cases, _successful_attempt, _summary_row)
        _write_case_report(output_dir, case.case_id)
    _refresh_outputs(output_dir, manifest, cases, _successful_attempt, _summary_row)
    _write_aggregate_report(output_dir, manifest)


def _validate_dense_instance(instance, case, args) -> None:
    validate_instance_case(
        instance,
        requested_seed=case.seed,
        num_trucks=case.dimensions["num_trucks"],
        num_customers=case.dimensions["num_customers"],
        num_hubs=case.dimensions["num_hubs"],
        distribution=case.distribution,
        drones_per_truck=case.dimensions["drones_per_truck"],
        expected_config=instance.config,
    )
    if instance.config.truck_arc_probability != args.truck_arc_probability:
        raise ValueError("truck arc probability mismatch")
    if instance.config.hub_arc_probability != args.hub_arc_probability:
        raise ValueError("hub arc probability mismatch")


def _validate_manifest(manifest, args, cases) -> None:
    expected_ids = [case.case_id for case in cases]
    if manifest.get("case_order") != expected_ids:
        raise ValueError("existing dense campaign case order mismatch")
    if manifest.get("time_limit_seconds_per_run") != args.time_limit:
        raise ValueError("existing dense campaign time limit mismatch")
    if manifest.get("threads") != args.threads:
        raise ValueError("existing dense campaign thread count mismatch")
    if manifest.get("truck_arc_probability") != args.truck_arc_probability:
        raise ValueError("existing dense campaign truck probability mismatch")
    if manifest.get("hub_arc_probability") != args.hub_arc_probability:
        raise ValueError("existing dense campaign hub probability mismatch")


def _refresh_outputs(output_dir, manifest, cases, successful_attempt, summary_row) -> None:
    rows = []
    failed = 0
    for case in cases:
        case_record = next(record for record in manifest["cases"] if record["case_id"] == case.case_id)
        for solver in ("bpc", "compact"):
            solver_dir = output_dir / "cases" / case.case_id / solver
            attempt = successful_attempt(solver_dir, solver, case.seed)
            failed += sum(
                1
                for path in solver_dir.glob("attempt_*/run_status.json")
                if not _read_json(path).get("completed")
            )
            if attempt is not None:
                rows.append(summary_row(case_record, solver, attempt))
    _write_json(output_dir / "campaign_summary.json", rows)
    _write_json(
        output_dir / "campaign_status.json",
        {
            "expected_runs": manifest["solver_run_count"],
            "completed_runs": len(rows),
            "pending_runs": manifest["solver_run_count"] - len(rows),
            "failed_attempts": failed,
            "updated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        },
    )


def _write_case_report(output_dir: Path, case_id: str) -> None:
    subprocess.run(
        [
            sys.executable,
            "-m",
            "thvrpd.single_case_solution_report",
            "--campaign-dir",
            str(output_dir),
            "--case-id",
            case_id,
        ],
        check=True,
        cwd=Path.cwd(),
    )


def _write_aggregate_report(output_dir: Path, manifest) -> None:
    records = []
    for case in manifest["cases"]:
        report_path = output_dir / f"{case['case_id']}_solution_process_report.json"
        report = _read_json(report_path)
        comparison = report["comparison"]
        stats = report["raw_results"]["bpc"].get("bpc_stats", {})
        certification = case.get("warm_start_certification")
        screen = None if certification is None else certification["diagnostics"]
        performance = case.get("performance_selection")
        records.append(
            {
                "case_id": case["case_id"],
                "distribution": case["distribution"],
                "seed": case["seed"],
                "realized_seed": case["realized_seed"],
                "warm_start_filtered": certification is not None,
                "warm_start_screen_attempt": None if screen is None else screen["attempt"],
                "warm_start_screen_objective": None if screen is None else screen["objective"],
                "warm_start_screen_first_incumbent": None if screen is None else screen["first_incumbent_time"],
                "performance_filtered": performance is not None,
                "performance_selection_attempt": None if performance is None else performance["attempt"],
                "performance_trial_bpc_gap": None if performance is None else performance["bpc_trial"]["gap"],
                "performance_trial_compact_status_code": (
                    None if performance is None else performance["compact_trial"]["status_code"]
                ),
                "transformed_nodes": case["transformed_node_count"],
                "transformed_arcs": case["transformed_arc_count"],
                "snapshot_sha256": case["instance_snapshot_sha256"],
                "bpc_status": comparison["bpc_status"],
                "bpc_objective": comparison["bpc_objective"],
                "bpc_bound": comparison["bpc_bound"],
                "bpc_gap": comparison["bpc_gap"],
                "bpc_runtime": comparison["bpc_runtime"],
                "bpc_nodes": comparison["bpc_nodes"],
                "bpc_first_incumbent": stats.get("time_to_first_incumbent"),
                "bpc_branches": stats.get("branching_nodes"),
                "bpc_sr_cuts": stats.get("sr_cuts_added"),
                "bpc_routes": stats.get("total_routes"),
                "bpc_labels": stats.get("pricing_labels_generated"),
                "compact_status": comparison["compact_termination"],
                "compact_objective": comparison["compact_objective"],
                "compact_bound": comparison["compact_bound"],
                "compact_gap": comparison["compact_gap"],
                "compact_runtime": comparison["compact_runtime"],
                "compact_nodes": comparison["compact_nodes"],
                "compact_first_incumbent": comparison["compact_first_incumbent_seconds"],
                "bpc_incumbent_valid": report["independent_validation"]["bpc"]["valid"],
                "compact_incumbent_valid": report["independent_validation"]["compact"]["valid"],
                "bpc_gurobi_version": report["bpc_root_log_summary"]["gurobi_version"],
                "compact_gurobi_version": report["compact_log_summary"]["gurobi_version"],
                "detail_report_json": str(report_path),
                "detail_report_markdown": str(
                    output_dir / f"{case['case_id']}_solution_process_report.md"
                ),
            }
        )
    _write_json(output_dir / "dense_campaign_comparison.json", {"cases": records})
    _write_csv(output_dir / "dense_campaign_comparison.csv", records)
    (output_dir / "dense_campaign_comparison.md").write_text(
        _aggregate_markdown(records),
        encoding="utf-8",
    )
    _write_csv(output_dir / "artifact_inventory.csv", _artifact_inventory(output_dir))


def _aggregate_markdown(records) -> str:
    lines = [
        "# Dense Large Indices 2–8 Solver Comparison",
        "",
        "PC6 and PC8 are warm-start-filtered replacements. PC8 is additionally arc- and performance-filtered; "
        "these are not unbiased draws comparable to the other cases.",
        "",
        "| Case | Realized seed | Filtered | Arcs | BPC status | BPC time | BPC gap | Compact status | Compact time | Compact gap |",
        "|---|---:|---|---:|---|---:|---:|---|---:|---:|",
    ]
    for record in records:
        lines.append(
            f"| {record['case_id']} | {record['realized_seed']} | {record['warm_start_filtered']} | "
            f"{record['transformed_arcs']} | {record['bpc_status']} | "
            f"{record['bpc_runtime']:.3f} | {_number(record['bpc_gap'])} | "
            f"{record['compact_status']} | {record['compact_runtime']:.3f} | "
            f"{_number(record['compact_gap'])} |"
        )
    return "\n".join(lines) + "\n"


def _artifact_inventory(output_dir: Path):
    excluded = {"artifact_inventory.csv"}
    return [
        {
            "relative_path": str(path.relative_to(output_dir)),
            "size_bytes": path.stat().st_size,
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        }
        for path in sorted(item for item in output_dir.rglob("*") if item.is_file())
        if path.name not in excluded
    ]


def _write_csv(path: Path, records) -> None:
    if not records:
        path.write_text("", encoding="utf-8")
        return
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(records[0]))
        writer.writeheader()
        for record in records:
            writer.writerow(record)


def _number(value) -> str:
    return "n/a" if value is None else f"{value:.6f}"


def _read_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def _write_json(path: Path, value) -> None:
    path.write_text(json.dumps(value, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
