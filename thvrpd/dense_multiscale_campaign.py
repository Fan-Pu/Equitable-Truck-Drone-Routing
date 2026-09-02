from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys
import time

from .dense_large_campaign import _artifact_inventory, _write_csv


SCALE_ORDER = ("small", "medium")
SCENARIOS = (("PS", (1, 2, 3, 4)), ("PC", (5, 6, 7, 8)))
LARGE_RESULT_DIRS = (
    "large_PS_seed1_dense400_bpc_compact_1800s",
    "large_dense400_indices2_8_bpc_compact_1800s_campaign",
)


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
    from .dense_large_campaign import (
        _artifact_inventory,
        _refresh_outputs,
        _validate_dense_instance,
        _write_case_report,
        _write_csv,
    )
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
    guard_path = output_dir / "preserved_large_artifacts_sha256.json"
    current_large_hashes = _large_artifact_hashes(output_dir.parent)
    if guard_path.exists():
        if _read_json(guard_path)["artifacts"] != current_large_hashes:
            raise RuntimeError("retained large-result artifacts changed")
    else:
        _write_json(
            guard_path,
            {
                "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                "large_result_dirs": list(LARGE_RESULT_DIRS),
                "artifacts": current_large_hashes,
            },
        )

    cases = tuple(
        CampaignCase(scale, distribution, seed, dict(SCALES[scale]))
        for scale in SCALE_ORDER
        for distribution, seeds in SCENARIOS
        for seed in seeds
    )
    manifest_path = output_dir / "campaign_manifest.json"
    if manifest_path.exists():
        manifest = _read_json(manifest_path)
        _validate_manifest(manifest, args, cases)
        case_records = {record["case_id"]: record for record in manifest["cases"]}
        for case in cases:
            record = case_records[case.case_id]
            instance, digest = read_instance_snapshot(
                Path(record["instance_snapshot_file"]),
                expected_sha256=record["instance_snapshot_sha256"],
            )
            _validate_dense_instance(instance, case, args)
            if digest != record["instance_snapshot_sha256"]:
                raise RuntimeError(f"snapshot digest changed for {case.case_id}")
    else:
        instances_dir = output_dir / "instances"
        instances_dir.mkdir(exist_ok=True)
        preparation_path = output_dir / "snapshot_preparation.json"
        records = _read_json(preparation_path)["cases"] if preparation_path.exists() else []
        prepared = {record["case_id"]: record for record in records}
        for case in cases:
            snapshot_path = instances_dir / f"{case.case_id}.json"
            if case.case_id in prepared:
                record = prepared[case.case_id]
                instance, digest = read_instance_snapshot(
                    snapshot_path,
                    expected_sha256=record["instance_snapshot_sha256"],
                )
                _validate_dense_instance(instance, case, args)
                continue
            config = InstanceConfig(
                seed=case.seed,
                distribution=case.distribution,
                truck_arc_probability=args.truck_arc_probability,
                hub_arc_probability=args.hub_arc_probability,
                **case.dimensions,
            )
            if snapshot_path.exists():
                instance, digest = read_instance_snapshot(snapshot_path)
                generation_wall = None
            else:
                generation_start = time.time()
                instance = generate_instance(config)
                generation_wall = time.time() - generation_start
                digest = write_instance_snapshot(instance, snapshot_path)
            _validate_dense_instance(instance, case, args)
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
                "compact_time_limit": args.time_limit,
                "compact_threads": args.threads,
            }
            records.append(record)
            prepared[case.case_id] = record
            _write_json(preparation_path, {"cases": records})
            print(
                f"SNAPSHOT {case.case_id} arcs={record['transformed_arc_count']} sha256={digest}",
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
        record = case_records[case.case_id]
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
                _refresh_outputs(output_dir, manifest, cases, _successful_attempt, _summary_row)
                raise RuntimeError(f"{case.case_id} {solver} failed: {run_status}")
            print(f"END   {case.case_id} {solver} {attempt_dir.name}", flush=True)
            _refresh_outputs(output_dir, manifest, cases, _successful_attempt, _summary_row)
        _write_case_report(output_dir, case.case_id)
    _refresh_outputs(output_dir, manifest, cases, _successful_attempt, _summary_row)
    _write_aggregate_report(output_dir, manifest)
    if _large_artifact_hashes(output_dir.parent) != current_large_hashes:
        raise RuntimeError("retained large-result artifacts changed during campaign")
    guard = _read_json(guard_path)
    guard["verified_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
    guard["verified"] = True
    _write_json(guard_path, guard)
    _write_csv(output_dir / "artifact_inventory.csv", _artifact_inventory(output_dir))


def _validate_manifest(manifest, args, cases) -> None:
    if manifest.get("case_order") != [case.case_id for case in cases]:
        raise ValueError("small/medium campaign case order mismatch")
    if manifest.get("time_limit_seconds_per_run") != args.time_limit:
        raise ValueError("small/medium campaign time limit mismatch")
    if manifest.get("threads") != args.threads:
        raise ValueError("small/medium campaign thread count mismatch")
    if manifest.get("truck_arc_probability") != args.truck_arc_probability:
        raise ValueError("small/medium campaign truck probability mismatch")
    if manifest.get("hub_arc_probability") != args.hub_arc_probability:
        raise ValueError("small/medium campaign hub probability mismatch")


def _write_aggregate_report(output_dir: Path, manifest) -> None:
    records = []
    for case in manifest["cases"]:
        report_path = output_dir / f"{case['case_id']}_solution_process_report.json"
        report = _read_json(report_path)
        comparison = report["comparison"]
        stats = report["raw_results"]["bpc"].get("bpc_stats", {})
        records.append(
            {
                "case_id": case["case_id"],
                "scale": case["scale"],
                "distribution": case["distribution"],
                "seed": case["seed"],
                "realized_seed": case["realized_seed"],
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
                "bpc_root_warm_start_status": stats.get("root_compact_status"),
                "bpc_root_warm_start_columns": stats.get("root_compact_accepted_columns"),
                "bpc_root_warm_start_validated": stats.get("root_compact_incumbent_validated"),
                "bpc_pricing_calls": (
                    stats.get("standard_pricing_calls", 0) + stats.get("farkas_pricing_calls", 0)
                ),
                "bpc_routes": stats.get("total_routes"),
                "bpc_labels": stats.get("pricing_labels_generated"),
                "bpc_sr_cuts": stats.get("sr_cuts_added"),
                "bpc_branches": stats.get("branching_nodes"),
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
                "detail_report_markdown": str(output_dir / f"{case['case_id']}_solution_process_report.md"),
            }
        )
    _write_json(output_dir / "dense_small_medium_comparison.json", {"cases": records})
    _write_csv(output_dir / "dense_small_medium_comparison.csv", records)
    lines = [
        "# Dense Small and Medium Solver Comparison",
        "",
        "| Case | Arcs | BPC status | BPC time | BPC gap | Compact status | Compact time | Compact gap |",
        "|---|---:|---|---:|---:|---|---:|---:|",
    ]
    for record in records:
        lines.append(
            f"| {record['case_id']} | {record['transformed_arcs']} | {record['bpc_status']} | "
            f"{record['bpc_runtime']:.3f} | {_format(record['bpc_gap'])} | {record['compact_status']} | "
            f"{record['compact_runtime']:.3f} | {_format(record['compact_gap'])} |"
        )
    (output_dir / "dense_small_medium_comparison.md").write_text(
        "\n".join(lines) + "\n",
        encoding="utf-8",
    )
    _write_csv(output_dir / "artifact_inventory.csv", _artifact_inventory(output_dir))


def _large_artifact_hashes(numerical_root: Path) -> dict[str, str]:
    records = {}
    for directory_name in LARGE_RESULT_DIRS:
        directory = numerical_root / directory_name
        if not directory.is_dir():
            raise FileNotFoundError(f"missing retained large-result directory: {directory}")
        for path in sorted(item for item in directory.rglob("*") if item.is_file()):
            relative = str(path.relative_to(numerical_root))
            records[relative] = hashlib.sha256(path.read_bytes()).hexdigest()
    return records


def _format(value) -> str:
    return "n/a" if value is None else f"{value:.6f}"


def _read_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def _write_json(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, allow_nan=False), encoding="utf-8")


if __name__ == "__main__":
    main()
