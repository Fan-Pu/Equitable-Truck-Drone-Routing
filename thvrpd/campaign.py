from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from typing import Any

import psutil

from .config import InstanceConfig, ObjectiveWeights, SolverConfig, case_defaults
from .experiments import SCALES
from .instance import (
    InstanceData,
    generate_instance,
    read_instance_snapshot,
    validate_instance_case,
    write_instance_snapshot,
)


SCENARIOS = (("PS", (1, 2, 3, 4)), ("PC", (5, 6, 7, 8)))
SOLVERS = ("bpc", "compact")


@dataclass(frozen=True)
class CampaignCase:
    scale: str
    distribution: str
    seed: int
    dimensions: dict[str, int]

    @property
    def case_id(self) -> str:
        return f"{self.scale}_{self.distribution}_seed{self.seed}"


@dataclass(frozen=True)
class FrozenCaseSnapshot:
    path: Path
    sha256: str
    instance: InstanceData


def campaign_cases(scales: tuple[str, ...] = ("large", "medium", "small")) -> tuple[CampaignCase, ...]:
    return tuple(
        CampaignCase(scale, distribution, seed, dict(SCALES[scale]))
        for scale in scales
        for distribution, seeds in SCENARIOS
        for seed in seeds
    )


def _expected_instance_config(case: CampaignCase) -> InstanceConfig:
    return InstanceConfig(seed=case.seed, distribution=case.distribution, **case.dimensions)


def _prepare_instance_snapshots(
    output_dir: Path,
    cases: tuple[CampaignCase, ...],
) -> dict[str, FrozenCaseSnapshot]:
    instances_dir = output_dir / "instances"
    instances_dir.mkdir(parents=True, exist_ok=True)
    snapshots: dict[str, FrozenCaseSnapshot] = {}
    for case in cases:
        expected_config = _expected_instance_config(case)
        snapshot_path = instances_dir / f"{case.case_id}.json"
        if snapshot_path.exists():
            instance, digest = read_instance_snapshot(snapshot_path)
        else:
            instance = generate_instance(expected_config)
            validate_instance_case(
                instance,
                requested_seed=case.seed,
                num_trucks=case.dimensions["num_trucks"],
                num_customers=case.dimensions["num_customers"],
                num_hubs=case.dimensions["num_hubs"],
                distribution=case.distribution,
                drones_per_truck=case.dimensions["drones_per_truck"],
                expected_config=expected_config,
            )
            digest = write_instance_snapshot(instance, snapshot_path)
            instance, verified_digest = read_instance_snapshot(snapshot_path, expected_sha256=digest)
            if verified_digest != digest:
                raise RuntimeError("newly written instance snapshot failed SHA-256 verification")
        validate_instance_case(
            instance,
            requested_seed=case.seed,
            num_trucks=case.dimensions["num_trucks"],
            num_customers=case.dimensions["num_customers"],
            num_hubs=case.dimensions["num_hubs"],
            distribution=case.distribution,
            drones_per_truck=case.dimensions["drones_per_truck"],
            expected_config=expected_config,
        )
        snapshots[case.case_id] = FrozenCaseSnapshot(snapshot_path, digest, instance)
    return snapshots


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--gurobi-python-path", type=Path, required=True)
    parser.add_argument("--time-limit", type=float, default=3600.0)
    parser.add_argument("--threads", type=int, default=0)
    parser.add_argument("--sample-period", type=float, default=1.0)
    parser.add_argument("--scales", nargs="+", choices=("large", "medium", "small"), default=["large", "medium", "small"])
    parser.add_argument("--solvers", nargs="+", choices=SOLVERS, default=list(SOLVERS))
    parser.add_argument("--max-runs", type=int)
    return parser


def main() -> None:
    args = _parser().parse_args()
    if args.time_limit <= 0.0 or args.threads < 0 or args.sample_period <= 0.0:
        raise ValueError("time limit and sample period must be positive, and thread count must be nonnegative")
    cases = campaign_cases(tuple(args.scales))
    args.output_dir.mkdir(parents=True, exist_ok=True)
    gurobi_path = str(args.gurobi_python_path.resolve())
    if gurobi_path not in sys.path:
        sys.path.append(gurobi_path)
    snapshots = _prepare_instance_snapshots(args.output_dir, cases)
    manifest = _campaign_manifest(
        cases,
        tuple(args.solvers),
        args.time_limit,
        args.threads,
        args.gurobi_python_path,
        snapshots,
    )
    manifest_path = args.output_dir / "campaign_manifest.json"
    if manifest_path.exists():
        existing = json.loads(manifest_path.read_text(encoding="utf-8"))
        if existing != manifest:
            raise ValueError("existing campaign manifest does not match the requested campaign")
    else:
        _write_json(manifest_path, manifest)

    executed = 0
    for case in cases:
        case_dir = args.output_dir / "cases" / case.case_id
        case_dir.mkdir(parents=True, exist_ok=True)
        snapshot = snapshots[case.case_id]
        _write_json(case_dir / "case_manifest.json", _case_manifest(case, args.time_limit, args.threads, snapshot))
        for solver in args.solvers:
            if _successful_attempt(case_dir / solver, solver, case.seed) is not None:
                continue
            if args.max_runs is not None and executed >= args.max_runs:
                _refresh_campaign_outputs(args.output_dir, manifest)
                return
            read_instance_snapshot(snapshot.path, expected_sha256=snapshot.sha256)
            attempt_dir = _new_attempt_dir(case_dir / solver)
            command = _solver_command(
                case,
                solver,
                attempt_dir,
                args.time_limit,
                args.threads,
                args.gurobi_python_path,
                snapshot.path,
                snapshot.sha256,
            )
            print(f"START {case.case_id} {solver} {attempt_dir.name}", flush=True)
            _run_monitored(command, attempt_dir, args.sample_period, case, solver, snapshot.sha256)
            print(f"END   {case.case_id} {solver} {attempt_dir.name}", flush=True)
            executed += 1
            _refresh_campaign_outputs(args.output_dir, manifest)
    _refresh_campaign_outputs(args.output_dir, manifest)


def _campaign_manifest(
    cases: tuple[CampaignCase, ...],
    solvers: tuple[str, ...],
    time_limit: float,
    threads: int,
    gurobi_python_path: Path,
    snapshots: dict[str, FrozenCaseSnapshot],
) -> dict[str, Any]:
    weights = ObjectiveWeights(0.4, 0.3, 0.3)
    return {
        "schema_version": 1,
        "case_count": len(cases),
        "solver_run_count": len(cases) * len(solvers),
        "case_order": [case.case_id for case in cases],
        "solvers": list(solvers),
        "time_limit_seconds_per_run": time_limit,
        "threads": threads,
        "objective_weights": asdict(weights),
        "gurobi_python_path": str(gurobi_python_path.resolve()),
        "python_executable": sys.executable,
        "cases": [_case_manifest(case, time_limit, threads, snapshots[case.case_id]) for case in cases],
    }


def _case_manifest(
    case: CampaignCase,
    time_limit: float,
    threads: int,
    snapshot: FrozenCaseSnapshot,
) -> dict[str, Any]:
    config = _expected_instance_config(case)
    defaults = case_defaults(**case.dimensions)
    solver = SolverConfig(time_limit=time_limit, threads=threads)
    return {
        "case_id": case.case_id,
        "scale": case.scale,
        "distribution": case.distribution,
        "seed": case.seed,
        "dimensions": case.dimensions,
        "instance_config": asdict(config),
        "case_defaults": defaults,
        "bpc_solver_config": asdict(solver),
        "compact_time_limit": time_limit,
        "compact_threads": threads,
        "instance_snapshot_file": str(snapshot.path.resolve()),
        "instance_snapshot_sha256": snapshot.sha256,
        "requested_seed": snapshot.instance.requested_seed,
        "realized_seed": snapshot.instance.config.seed,
        "generation_attempt": snapshot.instance.generation_attempt,
        "generation_feasibility_time": snapshot.instance.generation_feasibility_time,
        "truck_arc_count": len(snapshot.instance.truck_arcs),
        "drone_arc_count": len(snapshot.instance.drone_arcs),
    }


def _solver_command(
    case: CampaignCase,
    solver: str,
    output_dir: Path,
    time_limit: float,
    threads: int,
    gurobi_python_path: Path,
    instance_file: Path,
    instance_sha256: str,
) -> list[str]:
    dimensions = case.dimensions
    common = [
        "--output-dir", str(output_dir),
        "--seed", str(case.seed),
        "--num-trucks", str(dimensions["num_trucks"]),
        "--num-customers", str(dimensions["num_customers"]),
        "--num-hubs", str(dimensions["num_hubs"]),
        "--distribution", case.distribution,
        "--drones-per-truck", str(dimensions["drones_per_truck"]),
        "--weights", "0.4", "0.3", "0.3",
        "--threads", str(threads),
        "--gurobi-python-path", str(gurobi_python_path),
        "--instance-file", str(instance_file),
        "--instance-sha256", instance_sha256,
    ]
    if solver == "bpc":
        return [
            sys.executable, "-m", "thvrpd.solve", *common,
            "--time-limit", str(time_limit),
            "--logging-mode", "audit",
            "--progress-snapshot-period", "1",
        ]
    if solver == "compact":
        return [sys.executable, "-m", "thvrpd.compact_benchmark", *common, "--time-limit", str(time_limit)]
    raise ValueError(f"unknown campaign solver: {solver}")


def _new_attempt_dir(solver_dir: Path) -> Path:
    solver_dir.mkdir(parents=True, exist_ok=True)
    indices = [int(path.name.split("_")[1]) for path in solver_dir.glob("attempt_*") if path.is_dir()]
    attempt = solver_dir / f"attempt_{max(indices, default=0) + 1:03d}"
    attempt.mkdir()
    return attempt


def _expected_result(attempt_dir: Path, solver: str, seed: int) -> Path:
    return attempt_dir / (f"thvrpd-seed={seed}.json" if solver == "bpc" else f"compact-arc-seed={seed}.json")


def _successful_attempt(solver_dir: Path, solver: str, seed: int) -> Path | None:
    if not solver_dir.exists():
        return None
    for attempt in sorted(solver_dir.glob("attempt_*"), reverse=True):
        status_path = attempt / "run_status.json"
        if not status_path.exists():
            continue
        status = json.loads(status_path.read_text(encoding="utf-8"))
        if status["completed"] and _expected_result(attempt, solver, seed).exists():
            return attempt
    return None


def _run_monitored(
    command: list[str],
    attempt_dir: Path,
    sample_period: float,
    case: CampaignCase,
    solver: str,
    instance_sha256: str,
) -> None:
    _write_json(attempt_dir / "command.json", {"argv": command, "cwd": str(Path.cwd())})
    stdout_path = attempt_dir / "runner_stdout.log"
    stderr_path = attempt_dir / "runner_stderr.log"
    samples_path = attempt_dir / "resource_samples.jsonl"
    started_wall = time.time()
    started_iso = time.strftime("%Y-%m-%d %H:%M:%S")
    seen_cpu: dict[tuple[int, float], float] = {}
    peak_rss = 0
    peak_processes = 0
    peak_core_equivalent = 0.0
    last_sample_wall = started_wall
    last_total_cpu = 0.0
    with stdout_path.open("w", encoding="utf-8") as stdout, stderr_path.open("w", encoding="utf-8") as stderr, samples_path.open("w", encoding="utf-8") as samples:
        process = subprocess.Popen(command, cwd=Path.cwd(), stdout=stdout, stderr=stderr)
        root = psutil.Process(process.pid)
        while process.poll() is None:
            now = time.time()
            processes = [root, *root.children(recursive=True)]
            rss = 0
            for child in processes:
                try:
                    key = (child.pid, child.create_time())
                    cpu_times = child.cpu_times()
                    seen_cpu[key] = max(seen_cpu.get(key, 0.0), cpu_times.user + cpu_times.system)
                    rss += child.memory_info().rss
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue
            total_cpu = sum(seen_cpu.values())
            interval = max(now - last_sample_wall, 1e-9)
            interval_cores = max((total_cpu - last_total_cpu) / interval, 0.0)
            peak_core_equivalent = max(peak_core_equivalent, interval_cores)
            peak_rss = max(peak_rss, rss)
            peak_processes = max(peak_processes, len(processes))
            samples.write(json.dumps({
                "elapsed_seconds": now - started_wall,
                "process_count": len(processes),
                "aggregate_cpu_seconds": total_cpu,
                "interval_core_equivalent": interval_cores,
                "rss_bytes": rss,
            }, separators=(",", ":")) + "\n")
            samples.flush()
            last_sample_wall = now
            last_total_cpu = total_cpu
            time.sleep(sample_period)
        exit_code = process.wait()
    ended_wall = time.time()
    wall_runtime = ended_wall - started_wall
    total_cpu = sum(seen_cpu.values())
    logical_cpus = os.cpu_count() or 1
    telemetry = {
        "sample_period_seconds": sample_period,
        "wall_runtime_seconds": wall_runtime,
        "aggregate_process_tree_cpu_seconds": total_cpu,
        "mean_core_equivalent": total_cpu / wall_runtime,
        "mean_cpu_percent_of_logical_machine": 100.0 * total_cpu / wall_runtime / logical_cpus,
        "peak_interval_core_equivalent": peak_core_equivalent,
        "peak_process_count": peak_processes,
        "peak_rss_bytes": peak_rss,
        "logical_cpu_count": logical_cpus,
        "samples_file": str(samples_path),
    }
    _write_json(attempt_dir / "resource_usage.json", telemetry)
    result_path = _expected_result(attempt_dir, solver, case.seed)
    result_snapshot_verified = False
    if exit_code == 0 and result_path.exists():
        result_payload = json.loads(result_path.read_text(encoding="utf-8"))
        result_snapshot_verified = result_payload.get("instance_snapshot_sha256") == instance_sha256
    _write_json(attempt_dir / "run_status.json", {
        "case_id": case.case_id,
        "solver": solver,
        "attempt": attempt_dir.name,
        "started_at": started_iso,
        "ended_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "exit_code": exit_code,
        "completed": exit_code == 0 and result_path.exists() and result_snapshot_verified,
        "instance_snapshot_sha256": instance_sha256,
        "instance_snapshot_verified": result_snapshot_verified,
        "result_file": str(result_path),
        "stdout_file": str(stdout_path),
        "stderr_file": str(stderr_path),
        "resource_usage_file": str(attempt_dir / "resource_usage.json"),
    })


def _refresh_campaign_outputs(output_dir: Path, manifest: dict[str, Any]) -> None:
    rows: list[dict[str, Any]] = []
    completed = 0
    failed_attempts = 0
    for case_record in manifest["cases"]:
        case_id = case_record["case_id"]
        case_dir = output_dir / "cases" / case_id
        for solver in manifest["solvers"]:
            attempt = _successful_attempt(case_dir / solver, solver, int(case_record["seed"]))
            if attempt is None:
                failed_attempts += sum(
                    1
                    for path in (case_dir / solver).glob("attempt_*/run_status.json")
                    if not json.loads(path.read_text(encoding="utf-8"))["completed"]
                ) if (case_dir / solver).exists() else 0
                continue
            completed += 1
            rows.append(_summary_row(case_record, solver, attempt))
    expected = int(manifest["solver_run_count"])
    _write_json(output_dir / "campaign_status.json", {
        "expected_runs": expected,
        "completed_runs": completed,
        "pending_runs": expected - completed,
        "failed_attempts": failed_attempts,
        "updated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
    })
    _write_json(output_dir / "campaign_summary.json", rows)
    if rows:
        with (output_dir / "campaign_summary.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)


def _summary_row(case: dict[str, Any], solver: str, attempt: Path) -> dict[str, Any]:
    seed = int(case["seed"])
    result_path = _expected_result(attempt, solver, seed)
    result = json.loads(result_path.read_text(encoding="utf-8"))
    telemetry = json.loads((attempt / "resource_usage.json").read_text(encoding="utf-8"))
    row: dict[str, Any] = {
        "case_id": case["case_id"],
        "scale": case["scale"],
        "distribution": case["distribution"],
        "seed": seed,
        "requested_seed": case["requested_seed"],
        "realized_seed": case["realized_seed"],
        "generation_attempt": case["generation_attempt"],
        "generation_feasibility_time": case["generation_feasibility_time"],
        "instance_snapshot_file": case["instance_snapshot_file"],
        "instance_snapshot_sha256": case["instance_snapshot_sha256"],
        "truck_arc_count": case["truck_arc_count"],
        "drone_arc_count": case["drone_arc_count"],
        "solver": solver,
        "attempt": attempt.name,
        "status": result.get("status"),
        "objective_full": result.get("objective_full"),
        "lower_bound_full": result.get("lower_bound_full", result.get("objective_bound_full")),
        "gap": result.get("gap_full", result.get("mip_gap")),
        "runtime_seconds": result.get("runtime"),
        "nodes": result.get("nodes_processed", result.get("node_count")),
        "mean_core_equivalent": telemetry["mean_core_equivalent"],
        "mean_cpu_percent_of_logical_machine": telemetry["mean_cpu_percent_of_logical_machine"],
        "peak_rss_bytes": telemetry["peak_rss_bytes"],
        "result_file": str(result_path),
        "run_status_file": str(attempt / "run_status.json"),
    }
    if result.get("instance_snapshot_sha256") != case["instance_snapshot_sha256"]:
        raise ValueError(f"solver result snapshot mismatch for {case['case_id']} {solver}")
    if solver == "bpc":
        stats = result["bpc_stats"]
        row.update({
            "root_compact_objective_full": stats.get("root_compact_objective_full"),
            "root_compact_bound_full": stats.get("root_compact_bound_full"),
            "root_lower_bound_full": stats.get("root_lower_bound_full"),
            "root_closed": stats.get("root_closed"),
            "root_closure_time": stats.get("root_closure_time"),
            "root_rmp_is_integer": stats.get("root_rmp_is_integer"),
            "root_fractional_variable_count": stats.get("root_fractional_variable_count"),
            "root_nonzero_variable_count": stats.get("root_nonzero_variable_count"),
            "root_max_integrality_violation": stats.get("root_max_integrality_violation"),
            "root_incumbent_at_classification_full": stats.get("root_incumbent_at_classification_full"),
            "root_incumbent_at_fathom_full": stats.get("root_incumbent_at_fathom_full"),
            "root_fathom_reason": stats.get("root_fathom_reason"),
            "root_branch_required": stats.get("root_branch_required"),
            "sr_cuts_added": stats.get("sr_cuts_added"),
            "columns_added_standard": stats.get("columns_added_standard"),
            "columns_added_farkas": stats.get("columns_added_farkas"),
            "global_pool_routes": stats.get("global_pool_routes"),
            "pricing_labels_generated": stats.get("pricing_labels_generated"),
            "pricing_process_cpu_time": stats.get("pricing_process_cpu_time"),
            "branching_nodes": stats.get("branching_nodes"),
            "child_nodes_created": stats.get("child_nodes_created"),
            "open_nodes_at_termination": stats.get("open_nodes_at_termination"),
            "drone_sorties": stats.get("final_selected_drone_sorties"),
            "tree_history_file": str(attempt / "bpc_progress.jsonl"),
            "gurobi_log_file": str(attempt / "gurobi_logs" / "root_compact.log"),
        })
    else:
        row.update({
            "root_compact_objective_full": None,
            "root_compact_bound_full": None,
            "root_lower_bound_full": None,
            "root_closed": None,
            "root_closure_time": None,
            "root_rmp_is_integer": None,
            "root_fractional_variable_count": None,
            "root_nonzero_variable_count": None,
            "root_max_integrality_violation": None,
            "root_incumbent_at_classification_full": None,
            "root_incumbent_at_fathom_full": None,
            "root_fathom_reason": None,
            "root_branch_required": None,
            "sr_cuts_added": None,
            "columns_added_standard": None,
            "columns_added_farkas": None,
            "global_pool_routes": None,
            "pricing_labels_generated": None,
            "pricing_process_cpu_time": None,
            "branching_nodes": None,
            "child_nodes_created": None,
            "open_nodes_at_termination": None,
            "drone_sorties": result.get("service_metrics", {}).get("drone_sorties"),
            "tree_history_file": None,
            "gurobi_log_file": result.get("gurobi_log_file"),
        })
    return row


def _write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
