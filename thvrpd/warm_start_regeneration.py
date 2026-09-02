from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import shutil
import sys
import time

from .instance import InstanceAcceptanceResult


TARGET_SEEDS = (6, 8)
TARGET_CASE_IDS = tuple(f"large_PC_seed{seed}" for seed in TARGET_SEEDS)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--campaign-dir", type=Path, required=True)
    parser.add_argument("--gurobi-python-path", type=Path, required=True)
    parser.add_argument("--root-compact-limit", type=float, default=60.0)
    parser.add_argument("--first-incumbent-deadline", type=float, default=45.0)
    parser.add_argument("--screening-repetitions", type=int, default=2)
    parser.add_argument("--time-limit", type=float, default=1800.0)
    parser.add_argument("--threads", type=int, default=0)
    parser.add_argument("--truck-arc-probability", type=float, default=0.10)
    parser.add_argument("--hub-arc-probability", type=float, default=0.30)
    return parser


def main() -> None:
    args = _parser().parse_args()
    if (
        min(args.root_compact_limit, args.first_incumbent_deadline, args.time_limit) <= 0.0
        or args.first_incumbent_deadline >= args.root_compact_limit
        or args.screening_repetitions <= 0
        or args.threads < 0
    ):
        raise ValueError("time limits must be positive and threads nonnegative")
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
        _refresh_outputs,
        _validate_dense_instance,
        _write_aggregate_report,
        _write_case_report,
    )
    from .experiments import SCALES
    from .instance import (
        generate_instance,
        instance_physical_fingerprint,
        read_instance_snapshot,
        write_instance_snapshot,
    )
    from .transform import build_transformed_graph

    campaign_dir = args.campaign_dir.resolve()
    manifest_path = campaign_dir / "campaign_manifest.json"
    manifest = _read_json(manifest_path)
    _validate_campaign(manifest, args)
    cases = tuple(
        CampaignCase(record["scale"], record["distribution"], record["seed"], record["dimensions"])
        for record in manifest["cases"]
    )
    target_cases = tuple(case for case in cases if case.case_id in TARGET_CASE_IDS)
    if tuple(case.case_id for case in target_cases) != TARGET_CASE_IDS:
        raise RuntimeError(f"campaign does not contain targets in expected order: {TARGET_CASE_IDS}")

    screening_root = campaign_dir / "warm_start_screening"
    staging_root = screening_root / "staging"
    staging_root.mkdir(parents=True, exist_ok=True)
    weights = ObjectiveWeights(**manifest["objective_weights"])
    staged = {}
    for case in target_cases:
        stage_dir = staging_root / case.case_id
        stage_dir.mkdir(parents=True, exist_ok=True)
        snapshot_path = stage_dir / "accepted_snapshot.json"
        metadata_path = stage_dir / "accepted_screening.json"
        if snapshot_path.exists() or metadata_path.exists():
            if not snapshot_path.exists() or not metadata_path.exists():
                raise RuntimeError(f"incomplete staged replacement for {case.case_id}")
            metadata = _read_json(metadata_path)
            instance, digest = read_instance_snapshot(
                snapshot_path,
                expected_sha256=metadata["instance_snapshot_sha256"],
            )
        else:
            current_record = next(record for record in manifest["cases"] if record["case_id"] == case.case_id)
            current_instance, _ = read_instance_snapshot(
                Path(current_record["instance_snapshot_file"]),
                expected_sha256=current_record["instance_snapshot_sha256"],
            )
            start_attempt = (
                current_instance.generation_attempt + 1
                if current_record.get("warm_start_certification") is not None
                else 0
            )
            prior_time = current_instance.generation_feasibility_time if start_attempt else 0.0
            prior_diagnostics = (
                current_instance.generation_feasibility_diagnostics if start_attempt else tuple()
            )
            config = InstanceConfig(
                seed=case.seed,
                distribution=case.distribution,
                truck_arc_probability=args.truck_arc_probability,
                hub_arc_probability=args.hub_arc_probability,
                **case.dimensions,
            )

            def acceptance(instance, attempt, realized_seed, *, case_id=case.case_id):
                return screen_warm_start_candidate(
                    instance,
                    attempt,
                    realized_seed,
                    screening_root / case_id / f"candidate_{attempt:03d}",
                    weights,
                    args.root_compact_limit,
                    args.first_incumbent_deadline,
                    args.screening_repetitions,
                    args.threads,
                )

            generation_start = time.time()
            instance = generate_instance(
                config,
                post_feasibility_acceptance=acceptance,
                start_attempt=start_attempt,
                prior_feasibility_time=prior_time,
                prior_feasibility_diagnostics=prior_diagnostics,
            )
            generation_wall = time.time() - generation_start
            _validate_dense_instance(instance, case, args)
            digest = write_instance_snapshot(instance, snapshot_path)
            accepted_record = instance.generation_feasibility_diagnostics[-1]
            certification = accepted_record["post_feasibility_acceptance"]
            if not certification["accepted"]:
                raise RuntimeError(f"accepted snapshot lacks warm-start certification for {case.case_id}")
            metadata = {
                "case_id": case.case_id,
                "requested_seed": case.seed,
                "realized_seed": instance.config.seed,
                "generation_attempt": instance.generation_attempt,
                "generation_wall_time": generation_wall,
                "instance_snapshot_file": str(snapshot_path),
                "instance_snapshot_sha256": digest,
                "instance_physical_fingerprint": instance_physical_fingerprint(instance),
                "warm_start_certification": certification,
            }
            _write_json(metadata_path, metadata)
        _validate_dense_instance(instance, case, args)
        if not metadata["warm_start_certification"]["accepted"]:
            raise RuntimeError(f"staged snapshot is not warm-start certified: {case.case_id}")
        staged[case.case_id] = (instance, digest, snapshot_path, metadata)
        print(
            f"STAGED {case.case_id} realized_seed={instance.config.seed} "
            f"attempt={instance.generation_attempt} sha256={digest}",
            flush=True,
        )

    preserved_hashes = _preserved_artifact_hashes(campaign_dir)
    provenance_path = campaign_dir / "replacement_provenance.json"
    if not provenance_path.exists():
        _write_json(
            provenance_path,
            {
                "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                "selection_disclosure": (
                    "large_PC_seed6 and large_PC_seed8 were deterministically filtered until two "
                    "unseeded 60-second compact screens both returned validated incumbents within 45 seconds"
                ),
                "old_cases": _old_case_provenance(campaign_dir, manifest),
                "preserved_artifact_hashes": preserved_hashes,
            },
        )
    else:
        provenance = _read_json(provenance_path)
        if provenance["preserved_artifact_hashes"] != preserved_hashes:
            raise RuntimeError("non-target campaign artifacts changed since replacement staging")
        provenance["selection_disclosure"] = (
            "large_PC_seed6 and large_PC_seed8 were deterministically filtered until two "
            "unseeded 60-second compact screens both returned validated incumbents within 45 seconds"
        )
        _write_json(provenance_path, provenance)

    manifest = _install_replacements(
        campaign_dir,
        manifest,
        staged,
        args,
        build_transformed_graph,
        instance_physical_fingerprint,
    )
    _refresh_outputs(campaign_dir, manifest, cases, _successful_attempt, _summary_row)

    case_records = {record["case_id"]: record for record in manifest["cases"]}
    for case in target_cases:
        case_record = case_records[case.case_id]
        for solver in ("bpc", "compact"):
            solver_dir = campaign_dir / "cases" / case.case_id / solver
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
                _refresh_outputs(campaign_dir, manifest, cases, _successful_attempt, _summary_row)
                raise RuntimeError(f"{case.case_id} {solver} failed: {run_status}")
            if solver == "bpc":
                _validate_actual_bpc_warm_start(Path(run_status["result_file"]))
            print(f"END   {case.case_id} {solver} {attempt_dir.name}", flush=True)
            _refresh_outputs(campaign_dir, manifest, cases, _successful_attempt, _summary_row)
        _write_case_report(campaign_dir, case.case_id)

    _refresh_outputs(campaign_dir, manifest, cases, _successful_attempt, _summary_row)
    _write_aggregate_report(campaign_dir, manifest)
    _finalize_provenance(campaign_dir, manifest, preserved_hashes)


def screen_warm_start_candidate(
    instance,
    attempt: int,
    realized_seed: int,
    candidate_dir: Path,
    weights,
    root_compact_limit: float,
    first_incumbent_deadline: float,
    screening_repetitions: int,
    threads: int,
) -> InstanceAcceptanceResult:
    from gurobipy import gurobi

    from .compact import solve_compact_solution
    from .instance import instance_physical_fingerprint, read_instance_snapshot, write_instance_snapshot
    from .objective import build_objective_data
    from .routes import validate_route_cover
    from .transform import build_transformed_graph

    candidate_dir.mkdir(parents=True, exist_ok=True)
    snapshot_path = candidate_dir / "candidate_snapshot.json"
    result_path = candidate_dir / "screening_result.json"
    if snapshot_path.exists():
        recorded, digest = read_instance_snapshot(snapshot_path)
        if instance_physical_fingerprint(recorded) != instance_physical_fingerprint(instance):
            raise RuntimeError(f"screening candidate fingerprint changed at attempt {attempt}")
    else:
        digest = write_instance_snapshot(instance, snapshot_path)
    if result_path.exists():
        record = _read_json(result_path)
        if record["instance_snapshot_sha256"] != digest:
            raise RuntimeError(f"screening result hash mismatch at attempt {attempt}")
        return InstanceAcceptanceResult(
            accepted=record["accepted"],
            status=record["acceptance_status"],
            diagnostics=record,
        )

    objective = build_objective_data(instance, weights)
    graph = build_transformed_graph(instance)
    runs = []
    for repetition in range(1, screening_repetitions + 1):
        solution = solve_compact_solution(
            instance,
            weights,
            time_limit=root_compact_limit,
            threads=threads,
            require_optimal=False,
            log_file=str(candidate_dir / f"root_compact_screen_run_{repetition:02d}.log"),
            objective=objective,
        )
        routes = tuple()
        validation = {"incumbent_present": False, "valid": None, "route_count": 0}
        if solution.route_paths:
            routes = validate_route_cover(solution.route_paths, graph, objective)
            validation = {
                "incumbent_present": True,
                "valid": True,
                "route_count": len(routes),
                "routes": [route.to_record(objective) for route in routes],
            }
        repetition_accepted = (
            bool(solution.route_paths)
            and solution.first_incumbent_time is not None
            and solution.first_incumbent_time <= first_incumbent_deadline
        )
        runs.append(
            {
                "repetition": repetition,
                "accepted": repetition_accepted,
                "status": solution.status,
                "status_code": solution.status_code,
                "objective": solution.objective_full,
                "bound": solution.objective_bound_full,
                "gap": solution.mip_gap,
                "node_count": solution.node_count,
                "iteration_count": solution.iteration_count,
                "first_incumbent_time": solution.first_incumbent_time,
                "timing": asdict(solution.timing),
                "route_paths": [list(path) for path in solution.route_paths],
                "validation": validation,
            }
        )
        if not repetition_accepted:
            break
    accepted = len(runs) == screening_repetitions and all(run["accepted"] for run in runs)
    acceptance_status = (
        "warm_start_certified"
        if accepted
        else (
            "warm_start_rejected_late_incumbent"
            if any(run["route_paths"] for run in runs)
            else "warm_start_rejected_no_incumbent"
        )
    )
    last_run = runs[-1]
    record = {
        "attempt": attempt,
        "realized_seed": realized_seed,
        "accepted": accepted,
        "acceptance_status": acceptance_status,
        "instance_snapshot_file": str(snapshot_path),
        "instance_snapshot_sha256": digest,
        "instance_physical_fingerprint": instance_physical_fingerprint(instance),
        "gurobi_version": ".".join(map(str, gurobi.version())),
        "root_compact_limit_seconds": root_compact_limit,
        "first_incumbent_deadline_seconds": first_incumbent_deadline,
        "screening_repetitions_required": screening_repetitions,
        "screening_repetitions_completed": len(runs),
        "threads": threads,
        "status": last_run["status"],
        "status_code": last_run["status_code"],
        "objective": last_run["objective"],
        "bound": last_run["bound"],
        "gap": last_run["gap"],
        "node_count": last_run["node_count"],
        "iteration_count": last_run["iteration_count"],
        "first_incumbent_time": max(
            run["first_incumbent_time"] for run in runs if run["first_incumbent_time"] is not None
        ) if any(run["first_incumbent_time"] is not None for run in runs) else None,
        "timing": last_run["timing"],
        "route_paths": last_run["route_paths"],
        "validation": last_run["validation"],
        "runs": runs,
    }
    _write_json(result_path, record)
    return InstanceAcceptanceResult(accepted=accepted, status=acceptance_status, diagnostics=record)


def _validate_campaign(manifest, args) -> None:
    if manifest.get("case_order") != [
        "large_PS_seed2",
        "large_PS_seed3",
        "large_PS_seed4",
        "large_PC_seed5",
        "large_PC_seed6",
        "large_PC_seed7",
        "large_PC_seed8",
    ]:
        raise ValueError("unexpected dense campaign case order")
    if manifest.get("time_limit_seconds_per_run") != args.time_limit:
        raise ValueError("campaign time limit mismatch")
    if manifest.get("threads") != args.threads:
        raise ValueError("campaign thread count mismatch")
    if manifest.get("truck_arc_probability") != args.truck_arc_probability:
        raise ValueError("campaign truck probability mismatch")
    if manifest.get("hub_arc_probability") != args.hub_arc_probability:
        raise ValueError("campaign hub probability mismatch")


def _install_replacements(
    campaign_dir: Path,
    manifest,
    staged,
    args,
    build_transformed_graph,
    instance_physical_fingerprint,
):
    current = {record["case_id"]: record for record in manifest["cases"]}
    already_installed = all(
        current[case_id]["instance_snapshot_sha256"] == staged[case_id][1]
        for case_id in TARGET_CASE_IDS
    )
    if already_installed:
        return manifest

    for case_id in TARGET_CASE_IDS:
        case_dir = campaign_dir / "cases" / case_id
        if case_dir.exists():
            shutil.rmtree(case_dir)
        for suffix in ("_solution_process_report.json", "_solution_process_report.md"):
            report_path = campaign_dir / f"{case_id}{suffix}"
            if report_path.exists():
                report_path.unlink()
        official_snapshot = campaign_dir / "instances" / f"{case_id}.json"
        if official_snapshot.exists():
            official_snapshot.unlink()
        shutil.copy2(staged[case_id][2], official_snapshot)

    replacement_records = {}
    for case_id in TARGET_CASE_IDS:
        instance, digest, _, metadata = staged[case_id]
        old = current[case_id]
        transformed = build_transformed_graph(instance)
        replacement_records[case_id] = {
            **old,
            "instance_config": asdict(instance.config),
            "requested_seed": instance.requested_seed,
            "realized_seed": instance.config.seed,
            "generation_attempt": instance.generation_attempt,
            "generation_feasibility_time": instance.generation_feasibility_time,
            "generation_wall_time": metadata["generation_wall_time"],
            "instance_snapshot_file": str(campaign_dir / "instances" / f"{case_id}.json"),
            "instance_snapshot_sha256": digest,
            "instance_physical_fingerprint": instance_physical_fingerprint(instance),
            "truck_arc_count": len(instance.truck_arcs),
            "drone_arc_count": len(instance.drone_arcs),
            "transformed_node_count": len(transformed.nodes),
            "transformed_arc_count": len(transformed.arcs),
            "warm_start_certification": metadata["warm_start_certification"],
        }
    manifest = {
        **manifest,
        "schema_version": 2,
        "warm_start_filtered_cases": list(TARGET_CASE_IDS),
        "warm_start_filter_seconds": args.root_compact_limit,
        "warm_start_first_incumbent_deadline_seconds": args.first_incumbent_deadline,
        "warm_start_screening_repetitions": args.screening_repetitions,
        "cases": [replacement_records.get(record["case_id"], record) for record in manifest["cases"]],
    }
    _write_json(campaign_dir / "campaign_manifest.json", manifest)
    for name in (
        "dense_campaign_comparison.json",
        "dense_campaign_comparison.csv",
        "dense_campaign_comparison.md",
        "artifact_inventory.csv",
    ):
        path = campaign_dir / name
        if path.exists():
            path.unlink()
    return manifest


def _validate_actual_bpc_warm_start(result_path: Path) -> None:
    result = _read_json(result_path)
    stats = result.get("bpc_stats", {})
    if not stats.get("root_compact_attempted"):
        raise RuntimeError("BPC did not attempt the certified compact warm start")
    if not stats.get("root_compact_accepted_columns"):
        raise RuntimeError("BPC compact warm start did not return accepted route columns")
    if not stats.get("root_compact_incumbent_validated"):
        raise RuntimeError("BPC compact warm-start incumbent did not pass route-cover validation")
    if stats.get("root_compact_objective_full") is None:
        raise RuntimeError("BPC compact warm start did not return an incumbent objective")


def _old_case_provenance(campaign_dir: Path, manifest) -> list[dict[str, object]]:
    summary = _read_json(campaign_dir / "campaign_summary.json")
    records = []
    for case in manifest["cases"]:
        if case["case_id"] not in TARGET_CASE_IDS:
            continue
        report_path = campaign_dir / f"{case['case_id']}_solution_process_report.json"
        report = _read_json(report_path)
        records.append(
            {
                "case_id": case["case_id"],
                "snapshot_sha256": case["instance_snapshot_sha256"],
                "instance_physical_fingerprint": case["instance_physical_fingerprint"],
                "manifest_record": case,
                "summary_rows": [row for row in summary if row["case_id"] == case["case_id"]],
                "comparison": report["comparison"],
            }
        )
    return records


def _preserved_artifact_hashes(campaign_dir: Path) -> dict[str, str]:
    paths = []
    for path in (campaign_dir / "instances").glob("*.json"):
        if path.stem not in TARGET_CASE_IDS:
            paths.append(path)
    for case_dir in (campaign_dir / "cases").iterdir():
        if case_dir.is_dir() and case_dir.name not in TARGET_CASE_IDS:
            paths.extend(path for path in case_dir.rglob("*") if path.is_file())
    for path in campaign_dir.glob("*_solution_process_report.*"):
        if not any(path.name.startswith(case_id) for case_id in TARGET_CASE_IDS):
            paths.append(path)
    return {
        str(path.relative_to(campaign_dir)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted(paths)
    }


def _finalize_provenance(campaign_dir: Path, manifest, expected_preserved_hashes) -> None:
    actual = _preserved_artifact_hashes(campaign_dir)
    if actual != expected_preserved_hashes:
        raise RuntimeError("non-target campaign artifacts changed during replacement")
    provenance_path = campaign_dir / "replacement_provenance.json"
    provenance = _read_json(provenance_path)
    provenance["completed_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
    provenance["new_cases"] = [
        record for record in manifest["cases"] if record["case_id"] in TARGET_CASE_IDS
    ]
    provenance["preserved_artifacts_verified"] = True
    _write_json(provenance_path, provenance)
    from .dense_large_campaign import _artifact_inventory, _write_csv

    _write_csv(campaign_dir / "artifact_inventory.csv", _artifact_inventory(campaign_dir))


def _read_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def _write_json(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, allow_nan=False), encoding="utf-8")


if __name__ == "__main__":
    main()
