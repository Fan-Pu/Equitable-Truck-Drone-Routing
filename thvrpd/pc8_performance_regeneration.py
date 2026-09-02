from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path
import shutil
import sys
import time

from .instance import InstanceAcceptanceResult


CASE_ID = "large_PC_seed8"
REQUESTED_SEED = 8


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--campaign-dir", type=Path, required=True)
    parser.add_argument("--gurobi-python-path", type=Path, required=True)
    parser.add_argument("--time-limit", type=float, default=1800.0)
    parser.add_argument("--threads", type=int, default=0)
    parser.add_argument("--min-transformed-arcs", type=int, default=350)
    parser.add_argument("--max-transformed-arcs", type=int, default=450)
    parser.add_argument("--max-bpc-gap", type=float, default=0.05)
    parser.add_argument("--root-compact-limit", type=float, default=60.0)
    parser.add_argument("--first-incumbent-deadline", type=float, default=45.0)
    parser.add_argument("--screening-repetitions", type=int, default=2)
    parser.add_argument("--truck-arc-probability", type=float, default=0.10)
    parser.add_argument("--hub-arc-probability", type=float, default=0.30)
    return parser


def main() -> None:
    args = _parser().parse_args()
    if (
        min(args.time_limit, args.root_compact_limit, args.first_incumbent_deadline) <= 0.0
        or args.first_incumbent_deadline >= args.root_compact_limit
        or args.screening_repetitions <= 0
        or args.threads < 0
        or args.min_transformed_arcs > args.max_transformed_arcs
        or not 0.0 <= args.max_bpc_gap <= 1.0
    ):
        raise ValueError("invalid PC8 performance-selection limits")
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
    case = CampaignCase("large", "PC", REQUESTED_SEED, dict(SCALES["large"]))
    current_record = next(record for record in manifest["cases"] if record["case_id"] == CASE_ID)
    current_instance, _ = read_instance_snapshot(
        Path(current_record["instance_snapshot_file"]),
        expected_sha256=current_record["instance_snapshot_sha256"],
    )
    selection_root = campaign_dir / "pc8_performance_screening"
    staging_dir = selection_root / "staging"
    staging_dir.mkdir(parents=True, exist_ok=True)
    stage_snapshot = staging_dir / "accepted_snapshot.json"
    stage_metadata = staging_dir / "accepted_selection.json"
    weights = ObjectiveWeights(**manifest["objective_weights"])

    if stage_snapshot.exists() or stage_metadata.exists():
        if not stage_snapshot.exists() or not stage_metadata.exists():
            raise RuntimeError("incomplete staged PC8 performance replacement")
        metadata = _read_json(stage_metadata)
        instance, digest = read_instance_snapshot(
            stage_snapshot,
            expected_sha256=metadata["instance_snapshot_sha256"],
        )
    else:
        config = InstanceConfig(
            seed=REQUESTED_SEED,
            distribution="PC",
            truck_arc_probability=args.truck_arc_probability,
            hub_arc_probability=args.hub_arc_probability,
            **case.dimensions,
        )

        def acceptance(instance, attempt, realized_seed):
            return evaluate_candidate(
                instance,
                attempt,
                realized_seed,
                selection_root / f"candidate_{attempt:03d}",
                case,
                weights,
                args,
            )

        generation_start = time.time()
        instance = generate_instance(
            config,
            post_feasibility_acceptance=acceptance,
            start_attempt=current_instance.generation_attempt + 1,
            prior_feasibility_time=current_instance.generation_feasibility_time,
            prior_feasibility_diagnostics=current_instance.generation_feasibility_diagnostics,
        )
        generation_wall = time.time() - generation_start
        _validate_dense_instance(instance, case, args)
        digest = write_instance_snapshot(instance, stage_snapshot)
        selection = instance.generation_feasibility_diagnostics[-1]["post_feasibility_acceptance"]
        if not selection["accepted"]:
            raise RuntimeError("staged PC8 snapshot did not pass performance selection")
        metadata = {
            "case_id": CASE_ID,
            "requested_seed": REQUESTED_SEED,
            "realized_seed": instance.config.seed,
            "generation_attempt": instance.generation_attempt,
            "generation_wall_time": generation_wall,
            "instance_snapshot_file": str(stage_snapshot),
            "instance_snapshot_sha256": digest,
            "instance_physical_fingerprint": instance_physical_fingerprint(instance),
            "performance_selection": selection["diagnostics"],
        }
        _write_json(stage_metadata, metadata)
    _validate_dense_instance(instance, case, args)
    if not metadata["performance_selection"]["accepted"]:
        raise RuntimeError("staged PC8 snapshot is not performance-qualified")
    print(
        f"STAGED {CASE_ID} realized_seed={instance.config.seed} attempt={instance.generation_attempt} "
        f"arcs={metadata['performance_selection']['transformed_arc_count']} sha256={digest}",
        flush=True,
    )

    preserved = _preserved_artifact_hashes(campaign_dir)
    provenance_path = campaign_dir / "pc8_performance_replacement_provenance.json"
    if not provenance_path.exists():
        current_report = _read_json(campaign_dir / f"{CASE_ID}_solution_process_report.json")
        _write_json(
            provenance_path,
            {
                "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                "selection_disclosure": _selection_disclosure(args),
                "old_case": {
                    "manifest_record": current_record,
                    "comparison": current_report["comparison"],
                },
                "preserved_artifact_hashes": preserved,
            },
        )
    else:
        provenance = _read_json(provenance_path)
        if provenance["preserved_artifact_hashes"] != preserved:
            raise RuntimeError("non-PC8 campaign artifacts changed before replacement")

    manifest = _install_replacement(
        campaign_dir,
        manifest,
        instance,
        digest,
        stage_snapshot,
        metadata,
        args,
        build_transformed_graph,
        instance_physical_fingerprint,
    )
    cases = tuple(
        CampaignCase(record["scale"], record["distribution"], record["seed"], record["dimensions"])
        for record in manifest["cases"]
    )
    _refresh_outputs(campaign_dir, manifest, cases, _successful_attempt, _summary_row)
    record = next(item for item in manifest["cases"] if item["case_id"] == CASE_ID)
    for solver in ("bpc", "compact"):
        solver_dir = campaign_dir / "cases" / CASE_ID / solver
        existing = _successful_attempt(solver_dir, solver, REQUESTED_SEED)
        if existing is not None:
            print(f"SKIP  {CASE_ID} {solver} {existing.name}", flush=True)
            continue
        attempt_dir = _new_attempt_dir(solver_dir)
        command = _candidate_solver_command(
            case,
            solver,
            attempt_dir,
            args,
            Path(record["instance_snapshot_file"]),
            record["instance_snapshot_sha256"],
            _solver_command,
        )
        print(f"START {CASE_ID} {solver} {attempt_dir.name}", flush=True)
        _run_monitored(command, attempt_dir, 1.0, case, solver, record["instance_snapshot_sha256"])
        run_status = _read_json(attempt_dir / "run_status.json")
        if not run_status["completed"]:
            raise RuntimeError(f"final PC8 {solver} run failed: {run_status}")
        print(f"END   {CASE_ID} {solver} {attempt_dir.name}", flush=True)
        _refresh_outputs(campaign_dir, manifest, cases, _successful_attempt, _summary_row)
    _write_case_report(campaign_dir, CASE_ID)
    _refresh_outputs(campaign_dir, manifest, cases, _successful_attempt, _summary_row)
    _write_aggregate_report(campaign_dir, manifest)
    _validate_final_performance(campaign_dir, args)
    write_selection_summary(selection_root)
    _finalize_provenance(campaign_dir, manifest, preserved)


def evaluate_candidate(
    instance,
    attempt: int,
    realized_seed: int,
    candidate_dir: Path,
    case,
    weights,
    args,
) -> InstanceAcceptanceResult:
    from .campaign import _run_monitored, _solver_command, _successful_attempt
    from .instance import instance_physical_fingerprint, read_instance_snapshot, write_instance_snapshot
    from .objective import build_objective_data
    from .routes import validate_route_cover
    from .transform import build_transformed_graph
    from .warm_start_regeneration import screen_warm_start_candidate

    candidate_dir.mkdir(parents=True, exist_ok=True)
    result_path = candidate_dir / "selection_result.json"
    candidate_snapshot = candidate_dir / "trial_snapshot.json"
    trial_instance = replace(instance, requested_seed=REQUESTED_SEED, generation_attempt=attempt)
    if candidate_snapshot.exists():
        recorded, digest = read_instance_snapshot(candidate_snapshot)
        if instance_physical_fingerprint(recorded) != instance_physical_fingerprint(trial_instance):
            raise RuntimeError(f"PC8 candidate fingerprint changed at attempt {attempt}")
    else:
        digest = write_instance_snapshot(trial_instance, candidate_snapshot)
    if result_path.exists():
        record = _read_json(result_path)
        return InstanceAcceptanceResult(record["accepted"], record["acceptance_status"], record)

    graph = build_transformed_graph(trial_instance)
    transformed_arcs = len(graph.arcs)
    base = {
        "attempt": attempt,
        "realized_seed": realized_seed,
        "candidate_snapshot_file": str(candidate_snapshot),
        "candidate_snapshot_sha256": digest,
        "instance_physical_fingerprint": instance_physical_fingerprint(trial_instance),
        "transformed_node_count": len(graph.nodes),
        "transformed_arc_count": transformed_arcs,
        "arc_range": [args.min_transformed_arcs, args.max_transformed_arcs],
        "max_bpc_gap": args.max_bpc_gap,
        "compact_required_status_code": 9,
    }
    if not args.min_transformed_arcs <= transformed_arcs <= args.max_transformed_arcs:
        return _finish_candidate(
            result_path,
            base,
            False,
            "rejected_transformed_arc_count",
            witness=f"{transformed_arcs} not in [{args.min_transformed_arcs}, {args.max_transformed_arcs}]",
        )

    warm = screen_warm_start_candidate(
        trial_instance,
        attempt,
        realized_seed,
        candidate_dir / "warm_start",
        weights,
        args.root_compact_limit,
        args.first_incumbent_deadline,
        args.screening_repetitions,
        args.threads,
    )
    base["warm_start"] = warm.diagnostics
    if not warm.accepted:
        return _finish_candidate(result_path, base, False, "rejected_warm_start", witness=warm.status)

    objective = build_objective_data(trial_instance, weights)
    trials = candidate_dir / "performance_trials"
    bpc_dir = trials / "bpc"
    bpc_attempt = _successful_attempt(bpc_dir, "bpc", REQUESTED_SEED)
    if bpc_attempt is None:
        bpc_attempt = trials / "bpc" / "attempt_001"
        bpc_attempt.mkdir(parents=True, exist_ok=True)
        command = _candidate_solver_command(
            case,
            "bpc",
            bpc_attempt,
            args,
            candidate_snapshot,
            digest,
            _solver_command,
        )
        print(f"TRIAL {CASE_ID} candidate_{attempt:03d} bpc", flush=True)
        _run_monitored(command, bpc_attempt, 1.0, case, "bpc", digest)
    bpc_status = _read_json(bpc_attempt / "run_status.json")
    if not bpc_status["completed"]:
        raise RuntimeError(f"candidate BPC infrastructure failure: {bpc_status}")
    bpc = _read_json(Path(bpc_status["result_file"]))
    bpc_validation = _validate_bpc_result(trial_instance, objective, graph, bpc, validate_route_cover)
    bpc_gap = bpc.get("gap_full")
    bpc_accepted = bpc_trial_qualifies(bpc, bpc_validation, args.max_bpc_gap)
    base["bpc_trial"] = _solver_summary(bpc_status, bpc, bpc_validation)
    if not bpc_accepted:
        return _finish_candidate(
            result_path,
            base,
            False,
            "rejected_bpc_gap",
            witness=f"BPC gap {bpc_gap} exceeds {args.max_bpc_gap} or incumbent is invalid",
        )

    compact_dir = trials / "compact"
    compact_attempt = _successful_attempt(compact_dir, "compact", REQUESTED_SEED)
    if compact_attempt is None:
        compact_attempt = trials / "compact" / "attempt_001"
        compact_attempt.mkdir(parents=True, exist_ok=True)
        command = _candidate_solver_command(
            case,
            "compact",
            compact_attempt,
            args,
            candidate_snapshot,
            digest,
            _solver_command,
        )
        print(f"TRIAL {CASE_ID} candidate_{attempt:03d} compact", flush=True)
        _run_monitored(command, compact_attempt, 1.0, case, "compact", digest)
    compact_status = _read_json(compact_attempt / "run_status.json")
    if not compact_status["completed"]:
        raise RuntimeError(f"candidate compact infrastructure failure: {compact_status}")
    compact = _read_json(Path(compact_status["result_file"]))
    compact_validation = _validate_compact_result(trial_instance, objective, graph, compact, validate_route_cover)
    base["compact_trial"] = _solver_summary(compact_status, compact, compact_validation)
    if not compact_trial_qualifies(compact):
        return _finish_candidate(
            result_path,
            base,
            False,
            "rejected_compact_proved_optimal",
            witness=f"compact status code {compact.get('status_code')} is not TIME_LIMIT",
        )
    if not cross_solver_bounds_consistent(bpc, compact):
        return _finish_candidate(
            result_path,
            base,
            False,
            "rejected_cross_solver_bound",
            witness=(
                f"BPC lower bound {bpc.get('lower_bound_full')} exceeds compact incumbent "
                f"{compact.get('objective_full')}"
            ),
        )
    return _finish_candidate(result_path, base, True, "performance_selection_certified")


def _finish_candidate(path, base, accepted, status, *, witness=None) -> InstanceAcceptanceResult:
    record = {**base, "accepted": accepted, "acceptance_status": status, "witness": witness}
    _write_json(path, record)
    return InstanceAcceptanceResult(accepted, status, record)


def _candidate_solver_command(case, solver, attempt, args, snapshot, digest, solver_command):
    command = solver_command(
        case,
        solver,
        attempt,
        args.time_limit,
        args.threads,
        args.gurobi_python_path,
        snapshot,
        digest,
    )
    command.extend(
        [
            "--truck-arc-probability",
            str(args.truck_arc_probability),
            "--hub-arc-probability",
            str(args.hub_arc_probability),
        ]
    )
    return command


def _validate_bpc_result(instance, objective, graph, result, validate_route_cover):
    paths = tuple(tuple(route["path"]) for route in result.get("routes", []))
    if not paths:
        return {"incumbent_present": False, "valid": None, "route_count": 0}
    routes = validate_route_cover(paths, graph, objective)
    return {"incumbent_present": True, "valid": True, "route_count": len(routes)}


def _validate_compact_result(instance, objective, graph, result, validate_route_cover):
    paths = tuple(tuple(path) for path in result.get("route_paths", []))
    if not paths:
        return {"incumbent_present": False, "valid": None, "route_count": 0}
    routes = validate_route_cover(paths, graph, objective)
    return {"incumbent_present": True, "valid": True, "route_count": len(routes)}


def _solver_summary(run_status, result, validation):
    return {
        "run_status": run_status,
        "status": result.get("status"),
        "status_code": result.get("status_code"),
        "runtime": result.get("runtime"),
        "objective": result.get("objective_full"),
        "bound": result.get("lower_bound_full", result.get("objective_bound_full")),
        "gap": result.get("gap_full", result.get("mip_gap")),
        "nodes": result.get("nodes_processed", result.get("node_count")),
        "validation": validation,
        "root_compact_accepted_columns": result.get("bpc_stats", {}).get("root_compact_accepted_columns"),
        "root_compact_incumbent_validated": result.get("bpc_stats", {}).get("root_compact_incumbent_validated"),
    }


def bpc_trial_qualifies(result, validation, max_gap: float) -> bool:
    gap = result.get("gap_full")
    return (
        validation.get("valid") is True
        and gap is not None
        and gap <= max_gap
        and result.get("bpc_stats", {}).get("root_compact_incumbent_validated") is True
    )


def compact_trial_qualifies(result) -> bool:
    return result.get("status_code") == 9


def cross_solver_bounds_consistent(bpc_result, compact_result, tolerance: float = 1e-7) -> bool:
    lower_bound = bpc_result.get("lower_bound_full")
    compact_incumbent = compact_result.get("objective_full")
    return (
        lower_bound is None
        or compact_incumbent is None
        or lower_bound <= compact_incumbent + tolerance
    )


def _validate_campaign(manifest, args) -> None:
    if manifest.get("time_limit_seconds_per_run") != args.time_limit or manifest.get("threads") != args.threads:
        raise ValueError("campaign solver settings mismatch")
    if manifest.get("truck_arc_probability") != args.truck_arc_probability:
        raise ValueError("campaign truck probability mismatch")
    if manifest.get("hub_arc_probability") != args.hub_arc_probability:
        raise ValueError("campaign hub probability mismatch")


def _install_replacement(
    campaign_dir,
    manifest,
    instance,
    digest,
    stage_snapshot,
    metadata,
    args,
    build_transformed_graph,
    instance_physical_fingerprint,
):
    current = next(record for record in manifest["cases"] if record["case_id"] == CASE_ID)
    if current["instance_snapshot_sha256"] == digest:
        return manifest
    case_dir = campaign_dir / "cases" / CASE_ID
    if case_dir.exists():
        shutil.rmtree(case_dir)
    for suffix in ("_solution_process_report.json", "_solution_process_report.md"):
        path = campaign_dir / f"{CASE_ID}{suffix}"
        if path.exists():
            path.unlink()
    official = campaign_dir / "instances" / f"{CASE_ID}.json"
    official.unlink()
    shutil.copy2(stage_snapshot, official)
    transformed = build_transformed_graph(instance)
    selection = metadata["performance_selection"]
    replacement = {
        **current,
        "instance_config": asdict(instance.config),
        "requested_seed": instance.requested_seed,
        "realized_seed": instance.config.seed,
        "generation_attempt": instance.generation_attempt,
        "generation_feasibility_time": instance.generation_feasibility_time,
        "generation_wall_time": metadata["generation_wall_time"],
        "instance_snapshot_file": str(official),
        "instance_snapshot_sha256": digest,
        "instance_physical_fingerprint": instance_physical_fingerprint(instance),
        "truck_arc_count": len(instance.truck_arcs),
        "drone_arc_count": len(instance.drone_arcs),
        "transformed_node_count": len(transformed.nodes),
        "transformed_arc_count": len(transformed.arcs),
        "warm_start_certification": {
            "accepted": True,
            "status": "warm_start_certified",
            "diagnostics": selection["warm_start"],
        },
        "performance_selection": selection,
    }
    manifest = {
        **manifest,
        "performance_filtered_cases": [CASE_ID],
        "pc8_performance_selection": {
            "transformed_arc_range": [args.min_transformed_arcs, args.max_transformed_arcs],
            "max_bpc_gap": args.max_bpc_gap,
            "compact_required_status_code": 9,
            "time_limit_seconds": args.time_limit,
        },
        "cases": [replacement if record["case_id"] == CASE_ID else record for record in manifest["cases"]],
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


def _validate_final_performance(campaign_dir, args) -> None:
    bpc_status = _read_json(campaign_dir / "cases" / CASE_ID / "bpc" / "attempt_001" / "run_status.json")
    compact_status = _read_json(campaign_dir / "cases" / CASE_ID / "compact" / "attempt_001" / "run_status.json")
    bpc = _read_json(Path(bpc_status["result_file"]))
    compact = _read_json(Path(compact_status["result_file"]))
    if bpc.get("gap_full") is None or bpc["gap_full"] > args.max_bpc_gap:
        raise RuntimeError(f"final PC8 BPC gap exceeds selection threshold: {bpc.get('gap_full')}")
    if not bpc.get("bpc_stats", {}).get("root_compact_incumbent_validated"):
        raise RuntimeError("final PC8 BPC warm-start incumbent is not validated")
    if compact.get("status_code") != 9:
        raise RuntimeError(f"final PC8 compact unexpectedly proved optimal: {compact.get('status_code')}")
    if not cross_solver_bounds_consistent(bpc, compact):
        raise RuntimeError(
            f"final PC8 cross-solver contradiction: BPC bound {bpc.get('lower_bound_full')} "
            f"> compact incumbent {compact.get('objective_full')}"
        )


def _preserved_artifact_hashes(campaign_dir):
    paths = []
    for path in (campaign_dir / "instances").glob("*.json"):
        if path.stem != CASE_ID:
            paths.append(path)
    for case_dir in (campaign_dir / "cases").iterdir():
        if case_dir.is_dir() and case_dir.name != CASE_ID:
            paths.extend(path for path in case_dir.rglob("*") if path.is_file())
    for path in campaign_dir.glob("*_solution_process_report.*"):
        if not path.name.startswith(CASE_ID):
            paths.append(path)
    return {
        str(path.relative_to(campaign_dir)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted(paths)
    }


def _finalize_provenance(campaign_dir, manifest, expected_preserved):
    if _preserved_artifact_hashes(campaign_dir) != expected_preserved:
        raise RuntimeError("non-PC8 artifacts changed during performance replacement")
    path = campaign_dir / "pc8_performance_replacement_provenance.json"
    provenance = _read_json(path)
    provenance["completed_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
    provenance["new_case"] = next(record for record in manifest["cases"] if record["case_id"] == CASE_ID)
    provenance["preserved_artifacts_verified"] = True
    _write_json(path, provenance)
    from .dense_large_campaign import _artifact_inventory, _write_csv

    _write_csv(campaign_dir / "artifact_inventory.csv", _artifact_inventory(campaign_dir))


def _selection_disclosure(args) -> str:
    return (
        f"PC8 was deterministically filtered to {args.min_transformed_arcs}-{args.max_transformed_arcs} transformed "
        f"arcs, two validated warm starts by {args.first_incumbent_deadline}s, BPC gap <= {args.max_bpc_gap}, "
        f"and direct compact Gurobi TIME_LIMIT at {args.time_limit}s"
    )


def write_selection_summary(selection_root: Path) -> None:
    records = []
    for path in sorted(selection_root.glob("candidate_*/selection_result.json")):
        selection = _read_json(path)
        warm = selection.get("warm_start", {})
        warm_runs = warm.get("runs", [])
        final_rejection = path.parent / "final_rejection.json"
        effective_status = selection["acceptance_status"]
        if final_rejection.exists():
            effective_status = _read_json(final_rejection)["reason"]
        records.append(
            {
                "candidate": path.parent.name,
                "attempt": selection["attempt"],
                "realized_seed": selection["realized_seed"],
                "transformed_arcs": selection["transformed_arc_count"],
                "effective_status": effective_status,
                "warm_repetitions": len(warm_runs),
                "warm_max_first_incumbent": max(
                    (run["first_incumbent_time"] for run in warm_runs if run["first_incumbent_time"] is not None),
                    default=None,
                ),
                "bpc_status": selection.get("bpc_trial", {}).get("status"),
                "bpc_objective": selection.get("bpc_trial", {}).get("objective"),
                "bpc_bound": selection.get("bpc_trial", {}).get("bound"),
                "bpc_gap": selection.get("bpc_trial", {}).get("gap"),
                "bpc_runtime": selection.get("bpc_trial", {}).get("runtime"),
                "compact_status_code": selection.get("compact_trial", {}).get("status_code"),
                "compact_objective": selection.get("compact_trial", {}).get("objective"),
                "compact_bound": selection.get("compact_trial", {}).get("bound"),
                "compact_gap": selection.get("compact_trial", {}).get("gap"),
                "compact_runtime": selection.get("compact_trial", {}).get("runtime"),
            }
        )
    _write_json(selection_root / "pc8_performance_selection_summary.json", {"candidates": records})
    csv_path = selection_root / "pc8_performance_selection_summary.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)
    lines = [
        "# PC8 Performance Selection Summary",
        "",
        "PC8 was explicitly filtered by graph size and solver outcomes; it is not an unbiased generated draw.",
        "",
        "| Candidate | Seed | Arcs | Effective status | Warm max | BPC gap | Compact code |",
        "|---|---:|---:|---|---:|---:|---:|",
    ]
    for record in records:
        lines.append(
            f"| {record['candidate']} | {record['realized_seed']} | {record['transformed_arcs']} | "
            f"{record['effective_status']} | {_format(record['warm_max_first_incumbent'])} | "
            f"{_format(record['bpc_gap'])} | {_format(record['compact_status_code'])} |"
        )
    (selection_root / "pc8_performance_selection_summary.md").write_text(
        "\n".join(lines) + "\n",
        encoding="utf-8",
    )


def _format(value) -> str:
    return "n/a" if value is None else str(value)


def _read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, allow_nan=False), encoding="utf-8")


if __name__ == "__main__":
    main()
