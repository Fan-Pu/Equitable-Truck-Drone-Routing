from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--campaign-dir", type=Path, required=True)
    parser.add_argument("--case-id", required=True)
    return parser


def main() -> None:
    args = _parser().parse_args()
    campaign_dir = args.campaign_dir.resolve()
    summary = _read_json(campaign_dir / "campaign_summary.json")
    rows = [row for row in summary if row["case_id"] == args.case_id]
    by_solver = {row["solver"]: row for row in rows}
    if set(by_solver) != {"bpc", "compact"}:
        raise RuntimeError(f"expected completed BPC and compact rows for {args.case_id}, found {sorted(by_solver)}")

    manifest = _read_json(campaign_dir / "campaign_manifest.json")
    case = next(record for record in manifest["cases"] if record["case_id"] == args.case_id)
    bpc_path = Path(by_solver["bpc"]["result_file"]).resolve()
    compact_path = Path(by_solver["compact"]["result_file"]).resolve()
    bpc_attempt = bpc_path.parent
    compact_attempt = compact_path.parent
    bpc = _read_json(bpc_path)
    compact = _read_json(compact_path)
    expected_hash = case["instance_snapshot_sha256"]
    if bpc.get("instance_snapshot_sha256") != expected_hash:
        raise RuntimeError("BPC result snapshot hash mismatch")
    if compact.get("instance_snapshot_sha256") != expected_hash:
        raise RuntimeError("compact result snapshot hash mismatch")

    from .config import ObjectiveWeights
    from .instance import read_instance_snapshot
    from .objective import build_objective_data
    from .routes import route_from_path
    from .transform import build_transformed_graph

    instance, digest = read_instance_snapshot(Path(case["instance_snapshot_file"]), expected_sha256=expected_hash)
    weights = ObjectiveWeights(**manifest["objective_weights"])
    objective = build_objective_data(instance, weights)
    graph = build_transformed_graph(instance)
    bpc_paths = tuple(tuple(route["path"]) for route in bpc.get("routes", []))
    compact_paths = tuple(tuple(path) for path in compact.get("route_paths", []))
    bpc_validation = _validate_paths(instance, objective, graph, bpc_paths, route_from_path)
    compact_validation = _validate_paths(instance, objective, graph, compact_paths, route_from_path)
    same_route_set = {
        tuple(route["path"])
        for route in bpc_validation.get("routes", [])
    } == {
        tuple(route["path"])
        for route in compact_validation.get("routes", [])
    }

    bpc_stats = bpc.get("bpc_stats", {})
    pricing = bpc_stats.get("pricing_diagnostics", [])
    progress_path = bpc_attempt / "bpc_progress.jsonl"
    progress = [json.loads(line) for line in progress_path.read_text(encoding="utf-8").splitlines()]
    tree_summary = {
        "event_count": len(progress),
        "event_type_counts": {
            event: sum(record["event"] == event for record in progress)
            for event in sorted({record["event"] for record in progress})
        },
        "last_event": None if not progress else progress[-1],
    }
    bpc_resource = _read_json(bpc_attempt / "resource_usage.json")
    compact_resource = _read_json(compact_attempt / "resource_usage.json")
    compact_log_summary = _compact_log_summary(compact_attempt / "gurobi_logs" / "compact_arc.log")
    bpc_root_log_summary = _gurobi_version_summary(bpc_attempt / "gurobi_logs" / "root_compact.log")
    report = {
        "case": case,
        "snapshot_sha256": digest,
        "snapshot_hash_agreement": True,
        "campaign_rows": by_solver,
        "comparison": {
            "bpc_status": bpc.get("status"),
            "compact_status": compact.get("status"),
            "compact_termination": (
                "time_limit_with_incumbent"
                if compact.get("status_code") == 9 and compact.get("objective_full") is not None
                else compact.get("status")
            ),
            "bpc_objective": bpc.get("objective_full"),
            "compact_objective": compact.get("objective_full"),
            "objective_difference": _difference(bpc.get("objective_full"), compact.get("objective_full")),
            "absolute_objective_difference": (
                None
                if _difference(bpc.get("objective_full"), compact.get("objective_full")) is None
                else abs(_difference(bpc.get("objective_full"), compact.get("objective_full")))
            ),
            "same_route_set": same_route_set,
            "bpc_bound": bpc.get("lower_bound_full"),
            "compact_bound": compact.get("objective_bound_full"),
            "bpc_gap": bpc.get("gap_full"),
            "compact_gap": compact.get("mip_gap"),
            "bpc_runtime": bpc.get("runtime"),
            "compact_runtime": compact.get("runtime"),
            "bpc_nodes": bpc.get("nodes_processed"),
            "compact_nodes": compact.get("node_count"),
            "compact_first_incumbent_seconds": compact.get("first_incumbent_time", compact_log_summary["first_incumbent_seconds"]),
        },
        "independent_validation": {
            "bpc": bpc_validation,
            "compact": compact_validation,
        },
        "bpc_components": {
            "root": _select(
                bpc_stats,
                "root_compact_status", "root_compact_solve_budget_seconds", "root_compact_objective_full",
                "root_compact_bound_full", "root_compact_node_count", "root_compact_iteration_count",
                "root_compact_accepted_columns", "root_compact_incumbent_validated",
                "root_compact_route_paths", "root_lower_bound_full", "root_closed", "root_closure_time",
                "root_rmp_is_integer", "root_fractional_variable_count", "root_nonzero_variable_count",
                "root_max_integrality_violation", "root_incumbent_at_classification_full",
                "root_incumbent_at_fathom_full", "root_fathom_reason", "root_branch_required",
            ),
            "pricing": _select(
                bpc_stats,
                "standard_pricing_calls", "farkas_pricing_calls", "columns_added_standard",
                "columns_added_farkas", "total_routes", "global_pool_routes", "pricing_labels_generated",
                "pricing_labels_dominated", "pricing_labels_pruned", "pricing_labels_purged",
                "pricing_extensions_attempted", "pricing_extensions_rejected_by_deadline",
                "pricing_complete_routes_generated", "pricing_negative_routes_verified",
                "pricing_negative_routes_inserted", "pricing_standard_bound_pruned", "pricing_farkas_bound_pruned",
                "pricing_process_cpu_time", "pricing_cpu_core_equivalent_max", "best_reduced_cost_at_stop",
            ),
            "cuts_and_branching": _select(
                bpc_stats,
                "sr_cuts_added", "sr_cuts_added_root", "sr_cuts_added_postroot", "branching_nodes",
                "customer_pair_branches", "launch_pad_branches", "conditioned_arc_branches",
                "child_nodes_created", "postroot_nodes_processed", "open_nodes_at_termination",
                "postroot_open_nodes", "queue_bound_fathoms",
            ),
            "dynamic_splitting": _select(
                bpc_stats,
                "pricing_balanced_process_dynamic_calls", "pricing_dynamic_split_candidates",
                "pricing_dynamic_splits_performed", "pricing_dynamic_split_rejected_near_closure",
                "pricing_dynamic_split_rejected_small_frontier", "pricing_dynamic_split_rejected_elapsed",
                "pricing_dynamic_split_rejected_low_work", "pricing_dynamic_child_tasks_created",
                "pricing_dynamic_labels_transferred", "pricing_dynamic_bytes_transferred",
                "pricing_dynamic_split_control_time", "pricing_leaf_tasks_created", "pricing_leaf_tasks_closed",
            ),
            "workers": _select(
                bpc_stats,
                "pricing_parallel_workers_max", "pricing_parallel_calls", "pricing_pool_startup_count",
                "pricing_pool_startup_time", "pricing_pool_reused_calls", "pricing_pool_shutdown_time",
                "pricing_worker_busy_seconds", "pricing_worker_idle_seconds", "pricing_idle_work_requests",
                "pricing_task_submission_time", "pricing_process_cpu_time",
            ),
            "route_pool": _select(
                bpc_stats,
                "heuristic_calls", "heuristic_time", "heuristic_hard_pool_solves",
                "heuristic_hard_pool_time", "heuristic_hard_pool_feasible_solves",
                "heuristic_full_pool_calls", "heuristic_full_pool_time", "heuristic_full_pool_feasible",
                "heuristic_incumbent_updates", "heuristic_full_pool_incumbent_updates",
                "incumbent_source", "time_to_first_incumbent",
            ),
            "pricing_calls": pricing,
            "tree": tree_summary,
        },
        "resource_usage": {
            "bpc": bpc_resource,
            "compact": compact_resource,
        },
        "compact_log_summary": compact_log_summary,
        "bpc_root_log_summary": bpc_root_log_summary,
        "raw_results": {
            "bpc": bpc,
            "compact": compact,
        },
        "artifact_inventory": _inventory((bpc_attempt, compact_attempt), campaign_dir),
    }
    json_path = campaign_dir / f"{args.case_id}_solution_process_report.json"
    markdown_path = campaign_dir / f"{args.case_id}_solution_process_report.md"
    json_path.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    markdown_path.write_text(_markdown(report), encoding="utf-8")


def _validate_paths(instance, objective, graph, paths, route_from_path) -> dict[str, object]:
    if not paths:
        return {"incumbent_present": False, "valid": None, "route_count": 0}
    routes = tuple(route_from_path(index, path, graph, objective) for index, path in enumerate(paths))
    coverage = {
        customer: sum(customer in route.served for route in routes)
        for customer in instance.customers
    }
    valid = all(count == 1 for count in coverage.values()) and len(routes) <= instance.num_trucks
    if not valid:
        raise RuntimeError(f"independent route validation failed: {coverage}")
    return {
        "incumbent_present": True,
        "valid": True,
        "route_count": len(routes),
        "coverage": coverage,
        "routes": [route.to_record(objective) for route in routes],
    }


def _select(values: dict[str, object], *keys: str) -> dict[str, object]:
    return {key: values.get(key) for key in keys}


def _difference(left, right):
    return None if left is None or right is None else left - right


def _inventory(attempts: tuple[Path, ...], campaign_dir: Path) -> list[dict[str, object]]:
    records = []
    for attempt in attempts:
        for path in sorted(item for item in attempt.rglob("*") if item.is_file()):
            records.append(
                {
                    "relative_path": str(path.relative_to(campaign_dir)),
                    "size_bytes": path.stat().st_size,
                    "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                }
            )
    return records


def _compact_log_summary(path: Path) -> dict[str, object]:
    text = path.read_text(encoding="utf-8")
    incumbent_times = []
    for line in text.splitlines():
        stripped = line.lstrip()
        if not stripped.startswith(("H", "*")):
            continue
        match = re.search(r"\s(\d+(?:\.\d+)?)s\s*$", stripped)
        if match:
            incumbent_times.append(float(match.group(1)))
    return {
        **_gurobi_version_summary(path),
        "first_incumbent_seconds": min(incumbent_times) if incumbent_times else None,
        "incumbent_update_count": len(incumbent_times),
        "time_limit_reached": "Time limit reached" in text,
    }


def _gurobi_version_summary(path: Path) -> dict[str, object]:
    text = path.read_text(encoding="utf-8")
    match = re.search(r"Gurobi Optimizer version ([^\s]+)", text)
    return {"gurobi_version": None if match is None else match.group(1)}


def _read_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def _markdown(report: dict[str, object]) -> str:
    comparison = report["comparison"]
    bpc = report["raw_results"]["bpc"]
    compact = report["raw_results"]["compact"]
    components = report["bpc_components"]
    certification = report["case"].get("warm_start_certification")
    certification_lines = []
    if certification is not None:
        screen = certification["diagnostics"]
        certification_lines = [
            f"- Warm-start-filtered realized seed: `{report['case']['realized_seed']}`",
            f"- Warm-start screening attempt: `{screen['attempt']}`",
            f"- Screening incumbent/objective/time: `{screen['accepted']}` / `{screen['objective']}` / "
            f"`{screen['first_incumbent_time']}` s",
            "- Selection disclosure: this instance was filtered for a validated unseeded 60-second compact incumbent.",
        ]
    performance = report["case"].get("performance_selection")
    if performance is not None:
        certification_lines.extend(
            [
                f"- Performance-selection attempt/arcs: `{performance['attempt']}` / "
                f"`{performance['transformed_arc_count']}`",
                f"- Performance trial BPC gap: `{performance['bpc_trial']['gap']}`",
                f"- Performance trial compact status code: `{performance['compact_trial']['status_code']}`",
                "- Performance disclosure: PC8 was selected for low BPC gap and direct-Gurobi non-optimality.",
            ]
        )
    lines = [
        f"# {report['case']['case_id']} Solution Process Report",
        "",
        f"- Snapshot SHA-256: `{report['snapshot_sha256']}`",
        f"- BPC status: `{comparison['bpc_status']}`",
        f"- Compact status: `{comparison['compact_status']}`",
        f"- Compact termination: `{comparison['compact_termination']}`",
        f"- Same canonical route set: `{comparison['same_route_set']}`",
        f"- BPC incumbent/bound/gap: `{comparison['bpc_objective']}` / `{comparison['bpc_bound']}` / `{comparison['bpc_gap']}`",
        f"- Compact incumbent/bound/gap: `{comparison['compact_objective']}` / `{comparison['compact_bound']}` / `{comparison['compact_gap']}`",
        f"- BPC runtime/nodes: `{comparison['bpc_runtime']}` / `{comparison['bpc_nodes']}`",
        f"- Compact runtime/nodes: `{comparison['compact_runtime']}` / `{comparison['compact_nodes']}`",
        f"- Compact first incumbent: `{comparison['compact_first_incumbent_seconds']}` s",
        f"- BPC root/compact Gurobi versions: `{report['bpc_root_log_summary']['gurobi_version']}` / "
        f"`{report['compact_log_summary']['gurobi_version']}`",
        *certification_lines,
        "",
        "## BPC Root",
        "",
        _code_json(components["root"]),
        "",
        "## Pricing and Columns",
        "",
        _code_json(components["pricing"]),
        "",
        "## Cuts and Branching",
        "",
        _code_json(components["cuts_and_branching"]),
        "",
        "## Dynamic Splitting and Workers",
        "",
        _code_json({"dynamic_splitting": components["dynamic_splitting"], "workers": components["workers"]}),
        "",
        "## Route Pool and Incumbent",
        "",
        _code_json(components["route_pool"]),
        "",
        "## BPC Routes",
        "",
        *_route_lines(bpc.get("routes", [])),
        "",
        "## Compact Routes",
        "",
        *_compact_route_lines(compact),
        "",
        "## Resource Usage",
        "",
        _code_json(report["resource_usage"]),
        "",
        f"Full pricing-call diagnostics, tree summaries, raw solver results, and hashes for {len(report['artifact_inventory'])} artifacts are stored in the JSON report.",
    ]
    return "\n".join(lines) + "\n"


def _route_lines(routes) -> list[str]:
    if not routes:
        return ["No BPC incumbent route was returned."]
    return [
        f"- Route {route['id']}: truck `{route['truck_path']}`; drones `{route['drone_blocks']}`; "
        f"return `{route['return_time']}`; services `{route['service_times']}`"
        for route in routes
    ]


def _compact_route_lines(compact) -> list[str]:
    routes = compact.get("routes", [])
    if routes:
        return _route_lines(routes)
    paths = compact.get("route_paths", [])
    if not paths:
        return ["No compact incumbent route was returned."]
    return [f"- `{path}`" for path in paths]


def _code_json(value) -> str:
    return "```json\n" + json.dumps(value, indent=2, sort_keys=True) + "\n```"


if __name__ == "__main__":
    main()
