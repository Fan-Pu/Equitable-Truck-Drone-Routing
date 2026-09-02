from __future__ import annotations

import argparse
from dataclasses import asdict, replace
import json
from pathlib import Path
import sys
import time

from .config import InstanceConfig
from .experiments import SCALES
from .feasibility import exact_feasibility_check
from .instance import generate_candidate, write_instance_snapshot
from .repair import discover_or_repair_instance


BASELINE_GENERATION_SECONDS = 59_215.2259986401
BASELINE_CANDIDATE_COUNT = 17


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--gurobi-python-path", type=Path)
    parser.add_argument("--baseline-generation-seconds", type=float, default=BASELINE_GENERATION_SECONDS)
    return parser


def main() -> None:
    args = _parser().parse_args()
    if args.gurobi_python_path is not None:
        sys.path.append(str(args.gurobi_python_path))
    args.output_dir.mkdir(parents=True, exist_ok=False)

    config = InstanceConfig(seed=4, distribution="PS", **SCALES["large"])
    generation_start = time.time()
    candidate = generate_candidate(config)
    candidate_seconds = time.time() - generation_start

    pipeline_start = time.time()
    repair = discover_or_repair_instance(candidate)
    pipeline_seconds = time.time() - pipeline_start
    if not repair.feasible or repair.status != "repaired" or repair.instance is None:
        raise RuntimeError(f"PS4 attempt-0 candidate was not repaired: {repair.status}")
    repaired = repair.instance

    validation_start = time.time()
    validation = exact_feasibility_check(repaired)
    validation_seconds = time.time() - validation_start
    if not validation.feasible:
        raise RuntimeError(f"independent exact feasibility validation failed: {validation.status}")
    if not repair.diagnostics.certificate_validation.valid:
        raise RuntimeError("repair route certificate failed independent decoding")

    total_generation_seconds = candidate_seconds + pipeline_seconds
    accepted = replace(
        repaired,
        requested_seed=4,
        generation_attempt=0,
        generation_feasibility_time=pipeline_seconds,
        generation_feasibility_diagnostics=(
            {
                "attempt": 0,
                "realized_seed": 4,
                "status": repair.status,
                "feasible": repair.feasible,
                "elapsed_seconds": pipeline_seconds,
                "diagnostics": _json_record(asdict(repair.diagnostics)),
            },
        ),
    )
    snapshot_path = args.output_dir / "repaired_instance.json"
    snapshot_sha256 = write_instance_snapshot(accepted, snapshot_path)
    report = {
        "case": "large_PS_seed4_attempt0",
        "dimensions": SCALES["large"],
        "requested_seed": 4,
        "realized_seed": 4,
        "accepted_attempt": 0,
        "baseline_generation_seconds": args.baseline_generation_seconds,
        "baseline_candidate_count": BASELINE_CANDIDATE_COUNT,
        "candidate_construction_seconds": candidate_seconds,
        "repair_pipeline_seconds": pipeline_seconds,
        "observed_total_generation_seconds": total_generation_seconds,
        "observed_wall_time_speedup": args.baseline_generation_seconds / total_generation_seconds,
        "original_discovery_limit_seconds": config.generation_feasibility_discovery_limit,
        "original_discovery_status": repair.diagnostics.original_discovery_status,
        "original_discovery_seconds": repair.diagnostics.original_discovery_time_seconds,
        "original_discovery_nodes": repair.diagnostics.original_discovery_nodes,
        "original_discovery_iterations": repair.diagnostics.original_discovery_iterations,
        "original_discovery_fallback_used": repair.diagnostics.original_discovery_fallback_used,
        "repair_seconds": repair.diagnostics.repair_solve_time_seconds,
        "repair_build_seconds": repair.diagnostics.repair_build_time_seconds,
        "repair_nodes": repair.diagnostics.repair_node_count,
        "repair_iterations": repair.diagnostics.repair_iteration_count,
        "route_search_seconds": repair.diagnostics.route_search_solve_time_seconds,
        "route_search_build_seconds": repair.diagnostics.route_search_build_time_seconds,
        "route_search_nodes": repair.diagnostics.route_search_node_count,
        "route_search_iterations": repair.diagnostics.route_search_iteration_count,
        "final_acceptance_seconds": (
            repair.diagnostics.repair_solve_time_seconds
            - repair.diagnostics.route_search_solve_time_seconds
        ),
        "final_acceptance_build_seconds": (
            repair.diagnostics.repair_build_time_seconds
            - repair.diagnostics.route_search_build_time_seconds
        ),
        "repair_variable_count": repair.diagnostics.repair_variable_count,
        "repair_constraint_count": repair.diagnostics.repair_constraint_count,
        "repair_nonzero_count": repair.diagnostics.repair_nonzero_count,
        "hamming_edit_count": repair.diagnostics.hamming_edit_count,
        "added_arcs": [list(arc) for arc in repair.diagnostics.added_arcs],
        "removed_arcs": [list(arc) for arc in repair.diagnostics.removed_arcs],
        "original_arc_counts": repair.diagnostics.original_arc_counts,
        "repaired_arc_counts": repair.diagnostics.repaired_arc_counts,
        "rejected_pattern_count": repair.diagnostics.rejected_pattern_count,
        "certificate_routes": repair.diagnostics.certificate_routes,
        "certificate_validation": asdict(repair.diagnostics.certificate_validation),
        "independent_validation_status": validation.status,
        "independent_validation_seconds": validation_seconds,
        "independent_validation_diagnostics": asdict(validation.diagnostics),
        "original_graph_fingerprint": repair.diagnostics.original_graph_fingerprint,
        "repaired_graph_fingerprint": repair.diagnostics.repaired_graph_fingerprint,
        "original_deadline_fingerprint": repair.diagnostics.original_deadline_fingerprint,
        "repaired_deadline_fingerprint": repair.diagnostics.repaired_deadline_fingerprint,
        "original_benchmark_fingerprint": repair.diagnostics.original_benchmark_fingerprint,
        "repaired_benchmark_fingerprint": repair.diagnostics.repaired_benchmark_fingerprint,
        "repaired_snapshot": str(snapshot_path.resolve()),
        "repaired_snapshot_sha256": snapshot_sha256,
        "repair_diagnostics": asdict(repair.diagnostics),
    }
    report_path = args.output_dir / "comparison.json"
    report_path.write_text(json.dumps(_json_record(report), indent=2, sort_keys=True), encoding="utf-8")
    (args.output_dir / "comparison.md").write_text(_markdown_report(report), encoding="utf-8")
    print(report_path.resolve())


def _markdown_report(report: dict[str, object]) -> str:
    added = ", ".join(f"{left}->{right}" for left, right in report["added_arcs"])
    removed = ", ".join(f"{left}->{right}" for left, right in report["removed_arcs"])
    routes = []
    for route in report["certificate_routes"]:
        truck_path = " -> ".join(route["truck_path"])
        drone = ", ".join(
            f"{hub}: {','.join(customers)}"
            for hub, customers in route["drone_blocks"].items()
        ) or "none"
        routes.append(f"- Route {route['route_id']}: truck `{truck_path}`; drone `{drone}`; return {route['return_time']:.3f}")
    return "\n".join(
        [
            "# Large PS Seed 4 Arc-Swap Repair Comparison",
            "",
            "The comparison uses the deterministic attempt-0 candidate with 30 customers, 6 trucks, 3 pads, and 4 drones per truck.",
            "",
            "| Metric | Baseline rejection generation | Arc-swap repair |",
            "|---|---:|---:|",
            f"| Accepted candidate | 17 | 1 |",
            f"| Generation wall time (s) | {report['baseline_generation_seconds']:.3f} | {report['observed_total_generation_seconds']:.3f} |",
            f"| Speedup | 1.000x | {report['observed_wall_time_speedup']:.3f}x |",
            "",
            f"- Original discovery: `{report['original_discovery_status']}` in {report['original_discovery_seconds']:.3f}s "
            f"(limit {report['original_discovery_limit_seconds']:.1f}s).",
            f"- Repair solve: {report['repair_seconds']:.3f}s; build: {report['repair_build_seconds']:.3f}s; "
            f"nodes: {report['repair_nodes']:.0f}; iterations: {report['repair_iterations']:.0f}.",
            f"- Exact route projection: {report['route_search_seconds']:.3f}s; final fixed-cardinality acceptance: "
            f"{report['final_acceptance_seconds']:.3f}s.",
            f"- Hamming edits: {report['hamming_edit_count']}.",
            f"- Added arcs: {added}.",
            f"- Removed arcs: {removed}.",
            f"- Preserved arc-class counts: `{report['original_arc_counts'] == report['repaired_arc_counts']}` "
            f"({report['repaired_arc_counts']}).",
            f"- Certificate valid: `{report['certificate_validation']['valid']}`; independent aggregate validation: "
            f"`{report['independent_validation_status']}`.",
            "",
            "## Route Certificate",
            "",
            *routes,
            "",
            "Deadlines and objective benchmarks were recomputed from the repaired graph; their fingerprints are recorded in the JSON report.",
            "",
            "## Methodological Disclosure",
            "",
            "This generator replaces rejection sampling after the discovery phase with exact feasibility-preserving arc-swap repair. "
            "Locations, demands, drone data, mandatory-drone customers, fleet data, fixed depot arcs, and realized internal arc counts by class remain fixed, but internal truck-arc identities may change. "
            "Future numerical experiments using repaired instances must disclose this protocol and must not describe those instances as generated by pure rejection sampling.",
        ]
    ) + "\n"


def _json_record(value: object) -> object:
    return json.loads(json.dumps(value, allow_nan=False))


if __name__ == "__main__":
    main()
