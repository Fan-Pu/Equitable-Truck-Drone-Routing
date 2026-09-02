from __future__ import annotations

import argparse
from dataclasses import asdict, replace
import json
from pathlib import Path
import sys
import time

from .compact import solve_compact_solution
from .config import ObjectiveWeights
from .instance import (
    generate_instance,
    instance_physical_fingerprint,
    read_instance_snapshot,
    write_instance_snapshot,
)
from .objective import build_objective_data


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline-snapshot", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--gurobi-python-path", type=Path)
    return parser


def main() -> None:
    args = _parser().parse_args()
    if args.gurobi_python_path is not None:
        sys.path.append(str(args.gurobi_python_path))
    args.output_dir.mkdir(parents=True, exist_ok=False)
    baseline, baseline_hash = read_instance_snapshot(args.baseline_snapshot)
    if baseline.requested_seed is None:
        raise ValueError("baseline snapshot is missing its requested seed")
    requested_config = replace(baseline.config, seed=baseline.requested_seed)

    start = time.time()
    improved = generate_instance(requested_config)
    improved_wall = time.time() - start
    improved_snapshot = args.output_dir / "improved_instance.json"
    improved_hash = write_instance_snapshot(improved, improved_snapshot)
    baseline_fingerprint = instance_physical_fingerprint(baseline)
    improved_fingerprint = instance_physical_fingerprint(improved)
    same_instance = (
        improved.generation_attempt == baseline.generation_attempt
        and improved.config.seed == baseline.config.seed
        and improved_fingerprint == baseline_fingerprint
    )
    if not same_instance:
        raise RuntimeError("improved gate did not reproduce the baseline accepted physical instance")

    weights = ObjectiveWeights(1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0)
    objective = build_objective_data(improved, weights)
    legacy = solve_compact_solution(
        improved,
        weights,
        time_limit=None,
        threads=0,
        require_optimal=False,
        objective=objective,
        feasibility_only=True,
        solution_limit=1,
        dual_reductions=0,
        decode_routes=False,
    )
    if legacy.objective_full is None:
        raise RuntimeError("legacy gate failed to find the accepted feasible solution")

    attempts = list(improved.generation_feasibility_diagnostics)
    gate_attempts = [record for record in attempts if "diagnostics" in record]
    precheck_rejections = sum(record.get("status") == "infeasible_precheck" for record in gate_attempts)
    fallback_calls = sum(
        bool(record["diagnostics"].get("fallback_used"))
        for record in gate_attempts
    )
    final_gate = gate_attempts[-1]["diagnostics"]
    final_stage = final_gate["stages"][-1]
    baseline_seconds = baseline.generation_feasibility_time
    report = {
        "baseline_snapshot": str(args.baseline_snapshot.resolve()),
        "baseline_snapshot_sha256": baseline_hash,
        "improved_snapshot": str(improved_snapshot.resolve()),
        "improved_snapshot_sha256": improved_hash,
        "baseline_physical_fingerprint": baseline_fingerprint,
        "improved_physical_fingerprint": improved_fingerprint,
        "same_physical_instance": same_instance,
        "requested_seed": baseline.requested_seed,
        "realized_seed": improved.config.seed,
        "generation_attempt": improved.generation_attempt,
        "baseline_generation_seconds": baseline_seconds,
        "improved_generation_seconds": improved.generation_feasibility_time,
        "improved_wall_seconds": improved_wall,
        "speedup": baseline_seconds / improved.generation_feasibility_time,
        "attempt_count": len(attempts),
        "milp_gate_calls": len(gate_attempts),
        "precheck_rejections": precheck_rejections,
        "fallback_calls": fallback_calls,
        "aggregate_variable_count": final_gate["variable_count"],
        "legacy_variable_count": final_gate["legacy_variable_count"],
        "variable_reduction_fraction": 1.0 - final_gate["variable_count"] / final_gate["legacy_variable_count"],
        "aggregate_constraint_count": final_gate["constraint_count"],
        "aggregate_nonzero_count": final_gate["nonzero_count"],
        "aggregate_accepted_stage_nodes": final_stage["node_count"],
        "aggregate_accepted_stage_iterations": final_stage["iteration_count"],
        "legacy_accepted_stage_nodes": legacy.node_count,
        "legacy_accepted_stage_iterations": legacy.iteration_count,
        "attempt_diagnostics": attempts,
    }
    (args.output_dir / "comparison.json").write_text(
        json.dumps(report, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    (args.output_dir / "comparison.md").write_text(_markdown_report(report), encoding="utf-8")
    print(args.output_dir / "comparison.json")


def _markdown_report(report: dict[str, object]) -> str:
    return "\n".join(
        [
            "# Exact Feasibility-Gate Comparison",
            "",
            f"- Requested seed: {report['requested_seed']}",
            f"- Realized seed: {report['realized_seed']}",
            f"- Accepted attempt: {report['generation_attempt']}",
            f"- Same physical instance: {report['same_physical_instance']}",
            "",
            "| Metric | Baseline | Improved |",
            "|---|---:|---:|",
            f"| Generation seconds | {report['baseline_generation_seconds']:.3f} | {report['improved_generation_seconds']:.3f} |",
            f"| Speedup | 1.000x | {report['speedup']:.3f}x |",
            f"| Feasibility variables | {report['legacy_variable_count']} | {report['aggregate_variable_count']} |",
            f"| Accepted-stage nodes | {report['legacy_accepted_stage_nodes']} | {report['aggregate_accepted_stage_nodes']} |",
            f"| Accepted-stage iterations | {report['legacy_accepted_stage_iterations']} | {report['aggregate_accepted_stage_iterations']} |",
            "",
            f"Precheck rejections: {report['precheck_rejections']}; MILP gate calls: {report['milp_gate_calls']}; DualReductions fallbacks: {report['fallback_calls']}.",
        ]
    ) + "\n"


if __name__ == "__main__":
    main()
