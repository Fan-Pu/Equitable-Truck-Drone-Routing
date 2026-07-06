from __future__ import annotations

import argparse
import csv
import json
import os
from dataclasses import asdict
from pathlib import Path
import subprocess
import sys
import time
import traceback
import tracemalloc
from typing import Any

from .metrics import solution_service_metrics
from .service_windows import load_manual_service_deadline_bounds


SCALES = {
    "small": {"num_customers": 5, "num_trucks": 2, "num_hubs": 2, "drones_per_truck": 4},
    "medium": {"num_customers": 15, "num_trucks": 5, "num_hubs": 2, "drones_per_truck": 4},
    "large": {"num_customers": 25, "num_trucks": 8, "num_hubs": 2, "drones_per_truck": 4},
}
DEFAULT_SEEDS = [1, 2, 3]


def main() -> None:
    parser = argparse.ArgumentParser()
    legacy_help = argparse.SUPPRESS
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--seeds", nargs="+", type=int, default=DEFAULT_SEEDS)
    parser.add_argument("--distributions", nargs="+", choices=["PS", "PC", "mixed"], default=["PS"])
    parser.add_argument("--scales", nargs="+", choices=sorted(SCALES), default=["small", "medium", "large"])
    parser.add_argument("--weights", nargs=3, type=float, default=[0.4, 0.3, 0.3])
    parser.add_argument("--service-deadline-mode", choices=["none", "manual", "random_absolute"], default="none")
    parser.add_argument("--service-deadline-file", type=Path)
    parser.add_argument("--service-deadline-fraction", type=float, default=0.60, help=legacy_help)
    parser.add_argument("--service-deadline-offset-min", type=float, default=45.0)
    parser.add_argument("--service-deadline-offset-max", type=float, default=120.0)
    parser.add_argument("--service-deadline-witness-slack", type=float, default=5.0)
    parser.add_argument("--service-deadline-random-seed", type=int)
    parser.add_argument(
        "--service-deadline-witness-method",
        choices=["constructive", "compact", "constructive_then_compact"],
        default="constructive_then_compact",
    )
    parser.add_argument("--service-deadline-witness-time-limit", type=float, default=30.0)
    parser.add_argument("--threads", type=int, default=1)
    parser.add_argument("--case-time-limit", type=float, default=900.0)
    parser.add_argument("--external-timeout-grace", type=float, default=10.0)
    parser.add_argument("--pricing-batch-size", type=int, default=64)
    parser.add_argument("--min-pricing-batch-size", type=int, default=32)
    parser.add_argument("--max-root-pricing-batch-size", type=int, default=256)
    parser.add_argument("--nonroot-pricing-batch-size", type=int, default=16)
    parser.add_argument("--certification-pass-interval", type=int, default=5, help=legacy_help)
    parser.add_argument("--pricing-yield-window", type=int, default=5, help=legacy_help)
    parser.add_argument("--pricing-yield-low", type=float, default=0.25, help=legacy_help)
    parser.add_argument("--pricing-yield-high", type=float, default=1.0, help=legacy_help)
    parser.add_argument("--productive-pricing-slice-seconds", type=float, default=30.0)
    parser.add_argument("--productive-slice-min-seconds", type=float, default=5.0)
    parser.add_argument("--productive-slice-max-seconds", type=float, default=30.0)
    parser.add_argument("--productive-yield-window", type=int, default=5)
    parser.add_argument("--productive-yield-low-threshold", type=float, default=0.5)
    parser.add_argument("--productive-yield-high-threshold", type=float, default=2.0)
    parser.add_argument("--closure-attempt-batch-period", type=int, default=4)
    parser.add_argument("--closure-attempt-time-period", type=float, default=120.0)
    parser.add_argument("--closure-certification-time-budget", type=float)
    parser.add_argument("--closure-batch-size", type=int, default=32)
    parser.add_argument("--use-dual-stabilized-productive-search", action="store_true")
    parser.add_argument("--dual-stabilization-weight", type=float, default=0.3)
    parser.add_argument("--prefix-task-depth", type=int, default=1)
    parser.add_argument("--productive-candidate-multiplier", type=float, default=1.5)
    parser.add_argument("--source-neighbor-task-size", type=int, default=1)
    parser.add_argument("--pricing-diversity-batch-fraction", type=float, default=0.5)
    parser.add_argument("--first-incumbent-route-pool-time-limit", type=float, default=10.0)
    parser.add_argument("--post-incumbent-primal-budget-factor", type=float, default=0.25)
    parser.add_argument("--root-constructive-time-limit", type=float, default=5.0)
    parser.add_argument("--disable-constructive-root-incumbent", action="store_true")
    parser.add_argument(
        "--root-compact-after-constructive",
        choices=["skip", "small_budget", "full_budget", "conditional_small_budget", "conditional_wall_budget"],
        default="conditional_wall_budget",
    )
    parser.add_argument("--root-compact-time-limit-after-constructive", type=float, default=1.0)
    parser.add_argument("--root-compact-time-limit-without-constructive", type=float, default=5.0)
    parser.add_argument("--root-compact-wall-time-limit", type=float, default=1.0)
    parser.add_argument("--root-compact-solve-time-limit", type=float, default=1.0)
    parser.add_argument("--constructive-diversity-threshold", type=float, default=0.35)
    parser.add_argument("--constructive-incumbent-quality-threshold", type=float)
    parser.add_argument("--enable-drone-diversification-warm-start", action="store_true", default=True)
    parser.add_argument("--enable-incremental-rmp", action="store_true", default=True)
    parser.add_argument("--enable-active-coefficient-cache", action="store_true", default=True)
    parser.add_argument("--disable-sr-aging", action="store_true")
    parser.add_argument("--disable-postroot-sr-cut-removal", action="store_true")
    parser.add_argument("--sr-inactive-age-threshold", type=int, default=1)
    parser.add_argument("--sr-removal-batch-size", type=int, default=32)
    parser.add_argument("--sr-max-removals-per-node", type=int, default=32)
    parser.add_argument("--disable-sr-reactivation", action="store_true")
    parser.add_argument("--sr-removal-min-active-count", type=int, default=64)
    parser.add_argument("--sr-removal-rmp-growth-threshold", type=float, default=0.20)
    parser.add_argument("--sr-removal-build-time-threshold", type=float, default=0.50)
    parser.add_argument("--sr-removal-active-coeff-threshold", type=float, default=0.0)
    parser.add_argument("--sr-activity-tolerance", type=float, default=1e-7)
    parser.add_argument("--enable-global-branch-route-index", action="store_true", default=True)
    parser.add_argument("--enable-node-column-aging", action="store_true", default=True)
    parser.add_argument("--column-inactive-age-min", type=int, default=3)
    parser.add_argument("--column-active-value-tol", type=float, default=1e-8)
    parser.add_argument("--column-deactivation-min-active-columns", type=int, default=500)
    parser.add_argument("--column-deactivation-batch-size", type=int, default=256)
    parser.add_argument("--child-certification-slice-seconds", type=float, default=30.0)
    parser.add_argument("--child-certification-max-slices-per-node", type=int)
    parser.add_argument("--child-productive-before-certification", action="store_true")
    parser.add_argument("--enable-resumable-child-certification", action="store_true", default=True)
    parser.add_argument("--enable-child-closure-batch-adaptation", action="store_true", default=False)
    parser.add_argument("--child-closure-batch-min", type=int, default=16)
    parser.add_argument("--child-closure-batch-initial", type=int, default=32)
    parser.add_argument("--child-closure-batch-max", type=int, default=128)
    parser.add_argument("--child-certification-yield-window", type=int, default=5)
    parser.add_argument("--child-certification-yield-low", type=float, default=0.20)
    parser.add_argument("--child-certification-yield-high", type=float, default=0.60)
    parser.add_argument("--child-cert-useful-yield-window", type=int, default=5)
    parser.add_argument("--child-cert-no-route-yield-window", type=int, default=5)
    parser.add_argument("--child-cert-dual-stability-window", type=int, default=3)
    parser.add_argument("--child-cert-dual-change-tol", type=float, default=1e-6)
    parser.add_argument("--child-closure-batch-growth-factor", type=float, default=1.0)
    parser.add_argument("--child-closure-batch-shrink-factor", type=float, default=1.0)
    parser.add_argument("--child-useful-yield-low", type=float, default=0.20)
    parser.add_argument("--child-useful-yield-high", type=float, default=0.60)
    parser.add_argument("--child-no-route-yield-high", type=float, default=1.0)
    parser.add_argument("--enable-rmp-basis-reuse", action="store_true", default=True)
    parser.add_argument("--sr-removal-density-weight", type=float, default=1.0)
    parser.add_argument("--sr-removal-nnz-weight", type=float, default=1.0)
    parser.add_argument("--sr-removal-age-weight", type=float, default=1.0)
    parser.add_argument("--sr-removal-build-weight", type=float, default=1.0)
    parser.add_argument("--sr-removal-violation-weight", type=float, default=5.0)
    parser.add_argument("--sr-removal-dual-weight", type=float, default=5.0)
    parser.add_argument("--sr-removal-score-threshold", type=float, default=1.0)
    parser.add_argument("--sr-removal-max-per-node", type=int, default=20)
    parser.add_argument("--disable-row-local-sr-coeff-cache", action="store_true")
    parser.add_argument("--disable-dominance-prefilter-keys", action="store_true")
    parser.add_argument("--enable-promised-drone-construction", action="store_true", default=False, help=legacy_help)
    parser.add_argument("--promised-drone-construct-time-limit", type=float, default=5.0)
    parser.add_argument("--promised-drone-insert-top-k-customers", type=int, default=20)
    parser.add_argument("--promised-drone-insert-top-k-pads", type=int, default=5)
    parser.add_argument("--promised-drone-exchange-top-k-pairs", type=int, default=50)
    parser.add_argument("--promised-drone-min-improvement", type=float, default=1e-9)
    parser.add_argument("--disable-no-drone-incumbent-trigger", action="store_true")
    parser.add_argument(
        "--compact-after-no-drone-incumbent",
        choices=["small_budget", "full_budget"],
        default="small_budget",
        help=legacy_help,
    )
    parser.add_argument("--join-eval-budget", type=int, default=0, help=legacy_help)
    parser.add_argument("--pricing-certification-slice-seconds", type=float, default=0.0, help=legacy_help)
    parser.add_argument("--disable-join-lower-envelope", action="store_true", help=legacy_help)
    parser.add_argument("--join-generator-split-threshold", type=int, default=50_000, help=legacy_help)
    parser.add_argument("--join-generator-pair-batch-size", type=int, default=10_000, help=legacy_help)
    parser.add_argument("--disable-bucket-join-envelope", action="store_true", help=legacy_help)
    parser.add_argument("--disable-join-profile-cache", action="store_true", help=legacy_help)
    parser.add_argument("--small-join-pair-threshold", type=int, default=5_000, help=legacy_help)
    parser.add_argument("--small-join-cumulative-threshold", type=int, default=250_000, help=legacy_help)
    parser.add_argument("--max-join-bypass-calls", type=int, default=1_000, help=legacy_help)
    parser.add_argument("--small-dom-bucket-threshold", type=int, default=100, help=legacy_help)
    parser.add_argument("--small-dom-cumulative-threshold", type=int, default=500_000, help=legacy_help)
    parser.add_argument("--max-dom-bypass-calls", type=int, default=2_000, help=legacy_help)
    parser.add_argument("--root-max-side-pool-per-call", type=int, default=128)
    parser.add_argument("--side-pool-max-size", type=int, default=20_000)
    parser.add_argument("--side-pool-per-customer-keep", type=int, default=20)
    parser.add_argument("--join-payload-bin-width", type=float, default=1.0, help=legacy_help)
    parser.add_argument("--repair-time-fraction-of-pricing", type=float, default=0.05)
    parser.add_argument("--repair-time-hard-cap-root", type=float, default=30.0)
    parser.add_argument("--repair-stall-limit", type=int, default=3)
    parser.add_argument("--preclosure-pricing-call-interval", type=int, default=5)
    parser.add_argument("--preclosure-pool-growth-limit", type=int, default=100)
    parser.add_argument("--farkas-batch-size", type=int, default=16)
    parser.add_argument("--seed-batch-size", type=int, default=16)
    parser.add_argument("--repair-batch-size", type=int, default=16)
    parser.add_argument("--pricing-parallel-workers", type=int, default=2)
    parser.add_argument("--pricing-worker-backend", choices=["thread", "process"], default="thread")
    parser.add_argument("--prefix-task-depth-root", type=int, default=1)
    parser.add_argument("--prefix-task-depth-child", type=int, default=1)
    parser.add_argument("--prefix-task-min-branching-for-depth2", type=int, default=4)
    parser.add_argument("--logging-mode", choices=["audit", "light"], default="audit")
    parser.add_argument("--progress-snapshot-period", type=int, default=1)
    parser.add_argument("--disable-pricing-jsonl", action="store_true")
    parser.add_argument("--disable-bidirectional-pricing", action="store_true", help=legacy_help)
    parser.add_argument("--gurobi-python-path")
    parser.add_argument("--include-ablations", action="store_true")
    parser.add_argument("--worker", action="store_true")
    parser.add_argument("--case-file", type=Path)
    args = parser.parse_args()

    if args.worker:
        if args.case_file is None:
            raise ValueError("--case-file is required in worker mode")
        _run_worker(args.case_file)
        return

    if args.output_dir is None:
        raise ValueError("--output-dir is required")
    _run_parent(args)


def _run_parent(args: argparse.Namespace) -> None:
    output_dir: Path = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    args.service_deadline_manual_bounds = (
        load_manual_service_deadline_bounds(args.service_deadline_file)
        if args.service_deadline_file
        else None
    )
    cases = _build_case_specs(args)
    (output_dir / "experiment_manifest.json").write_text(
        json.dumps(
            {
                "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                "case_time_limit_seconds": args.case_time_limit,
                "external_timeout_grace_seconds": args.external_timeout_grace,
                "seeds": args.seeds,
                "distributions": args.distributions,
                "scales": args.scales,
                "weights": args.weights,
                "service_deadline_mode": args.service_deadline_mode,
                "service_deadline_file": str(args.service_deadline_file) if args.service_deadline_file else None,
                "service_deadline_fraction": args.service_deadline_fraction,
                "service_deadline_manual_bound_count": (
                    None if args.service_deadline_manual_bounds is None else len(args.service_deadline_manual_bounds)
                ),
                "service_deadline_offset_min": args.service_deadline_offset_min,
                "service_deadline_offset_max": args.service_deadline_offset_max,
                "service_deadline_witness_slack": args.service_deadline_witness_slack,
                "service_deadline_random_seed": args.service_deadline_random_seed,
                "service_deadline_witness_method": args.service_deadline_witness_method,
                "service_deadline_witness_time_limit": args.service_deadline_witness_time_limit,
                "threads": args.threads,
                "pricing_batch_size": args.pricing_batch_size,
                "min_pricing_batch_size": args.min_pricing_batch_size,
                "max_root_pricing_batch_size": args.max_root_pricing_batch_size,
                "nonroot_pricing_batch_size": args.nonroot_pricing_batch_size,
                "certification_pass_interval": args.certification_pass_interval,
                "pricing_yield_window": args.pricing_yield_window,
                "pricing_yield_low": args.pricing_yield_low,
                "pricing_yield_high": args.pricing_yield_high,
                "productive_pricing_slice_seconds": args.productive_pricing_slice_seconds,
                "productive_slice_min_seconds": args.productive_slice_min_seconds,
                "productive_slice_max_seconds": args.productive_slice_max_seconds,
                "productive_yield_window": args.productive_yield_window,
                "productive_yield_low_threshold": args.productive_yield_low_threshold,
                "productive_yield_high_threshold": args.productive_yield_high_threshold,
                "closure_attempt_batch_period": args.closure_attempt_batch_period,
                "closure_attempt_time_period": args.closure_attempt_time_period,
                "closure_certification_time_budget": args.closure_certification_time_budget,
                "closure_batch_size": args.closure_batch_size,
                "use_dual_stabilized_productive_search": args.use_dual_stabilized_productive_search,
                "dual_stabilization_weight": args.dual_stabilization_weight,
                "prefix_task_depth": args.prefix_task_depth,
                "productive_candidate_multiplier": args.productive_candidate_multiplier,
                "source_neighbor_task_size": args.source_neighbor_task_size,
                "pricing_diversity_batch_fraction": args.pricing_diversity_batch_fraction,
                "first_incumbent_route_pool_time_limit": args.first_incumbent_route_pool_time_limit,
                "post_incumbent_primal_budget_factor": args.post_incumbent_primal_budget_factor,
                "root_constructive_time_limit": args.root_constructive_time_limit,
                "enable_constructive_root_incumbent": not args.disable_constructive_root_incumbent,
                "root_compact_after_constructive": args.root_compact_after_constructive,
                "root_compact_time_limit_after_constructive": args.root_compact_time_limit_after_constructive,
                "root_compact_time_limit_without_constructive": args.root_compact_time_limit_without_constructive,
                "root_compact_wall_time_limit": args.root_compact_wall_time_limit,
                "root_compact_solve_time_limit": args.root_compact_solve_time_limit,
                "constructive_diversity_threshold": args.constructive_diversity_threshold,
                "constructive_incumbent_quality_threshold": args.constructive_incumbent_quality_threshold,
                "enable_drone_diversification_warm_start": args.enable_drone_diversification_warm_start,
                "enable_incremental_rmp": args.enable_incremental_rmp,
                "enable_active_coefficient_cache": args.enable_active_coefficient_cache,
                "enable_sr_aging": not args.disable_sr_aging,
                "enable_postroot_sr_cut_removal": not args.disable_postroot_sr_cut_removal,
                "sr_inactive_age_threshold": args.sr_inactive_age_threshold,
                "sr_removal_batch_size": args.sr_removal_batch_size,
                "sr_max_removals_per_node": args.sr_max_removals_per_node,
                "sr_reactivation_allowed": not args.disable_sr_reactivation,
                "sr_removal_min_active_count": args.sr_removal_min_active_count,
                "sr_removal_rmp_growth_threshold": args.sr_removal_rmp_growth_threshold,
                "sr_removal_build_time_threshold": args.sr_removal_build_time_threshold,
                "sr_removal_active_coeff_threshold": args.sr_removal_active_coeff_threshold,
                "sr_activity_tolerance": args.sr_activity_tolerance,
                "enable_global_branch_route_index": args.enable_global_branch_route_index,
                "enable_node_column_aging": args.enable_node_column_aging,
                "column_inactive_age_min": args.column_inactive_age_min,
                "column_active_value_tol": args.column_active_value_tol,
                "column_deactivation_min_active_columns": args.column_deactivation_min_active_columns,
                "column_deactivation_batch_size": args.column_deactivation_batch_size,
                "child_certification_slice_seconds": args.child_certification_slice_seconds,
                "child_certification_max_slices_per_node": args.child_certification_max_slices_per_node,
                "child_productive_before_certification": args.child_productive_before_certification,
                "enable_resumable_child_certification": args.enable_resumable_child_certification,
                "enable_child_closure_batch_adaptation": args.enable_child_closure_batch_adaptation,
                "child_closure_batch_min": args.child_closure_batch_min,
                "child_closure_batch_initial": args.child_closure_batch_initial,
                "child_closure_batch_max": args.child_closure_batch_max,
                "child_certification_yield_window": args.child_certification_yield_window,
                "child_certification_yield_low": args.child_certification_yield_low,
                "child_certification_yield_high": args.child_certification_yield_high,
                "child_cert_useful_yield_window": args.child_cert_useful_yield_window,
                "child_cert_no_route_yield_window": args.child_cert_no_route_yield_window,
                "child_cert_dual_stability_window": args.child_cert_dual_stability_window,
                "child_cert_dual_change_tol": args.child_cert_dual_change_tol,
                "child_closure_batch_growth_factor": args.child_closure_batch_growth_factor,
                "child_closure_batch_shrink_factor": args.child_closure_batch_shrink_factor,
                "child_useful_yield_low": args.child_useful_yield_low,
                "child_useful_yield_high": args.child_useful_yield_high,
                "child_no_route_yield_high": args.child_no_route_yield_high,
                "enable_rmp_basis_reuse": args.enable_rmp_basis_reuse,
                "sr_removal_density_weight": args.sr_removal_density_weight,
                "sr_removal_nnz_weight": args.sr_removal_nnz_weight,
                "sr_removal_age_weight": args.sr_removal_age_weight,
                "sr_removal_build_weight": args.sr_removal_build_weight,
                "sr_removal_violation_weight": args.sr_removal_violation_weight,
                "sr_removal_dual_weight": args.sr_removal_dual_weight,
                "sr_removal_score_threshold": args.sr_removal_score_threshold,
                "sr_removal_max_per_node": args.sr_removal_max_per_node,
                "use_row_local_sr_coeff_cache": not args.disable_row_local_sr_coeff_cache,
                "use_dominance_prefilter_keys": not args.disable_dominance_prefilter_keys,
                "use_promised_drone_construction": args.enable_promised_drone_construction,
                "promised_drone_construct_time_limit": args.promised_drone_construct_time_limit,
                "promised_drone_insert_top_k_customers": args.promised_drone_insert_top_k_customers,
                "promised_drone_insert_top_k_pads": args.promised_drone_insert_top_k_pads,
                "promised_drone_exchange_top_k_pairs": args.promised_drone_exchange_top_k_pairs,
                "promised_drone_min_improvement": args.promised_drone_min_improvement,
                "no_drone_incumbent_trigger": False,
                "compact_after_no_drone_incumbent": args.compact_after_no_drone_incumbent,
                "root_max_side_pool_per_call": args.root_max_side_pool_per_call,
                "side_pool_max_size": args.side_pool_max_size,
                "side_pool_per_customer_keep": args.side_pool_per_customer_keep,
                "repair_time_fraction_of_pricing": args.repair_time_fraction_of_pricing,
                "repair_time_hard_cap_root": args.repair_time_hard_cap_root,
                "repair_stall_limit": args.repair_stall_limit,
                "preclosure_pricing_call_interval": args.preclosure_pricing_call_interval,
                "preclosure_pool_growth_limit": args.preclosure_pool_growth_limit,
                "farkas_batch_size": args.farkas_batch_size,
                "seed_batch_size": args.seed_batch_size,
                "repair_batch_size": args.repair_batch_size,
                "pricing_parallel_workers": args.pricing_parallel_workers,
                "pricing_worker_backend": args.pricing_worker_backend,
                "prefix_task_depth_root": args.prefix_task_depth_root,
                "prefix_task_depth_child": args.prefix_task_depth_child,
                "prefix_task_min_branching_for_depth2": args.prefix_task_min_branching_for_depth2,
                "logging_mode": args.logging_mode,
                "progress_snapshot_period": args.progress_snapshot_period,
                "pricing_jsonl_enabled": not args.disable_pricing_jsonl,
                "pricing_engine": "source_neighbor_parallel_forward",
                "include_ablations": args.include_ablations,
                "cases": cases,
            },
            indent=2,
        ),
        encoding="utf-8",
    )
    summaries: list[dict[str, Any]] = []
    command_lines: list[str] = []
    for case in cases:
        case_dir = output_dir / case["case_id"]
        case_dir.mkdir(parents=True, exist_ok=True)
        case["case_dir"] = str(case_dir)
        case["solver_config"]["gurobi_log_dir"] = str(case_dir / "gurobi_logs")
        case_file = case_dir / "case_config.json"
        case_file.write_text(json.dumps(case, indent=2), encoding="utf-8")
        command = [sys.executable, "-m", "thvrpd.experiments", "--worker", "--case-file", str(case_file)]
        command_lines.append(" ".join(command))
        start = time.time()
        stdout_file = case_dir / "stdout.log"
        stderr_file = case_dir / "stderr.log"
        try:
            completed = subprocess.run(
                command,
                cwd=Path.cwd(),
                capture_output=True,
                text=True,
                timeout=args.case_time_limit + args.external_timeout_grace,
                env=os.environ.copy(),
            )
            stdout_file.write_text(completed.stdout, encoding="utf-8")
            stderr_file.write_text(completed.stderr, encoding="utf-8")
            result_path = _first_existing(case_dir, ["result.json", "timeout_result.json", "error_result.json"])
            if result_path is None:
                record = _parent_error_record(case, start, completed.returncode, "worker did not write a result file")
                (case_dir / "error_result.json").write_text(json.dumps(record, indent=2), encoding="utf-8")
            else:
                record = json.loads(result_path.read_text(encoding="utf-8"))
            record["worker_returncode"] = completed.returncode
        except subprocess.TimeoutExpired as exc:
            stdout_file.write_text(_decode_timeout_stream(exc.stdout), encoding="utf-8")
            stderr_file.write_text(_decode_timeout_stream(exc.stderr), encoding="utf-8")
            progress = _read_progress(case_dir)
            pricing_summary = _read_pricing_diagnostics(case_dir)
            progress_stats = progress.get("stats") if progress is not None else {}
            timeout_stats = _merge_timeout_stats(progress_stats, pricing_summary)
            record = {
                "case_id": case["case_id"],
                "scale": case["scale"],
                "variant": case["variant"],
                "distribution": case["distribution"],
                "seed": case["seed"],
                "status": "time_limited_external",
                "solver_status": "time_limited_external",
                "runtime_seconds": time.time() - start,
                "case_time_limit_seconds": args.case_time_limit,
                "instance_config": case["instance_config"],
                "solver_config": case["solver_config"],
                "error": (
                    f"worker exceeded parent timeout of "
                    f"{args.case_time_limit + args.external_timeout_grace} seconds "
                    f"including {args.external_timeout_grace} seconds of logging grace"
                ),
                "stdout_log": str(stdout_file),
                "stderr_log": str(stderr_file),
                "gurobi_log_files": _relative_files(case_dir, "gurobi_logs"),
                "bpc_progress": progress,
                "pricing_diagnostics_summary": pricing_summary,
                "bpc_stats": timeout_stats,
            }
            (case_dir / "timeout_result.json").write_text(json.dumps(record, indent=2), encoding="utf-8")
            record["worker_returncode"] = None
        summaries.append(_summary_row(record))
        _write_summary(output_dir, summaries)
    (output_dir / "commands.txt").write_text("\n".join(command_lines) + "\n", encoding="utf-8")
    _write_summary(output_dir, summaries)


def _run_worker(case_file: Path) -> None:
    case = json.loads(case_file.read_text(encoding="utf-8"))
    case_dir = Path(case["case_dir"])
    for filename in ("result.json", "timeout_result.json", "error_result.json", "bpc_progress.json", "pricing_diagnostics.jsonl"):
        (case_dir / filename).unlink(missing_ok=True)
    if case.get("gurobi_python_path"):
        sys.path.append(case["gurobi_python_path"])
    import gurobipy as gp

    from .bpc import BPCTimeLimitNoIncumbent, solve_branch_price_cut
    from .config import InstanceConfig, ObjectiveWeights, SolverConfig
    from .instance import generate_instance

    start = time.time()
    instance_generation_time = None
    bpc_wall_time = None
    bpc_start = None
    service_metrics_time = None
    tracemalloc.start()
    try:
        instance_config = InstanceConfig(**case["instance_config"])
        weights = ObjectiveWeights(*case["weights"])
        solver_config = SolverConfig(**case["solver_config"])
        instance_start = time.time()
        instance = generate_instance(instance_config)
        instance_generation_time = time.time() - instance_start
        bpc_start = time.time()
        result = solve_branch_price_cut(instance, weights, solver_config)
        bpc_wall_time = time.time() - bpc_start
        metrics_start = time.time()
        service_metrics = solution_service_metrics(instance, result)
        service_metrics_time = time.time() - metrics_start
        current_alloc, peak_alloc = tracemalloc.get_traced_memory()
        record = {
            "case_id": case["case_id"],
            "scale": case["scale"],
            "variant": case["variant"],
            "distribution": case["distribution"],
            "seed": case["seed"],
            "status": result.status,
            "solver_status": result.status,
            "runtime_seconds": time.time() - start,
            "case_time_limit_seconds": case["case_time_limit_seconds"],
            "instance_generation_time_seconds": instance_generation_time,
            "bpc_wall_time_seconds": bpc_wall_time,
            "service_metrics_time_seconds": service_metrics_time,
            "instance_config": asdict(instance_config),
            "solver_config": asdict(solver_config),
            "gurobi_version": ".".join(map(str, gp.gurobi.version())),
            "locations": instance.locations,
            "demand": instance.demand,
            "memory": {
                "python_current_allocated_mb": current_alloc / 1024.0 / 1024.0,
                "python_peak_allocated_mb": peak_alloc / 1024.0 / 1024.0,
            },
            "total_drones": instance.num_trucks * instance.drones_per_truck,
            **result.to_record(),
            "service_metrics": service_metrics,
            "gurobi_log_files": _relative_files(case_dir, "gurobi_logs"),
        }
        _write_worker_record(case_dir / "result.json", record)
    except BPCTimeLimitNoIncumbent as exc:
        if bpc_start is not None:
            bpc_wall_time = time.time() - bpc_start
        current_alloc, peak_alloc = tracemalloc.get_traced_memory()
        progress = _read_progress(case_dir)
        pricing_summary = _read_pricing_diagnostics(case_dir)
        timeout_stats = _merge_timeout_stats(asdict(exc.stats), pricing_summary)
        record = {
            "case_id": case["case_id"],
            "scale": case["scale"],
            "variant": case["variant"],
            "distribution": case["distribution"],
            "seed": case["seed"],
            "status": "time_limited_internal",
            "solver_status": "time_limited_internal",
            "runtime_seconds": time.time() - start,
            "case_time_limit_seconds": case["case_time_limit_seconds"],
            "instance_generation_time_seconds": instance_generation_time,
            "bpc_wall_time_seconds": bpc_wall_time,
            "service_metrics_time_seconds": service_metrics_time,
            "instance_config": asdict(instance_config),
            "solver_config": asdict(solver_config),
            "gurobi_version": ".".join(map(str, gp.gurobi.version())),
            "locations": instance.locations,
            "demand": instance.demand,
            "memory": {
                "python_current_allocated_mb": current_alloc / 1024.0 / 1024.0,
                "python_peak_allocated_mb": peak_alloc / 1024.0 / 1024.0,
            },
            "total_drones": instance.num_trucks * instance.drones_per_truck,
            "runtime": exc.runtime,
            "nodes_processed": exc.nodes_processed,
            "lower_bound_shifted": exc.lower_bound_shifted,
            "lower_bound_full": exc.objective.full_value_from_route_sum(exc.lower_bound_shifted),
            "normalization_bounds": asdict(exc.objective.bounds),
            "objective_coefficients": asdict(exc.objective.coeffs),
            "bpc_progress": progress,
            "pricing_diagnostics_summary": pricing_summary,
            "bpc_stats": timeout_stats,
            "gurobi_log_files": _relative_files(case_dir, "gurobi_logs"),
            "error": str(exc),
        }
        _write_worker_record(case_dir / "timeout_result.json", record)
    except Exception as exc:
        if bpc_start is not None:
            bpc_wall_time = time.time() - bpc_start
        _, peak_alloc = tracemalloc.get_traced_memory()
        record = {
            "case_id": case["case_id"],
            "scale": case["scale"],
            "variant": case["variant"],
            "distribution": case["distribution"],
            "seed": case["seed"],
            "status": "error",
            "solver_status": "error",
            "runtime_seconds": time.time() - start,
            "case_time_limit_seconds": case["case_time_limit_seconds"],
            "instance_generation_time_seconds": instance_generation_time,
            "bpc_wall_time_seconds": bpc_wall_time,
            "service_metrics_time_seconds": service_metrics_time,
            "instance_config": case["instance_config"],
            "solver_config": case["solver_config"],
            "error": repr(exc),
            "traceback": traceback.format_exc(),
            "memory": {"python_peak_allocated_mb": peak_alloc / 1024.0 / 1024.0},
            "gurobi_log_files": _relative_files(case_dir, "gurobi_logs"),
        }
        _write_worker_record(case_dir / "error_result.json", record)
        raise
    finally:
        tracemalloc.stop()


def _write_worker_record(path: Path, record: dict[str, Any]) -> None:
    output_start = time.time()
    _atomic_write_json(path, record)
    record["output_write_time_seconds"] = time.time() - output_start
    _atomic_write_json(path, record)


def _atomic_write_json(path: Path, record: dict[str, Any]) -> None:
    tmp_path = path.with_name(f"{path.name}.{time.time_ns()}.tmp")
    tmp_path.write_text(json.dumps(record, indent=2), encoding="utf-8")
    tmp_path.replace(path)


def _build_case_specs(args: argparse.Namespace) -> list[dict[str, Any]]:
    cases = []
    for scale in args.scales:
        scale_config = SCALES[scale]
        for distribution in args.distributions:
            for seed in args.seeds:
                cases.append(_case_spec(args, scale, distribution, seed, "full", scale_config, True, 5.0))
                if args.include_ablations and scale == "small":
                    cases.append(
                        _case_spec(args, scale, distribution, seed, "no_pricing_pruning", scale_config, False, 5.0)
                    )
                    cases.append(_case_spec(args, scale, distribution, seed, "no_root_extraction", scale_config, True, 0.0))
    return cases


def _case_spec(
    args: argparse.Namespace,
    scale: str,
    distribution: str,
    seed: int,
    variant: str,
    scale_config: dict[str, int],
    enable_pricing_pruning: bool,
    root_extraction_time_limit: float,
) -> dict[str, Any]:
    total_drones = scale_config["num_trucks"] * scale_config["drones_per_truck"]
    case_id = f"{scale}_{variant}_{distribution}_seed_{seed}"
    return {
        "case_id": case_id,
        "scale": scale,
        "variant": variant,
        "distribution": distribution,
        "seed": seed,
        "weights": args.weights,
        "case_time_limit_seconds": args.case_time_limit,
        "gurobi_python_path": args.gurobi_python_path,
        "total_drones": total_drones,
        "instance_config": {
            "seed": seed,
            "num_trucks": scale_config["num_trucks"],
            "num_customers": scale_config["num_customers"],
            "distribution": distribution,
            "drones_per_truck": scale_config["drones_per_truck"],
            "num_hubs": scale_config["num_hubs"],
            "service_deadline_mode": args.service_deadline_mode,
            "service_deadline_fraction": args.service_deadline_fraction,
            "service_deadline_manual_bounds": args.service_deadline_manual_bounds,
            "service_deadline_offset_min": args.service_deadline_offset_min,
            "service_deadline_offset_max": args.service_deadline_offset_max,
            "service_deadline_witness_slack": args.service_deadline_witness_slack,
            "service_deadline_random_seed": args.service_deadline_random_seed,
            "service_deadline_witness_method": args.service_deadline_witness_method,
            "service_deadline_witness_time_limit": args.service_deadline_witness_time_limit,
        },
        "solver_config": {
            "threads": args.threads,
            "time_limit": args.case_time_limit,
            "root_extraction_time_limit": root_extraction_time_limit,
            "enable_pricing_pruning": enable_pricing_pruning,
            "pricing_batch_size": args.pricing_batch_size,
            "min_pricing_batch_size": args.min_pricing_batch_size,
            "max_root_pricing_batch_size": args.max_root_pricing_batch_size,
            "nonroot_pricing_batch_size": args.nonroot_pricing_batch_size,
            "certification_pass_interval": args.certification_pass_interval,
            "pricing_yield_window": args.pricing_yield_window,
            "pricing_yield_low": args.pricing_yield_low,
            "pricing_yield_high": args.pricing_yield_high,
            "productive_pricing_slice_seconds": args.productive_pricing_slice_seconds,
            "productive_slice_min_seconds": args.productive_slice_min_seconds,
            "productive_slice_max_seconds": args.productive_slice_max_seconds,
            "productive_yield_window": args.productive_yield_window,
            "productive_yield_low_threshold": args.productive_yield_low_threshold,
            "productive_yield_high_threshold": args.productive_yield_high_threshold,
            "closure_attempt_batch_period": args.closure_attempt_batch_period,
            "closure_attempt_time_period": args.closure_attempt_time_period,
            "closure_certification_time_budget": args.closure_certification_time_budget,
            "closure_batch_size": args.closure_batch_size,
            "use_dual_stabilized_productive_search": args.use_dual_stabilized_productive_search,
            "dual_stabilization_weight": args.dual_stabilization_weight,
            "prefix_task_depth": args.prefix_task_depth,
            "productive_candidate_multiplier": args.productive_candidate_multiplier,
            "source_neighbor_task_size": args.source_neighbor_task_size,
            "pricing_diversity_batch_fraction": args.pricing_diversity_batch_fraction,
            "first_incumbent_route_pool_time_limit": args.first_incumbent_route_pool_time_limit,
            "post_incumbent_primal_budget_factor": args.post_incumbent_primal_budget_factor,
            "root_constructive_time_limit": args.root_constructive_time_limit,
            "enable_constructive_root_incumbent": not args.disable_constructive_root_incumbent,
            "root_compact_after_constructive": args.root_compact_after_constructive,
            "root_compact_time_limit_after_constructive": args.root_compact_time_limit_after_constructive,
            "root_compact_time_limit_without_constructive": args.root_compact_time_limit_without_constructive,
            "root_compact_wall_time_limit": args.root_compact_wall_time_limit,
            "root_compact_solve_time_limit": args.root_compact_solve_time_limit,
            "constructive_diversity_threshold": args.constructive_diversity_threshold,
            "constructive_incumbent_quality_threshold": args.constructive_incumbent_quality_threshold,
            "enable_drone_diversification_warm_start": args.enable_drone_diversification_warm_start,
            "enable_incremental_rmp": args.enable_incremental_rmp,
            "enable_active_coefficient_cache": args.enable_active_coefficient_cache,
            "enable_sr_aging": not args.disable_sr_aging,
            "enable_postroot_sr_cut_removal": not args.disable_postroot_sr_cut_removal,
            "sr_inactive_age_threshold": args.sr_inactive_age_threshold,
            "sr_removal_batch_size": args.sr_removal_batch_size,
            "sr_max_removals_per_node": args.sr_max_removals_per_node,
            "sr_reactivation_allowed": not args.disable_sr_reactivation,
            "sr_removal_min_active_count": args.sr_removal_min_active_count,
            "sr_removal_rmp_growth_threshold": args.sr_removal_rmp_growth_threshold,
            "sr_removal_build_time_threshold": args.sr_removal_build_time_threshold,
            "sr_removal_active_coeff_threshold": args.sr_removal_active_coeff_threshold,
            "sr_activity_tolerance": args.sr_activity_tolerance,
            "enable_global_branch_route_index": args.enable_global_branch_route_index,
            "enable_node_column_aging": args.enable_node_column_aging,
            "column_inactive_age_min": args.column_inactive_age_min,
            "column_active_value_tol": args.column_active_value_tol,
            "column_deactivation_min_active_columns": args.column_deactivation_min_active_columns,
            "column_deactivation_batch_size": args.column_deactivation_batch_size,
            "child_certification_slice_seconds": args.child_certification_slice_seconds,
            "child_certification_max_slices_per_node": args.child_certification_max_slices_per_node,
            "child_productive_before_certification": args.child_productive_before_certification,
            "enable_resumable_child_certification": args.enable_resumable_child_certification,
            "enable_child_closure_batch_adaptation": args.enable_child_closure_batch_adaptation,
            "child_closure_batch_min": args.child_closure_batch_min,
            "child_closure_batch_initial": args.child_closure_batch_initial,
            "child_closure_batch_max": args.child_closure_batch_max,
            "child_certification_yield_window": args.child_certification_yield_window,
            "child_certification_yield_low": args.child_certification_yield_low,
            "child_certification_yield_high": args.child_certification_yield_high,
            "child_cert_useful_yield_window": args.child_cert_useful_yield_window,
            "child_cert_no_route_yield_window": args.child_cert_no_route_yield_window,
            "child_cert_dual_stability_window": args.child_cert_dual_stability_window,
            "child_cert_dual_change_tol": args.child_cert_dual_change_tol,
            "child_closure_batch_growth_factor": args.child_closure_batch_growth_factor,
            "child_closure_batch_shrink_factor": args.child_closure_batch_shrink_factor,
            "child_useful_yield_low": args.child_useful_yield_low,
            "child_useful_yield_high": args.child_useful_yield_high,
            "child_no_route_yield_high": args.child_no_route_yield_high,
            "enable_rmp_basis_reuse": args.enable_rmp_basis_reuse,
            "sr_removal_density_weight": args.sr_removal_density_weight,
            "sr_removal_nnz_weight": args.sr_removal_nnz_weight,
            "sr_removal_age_weight": args.sr_removal_age_weight,
            "sr_removal_build_weight": args.sr_removal_build_weight,
            "sr_removal_violation_weight": args.sr_removal_violation_weight,
            "sr_removal_dual_weight": args.sr_removal_dual_weight,
            "sr_removal_score_threshold": args.sr_removal_score_threshold,
            "sr_removal_max_per_node": args.sr_removal_max_per_node,
            "use_row_local_sr_coeff_cache": not args.disable_row_local_sr_coeff_cache,
            "use_dominance_prefilter_keys": not args.disable_dominance_prefilter_keys,
            "use_promised_drone_construction": args.enable_promised_drone_construction,
            "promised_drone_construct_time_limit": args.promised_drone_construct_time_limit,
            "promised_drone_insert_top_k_customers": args.promised_drone_insert_top_k_customers,
            "promised_drone_insert_top_k_pads": args.promised_drone_insert_top_k_pads,
            "promised_drone_exchange_top_k_pairs": args.promised_drone_exchange_top_k_pairs,
            "promised_drone_min_improvement": args.promised_drone_min_improvement,
            "no_drone_incumbent_trigger": False,
            "compact_after_no_drone_incumbent": args.compact_after_no_drone_incumbent,
            "join_eval_budget": 0,
            "pricing_certification_slice_seconds": 0.0,
            "enable_join_lower_envelope": False,
            "join_generator_split_threshold": args.join_generator_split_threshold,
            "join_generator_pair_batch_size": args.join_generator_pair_batch_size,
            "enable_bucket_join_envelope": False,
            "enable_join_profile_cache": False,
            "small_join_pair_threshold": args.small_join_pair_threshold,
            "small_join_cumulative_threshold": args.small_join_cumulative_threshold,
            "max_join_bypass_calls": args.max_join_bypass_calls,
            "small_dom_bucket_threshold": args.small_dom_bucket_threshold,
            "small_dom_cumulative_threshold": args.small_dom_cumulative_threshold,
            "max_dom_bypass_calls": args.max_dom_bypass_calls,
            "root_max_side_pool_per_call": args.root_max_side_pool_per_call,
            "side_pool_max_size": args.side_pool_max_size,
            "side_pool_per_customer_keep": args.side_pool_per_customer_keep,
            "join_payload_bin_width": args.join_payload_bin_width,
            "repair_time_fraction_of_pricing": args.repair_time_fraction_of_pricing,
            "repair_time_hard_cap_root": args.repair_time_hard_cap_root,
            "repair_stall_limit": args.repair_stall_limit,
            "preclosure_pricing_call_interval": args.preclosure_pricing_call_interval,
            "preclosure_pool_growth_limit": args.preclosure_pool_growth_limit,
            "farkas_batch_size": args.farkas_batch_size,
            "seed_batch_size": args.seed_batch_size,
            "repair_batch_size": args.repair_batch_size,
            "enable_bidirectional_pricing": False,
            "pricing_parallel_workers": args.pricing_parallel_workers,
            "pricing_worker_backend": args.pricing_worker_backend,
            "prefix_task_depth_root": args.prefix_task_depth_root,
            "prefix_task_depth_child": args.prefix_task_depth_child,
            "prefix_task_min_branching_for_depth2": args.prefix_task_min_branching_for_depth2,
            "logging_mode": args.logging_mode,
            "progress_snapshot_period": args.progress_snapshot_period,
            "pricing_jsonl_enabled": not args.disable_pricing_jsonl,
            "gurobi_log_dir": None,
        },
    }


def _summary_row(record: dict[str, Any]) -> dict[str, Any]:
    instance = record.get("instance_config", {})
    metrics = record.get("service_metrics", {})
    components = record.get("objective_components", {})
    raw_components = components.get("raw", {})
    normalized_components = components.get("normalized", {})
    weighted_components = components.get("weighted_normalized", {})
    memory = record.get("memory") or {}
    bounds = record.get("normalization_bounds") or {}
    solver_config = record.get("solver_config") or {}
    progress = record.get("bpc_progress") or {}
    progress_node = progress.get("node") or {}
    stats = record.get("bpc_stats") or progress.get("stats") or {}
    worker_runtime = record.get("runtime_seconds")
    bpc_runtime = record.get("runtime")
    worker_overhead = None
    if worker_runtime is not None and bpc_runtime is not None:
        worker_overhead = worker_runtime - bpc_runtime
    root_extraction = stats.get("root_extraction_time")
    root_solver = stats.get("root_model_solve_time")
    root_compact_non_solver = None
    if root_extraction is not None and root_solver is not None:
        root_compact_non_solver = root_extraction - root_solver
    return {
        "case_id": record.get("case_id"),
        "scale": record.get("scale"),
        "variant": record.get("variant"),
        "distribution": record.get("distribution"),
        "seed": record.get("seed"),
        "customers": instance.get("num_customers"),
        "trucks": instance.get("num_trucks"),
        "hubs": instance.get("num_hubs"),
        "drones_per_truck": instance.get("drones_per_truck"),
        "service_deadline_mode": instance.get("service_deadline_mode"),
        "service_deadline_fraction": instance.get("service_deadline_fraction"),
        "service_deadline_manual_bound_count": (
            None
            if instance.get("service_deadline_manual_bounds") is None
            else len(instance.get("service_deadline_manual_bounds") or {})
        ),
        "service_deadline_offset_min": instance.get("service_deadline_offset_min"),
        "service_deadline_offset_max": instance.get("service_deadline_offset_max"),
        "service_deadline_witness_slack": instance.get("service_deadline_witness_slack"),
        "service_deadline_witness_method": bounds.get(
            "service_deadline_witness_method",
            instance.get("service_deadline_witness_method"),
        ),
        "service_deadline_witness_time_limit": instance.get("service_deadline_witness_time_limit"),
        "service_deadline_random_seed": bounds.get("service_deadline_random_seed", instance.get("service_deadline_random_seed")),
        "total_drones": instance.get("num_trucks", 0) * instance.get("drones_per_truck", 0) if instance else None,
        "status": record.get("status"),
        "solver_status": record.get("solver_status"),
        "objective_full": record.get("objective_full"),
        "lower_bound_shifted": record.get("lower_bound_shifted"),
        "lower_bound_full": record.get("lower_bound_full"),
        "upper_bound_shifted": record.get("upper_bound_shifted"),
        "upper_bound_full": record.get("upper_bound_full"),
        "gap": record.get("gap"),
        "gap_full": record.get("gap_full"),
        "gap_shifted": record.get("gap_shifted"),
        "raw_delay_square_sum": raw_components.get("delay_square_sum"),
        "raw_return_time_sum": raw_components.get("return_time_sum"),
        "raw_operating_cost": raw_components.get("operating_cost"),
        "normalized_delay": normalized_components.get("delay"),
        "normalized_return_time": normalized_components.get("return_time"),
        "normalized_cost": normalized_components.get("cost"),
        "weighted_delay": weighted_components.get("delay"),
        "weighted_return_time": weighted_components.get("return_time"),
        "weighted_cost": weighted_components.get("cost"),
        "runtime_seconds": record.get("runtime_seconds"),
        "bpc_runtime_seconds": bpc_runtime,
        "worker_overhead_seconds": worker_overhead,
        "instance_generation_time_seconds": record.get("instance_generation_time_seconds"),
        "bpc_wall_time_seconds": record.get("bpc_wall_time_seconds"),
        "service_metrics_time_seconds": record.get("service_metrics_time_seconds"),
        "output_write_time_seconds": record.get("output_write_time_seconds"),
        "python_current_allocated_mb": memory.get("python_current_allocated_mb"),
        "python_peak_allocated_mb": memory.get("python_peak_allocated_mb"),
        "nodes_processed": record.get("nodes_processed"),
        "rmp_solves": stats.get("rmp_solves"),
        "farkas_certificate_rhs": stats.get("farkas_certificate_rhs"),
        "farkas_certificate_max_column_activity": stats.get("farkas_certificate_max_column_activity"),
        "phase_i_solves": stats.get("phase_i_solves"),
        "phase_i_columns_added": stats.get("phase_i_columns_added"),
        "phase_i_uncovered_customers": stats.get("phase_i_uncovered_customers"),
        "seed_pricing_calls": stats.get("seed_pricing_calls"),
        "standard_pricing_calls": stats.get("standard_pricing_calls"),
        "farkas_pricing_calls": stats.get("farkas_pricing_calls"),
        "repair_pricing_calls": stats.get("repair_pricing_calls"),
        "columns_added_standard": stats.get("columns_added_standard"),
        "columns_added_farkas": stats.get("columns_added_farkas"),
        "duplicate_columns_rejected": stats.get("duplicate_columns_rejected"),
        "duplicate_merge_events": stats.get("duplicate_merge_events"),
        "sr_cuts_added": stats.get("sr_cuts_added"),
        "sr_cuts_added_root": stats.get("sr_cuts_added_root"),
        "sr_cuts_added_postroot": stats.get("sr_cuts_added_postroot"),
        "sr_cuts_aged": stats.get("sr_cuts_aged"),
        "sr_cuts_removed": stats.get("sr_cuts_removed"),
        "sr_removal_nodes": stats.get("sr_removal_nodes"),
        "sr_cuts_reactivated": stats.get("sr_cuts_reactivated"),
        "sr_cut_activity_updates": stats.get("sr_cut_activity_updates"),
        "sr_cut_dual_activity_updates": stats.get("sr_cut_dual_activity_updates"),
        "sr_cut_coefficient_nonzeros_observed": stats.get("sr_cut_coefficient_nonzeros_observed"),
        "sr_cut_coefficient_density_max": stats.get("sr_cut_coefficient_density_max"),
        "sr_cut_metadata_update_time": stats.get("sr_cut_metadata_update_time"),
        "sr_cut_repricing_after_removal": stats.get("sr_cut_repricing_after_removal"),
        "sr_cut_lower_bound_change_count": stats.get("sr_cut_lower_bound_change_count"),
        "sr_cut_lower_bound_change_sum": stats.get("sr_cut_lower_bound_change_sum"),
        "sr_cut_lower_bound_change_max_abs": stats.get("sr_cut_lower_bound_change_max_abs"),
        "sr_active_count_root": stats.get("sr_active_count_root"),
        "sr_active_count_postroot": stats.get("sr_active_count_postroot"),
        "pricing_labels_generated": stats.get("pricing_labels_generated"),
        "pricing_labels_dominated": stats.get("pricing_labels_dominated"),
        "pricing_labels_purged": stats.get("pricing_labels_purged"),
        "pricing_stale_labels_skipped": stats.get("pricing_stale_labels_skipped"),
        "pricing_labels_pruned": stats.get("pricing_labels_pruned"),
        "pricing_standard_bound_pruned": stats.get("pricing_standard_bound_pruned"),
        "pricing_farkas_bound_pruned": stats.get("pricing_farkas_bound_pruned"),
        "pricing_complete_routes_generated": stats.get("pricing_complete_routes_generated"),
        "pricing_max_queue_size": stats.get("pricing_max_queue_size"),
        "pricing_diagnostics_elapsed_seconds": stats.get("pricing_diagnostics_elapsed_seconds"),
        "service_deadline_active_count": bounds.get("service_deadline_active_count"),
        "min_service_deadline_slack": bounds.get("min_service_deadline_slack"),
        "service_deadline_setup_time": bounds.get("service_deadline_setup_time"),
        "service_deadline_witness_time": bounds.get("service_deadline_witness_time"),
        "service_deadline_witness_status": bounds.get("service_deadline_witness_status"),
        "service_deadline_witness_constructive_time": bounds.get("service_deadline_witness_constructive_time"),
        "service_deadline_witness_compact_time": bounds.get("service_deadline_witness_compact_time"),
        "service_deadline_witness_failed_customers": bounds.get("service_deadline_witness_failed_customers"),
        "num_witness_lifts": bounds.get("num_witness_lifts"),
        "max_witness_lift": bounds.get("max_witness_lift"),
        "mean_witness_lift": bounds.get("mean_witness_lift"),
        "pricing_extensions_attempted": stats.get("pricing_extensions_attempted"),
        "pricing_extensions_rejected_by_deadline": stats.get("pricing_extensions_rejected_by_deadline"),
        "pricing_forward_dominance_tests": stats.get("pricing_forward_dominance_tests"),
        "pricing_forward_same_node_dominance_tests": stats.get("pricing_forward_same_node_dominance_tests"),
        "pricing_forward_physical_location_dominance_tests": stats.get("pricing_forward_physical_location_dominance_tests"),
        "pricing_forward_physical_location_dominance_rejections": stats.get(
            "pricing_forward_physical_location_dominance_rejections"
        ),
        "pricing_forward_return_time_credit_checks": stats.get("pricing_forward_return_time_credit_checks"),
        "pricing_forward_return_time_credit_checks_skipped": stats.get(
            "pricing_forward_return_time_credit_checks_skipped"
        ),
        "pricing_forward_branch_language_failures": stats.get("pricing_forward_branch_language_failures"),
        "pricing_forward_mask_scalar_prefilter_failures": stats.get("pricing_forward_mask_scalar_prefilter_failures"),
        "pricing_dom_gate_pairs_seen": stats.get("pricing_dom_gate_pairs_seen"),
        "pricing_dom_gate_mask_failures": stats.get("pricing_dom_gate_mask_failures"),
        "pricing_dom_gate_scalar_failures": stats.get("pricing_dom_gate_scalar_failures"),
        "pricing_dom_gate_branch_failures": stats.get("pricing_dom_gate_branch_failures"),
        "pricing_dom_gate_deadline_failures": stats.get("pricing_dom_gate_deadline_failures"),
        "pricing_labels_dominated_same_node": stats.get("pricing_labels_dominated_same_node"),
        "pricing_labels_dominated_physical": stats.get("pricing_labels_dominated_physical"),
        "pricing_deadline_reachability_removed": stats.get("pricing_deadline_reachability_removed"),
        "pricing_reward_set_size_before_deadline": stats.get("pricing_reward_set_size_before_deadline"),
        "pricing_reward_set_size_after_deadline": stats.get("pricing_reward_set_size_after_deadline"),
        "pricing_deadline_reward_bound_calls": stats.get("pricing_deadline_reward_bound_calls"),
        "pricing_deadline_dominance_prefilter_skips": stats.get("pricing_deadline_dominance_prefilter_skips"),
        "pricing_routes_rejected_by_deadline_in_master": stats.get("pricing_routes_rejected_by_deadline_in_master"),
        "late_decoded_routes_rejected": stats.get("pricing_routes_rejected_by_deadline_in_master"),
        "late_rmp_routes_rejected": stats.get("late_rmp_routes_rejected"),
        "pricing_max_call_elapsed_seconds": stats.get("pricing_max_call_elapsed_seconds"),
        "pricing_forward_labels_generated": stats.get("pricing_forward_labels_generated"),
        "pricing_backward_labels_generated": stats.get("pricing_backward_labels_generated"),
        "pricing_backward_dominance_tests": stats.get("pricing_backward_dominance_tests"),
        "pricing_backward_labels_dominated": stats.get("pricing_backward_labels_dominated"),
        "pricing_backward_cost_function_build_time": stats.get("pricing_backward_cost_function_build_time"),
        "pricing_backward_cost_function_eval_time": stats.get("pricing_backward_cost_function_eval_time"),
        "pricing_join_sr_correction_time": stats.get("pricing_join_sr_correction_time"),
        "pricing_join_active_block_time": stats.get("pricing_join_active_block_time"),
        "pricing_joined_reduced_cost_evaluations": stats.get("pricing_joined_reduced_cost_evaluations"),
        "pricing_join_pairs_key_compatible": stats.get("pricing_join_pairs_key_compatible"),
        "pricing_join_pairs_after_bitset_filters": stats.get("pricing_join_pairs_after_bitset_filters"),
        "pricing_join_lower_envelope_rejects": stats.get("pricing_join_lower_envelope_rejects"),
        "pricing_join_bucket_lower_envelope_rejects": stats.get("pricing_join_bucket_lower_envelope_rejects"),
        "pricing_join_subbucket_lower_envelope_rejects": stats.get("pricing_join_subbucket_lower_envelope_rejects"),
        "pricing_join_pair_lower_envelope_rejects": stats.get("pricing_join_pair_lower_envelope_rejects"),
        "pricing_join_queue_pushes": stats.get("pricing_join_queue_pushes"),
        "pricing_join_queue_pops": stats.get("pricing_join_queue_pops"),
        "pricing_join_generator_queue_pushes": stats.get("pricing_join_generator_queue_pushes"),
        "pricing_join_generator_queue_pops": stats.get("pricing_join_generator_queue_pops"),
        "pricing_join_generator_splits": stats.get("pricing_join_generator_splits"),
        "pricing_join_materialized_pairs": stats.get("pricing_join_materialized_pairs"),
        "pricing_join_exact_rc_evals": stats.get("pricing_join_exact_rc_evals"),
        "pricing_join_exact_rc_time": stats.get("pricing_join_exact_rc_time"),
        "pricing_interface_cache_hits": stats.get("pricing_interface_cache_hits"),
        "pricing_interface_cache_misses": stats.get("pricing_interface_cache_misses"),
        "pricing_suffix_profile_cache_hits": stats.get("pricing_suffix_profile_cache_hits"),
        "pricing_suffix_profile_cache_misses": stats.get("pricing_suffix_profile_cache_misses"),
        "pricing_interface_profile_cache_hits": stats.get("pricing_interface_profile_cache_hits"),
        "pricing_interface_profile_cache_misses": stats.get("pricing_interface_profile_cache_misses"),
        "pricing_negative_routes_verified": stats.get("pricing_negative_routes_verified"),
        "pricing_negative_routes_inserted": stats.get("pricing_negative_routes_inserted"),
        "pricing_backward_dominance_cost_tests": stats.get("pricing_backward_dominance_cost_tests"),
        "pricing_backward_dominance_cost_rejected": stats.get("pricing_backward_dominance_cost_rejected"),
        "pricing_backward_exclusive_resource_violations": stats.get("pricing_backward_exclusive_resource_violations"),
        "pricing_join_pairs_tested": stats.get("pricing_join_pairs_tested"),
        "pricing_joined_routes_accepted": stats.get("pricing_joined_routes_accepted"),
        "pricing_forward_labeling_time": stats.get("pricing_forward_labeling_time"),
        "pricing_backward_labeling_time": stats.get("pricing_backward_labeling_time"),
        "pricing_join_time": stats.get("pricing_join_time"),
        "pricing_parallel_calls": stats.get("pricing_parallel_calls"),
        "pricing_parallel_workers_max": stats.get("pricing_parallel_workers_max"),
        "prefix_task_depth_root_config": solver_config.get("prefix_task_depth_root"),
        "prefix_task_depth_child_config": solver_config.get("prefix_task_depth_child"),
        "prefix_task_min_branching_for_depth2": solver_config.get("prefix_task_min_branching_for_depth2"),
        "logging_mode": solver_config.get("logging_mode"),
        "progress_snapshot_period": solver_config.get("progress_snapshot_period"),
        "pricing_jsonl_enabled": solver_config.get("pricing_jsonl_enabled"),
        "pricing_process_cpu_time": stats.get("pricing_process_cpu_time"),
        "pricing_cpu_core_equivalent_max": stats.get("pricing_cpu_core_equivalent_max"),
        "pricing_cpu_core_equivalent_weighted": stats.get("pricing_cpu_core_equivalent_weighted"),
        "pricing_worker_cpu_time": stats.get("pricing_worker_cpu_time"),
        "pricing_main_process_cpu_time": stats.get("pricing_main_process_cpu_time"),
        "pricing_main_merge_time": stats.get("pricing_main_merge_time"),
        "pricing_pool_startup_time": stats.get("pricing_pool_startup_time"),
        "pricing_pool_startup_count": stats.get("pricing_pool_startup_count"),
        "pricing_pool_reused_calls": stats.get("pricing_pool_reused_calls"),
        "pricing_pool_shutdown_time": stats.get("pricing_pool_shutdown_time"),
        "pricing_task_submission_time": stats.get("pricing_task_submission_time"),
        "pricing_worker_payload_count": stats.get("pricing_worker_payload_count"),
        "pricing_worker_response_count": stats.get("pricing_worker_response_count"),
        "pricing_candidate_paths_before_merge": stats.get("pricing_candidate_paths_before_merge"),
        "pricing_candidate_paths_after_merge": stats.get("pricing_candidate_paths_after_merge"),
        "pricing_decoded_routes_in_main": stats.get("pricing_decoded_routes_in_main"),
        "pricing_verified_routes_in_main": stats.get("pricing_verified_routes_in_main"),
        "pricing_batch_target_max": stats.get("pricing_batch_target_max"),
        "pricing_returned_batch_size_max": stats.get("pricing_returned_batch_size_max"),
        "pricing_first_hit_enabled_calls": stats.get("pricing_first_hit_enabled_calls"),
        "pricing_stale_response_rejections": stats.get("pricing_stale_response_rejections"),
        "pricing_worker_backend_thread_calls": stats.get("pricing_worker_backend_thread_calls"),
        "pricing_worker_backend_process_calls": stats.get("pricing_worker_backend_process_calls"),
        "pricing_source_neighbor_count_max": stats.get("pricing_source_neighbor_count_max"),
        "pricing_source_neighbor_task_count_max": stats.get("pricing_source_neighbor_task_count_max"),
        "pricing_source_neighbor_task_size_max": stats.get("pricing_source_neighbor_task_size_max"),
        "pricing_local_worker_candidate_quota_max": stats.get("pricing_local_worker_candidate_quota_max"),
        "pricing_diversity_quota_max": stats.get("pricing_diversity_quota_max"),
        "pricing_diversity_selected_routes": stats.get("pricing_diversity_selected_routes"),
        "pricing_diversity_selected_customers_max": stats.get("pricing_diversity_selected_customers_max"),
        "pricing_productive_slice_deadline_calls": stats.get("pricing_productive_slice_deadline_calls"),
        "pricing_productive_slice_time": stats.get("pricing_productive_slice_time"),
        "pricing_productive_time_limit_with_columns": stats.get("pricing_productive_time_limit_with_columns"),
        "pricing_productive_time_limit_no_columns": stats.get("pricing_productive_time_limit_no_columns"),
        "pricing_first_hit_exits": stats.get("pricing_first_hit_exits"),
        "pricing_interrupted_worker_calls": stats.get("pricing_interrupted_worker_calls"),
        "pricing_certification_worker_calls": stats.get("pricing_certification_worker_calls"),
        "pricing_productive_worker_calls": stats.get("pricing_productive_worker_calls"),
        "pricing_signature_cache_hits": stats.get("pricing_signature_cache_hits"),
        "pricing_signature_cache_misses": stats.get("pricing_signature_cache_misses"),
        "pricing_core_signature_cache_hits": stats.get("pricing_core_signature_cache_hits"),
        "pricing_core_signature_cache_misses": stats.get("pricing_core_signature_cache_misses"),
        "pricing_active_signature_cache_hits": stats.get("pricing_active_signature_cache_hits"),
        "pricing_active_signature_cache_misses": stats.get("pricing_active_signature_cache_misses"),
        "pricing_sr_coeff_cache_hits": stats.get("pricing_sr_coeff_cache_hits"),
        "pricing_sr_coeff_cache_misses": stats.get("pricing_sr_coeff_cache_misses"),
        "pricing_active_sr_key_cache_hits": stats.get("pricing_active_sr_key_cache_hits"),
        "pricing_active_sr_key_cache_misses": stats.get("pricing_active_sr_key_cache_misses"),
        "pricing_active_sr_coeffs_computed": stats.get("pricing_active_sr_coeffs_computed"),
        "pricing_triplet_masks_built": stats.get("pricing_triplet_masks_built"),
        "pricing_dominance_prefilter_pairs": stats.get("pricing_dominance_prefilter_pairs"),
        "pricing_dominance_prefilter_rejected": stats.get("pricing_dominance_prefilter_rejected"),
        "pricing_dominance_bucket_pairs_considered": stats.get("pricing_dominance_bucket_pairs_considered"),
        "pricing_dominance_bucket_pairs_rejected": stats.get("pricing_dominance_bucket_pairs_rejected"),
        "pricing_dominance_bucket_candidate_pairs": stats.get("pricing_dominance_bucket_candidate_pairs"),
        "pricing_dominance_compatible_keys_generated": stats.get("pricing_dominance_compatible_keys_generated"),
        "pricing_dominance_compatible_key_lookups": stats.get("pricing_dominance_compatible_key_lookups"),
        "pricing_dominance_bucket_scans_avoided": stats.get("pricing_dominance_bucket_scans_avoided"),
        "pricing_dominance_key_generation_time": stats.get("pricing_dominance_key_generation_time"),
        "pricing_dominance_key_cache_hits": stats.get("pricing_dominance_key_cache_hits"),
        "pricing_dominance_key_cache_misses": stats.get("pricing_dominance_key_cache_misses"),
        "pricing_dominance_small_bypass_calls": stats.get("pricing_dominance_small_bypass_calls"),
        "pricing_dominance_bypass_calls": stats.get("pricing_dominance_bypass_calls"),
        "pricing_dominance_indexed_activation_count": stats.get("pricing_dominance_indexed_activation_count"),
        "pricing_dominance_work_estimate": stats.get("pricing_dominance_work_estimate"),
        "pricing_sticky_indexed_dominance_activations": stats.get("pricing_sticky_indexed_dominance_activations"),
        "pricing_dominance_stage_reject_key": stats.get("pricing_dominance_stage_reject_key"),
        "pricing_dominance_stage_reject_branch": stats.get("pricing_dominance_stage_reject_branch"),
        "pricing_dominance_stage_reject_customer": stats.get("pricing_dominance_stage_reject_customer"),
        "pricing_dominance_stage_reject_truck_node": stats.get("pricing_dominance_stage_reject_truck_node"),
        "pricing_dominance_stage_reject_payload": stats.get("pricing_dominance_stage_reject_payload"),
        "pricing_dominance_stage_reject_block": stats.get("pricing_dominance_stage_reject_block"),
        "pricing_dominance_stage_reject_time": stats.get("pricing_dominance_stage_reject_time"),
        "pricing_dominance_stage_reject_cost": stats.get("pricing_dominance_stage_reject_cost"),
        "pricing_backward_full_dominance_tests": stats.get("pricing_backward_full_dominance_tests"),
        "pricing_join_prefilter_pairs": stats.get("pricing_join_prefilter_pairs"),
        "pricing_join_prefilter_rejected": stats.get("pricing_join_prefilter_rejected"),
        "pricing_join_bucket_pairs_considered": stats.get("pricing_join_bucket_pairs_considered"),
        "pricing_join_bucket_pairs_rejected": stats.get("pricing_join_bucket_pairs_rejected"),
        "pricing_join_bucket_candidate_pairs": stats.get("pricing_join_bucket_candidate_pairs"),
        "pricing_join_compatible_keys_generated": stats.get("pricing_join_compatible_keys_generated"),
        "pricing_join_compatible_key_lookups": stats.get("pricing_join_compatible_key_lookups"),
        "pricing_join_bucket_scans_avoided": stats.get("pricing_join_bucket_scans_avoided"),
        "pricing_join_key_generation_time": stats.get("pricing_join_key_generation_time"),
        "pricing_join_key_cache_hits": stats.get("pricing_join_key_cache_hits"),
        "pricing_join_key_cache_misses": stats.get("pricing_join_key_cache_misses"),
        "pricing_join_graph_build_time": stats.get("pricing_join_graph_build_time"),
        "pricing_join_subbucket_pairs_considered": stats.get("pricing_join_subbucket_pairs_considered"),
        "pricing_join_subbucket_pairs_rejected": stats.get("pricing_join_subbucket_pairs_rejected"),
        "pricing_join_small_bypass_calls": stats.get("pricing_join_small_bypass_calls"),
        "pricing_join_local_bypass_calls": stats.get("pricing_join_local_bypass_calls"),
        "pricing_join_cumulative_bypass_calls": stats.get("pricing_join_cumulative_bypass_calls"),
        "pricing_join_indexed_activation_count": stats.get("pricing_join_indexed_activation_count"),
        "pricing_join_work_estimate": stats.get("pricing_join_work_estimate"),
        "pricing_sticky_indexed_join_activations": stats.get("pricing_sticky_indexed_join_activations"),
        "pricing_join_stage_reject_key": stats.get("pricing_join_stage_reject_key"),
        "pricing_join_stage_reject_branch": stats.get("pricing_join_stage_reject_branch"),
        "pricing_join_stage_reject_customer": stats.get("pricing_join_stage_reject_customer"),
        "pricing_join_stage_reject_truck_node": stats.get("pricing_join_stage_reject_truck_node"),
        "pricing_join_stage_reject_payload": stats.get("pricing_join_stage_reject_payload"),
        "pricing_join_stage_reject_block": stats.get("pricing_join_stage_reject_block"),
        "pricing_join_stage_reject_reduced_cost": stats.get("pricing_join_stage_reject_reduced_cost"),
        "pricing_join_candidate_pairs_accepted": stats.get("pricing_join_candidate_pairs_accepted"),
        "pricing_join_label_pairs_materialized": stats.get("pricing_join_label_pairs_materialized"),
        "pricing_join_full_decodes": stats.get("pricing_join_full_decodes"),
        "pricing_lazy_rejected_before_decode": stats.get("pricing_lazy_rejected_before_decode"),
        "pricing_fully_decoded_routes": stats.get("pricing_fully_decoded_routes"),
        "pricing_duplicate_equivalent_rejected": stats.get("pricing_duplicate_equivalent_rejected"),
        "pricing_cost_dominated_rejected": stats.get("pricing_cost_dominated_rejected"),
        "pricing_signature_build_time": stats.get("pricing_signature_build_time"),
        "pricing_sr_coeff_build_time": stats.get("pricing_sr_coeff_build_time"),
        "pricing_duplicate_lookup_time": stats.get("pricing_duplicate_lookup_time"),
        "pricing_route_decode_time": stats.get("pricing_route_decode_time"),
        "pricing_reduced_cost_verification_time": stats.get("pricing_reduced_cost_verification_time"),
        "pricing_side_pool_routes_returned": stats.get("pricing_side_pool_routes_returned"),
        "pricing_side_pool_reduced_cost_min": stats.get("pricing_side_pool_reduced_cost_min"),
        "pricing_side_pool_candidates_seen": stats.get("pricing_side_pool_candidates_seen"),
        "pricing_side_pool_routes_retained": stats.get("pricing_side_pool_routes_retained"),
        "pricing_side_pool_routes_rejected_by_budget": stats.get("pricing_side_pool_routes_rejected_by_budget"),
        "route_pool_hydration_time": stats.get("route_pool_hydration_time"),
        "root_constructive_time": stats.get("root_constructive_time"),
        "root_constructive_status": stats.get("root_constructive_status"),
        "root_constructive_routes": stats.get("root_constructive_routes"),
        "root_constructive_incumbent_found": stats.get("root_constructive_incumbent_found"),
        "root_constructive_diversity_score": stats.get("root_constructive_diversity_score"),
        "root_constructive_drone_sorties": stats.get("root_constructive_drone_sorties"),
        "root_constructive_truck_count": stats.get("root_constructive_truck_count"),
        "root_constructive_value": stats.get("root_constructive_value"),
        "root_compact_attempted": stats.get("root_compact_attempted"),
        "root_compact_warm_start_only": stats.get("root_compact_warm_start_only"),
        "root_compact_accepted_columns": stats.get("root_compact_accepted_columns"),
        "root_compact_skipped_reason": stats.get("root_compact_skipped_reason"),
        "root_compact_budget_seconds": stats.get("root_compact_budget_seconds"),
        "root_compact_wall_budget_seconds": stats.get("root_compact_wall_budget_seconds"),
        "root_compact_solve_budget_seconds": stats.get("root_compact_solve_budget_seconds"),
        "root_compact_wall_budget_hit": stats.get("root_compact_wall_budget_hit"),
        "root_compact_decode_verification_time": stats.get("root_compact_decode_verification_time"),
        "root_compact_conditional_triggered": stats.get("root_compact_conditional_triggered"),
        "root_compact_conditional_reason": stats.get("root_compact_conditional_reason"),
        "drone_diversification_attempted": stats.get("drone_diversification_attempted"),
        "drone_diversification_routes_generated": stats.get("drone_diversification_routes_generated"),
        "drone_diversification_columns_accepted": stats.get("drone_diversification_columns_accepted"),
        "drone_diversification_incumbent_improved": stats.get("drone_diversification_incumbent_improved"),
        "drone_diversification_time": stats.get("drone_diversification_time"),
        "route_signature_build_time": stats.get("route_signature_build_time"),
        "sr_coeff_build_time": stats.get("sr_coeff_build_time"),
        "duplicate_lookup_time": stats.get("duplicate_lookup_time"),
        "rmp_column_insertion_time": stats.get("rmp_column_insertion_time"),
        "rmp_build_time": stats.get("rmp_build_time"),
        "rmp_solve_time": stats.get("rmp_solve_time"),
        "rmp_incremental_updates": stats.get("rmp_incremental_updates"),
        "rmp_full_rebuilds": stats.get("rmp_full_rebuilds"),
        "rmp_incremental_update_time": stats.get("rmp_incremental_update_time"),
        "rmp_active_coefficient_cache_hits": stats.get("rmp_active_coefficient_cache_hits"),
        "rmp_active_coefficient_cache_misses": stats.get("rmp_active_coefficient_cache_misses"),
        "rmp_active_sr_nonzero_count": stats.get("rmp_active_sr_nonzero_count"),
        "rmp_active_sr_coefficient_count": stats.get("rmp_active_sr_coefficient_count"),
        "rmp_active_sr_nonzero_density_max": stats.get("rmp_active_sr_nonzero_density_max"),
        "rmp_compatibility_failure_residual_customers": stats.get("rmp_compatibility_failure_residual_customers"),
        "rmp_compatibility_failure_fixed_routes": stats.get("rmp_compatibility_failure_fixed_routes"),
        "rmp_compatibility_failure_fleet_limit": stats.get("rmp_compatibility_failure_fleet_limit"),
        "rmp_compatibility_failure_fixed_cost": stats.get("rmp_compatibility_failure_fixed_cost"),
        "rmp_compatibility_failure_branch_state": stats.get("rmp_compatibility_failure_branch_state"),
        "rmp_compatibility_failure_active_sr": stats.get("rmp_compatibility_failure_active_sr"),
        "rmp_compatibility_failure_active_sr_version": stats.get("rmp_compatibility_failure_active_sr_version"),
        "rmp_compatibility_failure_service_deadline_version": stats.get(
            "rmp_compatibility_failure_service_deadline_version"
        ),
        "rmp_compatibility_failure_objective_scale_version": stats.get(
            "rmp_compatibility_failure_objective_scale_version"
        ),
        "max_positive_le_dual_violation": stats.get("max_positive_le_dual_violation"),
        "max_negative_ge_dual_violation": stats.get("max_negative_ge_dual_violation"),
        "sr_dual_sign_violations": stats.get("sr_dual_sign_violations"),
        "fleet_dual_sign_violations": stats.get("fleet_dual_sign_violations"),
        "rmp_basis_reuse_attempts": stats.get("rmp_basis_reuse_attempts"),
        "rmp_basis_reuse_success": stats.get("rmp_basis_reuse_success"),
        "rmp_basis_reuse_time": stats.get("rmp_basis_reuse_time"),
        "rmp_basis_store_time": stats.get("rmp_basis_store_time"),
        "column_hydration_time": stats.get("column_hydration_time"),
        "column_signature_lookup_time": stats.get("column_signature_lookup_time"),
        "progress_serialization_time": stats.get("progress_serialization_time"),
        "progress_events_seen": stats.get("progress_events_seen"),
        "progress_events_written": stats.get("progress_events_written"),
        "progress_events_skipped": stats.get("progress_events_skipped"),
        "signature_cache_hits": stats.get("signature_cache_hits"),
        "signature_cache_misses": stats.get("signature_cache_misses"),
        "core_signature_cache_hits": stats.get("core_signature_cache_hits"),
        "core_signature_cache_misses": stats.get("core_signature_cache_misses"),
        "active_signature_cache_hits": stats.get("active_signature_cache_hits"),
        "active_signature_cache_misses": stats.get("active_signature_cache_misses"),
        "sr_coeff_cache_hits": stats.get("sr_coeff_cache_hits"),
        "sr_coeff_cache_misses": stats.get("sr_coeff_cache_misses"),
        "active_sr_key_cache_hits": stats.get("active_sr_key_cache_hits"),
        "active_sr_key_cache_misses": stats.get("active_sr_key_cache_misses"),
        "active_sr_coeffs_computed": stats.get("active_sr_coeffs_computed"),
        "triplet_masks_built": stats.get("triplet_masks_built"),
        "column_index_hits": stats.get("column_index_hits"),
        "column_index_misses": stats.get("column_index_misses"),
        "column_index_replacements": stats.get("column_index_replacements"),
        "column_index_refreshes": stats.get("column_index_refreshes"),
        "column_index_refresh_time": stats.get("column_index_refresh_time"),
        "active_sr_version_refreshes": stats.get("active_sr_version_refreshes"),
        "active_sr_version_refresh_time": stats.get("active_sr_version_refresh_time"),
        "duplicate_equivalent_columns_rejected": stats.get("duplicate_equivalent_columns_rejected"),
        "cost_dominated_columns_rejected": stats.get("cost_dominated_columns_rejected"),
        "cost_dominated_columns_removed": stats.get("cost_dominated_columns_removed"),
        "total_routes": stats.get("total_routes"),
        "global_pool_routes": stats.get("global_pool_routes"),
        "max_node_columns": stats.get("max_node_columns"),
        "max_active_sr": stats.get("max_active_sr"),
        "max_active_sr_version": stats.get("max_active_sr_version"),
        "branching_nodes": stats.get("branching_nodes"),
        "child_nodes_created": stats.get("child_nodes_created"),
        "child_inherited_route_candidates": stats.get("child_inherited_route_candidates"),
        "child_inherited_route_accepted": stats.get("child_inherited_route_accepted"),
        "child_inherited_route_rejected": stats.get("child_inherited_route_rejected"),
        "child_branch_index_build_time": stats.get("child_branch_index_build_time"),
        "child_branch_index_query_time": stats.get("child_branch_index_query_time"),
        "child_branch_index_candidates_before": stats.get("child_branch_index_candidates_before"),
        "child_branch_index_candidates_after": stats.get("child_branch_index_candidates_after"),
        "child_branch_index_reject_route_forbidden": stats.get("child_branch_index_reject_route_forbidden"),
        "child_branch_index_reject_together": stats.get("child_branch_index_reject_together"),
        "child_branch_index_reject_separate": stats.get("child_branch_index_reject_separate"),
        "child_branch_index_reject_service_mode": stats.get("child_branch_index_reject_service_mode"),
        "child_branch_index_reject_launch_pad": stats.get("child_branch_index_reject_launch_pad"),
        "child_branch_index_reject_transformed_arc": stats.get("child_branch_index_reject_transformed_arc"),
        "global_branch_index_builds": stats.get("global_branch_index_builds"),
        "global_branch_index_incremental_updates": stats.get("global_branch_index_incremental_updates"),
        "global_branch_index_incremental_paths": stats.get("global_branch_index_incremental_paths"),
        "global_branch_index_queries": stats.get("global_branch_index_queries"),
        "global_branch_index_build_time": stats.get("global_branch_index_build_time"),
        "global_branch_index_query_time": stats.get("global_branch_index_query_time"),
        "global_branch_index_candidates": stats.get("global_branch_index_candidates"),
        "global_branch_index_rejections": stats.get("global_branch_index_rejections"),
        "child_reject_residual": stats.get("child_reject_residual"),
        "child_reject_fixed_route": stats.get("child_reject_fixed_route"),
        "child_reject_fleet": stats.get("child_reject_fleet"),
        "child_reject_branch": stats.get("child_reject_branch"),
        "child_reject_deadline": stats.get("child_reject_deadline"),
        "child_reject_sr_signature": stats.get("child_reject_sr_signature"),
        "child_reject_route_signature": stats.get("child_reject_route_signature"),
        "child_hydration_time": stats.get("child_hydration_time"),
        "child_refresh_count": stats.get("child_refresh_count"),
        "child_refresh_time": stats.get("child_refresh_time"),
        "postroot_nodes_processed": stats.get("postroot_nodes_processed"),
        "postroot_nodes_closed": stats.get("postroot_nodes_closed"),
        "child_certification_calls": stats.get("child_certification_calls"),
        "child_certification_calls_with_columns": stats.get("child_certification_calls_with_columns"),
        "child_certification_calls_exhausted": stats.get("child_certification_calls_exhausted"),
        "child_certification_time_limited": stats.get("child_certification_time_limited"),
        "child_certification_slice_seconds": stats.get("child_certification_slice_seconds"),
        "child_certification_slice_limited_calls": stats.get("child_certification_slice_limited_calls"),
        "child_certification_resumed_calls": stats.get("child_certification_resumed_calls"),
        "child_certification_state_discards": stats.get("child_certification_state_discards"),
        "child_certification_epochs_started": stats.get("child_certification_epochs_started"),
        "child_certification_epochs_completed": stats.get("child_certification_epochs_completed"),
        "child_certification_state_saved": stats.get("child_certification_state_saved"),
        "child_certification_state_resumed": stats.get("child_certification_state_resumed"),
        "child_certification_state_discarded_by_dual": stats.get("child_certification_state_discarded_by_dual"),
        "child_certification_state_discarded_by_sr": stats.get("child_certification_state_discarded_by_sr"),
        "child_certification_state_discarded_by_residual": stats.get(
            "child_certification_state_discarded_by_residual"
        ),
        "child_certification_state_discarded_by_branch": stats.get("child_certification_state_discarded_by_branch"),
        "child_certification_state_discarded_by_fixed_routes": stats.get(
            "child_certification_state_discarded_by_fixed_routes"
        ),
        "child_certification_state_discarded_by_active_columns": stats.get(
            "child_certification_state_discarded_by_active_columns"
        ),
        "child_certification_exhausted_tasks": stats.get("child_certification_exhausted_tasks"),
        "child_certification_unresolved_tasks": stats.get("child_certification_unresolved_tasks"),
        "child_closure_batch_min": stats.get("child_closure_batch_min"),
        "child_closure_batch_max": stats.get("child_closure_batch_max"),
        "child_closure_batch_last": stats.get("child_closure_batch_last"),
        "child_closure_batch_increases": stats.get("child_closure_batch_increases"),
        "child_closure_batch_decreases": stats.get("child_closure_batch_decreases"),
        "child_certification_yield_rate": stats.get("child_certification_yield_rate"),
        "child_certification_yield_observations": stats.get("child_certification_yield_observations"),
        "postroot_open_nodes": stats.get("postroot_open_nodes"),
        "best_open_bound": stats.get("best_open_bound"),
        "customer_pair_branches": stats.get("customer_pair_branches"),
        "service_mode_branches": stats.get("service_mode_branches"),
        "launch_pad_branches": stats.get("launch_pad_branches"),
        "transformed_arc_branches": stats.get("transformed_arc_branches"),
        "route_variable_branches": stats.get("route_variable_branches"),
        "root_closed": stats.get("root_closed"),
        "best_reduced_cost_at_stop": stats.get("best_reduced_cost_at_stop"),
        "adaptive_batch_size_min": stats.get("adaptive_batch_size_min"),
        "adaptive_batch_size_max": stats.get("adaptive_batch_size_max"),
        "adaptive_batch_size_last": stats.get("adaptive_batch_size_last"),
        "adaptive_batch_increases": stats.get("adaptive_batch_increases"),
        "adaptive_batch_decreases": stats.get("adaptive_batch_decreases"),
        "adaptive_batch_window_ratio": stats.get("adaptive_batch_window_ratio"),
        "adaptive_batch_low_windows": stats.get("adaptive_batch_low_windows"),
        "adaptive_batch_high_windows": stats.get("adaptive_batch_high_windows"),
        "certification_pricing_passes": stats.get("certification_pricing_passes"),
        "certification_pricing_passes_with_columns": stats.get("certification_pricing_passes_with_columns"),
        "root_productive_pricing_calls": stats.get("root_productive_pricing_calls"),
        "pricing_productive_mode_calls": stats.get("pricing_productive_mode_calls"),
        "pricing_closure_mode_calls": stats.get("pricing_closure_mode_calls"),
        "pricing_yield_ratio": stats.get("pricing_yield_ratio"),
        "closure_attempts": stats.get("closure_attempts"),
        "closure_attempts_returned_columns": stats.get("closure_attempts_returned_columns"),
        "closure_attempts_exhausted": stats.get("closure_attempts_exhausted"),
        "closure_attempts_time_limited": stats.get("closure_attempts_time_limited"),
        "productive_batches_since_last_cert": stats.get("productive_batches_since_last_cert"),
        "productive_time_since_last_cert": stats.get("productive_time_since_last_cert"),
        "certification_time_limit_with_columns": stats.get("certification_time_limit_with_columns"),
        "certification_time_limit_no_columns": stats.get("certification_time_limit_no_columns"),
        "sr_removal_trigger_active_count_failures": stats.get("sr_removal_trigger_active_count_failures"),
        "sr_removal_trigger_growth_failures": stats.get("sr_removal_trigger_growth_failures"),
        "sr_removal_trigger_build_burden_failures": stats.get("sr_removal_trigger_build_burden_failures"),
        "sr_removal_trigger_active_coeff_failures": stats.get("sr_removal_trigger_active_coeff_failures"),
        "sr_removal_trigger_activity_failures": stats.get("sr_removal_trigger_activity_failures"),
        "sr_removal_candidates": stats.get("sr_removal_candidates"),
        "sr_removal_candidate_marks": stats.get("sr_removal_candidate_marks"),
        "sr_removal_score_max": stats.get("sr_removal_score_max"),
        "sr_removal_rmp_growth_max": stats.get("sr_removal_rmp_growth_max"),
        "sr_removal_rmp_build_growth_max": stats.get("sr_removal_rmp_build_growth_max"),
        "inactive_columns_deactivated": stats.get("inactive_columns_deactivated"),
        "inactive_columns_rehydrated": stats.get("inactive_columns_rehydrated"),
        "inactive_column_deactivation_events": stats.get("inactive_column_deactivation_events"),
        "inactive_column_rehydration_events": stats.get("inactive_column_rehydration_events"),
        "inactive_column_reduced_cost_checks": stats.get("inactive_column_reduced_cost_checks"),
        "inactive_column_reduced_cost_time": stats.get("inactive_column_reduced_cost_time"),
        "inactive_column_lower_bound_change_count": stats.get("inactive_column_lower_bound_change_count"),
        "inactive_column_lower_bound_change_sum": stats.get("inactive_column_lower_bound_change_sum"),
        "inactive_column_lower_bound_change_max_abs": stats.get("inactive_column_lower_bound_change_max_abs"),
        "inactive_columns_at_stop": stats.get("inactive_columns_at_stop"),
        "adaptive_slice_seconds_min": stats.get("adaptive_slice_seconds_min"),
        "adaptive_slice_seconds_max": stats.get("adaptive_slice_seconds_max"),
        "adaptive_slice_seconds_last": stats.get("adaptive_slice_seconds_last"),
        "adaptive_slice_increases": stats.get("adaptive_slice_increases"),
        "adaptive_slice_decreases": stats.get("adaptive_slice_decreases"),
        "productive_yield_window_rate": stats.get("productive_yield_window_rate"),
        "prefix_task_depth": stats.get("prefix_task_depth"),
        "stabilized_dual_enabled": stats.get("stabilized_dual_enabled"),
        "stabilized_candidates_returned": stats.get("stabilized_candidates_returned"),
        "true_dual_rejected_candidates": stats.get("true_dual_rejected_candidates"),
        "max_abs_worker_true_rc_discrepancy": stats.get("max_abs_worker_true_rc_discrepancy"),
        "post_incumbent_heuristic_budget": stats.get("post_incumbent_heuristic_budget"),
        "post_incumbent_repair_budget": stats.get("post_incumbent_repair_budget"),
        "root_extraction_time": stats.get("root_extraction_time"),
        "root_compact_status": stats.get("root_compact_status"),
        "root_model_build_time": stats.get("root_model_build_time"),
        "root_model_solve_time": stats.get("root_model_solve_time"),
        "root_route_decode_time": stats.get("root_route_decode_time"),
        "root_compact_non_solver_time": root_compact_non_solver,
        "rmp_time": stats.get("rmp_time"),
        "root_rmp_time": stats.get("root_rmp_time"),
        "postroot_rmp_time": stats.get("postroot_rmp_time"),
        "phase_i_time": stats.get("phase_i_time"),
        "seed_pricing_time": stats.get("seed_pricing_time"),
        "standard_pricing_time": stats.get("standard_pricing_time"),
        "root_standard_pricing_time": stats.get("root_standard_pricing_time"),
        "postroot_standard_pricing_time": stats.get("postroot_standard_pricing_time"),
        "farkas_pricing_time": stats.get("farkas_pricing_time"),
        "root_farkas_pricing_time": stats.get("root_farkas_pricing_time"),
        "postroot_farkas_pricing_time": stats.get("postroot_farkas_pricing_time"),
        "sr_separation_time": stats.get("sr_separation_time"),
        "root_sr_separation_time": stats.get("root_sr_separation_time"),
        "postroot_sr_separation_time": stats.get("postroot_sr_separation_time"),
        "repair_pricing_time": stats.get("repair_pricing_time"),
        "heuristic_time": stats.get("heuristic_time"),
        "root_heuristic_time": stats.get("root_heuristic_time"),
        "postroot_heuristic_time": stats.get("postroot_heuristic_time"),
        "heuristic_calls": stats.get("heuristic_calls"),
        "preclosure_heuristic_calls": stats.get("preclosure_heuristic_calls"),
        "preclosure_heuristic_columns_generated": stats.get("preclosure_heuristic_columns_generated"),
        "preclosure_heuristic_incumbent_updates": stats.get("preclosure_heuristic_incumbent_updates"),
        "time_to_first_incumbent": stats.get("time_to_first_incumbent"),
        "first_incumbent_route_pool_calls": stats.get("first_incumbent_route_pool_calls"),
        "first_incumbent_route_pool_time": stats.get("first_incumbent_route_pool_time"),
        "first_incumbent_route_pool_feasible_solves": stats.get("first_incumbent_route_pool_feasible_solves"),
        "first_incumbent_route_pool_routes_max": stats.get("first_incumbent_route_pool_routes_max"),
        "incumbent_source": stats.get("incumbent_source"),
        "heuristic_columns_generated": stats.get("heuristic_columns_generated"),
        "heuristic_incumbent_updates": stats.get("heuristic_incumbent_updates"),
        "heuristic_hard_pool_solves": stats.get("heuristic_hard_pool_solves"),
        "heuristic_hard_pool_time": stats.get("heuristic_hard_pool_time"),
        "heuristic_hard_pool_feasible_solves": stats.get("heuristic_hard_pool_feasible_solves"),
        "heuristic_support_pool_calls": stats.get("heuristic_support_pool_calls"),
        "heuristic_support_pool_time": stats.get("heuristic_support_pool_time"),
        "heuristic_support_pool_feasible": stats.get("heuristic_support_pool_feasible"),
        "heuristic_support_pool_incumbent_updates": stats.get("heuristic_support_pool_incumbent_updates"),
        "heuristic_full_pool_calls": stats.get("heuristic_full_pool_calls"),
        "heuristic_full_pool_time": stats.get("heuristic_full_pool_time"),
        "heuristic_full_pool_feasible": stats.get("heuristic_full_pool_feasible"),
        "heuristic_full_pool_incumbent_updates": stats.get("heuristic_full_pool_incumbent_updates"),
        "heuristic_node_pool_to_support_ratio_max": stats.get("heuristic_node_pool_to_support_ratio_max"),
        "heuristic_soft_pool_solves": stats.get("heuristic_soft_pool_solves"),
        "heuristic_soft_pool_time": stats.get("heuristic_soft_pool_time"),
        "heuristic_soft_pool_feasible_solves": stats.get("heuristic_soft_pool_feasible_solves"),
        "heuristic_repair_customers": stats.get("heuristic_repair_customers"),
        "heuristic_repair_columns_generated": stats.get("heuristic_repair_columns_generated"),
        "postroot_heuristic_calls": stats.get("postroot_heuristic_calls"),
        "postroot_heuristic_hard_pool_solves": stats.get("postroot_heuristic_hard_pool_solves"),
        "postroot_heuristic_support_pool_calls": stats.get("postroot_heuristic_support_pool_calls"),
        "postroot_heuristic_full_pool_calls": stats.get("postroot_heuristic_full_pool_calls"),
        "postroot_heuristic_soft_pool_solves": stats.get("postroot_heuristic_soft_pool_solves"),
        "postroot_repair_calls": stats.get("postroot_repair_calls"),
        "postroot_repair_columns_generated": stats.get("postroot_repair_columns_generated"),
        "postroot_heuristic_incumbent_updates": stats.get("postroot_heuristic_incumbent_updates"),
        "postroot_heuristic_budget_hits": stats.get("postroot_heuristic_budget_hits"),
        "heuristic_max_node_pool_routes": stats.get("heuristic_max_node_pool_routes"),
        "heuristic_max_support_routes": stats.get("heuristic_max_support_routes"),
        "side_pool_routes": stats.get("side_pool_routes"),
        "side_pool_routes_added": stats.get("side_pool_routes_added"),
        "side_pool_routes_pruned": stats.get("side_pool_routes_pruned"),
        "side_pool_max_routes": stats.get("side_pool_max_routes"),
        "side_pool_prune_time": stats.get("side_pool_prune_time"),
        "side_pool_batch_size_last": stats.get("side_pool_batch_size_last"),
        "side_pool_candidates_seen": stats.get("side_pool_candidates_seen"),
        "side_pool_routes_rejected_by_budget": stats.get("side_pool_routes_rejected_by_budget"),
        "side_pool_per_customer_keep": stats.get("side_pool_per_customer_keep"),
        "repair_budget_hit": stats.get("repair_budget_hit"),
        "heuristic_budget_hit": stats.get("heuristic_budget_hit"),
        "heuristic_stall_count": stats.get("heuristic_stall_count"),
        "selected_trucks": metrics.get("selected_trucks"),
        "drone_sorties": metrics.get("drone_sorties"),
        "service_feasible": metrics.get("service_feasible"),
        "mean_delay": metrics.get("mean_delay"),
        "max_delay": metrics.get("max_delay"),
        "delay_square_sum": metrics.get("delay_square_sum"),
        "truck_served_customers": metrics.get("truck_served_customers"),
        "drone_served_customers": metrics.get("drone_served_customers"),
        "max_payload": metrics.get("max_payload"),
        "waiting_blocks": metrics.get("waiting_blocks"),
        "total_wait_time": metrics.get("total_wait_time"),
        "mean_wait_time": metrics.get("mean_wait_time"),
        "max_wait_time": metrics.get("max_wait_time"),
        "last_progress_event": progress.get("event"),
        "last_progress_node": progress_node.get("id"),
        "last_progress_columns": progress_node.get("columns"),
        "last_progress_active_sr": progress_node.get("active_sr"),
        "error": record.get("error"),
    }


def _write_summary(output_dir: Path, rows: list[dict[str, Any]]) -> None:
    (output_dir / "summary.json").write_text(json.dumps(rows, indent=2), encoding="utf-8")
    if not rows:
        return
    with (output_dir / "summary.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def _first_existing(directory: Path, names: list[str]) -> Path | None:
    for name in names:
        path = directory / name
        if path.exists():
            return path
    return None


def _read_progress(case_dir: Path) -> dict[str, Any] | None:
    path = case_dir / "bpc_progress.json"
    if not path.exists():
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def _read_pricing_diagnostics(case_dir: Path) -> dict[str, Any]:
    path = case_dir / "pricing_diagnostics.jsonl"
    if not path.exists():
        return {}
    records = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    if not records:
        return {}
    modes = sorted({record.get("mode") for record in records})
    mode_elapsed_seconds = {
        mode: sum(
            float(record.get("elapsed_seconds", 0.0) or 0.0)
            for record in records
            if record.get("mode") == mode
        )
        for mode in modes
    }
    return {
        "file": str(path),
        "count": len(records),
        "mode_counts": {
            mode: sum(1 for record in records if record.get("mode") == mode)
            for mode in modes
        },
        "mode_elapsed_seconds": mode_elapsed_seconds,
        "pricing_labels_generated": sum(int(record["labels_generated"]) for record in records),
        "pricing_labels_dominated": sum(int(record["labels_dominated"]) for record in records),
        "pricing_labels_purged": sum(int(record.get("labels_purged", 0) or 0) for record in records),
        "pricing_stale_labels_skipped": sum(int(record.get("stale_labels_skipped", 0) or 0) for record in records),
        "pricing_labels_pruned": sum(int(record["labels_pruned"]) for record in records),
        "pricing_standard_bound_pruned": sum(int(record.get("standard_bound_pruned", 0) or 0) for record in records),
        "pricing_farkas_bound_pruned": sum(int(record.get("farkas_bound_pruned", 0) or 0) for record in records),
        "pricing_complete_routes_generated": sum(int(record["complete_routes_generated"]) for record in records),
        "pricing_extensions_attempted": sum(int(record.get("extensions_attempted", 0) or 0) for record in records),
        "pricing_extensions_rejected_by_deadline": sum(int(record.get("extensions_rejected_by_deadline", 0) or 0) for record in records),
        "pricing_deadline_reachability_removed": sum(int(record.get("deadline_reachability_removed", 0) or 0) for record in records),
        "pricing_reward_set_size_before_deadline": sum(int(record.get("reward_set_size_before_deadline", 0) or 0) for record in records),
        "pricing_reward_set_size_after_deadline": sum(int(record.get("reward_set_size_after_deadline", 0) or 0) for record in records),
        "pricing_deadline_reward_bound_calls": sum(int(record.get("deadline_reward_bound_calls", 0) or 0) for record in records),
        "pricing_deadline_dominance_prefilter_skips": sum(int(record.get("deadline_dominance_prefilter_skips", 0) or 0) for record in records),
        "pricing_routes_rejected_by_deadline_in_master": sum(int(record.get("routes_rejected_by_deadline_in_master", 0) or 0) for record in records),
        "pricing_forward_dominance_tests": sum(int(record.get("forward_dominance_tests", 0) or 0) for record in records),
        "pricing_forward_same_node_dominance_tests": sum(
            int(record.get("forward_same_node_dominance_tests", 0) or 0) for record in records
        ),
        "pricing_forward_physical_location_dominance_tests": sum(
            int(record.get("forward_physical_location_dominance_tests", 0) or 0) for record in records
        ),
        "pricing_forward_physical_location_dominance_rejections": sum(
            int(record.get("forward_physical_location_dominance_rejections", 0) or 0) for record in records
        ),
        "pricing_forward_return_time_credit_checks": sum(
            int(record.get("forward_return_time_credit_checks", 0) or 0) for record in records
        ),
        "pricing_forward_return_time_credit_checks_skipped": sum(
            int(record.get("forward_return_time_credit_checks_skipped", 0) or 0) for record in records
        ),
        "pricing_forward_branch_language_failures": sum(
            int(record.get("forward_branch_language_failures", 0) or 0) for record in records
        ),
        "pricing_forward_mask_scalar_prefilter_failures": sum(
            int(record.get("forward_mask_scalar_prefilter_failures", 0) or 0) for record in records
        ),
        "pricing_dom_gate_pairs_seen": sum(int(record.get("dom_gate_pairs_seen", 0) or 0) for record in records),
        "pricing_dom_gate_mask_failures": sum(
            int(record.get("dom_gate_mask_failures", 0) or 0) for record in records
        ),
        "pricing_dom_gate_scalar_failures": sum(
            int(record.get("dom_gate_scalar_failures", 0) or 0) for record in records
        ),
        "pricing_dom_gate_branch_failures": sum(
            int(record.get("dom_gate_branch_failures", 0) or 0) for record in records
        ),
        "pricing_dom_gate_deadline_failures": sum(
            int(record.get("dom_gate_deadline_failures", 0) or 0) for record in records
        ),
        "pricing_labels_dominated_same_node": sum(
            int(record.get("labels_dominated_same_node", 0) or 0) for record in records
        ),
        "pricing_labels_dominated_physical": sum(
            int(record.get("labels_dominated_physical", 0) or 0) for record in records
        ),
        "pricing_dom_prefilter_pairs": sum(int(record.get("dom_prefilter_pairs", 0) or 0) for record in records),
        "pricing_dom_prefilter_mask_fail": sum(int(record.get("dom_prefilter_mask_fail", 0) or 0) for record in records),
        "pricing_dom_prefilter_branch_fail": sum(int(record.get("dom_prefilter_branch_fail", 0) or 0) for record in records),
        "pricing_dom_prefilter_payload_fail": sum(int(record.get("dom_prefilter_payload_fail", 0) or 0) for record in records),
        "pricing_dom_prefilter_block_fail": sum(int(record.get("dom_prefilter_block_fail", 0) or 0) for record in records),
        "pricing_dom_prefilter_return_credit_fail": sum(
            int(record.get("dom_prefilter_return_credit_fail", 0) or 0) for record in records
        ),
        "pricing_dom_full_tests": sum(int(record.get("dom_full_tests", 0) or 0) for record in records),
        "pricing_dom_full_rejections": sum(int(record.get("dom_full_rejections", 0) or 0) for record in records),
        "pricing_physical_location_full_tests": sum(
            int(record.get("physical_location_full_tests", 0) or 0) for record in records
        ),
        "pricing_physical_location_rejections": sum(
            int(record.get("physical_location_rejections", 0) or 0) for record in records
        ),
        "pricing_max_queue_size": max(int(record["max_queue_size"]) for record in records),
        "pricing_diagnostics_elapsed_seconds": sum(float(record.get("elapsed_seconds", 0.0) or 0.0) for record in records),
        "pricing_max_call_elapsed_seconds": max(float(record.get("elapsed_seconds", 0.0) or 0.0) for record in records),
        "pricing_forward_labels_generated": sum(int(record.get("forward_labels_generated", 0) or 0) for record in records),
        "pricing_backward_labels_generated": sum(int(record.get("backward_labels_generated", 0) or 0) for record in records),
        "pricing_backward_dominance_tests": sum(int(record.get("backward_dominance_tests", 0) or 0) for record in records),
        "pricing_backward_labels_dominated": sum(int(record.get("backward_labels_dominated", 0) or 0) for record in records),
        "pricing_backward_cost_function_build_time": sum(
            float(record.get("backward_cost_function_build_time_seconds", 0.0) or 0.0) for record in records
        ),
        "pricing_backward_cost_function_eval_time": sum(
            float(record.get("backward_cost_function_eval_time_seconds", 0.0) or 0.0) for record in records
        ),
        "pricing_join_sr_correction_time": sum(
            float(record.get("join_sr_correction_time_seconds", 0.0) or 0.0) for record in records
        ),
        "pricing_join_active_block_time": sum(
            float(record.get("join_active_block_time_seconds", 0.0) or 0.0) for record in records
        ),
        "pricing_joined_reduced_cost_evaluations": sum(
            int(record.get("joined_reduced_cost_evaluations", 0) or 0) for record in records
        ),
        "pricing_backward_dominance_cost_tests": sum(
            int(record.get("backward_dominance_cost_tests", 0) or 0) for record in records
        ),
        "pricing_backward_dominance_cost_rejected": sum(
            int(record.get("backward_dominance_cost_rejected", 0) or 0) for record in records
        ),
        "pricing_backward_exclusive_resource_violations": sum(
            int(record.get("backward_exclusive_resource_violations", 0) or 0) for record in records
        ),
        "pricing_join_pairs_tested": sum(int(record.get("join_pairs_tested", 0) or 0) for record in records),
        "pricing_joined_routes_accepted": sum(int(record.get("joined_routes_accepted", 0) or 0) for record in records),
        "pricing_forward_labeling_time": sum(float(record.get("forward_labeling_time_seconds", 0.0) or 0.0) for record in records),
        "pricing_backward_labeling_time": sum(float(record.get("backward_labeling_time_seconds", 0.0) or 0.0) for record in records),
        "pricing_join_time": sum(float(record.get("join_time_seconds", 0.0) or 0.0) for record in records),
        "pricing_parallel_calls": sum(int(record.get("parallel_calls", 0) or 0) for record in records),
        "pricing_parallel_workers_max": max(int(record.get("parallel_workers", 1) or 1) for record in records),
        "pricing_process_cpu_time": sum(float(record.get("process_cpu_time_seconds", 0.0) or 0.0) for record in records),
        "pricing_cpu_core_equivalent_max": max(float(record.get("cpu_core_equivalent", 0.0) or 0.0) for record in records),
        "pricing_cpu_core_equivalent_weighted": (
            sum(float(record.get("process_cpu_time_seconds", 0.0) or 0.0) for record in records)
            / sum(float(record.get("elapsed_seconds", 0.0) or 0.0) for record in records)
            if sum(float(record.get("elapsed_seconds", 0.0) or 0.0) for record in records) > 0.0
            else 0.0
        ),
        "pricing_worker_backend_thread_calls": sum(1 for record in records if record.get("pricing_worker_backend") == "thread"),
        "pricing_worker_backend_process_calls": sum(1 for record in records if record.get("pricing_worker_backend") == "process"),
        "pricing_source_neighbor_count_max": max(int(record.get("source_neighbor_count", 0) or 0) for record in records),
        "pricing_source_neighbor_task_count_max": max(int(record.get("source_neighbor_task_count", 0) or 0) for record in records),
        "pricing_source_neighbor_task_size_max": max(
            max((int(value) for value in record.get("source_neighbor_task_sizes", ()) or ()), default=0)
            for record in records
        ),
        "pricing_local_worker_candidate_quota_max": max(int(record.get("local_worker_candidate_quota", 0) or 0) for record in records),
        "pricing_diversity_quota_max": max(int(record.get("diversity_quota", 0) or 0) for record in records),
        "pricing_diversity_selected_routes": sum(int(record.get("diversity_selected_routes", 0) or 0) for record in records),
        "pricing_diversity_selected_customers_max": max(int(record.get("diversity_selected_customers", 0) or 0) for record in records),
        "pricing_productive_slice_deadline_calls": sum(1 for record in records if record.get("productive_slice_deadline_used")),
        "pricing_productive_slice_time": sum(float(record.get("productive_slice_seconds", 0.0) or 0.0) for record in records),
        "pricing_productive_time_limit_with_columns": sum(int(record.get("productive_time_limit_with_columns", 0) or 0) for record in records),
        "pricing_productive_time_limit_no_columns": sum(int(record.get("productive_time_limit_no_columns", 0) or 0) for record in records),
        "pricing_first_hit_exits": sum(int(record.get("first_hit_exits", 0) or 0) for record in records),
        "pricing_interrupted_worker_calls": sum(int(record.get("interrupted_worker_calls", 0) or 0) for record in records),
        "pricing_certification_worker_calls": sum(int(record.get("certification_worker_calls", 0) or 0) for record in records),
        "pricing_productive_worker_calls": sum(int(record.get("productive_worker_calls", 0) or 0) for record in records),
        "pricing_worker_cpu_time": sum(float(record.get("pricing_worker_cpu_time_seconds", 0.0) or 0.0) for record in records),
        "pricing_main_process_cpu_time": sum(
            float(record.get("pricing_main_process_cpu_time_seconds", 0.0) or 0.0) for record in records
        ),
        "pricing_main_merge_time": sum(float(record.get("pricing_main_merge_time_seconds", 0.0) or 0.0) for record in records),
        "pricing_pool_startup_time": sum(float(record.get("pricing_pool_startup_time_seconds", 0.0) or 0.0) for record in records),
        "pricing_pool_startup_count": sum(int(record.get("pricing_pool_startup_count", 0) or 0) for record in records),
        "pricing_pool_reused_calls": sum(int(record.get("pricing_pool_reused_calls", 0) or 0) for record in records),
        "pricing_pool_shutdown_time": sum(float(record.get("pricing_pool_shutdown_time_seconds", 0.0) or 0.0) for record in records),
        "pricing_task_submission_time": sum(float(record.get("pricing_task_submission_time_seconds", 0.0) or 0.0) for record in records),
        "pricing_worker_payload_count": sum(int(record.get("pricing_worker_payload_count", 0) or 0) for record in records),
        "pricing_worker_response_count": sum(int(record.get("pricing_worker_response_count", 0) or 0) for record in records),
        "pricing_candidate_paths_before_merge": sum(int(record.get("pricing_candidate_paths_before_merge", 0) or 0) for record in records),
        "pricing_candidate_paths_after_merge": sum(int(record.get("pricing_candidate_paths_after_merge", 0) or 0) for record in records),
        "pricing_decoded_routes_in_main": sum(int(record.get("pricing_decoded_routes_in_main", 0) or 0) for record in records),
        "pricing_verified_routes_in_main": sum(int(record.get("pricing_verified_routes_in_main", 0) or 0) for record in records),
        "pricing_batch_target_max": max(int(record.get("pricing_batch_target", 0) or 0) for record in records),
        "pricing_returned_batch_size_max": max(int(record.get("pricing_returned_batch_size", 0) or 0) for record in records),
        "pricing_first_hit_enabled_calls": sum(1 for record in records if record.get("pricing_first_hit_enabled")),
        "pricing_stale_response_rejections": sum(int(record.get("pricing_stale_response_rejections", 0) or 0) for record in records),
        "pricing_signature_cache_hits": sum(int(record.get("signature_cache_hits", 0) or 0) for record in records),
        "pricing_signature_cache_misses": sum(int(record.get("signature_cache_misses", 0) or 0) for record in records),
        "pricing_core_signature_cache_hits": sum(int(record.get("core_signature_cache_hits", 0) or 0) for record in records),
        "pricing_core_signature_cache_misses": sum(int(record.get("core_signature_cache_misses", 0) or 0) for record in records),
        "pricing_active_signature_cache_hits": sum(int(record.get("active_signature_cache_hits", 0) or 0) for record in records),
        "pricing_active_signature_cache_misses": sum(int(record.get("active_signature_cache_misses", 0) or 0) for record in records),
        "pricing_sr_coeff_cache_hits": sum(int(record.get("sr_coeff_cache_hits", 0) or 0) for record in records),
        "pricing_sr_coeff_cache_misses": sum(int(record.get("sr_coeff_cache_misses", 0) or 0) for record in records),
        "pricing_active_sr_key_cache_hits": sum(int(record.get("active_sr_key_cache_hits", 0) or 0) for record in records),
        "pricing_active_sr_key_cache_misses": sum(int(record.get("active_sr_key_cache_misses", 0) or 0) for record in records),
        "pricing_active_sr_coeffs_computed": sum(int(record.get("active_sr_coeffs_computed", 0) or 0) for record in records),
        "pricing_triplet_masks_built": sum(int(record.get("triplet_masks_built", 0) or 0) for record in records),
        "pricing_dominance_prefilter_pairs": sum(int(record.get("dominance_prefilter_pairs", 0) or 0) for record in records),
        "pricing_dominance_prefilter_rejected": sum(int(record.get("dominance_prefilter_rejected", 0) or 0) for record in records),
        "pricing_dominance_bucket_pairs_considered": sum(int(record.get("dominance_bucket_pairs_considered", 0) or 0) for record in records),
        "pricing_dominance_bucket_pairs_rejected": sum(int(record.get("dominance_bucket_pairs_rejected", 0) or 0) for record in records),
        "pricing_dominance_bucket_candidate_pairs": sum(int(record.get("dominance_bucket_candidate_pairs", 0) or 0) for record in records),
        "pricing_dominance_compatible_keys_generated": sum(int(record.get("dominance_compatible_keys_generated", 0) or 0) for record in records),
        "pricing_dominance_compatible_key_lookups": sum(int(record.get("dominance_compatible_key_lookups", 0) or 0) for record in records),
        "pricing_dominance_bucket_scans_avoided": sum(int(record.get("dominance_bucket_scans_avoided", 0) or 0) for record in records),
        "pricing_dominance_key_generation_time": sum(float(record.get("dominance_key_generation_time_seconds", 0.0) or 0.0) for record in records),
        "pricing_dominance_key_cache_hits": sum(int(record.get("dominance_key_cache_hits", 0) or 0) for record in records),
        "pricing_dominance_key_cache_misses": sum(int(record.get("dominance_key_cache_misses", 0) or 0) for record in records),
        "pricing_dominance_small_bypass_calls": sum(int(record.get("dominance_small_bypass_calls", 0) or 0) for record in records),
        "pricing_dominance_bypass_calls": sum(int(record.get("dominance_bypass_calls", 0) or 0) for record in records),
        "pricing_dominance_indexed_activation_count": sum(int(record.get("dominance_indexed_activation_count", 0) or 0) for record in records),
        "pricing_dominance_work_estimate": max(int(record.get("dominance_work_estimate", 0) or 0) for record in records),
        "pricing_sticky_indexed_dominance_activations": sum(
            int(record.get("sticky_indexed_dominance_activations", 0) or 0) for record in records
        ),
        "pricing_dominance_stage_reject_key": sum(int(record.get("dominance_stage_reject_key", 0) or 0) for record in records),
        "pricing_dominance_stage_reject_branch": sum(int(record.get("dominance_stage_reject_branch", 0) or 0) for record in records),
        "pricing_dominance_stage_reject_customer": sum(int(record.get("dominance_stage_reject_customer", 0) or 0) for record in records),
        "pricing_dominance_stage_reject_truck_node": sum(
            int(record.get("dominance_stage_reject_truck_node", 0) or 0) for record in records
        ),
        "pricing_dominance_stage_reject_payload": sum(int(record.get("dominance_stage_reject_payload", 0) or 0) for record in records),
        "pricing_dominance_stage_reject_block": sum(int(record.get("dominance_stage_reject_block", 0) or 0) for record in records),
        "pricing_dominance_stage_reject_time": sum(int(record.get("dominance_stage_reject_time", 0) or 0) for record in records),
        "pricing_dominance_stage_reject_cost": sum(int(record.get("dominance_stage_reject_cost", 0) or 0) for record in records),
        "pricing_backward_full_dominance_tests": sum(int(record.get("backward_full_dominance_tests", 0) or 0) for record in records),
        "pricing_join_prefilter_pairs": sum(int(record.get("join_prefilter_pairs", 0) or 0) for record in records),
        "pricing_join_prefilter_rejected": sum(int(record.get("join_prefilter_rejected", 0) or 0) for record in records),
        "pricing_join_bucket_pairs_considered": sum(int(record.get("join_bucket_pairs_considered", 0) or 0) for record in records),
        "pricing_join_bucket_pairs_rejected": sum(int(record.get("join_bucket_pairs_rejected", 0) or 0) for record in records),
        "pricing_join_bucket_candidate_pairs": sum(int(record.get("join_bucket_candidate_pairs", 0) or 0) for record in records),
        "pricing_join_compatible_keys_generated": sum(int(record.get("join_compatible_keys_generated", 0) or 0) for record in records),
        "pricing_join_compatible_key_lookups": sum(int(record.get("join_compatible_key_lookups", 0) or 0) for record in records),
        "pricing_join_bucket_scans_avoided": sum(int(record.get("join_bucket_scans_avoided", 0) or 0) for record in records),
        "pricing_join_key_generation_time": sum(float(record.get("join_key_generation_time_seconds", 0.0) or 0.0) for record in records),
        "pricing_join_key_cache_hits": sum(int(record.get("join_key_cache_hits", 0) or 0) for record in records),
        "pricing_join_key_cache_misses": sum(int(record.get("join_key_cache_misses", 0) or 0) for record in records),
        "pricing_join_graph_build_time": sum(float(record.get("join_graph_build_time_seconds", 0.0) or 0.0) for record in records),
        "pricing_join_subbucket_pairs_considered": sum(int(record.get("join_subbucket_pairs_considered", 0) or 0) for record in records),
        "pricing_join_subbucket_pairs_rejected": sum(int(record.get("join_subbucket_pairs_rejected", 0) or 0) for record in records),
        "pricing_join_bucket_lower_envelope_rejects": sum(
            int(record.get("join_bucket_lower_envelope_rejects", 0) or 0) for record in records
        ),
        "pricing_join_subbucket_lower_envelope_rejects": sum(
            int(record.get("join_subbucket_lower_envelope_rejects", 0) or 0) for record in records
        ),
        "pricing_join_pair_lower_envelope_rejects": sum(
            int(record.get("join_pair_lower_envelope_rejects", 0) or 0) for record in records
        ),
        "pricing_join_generator_queue_pushes": sum(int(record.get("join_generator_queue_pushes", 0) or 0) for record in records),
        "pricing_join_generator_queue_pops": sum(int(record.get("join_generator_queue_pops", 0) or 0) for record in records),
        "pricing_join_generator_splits": sum(int(record.get("join_generator_splits", 0) or 0) for record in records),
        "pricing_join_materialized_pairs": sum(int(record.get("join_materialized_pairs", 0) or 0) for record in records),
        "pricing_suffix_profile_cache_hits": sum(int(record.get("suffix_profile_cache_hits", 0) or 0) for record in records),
        "pricing_suffix_profile_cache_misses": sum(int(record.get("suffix_profile_cache_misses", 0) or 0) for record in records),
        "pricing_interface_profile_cache_hits": sum(int(record.get("interface_profile_cache_hits", 0) or 0) for record in records),
        "pricing_interface_profile_cache_misses": sum(int(record.get("interface_profile_cache_misses", 0) or 0) for record in records),
        "pricing_join_small_bypass_calls": sum(int(record.get("join_small_bypass_calls", 0) or 0) for record in records),
        "pricing_join_local_bypass_calls": sum(int(record.get("join_local_bypass_calls", 0) or 0) for record in records),
        "pricing_join_cumulative_bypass_calls": sum(int(record.get("join_cumulative_bypass_calls", 0) or 0) for record in records),
        "pricing_join_indexed_activation_count": sum(int(record.get("join_indexed_activation_count", 0) or 0) for record in records),
        "pricing_join_work_estimate": max(int(record.get("join_work_estimate", 0) or 0) for record in records),
        "pricing_sticky_indexed_join_activations": sum(int(record.get("sticky_indexed_join_activations", 0) or 0) for record in records),
        "pricing_join_stage_reject_key": sum(int(record.get("join_stage_reject_key", 0) or 0) for record in records),
        "pricing_join_stage_reject_branch": sum(int(record.get("join_stage_reject_branch", 0) or 0) for record in records),
        "pricing_join_stage_reject_customer": sum(int(record.get("join_stage_reject_customer", 0) or 0) for record in records),
        "pricing_join_stage_reject_truck_node": sum(int(record.get("join_stage_reject_truck_node", 0) or 0) for record in records),
        "pricing_join_stage_reject_payload": sum(int(record.get("join_stage_reject_payload", 0) or 0) for record in records),
        "pricing_join_stage_reject_block": sum(int(record.get("join_stage_reject_block", 0) or 0) for record in records),
        "pricing_join_stage_reject_reduced_cost": sum(int(record.get("join_stage_reject_reduced_cost", 0) or 0) for record in records),
        "pricing_join_candidate_pairs_accepted": sum(int(record.get("join_candidate_pairs_accepted", 0) or 0) for record in records),
        "pricing_join_label_pairs_materialized": sum(int(record.get("join_label_pairs_materialized", 0) or 0) for record in records),
        "pricing_join_full_decodes": sum(int(record.get("join_full_decodes", 0) or 0) for record in records),
        "pricing_lazy_rejected_before_decode": sum(int(record.get("lazy_rejected_before_decode", 0) or 0) for record in records),
        "pricing_fully_decoded_routes": sum(int(record.get("fully_decoded_routes", 0) or 0) for record in records),
        "pricing_duplicate_equivalent_rejected": sum(int(record.get("duplicate_equivalent_rejected", 0) or 0) for record in records),
        "pricing_cost_dominated_rejected": sum(int(record.get("cost_dominated_rejected", 0) or 0) for record in records),
        "pricing_signature_build_time": sum(float(record.get("signature_build_time_seconds", 0.0) or 0.0) for record in records),
        "pricing_sr_coeff_build_time": sum(float(record.get("sr_coeff_build_time_seconds", 0.0) or 0.0) for record in records),
        "pricing_duplicate_lookup_time": sum(float(record.get("duplicate_lookup_time_seconds", 0.0) or 0.0) for record in records),
        "pricing_route_decode_time": sum(float(record.get("route_decode_time_seconds", 0.0) or 0.0) for record in records),
        "pricing_reduced_cost_verification_time": sum(float(record.get("reduced_cost_verification_time_seconds", 0.0) or 0.0) for record in records),
        "pricing_side_pool_routes_returned": sum(int(record.get("side_pool_routes_returned", 0) or 0) for record in records),
        "pricing_side_pool_candidates_seen": sum(int(record.get("side_pool_candidates_seen", 0) or 0) for record in records),
        "pricing_side_pool_routes_retained": sum(int(record.get("side_pool_routes_retained", 0) or 0) for record in records),
        "pricing_side_pool_routes_rejected_by_budget": sum(
            int(record.get("side_pool_routes_rejected_by_budget", 0) or 0) for record in records
        ),
        "pricing_join_activation_mode_counts": {
            mode: sum(1 for record in records if record.get("join_activation_mode") == mode)
            for mode in sorted({record.get("join_activation_mode") for record in records if record.get("join_activation_mode")})
        },
        "pricing_dominance_activation_mode_counts": {
            mode: sum(1 for record in records if record.get("dominance_activation_mode") == mode)
            for mode in sorted(
                {record.get("dominance_activation_mode") for record in records if record.get("dominance_activation_mode")}
            )
        },
        "pricing_side_pool_reduced_cost_min": min(
            (
                float(record["side_pool_reduced_cost_min"])
                for record in records
                if record.get("side_pool_reduced_cost_min") is not None
            ),
            default=None,
        ),
        "pricing_productive_mode_calls": sum(
            1 for record in records if record.get("mode") == "standard" and record.get("pricing_mode") == "productive"
        ),
        "pricing_closure_mode_calls": sum(
            1 for record in records if record.get("mode") == "standard" and record.get("pricing_mode") == "closure"
        ),
        "pricing_yield_ratio": max(float(record.get("pricing_yield_ratio", 0.0) or 0.0) for record in records),
        "pricing_mode_counts": {
            mode: sum(1 for record in records if record.get("pricing_mode") == mode)
            for mode in sorted({record.get("pricing_mode") for record in records if record.get("pricing_mode")})
        },
        "last_pricing_diagnostic": records[-1],
    }


def _merge_timeout_stats(progress_stats: dict[str, Any], pricing_summary: dict[str, Any]) -> dict[str, Any]:
    merged = dict(progress_stats)
    if not pricing_summary:
        return merged
    for field in (
        "pricing_labels_generated",
        "pricing_labels_dominated",
        "pricing_labels_purged",
        "pricing_stale_labels_skipped",
        "pricing_labels_pruned",
        "pricing_standard_bound_pruned",
        "pricing_farkas_bound_pruned",
        "pricing_complete_routes_generated",
        "pricing_extensions_attempted",
        "pricing_extensions_rejected_by_deadline",
        "pricing_deadline_reachability_removed",
        "pricing_reward_set_size_before_deadline",
        "pricing_reward_set_size_after_deadline",
        "pricing_deadline_reward_bound_calls",
        "pricing_deadline_dominance_prefilter_skips",
        "pricing_routes_rejected_by_deadline_in_master",
        "pricing_forward_dominance_tests",
        "pricing_forward_same_node_dominance_tests",
        "pricing_forward_physical_location_dominance_tests",
        "pricing_forward_physical_location_dominance_rejections",
        "pricing_forward_return_time_credit_checks",
        "pricing_forward_return_time_credit_checks_skipped",
        "pricing_forward_branch_language_failures",
        "pricing_forward_mask_scalar_prefilter_failures",
        "pricing_dom_gate_pairs_seen",
        "pricing_dom_gate_mask_failures",
        "pricing_dom_gate_scalar_failures",
        "pricing_dom_gate_branch_failures",
        "pricing_dom_gate_deadline_failures",
        "pricing_labels_dominated_same_node",
        "pricing_labels_dominated_physical",
        "pricing_dom_prefilter_pairs",
        "pricing_dom_prefilter_mask_fail",
        "pricing_dom_prefilter_branch_fail",
        "pricing_dom_prefilter_payload_fail",
        "pricing_dom_prefilter_block_fail",
        "pricing_dom_prefilter_return_credit_fail",
        "pricing_dom_full_tests",
        "pricing_dom_full_rejections",
        "pricing_physical_location_full_tests",
        "pricing_physical_location_rejections",
        "pricing_forward_labels_generated",
        "pricing_backward_labels_generated",
        "pricing_backward_dominance_tests",
        "pricing_backward_labels_dominated",
        "pricing_joined_reduced_cost_evaluations",
        "pricing_backward_dominance_cost_tests",
        "pricing_backward_dominance_cost_rejected",
        "pricing_backward_exclusive_resource_violations",
        "pricing_join_pairs_tested",
        "pricing_joined_routes_accepted",
        "pricing_parallel_calls",
        "pricing_signature_cache_hits",
        "pricing_signature_cache_misses",
        "pricing_core_signature_cache_hits",
        "pricing_core_signature_cache_misses",
        "pricing_active_signature_cache_hits",
        "pricing_active_signature_cache_misses",
        "pricing_sr_coeff_cache_hits",
        "pricing_sr_coeff_cache_misses",
        "pricing_active_sr_key_cache_hits",
        "pricing_active_sr_key_cache_misses",
        "pricing_active_sr_coeffs_computed",
        "pricing_triplet_masks_built",
        "pricing_dominance_prefilter_pairs",
        "pricing_dominance_prefilter_rejected",
        "pricing_dominance_bucket_pairs_considered",
        "pricing_dominance_bucket_pairs_rejected",
        "pricing_dominance_bucket_candidate_pairs",
        "pricing_dominance_compatible_keys_generated",
        "pricing_dominance_compatible_key_lookups",
        "pricing_dominance_bucket_scans_avoided",
        "pricing_dominance_key_cache_hits",
        "pricing_dominance_key_cache_misses",
        "pricing_dominance_small_bypass_calls",
        "pricing_dominance_bypass_calls",
        "pricing_dominance_indexed_activation_count",
        "pricing_sticky_indexed_dominance_activations",
        "pricing_dominance_stage_reject_key",
        "pricing_dominance_stage_reject_branch",
        "pricing_dominance_stage_reject_customer",
        "pricing_dominance_stage_reject_truck_node",
        "pricing_dominance_stage_reject_payload",
        "pricing_dominance_stage_reject_block",
        "pricing_dominance_stage_reject_time",
        "pricing_dominance_stage_reject_cost",
        "pricing_backward_full_dominance_tests",
        "pricing_join_prefilter_pairs",
        "pricing_join_prefilter_rejected",
        "pricing_join_bucket_pairs_considered",
        "pricing_join_bucket_pairs_rejected",
        "pricing_join_bucket_candidate_pairs",
        "pricing_join_compatible_keys_generated",
        "pricing_join_compatible_key_lookups",
        "pricing_join_bucket_scans_avoided",
        "pricing_join_key_cache_hits",
        "pricing_join_key_cache_misses",
        "pricing_join_subbucket_pairs_considered",
        "pricing_join_subbucket_pairs_rejected",
        "pricing_join_bucket_lower_envelope_rejects",
        "pricing_join_subbucket_lower_envelope_rejects",
        "pricing_join_pair_lower_envelope_rejects",
        "pricing_join_generator_queue_pushes",
        "pricing_join_generator_queue_pops",
        "pricing_join_generator_splits",
        "pricing_join_materialized_pairs",
        "pricing_suffix_profile_cache_hits",
        "pricing_suffix_profile_cache_misses",
        "pricing_interface_profile_cache_hits",
        "pricing_interface_profile_cache_misses",
        "pricing_join_small_bypass_calls",
        "pricing_join_local_bypass_calls",
        "pricing_join_cumulative_bypass_calls",
        "pricing_join_indexed_activation_count",
        "pricing_sticky_indexed_join_activations",
        "pricing_join_stage_reject_key",
        "pricing_join_stage_reject_branch",
        "pricing_join_stage_reject_customer",
        "pricing_join_stage_reject_truck_node",
        "pricing_join_stage_reject_payload",
        "pricing_join_stage_reject_block",
        "pricing_join_stage_reject_reduced_cost",
        "pricing_join_candidate_pairs_accepted",
        "pricing_join_label_pairs_materialized",
        "pricing_join_full_decodes",
        "pricing_lazy_rejected_before_decode",
        "pricing_fully_decoded_routes",
        "pricing_duplicate_equivalent_rejected",
        "pricing_cost_dominated_rejected",
        "pricing_side_pool_routes_returned",
        "pricing_side_pool_candidates_seen",
        "pricing_side_pool_routes_retained",
        "pricing_side_pool_routes_rejected_by_budget",
        "pricing_productive_mode_calls",
        "pricing_closure_mode_calls",
        "pricing_worker_backend_thread_calls",
        "pricing_worker_backend_process_calls",
        "pricing_first_hit_exits",
        "pricing_interrupted_worker_calls",
        "pricing_certification_worker_calls",
        "pricing_productive_worker_calls",
        "pricing_pool_startup_count",
        "pricing_pool_reused_calls",
        "pricing_worker_payload_count",
        "pricing_worker_response_count",
        "pricing_candidate_paths_before_merge",
        "pricing_candidate_paths_after_merge",
        "pricing_decoded_routes_in_main",
        "pricing_verified_routes_in_main",
        "pricing_source_neighbor_task_count_max",
        "pricing_source_neighbor_task_size_max",
        "pricing_local_worker_candidate_quota_max",
        "pricing_diversity_quota_max",
        "pricing_diversity_selected_routes",
        "pricing_diversity_selected_customers_max",
        "pricing_productive_slice_deadline_calls",
        "pricing_productive_time_limit_with_columns",
        "pricing_productive_time_limit_no_columns",
        "pricing_first_hit_enabled_calls",
        "pricing_stale_response_rejections",
    ):
        merged[field] = max(int(merged.get(field, 0) or 0), int(pricing_summary[field]))
    for field in (
        "pricing_dominance_work_estimate",
        "pricing_join_work_estimate",
        "pricing_source_neighbor_count_max",
    ):
        merged[field] = max(int(merged.get(field, 0) or 0), int(pricing_summary[field]))
    for field in (
        "pricing_forward_labeling_time",
        "pricing_backward_labeling_time",
        "pricing_backward_cost_function_build_time",
        "pricing_backward_cost_function_eval_time",
        "pricing_join_sr_correction_time",
        "pricing_join_active_block_time",
        "pricing_join_time",
        "pricing_signature_build_time",
        "pricing_sr_coeff_build_time",
        "pricing_duplicate_lookup_time",
        "pricing_route_decode_time",
        "pricing_reduced_cost_verification_time",
        "pricing_dominance_key_generation_time",
        "pricing_join_key_generation_time",
        "pricing_join_graph_build_time",
        "pricing_yield_ratio",
        "pricing_process_cpu_time",
        "pricing_cpu_core_equivalent_max",
        "pricing_cpu_core_equivalent_weighted",
        "pricing_worker_cpu_time",
        "pricing_main_process_cpu_time",
        "pricing_main_merge_time",
        "pricing_pool_startup_time",
        "pricing_pool_shutdown_time",
        "pricing_task_submission_time",
        "pricing_productive_slice_time",
    ):
        merged[field] = max(float(merged.get(field, 0.0) or 0.0), float(pricing_summary[field]))
    for field in (
        "pricing_batch_target_max",
        "pricing_returned_batch_size_max",
    ):
        merged[field] = max(int(merged.get(field, 0) or 0), int(pricing_summary[field]))
    merged["pricing_parallel_workers_max"] = max(
        int(merged.get("pricing_parallel_workers_max", 1) or 1),
        int(pricing_summary["pricing_parallel_workers_max"]),
    )
    merged["pricing_max_queue_size"] = max(
        int(merged.get("pricing_max_queue_size", 0) or 0),
        int(pricing_summary["pricing_max_queue_size"]),
    )
    merged["pricing_diagnostics_elapsed_seconds"] = max(
        float(merged.get("pricing_diagnostics_elapsed_seconds", 0.0) or 0.0),
        float(pricing_summary["pricing_diagnostics_elapsed_seconds"]),
    )
    merged["pricing_max_call_elapsed_seconds"] = max(
        float(merged.get("pricing_max_call_elapsed_seconds", 0.0) or 0.0),
        float(pricing_summary["pricing_max_call_elapsed_seconds"]),
    )
    merged["pricing_diagnostics_count"] = max(
        int(merged.get("pricing_diagnostics_count", 0) or 0),
        int(pricing_summary["count"]),
    )
    merged["last_pricing_diagnostic"] = pricing_summary["last_pricing_diagnostic"]
    mode_counts = pricing_summary.get("mode_counts", {})
    standard_calls = int(mode_counts.get("standard", 0) or 0) + int(mode_counts.get("standard_interrupted", 0) or 0)
    if standard_calls:
        merged["standard_pricing_calls"] = max(
            int(merged.get("standard_pricing_calls", 0) or 0),
            standard_calls,
        )
    farkas_calls = int(mode_counts.get("farkas", 0) or 0) + int(mode_counts.get("farkas_interrupted", 0) or 0)
    if farkas_calls:
        merged["farkas_pricing_calls"] = max(
            int(merged.get("farkas_pricing_calls", 0) or 0),
            farkas_calls,
        )
    seed_calls = int(mode_counts.get("seed", 0) or 0) + int(mode_counts.get("seed_interrupted", 0) or 0)
    if seed_calls:
        merged["seed_pricing_calls"] = max(
            int(merged.get("seed_pricing_calls", 0) or 0),
            seed_calls,
        )
    repair_calls = int(mode_counts.get("repair", 0) or 0) + int(mode_counts.get("repair_interrupted", 0) or 0)
    if repair_calls:
        merged["repair_pricing_calls"] = max(
            int(merged.get("repair_pricing_calls", 0) or 0),
            repair_calls,
        )
    mode_elapsed = pricing_summary.get("mode_elapsed_seconds", {})
    for modes, field in (
        (("standard", "standard_interrupted"), "standard_pricing_time"),
        (("farkas", "farkas_interrupted"), "farkas_pricing_time"),
        (("seed", "seed_interrupted"), "seed_pricing_time"),
        (("repair", "repair_interrupted"), "repair_pricing_time"),
    ):
        elapsed = sum(float(mode_elapsed.get(mode, 0.0) or 0.0) for mode in modes)
        if elapsed:
            merged[field] = max(float(merged.get(field, 0.0) or 0.0), elapsed)
    return merged


def _parent_error_record(case: dict[str, Any], start: float, returncode: int, error: str) -> dict[str, Any]:
    return {
        "case_id": case["case_id"],
        "scale": case["scale"],
        "variant": case["variant"],
        "distribution": case["distribution"],
        "seed": case["seed"],
        "status": "error",
        "solver_status": "error",
        "runtime_seconds": time.time() - start,
        "instance_config": case["instance_config"],
        "solver_config": case["solver_config"],
        "worker_returncode": returncode,
        "error": error,
    }


def _decode_timeout_stream(value: str | bytes | None) -> str:
    if value is None:
        return ""
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    return value


def _relative_files(base: Path, subdir: str) -> list[str]:
    root = base / subdir
    if not root.exists():
        return []
    return [str(path) for path in sorted(root.rglob("*")) if path.is_file()]


if __name__ == "__main__":
    main()
