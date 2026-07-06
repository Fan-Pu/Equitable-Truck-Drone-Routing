from __future__ import annotations

import argparse
import json
from dataclasses import asdict
from pathlib import Path
import time
from typing import Any

from .config import InstanceConfig, ObjectiveWeights, SolverConfig
from .instance import generate_instance
from .metrics import solution_service_metrics
from .service_windows import load_manual_service_deadline_bounds


def main() -> None:
    parser = argparse.ArgumentParser()
    legacy_help = argparse.SUPPRESS
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--num-trucks", type=int, required=True)
    parser.add_argument("--num-customers", type=int, required=True)
    parser.add_argument("--num-hubs", type=int, default=2)
    parser.add_argument("--distribution", choices=["PS", "PC", "mixed"], required=True)
    parser.add_argument("--drones-per-truck", type=int, required=True)
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
    parser.add_argument("--weights", nargs=3, type=float, metavar=("DELAY", "RETURN", "COST"), required=True)
    parser.add_argument("--threads", type=int, default=1)
    parser.add_argument("--time-limit", type=float, default=1800.0)
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
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    manual_deadlines = load_manual_service_deadline_bounds(args.service_deadline_file) if args.service_deadline_file else None
    instance_config = InstanceConfig(
        seed=args.seed,
        num_trucks=args.num_trucks,
        num_customers=args.num_customers,
        num_hubs=args.num_hubs,
        distribution=args.distribution,
        drones_per_truck=args.drones_per_truck,
        service_deadline_mode=args.service_deadline_mode,
        service_deadline_fraction=args.service_deadline_fraction,
        service_deadline_manual_bounds=manual_deadlines,
        service_deadline_offset_min=args.service_deadline_offset_min,
        service_deadline_offset_max=args.service_deadline_offset_max,
        service_deadline_witness_slack=args.service_deadline_witness_slack,
        service_deadline_random_seed=args.service_deadline_random_seed,
        service_deadline_witness_method=args.service_deadline_witness_method,
        service_deadline_witness_time_limit=args.service_deadline_witness_time_limit,
    )
    weights = ObjectiveWeights(args.weights[0], args.weights[1], args.weights[2])
    solver_config = SolverConfig(
        threads=args.threads,
        time_limit=args.time_limit,
        pricing_batch_size=args.pricing_batch_size,
        min_pricing_batch_size=args.min_pricing_batch_size,
        max_root_pricing_batch_size=args.max_root_pricing_batch_size,
        nonroot_pricing_batch_size=args.nonroot_pricing_batch_size,
        certification_pass_interval=args.certification_pass_interval,
        pricing_yield_window=args.pricing_yield_window,
        pricing_yield_low=args.pricing_yield_low,
        pricing_yield_high=args.pricing_yield_high,
        productive_pricing_slice_seconds=args.productive_pricing_slice_seconds,
        productive_slice_min_seconds=args.productive_slice_min_seconds,
        productive_slice_max_seconds=args.productive_slice_max_seconds,
        productive_yield_window=args.productive_yield_window,
        productive_yield_low_threshold=args.productive_yield_low_threshold,
        productive_yield_high_threshold=args.productive_yield_high_threshold,
        closure_attempt_batch_period=args.closure_attempt_batch_period,
        closure_attempt_time_period=args.closure_attempt_time_period,
        closure_certification_time_budget=args.closure_certification_time_budget,
        closure_batch_size=args.closure_batch_size,
        use_dual_stabilized_productive_search=args.use_dual_stabilized_productive_search,
        dual_stabilization_weight=args.dual_stabilization_weight,
        prefix_task_depth=args.prefix_task_depth,
        productive_candidate_multiplier=args.productive_candidate_multiplier,
        source_neighbor_task_size=args.source_neighbor_task_size,
        pricing_diversity_batch_fraction=args.pricing_diversity_batch_fraction,
        first_incumbent_route_pool_time_limit=args.first_incumbent_route_pool_time_limit,
        post_incumbent_primal_budget_factor=args.post_incumbent_primal_budget_factor,
        root_constructive_time_limit=args.root_constructive_time_limit,
        enable_constructive_root_incumbent=not args.disable_constructive_root_incumbent,
        root_compact_after_constructive=args.root_compact_after_constructive,
        root_compact_time_limit_after_constructive=args.root_compact_time_limit_after_constructive,
        root_compact_time_limit_without_constructive=args.root_compact_time_limit_without_constructive,
        root_compact_wall_time_limit=args.root_compact_wall_time_limit,
        root_compact_solve_time_limit=args.root_compact_solve_time_limit,
        constructive_diversity_threshold=args.constructive_diversity_threshold,
        constructive_incumbent_quality_threshold=args.constructive_incumbent_quality_threshold,
        enable_drone_diversification_warm_start=args.enable_drone_diversification_warm_start,
        enable_incremental_rmp=args.enable_incremental_rmp,
        enable_active_coefficient_cache=args.enable_active_coefficient_cache,
        enable_sr_aging=not args.disable_sr_aging,
        enable_postroot_sr_cut_removal=not args.disable_postroot_sr_cut_removal,
        sr_inactive_age_threshold=args.sr_inactive_age_threshold,
        sr_removal_batch_size=args.sr_removal_batch_size,
        sr_max_removals_per_node=args.sr_max_removals_per_node,
        sr_reactivation_allowed=not args.disable_sr_reactivation,
        sr_removal_min_active_count=args.sr_removal_min_active_count,
        sr_removal_rmp_growth_threshold=args.sr_removal_rmp_growth_threshold,
        sr_removal_build_time_threshold=args.sr_removal_build_time_threshold,
        sr_removal_active_coeff_threshold=args.sr_removal_active_coeff_threshold,
        sr_activity_tolerance=args.sr_activity_tolerance,
        enable_global_branch_route_index=args.enable_global_branch_route_index,
        enable_node_column_aging=args.enable_node_column_aging,
        column_inactive_age_min=args.column_inactive_age_min,
        column_active_value_tol=args.column_active_value_tol,
        column_deactivation_min_active_columns=args.column_deactivation_min_active_columns,
        column_deactivation_batch_size=args.column_deactivation_batch_size,
        child_certification_slice_seconds=args.child_certification_slice_seconds,
        child_certification_max_slices_per_node=args.child_certification_max_slices_per_node,
        child_productive_before_certification=args.child_productive_before_certification,
        enable_resumable_child_certification=args.enable_resumable_child_certification,
        enable_child_closure_batch_adaptation=args.enable_child_closure_batch_adaptation,
        child_closure_batch_min=args.child_closure_batch_min,
        child_closure_batch_initial=args.child_closure_batch_initial,
        child_closure_batch_max=args.child_closure_batch_max,
        child_certification_yield_window=args.child_certification_yield_window,
        child_certification_yield_low=args.child_certification_yield_low,
        child_certification_yield_high=args.child_certification_yield_high,
        child_cert_useful_yield_window=args.child_cert_useful_yield_window,
        child_cert_no_route_yield_window=args.child_cert_no_route_yield_window,
        child_cert_dual_stability_window=args.child_cert_dual_stability_window,
        child_cert_dual_change_tol=args.child_cert_dual_change_tol,
        child_closure_batch_growth_factor=args.child_closure_batch_growth_factor,
        child_closure_batch_shrink_factor=args.child_closure_batch_shrink_factor,
        child_useful_yield_low=args.child_useful_yield_low,
        child_useful_yield_high=args.child_useful_yield_high,
        child_no_route_yield_high=args.child_no_route_yield_high,
        enable_rmp_basis_reuse=args.enable_rmp_basis_reuse,
        sr_removal_density_weight=args.sr_removal_density_weight,
        sr_removal_nnz_weight=args.sr_removal_nnz_weight,
        sr_removal_age_weight=args.sr_removal_age_weight,
        sr_removal_build_weight=args.sr_removal_build_weight,
        sr_removal_violation_weight=args.sr_removal_violation_weight,
        sr_removal_dual_weight=args.sr_removal_dual_weight,
        sr_removal_score_threshold=args.sr_removal_score_threshold,
        sr_removal_max_per_node=args.sr_removal_max_per_node,
        use_row_local_sr_coeff_cache=not args.disable_row_local_sr_coeff_cache,
        use_dominance_prefilter_keys=not args.disable_dominance_prefilter_keys,
        use_promised_drone_construction=args.enable_promised_drone_construction,
        promised_drone_construct_time_limit=args.promised_drone_construct_time_limit,
        promised_drone_insert_top_k_customers=args.promised_drone_insert_top_k_customers,
        promised_drone_insert_top_k_pads=args.promised_drone_insert_top_k_pads,
        promised_drone_exchange_top_k_pairs=args.promised_drone_exchange_top_k_pairs,
        promised_drone_min_improvement=args.promised_drone_min_improvement,
        no_drone_incumbent_trigger=False,
        compact_after_no_drone_incumbent=args.compact_after_no_drone_incumbent,
        join_eval_budget=0,
        pricing_certification_slice_seconds=0.0,
        enable_join_lower_envelope=False,
        join_generator_split_threshold=args.join_generator_split_threshold,
        join_generator_pair_batch_size=args.join_generator_pair_batch_size,
        enable_bucket_join_envelope=False,
        enable_join_profile_cache=False,
        small_join_pair_threshold=args.small_join_pair_threshold,
        small_join_cumulative_threshold=args.small_join_cumulative_threshold,
        max_join_bypass_calls=args.max_join_bypass_calls,
        small_dom_bucket_threshold=args.small_dom_bucket_threshold,
        small_dom_cumulative_threshold=args.small_dom_cumulative_threshold,
        max_dom_bypass_calls=args.max_dom_bypass_calls,
        root_max_side_pool_per_call=args.root_max_side_pool_per_call,
        side_pool_max_size=args.side_pool_max_size,
        side_pool_per_customer_keep=args.side_pool_per_customer_keep,
        join_payload_bin_width=args.join_payload_bin_width,
        repair_time_fraction_of_pricing=args.repair_time_fraction_of_pricing,
        repair_time_hard_cap_root=args.repair_time_hard_cap_root,
        repair_stall_limit=args.repair_stall_limit,
        preclosure_pricing_call_interval=args.preclosure_pricing_call_interval,
        preclosure_pool_growth_limit=args.preclosure_pool_growth_limit,
        farkas_batch_size=args.farkas_batch_size,
        seed_batch_size=args.seed_batch_size,
        repair_batch_size=args.repair_batch_size,
        enable_bidirectional_pricing=False,
        pricing_parallel_workers=args.pricing_parallel_workers,
        pricing_worker_backend=args.pricing_worker_backend,
        prefix_task_depth_root=args.prefix_task_depth_root,
        prefix_task_depth_child=args.prefix_task_depth_child,
        prefix_task_min_branching_for_depth2=args.prefix_task_min_branching_for_depth2,
        logging_mode=args.logging_mode,
        progress_snapshot_period=args.progress_snapshot_period,
        pricing_jsonl_enabled=not args.disable_pricing_jsonl,
        gurobi_log_dir=str(args.output_dir / "gurobi_logs"),
    )
    from .bpc import BPCTimeLimitNoIncumbent, solve_branch_price_cut

    instance = generate_instance(instance_config)
    start = time.time()
    try:
        result = solve_branch_price_cut(instance, weights, solver_config)
        record = _build_solve_record(instance_config, solver_config, instance, result, args.output_dir)
    except BPCTimeLimitNoIncumbent as exc:
        record = _build_solve_timeout_record(
            instance_config,
            solver_config,
            instance,
            exc,
            args.output_dir,
            time.time() - start,
        )
    output_file = args.output_dir / f"thvrpd-seed={args.seed}.json"
    output_file.write_text(json.dumps(record, indent=2), encoding="utf-8")
    print(output_file)


def _build_solve_record(
    instance_config: InstanceConfig,
    solver_config: SolverConfig,
    instance,
    result,
    output_dir: Path,
) -> dict:
    import gurobipy as gp

    return {
        "instance_config": asdict(instance_config),
        "solver_config": asdict(solver_config),
        "total_drones": instance.num_trucks * instance.drones_per_truck,
        "gurobi_version": ".".join(map(str, gp.gurobi.version())),
        "locations": instance.locations,
        "demand": instance.demand,
        **result.to_record(),
        "service_metrics": solution_service_metrics(instance, result),
        "gurobi_log_files": _relative_files(output_dir, "gurobi_logs"),
    }


def _build_solve_timeout_record(
    instance_config: InstanceConfig,
    solver_config: SolverConfig,
    instance,
    exc,
    output_dir: Path,
    runtime_seconds: float | None = None,
) -> dict:
    import gurobipy as gp

    pricing_summary = _read_pricing_diagnostics(output_dir)
    return {
        "instance_config": asdict(instance_config),
        "solver_config": asdict(solver_config),
        "total_drones": instance.num_trucks * instance.drones_per_truck,
        "gurobi_version": ".".join(map(str, gp.gurobi.version())),
        "locations": instance.locations,
        "demand": instance.demand,
        "status": "time_limited_internal",
        "solver_status": "time_limited_internal",
        "runtime_seconds": exc.runtime if runtime_seconds is None else runtime_seconds,
        "case_time_limit_seconds": solver_config.time_limit,
        "runtime": exc.runtime,
        "nodes_processed": exc.nodes_processed,
        "lower_bound_shifted": exc.lower_bound_shifted,
        "lower_bound_full": exc.objective.full_value_from_route_sum(exc.lower_bound_shifted),
        "normalization_bounds": asdict(exc.objective.bounds),
        "objective_coefficients": asdict(exc.objective.coeffs),
        "bpc_progress": _read_json(output_dir / "bpc_progress.json"),
        "pricing_diagnostics_summary": pricing_summary,
        "bpc_stats": _merge_timeout_stats(asdict(exc.stats), pricing_summary),
        "gurobi_log_files": _relative_files(output_dir, "gurobi_logs"),
        "error": str(exc),
    }


def _read_json(path: Path) -> dict[str, Any] | None:
    if not path.exists():
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def _read_pricing_diagnostics(output_dir: Path) -> dict[str, Any]:
    path = output_dir / "pricing_diagnostics.jsonl"
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


def _merge_timeout_stats(stats: dict[str, Any], pricing_summary: dict[str, Any]) -> dict[str, Any]:
    merged = dict(stats)
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
    for field in (
        "pricing_forward_labeling_time",
        "pricing_backward_labeling_time",
        "pricing_backward_cost_function_build_time",
        "pricing_backward_cost_function_eval_time",
        "pricing_join_sr_correction_time",
        "pricing_join_active_block_time",
        "pricing_join_time",
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
        merged[field] = max(
            float(merged.get(field, 0.0) or 0.0),
            float(pricing_summary[field]),
        )
    for field in (
        "pricing_batch_target_max",
        "pricing_returned_batch_size_max",
    ):
        merged[field] = max(int(merged.get(field, 0) or 0), int(pricing_summary[field]))
    merged["pricing_parallel_workers_max"] = max(
        int(merged.get("pricing_parallel_workers_max", 1) or 1),
        int(pricing_summary["pricing_parallel_workers_max"]),
    )
    merged["pricing_diagnostics_count"] = max(
        int(merged.get("pricing_diagnostics_count", 0) or 0),
        int(pricing_summary["count"]),
    )
    merged["last_pricing_diagnostic"] = pricing_summary["last_pricing_diagnostic"]
    mode_counts = pricing_summary.get("mode_counts", {})
    standard_calls = int(mode_counts.get("standard", 0) or 0) + int(mode_counts.get("standard_interrupted", 0) or 0)
    if standard_calls:
        merged["standard_pricing_calls"] = max(int(merged.get("standard_pricing_calls", 0) or 0), standard_calls)
    farkas_calls = int(mode_counts.get("farkas", 0) or 0) + int(mode_counts.get("farkas_interrupted", 0) or 0)
    if farkas_calls:
        merged["farkas_pricing_calls"] = max(int(merged.get("farkas_pricing_calls", 0) or 0), farkas_calls)
    seed_calls = int(mode_counts.get("seed", 0) or 0) + int(mode_counts.get("seed_interrupted", 0) or 0)
    if seed_calls:
        merged["seed_pricing_calls"] = max(int(merged.get("seed_pricing_calls", 0) or 0), seed_calls)
    repair_calls = int(mode_counts.get("repair", 0) or 0) + int(mode_counts.get("repair_interrupted", 0) or 0)
    if repair_calls:
        merged["repair_pricing_calls"] = max(int(merged.get("repair_pricing_calls", 0) or 0), repair_calls)
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


def _relative_files(base: Path, subdir: str) -> list[str]:
    root = base / subdir
    if not root.exists():
        return []
    return [str(path) for path in sorted(root.rglob("*")) if path.is_file()]


if __name__ == "__main__":
    main()
