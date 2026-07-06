from __future__ import annotations

from dataclasses import dataclass
from math import isclose, isfinite


@dataclass(frozen=True)
class ObjectiveWeights:
    delay: float
    return_time: float
    cost: float

    def __post_init__(self) -> None:
        total = self.delay + self.return_time + self.cost
        if not isclose(total, 1.0, rel_tol=0.0, abs_tol=1e-9):
            raise ValueError("objective weights must sum to one")
        if self.delay < 0 or self.return_time < 0 or self.cost < 0:
            raise ValueError("objective weights must be nonnegative")


@dataclass(frozen=True)
class InstanceConfig:
    seed: int
    num_trucks: int
    num_customers: int
    distribution: str
    drones_per_truck: int
    num_hubs: int = 2
    area_side: float = 14.0
    truck_arc_probability: float = 0.2
    hub_arc_probability: float = 0.5
    truck_speed: float = 40.0
    drone_speed: float = 40.0
    truck_payload: float = 50.0
    drone_payload: float = 2.3
    drone_endurance: float = 30.0
    truck_cost: float = 20.0
    drone_cost: float = 6.0
    low_demand_customer_ratio: float = 0.5
    low_demand_weight_mean: float = 1.0
    low_demand_weight_std: float = 5.0
    low_demand_weight_min: float = 0.5
    high_demand_weight_mean: float = 10.0
    high_demand_weight_std: float = 5.0
    high_demand_weight_min: float = 6.0
    service_deadline_mode: str = "none"
    service_deadline_fraction: float = 0.60
    service_deadline_manual_bounds: dict[str, float] | None = None
    service_deadline_offset_min: float = 45.0
    service_deadline_offset_max: float = 120.0
    service_deadline_witness_slack: float = 5.0
    service_deadline_random_seed: int | None = None
    service_deadline_witness_method: str = "constructive_then_compact"
    service_deadline_witness_time_limit: float = 30.0

    def __post_init__(self) -> None:
        if self.distribution not in {"PS", "PC", "mixed"}:
            raise ValueError("distribution must be PS, PC, or mixed")
        if self.num_trucks <= 0 or self.num_customers <= 0:
            raise ValueError("truck and customer counts must be positive")
        if self.drones_per_truck <= 0:
            raise ValueError("drones per truck must be positive")
        if self.num_hubs <= 0:
            raise ValueError("hub count must be positive")
        if self.service_deadline_mode not in {"none", "manual", "random_absolute"}:
            raise ValueError("service_deadline_mode must be none, manual, or random_absolute")
        if not 0.0 <= self.service_deadline_fraction <= 1.0:
            raise ValueError("service_deadline_fraction must be in [0, 1]")
        if self.service_deadline_mode == "manual":
            if not self.service_deadline_manual_bounds:
                raise ValueError("manual service deadline mode requires explicit customer bounds")
            for customer, bound in self.service_deadline_manual_bounds.items():
                if not isinstance(customer, str) or not customer:
                    raise ValueError("manual service deadline customer ids must be nonempty strings")
                if not isfinite(float(bound)):
                    raise ValueError(f"manual service deadline for {customer} must be finite")
        if (
            not isfinite(self.service_deadline_offset_min)
            or not isfinite(self.service_deadline_offset_max)
            or self.service_deadline_offset_min < 0.0
            or self.service_deadline_offset_max < self.service_deadline_offset_min
        ):
            raise ValueError("service deadline offset bounds must be finite and satisfy 0 <= min <= max")
        if self.service_deadline_witness_slack < 0.0:
            raise ValueError("service deadline witness slack must be nonnegative")
        if self.service_deadline_witness_method not in {"constructive", "compact", "constructive_then_compact"}:
            raise ValueError("service_deadline_witness_method must be constructive, compact, or constructive_then_compact")
        if self.service_deadline_witness_time_limit <= 0.0:
            raise ValueError("service_deadline_witness_time_limit must be positive")


@dataclass(frozen=True)
class SolverConfig:
    threads: int = 1
    time_limit: float = 1800.0
    pricing_tolerance: float = 1e-7
    cut_tolerance: float = 1e-7
    integrality_tolerance: float = 1e-6
    root_extraction_time_limit: float = 5.0
    route_pool_time_limit: float = 2.0
    support_threshold: float = 0.05
    support_best_per_customer: int = 3
    dive_reward: float = 1e-6
    repair_reward: float = 1.0
    seed_reward: float = 10.0
    phase_i_max_rounds: int = 20
    pricing_batch_size: int = 64
    min_pricing_batch_size: int = 32
    max_root_pricing_batch_size: int = 256
    nonroot_pricing_batch_size: int = 16
    certification_pass_interval: int = 5
    batch_insertion_time_threshold: float = 0.25
    batch_hysteresis_window: int = 5
    batch_hysteresis_consecutive: int = 2
    batch_low_insert_ratio: float = 0.05
    batch_high_insert_ratio: float = 0.20
    pricing_yield_window: int = 5
    pricing_yield_low: float = 0.25
    pricing_yield_high: float = 1.0
    productive_pricing_slice_seconds: float = 30.0
    productive_slice_min_seconds: float = 5.0
    productive_slice_max_seconds: float = 30.0
    productive_yield_window: int = 5
    productive_yield_low_threshold: float = 0.5
    productive_yield_high_threshold: float = 2.0
    closure_attempt_batch_period: int = 4
    closure_attempt_time_period: float = 120.0
    closure_certification_time_budget: float | None = None
    closure_batch_size: int = 32
    use_dual_stabilized_productive_search: bool = False
    dual_stabilization_weight: float = 0.3
    prefix_task_depth: int = 1
    productive_candidate_multiplier: float = 1.5
    source_neighbor_task_size: int = 1
    pricing_diversity_batch_fraction: float = 0.5
    first_incumbent_route_pool_time_limit: float = 10.0
    post_incumbent_primal_budget_factor: float = 0.25
    root_constructive_time_limit: float = 5.0
    enable_constructive_root_incumbent: bool = True
    root_compact_after_constructive: str = "conditional_wall_budget"
    root_compact_time_limit_after_constructive: float = 1.0
    root_compact_time_limit_without_constructive: float = 5.0
    root_compact_wall_time_limit: float = 1.0
    root_compact_solve_time_limit: float = 1.0
    constructive_diversity_threshold: float = 0.35
    constructive_incumbent_quality_threshold: float | None = None
    enable_drone_diversification_warm_start: bool = True
    enable_incremental_rmp: bool = True
    enable_active_coefficient_cache: bool = True
    enable_sr_aging: bool = True
    enable_postroot_sr_cut_removal: bool = True
    sr_inactive_age_threshold: int = 1
    sr_removal_batch_size: int = 32
    sr_max_removals_per_node: int = 32
    sr_reactivation_allowed: bool = True
    postroot_sr_cut_removal_batch_size: int = 32
    sr_removal_min_active_count: int = 64
    sr_removal_rmp_growth_threshold: float = 0.20
    sr_removal_build_time_threshold: float = 0.50
    sr_removal_active_coeff_threshold: float = 0.0
    sr_activity_tolerance: float = 1e-7
    enable_global_branch_route_index: bool = True
    enable_node_column_aging: bool = True
    column_inactive_age_min: int = 3
    column_active_value_tol: float = 1e-8
    column_deactivation_min_active_columns: int = 500
    column_deactivation_batch_size: int = 256
    child_certification_slice_seconds: float = 30.0
    child_certification_max_slices_per_node: int | None = None
    child_productive_before_certification: bool = False
    enable_resumable_child_certification: bool = True
    enable_child_closure_batch_adaptation: bool = False
    child_closure_batch_min: int = 16
    child_closure_batch_initial: int = 32
    child_closure_batch_max: int = 128
    child_certification_yield_window: int = 5
    child_certification_yield_low: float = 0.20
    child_certification_yield_high: float = 0.60
    child_cert_useful_yield_window: int = 5
    child_cert_no_route_yield_window: int = 5
    child_cert_dual_stability_window: int = 3
    child_cert_dual_change_tol: float = 1e-6
    child_closure_batch_growth_factor: float = 1.0
    child_closure_batch_shrink_factor: float = 1.0
    child_useful_yield_low: float = 0.20
    child_useful_yield_high: float = 0.60
    child_no_route_yield_high: float = 1.0
    enable_rmp_basis_reuse: bool = True
    sr_removal_density_weight: float = 1.0
    sr_removal_nnz_weight: float = 1.0
    sr_removal_age_weight: float = 1.0
    sr_removal_build_weight: float = 1.0
    sr_removal_violation_weight: float = 5.0
    sr_removal_dual_weight: float = 5.0
    sr_removal_score_threshold: float = 1.0
    sr_removal_max_per_node: int = 20
    use_row_local_sr_coeff_cache: bool = True
    use_dominance_prefilter_keys: bool = True
    use_promised_drone_construction: bool = False
    promised_drone_construct_time_limit: float = 5.0
    promised_drone_insert_top_k_customers: int = 20
    promised_drone_insert_top_k_pads: int = 5
    promised_drone_exchange_top_k_pairs: int = 50
    promised_drone_min_improvement: float = 1e-9
    no_drone_incumbent_trigger: bool = False
    compact_after_no_drone_incumbent: str = "small_budget"
    join_eval_budget: int = 0
    pricing_certification_slice_seconds: float = 0.0
    enable_join_lower_envelope: bool = False
    join_generator_split_threshold: int = 50_000
    join_generator_pair_batch_size: int = 10_000
    enable_bucket_join_envelope: bool = False
    enable_join_profile_cache: bool = False
    small_join_pair_threshold: int = 5_000
    small_join_cumulative_threshold: int = 250_000
    max_join_bypass_calls: int = 1_000
    small_dom_bucket_threshold: int = 100
    small_dom_cumulative_threshold: int = 500_000
    max_dom_bypass_calls: int = 2_000
    root_max_side_pool_per_call: int = 128
    side_pool_max_size: int = 20_000
    side_pool_per_customer_keep: int = 20
    join_payload_bin_width: float = 1.0
    repair_time_fraction_of_pricing: float = 0.05
    repair_time_hard_cap_root: float = 30.0
    repair_stall_limit: int = 3
    preclosure_pricing_call_interval: int = 5
    preclosure_pool_growth_limit: int = 100
    farkas_batch_size: int = 16
    seed_batch_size: int = 16
    repair_batch_size: int = 16
    enable_pricing_pruning: bool = True
    enable_bidirectional_pricing: bool = False
    pricing_parallel_workers: int = 2
    pricing_worker_backend: str = "thread"
    prefix_task_depth_root: int = 1
    prefix_task_depth_child: int = 1
    prefix_task_min_branching_for_depth2: int = 4
    logging_mode: str = "audit"
    progress_snapshot_period: int = 1
    pricing_jsonl_enabled: bool = True
    gurobi_log_dir: str | None = None

    def __post_init__(self) -> None:
        if self.threads <= 0:
            raise ValueError("threads must be positive")
        if self.time_limit <= 0:
            raise ValueError("time limit must be positive")
        if self.root_extraction_time_limit < 0 or self.route_pool_time_limit <= 0:
            raise ValueError("heuristic time limits must be nonnegative")
        if not 0.0 <= self.support_threshold < 1.0:
            raise ValueError("support threshold must be in [0, 1)")
        if self.support_best_per_customer <= 0:
            raise ValueError("support best-per-customer count must be positive")
        if self.dive_reward < 0 or self.repair_reward <= 0 or self.seed_reward <= 0:
            raise ValueError("heuristic rewards must be nonnegative, with positive repair and seed rewards")
        if self.phase_i_max_rounds <= 0:
            raise ValueError("Phase-I seeding rounds must be positive")
        if self.prefix_task_depth <= 0:
            raise ValueError("prefix_task_depth must be positive")
        if min(
            self.pricing_batch_size,
            self.min_pricing_batch_size,
            self.max_root_pricing_batch_size,
            self.nonroot_pricing_batch_size,
            self.certification_pass_interval,
            self.batch_hysteresis_window,
            self.batch_hysteresis_consecutive,
            self.pricing_yield_window,
            self.productive_yield_window,
            self.closure_attempt_batch_period,
            self.closure_batch_size,
            self.prefix_task_depth,
            self.prefix_task_depth_root,
            self.prefix_task_depth_child,
            self.prefix_task_min_branching_for_depth2,
            self.source_neighbor_task_size,
            self.sr_inactive_age_threshold,
            self.sr_removal_batch_size,
            self.sr_max_removals_per_node,
            self.postroot_sr_cut_removal_batch_size,
            self.sr_removal_min_active_count,
            self.column_inactive_age_min,
            self.column_deactivation_min_active_columns,
            self.column_deactivation_batch_size,
            self.child_closure_batch_min,
            self.child_closure_batch_initial,
            self.child_closure_batch_max,
            self.child_certification_yield_window,
            self.child_cert_useful_yield_window,
            self.child_cert_no_route_yield_window,
            self.child_cert_dual_stability_window,
            self.sr_removal_max_per_node,
            self.promised_drone_insert_top_k_customers,
            self.promised_drone_insert_top_k_pads,
            self.promised_drone_exchange_top_k_pairs,
            self.small_join_pair_threshold,
            self.small_join_cumulative_threshold,
            self.max_join_bypass_calls,
            self.join_generator_split_threshold,
            self.join_generator_pair_batch_size,
            self.small_dom_bucket_threshold,
            self.small_dom_cumulative_threshold,
            self.max_dom_bypass_calls,
            self.root_max_side_pool_per_call,
            self.side_pool_max_size,
            self.side_pool_per_customer_keep,
            self.repair_stall_limit,
            self.farkas_batch_size,
            self.seed_batch_size,
            self.repair_batch_size,
            self.preclosure_pricing_call_interval,
            self.preclosure_pool_growth_limit,
            self.progress_snapshot_period,
        ) <= 0:
            raise ValueError("pricing batch sizes must be positive")
        if self.max_root_pricing_batch_size < self.pricing_batch_size:
            raise ValueError("max root pricing batch size must be at least the initial pricing batch size")
        if self.batch_insertion_time_threshold <= 0:
            raise ValueError("batch insertion time threshold must be positive")
        if not 0.0 <= self.batch_low_insert_ratio < self.batch_high_insert_ratio:
            raise ValueError("batch hysteresis ratios must satisfy 0 <= low < high")
        if not 0.0 <= self.pricing_yield_low < self.pricing_yield_high:
            raise ValueError("pricing yield thresholds must satisfy 0 <= low < high")
        if self.productive_pricing_slice_seconds <= 0:
            raise ValueError("productive pricing slice must be positive")
        if not 0.0 < self.productive_slice_min_seconds <= self.productive_slice_max_seconds:
            raise ValueError("productive slice bounds must satisfy 0 < min <= max")
        if not self.productive_slice_min_seconds <= self.productive_pricing_slice_seconds <= self.productive_slice_max_seconds:
            raise ValueError("initial productive pricing slice must lie within the configured slice bounds")
        if not 0.0 <= self.productive_yield_low_threshold < self.productive_yield_high_threshold:
            raise ValueError("productive yield thresholds must satisfy 0 <= low < high")
        if self.closure_attempt_time_period <= 0:
            raise ValueError("closure attempt time period must be positive")
        if self.closure_certification_time_budget is not None and self.closure_certification_time_budget <= 0:
            raise ValueError("closure certification time budget must be positive when set")
        if self.productive_candidate_multiplier <= 0:
            raise ValueError("productive candidate multiplier must be positive")
        if not 0.0 <= self.pricing_diversity_batch_fraction <= 1.0:
            raise ValueError("pricing diversity batch fraction must be in [0, 1]")
        if self.first_incumbent_route_pool_time_limit < 0:
            raise ValueError("first-incumbent route-pool time limit must be nonnegative")
        if self.root_constructive_time_limit < 0:
            raise ValueError("root constructive time limit must be nonnegative")
        if self.root_compact_after_constructive not in {
            "skip",
            "small_budget",
            "full_budget",
            "conditional_small_budget",
            "conditional_wall_budget",
        }:
            raise ValueError(
                "root_compact_after_constructive must be skip, small_budget, full_budget, "
                "conditional_small_budget, or conditional_wall_budget"
            )
        if (
            self.root_compact_time_limit_after_constructive < 0
            or self.root_compact_time_limit_without_constructive < 0
            or self.root_compact_wall_time_limit < 0
            or self.root_compact_solve_time_limit < 0
        ):
            raise ValueError("root compact time limits must be nonnegative")
        if not 0.0 <= self.constructive_diversity_threshold <= 1.0:
            raise ValueError("constructive diversity threshold must be in [0, 1]")
        if self.constructive_incumbent_quality_threshold is not None and self.constructive_incumbent_quality_threshold < 0:
            raise ValueError("constructive incumbent quality threshold must be nonnegative when set")
        if (
            self.sr_removal_rmp_growth_threshold < 0
            or self.sr_removal_build_time_threshold < 0
            or self.sr_removal_active_coeff_threshold < 0
            or self.sr_activity_tolerance < 0
        ):
            raise ValueError("SR removal burden and activity parameters must be nonnegative")
        if self.column_active_value_tol < 0:
            raise ValueError("column active value tolerance must be nonnegative")
        if self.child_certification_slice_seconds < 0:
            raise ValueError("child certification slice must be nonnegative")
        if self.child_certification_max_slices_per_node is not None and self.child_certification_max_slices_per_node <= 0:
            raise ValueError("child certification max slices must be positive when set")
        if not self.child_closure_batch_min <= self.child_closure_batch_initial <= self.child_closure_batch_max:
            raise ValueError("child closure batches must satisfy min <= initial <= max")
        if not 0.0 <= self.child_certification_yield_low < self.child_certification_yield_high:
            raise ValueError("child certification yield thresholds must satisfy 0 <= low < high")
        if not 0.0 <= self.child_useful_yield_low < self.child_useful_yield_high <= 1.0:
            raise ValueError("child useful yield thresholds must satisfy 0 <= low < high <= 1")
        if not 0.0 <= self.child_no_route_yield_high <= 1.0:
            raise ValueError("child no-route yield threshold must be in [0, 1]")
        if self.child_cert_dual_change_tol < 0.0:
            raise ValueError("child certification dual change tolerance must be nonnegative")
        if self.child_closure_batch_growth_factor < 1.0 or not 0.0 < self.child_closure_batch_shrink_factor <= 1.0:
            raise ValueError("child closure batch factors must satisfy growth >= 1 and 0 < shrink <= 1")
        if (
            self.sr_removal_density_weight < 0.0
            or self.sr_removal_nnz_weight < 0.0
            or self.sr_removal_age_weight < 0.0
            or self.sr_removal_build_weight < 0.0
            or self.sr_removal_violation_weight < 0.0
            or self.sr_removal_dual_weight < 0.0
            or self.sr_removal_score_threshold < 0.0
        ):
            raise ValueError("SR removal score weights and threshold must be nonnegative")
        if self.promised_drone_construct_time_limit < 0.0:
            raise ValueError("promised drone construction time limit must be nonnegative")
        if self.promised_drone_min_improvement < 0.0:
            raise ValueError("promised drone minimum improvement must be nonnegative")
        if self.compact_after_no_drone_incumbent not in {"small_budget", "full_budget"}:
            raise ValueError("compact_after_no_drone_incumbent must be small_budget or full_budget")
        if not 0.0 <= self.dual_stabilization_weight <= 1.0:
            raise ValueError("dual stabilization weight must be in [0, 1]")
        if not 0.0 <= self.post_incumbent_primal_budget_factor <= 1.0:
            raise ValueError("post-incumbent primal budget factor must be in [0, 1]")
        if self.join_eval_budget < 0 or self.pricing_certification_slice_seconds < 0:
            raise ValueError("join evaluation budget and certification slice must be nonnegative")
        if self.join_payload_bin_width <= 0:
            raise ValueError("join payload bin width must be positive")
        if self.repair_time_fraction_of_pricing < 0 or self.repair_time_hard_cap_root < 0:
            raise ValueError("repair budget parameters must be nonnegative")
        if self.pricing_parallel_workers <= 0:
            raise ValueError("pricing parallel workers must be positive")
        if self.pricing_worker_backend not in {"thread", "process"}:
            raise ValueError("pricing worker backend must be thread or process")
        if self.logging_mode not in {"audit", "light"}:
            raise ValueError("logging_mode must be audit or light")
