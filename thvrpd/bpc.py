from __future__ import annotations

from collections import deque
from dataclasses import asdict, dataclass, field, replace
from heapq import heappop, heappush
from itertools import combinations, count
import json
from pathlib import Path
import time

from gurobipy import GRB

from .branching import BranchRestrictions
from .columns import (
    RouteSignatureCache,
    build_branch_route_index,
    extend_branch_route_index,
    insert_node_column,
    merge_duplicate_column_paths,
    query_branch_route_index,
    refresh_node_column_index,
    route_signature,
)
from .compact import solve_compact_solution
from .config import ObjectiveWeights, SolverConfig
from .heuristics import run_route_pool_heuristic
from .instance import InstanceData
from .objective import ObjectiveData, build_objective_data
from .phasei import PhaseISeeder
from .pricing import (
    _DynamicRefinementConfig,
    _KCoreBalanceConfig,
    PricingDuals,
    PricingResult,
    PricingTimeLimitReached,
    SourceNeighborPricingPool,
    price_route,
    route_reduced_cost,
)
from .rmp import NodeState, RestrictedMaster, SRCutMetadata
from .routes import Route, ServiceEnvelopeViolation, route_from_path
from .transform import build_transformed_graph, duplicate_node, is_duplicate


class _NodeTimeLimitReached(RuntimeError):
    pass


class BPCTimeLimitNoIncumbent(RuntimeError):
    def __init__(
        self,
        runtime: float,
        nodes_processed: int,
        lower_bound_shifted: float,
        stats: "BPCStats",
        objective: ObjectiveData,
    ) -> None:
        super().__init__("BPC time limit reached before a feasible incumbent was found")
        self.runtime = runtime
        self.nodes_processed = nodes_processed
        self.lower_bound_shifted = lower_bound_shifted
        self.stats = stats
        self.objective = objective


@dataclass
class _IncumbentState:
    value: float = float("inf")
    routes: tuple[Route, ...] = tuple()


@dataclass
class BPCStats:
    status: str = "unknown"
    initialization_time: float = 0.0
    root_constructive_time: float = 0.0
    root_constructive_status: str = "not_run"
    root_constructive_routes: int = 0
    root_constructive_incumbent_found: bool = False
    root_constructive_diversity_score: float = 0.0
    root_constructive_drone_sorties: int = 0
    root_constructive_truck_count: int = 0
    root_constructive_value: float | None = None
    root_compact_conditional_triggered: bool = False
    root_compact_conditional_reason: str = ""
    root_extraction_time: float = 0.0
    root_model_build_time: float = 0.0
    root_model_solve_time: float = 0.0
    root_route_decode_time: float = 0.0
    root_compact_status: str = "not_run"
    root_compact_attempted: bool = False
    root_compact_warm_start_only: bool = False
    root_compact_accepted_columns: int = 0
    root_compact_skipped_reason: str = ""
    root_compact_budget_seconds: float = 0.0
    root_compact_wall_budget_seconds: float = 0.0
    root_compact_solve_budget_seconds: float = 0.0
    root_compact_wall_budget_hit: bool = False
    root_compact_decode_verification_time: float = 0.0
    drone_diversification_attempted: bool = False
    drone_diversification_routes_generated: int = 0
    drone_diversification_columns_accepted: int = 0
    drone_diversification_incumbent_improved: bool = False
    drone_diversification_time: float = 0.0
    no_drone_incumbent_flag: bool = False
    final_selected_drone_sorties: int = 0
    drone_served_customers: int = 0
    truck_served_customers: int = 0
    drone_insertion_attempts: int = 0
    drone_insertion_verified: int = 0
    drone_exchange_attempts: int = 0
    drone_exchange_verified: int = 0
    drone_primal_pool_size: int = 0
    drone_primal_improvements: int = 0
    rmp_time: float = 0.0
    root_rmp_time: float = 0.0
    postroot_rmp_time: float = 0.0
    phase_i_time: float = 0.0
    standard_pricing_time: float = 0.0
    root_standard_pricing_time: float = 0.0
    postroot_standard_pricing_time: float = 0.0
    farkas_pricing_time: float = 0.0
    root_farkas_pricing_time: float = 0.0
    postroot_farkas_pricing_time: float = 0.0
    sr_separation_time: float = 0.0
    root_sr_separation_time: float = 0.0
    postroot_sr_separation_time: float = 0.0
    heuristic_time: float = 0.0
    root_heuristic_time: float = 0.0
    postroot_heuristic_time: float = 0.0
    farkas_certificate_rhs: float | None = None
    farkas_certificate_max_column_activity: float | None = None
    initial_routes: int = 0
    root_extracted_routes: int = 0
    total_routes: int = 0
    global_pool_routes: int = 0
    transformed_nodes: int = 0
    transformed_arcs: int = 0
    duplicate_nodes: int = 0
    duplicate_arcs: int = 0
    rmp_solves: int = 0
    phase_i_solves: int = 0
    phase_i_columns_added: int = 0
    phase_i_uncovered_customers: int = 0
    seed_pricing_calls: int = 0
    standard_pricing_calls: int = 0
    farkas_pricing_calls: int = 0
    repair_pricing_calls: int = 0
    columns_added_standard: int = 0
    columns_added_farkas: int = 0
    duplicate_columns_rejected: int = 0
    duplicate_equivalent_columns_rejected: int = 0
    cost_dominated_columns_rejected: int = 0
    cost_dominated_columns_removed: int = 0
    sr_cuts_added: int = 0
    sr_cuts_added_root: int = 0
    sr_cuts_added_postroot: int = 0
    sr_cuts_aged: int = 0
    sr_cuts_removed: int = 0
    sr_removal_nodes: int = 0
    sr_cuts_reactivated: int = 0
    sr_active_count_root: int = 0
    sr_active_count_postroot: int = 0
    sr_cut_activity_updates: int = 0
    sr_cut_dual_activity_updates: int = 0
    sr_cut_coefficient_nonzeros_observed: int = 0
    sr_cut_coefficient_density_max: float = 0.0
    sr_cut_metadata_update_time: float = 0.0
    sr_cut_repricing_after_removal: int = 0
    sr_cut_lower_bound_change_count: int = 0
    sr_cut_lower_bound_change_sum: float = 0.0
    sr_cut_lower_bound_change_max_abs: float = 0.0
    sr_removal_trigger_active_count_failures: int = 0
    sr_removal_trigger_growth_failures: int = 0
    sr_removal_trigger_build_burden_failures: int = 0
    sr_removal_trigger_active_coeff_failures: int = 0
    sr_removal_trigger_activity_failures: int = 0
    sr_removal_candidates: int = 0
    sr_removal_candidate_marks: int = 0
    sr_removal_score_max: float = 0.0
    sr_removal_rmp_growth_max: float = 0.0
    sr_removal_rmp_build_growth_max: float = 0.0
    duplicate_merge_events: int = 0
    pricing_labels_generated: int = 0
    pricing_labels_dominated: int = 0
    pricing_labels_purged: int = 0
    pricing_stale_labels_skipped: int = 0
    pricing_labels_pruned: int = 0
    pricing_standard_bound_pruned: int = 0
    pricing_farkas_bound_pruned: int = 0
    pricing_complete_routes_generated: int = 0
    pricing_max_queue_size: int = 0
    pricing_extensions_attempted: int = 0
    pricing_extensions_rejected_by_deadline: int = 0
    pricing_deadline_reachability_removed: int = 0
    pricing_reward_set_size_before_deadline: int = 0
    pricing_reward_set_size_after_deadline: int = 0
    pricing_deadline_reward_bound_calls: int = 0
    pricing_deadline_dominance_prefilter_skips: int = 0
    pricing_routes_rejected_by_deadline_in_master: int = 0
    late_rmp_routes_rejected: int = 0
    pricing_diagnostics_elapsed_seconds: float = 0.0
    pricing_max_call_elapsed_seconds: float = 0.0
    pricing_forward_labels_generated: int = 0
    pricing_backward_labels_generated: int = 0
    pricing_backward_dominance_tests: int = 0
    pricing_backward_labels_dominated: int = 0
    pricing_backward_cost_function_build_time: float = 0.0
    pricing_backward_cost_function_eval_time: float = 0.0
    pricing_join_sr_correction_time: float = 0.0
    pricing_join_active_block_time: float = 0.0
    pricing_joined_reduced_cost_evaluations: int = 0
    pricing_join_pairs_key_compatible: int = 0
    pricing_join_pairs_after_bitset_filters: int = 0
    pricing_join_lower_envelope_rejects: int = 0
    pricing_join_bucket_lower_envelope_rejects: int = 0
    pricing_join_subbucket_lower_envelope_rejects: int = 0
    pricing_join_pair_lower_envelope_rejects: int = 0
    pricing_join_queue_pushes: int = 0
    pricing_join_queue_pops: int = 0
    pricing_join_generator_queue_pushes: int = 0
    pricing_join_generator_queue_pops: int = 0
    pricing_join_generator_splits: int = 0
    pricing_join_materialized_pairs: int = 0
    pricing_join_exact_rc_evals: int = 0
    pricing_join_exact_rc_time: float = 0.0
    pricing_interface_cache_hits: int = 0
    pricing_interface_cache_misses: int = 0
    pricing_suffix_profile_cache_hits: int = 0
    pricing_suffix_profile_cache_misses: int = 0
    pricing_interface_profile_cache_hits: int = 0
    pricing_interface_profile_cache_misses: int = 0
    pricing_negative_routes_verified: int = 0
    pricing_negative_routes_inserted: int = 0
    pricing_backward_dominance_cost_tests: int = 0
    pricing_backward_dominance_cost_rejected: int = 0
    pricing_backward_exclusive_resource_violations: int = 0
    pricing_join_pairs_tested: int = 0
    pricing_joined_routes_accepted: int = 0
    pricing_forward_labeling_time: float = 0.0
    pricing_backward_labeling_time: float = 0.0
    pricing_join_time: float = 0.0
    pricing_parallel_calls: int = 0
    pricing_parallel_workers_max: int = 1
    pricing_process_cpu_time: float = 0.0
    pricing_cpu_core_equivalent_max: float = 0.0
    pricing_cpu_core_equivalent_weighted: float = 0.0
    pricing_worker_cpu_time: float = 0.0
    pricing_main_process_cpu_time: float = 0.0
    pricing_main_merge_time: float = 0.0
    pricing_core_subspace_count_max: int = 0
    pricing_core_empty_blocks_max: int = 0
    pricing_min_core_reduced_cost: float | None = None
    pricing_productive_first_hit_core_id_last: int | None = None
    pricing_productive_interrupted_cores: int = 0
    pricing_certification_core_closed_count: int = 0
    pricing_certification_core_unresolved_count: int = 0
    pricing_root_closed_by_all_cores_calls: int = 0
    pricing_stale_worker_results_discarded: int = 0
    pricing_number_of_productive_restarts: int = 0
    pricing_number_of_certification_calls: int = 0
    pricing_number_of_certification_failures_due_to_negative_column: int = 0
    pricing_number_of_certification_timeouts_unresolved: int = 0
    pricing_pool_startup_time: float = 0.0
    pricing_pool_startup_count: int = 0
    pricing_pool_reused_calls: int = 0
    pricing_pool_shutdown_time: float = 0.0
    pricing_task_submission_time: float = 0.0
    pricing_worker_payload_count: int = 0
    pricing_worker_response_count: int = 0
    pricing_candidate_paths_before_merge: int = 0
    pricing_candidate_paths_after_merge: int = 0
    pricing_decoded_routes_in_main: int = 0
    pricing_verified_routes_in_main: int = 0
    pricing_batch_target_max: int = 0
    pricing_returned_batch_size_max: int = 0
    pricing_first_hit_enabled_calls: int = 0
    pricing_stale_response_rejections: int = 0
    pricing_worker_backend_thread_calls: int = 0
    pricing_worker_backend_process_calls: int = 0
    pricing_source_neighbor_count_max: int = 0
    pricing_source_neighbor_task_count_max: int = 0
    pricing_source_neighbor_task_size_max: int = 0
    pricing_initial_source_neighbors_max: int = 0
    pricing_initial_task_count_max: int = 0
    pricing_initial_load_imbalance_max_mean: float = 0.0
    pricing_empty_initial_blocks_max: int = 0
    pricing_idle_worker_seconds: float = 0.0
    pricing_dynamic_split_candidates: int = 0
    pricing_dynamic_splits_performed: int = 0
    pricing_dynamic_split_rejected_close_to_closure: int = 0
    pricing_dynamic_split_rejected_small_queue: int = 0
    pricing_dynamic_split_rejected_short_elapsed: int = 0
    pricing_dynamic_split_rejected_low_workload: int = 0
    pricing_dynamic_child_tasks_created: int = 0
    pricing_dynamic_labels_transferred: int = 0
    pricing_dynamic_split_overhead_seconds: float = 0.0
    pricing_leaf_tasks_closed: int = 0
    pricing_leaf_tasks_stale: int = 0
    pricing_best_active_task_gap_max: float = 0.0
    pricing_open_labels_by_task_max: int = 0
    pricing_epoch_invalidations_dual: int = 0
    pricing_epoch_invalidations_sr: int = 0
    pricing_epoch_invalidations_residual_branch: int = 0
    pricing_epoch_invalidations_fixed_routes: int = 0
    pricing_epoch_invalidations_active_columns: int = 0
    pricing_epoch_invalidations_rmp_structure: int = 0
    pricing_epoch_invalidations_objective_window: int = 0
    pricing_stale_task_reuse_attempts: int = 0
    pricing_stale_task_reuse_blocked: int = 0
    pricing_cross_task_dominance_attempts: int = 0
    pricing_cross_task_dominance_blocked: int = 0
    pricing_local_worker_candidate_quota_max: int = 0
    pricing_diversity_quota_max: int = 0
    pricing_diversity_selected_routes: int = 0
    pricing_diversity_selected_customers_max: int = 0
    pricing_productive_slice_deadline_calls: int = 0
    pricing_productive_slice_time: float = 0.0
    pricing_productive_time_limit_with_columns: int = 0
    pricing_productive_time_limit_no_columns: int = 0
    adaptive_slice_seconds_min: float = 0.0
    adaptive_slice_seconds_max: float = 0.0
    adaptive_slice_seconds_last: float = 0.0
    adaptive_slice_increases: int = 0
    adaptive_slice_decreases: int = 0
    productive_yield_window_rate: float = 0.0
    productive_batches_since_last_cert: int = 0
    productive_time_since_last_cert: float = 0.0
    certification_time_limit_with_columns: int = 0
    certification_time_limit_no_columns: int = 0
    prefix_task_depth: int = 1
    stabilized_dual_enabled: bool = False
    stabilized_candidates_returned: int = 0
    true_dual_rejected_candidates: int = 0
    max_abs_worker_true_rc_discrepancy: float = 0.0
    post_incumbent_heuristic_budget: float = 0.0
    post_incumbent_repair_budget: float = 0.0
    pricing_first_hit_exits: int = 0
    pricing_interrupted_worker_calls: int = 0
    pricing_certification_worker_calls: int = 0
    pricing_productive_worker_calls: int = 0
    pricing_signature_cache_hits: int = 0
    pricing_signature_cache_misses: int = 0
    pricing_core_signature_cache_hits: int = 0
    pricing_core_signature_cache_misses: int = 0
    pricing_active_signature_cache_hits: int = 0
    pricing_active_signature_cache_misses: int = 0
    pricing_sr_coeff_cache_hits: int = 0
    pricing_sr_coeff_cache_misses: int = 0
    pricing_active_sr_key_cache_hits: int = 0
    pricing_active_sr_key_cache_misses: int = 0
    pricing_active_sr_coeffs_computed: int = 0
    pricing_triplet_masks_built: int = 0
    pricing_dominance_prefilter_pairs: int = 0
    pricing_dominance_prefilter_rejected: int = 0
    pricing_dominance_bucket_pairs_considered: int = 0
    pricing_dominance_bucket_pairs_rejected: int = 0
    pricing_dominance_bucket_candidate_pairs: int = 0
    pricing_dominance_bucket_queries: int = 0
    pricing_dominance_bucket_skipped_by_mask: int = 0
    pricing_dominance_bucket_skipped_by_scalar: int = 0
    pricing_dominance_bucket_skipped_by_branch: int = 0
    pricing_dominance_bucket_skipped_by_deadline: int = 0
    pricing_dominance_bucket_skipped_by_return_credit: int = 0
    pricing_dom_frontier_queries: int = 0
    pricing_dom_frontier_keys_scanned: int = 0
    pricing_dom_frontier_keys_skipped_by_mask: int = 0
    pricing_dom_frontier_keys_skipped_by_branch: int = 0
    pricing_dom_frontier_keys_skipped_by_deadline: int = 0
    pricing_dom_frontier_keys_skipped_by_return_credit: int = 0
    pricing_frontier_cells_created: int = 0
    pricing_frontier_cells_split: int = 0
    pricing_frontier_cell_lb_closed: int = 0
    pricing_frontier_cell_lb_invalidations: int = 0
    pricing_mask_trie_subset_queries: int = 0
    pricing_mask_trie_superset_queries: int = 0
    pricing_mask_trie_returned_items: int = 0
    pricing_mask_subset_queries: int = 0
    pricing_mask_superset_queries: int = 0
    pricing_mask_query_cache_hits: int = 0
    pricing_mask_query_cache_misses: int = 0
    pricing_cell_splits: int = 0
    pricing_cell_pair_products_before_split: int = 0
    pricing_cell_pairs_considered: int = 0
    pricing_cell_pairs_rejected_by_mask: int = 0
    pricing_cell_pairs_rejected_by_envelope: int = 0
    pricing_cell_pairs_rejected_by_lb: int = 0
    pricing_cell_pairs_rejected_by_closure_lb: int = 0
    pricing_label_pairs_materialized: int = 0
    pricing_labels_certified_by_cell_lb: int = 0
    pricing_full_same_node_tests: int = 0
    pricing_full_physical_location_tests: int = 0
    pricing_labels_deleted_same_node: int = 0
    pricing_labels_deleted_physical_location: int = 0
    pricing_closure_queue_pushes: int = 0
    pricing_closure_queue_pops: int = 0
    pricing_certification_tasks_exhausted_by_cell_lb: int = 0
    pricing_certification_tasks_closed_by_cell_lb: int = 0
    pricing_certification_tasks_exhausted_by_label_search: int = 0
    pricing_resource_reward_bound_calls: int = 0
    pricing_resource_reward_bound_time: float = 0.0
    pricing_resource_reward_bound_fallbacks: int = 0
    pricing_physdom_cell_pairs_considered: int = 0
    pricing_physdom_cell_pairs_rejected_by_mask: int = 0
    pricing_physdom_cell_pairs_rejected_by_envelope: int = 0
    pricing_physdom_label_pairs_materialized: int = 0
    pricing_physdom_full_tests: int = 0
    pricing_physdom_deletions: int = 0
    pricing_physdom_time: float = 0.0
    pricing_return_credit_incompatible_pairs: int = 0
    pricing_dom_pairs_avoided_before_materialization: int = 0
    pricing_dom_candidate_pairs_materialized: int = 0
    pricing_dom_full_tests_same_node: int = 0
    pricing_dom_full_tests_physical_location: int = 0
    pricing_dom_labels_deleted_same_node: int = 0
    pricing_dom_labels_deleted_physical_location: int = 0
    pricing_dominance_compatible_keys_generated: int = 0
    pricing_dominance_compatible_key_lookups: int = 0
    pricing_dominance_bucket_scans_avoided: int = 0
    pricing_dominance_key_generation_time: float = 0.0
    pricing_backward_full_dominance_tests: int = 0
    pricing_join_prefilter_pairs: int = 0
    pricing_join_prefilter_rejected: int = 0
    pricing_join_bucket_pairs_considered: int = 0
    pricing_join_bucket_pairs_rejected: int = 0
    pricing_join_bucket_candidate_pairs: int = 0
    pricing_join_compatible_keys_generated: int = 0
    pricing_join_compatible_key_lookups: int = 0
    pricing_join_bucket_scans_avoided: int = 0
    pricing_join_key_generation_time: float = 0.0
    pricing_join_key_cache_hits: int = 0
    pricing_join_key_cache_misses: int = 0
    pricing_join_graph_build_time: float = 0.0
    pricing_join_subbucket_pairs_considered: int = 0
    pricing_join_subbucket_pairs_rejected: int = 0
    pricing_join_small_bypass_calls: int = 0
    pricing_join_local_bypass_calls: int = 0
    pricing_join_cumulative_bypass_calls: int = 0
    pricing_join_indexed_activation_count: int = 0
    pricing_join_work_estimate: int = 0
    pricing_join_candidate_pairs_accepted: int = 0
    pricing_join_label_pairs_materialized: int = 0
    pricing_join_full_decodes: int = 0
    pricing_lazy_rejected_before_decode: int = 0
    pricing_fully_decoded_routes: int = 0
    pricing_duplicate_equivalent_rejected: int = 0
    pricing_cost_dominated_rejected: int = 0
    pricing_signature_build_time: float = 0.0
    pricing_sr_coeff_build_time: float = 0.0
    pricing_duplicate_lookup_time: float = 0.0
    pricing_route_decode_time: float = 0.0
    pricing_reduced_cost_verification_time: float = 0.0
    pricing_dominance_key_cache_hits: int = 0
    pricing_dominance_key_cache_misses: int = 0
    pricing_dominance_small_bypass_calls: int = 0
    pricing_dominance_bypass_calls: int = 0
    pricing_dominance_indexed_activation_count: int = 0
    pricing_dominance_work_estimate: int = 0
    pricing_sticky_indexed_join_activations: int = 0
    pricing_sticky_indexed_dominance_activations: int = 0
    pricing_join_stage_reject_key: int = 0
    pricing_join_stage_reject_branch: int = 0
    pricing_join_stage_reject_customer: int = 0
    pricing_join_stage_reject_truck_node: int = 0
    pricing_join_stage_reject_payload: int = 0
    pricing_join_stage_reject_block: int = 0
    pricing_join_stage_reject_reduced_cost: int = 0
    pricing_dominance_stage_reject_key: int = 0
    pricing_dominance_stage_reject_branch: int = 0
    pricing_dominance_stage_reject_customer: int = 0
    pricing_dominance_stage_reject_truck_node: int = 0
    pricing_dominance_stage_reject_payload: int = 0
    pricing_dominance_stage_reject_block: int = 0
    pricing_dominance_stage_reject_time: int = 0
    pricing_dominance_stage_reject_cost: int = 0
    pricing_forward_dominance_tests: int = 0
    pricing_forward_same_node_dominance_tests: int = 0
    pricing_forward_physical_location_dominance_tests: int = 0
    pricing_forward_physical_location_dominance_rejections: int = 0
    pricing_forward_return_time_credit_checks: int = 0
    pricing_forward_return_time_credit_checks_skipped: int = 0
    pricing_forward_branch_language_failures: int = 0
    pricing_forward_mask_scalar_prefilter_failures: int = 0
    pricing_dom_gate_pairs_seen: int = 0
    pricing_dom_gate_mask_failures: int = 0
    pricing_dom_gate_scalar_failures: int = 0
    pricing_dom_gate_branch_failures: int = 0
    pricing_dom_gate_deadline_failures: int = 0
    pricing_labels_dominated_same_node: int = 0
    pricing_labels_dominated_physical: int = 0
    pricing_dom_prefilter_pairs: int = 0
    pricing_dom_prefilter_mask_fail: int = 0
    pricing_dom_prefilter_branch_fail: int = 0
    pricing_dom_prefilter_payload_fail: int = 0
    pricing_dom_prefilter_block_fail: int = 0
    pricing_dom_prefilter_return_credit_fail: int = 0
    pricing_dom_full_tests: int = 0
    pricing_dom_full_rejections: int = 0
    pricing_physical_location_full_tests: int = 0
    pricing_physical_location_rejections: int = 0
    pricing_productive_mode_calls: int = 0
    pricing_closure_mode_calls: int = 0
    pricing_yield_ratio: float = 0.0
    closure_attempts: int = 0
    closure_attempts_returned_columns: int = 0
    closure_attempts_exhausted: int = 0
    closure_attempts_time_limited: int = 0
    pricing_side_pool_routes_returned: int = 0
    pricing_side_pool_reduced_cost_min: float | None = None
    pricing_side_pool_candidates_seen: int = 0
    pricing_side_pool_routes_retained: int = 0
    pricing_side_pool_routes_rejected_by_budget: int = 0
    seed_pricing_time: float = 0.0
    repair_pricing_time: float = 0.0
    pricing_diagnostics: list[dict] = field(default_factory=list)
    route_pool_hydration_time: float = 0.0
    route_signature_build_time: float = 0.0
    sr_coeff_build_time: float = 0.0
    duplicate_lookup_time: float = 0.0
    rmp_column_insertion_time: float = 0.0
    rmp_build_time: float = 0.0
    rmp_solve_time: float = 0.0
    rmp_incremental_updates: int = 0
    rmp_full_rebuilds: int = 0
    rmp_incremental_update_time: float = 0.0
    rmp_active_coefficient_cache_hits: int = 0
    rmp_active_coefficient_cache_misses: int = 0
    rmp_active_sr_nonzero_count: int = 0
    rmp_active_sr_coefficient_count: int = 0
    rmp_active_sr_nonzero_density_max: float = 0.0
    active_sr_rows_added: int = 0
    active_sr_rows_removed: int = 0
    active_sr_row_local_updates: int = 0
    active_sr_full_rebuilds: int = 0
    rmp_compatibility_failure_residual_customers: int = 0
    rmp_compatibility_failure_fixed_routes: int = 0
    rmp_compatibility_failure_fleet_limit: int = 0
    rmp_compatibility_failure_fixed_cost: int = 0
    rmp_compatibility_failure_branch_state: int = 0
    rmp_compatibility_failure_active_sr: int = 0
    rmp_compatibility_failure_active_sr_version: int = 0
    rmp_compatibility_failure_service_deadline_version: int = 0
    rmp_compatibility_failure_objective_scale_version: int = 0
    rmp_compatibility_failure_active_column_version: int = 0
    rmp_compatibility_failure_rmp_structure_version: int = 0
    max_positive_le_dual_violation: float = 0.0
    max_negative_ge_dual_violation: float = 0.0
    sr_dual_sign_violations: int = 0
    fleet_dual_sign_violations: int = 0
    rmp_basis_reuse_attempts: int = 0
    rmp_basis_reuse_success: int = 0
    rmp_basis_reuse_time: float = 0.0
    rmp_basis_store_time: float = 0.0
    column_hydration_time: float = 0.0
    column_signature_lookup_time: float = 0.0
    progress_serialization_time: float = 0.0
    progress_events_seen: int = 0
    progress_events_written: int = 0
    progress_events_skipped: int = 0
    signature_cache_hits: int = 0
    signature_cache_misses: int = 0
    core_signature_cache_hits: int = 0
    core_signature_cache_misses: int = 0
    active_signature_cache_hits: int = 0
    active_signature_cache_misses: int = 0
    sr_coeff_cache_hits: int = 0
    sr_coeff_cache_misses: int = 0
    active_sr_key_cache_hits: int = 0
    active_sr_key_cache_misses: int = 0
    active_sr_coeffs_computed: int = 0
    triplet_masks_built: int = 0
    column_index_hits: int = 0
    column_index_misses: int = 0
    column_index_replacements: int = 0
    column_index_refreshes: int = 0
    column_index_refresh_time: float = 0.0
    active_sr_version_refreshes: int = 0
    active_sr_version_refresh_time: float = 0.0
    heuristic_calls: int = 0
    preclosure_heuristic_calls: int = 0
    preclosure_heuristic_columns_generated: int = 0
    preclosure_heuristic_incumbent_updates: int = 0
    time_to_first_incumbent: float | None = None
    first_incumbent_route_pool_calls: int = 0
    first_incumbent_route_pool_time: float = 0.0
    first_incumbent_route_pool_feasible_solves: int = 0
    first_incumbent_route_pool_routes_max: int = 0
    incumbent_source: str = "none"
    heuristic_columns_generated: int = 0
    heuristic_incumbent_updates: int = 0
    heuristic_hard_pool_solves: int = 0
    heuristic_hard_pool_time: float = 0.0
    heuristic_hard_pool_feasible_solves: int = 0
    heuristic_support_pool_calls: int = 0
    heuristic_support_pool_time: float = 0.0
    heuristic_support_pool_feasible: int = 0
    heuristic_support_pool_incumbent_updates: int = 0
    heuristic_full_pool_calls: int = 0
    heuristic_full_pool_time: float = 0.0
    heuristic_full_pool_feasible: int = 0
    heuristic_full_pool_incumbent_updates: int = 0
    heuristic_node_pool_to_support_ratio_max: float = 0.0
    heuristic_soft_pool_solves: int = 0
    heuristic_soft_pool_time: float = 0.0
    heuristic_soft_pool_feasible_solves: int = 0
    heuristic_repair_customers: int = 0
    heuristic_repair_columns_generated: int = 0
    postroot_heuristic_calls: int = 0
    postroot_heuristic_hard_pool_solves: int = 0
    postroot_heuristic_support_pool_calls: int = 0
    postroot_heuristic_full_pool_calls: int = 0
    postroot_heuristic_soft_pool_solves: int = 0
    postroot_repair_calls: int = 0
    postroot_repair_columns_generated: int = 0
    postroot_heuristic_incumbent_updates: int = 0
    postroot_heuristic_budget_hits: int = 0
    heuristic_max_node_pool_routes: int = 0
    heuristic_max_support_routes: int = 0
    side_pool_routes: int = 0
    side_pool_routes_added: int = 0
    side_pool_routes_pruned: int = 0
    side_pool_max_routes: int = 0
    side_pool_prune_time: float = 0.0
    side_pool_batch_size_last: int = 0
    side_pool_candidates_seen: int = 0
    side_pool_routes_rejected_by_budget: int = 0
    side_pool_per_customer_keep: int = 0
    repair_budget_hit: int = 0
    heuristic_budget_hit: int = 0
    heuristic_stall_count: int = 0
    branching_nodes: int = 0
    customer_pair_branches: int = 0
    service_mode_branches: int = 0
    launch_pad_branches: int = 0
    transformed_arc_branches: int = 0
    route_variable_branches: int = 0
    child_nodes_created: int = 0
    child_inherited_route_candidates: int = 0
    child_inherited_route_accepted: int = 0
    child_inherited_route_rejected: int = 0
    child_branch_index_build_time: float = 0.0
    child_branch_index_query_time: float = 0.0
    child_branch_index_candidates_before: int = 0
    child_branch_index_candidates_after: int = 0
    child_branch_index_reject_route_forbidden: int = 0
    child_branch_index_reject_together: int = 0
    child_branch_index_reject_separate: int = 0
    child_branch_index_reject_service_mode: int = 0
    child_branch_index_reject_launch_pad: int = 0
    child_branch_index_reject_transformed_arc: int = 0
    global_branch_index_builds: int = 0
    global_branch_index_incremental_updates: int = 0
    global_branch_index_incremental_paths: int = 0
    global_branch_index_queries: int = 0
    global_branch_index_build_time: float = 0.0
    global_branch_index_query_time: float = 0.0
    global_branch_index_candidates: int = 0
    global_branch_index_rejections: int = 0
    child_reject_residual: int = 0
    child_reject_fixed_route: int = 0
    child_reject_fleet: int = 0
    child_reject_branch: int = 0
    child_reject_deadline: int = 0
    child_reject_sr_signature: int = 0
    child_reject_route_signature: int = 0
    child_hydration_time: float = 0.0
    child_refresh_count: int = 0
    child_refresh_time: float = 0.0
    postroot_nodes_processed: int = 0
    postroot_nodes_closed: int = 0
    child_certification_calls: int = 0
    child_certification_calls_with_columns: int = 0
    child_certification_calls_exhausted: int = 0
    child_certification_time_limited: int = 0
    child_certification_slice_seconds: float = 0.0
    child_certification_slice_limited_calls: int = 0
    child_certification_resumed_calls: int = 0
    child_certification_state_discards: int = 0
    child_certification_epochs_started: int = 0
    child_certification_epochs_completed: int = 0
    child_certification_state_saved: int = 0
    child_certification_state_resumed: int = 0
    child_certification_state_discarded_by_dual: int = 0
    child_certification_state_discarded_by_sr: int = 0
    child_certification_state_discarded_by_residual: int = 0
    child_certification_state_discarded_by_branch: int = 0
    child_certification_state_discarded_by_fixed_routes: int = 0
    child_certification_state_discarded_by_active_columns: int = 0
    child_certification_state_discarded_by_rmp_structure: int = 0
    child_certification_exhausted_tasks: int = 0
    child_certification_unresolved_tasks: int = 0
    child_closure_batch_min: int = 0
    child_closure_batch_max: int = 0
    child_closure_batch_last: int = 0
    child_closure_batch_increases: int = 0
    child_closure_batch_decreases: int = 0
    child_certification_yield_rate: float = 0.0
    child_certification_yield_observations: int = 0
    child_useful_yield_rate: float = 0.0
    child_no_route_yield_rate: float = 0.0
    child_dual_stable_observations: int = 0
    child_dual_unstable_observations: int = 0
    postroot_open_nodes: int = 0
    best_open_bound: float | None = None
    max_node_columns: int = 0
    max_active_sr: int = 0
    max_active_sr_version: int = 0
    root_closed: bool = False
    best_reduced_cost_at_stop: float | None = None
    open_nodes_at_termination: int = 0
    adaptive_batch_size_min: int = 64
    adaptive_batch_size_max: int = 64
    adaptive_batch_size_last: int = 64
    adaptive_batch_increases: int = 0
    adaptive_batch_decreases: int = 0
    adaptive_batch_window_ratio: float = 0.0
    adaptive_batch_low_windows: int = 0
    adaptive_batch_high_windows: int = 0
    certification_pricing_passes: int = 0
    certification_pricing_passes_with_columns: int = 0
    root_productive_pricing_calls: int = 0
    root_columns_since_last_heuristic: int = 0
    inactive_columns_deactivated: int = 0
    inactive_columns_rehydrated: int = 0
    inactive_column_deactivation_events: int = 0
    inactive_column_rehydration_events: int = 0
    inactive_column_reduced_cost_checks: int = 0
    inactive_column_reduced_cost_time: float = 0.0
    inactive_column_lower_bound_change_count: int = 0
    inactive_column_lower_bound_change_sum: float = 0.0
    inactive_column_lower_bound_change_max_abs: float = 0.0
    inactive_columns_at_stop: int = 0


@dataclass(frozen=True)
class BranchDecision:
    branch_type: str
    left: NodeState | None
    right: NodeState | None


@dataclass
class BPCResult:
    status: str
    objective_shifted: float
    objective_full: float
    routes: tuple[Route, ...]
    runtime: float
    nodes_processed: int
    lower_bound_shifted: float
    lower_bound_full: float
    upper_bound_shifted: float
    upper_bound_full: float
    gap: float
    gap_full: float
    gap_shifted: float
    objective: ObjectiveData
    stats: BPCStats

    def to_record(self) -> dict:
        customer_service_times = {
            customer: route.service_times[customer]
            for route in self.routes
            for customer in route.served
        }
        return {
            "status": self.status,
            "objective_shifted": self.objective_shifted,
            "objective_full": self.objective_full,
            "runtime": self.runtime,
            "nodes_processed": self.nodes_processed,
            "lower_bound_shifted": self.lower_bound_shifted,
            "lower_bound_full": self.lower_bound_full,
            "upper_bound_shifted": self.upper_bound_shifted,
            "upper_bound_full": self.upper_bound_full,
            "gap": self.gap,
            "gap_full": self.gap_full,
            "gap_shifted": self.gap_shifted,
            "normalization_bounds": asdict(self.objective.bounds),
            "objective_coefficients": asdict(self.objective.coeffs),
            "objective_components": _objective_components(self.routes, self.objective),
            "customer_service_times": dict(sorted(customer_service_times.items())),
            "return_times": [route.return_time for route in self.routes],
            "drone_sorties": [
                {"route_id": route.id, "pad_served": sorted(route.pad_served)}
                for route in self.routes
                if route.pad_served
            ],
            "routes": [route.to_record(self.objective) for route in self.routes],
            "bpc_stats": asdict(self.stats),
        }


@dataclass
class _BatchRoundStats:
    pricing_time: float
    insertion_time: float
    inserted_columns: int


@dataclass
class _BatchController:
    batch_size: int
    min_batch_size: int
    max_batch_size: int
    window_size: int
    low_ratio: float
    high_ratio: float
    consecutive_required: int
    window: deque[_BatchRoundStats] = field(default_factory=deque)
    low_windows: int = 0
    high_windows: int = 0
    last_ratio: float = 0.0

    def observe(self, round_stats: _BatchRoundStats) -> str | None:
        self.window.append(round_stats)
        while len(self.window) > self.window_size:
            self.window.popleft()
        if len(self.window) < self.window_size:
            return None
        pricing_time = sum(item.pricing_time for item in self.window)
        insertion_time = sum(item.insertion_time for item in self.window)
        self.last_ratio = insertion_time / max(pricing_time, 1e-12)
        productive = all(item.inserted_columns > 0 for item in self.window)
        if productive and self.last_ratio <= self.low_ratio:
            self.low_windows += 1
            self.high_windows = 0
        elif self.last_ratio >= self.high_ratio:
            self.high_windows += 1
            self.low_windows = 0
        else:
            self.low_windows = 0
            self.high_windows = 0
        if self.low_windows >= self.consecutive_required and self.batch_size < self.max_batch_size:
            self.batch_size = min(self.batch_size * 2, self.max_batch_size)
            self.low_windows = 0
            return "increase"
        if self.high_windows >= self.consecutive_required and self.batch_size > self.min_batch_size:
            self.batch_size = max(self.batch_size // 2, self.min_batch_size)
            self.high_windows = 0
            return "decrease"
        return None


@dataclass
class _PricingYieldController:
    window_size: int
    low_yield: float
    high_yield: float
    window: deque[tuple[float, int]] = field(default_factory=deque)
    last_ratio: float = 0.0
    mode: str = "productive"

    def observe(self, pricing_time: float, returned_routes: int) -> str:
        self.window.append((pricing_time, returned_routes))
        while len(self.window) > self.window_size:
            self.window.popleft()
        total_time = sum(item[0] for item in self.window)
        total_routes = sum(item[1] for item in self.window)
        self.last_ratio = total_routes / max(total_time, 1e-12)
        if len(self.window) < self.window_size:
            self.mode = "productive"
        elif self.last_ratio <= self.low_yield:
            self.mode = "closure"
        elif self.last_ratio >= self.high_yield:
            self.mode = "productive"
        return self.mode


@dataclass
class _ClosureAwarePricingState:
    batches_since_cert: int = 0
    time_since_cert: float = 0.0
    force_next_certification: bool = False

    def observe_productive_batch(self, elapsed: float) -> None:
        self.batches_since_cert += 1
        self.time_since_cert += elapsed

    def reset_after_certification_attempt(self) -> None:
        self.batches_since_cert = 0
        self.time_since_cert = 0.0
        self.force_next_certification = False

    def should_certify(self, solver_config: SolverConfig) -> bool:
        return (
            self.force_next_certification
            or self.batches_since_cert >= solver_config.closure_attempt_batch_period
            or self.time_since_cert >= solver_config.closure_attempt_time_period
        )


@dataclass
class _AdaptiveProductiveSliceController:
    current_seconds: float
    min_seconds: float
    max_seconds: float
    low_threshold: float
    high_threshold: float
    window_size: int
    history: deque[tuple[float, int, int]] = field(default_factory=deque)
    last_yield_rate: float = 0.0
    increases: int = 0
    decreases: int = 0
    observed_min: float = 0.0
    observed_max: float = 0.0

    def __post_init__(self) -> None:
        self.current_seconds = min(max(self.current_seconds, self.min_seconds), self.max_seconds)
        self.observed_min = self.current_seconds
        self.observed_max = self.current_seconds

    def observe(self, elapsed: float, verified_count: int, inserted_count: int) -> str:
        self.history.append((elapsed, verified_count, inserted_count))
        while len(self.history) > self.window_size:
            self.history.popleft()
        total_elapsed = sum(item[0] for item in self.history)
        total_verified = sum(item[1] for item in self.history)
        self.last_yield_rate = total_verified / total_elapsed if total_elapsed > 0.0 else 0.0
        old = self.current_seconds
        if self.last_yield_rate < self.low_threshold:
            self.current_seconds = max(self.min_seconds, 0.5 * self.current_seconds)
        elif self.last_yield_rate > self.high_threshold:
            self.current_seconds = min(self.max_seconds, 1.25 * self.current_seconds)
        self.observed_min = min(self.observed_min, self.current_seconds)
        self.observed_max = max(self.observed_max, self.current_seconds)
        if self.current_seconds < old:
            self.decreases += 1
            return "decrease"
        if self.current_seconds > old:
            self.increases += 1
            return "increase"
        return "keep"


def _blend_pricing_duals(current: PricingDuals, previous: PricingDuals | None, weight: float) -> PricingDuals | None:
    if previous is None:
        return None
    if set(current.mu) != set(previous.mu) or set(current.nu) != set(previous.nu):
        return None
    return PricingDuals(
        mu={customer: weight * current.mu[customer] + (1.0 - weight) * previous.mu[customer] for customer in current.mu},
        kappa=weight * current.kappa + (1.0 - weight) * previous.kappa,
        nu={triplet: weight * current.nu[triplet] + (1.0 - weight) * previous.nu[triplet] for triplet in current.nu},
    )


def _closure_certification_deadline(solver_config: SolverConfig, deadline: float) -> float:
    if solver_config.closure_certification_time_budget is None:
        return deadline
    return min(deadline, time.time() + solver_config.closure_certification_time_budget)


def _objective_components(routes: tuple[Route, ...], objective: ObjectiveData) -> dict:
    raw_delay = sum(route.delay_square_sum for route in routes)
    raw_return = sum(route.return_time for route in routes)
    raw_cost = sum(route.operating_cost for route in routes)
    normalized_delay = _normalized_component(raw_delay, objective.bounds.delay_lb, objective.bounds.delay_ub)
    normalized_return = _normalized_component(raw_return, objective.bounds.return_lb, objective.bounds.return_ub)
    normalized_cost = _normalized_component(raw_cost, objective.bounds.cost_lb, objective.bounds.cost_ub)
    shifted_delay = objective.coeffs.delay * raw_delay
    shifted_return = objective.coeffs.return_time * raw_return
    shifted_cost = objective.coeffs.cost * raw_cost
    return {
        "raw": {
            "delay_square_sum": raw_delay,
            "return_time_sum": raw_return,
            "operating_cost": raw_cost,
        },
        "normalized": {
            "delay": normalized_delay,
            "return_time": normalized_return,
            "cost": normalized_cost,
        },
        "weighted_normalized": {
            "delay": objective.weights.delay * normalized_delay,
            "return_time": objective.weights.return_time * normalized_return,
            "cost": objective.weights.cost * normalized_cost,
        },
        "shifted_route_sum_components": {
            "delay": shifted_delay,
            "return_time": shifted_return,
            "cost": shifted_cost,
            "shift": objective.coeffs.shift,
            "route_sum_without_shift": shifted_delay + shifted_return + shifted_cost,
            "full_with_shift": shifted_delay + shifted_return + shifted_cost + objective.coeffs.shift,
        },
    }


def _normalized_component(value: float, lower: float, upper: float) -> float:
    if upper == lower:
        return 0.0
    return (value - lower) / (upper - lower)


def solve_branch_price_cut(
    instance: InstanceData,
    weights: ObjectiveWeights,
    solver_config: SolverConfig,
) -> BPCResult:
    start = time.time()
    deadline = start + solver_config.time_limit
    stats = BPCStats()
    init_start = time.time()
    objective = build_objective_data(instance, weights)
    graph = build_transformed_graph(instance)
    stats.transformed_nodes = len(graph.nodes)
    stats.transformed_arcs = len(graph.arcs)
    stats.duplicate_nodes = len(graph.duplicate_nodes)
    stats.duplicate_arcs = len(graph.hub_duplicate_arcs) + len(graph.duplicate_duplicate_arcs) + len(graph.duplicate_regular_arcs)
    signature_cache = RouteSignatureCache()
    routes: dict[tuple[str, ...], Route] = {}
    _add_initial_routes(graph, objective, routes)
    stats.initial_routes = len(routes)
    if solver_config.enable_constructive_root_incumbent:
        constructive_start = time.time()
        root_constructive_paths, root_constructive_selected = _construct_root_incumbent_routes(
            graph,
            objective,
            solver_config,
            routes,
            stats,
            deadline,
        )
        stats.root_constructive_time = time.time() - constructive_start
    else:
        root_constructive_paths, root_constructive_selected = set(), tuple()
        stats.root_constructive_status = "skipped"
    incumbent = _IncumbentState()
    if root_constructive_selected:
        selected = tuple(routes[path] for path in root_constructive_selected)
        stats.root_constructive_truck_count = len(selected)
        stats.root_constructive_drone_sorties = sum(len(route.pad_served) for route in selected)
        stats.root_constructive_value = sum(route.cost for route in selected)
        stats.root_constructive_diversity_score = _constructive_route_diversity_score(instance, selected)
        covered = frozenset().union(*(route.served for route in selected))
        if covered == frozenset(instance.customers) and len(selected) <= instance.num_trucks:
            incumbent.value = stats.root_constructive_value
            incumbent.routes = selected
            stats.root_constructive_incumbent_found = True
            stats.time_to_first_incumbent = time.time() - start
            stats.incumbent_source = "root_constructive"
    stats.no_drone_incumbent_flag = incumbent.value < float("inf") and sum(route.drone_sorties for route in incumbent.routes) == 0
    diversified_paths = _diversify_constructive_drone_routes(
        graph,
        objective,
        solver_config,
        routes,
        stats,
        incumbent,
        root_constructive_selected,
        start,
        deadline,
    )
    stats.no_drone_incumbent_flag = incumbent.value < float("inf") and sum(route.drone_sorties for route in incumbent.routes) == 0
    global_pool_paths: set[tuple[str, ...]] = set(routes)
    stats.initialization_time = time.time() - init_start
    stats.root_compact_warm_start_only = incumbent.value < float("inf")
    root_start = time.time()
    root_extracted = _extract_root_routes(
        instance,
        weights,
        solver_config,
        graph,
        objective,
        routes,
        stats,
        deadline,
        constructive_incumbent_found=stats.root_constructive_incumbent_found,
    )
    stats.root_extraction_time = time.time() - root_start
    stats.root_extracted_routes = len(root_extracted)
    stats.root_compact_accepted_columns = len(root_extracted)
    global_pool_paths.update(root_constructive_paths)
    global_pool_paths.update(diversified_paths)
    global_pool_paths.update(root_extracted)
    root = NodeState(
        id=1,
        depth=0,
        restrictions=BranchRestrictions(),
        fixed_routes=tuple(),
        residual_customers=frozenset(instance.customers),
        fleet_limit=instance.num_trucks,
        fixed_cost=0.0,
        column_paths=set(global_pool_paths),
        active_sr=set(),
    )
    node_ids = count(2)
    queue: list[tuple[float, int, NodeState]] = []
    heappush(queue, (0.0, root.id, root))
    nodes_processed = 0
    if root_extracted:
        selected = tuple(routes[path] for path in sorted(root_extracted))
        covered = frozenset().union(*(route.served for route in selected))
        value = sum(route.cost for route in selected)
        if covered == frozenset(instance.customers) and len(selected) <= instance.num_trucks and value < incumbent.value:
            incumbent.value = value
            incumbent.routes = selected
            if stats.time_to_first_incumbent is None:
                stats.time_to_first_incumbent = time.time() - start
            stats.incumbent_source = "compact_root"
    pricing_pool = (
        SourceNeighborPricingPool(
            graph,
            objective,
            solver_config.pricing_parallel_workers,
            solver_config.source_neighbor_task_size,
            _pricing_balance_config(solver_config),
            _pricing_refinement_config(solver_config),
        )
        if solver_config.pricing_worker_backend == "process" and solver_config.pricing_parallel_workers > 1
        else None
    )
    branch_index_cache: dict[tuple[tuple[str, ...], int], object] = {}

    while queue:
        if time.time() >= deadline:
            stats.status = "time_limit"
            break
        node_queue_bound, _, node = heappop(queue)
        nodes_processed += 1
        if node.depth > 0:
            stats.postroot_nodes_processed += 1
        try:
            node_result = _solve_node(
                graph,
                objective,
                solver_config,
                node,
                routes,
                global_pool_paths,
                signature_cache,
                incumbent,
                stats,
                deadline,
                start,
                pricing_pool,
                branch_index_cache,
            )
        except _NodeTimeLimitReached:
            stats.status = "time_limit"
            heappush(queue, (node_queue_bound, node.id, node))
            break
        if node_result is None:
            continue
        node_bound, z_values = node_result
        if node_bound >= incumbent.value - solver_config.integrality_tolerance:
            continue
        heuristic_start = time.time()
        try:
            heuristic = run_route_pool_heuristic(
                graph,
                objective,
                node,
                routes,
                global_pool_paths,
                z_values,
                solver_config,
                len(routes),
                incumbent.value,
                deadline,
                pricing_process_pool=pricing_pool,
            )
        except PricingTimeLimitReached as exc:
            _record_heuristic_time(stats, node, time.time() - heuristic_start)
            stats.heuristic_calls += 1
            if node.depth > 0:
                stats.postroot_heuristic_calls += 1
            _record_pricing_timeout(stats, "heuristic_interrupted", exc, solver_config, node, routes, global_pool_paths, signature_cache)
            heappush(queue, (node_bound, node.id, node))
            break
        _record_heuristic_time(stats, node, time.time() - heuristic_start)
        stats.heuristic_calls += 1
        if node.depth > 0:
            stats.postroot_heuristic_calls += 1
        stats.heuristic_columns_generated += len(heuristic.generated_paths)
        _record_heuristic_diagnostics(stats, heuristic.diagnostics)
        if node.depth > 0:
            _record_postroot_heuristic_diagnostics(stats, heuristic.diagnostics)
        for diagnostic in heuristic.pricing_diagnostics:
            _record_pricing_diagnostic_dict(stats, diagnostic, solver_config)
        global_pool_paths.update(heuristic.generated_paths)
        _sync_route_stats(stats, routes, global_pool_paths, signature_cache)
        if heuristic.value is not None and heuristic.value < incumbent.value:
            incumbent.value = heuristic.value
            incumbent.routes = node.fixed_routes + heuristic.selected_routes
            if stats.time_to_first_incumbent is None:
                stats.time_to_first_incumbent = time.time() - start
                stats.incumbent_source = "route_pool_after_node_close"
            stats.heuristic_incumbent_updates += 1
            if node.depth > 0:
                stats.postroot_heuristic_incumbent_updates += 1
        if _is_integer(z_values, solver_config.integrality_tolerance):
            selected = tuple(routes[path] for path, value in z_values.items() if value > 0.5)
            value = sum(route.cost for route in selected) + node.fixed_cost
            if value < incumbent.value:
                incumbent.value = value
                incumbent.routes = node.fixed_routes + selected
                if stats.time_to_first_incumbent is None:
                    stats.time_to_first_incumbent = time.time() - start
                    stats.incumbent_source = "rmp_integer_solution"
            continue
        decision = _branch(node, z_values, routes, graph, next(node_ids), next(node_ids), solver_config)
        stats.branching_nodes += 1
        _record_branch_decision(stats, decision.branch_type)
        left, right = decision.left, decision.right
        if left is not None:
            _inherit_child_routes(graph, objective, node, left, routes, global_pool_paths, stats, signature_cache, solver_config, branch_index_cache)
            heappush(queue, (node_bound, left.id, left))
            stats.child_nodes_created += 1
        if right is not None:
            _inherit_child_routes(graph, objective, node, right, routes, global_pool_paths, stats, signature_cache, solver_config, branch_index_cache)
            heappush(queue, (node_bound, right.id, right))
            stats.child_nodes_created += 1

    if queue:
        lower_bound = min(bound for bound, _, _ in queue)
        stats.best_open_bound = lower_bound
    else:
        lower_bound = incumbent.value
        stats.best_open_bound = None
        if stats.status == "unknown":
            stats.status = "optimal"
    if nodes_processed == 0:
        if pricing_pool is not None:
            pricing_pool.shutdown()
            stats.pricing_pool_shutdown_time += pricing_pool.shutdown_time_seconds
        if stats.status == "time_limit":
            stats.open_nodes_at_termination = len(queue)
            stats.postroot_open_nodes = sum(1 for _, _, queued_node in queue if queued_node.depth > 0)
            stats.best_open_bound = min((bound for bound, _, _ in queue), default=None)
            _sync_route_stats(stats, routes, global_pool_paths, signature_cache)
            raise BPCTimeLimitNoIncumbent(time.time() - start, nodes_processed, lower_bound, stats, objective)
        raise RuntimeError("BPC time limit elapsed before processing the root node")
    if incumbent.value == float("inf"):
        if pricing_pool is not None:
            pricing_pool.shutdown()
            stats.pricing_pool_shutdown_time += pricing_pool.shutdown_time_seconds
        stats.open_nodes_at_termination = len(queue)
        stats.postroot_open_nodes = sum(1 for _, _, queued_node in queue if queued_node.depth > 0)
        stats.best_open_bound = min((bound for bound, _, _ in queue), default=lower_bound)
        _sync_route_stats(stats, routes, global_pool_paths, signature_cache)
        raise BPCTimeLimitNoIncumbent(time.time() - start, nodes_processed, lower_bound, stats, objective)
    if stats.status == "unknown":
        stats.status = "time_limit"
    stats.open_nodes_at_termination = len(queue)
    stats.postroot_open_nodes = sum(1 for _, _, queued_node in queue if queued_node.depth > 0)
    _record_final_incumbent_service_mix(stats, incumbent.routes)
    _sync_route_stats(stats, routes, global_pool_paths, signature_cache)
    lower_bound_full = objective.full_value_from_route_sum(lower_bound)
    upper_bound_full = objective.full_value_from_route_sum(incumbent.value)
    gap_shifted = _relative_gap(incumbent.value, lower_bound)
    gap_full = _relative_gap(upper_bound_full, lower_bound_full)
    if pricing_pool is not None:
        pricing_pool.shutdown()
        stats.pricing_pool_shutdown_time += pricing_pool.shutdown_time_seconds
    return BPCResult(
        status=stats.status,
        objective_shifted=incumbent.value,
        objective_full=upper_bound_full,
        routes=incumbent.routes,
        runtime=time.time() - start,
        nodes_processed=nodes_processed,
        lower_bound_shifted=lower_bound,
        lower_bound_full=lower_bound_full,
        upper_bound_shifted=incumbent.value,
        upper_bound_full=upper_bound_full,
        gap=gap_full,
        gap_full=gap_full,
        gap_shifted=gap_shifted,
        objective=objective,
        stats=stats,
    )


def _record_final_incumbent_service_mix(stats: BPCStats, selected_routes: tuple[Route, ...]) -> None:
    stats.final_selected_drone_sorties = sum(route.drone_sorties for route in selected_routes)
    drone_served = set()
    truck_served = set()
    for route in selected_routes:
        drone_served.update(route.drone_served)
        truck_served.update(route.truck_served)
    stats.drone_served_customers = len(drone_served)
    stats.truck_served_customers = len(truck_served)


def _relative_gap(upper_bound: float, lower_bound: float) -> float:
    if upper_bound == 0.0:
        return 0.0
    return max((upper_bound - lower_bound) / abs(upper_bound), 0.0)


def _constructive_route_diversity_score(instance: InstanceData, routes: tuple[Route, ...]) -> float:
    if not routes or not instance.customers:
        return 0.0
    drone_served = frozenset().union(*(route.drone_served for route in routes))
    truck_utilization = min(len(routes) / max(instance.num_trucks, 1), 1.0)
    drone_service_share = len(drone_served) / len(instance.customers)
    route_length_values = [len(route.served) for route in routes if route.served]
    if len(route_length_values) <= 1:
        balance = 1.0
    else:
        mean_length = sum(route_length_values) / len(route_length_values)
        imbalance = sum(abs(value - mean_length) for value in route_length_values) / (len(route_length_values) * max(mean_length, 1.0))
        balance = max(0.0, 1.0 - imbalance)
    return (drone_service_share + (1.0 - truck_utilization) + balance) / 3.0


def _record_rmp_time(stats: BPCStats, node: NodeState, elapsed: float) -> None:
    stats.rmp_time += elapsed
    if node.depth == 0:
        stats.root_rmp_time += elapsed
    else:
        stats.postroot_rmp_time += elapsed


def _record_standard_pricing_time(stats: BPCStats, node: NodeState, elapsed: float) -> None:
    stats.standard_pricing_time += elapsed
    if node.depth == 0:
        stats.root_standard_pricing_time += elapsed
    else:
        stats.postroot_standard_pricing_time += elapsed


def _record_farkas_pricing_time(stats: BPCStats, node: NodeState, elapsed: float) -> None:
    stats.farkas_pricing_time += elapsed
    if node.depth == 0:
        stats.root_farkas_pricing_time += elapsed
    else:
        stats.postroot_farkas_pricing_time += elapsed


def _record_sr_separation_time(stats: BPCStats, node: NodeState, elapsed: float) -> None:
    stats.sr_separation_time += elapsed
    if node.depth == 0:
        stats.root_sr_separation_time += elapsed
    else:
        stats.postroot_sr_separation_time += elapsed


def _record_heuristic_time(stats: BPCStats, node: NodeState, elapsed: float) -> None:
    stats.heuristic_time += elapsed
    if node.depth == 0:
        stats.root_heuristic_time += elapsed
    else:
        stats.postroot_heuristic_time += elapsed


def _solve_node(
    graph,
    objective: ObjectiveData,
    solver_config: SolverConfig,
    node: NodeState,
    routes: dict[tuple[str, ...], Route],
    global_pool_paths: set[tuple[str, ...]],
    signature_cache: RouteSignatureCache,
    incumbent: _IncumbentState,
    stats: BPCStats,
    deadline: float,
    solver_start: float,
    pricing_pool: SourceNeighborPricingPool | None,
    branch_index_cache: dict,
) -> tuple[float, dict[tuple[str, ...], float]] | None:
    root_batch = _BatchController(
        batch_size=solver_config.pricing_batch_size if node.depth == 0 else solver_config.nonroot_pricing_batch_size,
        min_batch_size=solver_config.min_pricing_batch_size,
        max_batch_size=solver_config.max_root_pricing_batch_size,
        window_size=solver_config.batch_hysteresis_window,
        low_ratio=solver_config.batch_low_insert_ratio,
        high_ratio=solver_config.batch_high_insert_ratio,
        consecutive_required=solver_config.batch_hysteresis_consecutive,
    )
    closure_state = _ClosureAwarePricingState()
    slice_controller = _AdaptiveProductiveSliceController(
        current_seconds=solver_config.productive_pricing_slice_seconds,
        min_seconds=solver_config.productive_slice_min_seconds,
        max_seconds=solver_config.productive_slice_max_seconds,
        low_threshold=solver_config.productive_yield_low_threshold,
        high_threshold=solver_config.productive_yield_high_threshold,
        window_size=solver_config.productive_yield_window,
    )
    stable_duals: PricingDuals | None = None
    root_columns_since_heuristic = 0
    root_heuristic_stall_count = 0
    side_pool_paths: set[tuple[str, ...]] = set()
    child_certification_slices_used = 0
    stats.child_certification_slice_seconds = solver_config.child_certification_slice_seconds
    stats.child_closure_batch_min = solver_config.child_closure_batch_min
    stats.child_closure_batch_max = solver_config.child_closure_batch_max
    stats.adaptive_batch_size_min = min(stats.adaptive_batch_size_min, root_batch.batch_size)
    stats.adaptive_batch_size_max = max(stats.adaptive_batch_size_max, root_batch.batch_size)
    stats.adaptive_batch_size_last = root_batch.batch_size
    stats.adaptive_slice_seconds_min = slice_controller.observed_min
    stats.adaptive_slice_seconds_max = slice_controller.observed_max
    stats.adaptive_slice_seconds_last = slice_controller.current_seconds
    stats.prefix_task_depth = max(stats.prefix_task_depth, _prefix_task_depth_for_node(solver_config, node, stats))
    while True:
        if time.time() >= deadline:
            stats.status = "time_limit"
            _sync_route_stats(stats, routes, global_pool_paths, signature_cache)
            _write_progress(solver_config, stats, node, "time_limit_node_unresolved")
            raise _NodeTimeLimitReached()
        _hydrate_node_from_pool(graph, node, routes, global_pool_paths, stats, signature_cache, solver_config, branch_index_cache)
        _sync_route_stats(stats, routes, global_pool_paths, signature_cache)
        stats.max_node_columns = max(stats.max_node_columns, len(node.column_paths))
        stats.max_active_sr = max(stats.max_active_sr, len(node.active_sr))
        stats.max_active_sr_version = max(stats.max_active_sr_version, node.active_sr_version)
        rmp_start = time.time()
        rmp_build_start = time.time()
        cache_hits_before = signature_cache.stats.sr_coeff_cache_hits + signature_cache.stats.active_sr_key_cache_hits
        cache_misses_before = signature_cache.stats.sr_coeff_cache_misses + signature_cache.stats.active_sr_key_cache_misses
        rmp = RestrictedMaster(graph, node, routes, solver_config, signature_cache)
        rmp_build_elapsed = time.time() - rmp_build_start
        stats.rmp_build_time += rmp_build_elapsed
        if rmp.used_incremental_update:
            stats.rmp_incremental_updates += 1
            stats.rmp_incremental_update_time += rmp_build_elapsed
        else:
            stats.rmp_full_rebuilds += 1
            _record_rmp_compatibility_failures(stats, rmp.compatibility_failure_reasons)
        stats.rmp_active_sr_nonzero_count += rmp.active_sr_nonzero_count
        stats.rmp_active_sr_coefficient_count += rmp.active_sr_coefficient_count
        stats.active_sr_rows_added += rmp.active_sr_rows_added
        stats.active_sr_rows_removed += rmp.active_sr_rows_removed
        stats.active_sr_row_local_updates += rmp.active_sr_row_local_updates
        stats.active_sr_full_rebuilds += rmp.active_sr_full_rebuilds
        if rmp.active_sr_coefficient_count:
            stats.rmp_active_sr_nonzero_density_max = max(
                stats.rmp_active_sr_nonzero_density_max,
                rmp.active_sr_nonzero_count / rmp.active_sr_coefficient_count,
            )
        stats.rmp_active_coefficient_cache_hits += (
            signature_cache.stats.sr_coeff_cache_hits
            + signature_cache.stats.active_sr_key_cache_hits
            - cache_hits_before
        )
        stats.rmp_active_coefficient_cache_misses += (
            signature_cache.stats.sr_coeff_cache_misses
            + signature_cache.stats.active_sr_key_cache_misses
            - cache_misses_before
        )
        if node.previous_rmp_build_time > 0.0:
            node.rmp_build_time_growth = max(
                (rmp_build_elapsed - node.previous_rmp_build_time) / node.previous_rmp_build_time,
                0.0,
            )
            stats.sr_removal_rmp_build_growth_max = max(stats.sr_removal_rmp_build_growth_max, node.rmp_build_time_growth)
        node.previous_rmp_build_time = rmp_build_elapsed
        basis_start = time.time()
        if solver_config.enable_rmp_basis_reuse and node.basis_variables:
            stats.rmp_basis_reuse_attempts += 1
            if rmp.load_basis_from_node():
                stats.rmp_basis_reuse_success += 1
        stats.rmp_basis_reuse_time += time.time() - basis_start
        rmp_solve_start = time.time()
        result = rmp.solve()
        _record_dual_sign_diagnostics(stats, result.duals)
        rmp_solve_elapsed = time.time() - rmp_solve_start
        stats.rmp_solve_time += rmp_solve_elapsed
        if node.previous_rmp_solve_time > 0.0:
            node.rmp_solve_time_growth = max(
                (rmp_solve_elapsed - node.previous_rmp_solve_time) / node.previous_rmp_solve_time,
                0.0,
            )
            stats.sr_removal_rmp_growth_max = max(stats.sr_removal_rmp_growth_max, node.rmp_solve_time_growth)
        node.previous_rmp_solve_time = rmp_solve_elapsed
        basis_store_start = time.time()
        rmp.store_basis_to_node()
        stats.rmp_basis_store_time += time.time() - basis_store_start
        _record_rmp_time(stats, node, time.time() - rmp_start)
        stats.rmp_solves += 1
        if node.pending_sr_removal_bound is not None and result.objective is not None:
            bound_change = result.objective - node.pending_sr_removal_bound
            stats.sr_cut_lower_bound_change_count += 1
            stats.sr_cut_lower_bound_change_sum += bound_change
            stats.sr_cut_lower_bound_change_max_abs = max(
                stats.sr_cut_lower_bound_change_max_abs,
                abs(bound_change),
            )
            node.pending_sr_removal_bound = None
        if node.pending_column_deactivation_bound is not None and result.objective is not None:
            bound_change = result.objective - node.pending_column_deactivation_bound
            stats.inactive_column_lower_bound_change_count += 1
            stats.inactive_column_lower_bound_change_sum += bound_change
            stats.inactive_column_lower_bound_change_max_abs = max(
                stats.inactive_column_lower_bound_change_max_abs,
                abs(bound_change),
            )
            node.pending_column_deactivation_bound = None
        _write_progress(solver_config, stats, node, "rmp_solved", {"rmp_status": result.status})
        if result.status == GRB.INFEASIBLE:
            stats.farkas_certificate_rhs = result.farkas_rhs
            stats.farkas_certificate_max_column_activity = result.max_farkas_column_activity
            phase_i_added = _run_phase_i_seeding(
                graph,
                objective,
                solver_config,
                node,
                routes,
                global_pool_paths,
                signature_cache,
                stats,
                deadline,
                pricing_pool,
            )
            if phase_i_added:
                heuristic_config, repair_budget = _root_primal_budgets(
                    solver_config,
                    stats,
                    incumbent,
                    first_incumbent_attempt=False,
                )
                improved = False
                if heuristic_config is not None:
                    improved = _run_preclosure_heuristic(
                        graph,
                        objective,
                        heuristic_config,
                        node,
                        routes,
                        global_pool_paths,
                        side_pool_paths,
                        signature_cache,
                        {},
                        stats,
                        incumbent,
                        deadline,
                        solver_start,
                        "after_phase_i_seed",
                        allow_repair=repair_budget > 0.0,
                        repair_time_budget=repair_budget,
                        pricing_pool=pricing_pool,
                    )
                root_heuristic_stall_count = 0 if improved else root_heuristic_stall_count + 1
                stats.heuristic_stall_count = root_heuristic_stall_count
                continue
            pricing_start = time.time()
            try:
                priced = price_route(
                    graph,
                    objective,
                    node.residual_customers,
                    node.restrictions,
                    result.duals,
                    len(routes),
                    farkas=True,
                    pricing_tolerance=solver_config.pricing_tolerance,
                    batch_size=solver_config.farkas_batch_size,
                    deadline=deadline,
                    enable_bidirectional=solver_config.enable_bidirectional_pricing,
                    parallel_workers=solver_config.pricing_parallel_workers,
                    pricing_worker_backend=solver_config.pricing_worker_backend,
                    existing_routes=routes,
                    existing_column_paths=set(node.column_paths),
                    small_join_pair_threshold=solver_config.small_join_pair_threshold,
                    small_join_cumulative_threshold=solver_config.small_join_cumulative_threshold,
                    max_join_bypass_calls=solver_config.max_join_bypass_calls,
                    small_dom_bucket_threshold=solver_config.small_dom_bucket_threshold,
                    small_dom_cumulative_threshold=solver_config.small_dom_cumulative_threshold,
                    max_dom_bypass_calls=solver_config.max_dom_bypass_calls,
                    join_payload_bin_width=solver_config.join_payload_bin_width,
                    join_eval_budget=solver_config.join_eval_budget,
                    pricing_certification_slice_seconds=solver_config.pricing_certification_slice_seconds,
                    enable_join_lower_envelope=solver_config.enable_join_lower_envelope,
                    join_generator_split_threshold=solver_config.join_generator_split_threshold,
                    join_generator_pair_batch_size=solver_config.join_generator_pair_batch_size,
                    enable_bucket_join_envelope=solver_config.enable_bucket_join_envelope,
                    enable_join_profile_cache=solver_config.enable_join_profile_cache,
                    side_pool_batch_size=0,
                    pricing_process_pool=pricing_pool,
                    productive_candidate_multiplier=solver_config.productive_candidate_multiplier,
                    source_neighbor_task_size=solver_config.source_neighbor_task_size,
                    pricing_diversity_batch_fraction=solver_config.pricing_diversity_batch_fraction,
                    prefix_task_depth=_prefix_task_depth_for_node(solver_config, node, stats),
                    enable_mask_trie_frontier=solver_config.enable_mask_trie_frontier,
                    max_frontier_cell_size=solver_config.max_frontier_cell_size,
                    max_frontier_pair_product=solver_config.max_frontier_pair_product,
                    max_frontier_split_depth=solver_config.max_frontier_split_depth,
                    enable_resource_restricted_closure_bound=solver_config.enable_resource_restricted_closure_bound,
                    resource_bound_method=solver_config.resource_bound_method,
                    **_pricing_scheduler_kwargs(solver_config),
            )
            except PricingTimeLimitReached as exc:
                _record_farkas_pricing_time(stats, node, time.time() - pricing_start)
                stats.farkas_pricing_calls += 1
                _record_pricing_timeout(stats, "farkas_interrupted", exc, solver_config, node, routes, global_pool_paths, signature_cache)
                raise _NodeTimeLimitReached()
            _record_farkas_pricing_time(stats, node, time.time() - pricing_start)
            stats.farkas_pricing_calls += 1
            _record_pricing_diagnostic(stats, "farkas", priced, solver_config)
            if not priced.routes:
                _write_progress(solver_config, stats, node, "farkas_certificate")
                return None
            added_paths = []
            for route in priced.routes:
                path, added = _insert_node_column(route, routes, node, graph, stats, signature_cache, objective)
                global_pool_paths.add(path)
                if not added:
                    stats.duplicate_columns_rejected += 1
                    raise RuntimeError("Farkas pricing returned a duplicate-equivalent column that should already satisfy the certificate")
                stats.columns_added_farkas += 1
                added_paths.append(path)
            _sync_route_stats(stats, routes, global_pool_paths, signature_cache)
            _write_progress(solver_config, stats, node, "farkas_columns_added", {"paths": added_paths})
            continue
        _update_node_column_ages(node, result.z_values, solver_config, stats)
        if _rehydrate_negative_inactive_columns(graph, objective, node, routes, result.duals, solver_config, stats, signature_cache):
            _write_progress(solver_config, stats, node, "inactive_columns_rehydrated")
            continue
        pricing_start = time.time()
        closure_attempt = node.depth == 0 and closure_state.should_certify(solver_config)
        child_certification_attempt = node.depth > 0 and not solver_config.child_productive_before_certification
        certification_attempt = closure_attempt or child_certification_attempt
        pricing_mode = "closure" if certification_attempt else "productive"
        if certification_attempt:
            stats.certification_pricing_passes += 1
        if closure_attempt:
            stats.closure_attempts += 1
        if child_certification_attempt:
            stats.child_certification_calls += 1
            if solver_config.enable_resumable_child_certification:
                cert_signature = _child_certification_signature(node, result.duals)
                if node.child_certification_signature == cert_signature:
                    stats.child_certification_resumed_calls += 1
                    stats.child_certification_state_resumed += 1
                elif node.child_certification_signature is not None:
                    stats.child_certification_state_discards += 1
                    _record_child_certification_epoch_discard(
                        stats,
                        node.child_certification_signature,
                        cert_signature,
                    )
                    node.child_certification_exhausted_task_count = 0
                    node.child_certification_unresolved_task_count = 0
                    stats.child_certification_epochs_started += 1
                else:
                    stats.child_certification_epochs_started += 1
                node.child_certification_signature = cert_signature
        standard_batch_size = (
            solver_config.closure_batch_size
            if certification_attempt
            else root_batch.batch_size if node.depth == 0 else solver_config.nonroot_pricing_batch_size
        )
        standard_deadline = _closure_certification_deadline(solver_config, deadline) if closure_attempt else deadline
        if (
            child_certification_attempt
            and solver_config.child_certification_max_slices_per_node is not None
            and solver_config.child_certification_slice_seconds > 0.0
        ):
            if child_certification_slices_used >= solver_config.child_certification_max_slices_per_node:
                stats.child_certification_time_limited += 1
                stats.certification_time_limit_no_columns += 1
                _write_progress(solver_config, stats, node, "child_certification_slice_limit")
                raise _NodeTimeLimitReached()
            standard_deadline = min(deadline, time.time() + solver_config.child_certification_slice_seconds)
            child_certification_slices_used += 1
            stats.child_certification_slice_limited_calls += 1
        productive_slice_used = False
        productive_slice_seconds = 0.0
        if (
            node.depth == 0
            and pricing_mode == "productive"
        ):
            productive_slice_seconds = slice_controller.current_seconds
            standard_deadline = min(deadline, time.time() + slice_controller.current_seconds)
            productive_slice_used = standard_deadline < deadline
        search_duals = (
            _blend_pricing_duals(result.duals, stable_duals, solver_config.dual_stabilization_weight)
            if (
                solver_config.use_dual_stabilized_productive_search
                and pricing_mode == "productive"
                and not closure_attempt
            )
            else None
        )
        try:
            priced = price_route(
                graph,
                objective,
                node.residual_customers,
                node.restrictions,
                result.duals,
                len(routes),
                farkas=False,
                pricing_tolerance=solver_config.pricing_tolerance,
                use_standard_acceleration=solver_config.enable_pricing_pruning,
                stop_at_first_negative=standard_batch_size == 1,
                batch_size=standard_batch_size,
                deadline=standard_deadline,
                enable_bidirectional=solver_config.enable_bidirectional_pricing,
                parallel_workers=solver_config.pricing_parallel_workers,
                pricing_worker_backend=solver_config.pricing_worker_backend,
                existing_routes=routes,
                existing_column_paths=set(node.column_paths),
                small_join_pair_threshold=solver_config.small_join_pair_threshold,
                small_join_cumulative_threshold=solver_config.small_join_cumulative_threshold,
                max_join_bypass_calls=solver_config.max_join_bypass_calls,
                small_dom_bucket_threshold=solver_config.small_dom_bucket_threshold,
                small_dom_cumulative_threshold=solver_config.small_dom_cumulative_threshold,
                max_dom_bypass_calls=solver_config.max_dom_bypass_calls,
                join_payload_bin_width=solver_config.join_payload_bin_width,
                join_eval_budget=solver_config.join_eval_budget,
                pricing_certification_slice_seconds=solver_config.pricing_certification_slice_seconds,
                enable_join_lower_envelope=solver_config.enable_join_lower_envelope,
                join_generator_split_threshold=solver_config.join_generator_split_threshold,
                join_generator_pair_batch_size=solver_config.join_generator_pair_batch_size,
                enable_bucket_join_envelope=solver_config.enable_bucket_join_envelope,
                enable_join_profile_cache=solver_config.enable_join_profile_cache,
                side_pool_batch_size=(
                    solver_config.root_max_side_pool_per_call
                    if node.depth == 0 and pricing_mode == "productive"
                    else 0
                ),
                pricing_mode=pricing_mode,
                pricing_yield_ratio=slice_controller.last_yield_rate,
                pricing_process_pool=pricing_pool,
                productive_candidate_multiplier=solver_config.productive_candidate_multiplier,
                source_neighbor_task_size=solver_config.source_neighbor_task_size,
                pricing_diversity_batch_fraction=solver_config.pricing_diversity_batch_fraction,
                productive_slice_seconds=productive_slice_seconds,
                productive_slice_deadline_used=productive_slice_used,
                adaptive_slice_seconds=slice_controller.current_seconds,
                productive_yield_window_rate=slice_controller.last_yield_rate,
                search_duals=search_duals,
                prefix_task_depth=_prefix_task_depth_for_node(solver_config, node, stats),
                enable_mask_trie_frontier=solver_config.enable_mask_trie_frontier,
                max_frontier_cell_size=solver_config.max_frontier_cell_size,
                max_frontier_pair_product=solver_config.max_frontier_pair_product,
                max_frontier_split_depth=solver_config.max_frontier_split_depth,
                enable_resource_restricted_closure_bound=solver_config.enable_resource_restricted_closure_bound,
                resource_bound_method=solver_config.resource_bound_method,
                **_pricing_scheduler_kwargs(solver_config),
            )
        except PricingTimeLimitReached as exc:
            _record_standard_pricing_time(stats, node, time.time() - pricing_start)
            stats.standard_pricing_calls += 1
            if certification_attempt:
                if child_certification_attempt:
                    stats.child_certification_time_limited += 1
                    _observe_child_certification_yield(
                        node,
                        0,
                        solver_config,
                        stats,
                        no_route=False,
                    )
                    if solver_config.enable_resumable_child_certification:
                        unresolved = len(exc.diagnostics.source_neighbor_block_sizes) or exc.diagnostics.source_neighbor_task_count
                        node.child_certification_unresolved_task_count = unresolved
                        stats.child_certification_unresolved_tasks += unresolved
                        stats.child_certification_state_saved += 1
                if closure_attempt:
                    stats.closure_attempts_time_limited += 1
                stats.certification_time_limit_no_columns += 1
                _record_pricing_timeout(stats, "standard_closure_interrupted", exc, solver_config, node, routes, global_pool_paths, signature_cache)
                raise _NodeTimeLimitReached()
            if productive_slice_used and time.time() < deadline:
                _record_productive_slice_timeout(
                    stats,
                    exc,
                    solver_config,
                    node,
                    routes,
                    global_pool_paths,
                    signature_cache,
                    productive_slice_seconds=productive_slice_seconds,
                )
                slice_controller.observe(time.time() - pricing_start, 0, 0)
                _sync_closure_aware_pricing_stats(stats, closure_state, slice_controller, solver_config)
                closure_state.force_next_certification = True
                continue
            _record_pricing_timeout(stats, "standard_interrupted", exc, solver_config, node, routes, global_pool_paths, signature_cache)
            raise _NodeTimeLimitReached()
        pricing_elapsed = time.time() - pricing_start
        _record_standard_pricing_time(stats, node, pricing_elapsed)
        stats.standard_pricing_calls += 1
        _record_pricing_diagnostic(stats, "standard", priced, solver_config)
        stable_duals = result.duals
        if node.depth == 0 and pricing_mode == "productive":
            slice_controller.observe(pricing_elapsed, priced.diagnostics.negative_routes_verified, len(priced.routes))
            _sync_closure_aware_pricing_stats(stats, closure_state, slice_controller, solver_config)
        if priced.routes:
            insertion_start = time.time()
            added_paths = []
            for route, reduced_cost in zip(priced.routes, priced.reduced_costs):
                if reduced_cost >= -solver_config.pricing_tolerance:
                    raise RuntimeError("standard pricing returned a nonnegative column in the entering batch")
                path, added = _insert_node_column(route, routes, node, graph, stats, signature_cache, objective)
                global_pool_paths.add(path)
                if not added:
                    stats.duplicate_columns_rejected += 1
                    raise RuntimeError("standard pricing returned a negative duplicate-equivalent column already in the RMP")
                stats.columns_added_standard += 1
                added_paths.append(path)
            _add_side_pool_routes(
                priced.side_pool_routes,
                routes,
                side_pool_paths,
                node,
                graph,
                stats,
                solver_config,
                result.z_values,
                objective,
                {route.path: cost for route, cost in zip(priced.side_pool_routes, priced.side_pool_reduced_costs)},
            )
            insertion_elapsed = time.time() - insertion_start
            if node.depth == 0:
                if pricing_mode == "productive":
                    stats.root_productive_pricing_calls += 1
                    closure_state.observe_productive_batch(pricing_elapsed)
                    _sync_closure_aware_pricing_stats(stats, closure_state, slice_controller, solver_config)
                elif closure_attempt:
                    stats.closure_attempts_returned_columns += 1
                    closure_state.reset_after_certification_attempt()
                    _sync_closure_aware_pricing_stats(stats, closure_state, slice_controller, solver_config)
                elif child_certification_attempt:
                    stats.child_certification_calls_with_columns += 1
                if certification_attempt:
                    stats.certification_pricing_passes_with_columns += 1
                    if priced.diagnostics.termination_reason == "time_limit_with_columns":
                        stats.certification_time_limit_with_columns += 1
                root_columns_since_heuristic += len(added_paths)
                stats.root_columns_since_last_heuristic = root_columns_since_heuristic
                if pricing_mode == "productive":
                    batch_action = root_batch.observe(
                        _BatchRoundStats(
                            pricing_time=pricing_elapsed,
                            insertion_time=insertion_elapsed,
                            inserted_columns=len(added_paths),
                        )
                    )
                    if batch_action == "increase":
                        stats.adaptive_batch_increases += 1
                    elif batch_action == "decrease":
                        stats.adaptive_batch_decreases += 1
                    stats.adaptive_batch_window_ratio = root_batch.last_ratio
                    stats.adaptive_batch_low_windows = root_batch.low_windows
                    stats.adaptive_batch_high_windows = root_batch.high_windows
                    stats.adaptive_batch_size_min = min(stats.adaptive_batch_size_min, root_batch.batch_size)
                    stats.adaptive_batch_size_max = max(stats.adaptive_batch_size_max, root_batch.batch_size)
                    stats.adaptive_batch_size_last = root_batch.batch_size
            if node.depth > 0 and child_certification_attempt:
                stats.child_certification_calls_with_columns += 1
                stats.certification_pricing_passes_with_columns += 1
                _observe_child_certification_yield(
                    node,
                    len(added_paths),
                    solver_config,
                    stats,
                    no_route=False,
                )
                if solver_config.enable_resumable_child_certification:
                    node.child_certification_signature = None
                    node.child_certification_exhausted_task_count = 0
                    node.child_certification_unresolved_task_count = 0
                    stats.child_certification_state_discards += 1
                    stats.child_certification_state_discarded_by_active_columns += 1
                if priced.diagnostics.termination_reason == "time_limit_with_columns":
                    stats.certification_time_limit_with_columns += 1
            _sync_route_stats(stats, routes, global_pool_paths, signature_cache)
            first_incumbent_ready = (
                incumbent.value == float("inf")
                and solver_config.first_incumbent_route_pool_time_limit > 0.0
                and len(added_paths) >= 32
            )
            if (
                node.depth == 0
                and pricing_mode == "productive"
                and (
                    first_incumbent_ready
                    or stats.root_productive_pricing_calls % solver_config.preclosure_pricing_call_interval == 0
                    or root_columns_since_heuristic >= solver_config.preclosure_pool_growth_limit
                )
            ):
                first_incumbent_attempt = first_incumbent_ready
                heuristic_config, repair_budget = _root_primal_budgets(
                    solver_config,
                    stats,
                    incumbent,
                    first_incumbent_attempt=first_incumbent_attempt,
                )
                allow_repair = False
                improved = False
                if heuristic_config is not None:
                    allow_repair = repair_budget > 0.0 and (
                        root_heuristic_stall_count < solver_config.repair_stall_limit
                        or root_columns_since_heuristic >= solver_config.preclosure_pool_growth_limit
                        or first_incumbent_attempt
                    )
                    if not allow_repair:
                        stats.repair_budget_hit += 1
                    improved = _run_preclosure_heuristic(
                        graph,
                        objective,
                        heuristic_config,
                        node,
                        routes,
                        global_pool_paths,
                        side_pool_paths,
                        signature_cache,
                        result.z_values,
                        stats,
                        incumbent,
                        deadline,
                        solver_start,
                        "first_incumbent_after_standard_batch" if first_incumbent_attempt else "after_standard_batch",
                        allow_repair=allow_repair,
                        repair_time_budget=repair_budget,
                        pricing_pool=pricing_pool,
                    )
                root_heuristic_stall_count = 0 if improved else root_heuristic_stall_count + 1
                stats.heuristic_stall_count = root_heuristic_stall_count
                root_columns_since_heuristic = 0
                stats.root_columns_since_last_heuristic = 0
            _write_progress(solver_config, stats, node, "standard_columns_added", {"paths": added_paths})
            continue
        if node.depth == 0 and not closure_attempt:
            closure_state.force_next_certification = True
            _sync_closure_aware_pricing_stats(stats, closure_state, slice_controller, solver_config)
            _write_progress(
                solver_config,
                stats,
                node,
                "productive_no_columns_not_certified",
                {"pricing_mode": pricing_mode},
            )
            continue
        if closure_attempt:
            stats.closure_attempts_exhausted += 1
            closure_state.reset_after_certification_attempt()
            _sync_closure_aware_pricing_stats(stats, closure_state, slice_controller, solver_config)
        if child_certification_attempt:
            stats.child_certification_calls_exhausted += 1
            _observe_child_certification_yield(
                node,
                0,
                solver_config,
                stats,
                no_route=True,
            )
            if solver_config.enable_resumable_child_certification:
                exhausted_tasks = priced.diagnostics.source_neighbor_task_count or len(priced.diagnostics.source_neighbor_block_sizes)
                node.child_certification_exhausted_task_count = exhausted_tasks
                node.child_certification_unresolved_task_count = 0
                stats.child_certification_exhausted_tasks += exhausted_tasks
                stats.child_certification_epochs_completed += 1
        sr_start = time.time()
        violated_activities = rmp.violated_sr_cut_activities(result.z_values, solver_config.cut_tolerance)
        selected_activities = _select_sr_cut_batch(violated_activities, solver_config.sr_cut_add_batch_size)
        violated = set(selected_activities)
        _record_sr_separation_time(stats, node, time.time() - sr_start)
        if violated:
            added_cuts = _activate_sr_cuts(node, selected_activities, stats)
            stats.sr_cuts_added += added_cuts
            if node.depth == 0:
                stats.sr_cuts_added_root += added_cuts
                stats.sr_active_count_root = len(node.active_sr)
            else:
                stats.sr_cuts_added_postroot += added_cuts
                stats.sr_active_count_postroot = max(stats.sr_active_count_postroot, len(node.active_sr))
            _write_progress(solver_config, stats, node, "sr_cuts_added", {"cuts": sorted(violated)})
            continue
        if _remove_inactive_sr_cuts_after_closure(rmp, result, node, solver_config, stats):
            _write_progress(
                solver_config,
                stats,
                node,
                "sr_cuts_removed_for_reprice",
                {"active_sr": len(node.active_sr), "removed_sr": len(node.removed_sr)},
            )
            continue
        merged_paths = _merge_duplicate_column_paths_for_node(node, routes, graph, stats, signature_cache)
        if merged_paths != node.column_paths:
            node.column_paths = merged_paths
            stats.duplicate_merge_events += 1
            _write_progress(solver_config, stats, node, "duplicate_columns_merged")
            continue
        if _deactivate_inactive_node_columns(node, result, solver_config, stats):
            _write_progress(
                solver_config,
                stats,
                node,
                "inactive_columns_deactivated_for_reprice",
                {"active_columns": len(node.column_paths), "inactive_columns": len(node.inactive_column_paths)},
            )
            continue
        _write_progress(solver_config, stats, node, "node_closed", {"bound": result.objective})
        if node.depth == 0:
            stats.root_closed = True
        else:
            stats.postroot_nodes_closed += 1
        return result.objective, result.z_values


def _run_phase_i_seeding(
    graph,
    objective: ObjectiveData,
    solver_config: SolverConfig,
    node: NodeState,
    routes: dict[tuple[str, ...], Route],
    global_pool_paths: set[tuple[str, ...]],
    signature_cache: RouteSignatureCache,
    stats: BPCStats,
    deadline: float,
    pricing_pool: SourceNeighborPricingPool | None,
) -> int:
    added_count = 0
    for _ in range(solver_config.phase_i_max_rounds):
        phase_i_start = time.time()
        phase_i = PhaseISeeder(graph, node, routes, solver_config)
        result = phase_i.solve()
        stats.phase_i_time += time.time() - phase_i_start
        stats.phase_i_solves += 1
        stats.phase_i_uncovered_customers += len(result.uncovered)
        if result.objective <= solver_config.integrality_tolerance or not result.uncovered:
            return added_count
        seed_duals = PricingDuals(
            mu={customer: solver_config.seed_reward if customer in result.uncovered else 0.0 for customer in node.residual_customers},
            kappa=0.0,
            nu={},
        )
        seed_restrictions = node.restrictions
        for path in node.column_paths:
            seed_restrictions = seed_restrictions.with_route_forbidden(path)
        try:
            priced = price_route(
                graph,
                objective,
                node.residual_customers,
                seed_restrictions,
                seed_duals,
                len(routes),
                farkas=False,
                pricing_tolerance=solver_config.pricing_tolerance,
                use_standard_acceleration=False,
                stop_at_first_negative=solver_config.seed_batch_size == 1,
                batch_size=solver_config.seed_batch_size,
                deadline=deadline,
                enable_bidirectional=solver_config.enable_bidirectional_pricing,
                parallel_workers=solver_config.pricing_parallel_workers,
                pricing_worker_backend=solver_config.pricing_worker_backend,
                existing_routes=routes,
                existing_column_paths=set(node.column_paths),
                small_join_pair_threshold=solver_config.small_join_pair_threshold,
                small_join_cumulative_threshold=solver_config.small_join_cumulative_threshold,
                max_join_bypass_calls=solver_config.max_join_bypass_calls,
                small_dom_bucket_threshold=solver_config.small_dom_bucket_threshold,
                small_dom_cumulative_threshold=solver_config.small_dom_cumulative_threshold,
                max_dom_bypass_calls=solver_config.max_dom_bypass_calls,
                join_payload_bin_width=solver_config.join_payload_bin_width,
                join_eval_budget=solver_config.join_eval_budget,
                pricing_certification_slice_seconds=solver_config.pricing_certification_slice_seconds,
                enable_join_lower_envelope=solver_config.enable_join_lower_envelope,
                join_generator_split_threshold=solver_config.join_generator_split_threshold,
                join_generator_pair_batch_size=solver_config.join_generator_pair_batch_size,
                enable_bucket_join_envelope=solver_config.enable_bucket_join_envelope,
                    enable_join_profile_cache=solver_config.enable_join_profile_cache,
                    side_pool_batch_size=0,
                    pricing_process_pool=pricing_pool,
                    productive_candidate_multiplier=solver_config.productive_candidate_multiplier,
                    source_neighbor_task_size=solver_config.source_neighbor_task_size,
                    pricing_diversity_batch_fraction=solver_config.pricing_diversity_batch_fraction,
                    enable_mask_trie_frontier=solver_config.enable_mask_trie_frontier,
                    max_frontier_cell_size=solver_config.max_frontier_cell_size,
                    max_frontier_pair_product=solver_config.max_frontier_pair_product,
                    max_frontier_split_depth=solver_config.max_frontier_split_depth,
                    enable_resource_restricted_closure_bound=solver_config.enable_resource_restricted_closure_bound,
                    resource_bound_method=solver_config.resource_bound_method,
                    **_pricing_scheduler_kwargs(solver_config),
                )
        except PricingTimeLimitReached as exc:
            _record_pricing_timeout(stats, "seed_interrupted", exc, solver_config, node, routes, global_pool_paths, signature_cache)
            raise _NodeTimeLimitReached()
        _record_pricing_diagnostic(stats, "seed", priced, solver_config)
        if not priced.routes:
            return added_count
        added_paths = []
        for route, reduced_cost in zip(priced.routes, priced.reduced_costs):
            if reduced_cost >= -solver_config.pricing_tolerance:
                raise RuntimeError("Phase-I seed pricing returned a nonnegative column in the entering batch")
            path, added = _insert_node_column(route, routes, node, graph, stats, signature_cache, objective)
            global_pool_paths.add(path)
            if not added:
                stats.duplicate_columns_rejected += 1
                return added_count
            stats.phase_i_columns_added += 1
            added_count += 1
            added_paths.append(path)
        _sync_route_stats(stats, routes, global_pool_paths, signature_cache)
        _write_progress(solver_config, stats, node, "phase_i_seed_columns_added", {"paths": added_paths})
    return added_count


def _run_preclosure_heuristic(
    graph,
    objective: ObjectiveData,
    solver_config: SolverConfig,
    node: NodeState,
    routes: dict[tuple[str, ...], Route],
    global_pool_paths: set[tuple[str, ...]],
    side_pool_paths: set[tuple[str, ...]],
    signature_cache: RouteSignatureCache,
    z_values: dict[tuple[str, ...], float],
    stats: BPCStats,
    incumbent: _IncumbentState,
    deadline: float,
    solver_start: float,
    reason: str,
    allow_repair: bool = True,
    repair_time_budget: float | None = None,
    pricing_pool: SourceNeighborPricingPool | None = None,
) -> bool:
    if time.time() >= deadline:
        raise _NodeTimeLimitReached()
    first_incumbent_attempt = (
        incumbent.value == float("inf")
        and solver_config.first_incumbent_route_pool_time_limit > 0.0
        and reason.startswith("first_incumbent")
    )
    heuristic_start = time.time()
    try:
        heuristic = run_route_pool_heuristic(
            graph,
            objective,
            node,
            routes,
            global_pool_paths,
            z_values,
            solver_config,
            len(routes),
            incumbent.value,
            deadline,
            side_pool_paths=side_pool_paths,
            allow_repair=allow_repair,
            repair_time_budget=repair_time_budget,
            pricing_process_pool=pricing_pool,
        )
    except PricingTimeLimitReached as exc:
        heuristic_elapsed = time.time() - heuristic_start
        _record_heuristic_time(stats, node, heuristic_elapsed)
        stats.heuristic_calls += 1
        stats.preclosure_heuristic_calls += 1
        if node.depth > 0:
            stats.postroot_heuristic_calls += 1
        if first_incumbent_attempt:
            stats.first_incumbent_route_pool_calls += 1
            stats.first_incumbent_route_pool_time += heuristic_elapsed
        _record_pricing_timeout(stats, "heuristic_interrupted", exc, solver_config, node, routes, global_pool_paths, signature_cache)
        raise _NodeTimeLimitReached()
    heuristic_elapsed = time.time() - heuristic_start
    _record_heuristic_time(stats, node, heuristic_elapsed)
    stats.heuristic_calls += 1
    stats.preclosure_heuristic_calls += 1
    if node.depth > 0:
        stats.postroot_heuristic_calls += 1
    if first_incumbent_attempt:
        stats.first_incumbent_route_pool_calls += 1
        stats.first_incumbent_route_pool_time += heuristic_elapsed
    stats.heuristic_columns_generated += len(heuristic.generated_paths)
    stats.preclosure_heuristic_columns_generated += len(heuristic.generated_paths)
    _record_heuristic_diagnostics(stats, heuristic.diagnostics)
    if node.depth > 0:
        _record_postroot_heuristic_diagnostics(stats, heuristic.diagnostics)
    if first_incumbent_attempt:
        stats.first_incumbent_route_pool_feasible_solves += (
            heuristic.diagnostics.hard_pool_feasible_solves
            + heuristic.diagnostics.soft_pool_feasible_solves
        )
        stats.first_incumbent_route_pool_routes_max = max(
            stats.first_incumbent_route_pool_routes_max,
            heuristic.diagnostics.node_pool_routes,
        )
    for diagnostic in heuristic.pricing_diagnostics:
        _record_pricing_diagnostic_dict(stats, diagnostic, solver_config)
    global_pool_paths.update(heuristic.generated_paths)
    _sync_route_stats(stats, routes, global_pool_paths, signature_cache)
    improved = False
    if heuristic.value is not None and heuristic.value < incumbent.value:
        incumbent.value = heuristic.value
        incumbent.routes = node.fixed_routes + heuristic.selected_routes
        stats.preclosure_heuristic_incumbent_updates += 1
        stats.heuristic_incumbent_updates += 1
        if node.depth > 0:
            stats.postroot_heuristic_incumbent_updates += 1
        improved = True
        if stats.time_to_first_incumbent is None:
            stats.time_to_first_incumbent = time.time() - solver_start
            stats.incumbent_source = reason
    _write_progress(
        solver_config,
        stats,
        node,
        "preclosure_heuristic",
        {
            "reason": reason,
            "value": heuristic.value,
            "generated_paths": [list(path) for path in sorted(heuristic.generated_paths)],
        },
    )
    return improved


def _activate_sr_cuts(
    node: NodeState,
    violated_activities: dict[tuple[str, str, str], float],
    stats: BPCStats,
) -> int:
    new_count = 0
    for triplet, activity in violated_activities.items():
        meta = node.sr_cut_meta.setdefault(triplet, SRCutMetadata())
        meta.last_activity = activity
        meta.last_violation = max(activity - 1.0, 0.0)
        meta.inactive_count = 0
        if triplet in node.removed_sr:
            node.removed_sr.remove(triplet)
            meta.reactivation_count += 1
            stats.sr_cuts_reactivated += 1
        elif triplet not in node.active_sr:
            new_count += 1
        node.active_sr.add(triplet)
    if violated_activities:
        node.active_sr_version += 1
    return new_count


def _select_sr_cut_batch(
    violated_activities: dict[tuple[str, str, str], float],
    batch_size: int,
) -> dict[tuple[str, str, str], float]:
    ordered = sorted(
        violated_activities.items(),
        key=lambda item: (-item[1], item[0]),
    )
    return dict(ordered[:batch_size])


def _remove_inactive_sr_cuts_after_closure(
    rmp: RestrictedMaster,
    result,
    node: NodeState,
    solver_config: SolverConfig,
    stats: BPCStats,
) -> bool:
    _update_active_sr_cut_metadata(rmp, result.z_values, node, solver_config.cut_tolerance, stats, result.duals)
    if (
        not solver_config.enable_sr_aging
        or not solver_config.enable_postroot_sr_cut_removal
        or not solver_config.sr_reactivation_allowed
        or node.depth == 0
        or not node.active_sr
        or result.objective is None
        or node.sr_removals_performed >= solver_config.sr_max_removals_per_node
    ):
        return False
    if len(node.active_sr) < solver_config.sr_removal_min_active_count:
        stats.sr_removal_trigger_active_count_failures += 1
        return False
    build_burden = (
        node.rmp_build_time_growth >= solver_config.sr_removal_rmp_growth_threshold
        or node.previous_rmp_build_time >= solver_config.sr_removal_build_time_threshold
    )
    solve_burden = node.rmp_solve_time_growth >= solver_config.sr_removal_rmp_growth_threshold
    coeff_burden = stats.sr_coeff_build_time >= solver_config.sr_removal_active_coeff_threshold
    if not solve_burden:
        stats.sr_removal_trigger_growth_failures += 1
    if not build_burden:
        stats.sr_removal_trigger_build_burden_failures += 1
    if not coeff_burden:
        stats.sr_removal_trigger_active_coeff_failures += 1
    if not (solve_burden or build_burden or coeff_burden):
        return False
    inactive_candidates = [
        triplet
        for triplet in sorted(node.active_sr)
        if node.sr_cut_meta[triplet].inactive_count >= solver_config.sr_inactive_age_threshold
        and node.sr_cut_meta[triplet].removal_count < solver_config.sr_max_removals_per_node
    ]
    stats.sr_removal_candidates += len(inactive_candidates)
    for triplet in inactive_candidates:
        node.sr_cut_meta[triplet].removal_candidate_count += 1
    stats.sr_removal_candidate_marks += len(inactive_candidates)
    if not inactive_candidates:
        return False
    scored_removable = []
    for triplet in inactive_candidates:
        meta = node.sr_cut_meta[triplet]
        recent_dual = result.duals.nu.get(triplet, 0.0)
        row_nnz = sum(1 for path in rmp.z if rmp._sr_coeff(path, triplet))
        row_density = row_nnz / max(len(rmp.z), 1)
        recently_violated = meta.last_violation > solver_config.cut_tolerance
        dual_active = abs(recent_dual) > solver_config.sr_activity_tolerance
        if (
            meta.last_activity <= solver_config.sr_activity_tolerance
            and not recently_violated
            and not dual_active
        ):
            score = (
                meta.inactive_count
                + row_density
                + row_nnz / max(len(rmp.z), 1)
                + node.rmp_build_time_growth
                + node.rmp_solve_time_growth
                + len(node.active_sr) / max(solver_config.sr_removal_min_active_count, 1)
            )
            stats.sr_removal_score_max = max(stats.sr_removal_score_max, score)
            scored_removable.append((-score, triplet))
    removable = [triplet for _, triplet in sorted(scored_removable)]
    if not removable:
        stats.sr_removal_trigger_activity_failures += 1
        return False
    remaining = solver_config.sr_max_removals_per_node - node.sr_removals_performed
    selected = removable[: min(solver_config.sr_removal_batch_size, solver_config.postroot_sr_cut_removal_batch_size, remaining)]
    if not selected:
        return False
    for triplet in selected:
        meta = node.sr_cut_meta[triplet]
        meta.removal_count += 1
        node.active_sr.remove(triplet)
        node.removed_sr.add(triplet)
    node.sr_removals_performed += len(selected)
    node.active_sr_version += 1
    node.pending_sr_removal_bound = result.objective
    stats.sr_cuts_removed += len(selected)
    stats.sr_removal_nodes += 1
    stats.sr_cut_repricing_after_removal += 1
    stats.sr_active_count_postroot = max(stats.sr_active_count_postroot, len(node.active_sr))
    return True


def _update_node_column_ages(
    node: NodeState,
    z_values: dict[tuple[str, ...], float],
    solver_config: SolverConfig,
    stats: BPCStats,
) -> None:
    if not solver_config.enable_node_column_aging:
        return
    for path in sorted(node.column_paths):
        value = z_values.get(path, 0.0)
        if abs(value) <= solver_config.column_active_value_tol:
            node.column_age[path] = node.column_age.get(path, 0) + 1
        else:
            node.column_age[path] = 0
    stats.inactive_columns_at_stop = len(node.inactive_column_paths)


def _rehydrate_negative_inactive_columns(
    graph,
    objective: ObjectiveData,
    node: NodeState,
    routes: dict[tuple[str, ...], Route],
    duals: PricingDuals,
    solver_config: SolverConfig,
    stats: BPCStats,
    signature_cache: RouteSignatureCache,
) -> bool:
    if not solver_config.enable_node_column_aging or not node.inactive_column_paths:
        return False
    start = time.time()
    arc_customer_sets = {arc: graph.arc_customer_set(arc) for arc in graph.arcs}
    rehydrated: list[tuple[str, ...]] = []
    for path in sorted(node.inactive_column_paths):
        route = routes[path]
        rejection = _child_inheritance_rejection_reason(
            graph,
            objective,
            node,
            route,
            arc_customer_sets,
            signature_cache,
        )
        if rejection is not None:
            continue
        stats.inactive_column_reduced_cost_checks += 1
        if route_reduced_cost(route, duals) < -solver_config.pricing_tolerance:
            rehydrated.append(path)
    stats.inactive_column_reduced_cost_time += time.time() - start
    if not rehydrated:
        stats.inactive_columns_at_stop = len(node.inactive_column_paths)
        return False
    for path in rehydrated:
        node.inactive_column_paths.remove(path)
        node.column_paths.add(path)
        node.column_age[path] = 0
    stats.inactive_columns_rehydrated += len(rehydrated)
    stats.inactive_column_rehydration_events += 1
    stats.inactive_columns_at_stop = len(node.inactive_column_paths)
    _ensure_node_column_index(graph, node, routes, stats, signature_cache)
    return True


def _deactivate_inactive_node_columns(
    node: NodeState,
    result,
    solver_config: SolverConfig,
    stats: BPCStats,
) -> bool:
    if (
        not solver_config.enable_node_column_aging
        or node.depth == 0
        or result.objective is None
        or len(node.column_paths) < solver_config.column_deactivation_min_active_columns
    ):
        return False
    candidates = [
        path
        for path in sorted(node.column_paths)
        if node.column_age.get(path, 0) >= solver_config.column_inactive_age_min
        and abs(result.z_values.get(path, 0.0)) <= solver_config.column_active_value_tol
    ]
    if not candidates:
        stats.inactive_columns_at_stop = len(node.inactive_column_paths)
        return False
    selected = candidates[: solver_config.column_deactivation_batch_size]
    for path in selected:
        node.column_paths.remove(path)
        node.inactive_column_paths.add(path)
    node.pending_column_deactivation_bound = result.objective
    stats.inactive_columns_deactivated += len(selected)
    stats.inactive_column_deactivation_events += 1
    stats.inactive_columns_at_stop = len(node.inactive_column_paths)
    return True


def _update_active_sr_cut_metadata(
    rmp: RestrictedMaster,
    z_values: dict[tuple[str, ...], float],
    node: NodeState,
    cut_tolerance: float,
    stats: BPCStats,
    duals: PricingDuals,
) -> None:
    for triplet in sorted(node.active_sr):
        row_start = time.time()
        activity = rmp.sr_cut_activity(z_values, triplet)
        meta = node.sr_cut_meta.setdefault(triplet, SRCutMetadata())
        meta.age += 1
        meta.last_activity = activity
        row_nnz = sum(1 for path in rmp.z if rmp._sr_coeff(path, triplet))
        row_density = row_nnz / max(len(rmp.z), 1)
        meta.nonzero_count = row_nnz
        meta.coefficient_density = row_density
        stats.sr_cut_coefficient_nonzeros_observed += row_nnz
        stats.sr_cut_coefficient_density_max = max(stats.sr_cut_coefficient_density_max, row_density)
        if abs(duals.nu.get(triplet, 0.0)) > cut_tolerance:
            meta.last_positive_dual_iteration = meta.age
            stats.sr_cut_dual_activity_updates += 1
        if activity >= 1.0 - cut_tolerance:
            meta.inactive_count = 0
            meta.last_violation = max(activity - 1.0, 0.0)
            meta.activity_count += 1
        else:
            meta.inactive_count += 1
        stats.sr_cuts_aged += 1
        stats.sr_cut_activity_updates += 1
        elapsed = time.time() - row_start
        meta.update_time_contribution += elapsed
        stats.sr_cut_metadata_update_time += elapsed


def _remaining_root_repair_budget(solver_config: SolverConfig, stats: BPCStats) -> float:
    budget = min(
        solver_config.repair_time_hard_cap_root,
        solver_config.repair_time_fraction_of_pricing * stats.standard_pricing_time,
    )
    return max(budget - stats.repair_pricing_time, 0.0)


def _sync_closure_aware_pricing_stats(
    stats: BPCStats,
    closure_state: _ClosureAwarePricingState,
    slice_controller: _AdaptiveProductiveSliceController,
    solver_config: SolverConfig,
) -> None:
    stats.productive_batches_since_last_cert = closure_state.batches_since_cert
    stats.productive_time_since_last_cert = closure_state.time_since_cert
    stats.adaptive_slice_seconds_min = slice_controller.observed_min
    stats.adaptive_slice_seconds_max = slice_controller.observed_max
    stats.adaptive_slice_seconds_last = slice_controller.current_seconds
    stats.adaptive_slice_increases = slice_controller.increases
    stats.adaptive_slice_decreases = slice_controller.decreases
    stats.productive_yield_window_rate = slice_controller.last_yield_rate
    stats.pricing_yield_ratio = slice_controller.last_yield_rate
    stats.prefix_task_depth = max(
        stats.prefix_task_depth,
        solver_config.prefix_task_depth,
        solver_config.prefix_task_depth_root,
    )
    stats.stabilized_dual_enabled = solver_config.use_dual_stabilized_productive_search


def _pricing_balance_config(solver_config: SolverConfig) -> _KCoreBalanceConfig:
    return _KCoreBalanceConfig(
        enabled=solver_config.enable_balanced_kcore_pricing,
        alpha_reachable_customers=solver_config.kcore_balance_alpha_reachable_customers,
        alpha_out_degree=solver_config.kcore_balance_alpha_out_degree,
        alpha_drone_pads=solver_config.kcore_balance_alpha_drone_pads,
        alpha_deadline_customers=solver_config.kcore_balance_alpha_deadline_customers,
    )


def _pricing_refinement_config(solver_config: SolverConfig) -> _DynamicRefinementConfig:
    return _DynamicRefinementConfig(
        enabled=solver_config.enable_dynamic_kcore_refinement,
        split_label_threshold=solver_config.dynamic_split_label_threshold,
        split_gap_multiplier=solver_config.dynamic_split_gap_multiplier,
        split_time_threshold=solver_config.dynamic_split_time_threshold,
        split_work_threshold=solver_config.dynamic_split_work_threshold,
        refinement_depth=solver_config.dynamic_refinement_depth,
        checkpoint_extension_period=solver_config.checkpoint_extension_period,
    )


def _pricing_scheduler_kwargs(solver_config: SolverConfig) -> dict[str, object]:
    return {
        "enable_balanced_kcore_pricing": solver_config.enable_balanced_kcore_pricing,
        "enable_dynamic_kcore_refinement": solver_config.enable_dynamic_kcore_refinement,
        "kcore_balance_alpha_reachable_customers": solver_config.kcore_balance_alpha_reachable_customers,
        "kcore_balance_alpha_out_degree": solver_config.kcore_balance_alpha_out_degree,
        "kcore_balance_alpha_drone_pads": solver_config.kcore_balance_alpha_drone_pads,
        "kcore_balance_alpha_deadline_customers": solver_config.kcore_balance_alpha_deadline_customers,
        "dynamic_split_label_threshold": solver_config.dynamic_split_label_threshold,
        "dynamic_split_gap_multiplier": solver_config.dynamic_split_gap_multiplier,
        "dynamic_split_time_threshold": solver_config.dynamic_split_time_threshold,
        "dynamic_split_work_threshold": solver_config.dynamic_split_work_threshold,
        "dynamic_refinement_depth": solver_config.dynamic_refinement_depth,
        "checkpoint_extension_period": solver_config.checkpoint_extension_period,
    }


def _prefix_task_depth_for_node(solver_config: SolverConfig, node: NodeState, stats: BPCStats) -> int:
    if node.depth == 0:
        return max(solver_config.prefix_task_depth, solver_config.prefix_task_depth_root)
    if stats.branching_nodes >= solver_config.prefix_task_min_branching_for_depth2:
        return max(solver_config.prefix_task_depth, solver_config.prefix_task_depth_child)
    return max(solver_config.prefix_task_depth, solver_config.prefix_task_depth_root)


def _root_primal_budgets(
    solver_config: SolverConfig,
    stats: BPCStats,
    incumbent: _IncumbentState,
    first_incumbent_attempt: bool,
) -> tuple[SolverConfig | None, float]:
    repair_budget = _remaining_root_repair_budget(solver_config, stats)
    if first_incumbent_attempt:
        stats.post_incumbent_heuristic_budget = 0.0
        stats.post_incumbent_repair_budget = 0.0
        return (
            replace(
                solver_config,
                route_pool_time_limit=solver_config.first_incumbent_route_pool_time_limit,
            ),
            repair_budget,
        )
    if incumbent.value < float("inf"):
        heuristic_budget = solver_config.route_pool_time_limit * solver_config.post_incumbent_primal_budget_factor
        repair_budget *= solver_config.post_incumbent_primal_budget_factor
        stats.post_incumbent_heuristic_budget = heuristic_budget
        stats.post_incumbent_repair_budget = repair_budget
        if heuristic_budget <= 0.0 and repair_budget <= 0.0:
            return None, 0.0
        return replace(solver_config, route_pool_time_limit=heuristic_budget), repair_budget
    stats.post_incumbent_heuristic_budget = solver_config.route_pool_time_limit
    stats.post_incumbent_repair_budget = repair_budget
    return solver_config, repair_budget


def _record_pricing_diagnostic(
    stats: BPCStats,
    mode: str,
    priced: PricingResult,
    solver_config: SolverConfig,
) -> None:
    _record_pricing_diagnostic_dict(stats, {"mode": mode, **asdict(priced.diagnostics)}, solver_config)


def _record_pricing_timeout(
    stats: BPCStats,
    mode: str,
    exc: PricingTimeLimitReached,
    solver_config: SolverConfig,
    node: NodeState,
    routes: dict[tuple[str, ...], Route],
    global_pool_paths: set[tuple[str, ...]],
    signature_cache: RouteSignatureCache | None = None,
) -> None:
    stats.status = "time_limit"
    _record_pricing_diagnostic_dict(stats, {"mode": mode, **asdict(exc.diagnostics)}, solver_config)
    _sync_route_stats(stats, routes, global_pool_paths, signature_cache)
    _write_progress(solver_config, stats, node, "time_limit_pricing_unresolved", {"mode": mode})


def _record_productive_slice_timeout(
    stats: BPCStats,
    exc: PricingTimeLimitReached,
    solver_config: SolverConfig,
    node: NodeState,
    routes: dict[tuple[str, ...], Route],
    global_pool_paths: set[tuple[str, ...]],
    signature_cache: RouteSignatureCache | None = None,
    productive_slice_seconds: float | None = None,
) -> None:
    slice_seconds = (
        solver_config.productive_pricing_slice_seconds
        if productive_slice_seconds is None
        else productive_slice_seconds
    )
    diagnostic = {
        "mode": "standard_productive_slice_interrupted",
        **asdict(exc.diagnostics),
        "productive_slice_seconds": slice_seconds,
        "adaptive_slice_seconds": slice_seconds,
        "productive_slice_deadline_used": True,
        "productive_time_limit_no_columns": 1,
    }
    _record_pricing_diagnostic_dict(stats, diagnostic, solver_config)
    _sync_route_stats(stats, routes, global_pool_paths, signature_cache)
    _write_progress(
        solver_config,
        stats,
        node,
        "productive_slice_pricing_unresolved",
        {"mode": "standard", "pricing_mode": "productive"},
    )


def _record_pricing_diagnostic_dict(
    stats: BPCStats,
    diagnostic: dict,
    solver_config: SolverConfig,
) -> None:
    record = {"call_index": len(stats.pricing_diagnostics) + 1, **diagnostic}
    stats.pricing_diagnostics.append(record)
    stats.pricing_labels_generated += int(record["labels_generated"])
    stats.pricing_labels_dominated += int(record["labels_dominated"])
    stats.pricing_labels_purged += int(record.get("labels_purged", 0) or 0)
    stats.pricing_stale_labels_skipped += int(record.get("stale_labels_skipped", 0) or 0)
    stats.pricing_labels_pruned += int(record["labels_pruned"])
    stats.pricing_standard_bound_pruned += int(record.get("standard_bound_pruned", 0) or 0)
    stats.pricing_farkas_bound_pruned += int(record.get("farkas_bound_pruned", 0) or 0)
    stats.pricing_complete_routes_generated += int(record["complete_routes_generated"])
    stats.pricing_extensions_attempted += int(record.get("extensions_attempted", 0) or 0)
    stats.pricing_extensions_rejected_by_deadline += int(record.get("extensions_rejected_by_deadline", 0) or 0)
    stats.pricing_deadline_reachability_removed += int(record.get("deadline_reachability_removed", 0) or 0)
    stats.pricing_reward_set_size_before_deadline += int(record.get("reward_set_size_before_deadline", 0) or 0)
    stats.pricing_reward_set_size_after_deadline += int(record.get("reward_set_size_after_deadline", 0) or 0)
    stats.pricing_deadline_reward_bound_calls += int(record.get("deadline_reward_bound_calls", 0) or 0)
    stats.pricing_deadline_dominance_prefilter_skips += int(record.get("deadline_dominance_prefilter_skips", 0) or 0)
    stats.pricing_routes_rejected_by_deadline_in_master += int(record.get("routes_rejected_by_deadline_in_master", 0) or 0)
    stats.pricing_forward_dominance_tests += int(record.get("forward_dominance_tests", 0) or 0)
    stats.pricing_forward_same_node_dominance_tests += int(record.get("forward_same_node_dominance_tests", 0) or 0)
    stats.pricing_forward_physical_location_dominance_tests += int(
        record.get("forward_physical_location_dominance_tests", 0) or 0
    )
    stats.pricing_forward_physical_location_dominance_rejections += int(
        record.get("forward_physical_location_dominance_rejections", 0) or 0
    )
    stats.pricing_forward_return_time_credit_checks += int(record.get("forward_return_time_credit_checks", 0) or 0)
    stats.pricing_forward_return_time_credit_checks_skipped += int(
        record.get("forward_return_time_credit_checks_skipped", 0) or 0
    )
    stats.pricing_forward_branch_language_failures += int(record.get("forward_branch_language_failures", 0) or 0)
    stats.pricing_forward_mask_scalar_prefilter_failures += int(
        record.get("forward_mask_scalar_prefilter_failures", 0) or 0
    )
    stats.pricing_dom_gate_pairs_seen += int(record.get("dom_gate_pairs_seen", 0) or 0)
    stats.pricing_dom_gate_mask_failures += int(record.get("dom_gate_mask_failures", 0) or 0)
    stats.pricing_dom_gate_scalar_failures += int(record.get("dom_gate_scalar_failures", 0) or 0)
    stats.pricing_dom_gate_branch_failures += int(record.get("dom_gate_branch_failures", 0) or 0)
    stats.pricing_dom_gate_deadline_failures += int(record.get("dom_gate_deadline_failures", 0) or 0)
    stats.pricing_labels_dominated_same_node += int(record.get("labels_dominated_same_node", 0) or 0)
    stats.pricing_labels_dominated_physical += int(record.get("labels_dominated_physical", 0) or 0)
    stats.pricing_dom_prefilter_pairs += int(record.get("dom_prefilter_pairs", 0) or 0)
    stats.pricing_dom_prefilter_mask_fail += int(record.get("dom_prefilter_mask_fail", 0) or 0)
    stats.pricing_dom_prefilter_branch_fail += int(record.get("dom_prefilter_branch_fail", 0) or 0)
    stats.pricing_dom_prefilter_payload_fail += int(record.get("dom_prefilter_payload_fail", 0) or 0)
    stats.pricing_dom_prefilter_block_fail += int(record.get("dom_prefilter_block_fail", 0) or 0)
    stats.pricing_dom_prefilter_return_credit_fail += int(record.get("dom_prefilter_return_credit_fail", 0) or 0)
    stats.pricing_dom_full_tests += int(record.get("dom_full_tests", 0) or 0)
    stats.pricing_dom_full_rejections += int(record.get("dom_full_rejections", 0) or 0)
    stats.pricing_physical_location_full_tests += int(record.get("physical_location_full_tests", 0) or 0)
    stats.pricing_physical_location_rejections += int(record.get("physical_location_rejections", 0) or 0)
    stats.pricing_max_queue_size = max(stats.pricing_max_queue_size, int(record["max_queue_size"]))
    elapsed_seconds = float(record.get("elapsed_seconds", 0.0) or 0.0)
    stats.pricing_diagnostics_elapsed_seconds += elapsed_seconds
    stats.pricing_max_call_elapsed_seconds = max(stats.pricing_max_call_elapsed_seconds, elapsed_seconds)
    stats.pricing_forward_labels_generated += int(record.get("forward_labels_generated", 0) or 0)
    stats.pricing_backward_labels_generated += int(record.get("backward_labels_generated", 0) or 0)
    stats.pricing_backward_dominance_tests += int(record.get("backward_dominance_tests", 0) or 0)
    stats.pricing_backward_labels_dominated += int(record.get("backward_labels_dominated", 0) or 0)
    stats.pricing_backward_cost_function_build_time += float(record.get("backward_cost_function_build_time_seconds", 0.0) or 0.0)
    stats.pricing_backward_cost_function_eval_time += float(record.get("backward_cost_function_eval_time_seconds", 0.0) or 0.0)
    stats.pricing_join_sr_correction_time += float(record.get("join_sr_correction_time_seconds", 0.0) or 0.0)
    stats.pricing_join_active_block_time += float(record.get("join_active_block_time_seconds", 0.0) or 0.0)
    stats.pricing_joined_reduced_cost_evaluations += int(record.get("joined_reduced_cost_evaluations", 0) or 0)
    stats.pricing_join_pairs_key_compatible += int(record.get("join_pairs_key_compatible", 0) or 0)
    stats.pricing_join_pairs_after_bitset_filters += int(record.get("join_pairs_after_bitset_filters", 0) or 0)
    stats.pricing_join_lower_envelope_rejects += int(record.get("join_lower_envelope_rejects", 0) or 0)
    stats.pricing_join_bucket_lower_envelope_rejects += int(record.get("join_bucket_lower_envelope_rejects", 0) or 0)
    stats.pricing_join_subbucket_lower_envelope_rejects += int(record.get("join_subbucket_lower_envelope_rejects", 0) or 0)
    stats.pricing_join_pair_lower_envelope_rejects += int(record.get("join_pair_lower_envelope_rejects", 0) or 0)
    stats.pricing_join_queue_pushes += int(record.get("join_queue_pushes", 0) or 0)
    stats.pricing_join_queue_pops += int(record.get("join_queue_pops", 0) or 0)
    stats.pricing_join_generator_queue_pushes += int(record.get("join_generator_queue_pushes", 0) or 0)
    stats.pricing_join_generator_queue_pops += int(record.get("join_generator_queue_pops", 0) or 0)
    stats.pricing_join_generator_splits += int(record.get("join_generator_splits", 0) or 0)
    stats.pricing_join_materialized_pairs += int(record.get("join_materialized_pairs", 0) or 0)
    stats.pricing_join_exact_rc_evals += int(record.get("join_exact_rc_evals", 0) or 0)
    stats.pricing_join_exact_rc_time += float(record.get("join_exact_rc_time_seconds", 0.0) or 0.0)
    stats.pricing_interface_cache_hits += int(record.get("interface_cache_hits", 0) or 0)
    stats.pricing_interface_cache_misses += int(record.get("interface_cache_misses", 0) or 0)
    stats.pricing_suffix_profile_cache_hits += int(record.get("suffix_profile_cache_hits", 0) or 0)
    stats.pricing_suffix_profile_cache_misses += int(record.get("suffix_profile_cache_misses", 0) or 0)
    stats.pricing_interface_profile_cache_hits += int(record.get("interface_profile_cache_hits", 0) or 0)
    stats.pricing_interface_profile_cache_misses += int(record.get("interface_profile_cache_misses", 0) or 0)
    stats.pricing_negative_routes_verified += int(record.get("negative_routes_verified", 0) or 0)
    stats.pricing_negative_routes_inserted += int(record.get("negative_routes_inserted", 0) or 0)
    stats.pricing_backward_dominance_cost_tests += int(record.get("backward_dominance_cost_tests", 0) or 0)
    stats.pricing_backward_dominance_cost_rejected += int(record.get("backward_dominance_cost_rejected", 0) or 0)
    stats.pricing_backward_exclusive_resource_violations += int(record.get("backward_exclusive_resource_violations", 0) or 0)
    stats.pricing_join_pairs_tested += int(record.get("join_pairs_tested", 0) or 0)
    stats.pricing_joined_routes_accepted += int(record.get("joined_routes_accepted", 0) or 0)
    stats.pricing_forward_labeling_time += float(record.get("forward_labeling_time_seconds", 0.0) or 0.0)
    stats.pricing_backward_labeling_time += float(record.get("backward_labeling_time_seconds", 0.0) or 0.0)
    stats.pricing_join_time += float(record.get("join_time_seconds", 0.0) or 0.0)
    stats.pricing_parallel_calls += int(record.get("parallel_calls", 0) or 0)
    stats.pricing_parallel_workers_max = max(
        stats.pricing_parallel_workers_max,
        int(record.get("parallel_workers", 1) or 1),
    )
    process_cpu_time = float(record.get("process_cpu_time_seconds", 0.0) or 0.0)
    cpu_core_equivalent = float(record.get("cpu_core_equivalent", 0.0) or 0.0)
    stats.pricing_process_cpu_time += process_cpu_time
    stats.pricing_worker_cpu_time += float(record.get("pricing_worker_cpu_time_seconds", 0.0) or 0.0)
    stats.pricing_main_process_cpu_time += float(record.get("pricing_main_process_cpu_time_seconds", 0.0) or 0.0)
    stats.pricing_main_merge_time += float(record.get("pricing_main_merge_time_seconds", 0.0) or 0.0)
    stats.pricing_core_subspace_count_max = max(
        stats.pricing_core_subspace_count_max,
        int(record.get("core_subspace_count", 0) or 0),
    )
    stats.pricing_core_empty_blocks_max = max(
        stats.pricing_core_empty_blocks_max,
        int(record.get("core_empty_blocks", 0) or 0),
    )
    min_core_reduced_cost = record.get("min_core_reduced_cost")
    if min_core_reduced_cost is not None:
        value = float(min_core_reduced_cost)
        stats.pricing_min_core_reduced_cost = (
            value
            if stats.pricing_min_core_reduced_cost is None
            else min(stats.pricing_min_core_reduced_cost, value)
        )
    if record.get("productive_first_hit_core_id") is not None:
        stats.pricing_productive_first_hit_core_id_last = int(record.get("productive_first_hit_core_id"))
    stats.pricing_productive_interrupted_cores += int(record.get("productive_interrupted_cores", 0) or 0)
    stats.pricing_certification_core_closed_count += int(record.get("certification_core_closed_count", 0) or 0)
    stats.pricing_certification_core_unresolved_count += int(record.get("certification_core_unresolved_count", 0) or 0)
    if bool(record.get("root_closed_by_all_cores", False)):
        stats.pricing_root_closed_by_all_cores_calls += 1
    stats.pricing_stale_worker_results_discarded += int(record.get("stale_worker_results_discarded", 0) or 0)
    stats.pricing_number_of_productive_restarts += int(record.get("number_of_productive_restarts", 0) or 0)
    stats.pricing_number_of_certification_calls += int(record.get("number_of_certification_calls", 0) or 0)
    stats.pricing_number_of_certification_failures_due_to_negative_column += int(
        record.get("number_of_certification_failures_due_to_negative_column", 0) or 0
    )
    stats.pricing_number_of_certification_timeouts_unresolved += int(
        record.get("number_of_certification_timeouts_unresolved", 0) or 0
    )
    stats.pricing_pool_startup_time += float(record.get("pricing_pool_startup_time_seconds", 0.0) or 0.0)
    stats.pricing_pool_startup_count += int(record.get("pricing_pool_startup_count", 0) or 0)
    stats.pricing_pool_reused_calls += int(record.get("pricing_pool_reused_calls", 0) or 0)
    stats.pricing_pool_shutdown_time += float(record.get("pricing_pool_shutdown_time_seconds", 0.0) or 0.0)
    stats.pricing_task_submission_time += float(record.get("pricing_task_submission_time_seconds", 0.0) or 0.0)
    stats.pricing_worker_payload_count += int(record.get("pricing_worker_payload_count", 0) or 0)
    stats.pricing_worker_response_count += int(record.get("pricing_worker_response_count", 0) or 0)
    stats.pricing_candidate_paths_before_merge += int(record.get("pricing_candidate_paths_before_merge", 0) or 0)
    stats.pricing_candidate_paths_after_merge += int(record.get("pricing_candidate_paths_after_merge", 0) or 0)
    stats.pricing_decoded_routes_in_main += int(record.get("pricing_decoded_routes_in_main", 0) or 0)
    stats.pricing_verified_routes_in_main += int(record.get("pricing_verified_routes_in_main", 0) or 0)
    stats.pricing_batch_target_max = max(
        stats.pricing_batch_target_max,
        int(record.get("pricing_batch_target", 0) or 0),
    )
    stats.pricing_returned_batch_size_max = max(
        stats.pricing_returned_batch_size_max,
        int(record.get("pricing_returned_batch_size", 0) or 0),
    )
    if bool(record.get("pricing_first_hit_enabled", False)):
        stats.pricing_first_hit_enabled_calls += 1
    stats.pricing_stale_response_rejections += int(record.get("pricing_stale_response_rejections", 0) or 0)
    stats.pricing_cpu_core_equivalent_max = max(stats.pricing_cpu_core_equivalent_max, cpu_core_equivalent)
    if stats.pricing_diagnostics_elapsed_seconds > 0.0:
        stats.pricing_cpu_core_equivalent_weighted = stats.pricing_process_cpu_time / stats.pricing_diagnostics_elapsed_seconds
    backend = record.get("pricing_worker_backend")
    if backend == "thread":
        stats.pricing_worker_backend_thread_calls += 1
    elif backend == "process":
        stats.pricing_worker_backend_process_calls += 1
    stats.pricing_source_neighbor_count_max = max(
        stats.pricing_source_neighbor_count_max,
        int(record.get("source_neighbor_count", 0) or 0),
    )
    stats.pricing_source_neighbor_task_count_max = max(
        stats.pricing_source_neighbor_task_count_max,
        int(record.get("source_neighbor_task_count", 0) or 0),
    )
    task_sizes = tuple(int(value) for value in record.get("source_neighbor_task_sizes", ()) or ())
    stats.pricing_source_neighbor_task_size_max = max(
        stats.pricing_source_neighbor_task_size_max,
        max(task_sizes, default=0),
    )
    stats.pricing_initial_source_neighbors_max = max(
        stats.pricing_initial_source_neighbors_max,
        int(record.get("pricing_initial_source_neighbors", 0) or 0),
    )
    stats.pricing_initial_task_count_max = max(
        stats.pricing_initial_task_count_max,
        int(record.get("pricing_initial_task_count", 0) or 0),
    )
    stats.pricing_initial_load_imbalance_max_mean = max(
        stats.pricing_initial_load_imbalance_max_mean,
        float(record.get("pricing_initial_load_imbalance_max_mean", 0.0) or 0.0),
    )
    stats.pricing_empty_initial_blocks_max = max(
        stats.pricing_empty_initial_blocks_max,
        int(record.get("pricing_empty_initial_blocks", 0) or 0),
    )
    stats.pricing_idle_worker_seconds += float(record.get("pricing_idle_worker_seconds", 0.0) or 0.0)
    stats.pricing_dynamic_split_candidates += int(record.get("pricing_dynamic_split_candidates", 0) or 0)
    stats.pricing_dynamic_splits_performed += int(record.get("pricing_dynamic_splits_performed", 0) or 0)
    stats.pricing_dynamic_split_rejected_close_to_closure += int(
        record.get("pricing_dynamic_split_rejected_close_to_closure", 0) or 0
    )
    stats.pricing_dynamic_split_rejected_small_queue += int(
        record.get("pricing_dynamic_split_rejected_small_queue", 0) or 0
    )
    stats.pricing_dynamic_split_rejected_short_elapsed += int(
        record.get("pricing_dynamic_split_rejected_short_elapsed", 0) or 0
    )
    stats.pricing_dynamic_split_rejected_low_workload += int(
        record.get("pricing_dynamic_split_rejected_low_workload", 0) or 0
    )
    stats.pricing_dynamic_child_tasks_created += int(record.get("pricing_dynamic_child_tasks_created", 0) or 0)
    stats.pricing_dynamic_labels_transferred += int(
        record.get("pricing_labels_transferred_to_idle_workers", 0) or 0
    )
    stats.pricing_dynamic_split_overhead_seconds += float(
        record.get("pricing_split_overhead_time", 0.0) or 0.0
    )
    stats.pricing_leaf_tasks_closed += int(record.get("pricing_leaf_tasks_closed", 0) or 0)
    stats.pricing_leaf_tasks_stale += int(record.get("pricing_leaf_tasks_stale_discarded", 0) or 0)
    stats.pricing_best_active_task_gap_max = max(
        stats.pricing_best_active_task_gap_max,
        float(record.get("pricing_best_active_task_gap", 0.0) or 0.0),
    )
    stats.pricing_open_labels_by_task_max = max(
        stats.pricing_open_labels_by_task_max,
        int(record.get("pricing_open_labels_by_task_max", 0) or 0),
    )
    stats.pricing_epoch_invalidations_dual += int(record.get("epoch_invalidations_due_to_dual_change", 0) or 0)
    stats.pricing_epoch_invalidations_sr += int(record.get("epoch_invalidations_due_to_sr_change", 0) or 0)
    stats.pricing_epoch_invalidations_residual_branch += int(
        (record.get("epoch_invalidations_due_to_branch_change", 0) or 0)
        + (record.get("epoch_invalidations_due_to_residual_change", 0) or 0)
    )
    stats.pricing_epoch_invalidations_active_columns += int(
        record.get("epoch_invalidations_due_to_route_insert", 0) or 0
    )
    stats.pricing_stale_task_reuse_attempts += int(record.get("stale_task_reuse_attempts", 0) or 0)
    stats.pricing_stale_task_reuse_blocked += int(record.get("stale_task_reuse_blocked", 0) or 0)
    stats.pricing_cross_task_dominance_attempts += int(
        record.get("cross_task_dominance_attempts", 0) or 0
    )
    stats.pricing_cross_task_dominance_blocked += int(
        record.get("cross_task_dominance_blocked", 0) or 0
    )
    stats.pricing_local_worker_candidate_quota_max = max(
        stats.pricing_local_worker_candidate_quota_max,
        int(record.get("local_worker_candidate_quota", 0) or 0),
    )
    stats.pricing_diversity_quota_max = max(
        stats.pricing_diversity_quota_max,
        int(record.get("diversity_quota", 0) or 0),
    )
    stats.pricing_diversity_selected_routes += int(record.get("diversity_selected_routes", 0) or 0)
    stats.pricing_diversity_selected_customers_max = max(
        stats.pricing_diversity_selected_customers_max,
        int(record.get("diversity_selected_customers", 0) or 0),
    )
    if bool(record.get("productive_slice_deadline_used", False)):
        stats.pricing_productive_slice_deadline_calls += 1
    stats.pricing_productive_slice_time += float(record.get("productive_slice_seconds", 0.0) or 0.0)
    adaptive_slice = float(record.get("adaptive_slice_seconds", 0.0) or 0.0)
    if adaptive_slice > 0.0:
        if stats.adaptive_slice_seconds_min <= 0.0:
            stats.adaptive_slice_seconds_min = adaptive_slice
        else:
            stats.adaptive_slice_seconds_min = min(stats.adaptive_slice_seconds_min, adaptive_slice)
        stats.adaptive_slice_seconds_max = max(stats.adaptive_slice_seconds_max, adaptive_slice)
        stats.adaptive_slice_seconds_last = adaptive_slice
    if record.get("productive_yield_window_rate") is not None:
        stats.productive_yield_window_rate = float(record.get("productive_yield_window_rate") or 0.0)
        stats.pricing_yield_ratio = stats.productive_yield_window_rate
    stats.prefix_task_depth = max(stats.prefix_task_depth, int(record.get("prefix_task_depth", 1) or 1))
    if bool(record.get("stabilized_dual_enabled", False)):
        stats.stabilized_dual_enabled = True
    stats.stabilized_candidates_returned += int(record.get("stabilized_candidates_returned", 0) or 0)
    stats.true_dual_rejected_candidates += int(record.get("true_dual_rejected_candidates", 0) or 0)
    stats.max_abs_worker_true_rc_discrepancy = max(
        stats.max_abs_worker_true_rc_discrepancy,
        float(record.get("max_abs_worker_true_rc_discrepancy", 0.0) or 0.0),
    )
    stats.pricing_productive_time_limit_with_columns += int(record.get("productive_time_limit_with_columns", 0) or 0)
    stats.pricing_productive_time_limit_no_columns += int(record.get("productive_time_limit_no_columns", 0) or 0)
    stats.pricing_first_hit_exits += int(record.get("first_hit_exits", 0) or 0)
    stats.pricing_interrupted_worker_calls += int(record.get("interrupted_worker_calls", 0) or 0)
    stats.pricing_certification_worker_calls += int(record.get("certification_worker_calls", 0) or 0)
    stats.pricing_productive_worker_calls += int(record.get("productive_worker_calls", 0) or 0)
    stats.pricing_signature_cache_hits += int(record.get("signature_cache_hits", 0) or 0)
    stats.pricing_signature_cache_misses += int(record.get("signature_cache_misses", 0) or 0)
    stats.pricing_core_signature_cache_hits += int(record.get("core_signature_cache_hits", 0) or 0)
    stats.pricing_core_signature_cache_misses += int(record.get("core_signature_cache_misses", 0) or 0)
    stats.pricing_active_signature_cache_hits += int(record.get("active_signature_cache_hits", 0) or 0)
    stats.pricing_active_signature_cache_misses += int(record.get("active_signature_cache_misses", 0) or 0)
    stats.pricing_sr_coeff_cache_hits += int(record.get("sr_coeff_cache_hits", 0) or 0)
    stats.pricing_sr_coeff_cache_misses += int(record.get("sr_coeff_cache_misses", 0) or 0)
    stats.pricing_active_sr_key_cache_hits += int(record.get("active_sr_key_cache_hits", 0) or 0)
    stats.pricing_active_sr_key_cache_misses += int(record.get("active_sr_key_cache_misses", 0) or 0)
    stats.pricing_active_sr_coeffs_computed += int(record.get("active_sr_coeffs_computed", 0) or 0)
    stats.pricing_triplet_masks_built += int(record.get("triplet_masks_built", 0) or 0)
    stats.pricing_dominance_prefilter_pairs += int(record.get("dominance_prefilter_pairs", 0) or 0)
    stats.pricing_dominance_prefilter_rejected += int(record.get("dominance_prefilter_rejected", 0) or 0)
    stats.pricing_dominance_bucket_pairs_considered += int(record.get("dominance_bucket_pairs_considered", 0) or 0)
    stats.pricing_dominance_bucket_pairs_rejected += int(record.get("dominance_bucket_pairs_rejected", 0) or 0)
    stats.pricing_dominance_bucket_candidate_pairs += int(record.get("dominance_bucket_candidate_pairs", 0) or 0)
    stats.pricing_dominance_bucket_queries += int(record.get("dominance_bucket_queries", 0) or 0)
    stats.pricing_dominance_bucket_skipped_by_mask += int(record.get("dominance_bucket_skipped_by_mask", 0) or 0)
    stats.pricing_dominance_bucket_skipped_by_scalar += int(record.get("dominance_bucket_skipped_by_scalar", 0) or 0)
    stats.pricing_dominance_bucket_skipped_by_branch += int(record.get("dominance_bucket_skipped_by_branch", 0) or 0)
    stats.pricing_dominance_bucket_skipped_by_deadline += int(record.get("dominance_bucket_skipped_by_deadline", 0) or 0)
    stats.pricing_dominance_bucket_skipped_by_return_credit += int(
        record.get("dominance_bucket_skipped_by_return_credit", 0) or 0
    )
    stats.pricing_dom_frontier_queries += int(record.get("dom_frontier_queries", 0) or 0)
    stats.pricing_dom_frontier_keys_scanned += int(record.get("dom_frontier_keys_scanned", 0) or 0)
    stats.pricing_dom_frontier_keys_skipped_by_mask += int(record.get("dom_frontier_keys_skipped_by_mask", 0) or 0)
    stats.pricing_dom_frontier_keys_skipped_by_branch += int(record.get("dom_frontier_keys_skipped_by_branch", 0) or 0)
    stats.pricing_dom_frontier_keys_skipped_by_deadline += int(record.get("dom_frontier_keys_skipped_by_deadline", 0) or 0)
    stats.pricing_dom_frontier_keys_skipped_by_return_credit += int(
        record.get("dom_frontier_keys_skipped_by_return_credit", 0) or 0
    )
    stats.pricing_frontier_cells_created = max(
        stats.pricing_frontier_cells_created,
        int(record.get("frontier_cells_created", 0) or 0),
    )
    stats.pricing_frontier_cells_split = max(
        stats.pricing_frontier_cells_split,
        int(record.get("frontier_cells_split", 0) or 0),
    )
    stats.pricing_frontier_cell_lb_closed += int(record.get("frontier_cell_lb_closed", 0) or 0)
    stats.pricing_frontier_cell_lb_invalidations += int(record.get("frontier_cell_lb_invalidations", 0) or 0)
    stats.pricing_mask_trie_subset_queries += int(record.get("mask_trie_subset_queries", 0) or 0)
    stats.pricing_mask_trie_superset_queries += int(record.get("mask_trie_superset_queries", 0) or 0)
    stats.pricing_mask_trie_returned_items += int(record.get("mask_trie_returned_items", 0) or 0)
    stats.pricing_mask_subset_queries += int(record.get("mask_subset_queries", 0) or 0)
    stats.pricing_mask_superset_queries += int(record.get("mask_superset_queries", 0) or 0)
    stats.pricing_mask_query_cache_hits += int(record.get("mask_query_cache_hits", 0) or 0)
    stats.pricing_mask_query_cache_misses += int(record.get("mask_query_cache_misses", 0) or 0)
    stats.pricing_cell_splits += int(record.get("cell_splits", 0) or 0)
    stats.pricing_cell_pair_products_before_split += int(record.get("cell_pair_products_before_split", 0) or 0)
    stats.pricing_cell_pairs_considered += int(record.get("cell_pairs_considered", 0) or 0)
    stats.pricing_cell_pairs_rejected_by_mask += int(record.get("cell_pairs_rejected_by_mask", 0) or 0)
    stats.pricing_cell_pairs_rejected_by_envelope += int(record.get("cell_pairs_rejected_by_envelope", 0) or 0)
    stats.pricing_cell_pairs_rejected_by_lb += int(record.get("cell_pairs_rejected_by_lb", 0) or 0)
    stats.pricing_cell_pairs_rejected_by_closure_lb += int(record.get("cell_pairs_rejected_by_closure_lb", 0) or 0)
    stats.pricing_label_pairs_materialized += int(record.get("label_pairs_materialized", 0) or 0)
    stats.pricing_labels_certified_by_cell_lb += int(record.get("labels_certified_by_cell_lb", 0) or 0)
    stats.pricing_full_same_node_tests += int(record.get("full_same_node_tests", 0) or 0)
    stats.pricing_full_physical_location_tests += int(record.get("full_physical_location_tests", 0) or 0)
    stats.pricing_labels_deleted_same_node += int(record.get("labels_deleted_same_node", 0) or 0)
    stats.pricing_labels_deleted_physical_location += int(record.get("labels_deleted_physical_location", 0) or 0)
    stats.pricing_closure_queue_pushes += int(record.get("closure_queue_pushes", 0) or 0)
    stats.pricing_closure_queue_pops += int(record.get("closure_queue_pops", 0) or 0)
    stats.pricing_certification_tasks_exhausted_by_cell_lb += int(
        record.get("certification_tasks_exhausted_by_cell_lb", 0) or 0
    )
    stats.pricing_certification_tasks_closed_by_cell_lb += int(
        record.get("certification_tasks_closed_by_cell_lb", 0) or 0
    )
    stats.pricing_certification_tasks_exhausted_by_label_search += int(
        record.get("certification_tasks_exhausted_by_label_search", 0) or 0
    )
    stats.pricing_resource_reward_bound_calls += int(record.get("resource_reward_bound_calls", 0) or 0)
    stats.pricing_resource_reward_bound_time += float(record.get("resource_reward_bound_time", 0.0) or 0.0)
    stats.pricing_resource_reward_bound_fallbacks += int(record.get("resource_reward_bound_fallbacks", 0) or 0)
    stats.pricing_physdom_cell_pairs_considered += int(record.get("physdom_cell_pairs_considered", 0) or 0)
    stats.pricing_physdom_cell_pairs_rejected_by_mask += int(
        record.get("physdom_cell_pairs_rejected_by_mask", 0) or 0
    )
    stats.pricing_physdom_cell_pairs_rejected_by_envelope += int(
        record.get("physdom_cell_pairs_rejected_by_envelope", 0) or 0
    )
    stats.pricing_physdom_label_pairs_materialized += int(record.get("physdom_label_pairs_materialized", 0) or 0)
    stats.pricing_physdom_full_tests += int(record.get("physdom_full_tests", 0) or 0)
    stats.pricing_physdom_deletions += int(record.get("physdom_deletions", 0) or 0)
    stats.pricing_physdom_time += float(record.get("physdom_time", 0.0) or 0.0)
    stats.pricing_return_credit_incompatible_pairs += int(record.get("return_credit_incompatible_pairs", 0) or 0)
    stats.pricing_dom_pairs_avoided_before_materialization += int(
        record.get("dom_pairs_avoided_before_materialization", 0) or 0
    )
    stats.pricing_dom_candidate_pairs_materialized += int(record.get("dom_candidate_pairs_materialized", 0) or 0)
    stats.pricing_dom_full_tests_same_node += int(record.get("dom_full_tests_same_node", 0) or 0)
    stats.pricing_dom_full_tests_physical_location += int(record.get("dom_full_tests_physical_location", 0) or 0)
    stats.pricing_dom_labels_deleted_same_node += int(record.get("dom_labels_deleted_same_node", 0) or 0)
    stats.pricing_dom_labels_deleted_physical_location += int(
        record.get("dom_labels_deleted_physical_location", 0) or 0
    )
    stats.pricing_dominance_compatible_keys_generated += int(record.get("dominance_compatible_keys_generated", 0) or 0)
    stats.pricing_dominance_compatible_key_lookups += int(record.get("dominance_compatible_key_lookups", 0) or 0)
    stats.pricing_dominance_bucket_scans_avoided += int(record.get("dominance_bucket_scans_avoided", 0) or 0)
    stats.pricing_dominance_key_generation_time += float(record.get("dominance_key_generation_time_seconds", 0.0) or 0.0)
    stats.pricing_dominance_key_cache_hits += int(record.get("dominance_key_cache_hits", 0) or 0)
    stats.pricing_dominance_key_cache_misses += int(record.get("dominance_key_cache_misses", 0) or 0)
    stats.pricing_dominance_small_bypass_calls += int(record.get("dominance_small_bypass_calls", 0) or 0)
    stats.pricing_dominance_bypass_calls += int(record.get("dominance_bypass_calls", 0) or 0)
    stats.pricing_dominance_indexed_activation_count += int(record.get("dominance_indexed_activation_count", 0) or 0)
    stats.pricing_dominance_work_estimate = max(stats.pricing_dominance_work_estimate, int(record.get("dominance_work_estimate", 0) or 0))
    stats.pricing_sticky_indexed_dominance_activations += int(record.get("sticky_indexed_dominance_activations", 0) or 0)
    stats.pricing_dominance_stage_reject_key += int(record.get("dominance_stage_reject_key", 0) or 0)
    stats.pricing_dominance_stage_reject_branch += int(record.get("dominance_stage_reject_branch", 0) or 0)
    stats.pricing_dominance_stage_reject_customer += int(record.get("dominance_stage_reject_customer", 0) or 0)
    stats.pricing_dominance_stage_reject_truck_node += int(record.get("dominance_stage_reject_truck_node", 0) or 0)
    stats.pricing_dominance_stage_reject_payload += int(record.get("dominance_stage_reject_payload", 0) or 0)
    stats.pricing_dominance_stage_reject_block += int(record.get("dominance_stage_reject_block", 0) or 0)
    stats.pricing_dominance_stage_reject_time += int(record.get("dominance_stage_reject_time", 0) or 0)
    stats.pricing_dominance_stage_reject_cost += int(record.get("dominance_stage_reject_cost", 0) or 0)
    stats.pricing_backward_full_dominance_tests += int(record.get("backward_full_dominance_tests", 0) or 0)
    stats.pricing_join_prefilter_pairs += int(record.get("join_prefilter_pairs", 0) or 0)
    stats.pricing_join_prefilter_rejected += int(record.get("join_prefilter_rejected", 0) or 0)
    stats.pricing_join_bucket_pairs_considered += int(record.get("join_bucket_pairs_considered", 0) or 0)
    stats.pricing_join_bucket_pairs_rejected += int(record.get("join_bucket_pairs_rejected", 0) or 0)
    stats.pricing_join_bucket_candidate_pairs += int(record.get("join_bucket_candidate_pairs", 0) or 0)
    stats.pricing_join_compatible_keys_generated += int(record.get("join_compatible_keys_generated", 0) or 0)
    stats.pricing_join_compatible_key_lookups += int(record.get("join_compatible_key_lookups", 0) or 0)
    stats.pricing_join_bucket_scans_avoided += int(record.get("join_bucket_scans_avoided", 0) or 0)
    stats.pricing_join_key_generation_time += float(record.get("join_key_generation_time_seconds", 0.0) or 0.0)
    stats.pricing_join_key_cache_hits += int(record.get("join_key_cache_hits", 0) or 0)
    stats.pricing_join_key_cache_misses += int(record.get("join_key_cache_misses", 0) or 0)
    stats.pricing_join_graph_build_time += float(record.get("join_graph_build_time_seconds", 0.0) or 0.0)
    stats.pricing_join_subbucket_pairs_considered += int(record.get("join_subbucket_pairs_considered", 0) or 0)
    stats.pricing_join_subbucket_pairs_rejected += int(record.get("join_subbucket_pairs_rejected", 0) or 0)
    stats.pricing_join_small_bypass_calls += int(record.get("join_small_bypass_calls", 0) or 0)
    stats.pricing_join_local_bypass_calls += int(record.get("join_local_bypass_calls", 0) or 0)
    stats.pricing_join_cumulative_bypass_calls += int(record.get("join_cumulative_bypass_calls", 0) or 0)
    stats.pricing_join_indexed_activation_count += int(record.get("join_indexed_activation_count", 0) or 0)
    stats.pricing_join_work_estimate = max(stats.pricing_join_work_estimate, int(record.get("join_work_estimate", 0) or 0))
    stats.pricing_sticky_indexed_join_activations += int(record.get("sticky_indexed_join_activations", 0) or 0)
    stats.pricing_join_stage_reject_key += int(record.get("join_stage_reject_key", 0) or 0)
    stats.pricing_join_stage_reject_branch += int(record.get("join_stage_reject_branch", 0) or 0)
    stats.pricing_join_stage_reject_customer += int(record.get("join_stage_reject_customer", 0) or 0)
    stats.pricing_join_stage_reject_truck_node += int(record.get("join_stage_reject_truck_node", 0) or 0)
    stats.pricing_join_stage_reject_payload += int(record.get("join_stage_reject_payload", 0) or 0)
    stats.pricing_join_stage_reject_block += int(record.get("join_stage_reject_block", 0) or 0)
    stats.pricing_join_stage_reject_reduced_cost += int(record.get("join_stage_reject_reduced_cost", 0) or 0)
    stats.pricing_join_candidate_pairs_accepted += int(record.get("join_candidate_pairs_accepted", 0) or 0)
    stats.pricing_join_label_pairs_materialized += int(record.get("join_label_pairs_materialized", 0) or 0)
    stats.pricing_join_full_decodes += int(record.get("join_full_decodes", 0) or 0)
    stats.pricing_lazy_rejected_before_decode += int(record.get("lazy_rejected_before_decode", 0) or 0)
    stats.pricing_fully_decoded_routes += int(record.get("fully_decoded_routes", 0) or 0)
    stats.pricing_duplicate_equivalent_rejected += int(record.get("duplicate_equivalent_rejected", 0) or 0)
    stats.pricing_cost_dominated_rejected += int(record.get("cost_dominated_rejected", 0) or 0)
    stats.pricing_signature_build_time += float(record.get("signature_build_time_seconds", 0.0) or 0.0)
    stats.pricing_sr_coeff_build_time += float(record.get("sr_coeff_build_time_seconds", 0.0) or 0.0)
    stats.pricing_duplicate_lookup_time += float(record.get("duplicate_lookup_time_seconds", 0.0) or 0.0)
    stats.pricing_route_decode_time += float(record.get("route_decode_time_seconds", 0.0) or 0.0)
    stats.pricing_reduced_cost_verification_time += float(record.get("reduced_cost_verification_time_seconds", 0.0) or 0.0)
    stats.pricing_side_pool_routes_returned += int(record.get("side_pool_routes_returned", 0) or 0)
    stats.pricing_side_pool_candidates_seen += int(record.get("side_pool_candidates_seen", 0) or 0)
    stats.pricing_side_pool_routes_retained += int(record.get("side_pool_routes_retained", 0) or 0)
    stats.pricing_side_pool_routes_rejected_by_budget += int(record.get("side_pool_routes_rejected_by_budget", 0) or 0)
    side_pool_min = record.get("side_pool_reduced_cost_min")
    if side_pool_min is not None:
        value = float(side_pool_min)
        stats.pricing_side_pool_reduced_cost_min = (
            value
            if stats.pricing_side_pool_reduced_cost_min is None
            else min(stats.pricing_side_pool_reduced_cost_min, value)
        )
    pricing_mode = record.get("pricing_mode")
    if record.get("mode") in {"standard", "standard_interrupted", "standard_productive_slice_interrupted"}:
        if pricing_mode == "closure":
            stats.pricing_closure_mode_calls += 1
        elif pricing_mode == "productive":
            stats.pricing_productive_mode_calls += 1
    if record.get("pricing_yield_ratio") is not None:
        stats.pricing_yield_ratio = float(record.get("pricing_yield_ratio") or 0.0)
    best_reduced = record.get("best_reduced_cost")
    if best_reduced is not None:
        stats.best_reduced_cost_at_stop = float(best_reduced)
    mode = record.get("mode")
    if mode in {"seed", "seed_interrupted"}:
        stats.seed_pricing_calls += 1
        stats.seed_pricing_time += elapsed_seconds
    elif mode in {"repair", "repair_interrupted"}:
        stats.repair_pricing_calls += 1
        stats.repair_pricing_time += elapsed_seconds
    _append_pricing_diagnostic(solver_config, record)


def _record_heuristic_diagnostics(stats: BPCStats, diagnostic) -> None:
    stats.heuristic_hard_pool_solves += diagnostic.hard_pool_solves
    stats.heuristic_hard_pool_time += diagnostic.hard_pool_time
    stats.heuristic_hard_pool_feasible_solves += diagnostic.hard_pool_feasible_solves
    stats.heuristic_support_pool_calls += diagnostic.support_pool_calls
    stats.heuristic_support_pool_time += diagnostic.support_pool_time
    stats.heuristic_support_pool_feasible += diagnostic.support_pool_feasible
    stats.heuristic_support_pool_incumbent_updates += diagnostic.support_pool_incumbent_updates
    stats.heuristic_full_pool_calls += diagnostic.full_pool_calls
    stats.heuristic_full_pool_time += diagnostic.full_pool_time
    stats.heuristic_full_pool_feasible += diagnostic.full_pool_feasible
    stats.heuristic_full_pool_incumbent_updates += diagnostic.full_pool_incumbent_updates
    stats.heuristic_soft_pool_solves += diagnostic.soft_pool_solves
    stats.heuristic_soft_pool_time += diagnostic.soft_pool_time
    stats.heuristic_soft_pool_feasible_solves += diagnostic.soft_pool_feasible_solves
    stats.heuristic_repair_customers += diagnostic.repair_customers
    stats.heuristic_repair_columns_generated += diagnostic.repair_columns_generated
    stats.heuristic_max_node_pool_routes = max(
        stats.heuristic_max_node_pool_routes,
        diagnostic.max_node_pool_routes,
        diagnostic.node_pool_routes,
    )
    stats.heuristic_max_support_routes = max(
        stats.heuristic_max_support_routes,
        diagnostic.max_support_routes,
        diagnostic.support_routes,
    )
    stats.heuristic_node_pool_to_support_ratio_max = max(
        stats.heuristic_node_pool_to_support_ratio_max,
        diagnostic.node_pool_to_support_ratio,
    )
    stats.repair_budget_hit += diagnostic.repair_budget_hit
    stats.heuristic_budget_hit += diagnostic.heuristic_budget_hit


def _record_postroot_heuristic_diagnostics(stats: BPCStats, diagnostic) -> None:
    stats.postroot_heuristic_hard_pool_solves += diagnostic.hard_pool_solves
    stats.postroot_heuristic_support_pool_calls += diagnostic.support_pool_calls
    stats.postroot_heuristic_full_pool_calls += diagnostic.full_pool_calls
    stats.postroot_heuristic_soft_pool_solves += diagnostic.soft_pool_solves
    stats.postroot_repair_columns_generated += diagnostic.repair_columns_generated
    stats.postroot_heuristic_budget_hits += diagnostic.heuristic_budget_hit
    if diagnostic.repair_customers or diagnostic.repair_columns_generated:
        stats.postroot_repair_calls += 1


def _record_dual_sign_diagnostics(stats: BPCStats, duals: PricingDuals) -> None:
    positive_sr = [value for value in duals.nu.values() if value > 0.0]
    if positive_sr:
        stats.max_positive_le_dual_violation = max(stats.max_positive_le_dual_violation, max(positive_sr))
        stats.sr_dual_sign_violations += len(positive_sr)
    if duals.kappa > 0.0:
        stats.max_positive_le_dual_violation = max(stats.max_positive_le_dual_violation, duals.kappa)
        stats.fleet_dual_sign_violations += 1


def _record_rmp_compatibility_failures(stats: BPCStats, reasons: tuple[str, ...]) -> None:
    for reason in reasons:
        if reason == "residual_customers":
            stats.rmp_compatibility_failure_residual_customers += 1
        elif reason == "fixed_routes":
            stats.rmp_compatibility_failure_fixed_routes += 1
        elif reason == "fleet_limit":
            stats.rmp_compatibility_failure_fleet_limit += 1
        elif reason == "fixed_cost":
            stats.rmp_compatibility_failure_fixed_cost += 1
        elif reason == "branch_state":
            stats.rmp_compatibility_failure_branch_state += 1
        elif reason == "active_sr":
            stats.rmp_compatibility_failure_active_sr += 1
        elif reason == "active_sr_version":
            stats.rmp_compatibility_failure_active_sr_version += 1
        elif reason == "service_deadline_version":
            stats.rmp_compatibility_failure_service_deadline_version += 1
        elif reason == "objective_scale_version":
            stats.rmp_compatibility_failure_objective_scale_version += 1
        elif reason == "active_column_version":
            stats.rmp_compatibility_failure_active_column_version += 1
        elif reason == "rmp_structure_version":
            stats.rmp_compatibility_failure_rmp_structure_version += 1
        else:
            raise RuntimeError(f"unknown RMP compatibility failure reason {reason}")


def _ensure_child_closure_batch_state(
    node: NodeState,
    solver_config: SolverConfig,
    stats: BPCStats,
) -> None:
    if node.child_closure_batch_limit == 0:
        node.child_closure_batch_limit = solver_config.child_closure_batch_initial
    stats.child_closure_batch_last = node.child_closure_batch_limit
    if stats.child_closure_batch_min == 0:
        stats.child_closure_batch_min = solver_config.child_closure_batch_min
    stats.child_closure_batch_max = max(stats.child_closure_batch_max, node.child_closure_batch_limit)


def _observe_child_certification_yield(
    node: NodeState,
    useful_columns: int,
    solver_config: SolverConfig,
    stats: BPCStats,
    *,
    no_route: bool = False,
) -> None:
    if node.depth == 0:
        return
    _ensure_child_closure_batch_state(node, solver_config, stats)
    yield_value = useful_columns / max(node.child_closure_batch_limit, 1)
    node.child_certification_yield_history.append(yield_value)
    if len(node.child_certification_yield_history) > solver_config.child_certification_yield_window:
        del node.child_certification_yield_history[0]
    window_rate = sum(node.child_certification_yield_history) / len(node.child_certification_yield_history)
    stats.child_certification_yield_rate = window_rate
    stats.child_certification_yield_observations += 1
    stats.child_closure_batch_last = node.child_closure_batch_limit
    stats.child_closure_batch_min = min(
        value for value in (stats.child_closure_batch_min, node.child_closure_batch_limit) if value
    )
    stats.child_closure_batch_max = max(stats.child_closure_batch_max, node.child_closure_batch_limit)


def _child_dual_signature(duals: PricingDuals, tolerance: float) -> tuple:
    scale = max(tolerance, 1e-12)

    def q(value: float) -> int:
        return round(value / scale)

    return (
        tuple(sorted((customer, q(value)) for customer, value in duals.mu.items())),
        q(duals.kappa),
        tuple(sorted((triplet, q(value)) for triplet, value in duals.nu.items())),
    )


def _observe_child_dual_stability(node: NodeState, duals: PricingDuals, solver_config: SolverConfig) -> bool:
    signature = _child_dual_signature(duals, solver_config.child_cert_dual_change_tol)
    node.child_dual_signature_history.append(signature)
    if len(node.child_dual_signature_history) > solver_config.child_cert_dual_stability_window:
        del node.child_dual_signature_history[0]
    return (
        len(node.child_dual_signature_history) >= solver_config.child_cert_dual_stability_window
        and len(set(node.child_dual_signature_history)) == 1
    )


def _child_certification_signature(node: NodeState, duals: PricingDuals) -> tuple:
    dual_tol_key = 1e-12

    def q(value: float) -> int:
        return round(value / dual_tol_key)

    residual_signature = tuple(sorted(node.residual_customers))
    active_sr_signature = tuple(sorted(node.active_sr))
    branch_signature = tuple(sorted(node.restrictions.__dict__.items(), key=lambda item: item[0]))
    fixed_route_signature = tuple(route.path for route in node.fixed_routes)
    active_column_version = (tuple(sorted(node.column_paths)), tuple(sorted(node.inactive_column_paths)))
    rmp_structure_version = (
        ("residual_customer_mask", residual_signature),
        ("active_sr_set_hash", active_sr_signature),
        ("active_sr_version", node.active_sr_version),
        ("fixed_route_signature", fixed_route_signature),
        ("active_column_version", active_column_version),
        ("fleet_limit", node.fleet_limit),
        ("fixed_cost", node.fixed_cost),
    )
    dual_signature = (
        tuple(sorted((customer, q(value)) for customer, value in duals.mu.items())),
        q(duals.kappa),
        tuple(sorted((triplet, q(value)) for triplet, value in duals.nu.items())),
    )
    return (
        ("true_dual_hash", dual_signature),
        ("active_sr_set_hash", active_sr_signature),
        ("residual_customer_mask", residual_signature),
        ("branch_state_signature", branch_signature),
        ("fixed_route_signature", fixed_route_signature),
        ("active_column_version", active_column_version),
        ("rmp_structure_version", rmp_structure_version),
        ("residual", residual_signature),
        ("fixed_routes", fixed_route_signature),
        ("fixed_cost", node.fixed_cost),
        ("fleet_limit", node.fleet_limit),
        ("active_sr_version", node.active_sr_version),
        ("active_sr", active_sr_signature),
        ("branch", branch_signature),
        ("active_columns", active_column_version[0]),
        ("inactive_columns", active_column_version[1]),
        ("objective_scale_version", 0),
        ("dual_mu", dual_signature[0]),
        ("dual_kappa", dual_signature[1]),
        ("dual_nu", dual_signature[2]),
    )


def _record_child_certification_epoch_discard(
    stats: BPCStats,
    old_signature: tuple,
    new_signature: tuple,
) -> None:
    old = dict(old_signature)
    new = dict(new_signature)
    if old.get("true_dual_hash") != new.get("true_dual_hash"):
        stats.child_certification_state_discarded_by_dual += 1
    if old.get("active_sr_version") != new.get("active_sr_version") or old.get("active_sr_set_hash") != new.get("active_sr_set_hash"):
        stats.child_certification_state_discarded_by_sr += 1
    if old.get("residual_customer_mask") != new.get("residual_customer_mask"):
        stats.child_certification_state_discarded_by_residual += 1
    if old.get("branch_state_signature") != new.get("branch_state_signature"):
        stats.child_certification_state_discarded_by_branch += 1
    if (
        old.get("fixed_route_signature") != new.get("fixed_route_signature")
        or old.get("fixed_cost") != new.get("fixed_cost")
        or old.get("fleet_limit") != new.get("fleet_limit")
    ):
        stats.child_certification_state_discarded_by_fixed_routes += 1
    if old.get("active_column_version") != new.get("active_column_version"):
        stats.child_certification_state_discarded_by_active_columns += 1
    if old.get("rmp_structure_version") != new.get("rmp_structure_version"):
        stats.child_certification_state_discarded_by_rmp_structure += 1


def _record_branch_decision(stats: BPCStats, branch_type: str) -> None:
    if branch_type == "customer_pair":
        stats.customer_pair_branches += 1
    elif branch_type == "service_mode":
        stats.service_mode_branches += 1
    elif branch_type == "launch_pad":
        stats.launch_pad_branches += 1
    elif branch_type == "transformed_arc":
        stats.transformed_arc_branches += 1
    elif branch_type == "route_variable":
        stats.route_variable_branches += 1
    else:
        raise RuntimeError(f"unknown branch type {branch_type}")


def _append_pricing_diagnostic(solver_config: SolverConfig, diagnostic: dict) -> None:
    if solver_config.gurobi_log_dir is None or not solver_config.pricing_jsonl_enabled:
        return
    path = Path(solver_config.gurobi_log_dir).parent / "pricing_diagnostics.jsonl"
    with path.open("a", encoding="utf-8") as file:
        file.write(json.dumps(diagnostic, sort_keys=True) + "\n")


def _sync_route_stats(
    stats: BPCStats,
    routes: dict[tuple[str, ...], Route],
    global_pool_paths: set[tuple[str, ...]],
    signature_cache: RouteSignatureCache | None = None,
) -> None:
    stats.total_routes = len(routes)
    stats.global_pool_routes = len(global_pool_paths)
    if signature_cache is not None:
        stats.signature_cache_hits = signature_cache.stats.signature_cache_hits
        stats.signature_cache_misses = signature_cache.stats.signature_cache_misses
        stats.core_signature_cache_hits = signature_cache.stats.core_signature_cache_hits
        stats.core_signature_cache_misses = signature_cache.stats.core_signature_cache_misses
        stats.active_signature_cache_hits = signature_cache.stats.active_signature_cache_hits
        stats.active_signature_cache_misses = signature_cache.stats.active_signature_cache_misses
        stats.sr_coeff_cache_hits = signature_cache.stats.sr_coeff_cache_hits
        stats.sr_coeff_cache_misses = signature_cache.stats.sr_coeff_cache_misses
        stats.active_sr_key_cache_hits = signature_cache.stats.active_sr_key_cache_hits
        stats.active_sr_key_cache_misses = signature_cache.stats.active_sr_key_cache_misses
        stats.active_sr_coeffs_computed = signature_cache.stats.active_sr_coeffs_computed
        stats.triplet_masks_built = signature_cache.stats.triplet_masks_built
        stats.column_index_hits = signature_cache.stats.column_index_hits
        stats.column_index_misses = signature_cache.stats.column_index_misses
        stats.column_index_replacements = signature_cache.stats.column_index_replacements
        stats.column_index_refreshes = signature_cache.stats.column_index_refreshes
        stats.column_index_refresh_time = signature_cache.stats.column_index_refresh_time
        stats.route_signature_build_time = signature_cache.stats.signature_build_time
        stats.sr_coeff_build_time = signature_cache.stats.sr_coeff_build_time
        stats.duplicate_equivalent_columns_rejected = signature_cache.stats.duplicate_equivalent_rejected
        stats.cost_dominated_columns_rejected = signature_cache.stats.cost_dominated_rejected
        stats.cost_dominated_columns_removed = signature_cache.stats.cost_dominated_removed


def _write_progress(
    solver_config: SolverConfig,
    stats: BPCStats,
    node: NodeState,
    event: str,
    extra: dict | None = None,
) -> None:
    if solver_config.gurobi_log_dir is None:
        return
    stats.progress_events_seen += 1
    if not _should_write_progress_event(solver_config, stats, event):
        stats.progress_events_skipped += 1
        return
    serialization_start = time.time()
    stats_record = asdict(stats)
    diagnostics = stats_record.pop("pricing_diagnostics")
    stats_record["pricing_diagnostics_count"] = len(diagnostics)
    stats_record["last_pricing_diagnostic"] = diagnostics[-1] if diagnostics else None
    record = {
        "event": event,
        "updated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "node": {
            "id": node.id,
            "depth": node.depth,
            "residual_customers": sorted(node.residual_customers),
            "fleet_limit": node.fleet_limit,
            "fixed_routes": [route.path for route in node.fixed_routes],
            "columns": len(node.column_paths),
            "active_sr": len(node.active_sr),
            "active_sr_version": node.active_sr_version,
            "fixed_cost": node.fixed_cost,
        },
        "stats": stats_record,
    }
    if extra is not None:
        record["extra"] = extra
    progress_path = Path(solver_config.gurobi_log_dir).parent / "bpc_progress.json"
    _atomic_write_json(progress_path, record)
    stats.progress_events_written += 1
    stats.progress_serialization_time += time.time() - serialization_start


def _should_write_progress_event(solver_config: SolverConfig, stats: BPCStats, event: str) -> bool:
    if solver_config.logging_mode == "audit":
        return stats.progress_events_seen % solver_config.progress_snapshot_period == 0
    major_events = {
        "time_limit_node_unresolved",
        "farkas_certificate",
        "farkas_columns_added",
        "productive_no_columns_not_certified",
        "productive_slice_pricing_unresolved",
        "sr_cuts_added",
        "sr_cuts_removed_for_reprice",
        "inactive_columns_deactivated_for_reprice",
        "inactive_columns_rehydrated",
        "child_certification_slice_limit",
        "node_closed",
        "time_limit_pricing_unresolved",
        "pricing_timeout_with_columns",
        "preclosure_heuristic",
    }
    return event in major_events or (
        solver_config.progress_snapshot_period > 1
        and stats.progress_events_seen % solver_config.progress_snapshot_period == 0
    )


def _atomic_write_json(path: Path, record: dict) -> None:
    path.write_text(json.dumps(record, indent=2), encoding="utf-8")


def _add_initial_routes(graph, objective: ObjectiveData, routes: dict[tuple[str, ...], Route]) -> None:
    instance = graph.instance
    for customer in instance.customers:
        candidates: list[tuple[float, tuple[str, ...]]] = []
        if (instance.depot_source, customer) in graph.truck_arcs and (customer, instance.depot_sink) in graph.truck_arcs:
            path = (instance.depot_source, customer, instance.depot_sink)
            candidates.append((instance.truck_time[(instance.depot_source, customer)], path))
        for hub in instance.hubs:
            if (hub, customer) not in instance.drone_arcs or (instance.depot_source, hub) not in graph.truck_arcs or (hub, instance.depot_sink) not in graph.truck_arcs:
                continue
            from .transform import duplicate_node
            path = (instance.depot_source, hub, duplicate_node(hub, customer), instance.depot_sink)
            service_time = instance.truck_time[(instance.depot_source, hub)] + instance.drone_time[(hub, customer)]
            candidates.append((service_time, path))
        for _, path in sorted(candidates, key=lambda item: (item[0], item[1])):
            try:
                routes[path] = route_from_path(len(routes), path, graph, objective)
                break
            except ServiceEnvelopeViolation:
                continue
        else:
            raise ValueError(f"customer {customer} has no single-customer route satisfying the service envelope")


def _construct_root_incumbent_routes(
    graph,
    objective: ObjectiveData,
    solver_config: SolverConfig,
    routes: dict[tuple[str, ...], Route],
    stats: BPCStats,
    deadline: float,
) -> tuple[set[tuple[str, ...]], tuple[tuple[str, ...], ...]]:
    if solver_config.root_constructive_time_limit == 0.0:
        stats.root_constructive_status = "skipped"
        return set(), tuple()
    local_deadline = min(deadline, time.time() + solver_config.root_constructive_time_limit)
    instance = graph.instance
    residual = set(instance.customers)
    generated: set[tuple[str, ...]] = set()
    selected: list[tuple[str, ...]] = []
    for _ in range(instance.num_trucks):
        if not residual or time.time() >= local_deadline:
            break
        prefix = [instance.depot_source]
        while residual and time.time() < local_deadline:
            extension = _best_constructive_extension(prefix, residual, graph, objective)
            if extension is None:
                break
            prefix, route = extension
        if len(prefix) == 1:
            continue
        path = tuple(prefix + [instance.depot_sink])
        route = route_from_path(len(routes), path, graph, objective)
        routes.setdefault(path, route)
        generated.add(path)
        selected.append(path)
        residual.difference_update(route.served)
    stats.root_constructive_routes = len(generated)
    if not residual:
        stats.root_constructive_status = "success"
        return generated, tuple(selected)
    stats.root_constructive_status = "timeout" if time.time() >= local_deadline else "partial"
    return generated, tuple()


def _diversify_constructive_drone_routes(
    graph,
    objective: ObjectiveData,
    solver_config: SolverConfig,
    routes: dict[tuple[str, ...], Route],
    stats: BPCStats,
    incumbent: _IncumbentState,
    selected_paths: tuple[tuple[str, ...], ...],
    solver_start: float,
    deadline: float,
) -> set[tuple[str, ...]]:
    if (
        not solver_config.enable_drone_diversification_warm_start
        or not selected_paths
    ):
        return set()
    start = time.time()
    local_deadline = deadline
    trigger = (
        incumbent.value == float("inf")
        or (incumbent.routes and sum(route.drone_sorties for route in incumbent.routes) == 0)
        or all(routes[path].drone_sorties == 0 for path in selected_paths)
    )
    if not trigger:
        return set()
    stats.drone_diversification_attempted = True
    generated: set[tuple[str, ...]] = set()
    best_selected = tuple(selected_paths)
    best_value = sum(routes[path].cost for path in best_selected)
    for base_path in selected_paths:
        if time.time() >= local_deadline:
            break
        base_route = routes[base_path]
        candidate_paths, insertion_attempts = _single_drone_diversification_paths(
            base_path,
            base_route,
            graph,
        )
        stats.drone_insertion_attempts += insertion_attempts
        for candidate_path in candidate_paths:
            if time.time() >= local_deadline:
                break
            try:
                candidate_route = route_from_path(len(routes), candidate_path, graph, objective)
            except (ServiceEnvelopeViolation, ValueError):
                continue
            if candidate_route.served != base_route.served:
                continue
            if candidate_route.drone_sorties <= base_route.drone_sorties:
                continue
            routes.setdefault(candidate_path, candidate_route)
            generated.add(candidate_path)
            stats.drone_insertion_verified += 1
            replacement = tuple(candidate_path if path == base_path else path for path in selected_paths)
            replacement_value = sum(routes[path].cost for path in replacement)
            if replacement_value < best_value - solver_config.pricing_tolerance:
                best_value = replacement_value
                best_selected = replacement
                stats.drone_primal_improvements += 1
    stats.drone_diversification_routes_generated += len(generated)
    stats.drone_diversification_columns_accepted += len(generated)
    stats.drone_primal_pool_size += len(generated)
    if generated and best_value < incumbent.value - solver_config.pricing_tolerance:
        selected_routes = tuple(routes[path] for path in best_selected)
        covered = frozenset().union(*(route.served for route in selected_routes))
        if covered == frozenset(graph.instance.customers) and len(selected_routes) <= graph.instance.num_trucks:
            incumbent.value = best_value
            incumbent.routes = selected_routes
            stats.drone_diversification_incumbent_improved = True
            if stats.time_to_first_incumbent is None:
                stats.time_to_first_incumbent = time.time() - solver_start
            stats.incumbent_source = "drone_diversification"
    stats.drone_diversification_time += time.time() - start
    return generated


def _single_drone_diversification_paths(
    path: tuple[str, ...],
    route: Route,
    graph,
) -> tuple[tuple[tuple[str, ...], ...], int]:
    instance = graph.instance
    candidates: list[tuple[str, ...]] = []
    path_list = list(path)
    truck_customers = [
        customer
        for customer in sorted(route.truck_served, key=lambda c: (route.service_times[c], c))
        if customer in path_list and instance.demand[customer] <= instance.drone_payload
    ]
    insertion_attempts = 0
    for hub_index, hub in enumerate(path_list[:-1]):
        if hub not in instance.hubs:
            continue
        for customer in sorted(
            truck_customers,
            key=lambda item: (instance.drone_trip_time.get((hub, item), float("inf")), item),
        ):
            customer_index = path_list.index(customer)
            if customer_index <= hub_index or (hub, customer) not in instance.drone_arcs:
                continue
            insertion_attempts += 1
            duplicate = duplicate_node(hub, customer)
            if duplicate in path_list or (hub, duplicate) not in graph.arcs:
                continue
            suffix = [node for idx, node in enumerate(path_list[hub_index + 1 :]) if hub_index + 1 + idx != customer_index]
            if not suffix:
                continue
            candidate = tuple(path_list[: hub_index + 1] + [duplicate] + suffix)
            if any(is_duplicate(node) and node == duplicate for node in path_list):
                continue
            if all((i, j) in graph.arcs for i, j in zip(candidate, candidate[1:])):
                candidates.append(candidate)
    return tuple(sorted(set(candidates))), insertion_attempts


def _best_constructive_extension(
    prefix: list[str],
    residual: set[str],
    graph,
    objective: ObjectiveData,
) -> tuple[list[str], Route] | None:
    instance = graph.instance
    best: tuple[tuple[float, float, str, tuple[str, ...]], list[str], Route] | None = None
    for customer in sorted(residual):
        for connector in _constructive_connectors(prefix, customer, graph):
            candidate_prefix = prefix + list(connector)
            path = tuple(candidate_prefix + [instance.depot_sink])
            try:
                route = route_from_path(-1, path, graph, objective)
            except (ServiceEnvelopeViolation, ValueError):
                continue
            if not route.served.issubset(residual):
                continue
            payload = sum(instance.demand[c] for c in route.served)
            if payload > instance.truck_payload:
                continue
            min_slack = min(objective.bounds.service_ub[c] - route.service_times[c] for c in route.served)
            score = (min_slack, route.return_time, customer, path)
            if best is None or score < best[0]:
                best = (score, candidate_prefix, route)
    if best is None:
        return None
    _, candidate_prefix, route = best
    return candidate_prefix, route


def _constructive_connectors(prefix: list[str], customer: str, graph) -> tuple[tuple[str, ...], ...]:
    instance = graph.instance
    current = prefix[-1]
    visited = set(prefix)
    connectors: list[tuple[str, ...]] = []
    if customer not in visited and (current, customer) in graph.truck_arcs:
        connectors.append((customer,))
    for hub in instance.hubs:
        if hub in visited or customer in visited:
            continue
        if (current, hub) in graph.truck_arcs and (hub, customer) in graph.truck_arcs:
            connectors.append((hub, customer))
    return tuple(connectors)


def _extract_root_routes(
    instance: InstanceData,
    weights: ObjectiveWeights,
    solver_config: SolverConfig,
    graph,
    objective: ObjectiveData,
    routes: dict[tuple[str, ...], Route],
    stats: BPCStats,
    deadline: float,
    constructive_incumbent_found: bool,
) -> set[tuple[str, ...]]:
    if solver_config.root_extraction_time_limit == 0:
        stats.root_compact_status = "skipped"
        stats.root_compact_skipped_reason = "disabled"
        return set()
    if constructive_incumbent_found:
        if solver_config.root_compact_after_constructive == "skip":
            stats.root_compact_status = "skipped"
            stats.root_compact_skipped_reason = "constructive_incumbent"
            return set()
        if solver_config.root_compact_after_constructive in {"conditional_small_budget", "conditional_wall_budget"}:
            trigger_reasons = []
            if stats.root_constructive_diversity_score < solver_config.constructive_diversity_threshold:
                trigger_reasons.append("low_diversity")
            if stats.root_constructive_truck_count >= graph.instance.num_trucks:
                trigger_reasons.append("all_trucks_used")
            if stats.root_constructive_drone_sorties == 0 and graph.instance.drones_per_truck > 0:
                trigger_reasons.append("no_drone_sorties")
            if (
                solver_config.constructive_incumbent_quality_threshold is not None
                and stats.root_constructive_value is not None
                and stats.root_constructive_value > solver_config.constructive_incumbent_quality_threshold
            ):
                trigger_reasons.append("quality_threshold")
            if not trigger_reasons:
                stats.root_compact_status = "skipped"
                stats.root_compact_skipped_reason = "constructive_condition_not_triggered"
                return set()
            stats.root_compact_conditional_triggered = True
            stats.root_compact_conditional_reason = ",".join(trigger_reasons)
            budget = (
                solver_config.root_compact_solve_time_limit
                if solver_config.root_compact_after_constructive == "conditional_wall_budget"
                else solver_config.root_compact_time_limit_after_constructive
            )
        else:
            if solver_config.root_compact_after_constructive == "small_budget":
                budget = solver_config.root_compact_time_limit_after_constructive
            elif solver_config.root_compact_after_constructive == "full_budget":
                budget = solver_config.root_compact_solve_time_limit
            else:
                budget = solver_config.root_extraction_time_limit
    else:
        budget = solver_config.root_compact_time_limit_without_constructive
    stats.root_compact_budget_seconds = budget
    stats.root_compact_wall_budget_seconds = 0.0
    solve_budget = (
        solver_config.root_compact_solve_time_limit
        if solver_config.root_compact_after_constructive in {"conditional_wall_budget", "full_budget"}
        else budget
    )
    stats.root_compact_solve_budget_seconds = solve_budget
    if budget <= 0.0:
        stats.root_compact_status = "skipped"
        stats.root_compact_skipped_reason = "zero_budget"
        return set()
    if deadline - time.time() < budget:
        stats.root_compact_status = "skipped"
        stats.root_compact_skipped_reason = "insufficient_remaining_time"
        return set()
    stats.root_compact_attempted = True
    log_file = None
    if solver_config.gurobi_log_dir is not None:
        log_file = str(Path(solver_config.gurobi_log_dir) / "root_compact.log")
    solution = solve_compact_solution(
        instance,
        weights,
        time_limit=solve_budget,
        threads=solver_config.threads,
        require_optimal=False,
        log_file=log_file,
        objective=objective,
    )
    stats.root_model_build_time += solution.timing.model_build_time
    stats.root_model_solve_time += solution.timing.solve_time
    stats.root_route_decode_time += solution.timing.route_decode_time
    stats.root_compact_status = solution.status if solution.status != "unknown" else ("success" if solution.route_paths else "timeout")
    stats.root_compact_wall_budget_hit = False
    extracted = set()
    for path in solution.route_paths:
        decode_start = time.time()
        route = route_from_path(len(routes), path, graph, objective)
        stats.root_compact_decode_verification_time += time.time() - decode_start
        routes.setdefault(path, route)
        extracted.add(path)
    return extracted


def _hydrate_node_from_pool(
    graph,
    node: NodeState,
    routes: dict[tuple[str, ...], Route],
    global_pool_paths: set[tuple[str, ...]],
    stats: BPCStats,
    signature_cache: RouteSignatureCache,
    solver_config: SolverConfig,
    branch_index_cache: dict,
) -> None:
    start = time.time()
    arc_customer_sets = {arc: graph.arc_customer_set(arc) for arc in graph.arcs}
    candidates = (
        _query_global_branch_index(
            graph,
            node,
            routes,
            global_pool_paths,
            stats,
            signature_cache,
            solver_config,
            branch_index_cache,
        )
        if solver_config.enable_global_branch_route_index
        else set(global_pool_paths)
    )
    for path in sorted(candidates):
        if path in node.inactive_column_paths:
            continue
        route = routes[path]
        if (
            route.served
            and route.served.issubset(node.residual_customers)
            and node.restrictions.route_allowed(route, arc_customer_sets)
        ):
            node.column_paths.add(path)
    elapsed = time.time() - start
    stats.route_pool_hydration_time += elapsed
    stats.column_hydration_time += elapsed
    _sync_route_stats(stats, routes, global_pool_paths, signature_cache)


def _inherit_child_routes(
    graph,
    objective: ObjectiveData,
    parent: NodeState,
    child: NodeState,
    routes: dict[tuple[str, ...], Route],
    global_pool_paths: set[tuple[str, ...]],
    stats: BPCStats,
    signature_cache: RouteSignatureCache,
    solver_config: SolverConfig | None = None,
    branch_index_cache: dict | None = None,
) -> None:
    solver_config = solver_config or SolverConfig(enable_global_branch_route_index=False)
    branch_index_cache = branch_index_cache if branch_index_cache is not None else {}
    start = time.time()
    arc_customer_sets = {arc: graph.arc_customer_set(arc) for arc in graph.arcs}
    candidates = set(parent.column_paths) | set(global_pool_paths)
    stats.child_branch_index_candidates_before += len(candidates)
    if solver_config.enable_global_branch_route_index and candidates.issubset(global_pool_paths):
        query_start = time.time()
        indexed_candidates = _query_global_branch_index(
            graph,
            child,
            routes,
            global_pool_paths,
            stats,
            signature_cache,
            solver_config,
            branch_index_cache,
        )
        stats.child_branch_index_query_time += time.time() - query_start
        branch_rejections = {
            "route_forbidden": 0,
            "together": 0,
            "separate": 0,
            "service_mode": 0,
            "launch_pad": 0,
            "transformed_arc": 0,
        }
    else:
        index_start = time.time()
        branch_index = build_branch_route_index(candidates, routes, graph, child.residual_customers, signature_cache)
        stats.child_branch_index_build_time += time.time() - index_start
        query_start = time.time()
        indexed_candidates, branch_rejections = query_branch_route_index(
            branch_index,
            child.restrictions,
            arc_customer_sets,
        )
        stats.child_branch_index_query_time += time.time() - query_start
    stats.child_branch_index_candidates_after += len(indexed_candidates)
    stats.child_branch_index_reject_route_forbidden += branch_rejections["route_forbidden"]
    stats.child_branch_index_reject_together += branch_rejections["together"]
    stats.child_branch_index_reject_separate += branch_rejections["separate"]
    stats.child_branch_index_reject_service_mode += branch_rejections["service_mode"]
    stats.child_branch_index_reject_launch_pad += branch_rejections["launch_pad"]
    stats.child_branch_index_reject_transformed_arc += branch_rejections["transformed_arc"]
    accepted: set[tuple[str, ...]] = set()
    for path in sorted(indexed_candidates):
        route = routes[path]
        stats.child_inherited_route_candidates += 1
        rejection = _child_inheritance_rejection_reason(
            graph,
            objective,
            child,
            route,
            arc_customer_sets,
            signature_cache,
        )
        if rejection is None:
            accepted.add(path)
            stats.child_inherited_route_accepted += 1
        else:
            stats.child_inherited_route_rejected += 1
            _record_child_inheritance_rejection(stats, rejection)
    child.column_paths = accepted
    refresh_start = time.time()
    refreshed = _ensure_node_column_index(graph, child, routes, stats, signature_cache)
    stats.child_refresh_count += len(refreshed)
    stats.child_refresh_time += time.time() - refresh_start
    elapsed = time.time() - start
    stats.route_pool_hydration_time += elapsed
    stats.column_hydration_time += elapsed
    stats.child_hydration_time += elapsed
    _sync_route_stats(stats, routes, global_pool_paths, signature_cache)


def _query_global_branch_index(
    graph,
    node: NodeState,
    routes: dict[tuple[str, ...], Route],
    global_pool_paths: set[tuple[str, ...]],
    stats: BPCStats,
    signature_cache: RouteSignatureCache,
    solver_config: SolverConfig,
    branch_index_cache: dict,
) -> set[tuple[str, ...]]:
    stats.global_branch_index_queries += 1
    key = (tuple(sorted(node.residual_customers)), 0)
    branch_index = branch_index_cache.get(key)
    if branch_index is None:
        build_start = time.time()
        branch_index = build_branch_route_index(global_pool_paths, routes, graph, node.residual_customers, signature_cache)
        branch_index_cache[key] = branch_index
        stats.global_branch_index_builds += 1
        stats.global_branch_index_build_time += time.time() - build_start
    else:
        missing_paths = frozenset(global_pool_paths).difference(branch_index.all_paths)
        if missing_paths:
            build_start = time.time()
            branch_index = extend_branch_route_index(
                branch_index,
                missing_paths,
                routes,
                graph,
                node.residual_customers,
                signature_cache,
            )
            branch_index_cache[key] = branch_index
            stats.global_branch_index_incremental_updates += 1
            stats.global_branch_index_incremental_paths += len(missing_paths)
            stats.global_branch_index_build_time += time.time() - build_start
    arc_customer_sets = {arc: graph.arc_customer_set(arc) for arc in graph.arcs}
    query_start = time.time()
    candidates, rejections = query_branch_route_index(branch_index, node.restrictions, arc_customer_sets)
    query_elapsed = time.time() - query_start
    rejected = sum(rejections.values())
    stats.global_branch_index_query_time += query_elapsed
    stats.global_branch_index_candidates += len(candidates)
    stats.global_branch_index_rejections += rejected
    return candidates


def _child_inheritance_rejection_reason(
    graph,
    objective: ObjectiveData,
    child: NodeState,
    route: Route,
    arc_customer_sets: dict[tuple[str, str], frozenset[str]],
    signature_cache: RouteSignatureCache,
) -> str | None:
    if not route.served:
        return "residual"
    fixed_served = frozenset().union(*(fixed.served for fixed in child.fixed_routes)) if child.fixed_routes else frozenset()
    if route.served.intersection(fixed_served):
        return "fixed_route"
    if child.fleet_limit <= 0:
        return "fleet"
    if not route.served.issubset(child.residual_customers):
        return "residual"
    for customer, service_time in route.service_times.items():
        if customer in child.residual_customers and service_time > objective.bounds.service_ub[customer] + 1e-9:
            return "deadline"
    if not child.restrictions.route_allowed(route, arc_customer_sets):
        return "branch"
    try:
        route_signature(
            route,
            graph,
            child.residual_customers,
            signature_cache,
            child.active_sr,
            child.active_sr_version,
        )
    except (KeyError, ValueError):
        return "sr_signature"
    return None


def _record_child_inheritance_rejection(stats: BPCStats, reason: str) -> None:
    if reason == "residual":
        stats.child_reject_residual += 1
    elif reason == "fixed_route":
        stats.child_reject_fixed_route += 1
    elif reason == "fleet":
        stats.child_reject_fleet += 1
    elif reason == "deadline":
        stats.child_reject_deadline += 1
    elif reason == "branch":
        stats.child_reject_branch += 1
    elif reason == "sr_signature":
        stats.child_reject_sr_signature += 1
    else:
        stats.child_reject_route_signature += 1


def _node_admissible_column_paths(
    graph,
    node: NodeState,
    routes: dict[tuple[str, ...], Route],
) -> set[tuple[str, ...]]:
    arc_customer_sets = {arc: graph.arc_customer_set(arc) for arc in graph.arcs}
    return {
        path
        for path in node.column_paths
        if (
            routes[path].served
            and routes[path].served.issubset(node.residual_customers)
            and node.restrictions.route_allowed(routes[path], arc_customer_sets)
        )
    }


def _ensure_node_column_index(
    graph,
    node: NodeState,
    routes: dict[tuple[str, ...], Route],
    stats: BPCStats | None,
    signature_cache: RouteSignatureCache | None,
) -> set[tuple[str, ...]]:
    admissible_paths = _node_admissible_column_paths(graph, node, routes)
    if not node.column_index.matches(node.residual_customers, node.active_sr, node.active_sr_version, admissible_paths):
        previous_version = node.column_index.active_sr_version
        refresh_start = time.time()
        refresh_node_column_index(
            node.column_index,
            routes,
            admissible_paths,
            graph,
            node.residual_customers,
            signature_cache,
            active_sr=node.active_sr,
            active_sr_version=node.active_sr_version,
        )
        refresh_elapsed = time.time() - refresh_start
        if stats is not None:
            stats.column_signature_lookup_time += refresh_elapsed
        if stats is not None and previous_version >= 0 and previous_version != node.active_sr_version:
            stats.active_sr_version_refreshes += 1
            stats.active_sr_version_refresh_time += refresh_elapsed
    return admissible_paths


def _insert_node_column(
    route: Route,
    routes: dict[tuple[str, ...], Route],
    node: NodeState,
    graph,
    stats: BPCStats | None = None,
    signature_cache: RouteSignatureCache | None = None,
    objective: ObjectiveData | None = None,
) -> tuple[tuple[str, ...], bool]:
    _assert_route_service_envelope(route, objective, stats)
    start = time.time()
    admissible_paths = _ensure_node_column_index(graph, node, routes, stats, signature_cache)
    indexed_paths = set(admissible_paths)
    result = insert_node_column(
        route,
        routes,
        indexed_paths,
        graph,
        node.residual_customers,
        cache=signature_cache,
        active_sr=node.active_sr,
        active_sr_version=node.active_sr_version,
        column_index=node.column_index,
    )
    node.column_paths.difference_update(admissible_paths - indexed_paths)
    node.column_paths.update(indexed_paths - admissible_paths)
    if stats is not None:
        stats.rmp_column_insertion_time += time.time() - start
    return result


def _add_side_pool_routes(
    side_routes: tuple[Route, ...],
    routes: dict[tuple[str, ...], Route],
    side_pool_paths: set[tuple[str, ...]],
    node: NodeState,
    graph,
    stats: BPCStats,
    solver_config: SolverConfig,
    z_values: dict[tuple[str, ...], float],
    objective: ObjectiveData | None = None,
    side_route_reduced_costs: dict[tuple[str, ...], float] | None = None,
) -> None:
    if not side_routes:
        stats.side_pool_routes = len(side_pool_paths)
        stats.side_pool_max_routes = max(stats.side_pool_max_routes, len(side_pool_paths))
        stats.side_pool_batch_size_last = 0
        return
    stats.side_pool_candidates_seen += len(side_routes)
    arc_customer_sets = {arc: graph.arc_customer_set(arc) for arc in graph.arcs}
    added = 0
    for route in side_routes:
        _assert_route_service_envelope(route, objective, stats)
        if not route.served or not route.served.issubset(node.residual_customers):
            raise RuntimeError("side-pool route violates residual-customer scope")
        if not node.restrictions.route_allowed(route, arc_customer_sets):
            raise RuntimeError("side-pool route violates branch restrictions")
        if route.path in node.column_paths:
            continue
        routes.setdefault(route.path, route)
        if route.path not in side_pool_paths:
            side_pool_paths.add(route.path)
            added += 1
    stats.side_pool_routes_added += added
    stats.side_pool_batch_size_last = added
    _prune_side_pool(routes, side_pool_paths, node, solver_config, z_values, stats, side_route_reduced_costs)
    stats.side_pool_routes = len(side_pool_paths)
    stats.side_pool_max_routes = max(stats.side_pool_max_routes, len(side_pool_paths))


def _assert_route_service_envelope(
    route: Route,
    objective: ObjectiveData | None,
    stats: BPCStats | None = None,
) -> None:
    if objective is not None:
        for customer, service_time in route.service_times.items():
            if service_time > objective.bounds.service_ub[customer] + 1e-9:
                if stats is not None:
                    stats.late_rmp_routes_rejected += 1
                raise RuntimeError("route violates service upper bound before RMP insertion")
    if not route.served.issubset(route.service_times):
        if stats is not None:
            stats.late_rmp_routes_rejected += 1
        raise RuntimeError("route is missing service times before RMP insertion")


def _prune_side_pool(
    routes: dict[tuple[str, ...], Route],
    side_pool_paths: set[tuple[str, ...]],
    node: NodeState,
    solver_config: SolverConfig,
    z_values: dict[tuple[str, ...], float],
    stats: BPCStats,
    side_route_reduced_costs: dict[tuple[str, ...], float] | None = None,
) -> None:
    if not side_pool_paths:
        return
    prune_start = time.time()
    stats.side_pool_per_customer_keep = solver_config.side_pool_per_customer_keep
    coverage_frequency: dict[str, int] = {customer: 0 for customer in node.residual_customers}
    for path in side_pool_paths:
        for customer in routes[path].served:
            if customer in coverage_frequency:
                coverage_frequency[customer] += 1
    scored = sorted(
        (
            _primal_side_pool_score(
                routes[path],
                node,
                z_values,
                coverage_frequency,
                solver_config,
                side_route_reduced_costs,
            ),
            routes[path].cost,
            path,
        )
        for path in side_pool_paths
    )
    keep: set[tuple[str, ...]] = set()
    per_customer_counts: dict[str, int] = {customer: 0 for customer in node.residual_customers}
    for _, _, path in scored:
        if len(keep) >= solver_config.side_pool_max_size:
            break
        served = routes[path].served.intersection(node.residual_customers)
        if not served:
            continue
        if all(per_customer_counts[customer] >= solver_config.side_pool_per_customer_keep for customer in served):
            continue
        keep.add(path)
        for customer in served:
            per_customer_counts[customer] += 1
    pruned = len(side_pool_paths - keep)
    side_pool_paths.intersection_update(keep)
    stats.side_pool_routes_pruned += pruned
    stats.side_pool_prune_time += time.time() - prune_start


def _primal_side_pool_score(
    route: Route,
    node: NodeState,
    z_values: dict[tuple[str, ...], float],
    coverage_frequency: dict[str, int],
    solver_config: SolverConfig,
    side_route_reduced_costs: dict[tuple[str, ...], float] | None = None,
) -> float:
    coverage_gain = len(route.served.intersection(node.residual_customers))
    lp_support = z_values.get(route.path, 0.0)
    redundancy = sum(coverage_frequency.get(customer, 0) for customer in route.served) / max(len(route.served), 1)
    nonnegative_reduced_cost_penalty = 0.0
    if side_route_reduced_costs is not None and route.path in side_route_reduced_costs:
        nonnegative_reduced_cost_penalty = max(side_route_reduced_costs[route.path], 0.0)
    duration_penalty = route.return_time / max(len(route.served), 1)
    return (
        route.cost
        + nonnegative_reduced_cost_penalty
        - solver_config.seed_reward * coverage_gain
        - solver_config.dive_reward * lp_support
        + solver_config.dive_reward * redundancy
        + solver_config.dive_reward * duration_penalty
    )


def _merge_duplicate_column_paths_for_node(
    node: NodeState,
    routes: dict[tuple[str, ...], Route],
    graph,
    stats: BPCStats | None = None,
    signature_cache: RouteSignatureCache | None = None,
) -> set[tuple[str, ...]]:
    start = time.time()
    admissible_paths = _node_admissible_column_paths(graph, node, routes)
    inactive_paths = node.column_paths - admissible_paths
    merged = inactive_paths | merge_duplicate_column_paths(
        admissible_paths,
        routes,
        graph,
        node.residual_customers,
        signature_cache,
        active_sr=node.active_sr,
        active_sr_version=node.active_sr_version,
    )
    if stats is not None:
        stats.duplicate_lookup_time += time.time() - start
    return merged


def _is_integer(z_values: dict[tuple[str, ...], float], tolerance: float) -> bool:
    return all(abs(value - round(value)) <= tolerance for value in z_values.values())


def _branch(
    node: NodeState,
    z_values: dict[tuple[str, ...], float],
    routes: dict[tuple[str, ...], Route],
    graph,
    left_id: int,
    right_id: int,
    solver_config: SolverConfig,
) -> BranchDecision:
    residual = sorted(node.residual_customers)
    fractional_pairs = []
    for p, q in combinations(residual, 2):
        flow = sum(value for path, value in z_values.items() if p in routes[path].served and q in routes[path].served)
        if _fractional(flow, solver_config.integrality_tolerance):
            fractional_pairs.append((abs(flow - 0.5), p, q))
    if fractional_pairs:
        _, p, q = min(fractional_pairs)
        return BranchDecision(
            "customer_pair",
            node.copy_for_child(left_id, node.restrictions.with_together(p, q)),
            node.copy_for_child(right_id, node.restrictions.with_separate(p, q)),
        )

    fractional_modes = []
    for customer in residual:
        flow = sum(value for path, value in z_values.items() if customer in routes[path].drone_served)
        if _fractional(flow, solver_config.integrality_tolerance):
            fractional_modes.append((abs(flow - 0.5), customer))
    if fractional_modes:
        _, customer = min(fractional_modes)
        return BranchDecision(
            "service_mode",
            node.copy_for_child(left_id, node.restrictions.with_truck_service(customer)),
            node.copy_for_child(right_id, node.restrictions.with_drone_service(customer)),
        )

    fractional_pads = []
    for hub in graph.instance.hubs:
        for customer in residual:
            flow = sum(value for path, value in z_values.items() if (hub, customer) in routes[path].pad_served)
            if _fractional(flow, solver_config.integrality_tolerance):
                fractional_pads.append((abs(flow - 0.5), hub, customer))
    if fractional_pads:
        _, hub, customer = min(fractional_pads)
        return BranchDecision(
            "launch_pad",
            node.copy_for_child(left_id, node.restrictions.with_pad_forbidden(hub, customer)),
            node.copy_for_child(right_id, node.restrictions.with_pad_required(hub, customer)),
        )

    fractional_arcs = []
    for arc in sorted(graph.arcs):
        if not graph.arc_customer_set(arc):
            continue
        flow = sum(value for path, value in z_values.items() if arc in routes[path].used_arcs)
        if _fractional(flow, solver_config.integrality_tolerance):
            fractional_arcs.append((abs(flow - 0.5), arc))
    if fractional_arcs:
        _, arc = min(fractional_arcs)
        return BranchDecision(
            "transformed_arc",
            node.copy_for_child(left_id, node.restrictions.with_trans_arc_forbidden(arc)),
            node.copy_for_child(right_id, node.restrictions.with_trans_arc_required(arc)),
        )

    path, value = min(
        ((path, value) for path, value in z_values.items() if _fractional(value, solver_config.integrality_tolerance)),
        key=lambda item: abs(item[1] - 0.5),
    )
    down = node.copy_for_child(left_id, node.restrictions.with_route_forbidden(path))
    route = routes[path]
    child_residual = frozenset(node.residual_customers - route.served)
    child_active_sr = {triplet for triplet in node.active_sr if set(triplet).issubset(child_residual)}
    child_removed_sr = {triplet for triplet in node.removed_sr if set(triplet).issubset(child_residual)}
    child_sr_meta = {
        triplet: SRCutMetadata(
            age=meta.age,
            inactive_count=meta.inactive_count,
            last_activity=meta.last_activity,
            last_violation=meta.last_violation,
            last_positive_dual_iteration=meta.last_positive_dual_iteration,
            activity_count=meta.activity_count,
            nonzero_count=meta.nonzero_count,
            coefficient_density=meta.coefficient_density,
            update_time_contribution=meta.update_time_contribution,
            removal_candidate_count=meta.removal_candidate_count,
            removal_count=meta.removal_count,
            reactivation_count=meta.reactivation_count,
        )
        for triplet, meta in node.sr_cut_meta.items()
        if set(triplet).issubset(child_residual)
    }
    up = NodeState(
        id=right_id,
        depth=node.depth + 1,
        restrictions=node.restrictions,
        fixed_routes=node.fixed_routes + (route,),
        residual_customers=child_residual,
        fleet_limit=node.fleet_limit - 1,
        fixed_cost=node.fixed_cost + route.cost,
        column_paths=set(),
        active_sr=child_active_sr,
        active_sr_version=node.active_sr_version + 1,
        sr_cut_meta=child_sr_meta,
        removed_sr=child_removed_sr,
    )
    return BranchDecision("route_variable", down, up)


def _fractional(value: float, tolerance: float) -> bool:
    return tolerance < value < 1.0 - tolerance and abs(value - round(value)) > tolerance
