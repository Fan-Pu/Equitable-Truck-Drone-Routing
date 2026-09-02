from __future__ import annotations
from dataclasses import asdict, dataclass, field

from heapq import heappop, heappush

from itertools import combinations, count

import json

import os

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

from .pricing import (
    PricingDuals,
    PricingEpochContext,
    PricingResult,
    PricingSchedulerConfig,
    PricingTimeLimitReached,
    SourceNeighborPricingPool,
    price_route,
)

from .rmp import NodeState, RestrictedMaster, SRCutMetadata

from .routes import Route, route_from_path, validate_route_cover

from .transform import build_transformed_graph

class _NodeTimeLimitReached(RuntimeError):
    pass


class BPCBoundInconsistency(RuntimeError):
    def __init__(self, lower_bound: float, upper_bound: float, tolerance: float) -> None:
        super().__init__(
            "root lower bound exceeds the feasible incumbent "
            f"({lower_bound} > {upper_bound} + {tolerance})"
        )
        self.lower_bound = lower_bound
        self.upper_bound = upper_bound
        self.tolerance = tolerance


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
    active_signature_cache_hits: int = 0
    active_signature_cache_misses: int = 0
    active_sr_coeffs_computed: int = 0
    active_sr_full_rebuilds: int = 0
    active_sr_key_cache_hits: int = 0
    active_sr_key_cache_misses: int = 0
    active_sr_row_local_updates: int = 0
    active_sr_rows_added: int = 0
    active_sr_rows_removed: int = 0
    active_sr_version_refresh_time: float = 0.0
    active_sr_version_refreshes: int = 0
    best_open_bound: float | None = None
    best_reduced_cost_at_stop: float | None = None
    branching_nodes: int = 0
    child_branch_index_build_time: float = 0.0
    child_branch_index_candidates_after: int = 0
    child_branch_index_candidates_before: int = 0
    child_branch_index_query_time: float = 0.0
    child_branch_index_reject_launch_pad: int = 0
    child_branch_index_reject_separate: int = 0
    child_branch_index_reject_together: int = 0
    child_branch_index_reject_conditioned_arc: int = 0
    child_hydration_time: float = 0.0
    child_inherited_route_accepted: int = 0
    child_inherited_route_candidates: int = 0
    child_inherited_route_rejected: int = 0
    child_nodes_created: int = 0
    child_refresh_count: int = 0
    child_refresh_time: float = 0.0
    child_reject_branch: int = 0
    child_reject_deadline: int = 0
    child_reject_fixed_route: int = 0
    child_reject_fleet: int = 0
    child_reject_residual: int = 0
    child_reject_route_signature: int = 0
    child_reject_sr_signature: int = 0
    column_hydration_time: float = 0.0
    column_index_hits: int = 0
    column_index_misses: int = 0
    column_index_refresh_time: float = 0.0
    column_index_refreshes: int = 0
    column_index_replacements: int = 0
    column_signature_lookup_time: float = 0.0
    columns_added_farkas: int = 0
    columns_added_standard: int = 0
    core_signature_cache_hits: int = 0
    core_signature_cache_misses: int = 0
    cost_dominated_columns_rejected: int = 0
    cost_dominated_columns_removed: int = 0
    customer_pair_branches: int = 0
    drone_served_customers: int = 0
    duplicate_arcs: int = 0
    duplicate_columns_rejected: int = 0
    duplicate_equivalent_columns_rejected: int = 0
    duplicate_lookup_time: float = 0.0
    duplicate_merge_events: int = 0
    duplicate_nodes: int = 0
    farkas_certificate_max_column_activity: float | None = None
    farkas_certificate_rhs: float | None = None
    farkas_pricing_calls: int = 0
    farkas_pricing_time: float = 0.0
    final_selected_drone_sorties: int = 0
    fleet_dual_sign_violations: int = 0
    global_branch_index_build_time: float = 0.0
    global_branch_index_builds: int = 0
    global_branch_index_candidates: int = 0
    global_branch_index_incremental_paths: int = 0
    global_branch_index_incremental_updates: int = 0
    global_branch_index_queries: int = 0
    global_branch_index_query_time: float = 0.0
    global_branch_index_rejections: int = 0
    global_pool_routes: int = 0
    heuristic_calls: int = 0
    heuristic_full_pool_calls: int = 0
    heuristic_full_pool_feasible: int = 0
    heuristic_full_pool_incumbent_updates: int = 0
    heuristic_full_pool_time: float = 0.0
    heuristic_hard_pool_feasible_solves: int = 0
    heuristic_hard_pool_solves: int = 0
    heuristic_hard_pool_time: float = 0.0
    heuristic_incumbent_updates: int = 0
    heuristic_max_node_pool_routes: int = 0
    heuristic_time: float = 0.0
    incumbent_source: str = "none"
    initialization_time: float = 0.0
    late_rmp_routes_rejected: int = 0
    launch_pad_branches: int = 0
    max_active_sr: int = 0
    max_active_sr_version: int = 0
    max_node_columns: int = 0
    max_positive_le_dual_violation: float = 0.0
    open_nodes_at_termination: int = 0
    postroot_farkas_pricing_time: float = 0.0
    postroot_heuristic_calls: int = 0
    postroot_heuristic_full_pool_calls: int = 0
    postroot_heuristic_hard_pool_solves: int = 0
    postroot_heuristic_incumbent_updates: int = 0
    postroot_heuristic_time: float = 0.0
    postroot_nodes_processed: int = 0
    postroot_open_nodes: int = 0
    postroot_rmp_time: float = 0.0
    postroot_sr_separation_time: float = 0.0
    postroot_standard_pricing_time: float = 0.0
    pricing_candidate_paths_after_merge: int = 0
    pricing_candidate_paths_before_merge: int = 0
    pricing_certification_worker_calls: int = 0
    pricing_certified_calls: int = 0
    pricing_closure_mode_calls: int = 0
    pricing_complete_routes_generated: int = 0
    pricing_cpu_core_equivalent_max: float = 0.0
    pricing_deadline_reachability_removed: int = 0
    pricing_decoded_routes_in_main: int = 0
    pricing_diagnostics: list[dict] = field(default_factory=list)
    pricing_diagnostics_elapsed_seconds: float = 0.0
    pricing_dom_candidate_pairs_materialized: int = 0
    pricing_dom_frontier_keys_scanned: int = 0
    pricing_dom_frontier_queries: int = 0
    pricing_dom_labels_deleted_physical_location: int = 0
    pricing_dom_labels_deleted_same_node: int = 0
    pricing_dom_pairs_avoided_before_materialization: int = 0
    pricing_engine: str = "source_neighbor_parallel_forward"
    queue_bound_fathoms: int = 0
    pricing_extensions_attempted: int = 0
    pricing_extensions_rejected_by_deadline: int = 0
    pricing_farkas_bound_pruned: int = 0
    pricing_forward_branch_interface_failures: int = 0
    pricing_forward_branch_language_failures: int = 0
    pricing_forward_dominance_tests: int = 0
    pricing_forward_labeling_time: float = 0.0
    pricing_forward_labels_generated: int = 0
    pricing_forward_physical_location_dominance_tests: int = 0
    pricing_forward_return_time_credit_checks: int = 0
    pricing_forward_return_time_credit_checks_skipped: int = 0
    pricing_forward_same_node_dominance_tests: int = 0
    pricing_labels_dominated: int = 0
    pricing_labels_generated: int = 0
    pricing_labels_pruned: int = 0
    pricing_labels_purged: int = 0
    pricing_together_branch_reachability_pruned: int = 0
    pricing_max_queue_size: int = 0
    pricing_negative_routes_inserted: int = 0
    pricing_negative_routes_verified: int = 0
    pricing_parallel_calls: int = 0
    pricing_parallel_workers_max: int = 1
    pricing_pool_reused_calls: int = 0
    pricing_pool_shutdown_time: float = 0.0
    pricing_pool_startup_count: int = 0
    pricing_pool_startup_time: float = 0.0
    pricing_process_cpu_time: float = 0.0
    pricing_productive_mode_calls: int = 0
    pricing_productive_worker_calls: int = 0
    pricing_source_neighbor_count_max: int = 0
    pricing_source_neighbor_task_count_max: int = 0
    pricing_stale_labels_skipped: int = 0
    pricing_standard_bound_pruned: int = 0
    pricing_task_submission_time: float = 0.0
    pricing_tolerance: float = 0.0001
    pricing_verified_routes_in_main: int = 0
    pricing_balanced_process_dynamic_calls: int = 0
    pricing_initial_load_imbalance_max_mean: float = 0.0
    pricing_worker_busy_seconds: float = 0.0
    pricing_worker_idle_seconds: float = 0.0
    pricing_idle_work_requests: int = 0
    pricing_dynamic_split_candidates: int = 0
    pricing_dynamic_splits_performed: int = 0
    pricing_dynamic_split_rejected_near_closure: int = 0
    pricing_dynamic_split_rejected_small_frontier: int = 0
    pricing_dynamic_split_rejected_elapsed: int = 0
    pricing_dynamic_split_rejected_low_work: int = 0
    pricing_dynamic_child_tasks_created: int = 0
    pricing_dynamic_labels_transferred: int = 0
    pricing_dynamic_bytes_transferred: int = 0
    pricing_dynamic_split_control_time: float = 0.0
    pricing_leaf_tasks_created: int = 0
    pricing_leaf_tasks_closed: int = 0
    pricing_pending_transfer_peak: int = 0
    pricing_stale_worker_results_discarded: int = 0
    pricing_epoch_invalidations: int = 0
    pricing_candidate_verification_time: float = 0.0
    pricing_master_control_time: float = 0.0
    pricing_candidate_checkpoints: int = 0
    pricing_candidate_worker_resumptions: int = 0
    pricing_global_verified_candidates: int = 0
    pricing_global_batch_limit_cancellations: int = 0
    progress_events_seen: int = 0
    progress_events_skipped: int = 0
    progress_events_written: int = 0
    progress_serialization_time: float = 0.0
    rmp_active_coefficient_cache_hits: int = 0
    rmp_active_coefficient_cache_misses: int = 0
    rmp_active_sr_coefficient_count: int = 0
    rmp_active_sr_nonzero_count: int = 0
    rmp_active_sr_nonzero_density_max: float = 0.0
    rmp_build_time: float = 0.0
    rmp_column_insertion_time: float = 0.0
    rmp_compatibility_failure_active_sr: int = 0
    rmp_compatibility_failure_active_sr_version: int = 0
    rmp_compatibility_failure_branch_state: int = 0
    rmp_compatibility_failure_fixed_cost: int = 0
    rmp_compatibility_failure_fixed_routes: int = 0
    rmp_compatibility_failure_fleet_limit: int = 0
    rmp_compatibility_failure_objective_scale_version: int = 0
    rmp_compatibility_failure_residual_customers: int = 0
    rmp_compatibility_failure_rmp_structure_version: int = 0
    rmp_compatibility_failure_service_deadline_version: int = 0
    rmp_full_rebuilds: int = 0
    rmp_incremental_update_time: float = 0.0
    rmp_incremental_updates: int = 0
    rmp_solve_time: float = 0.0
    rmp_solves: int = 0
    rmp_time: float = 0.0
    root_closed: bool = False
    root_closure_time: int = 0
    root_compact_accepted_columns: int = 0
    root_compact_attempted: bool = False
    root_compact_objective_full: float | None = None
    root_compact_bound_full: float | None = None
    root_compact_mip_gap: float | None = None
    root_compact_node_count: float | None = None
    root_compact_iteration_count: float | None = None
    root_compact_decode_verification_time: float = 0.0
    root_compact_incumbent_validated: bool = False
    root_compact_route_paths: tuple[tuple[str, ...], ...] = ()
    root_compact_skipped_reason: str = ""
    root_compact_solve_budget_seconds: float = 0.0
    root_compact_status: str = "not_run"
    root_extracted_routes: int = 0
    root_extraction_time: float = 0.0
    root_farkas_pricing_time: float = 0.0
    root_heuristic_time: float = 0.0
    root_model_build_time: float = 0.0
    root_model_solve_time: float = 0.0
    root_rmp_is_integer: bool | None = None
    root_fractional_variable_count: int | None = None
    root_nonzero_variable_count: int | None = None
    root_max_integrality_violation: float | None = None
    root_incumbent_at_classification_shifted: float | None = None
    root_incumbent_at_classification_full: float | None = None
    root_incumbent_at_fathom_shifted: float | None = None
    root_incumbent_at_fathom_full: float | None = None
    root_fathom_reason: str | None = None
    root_branch_required: bool = False
    root_rmp_time: float = 0.0
    root_route_decode_time: float = 0.0
    root_sr_separation_time: float = 0.0
    root_standard_pricing_time: float = 0.0
    root_lower_bound_shifted: float | None = None
    root_lower_bound_full: float | None = None
    route_pool_hydration_time: float = 0.0
    route_signature_build_time: float = 0.0
    signature_cache_hits: int = 0
    signature_cache_misses: int = 0
    sr_active_count_postroot: int = 0
    sr_active_count_root: int = 0
    sr_coeff_build_time: float = 0.0
    sr_coeff_cache_hits: int = 0
    sr_coeff_cache_misses: int = 0
    sr_cuts_added: int = 0
    sr_cuts_added_postroot: int = 0
    sr_cuts_added_root: int = 0
    sr_dual_sign_violations: int = 0
    sr_separation_time: float = 0.0
    standard_pricing_calls: int = 0
    standard_pricing_time: float = 0.0
    status: str = "unknown"
    time_to_first_incumbent: float | None = None
    total_routes: int = 0
    conditioned_arc_branches: int = 0
    transformed_arcs: int = 0
    transformed_nodes: int = 0
    triplet_masks_built: int = 0
    truck_served_customers: int = 0

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
    stats = BPCStats(pricing_tolerance=solver_config.pricing_tolerance)
    _initialize_progress_history(solver_config)
    init_start = time.time()
    objective = build_objective_data(instance, weights)
    graph = build_transformed_graph(instance)
    stats.transformed_nodes = len(graph.nodes)
    stats.transformed_arcs = len(graph.arcs)
    stats.duplicate_nodes = len(graph.duplicate_nodes)
    stats.duplicate_arcs = len(graph.hub_duplicate_arcs) + len(graph.duplicate_duplicate_arcs) + len(graph.duplicate_regular_arcs)
    signature_cache = RouteSignatureCache()
    routes: dict[tuple[str, ...], Route] = {}
    incumbent = _IncumbentState()
    global_pool_paths: set[tuple[str, ...]] = set(routes)
    stats.initialization_time = time.time() - init_start
    root_start = time.time()
    routes_before_compact = set(routes)
    root_extracted = _extract_root_routes(
        instance,
        weights,
        solver_config,
        graph,
        objective,
        routes,
        stats,
        deadline,
    )
    stats.root_extraction_time = time.time() - root_start
    stats.root_extracted_routes = len(root_extracted)
    stats.root_compact_accepted_columns = len(root_extracted - routes_before_compact)
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
            _pricing_scheduler_config(solver_config),
        )
        if solver_config.pricing_worker_backend == "process" and solver_config.pricing_parallel_workers > 1
        else None
    )
    branch_index_cache: dict[tuple[tuple[str, ...], int], object] = {}

    while queue:
        if _bound_fathoms(queue[0][0], incumbent.value, solver_config.integrality_tolerance):
            stats.queue_bound_fathoms += len(queue)
            queue.clear()
            break
        if time.time() >= deadline:
            stats.status = "time_limit"
            break
        node_queue_bound, _, node = heappop(queue)
        nodes_processed += 1
        _write_progress(
            solver_config,
            stats,
            node,
            "node_started",
            {"queue_bound": node_queue_bound, "open_nodes_after_pop": len(queue)},
        )
        if _bound_fathoms(node_queue_bound, incumbent.value, solver_config.integrality_tolerance):
            stats.queue_bound_fathoms += 1
            _write_progress(
                solver_config,
                stats,
                node,
                "node_fathomed_queue_bound",
                {"queue_bound": node_queue_bound, "incumbent": incumbent.value},
            )
            continue
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
            if node.depth == 0:
                stats.root_fathom_reason = "infeasible"
            _write_progress(solver_config, stats, node, "node_fathomed_infeasible")
            continue
        node_bound, z_values = node_result
        _record_root_rmp_classification(
            node,
            node_bound,
            z_values,
            incumbent.value,
            objective,
            solver_config,
            stats,
        )
        _report_root_bound_inconsistency(node, node_bound, incumbent.value, solver_config, stats)
        if _bound_fathoms(node_bound, incumbent.value, solver_config.integrality_tolerance):
            _record_root_fathom(
                node,
                incumbent.value,
                objective,
                stats,
                "bound_matches_existing_incumbent",
            )
            _write_progress(
                solver_config,
                stats,
                node,
                "node_fathomed_bound",
                {"node_bound": node_bound, "incumbent": incumbent.value},
            )
            continue
        heuristic_start = time.time()
        try:
            heuristic = run_route_pool_heuristic(
                graph,
                objective,
                node,
                routes,
                global_pool_paths,
                solver_config,
                incumbent.value,
                deadline,
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
        _record_heuristic_diagnostics(stats, heuristic.diagnostics)
        if node.depth > 0:
            _record_postroot_heuristic_diagnostics(stats, heuristic.diagnostics)
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
            _write_progress(
                solver_config,
                stats,
                node,
                "incumbent_updated",
                {"source": "route_pool", "value": incumbent.value, "route_paths": [route.path for route in incumbent.routes]},
            )
        _report_root_bound_inconsistency(node, node_bound, incumbent.value, solver_config, stats)
        if _bound_fathoms(node_bound, incumbent.value, solver_config.integrality_tolerance):
            _record_root_fathom(
                node,
                incumbent.value,
                objective,
                stats,
                "bound_matches_route_pool_incumbent",
            )
            stats.queue_bound_fathoms += 1
            _write_progress(
                solver_config,
                stats,
                node,
                "node_fathomed_after_incumbent_update",
                {"node_bound": node_bound, "incumbent": incumbent.value},
            )
            continue
        if _is_integer(z_values, solver_config.integrality_tolerance):
            selected = tuple(routes[path] for path, value in z_values.items() if value > 0.5)
            value = sum(route.cost for route in selected) + node.fixed_cost
            if value < incumbent.value:
                incumbent.value = value
                incumbent.routes = node.fixed_routes + selected
                if stats.time_to_first_incumbent is None:
                    stats.time_to_first_incumbent = time.time() - start
                    stats.incumbent_source = "rmp_integer_solution"
                _write_progress(
                    solver_config,
                    stats,
                    node,
                    "incumbent_updated",
                    {"source": "rmp_integer", "value": incumbent.value, "route_paths": [route.path for route in incumbent.routes]},
                )
            _report_root_bound_inconsistency(node, node_bound, incumbent.value, solver_config, stats)
            _record_root_fathom(node, incumbent.value, objective, stats, "integer_rmp_solution")
            _write_progress(solver_config, stats, node, "node_fathomed_integer", {"value": value})
            continue
        if node.depth == 0:
            stats.root_branch_required = True
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
        _write_progress(
            solver_config,
            stats,
            node,
            "branch_created",
            {
                "branch_type": decision.branch_type,
                "node_bound": node_bound,
                "left_child": left.id if left is not None else None,
                "right_child": right.id if right is not None else None,
                "open_nodes_after_branch": len(queue),
            },
        )

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

def _bound_fathoms(lower_bound: float, upper_bound: float, tolerance: float) -> bool:
    return upper_bound < float("inf") and lower_bound >= upper_bound - tolerance


def _rmp_integrality_diagnostics(
    z_values: dict[tuple[str, ...], float],
    tolerance: float,
) -> tuple[bool, int, int, float]:
    violations = [abs(value - round(value)) for value in z_values.values()]
    fractional_count = sum(violation > tolerance for violation in violations)
    nonzero_count = sum(abs(value) > tolerance for value in z_values.values())
    return fractional_count == 0, fractional_count, nonzero_count, max(violations, default=0.0)


def _record_root_rmp_classification(
    node: NodeState,
    node_bound: float,
    z_values: dict[tuple[str, ...], float],
    incumbent_value: float,
    objective: ObjectiveData,
    solver_config: SolverConfig,
    stats: BPCStats,
) -> None:
    if node.depth != 0:
        return
    is_integer, fractional_count, nonzero_count, max_violation = _rmp_integrality_diagnostics(
        z_values,
        solver_config.integrality_tolerance,
    )
    stats.root_rmp_is_integer = is_integer
    stats.root_fractional_variable_count = fractional_count
    stats.root_nonzero_variable_count = nonzero_count
    stats.root_max_integrality_violation = max_violation
    if incumbent_value < float("inf"):
        stats.root_incumbent_at_classification_shifted = incumbent_value
        stats.root_incumbent_at_classification_full = objective.full_value_from_route_sum(incumbent_value)
    _write_progress(
        solver_config,
        stats,
        node,
        "root_rmp_classified",
        {
            "root_bound_shifted": node_bound,
            "root_bound_full": objective.full_value_from_route_sum(node_bound),
            "is_integer": is_integer,
            "fractional_variable_count": fractional_count,
            "nonzero_variable_count": nonzero_count,
            "max_integrality_violation": max_violation,
            "incumbent_shifted": None if incumbent_value == float("inf") else incumbent_value,
            "incumbent_full": (
                None
                if incumbent_value == float("inf")
                else objective.full_value_from_route_sum(incumbent_value)
            ),
        },
    )


def _record_root_fathom(
    node: NodeState,
    incumbent_value: float,
    objective: ObjectiveData,
    stats: BPCStats,
    reason: str,
) -> None:
    if node.depth != 0:
        return
    classification = "integral_rmp" if stats.root_rmp_is_integer else "fractional_rmp"
    stats.root_fathom_reason = f"{classification}_{reason}"
    stats.root_incumbent_at_fathom_shifted = incumbent_value
    stats.root_incumbent_at_fathom_full = objective.full_value_from_route_sum(incumbent_value)


def _validate_root_bound_order(lower_bound: float, upper_bound: float, tolerance: float) -> None:
    if upper_bound < float("inf") and lower_bound > upper_bound + tolerance:
        raise BPCBoundInconsistency(lower_bound, upper_bound, tolerance)


def _report_root_bound_inconsistency(
    node: NodeState,
    lower_bound: float,
    upper_bound: float,
    solver_config: SolverConfig,
    stats: BPCStats,
) -> None:
    if node.depth != 0:
        return
    try:
        _validate_root_bound_order(lower_bound, upper_bound, solver_config.integrality_tolerance)
    except BPCBoundInconsistency:
        stats.root_fathom_reason = "invalid_bound"
        _write_progress(
            solver_config,
            stats,
            node,
            "invalid_root_bound",
            {
                "lower_bound": lower_bound,
                "upper_bound": upper_bound,
                "tolerance": solver_config.integrality_tolerance,
            },
        )
        raise


def _relative_gap(upper_bound: float, lower_bound: float) -> float:
    if lower_bound > upper_bound:
        raise ValueError(f"lower bound exceeds upper bound: {lower_bound} > {upper_bound}")
    if upper_bound == 0.0:
        return 0.0 if lower_bound == 0.0 else float("inf")
    return (upper_bound - lower_bound) / abs(upper_bound)

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
    del incumbent
    while True:
        if time.time() >= deadline:
            stats.status = "time_limit"
            _sync_route_stats(stats, routes, global_pool_paths, signature_cache)
            _write_progress(solver_config, stats, node, "time_limit_node_unresolved")
            raise _NodeTimeLimitReached()

        _hydrate_node_from_pool(
            graph,
            node,
            routes,
            global_pool_paths,
            stats,
            signature_cache,
            solver_config,
            branch_index_cache,
        )
        _sync_route_stats(stats, routes, global_pool_paths, signature_cache)
        stats.max_node_columns = max(stats.max_node_columns, len(node.column_paths))
        stats.max_active_sr = max(stats.max_active_sr, len(node.active_sr))
        stats.max_active_sr_version = max(stats.max_active_sr_version, node.active_sr_version)

        rmp_start = time.time()
        cache_hits_before = signature_cache.stats.sr_coeff_cache_hits + signature_cache.stats.active_sr_key_cache_hits
        cache_misses_before = signature_cache.stats.sr_coeff_cache_misses + signature_cache.stats.active_sr_key_cache_misses
        rmp = RestrictedMaster(graph, node, routes, solver_config, signature_cache)
        build_elapsed = time.time() - rmp_start
        stats.rmp_build_time += build_elapsed
        if rmp.used_incremental_update:
            stats.rmp_incremental_updates += 1
            stats.rmp_incremental_update_time += build_elapsed
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
        solve_start = time.time()
        result = rmp.solve()
        stats.rmp_solve_time += time.time() - solve_start
        _record_dual_sign_diagnostics(stats, result.duals)
        _record_rmp_time(stats, node, time.time() - rmp_start)
        stats.rmp_solves += 1
        _write_progress(solver_config, stats, node, "rmp_solved", {"rmp_status": result.status})

        if result.status == GRB.INFEASIBLE:
            stats.farkas_certificate_rhs = result.farkas_rhs
            stats.farkas_certificate_max_column_activity = result.max_farkas_column_activity
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
                    use_standard_acceleration=True,
                    batch_size=solver_config.farkas_batch_size,
                    deadline=deadline,
                    parallel_workers=solver_config.pricing_parallel_workers,
                    pricing_worker_backend=solver_config.pricing_worker_backend,
                    pricing_process_pool=pricing_pool,
                    existing_routes=routes,
                    existing_column_paths=set(node.column_paths),
                    scheduler_config=_pricing_scheduler_config(solver_config),
                    epoch_context=_pricing_epoch_context(node),
                    pricing_mode="closure",
                )
            except PricingTimeLimitReached as exc:
                _record_pricing_timeout(stats, "farkas_interrupted", exc, solver_config, node, routes, global_pool_paths, signature_cache)
                raise _NodeTimeLimitReached()
            _record_farkas_pricing_time(stats, node, time.time() - pricing_start)
            stats.farkas_pricing_calls += 1
            _record_pricing_diagnostic(stats, "farkas", priced, solver_config)
            if priced.routes:
                for route in priced.routes:
                    path, added = _insert_node_column(route, routes, node, graph, stats, signature_cache, objective)
                    global_pool_paths.add(path)
                    if added:
                        stats.columns_added_farkas += 1
                continue
            if not priced.diagnostics.exact_completion:
                raise RuntimeError("Farkas pricing returned no route without certifying closure")
            _write_progress(solver_config, stats, node, "farkas_infeasible")
            return None

        if result.status != GRB.OPTIMAL or result.objective is None:
            raise RuntimeError(f"unexpected RMP status {result.status}")
        pricing_start = time.time()
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
                batch_size=solver_config.pricing_batch_size,
                deadline=deadline,
                parallel_workers=solver_config.pricing_parallel_workers,
                pricing_worker_backend=solver_config.pricing_worker_backend,
                pricing_process_pool=pricing_pool,
                existing_routes=routes,
                existing_column_paths=set(node.column_paths),
                scheduler_config=_pricing_scheduler_config(solver_config),
                epoch_context=_pricing_epoch_context(node),
                pricing_mode="closure",
            )
        except PricingTimeLimitReached as exc:
            _record_standard_pricing_time(stats, node, time.time() - pricing_start)
            stats.standard_pricing_calls += 1
            _record_pricing_timeout(stats, "standard_interrupted", exc, solver_config, node, routes, global_pool_paths, signature_cache)
            raise _NodeTimeLimitReached()
        _record_standard_pricing_time(stats, node, time.time() - pricing_start)
        stats.standard_pricing_calls += 1
        _record_pricing_diagnostic(stats, "standard", priced, solver_config)

        if priced.routes:
            added_paths: list[tuple[str, ...]] = []
            for route, reduced_cost in zip(priced.routes, priced.reduced_costs):
                if reduced_cost >= -solver_config.pricing_tolerance:
                    raise RuntimeError("standard pricing returned a nonnegative column")
                path, added = _insert_node_column(route, routes, node, graph, stats, signature_cache, objective)
                global_pool_paths.add(path)
                if not added:
                    raise RuntimeError("standard pricing returned a duplicate-equivalent entering column")
                stats.columns_added_standard += 1
                added_paths.append(path)
            _sync_route_stats(stats, routes, global_pool_paths, signature_cache)
            _write_progress(solver_config, stats, node, "standard_columns_added", {"paths": added_paths})
            continue

        if not priced.diagnostics.exact_completion:
            raise RuntimeError("standard pricing returned no route without certifying closure")

        sr_start = time.time()
        violated_activities = rmp.violated_sr_cut_activities(result.z_values, solver_config.cut_tolerance)
        selected_activities = _select_sr_cut_batch(violated_activities, solver_config.sr_cut_add_batch_size)
        _record_sr_separation_time(stats, node, time.time() - sr_start)
        if selected_activities:
            added_cuts = _activate_sr_cuts(node, selected_activities, stats)
            stats.sr_cuts_added += added_cuts
            if node.depth == 0:
                stats.sr_cuts_added_root += added_cuts
                stats.sr_active_count_root = len(node.active_sr)
            else:
                stats.sr_cuts_added_postroot += added_cuts
                stats.sr_active_count_postroot = max(stats.sr_active_count_postroot, len(node.active_sr))
            _write_progress(solver_config, stats, node, "sr_cuts_added", {"cuts": sorted(selected_activities)})
            continue

        merged_paths = _merge_duplicate_column_paths_for_node(node, routes, graph, stats, signature_cache)
        if merged_paths != node.column_paths:
            node.column_paths = merged_paths
            stats.duplicate_merge_events += 1
            continue
        if node.depth == 0:
            stats.root_lower_bound_shifted = result.objective
            stats.root_lower_bound_full = objective.full_value_from_route_sum(result.objective)
        _write_progress(solver_config, stats, node, "node_closed", {"bound": result.objective})
        if node.depth == 0:
            stats.root_closed = True
            stats.root_closure_time = time.time() - solver_start
        return result.objective, result.z_values

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
        if triplet not in node.active_sr:
            new_count += 1
            node.active_sr.add(triplet)
    if new_count:
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

def _pricing_scheduler_config(solver_config: SolverConfig) -> PricingSchedulerConfig:
    return PricingSchedulerConfig(
        customer_weight=solver_config.pricing_workload_customer_weight,
        out_degree_weight=solver_config.pricing_workload_out_degree_weight,
        drone_pad_weight=solver_config.pricing_workload_drone_pad_weight,
        deadline_weight=solver_config.pricing_workload_deadline_weight,
        split_open_labels_min=solver_config.pricing_split_open_labels_min,
        split_gap_factor=solver_config.pricing_split_gap_factor,
        split_elapsed_min=solver_config.pricing_split_elapsed_min,
        split_work_min=solver_config.pricing_split_work_min,
        refinement_depth=solver_config.pricing_dynamic_refinement_depth,
        checkpoint_extension_period=solver_config.pricing_checkpoint_extension_period,
    )

def _pricing_epoch_context(node: NodeState) -> PricingEpochContext:
    active_paths = tuple(sorted(node.column_paths))
    rmp_structure = (
        tuple(sorted(node.residual_customers)),
        tuple(route.path for route in node.fixed_routes),
        node.fleet_limit,
        node.fixed_cost,
        tuple(sorted(node.active_sr)),
        node.active_sr_version,
        active_paths,
        None if node.rmp_model_state is None else node.rmp_model_state.compatibility_key,
    )
    return PricingEpochContext(
        active_sr_version=node.active_sr_version,
        fixed_route_signature=tuple(route.path for route in node.fixed_routes),
        active_column_version=active_paths,
        rmp_structure_version=rmp_structure,
    )

def _record_pricing_diagnostic(
    stats: BPCStats,
    mode: str,
    priced: PricingResult,
    solver_config: SolverConfig,
) -> None:
    _record_pricing_diagnostic_dict(stats, mode, asdict(priced.diagnostics))

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
    _record_pricing_diagnostic_dict(stats, mode, asdict(exc.diagnostics))
    _sync_route_stats(stats, routes, global_pool_paths, signature_cache)
    _write_progress(solver_config, stats, node, "time_limit_pricing_unresolved", {"mode": mode})

def _record_pricing_diagnostic_dict(stats: BPCStats, mode: str, record: dict) -> None:
    stats.pricing_diagnostics.append({"mode": mode, **record})
    stats.pricing_labels_generated += int(record["labels_generated"])
    stats.pricing_labels_dominated += int(record["labels_dominated"])
    stats.pricing_labels_pruned += int(record["labels_pruned"])
    stats.pricing_labels_purged += int(record["labels_purged"])
    stats.pricing_stale_labels_skipped += int(record["stale_labels_skipped"])
    stats.pricing_standard_bound_pruned += int(record["standard_bound_pruned"])
    stats.pricing_farkas_bound_pruned += int(record["farkas_bound_pruned"])
    stats.pricing_max_queue_size = max(stats.pricing_max_queue_size, int(record["max_queue_size"]))
    stats.pricing_complete_routes_generated += int(record["complete_routes_generated"])
    stats.pricing_diagnostics_elapsed_seconds += float(record["elapsed_seconds"])
    stats.pricing_forward_labels_generated += int(record["forward_labels_generated"])
    stats.pricing_forward_labeling_time += float(record["forward_labeling_time_seconds"])
    stats.pricing_extensions_attempted += int(record["extensions_attempted"])
    stats.pricing_extensions_rejected_by_deadline += int(record["extensions_rejected_by_deadline"])
    stats.pricing_together_branch_reachability_pruned += int(record["together_branch_reachability_pruned"])
    stats.pricing_deadline_reachability_removed += int(record["deadline_reachability_removed"])
    stats.pricing_forward_dominance_tests += int(record["forward_dominance_tests"])
    stats.pricing_forward_same_node_dominance_tests += int(record["forward_same_node_dominance_tests"])
    stats.pricing_forward_physical_location_dominance_tests += int(
        record["forward_physical_location_dominance_tests"]
    )
    stats.pricing_forward_return_time_credit_checks += int(record["forward_return_time_credit_checks"])
    stats.pricing_forward_return_time_credit_checks_skipped += int(
        record["forward_return_time_credit_checks_skipped"]
    )
    stats.pricing_forward_branch_language_failures += int(record["forward_branch_language_failures"])
    stats.pricing_forward_branch_interface_failures += int(record["forward_branch_interface_failures"])
    stats.pricing_dom_frontier_queries += int(record["dom_frontier_queries"])
    stats.pricing_dom_frontier_keys_scanned += int(record["dom_frontier_keys_scanned"])
    stats.pricing_dom_candidate_pairs_materialized += int(record["dom_candidate_pairs_materialized"])
    stats.pricing_dom_pairs_avoided_before_materialization += int(
        record["dom_pairs_avoided_before_materialization"]
    )
    stats.pricing_dom_labels_deleted_same_node += int(record["dom_labels_deleted_same_node"])
    stats.pricing_dom_labels_deleted_physical_location += int(record["dom_labels_deleted_physical_location"])
    stats.pricing_parallel_workers_max = max(stats.pricing_parallel_workers_max, int(record["parallel_workers"]))
    stats.pricing_parallel_calls += int(bool(record["parallel_labeling_used"]))
    stats.pricing_source_neighbor_count_max = max(
        stats.pricing_source_neighbor_count_max,
        int(record["source_neighbor_count"]),
    )
    stats.pricing_source_neighbor_task_count_max = max(
        stats.pricing_source_neighbor_task_count_max,
        int(record["source_neighbor_task_count"]),
    )
    stats.pricing_certification_worker_calls += int(record["certification_worker_calls"])
    stats.pricing_productive_worker_calls += int(record["productive_worker_calls"])
    stats.pricing_negative_routes_verified += int(record["negative_routes_verified"])
    stats.pricing_negative_routes_inserted += int(record["negative_routes_inserted"])
    stats.pricing_candidate_paths_before_merge += int(record["pricing_candidate_paths_before_merge"])
    stats.pricing_candidate_paths_after_merge += int(record["pricing_candidate_paths_after_merge"])
    stats.pricing_decoded_routes_in_main += int(record["pricing_decoded_routes_in_main"])
    stats.pricing_verified_routes_in_main += int(record["pricing_verified_routes_in_main"])
    stats.pricing_pool_startup_time += float(record["pricing_pool_startup_time_seconds"])
    stats.pricing_pool_startup_count += int(record["pricing_pool_startup_count"])
    stats.pricing_pool_reused_calls += int(record["pricing_pool_reused_calls"])
    stats.pricing_task_submission_time += float(record["pricing_task_submission_time_seconds"])
    stats.pricing_process_cpu_time += float(record["process_cpu_time_seconds"])
    stats.pricing_balanced_process_dynamic_calls += int(bool(record["balanced_process_dynamic"]))
    stats.pricing_initial_load_imbalance_max_mean = max(
        stats.pricing_initial_load_imbalance_max_mean,
        float(record["initial_load_imbalance_max_mean"]),
    )
    stats.pricing_worker_busy_seconds += sum(float(value) for _, value in record["per_worker_busy_seconds"])
    stats.pricing_worker_idle_seconds += sum(float(value) for _, value in record["per_worker_idle_seconds"])
    stats.pricing_idle_work_requests += int(record["idle_work_requests"])
    stats.pricing_dynamic_split_candidates += int(record["dynamic_split_candidates"])
    stats.pricing_dynamic_splits_performed += int(record["dynamic_splits_performed"])
    stats.pricing_dynamic_split_rejected_near_closure += int(record["dynamic_split_rejected_near_closure"])
    stats.pricing_dynamic_split_rejected_small_frontier += int(record["dynamic_split_rejected_small_frontier"])
    stats.pricing_dynamic_split_rejected_elapsed += int(record["dynamic_split_rejected_elapsed"])
    stats.pricing_dynamic_split_rejected_low_work += int(record["dynamic_split_rejected_low_work"])
    stats.pricing_dynamic_child_tasks_created += int(record["dynamic_child_tasks_created"])
    stats.pricing_dynamic_labels_transferred += int(record["dynamic_labels_transferred"])
    stats.pricing_dynamic_bytes_transferred += int(record["dynamic_bytes_transferred"])
    stats.pricing_dynamic_split_control_time += float(record["dynamic_split_control_seconds"])
    stats.pricing_leaf_tasks_created += int(record["leaf_tasks_created"])
    stats.pricing_leaf_tasks_closed += int(record["leaf_tasks_closed"])
    stats.pricing_pending_transfer_peak = max(stats.pricing_pending_transfer_peak, int(record["pending_transfer_peak"]))
    stats.pricing_stale_worker_results_discarded += int(record["stale_worker_results_discarded"])
    stats.pricing_epoch_invalidations += int(record["pricing_epoch_invalidations"])
    stats.pricing_candidate_verification_time += float(record["candidate_verification_seconds"])
    stats.pricing_master_control_time += float(record["master_control_seconds"])
    stats.pricing_candidate_checkpoints += int(record["candidate_checkpoints"])
    stats.pricing_candidate_worker_resumptions += int(record["candidate_worker_resumptions"])
    stats.pricing_global_verified_candidates += int(record["global_verified_candidates"])
    stats.pricing_global_batch_limit_cancellations += int(record["global_batch_limit_cancellations"])
    stats.pricing_cpu_core_equivalent_max = max(
        stats.pricing_cpu_core_equivalent_max,
        float(record["cpu_core_equivalent"]),
    )
    if record["exact_completion"]:
        stats.pricing_certified_calls += 1
    if record["pricing_mode"] == "productive":
        stats.pricing_productive_mode_calls += 1
    else:
        stats.pricing_closure_mode_calls += 1
    best_reduced_cost = record["best_reduced_cost"]
    if best_reduced_cost is not None:
        value = float(best_reduced_cost)
        stats.best_reduced_cost_at_stop = (
            value if stats.best_reduced_cost_at_stop is None else min(stats.best_reduced_cost_at_stop, value)
        )

def _record_heuristic_diagnostics(stats: BPCStats, diagnostic) -> None:
    stats.heuristic_hard_pool_solves += diagnostic.hard_pool_solves
    stats.heuristic_hard_pool_time += diagnostic.hard_pool_time
    stats.heuristic_hard_pool_feasible_solves += diagnostic.hard_pool_feasible_solves
    stats.heuristic_full_pool_calls += diagnostic.full_pool_calls
    stats.heuristic_full_pool_time += diagnostic.full_pool_time
    stats.heuristic_full_pool_feasible += diagnostic.full_pool_feasible
    stats.heuristic_full_pool_incumbent_updates += diagnostic.full_pool_incumbent_updates
    stats.heuristic_max_node_pool_routes = max(
        stats.heuristic_max_node_pool_routes,
        diagnostic.max_node_pool_routes,
        diagnostic.node_pool_routes,
    )

def _record_postroot_heuristic_diagnostics(stats: BPCStats, diagnostic) -> None:
    stats.postroot_heuristic_hard_pool_solves += diagnostic.hard_pool_solves
    stats.postroot_heuristic_full_pool_calls += diagnostic.full_pool_calls

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
        elif reason == "rmp_structure_version":
            stats.rmp_compatibility_failure_rmp_structure_version += 1
        else:
            raise RuntimeError(f"unknown RMP compatibility failure reason {reason}")

def _record_branch_decision(stats: BPCStats, branch_type: str) -> None:
    if branch_type == "customer_pair":
        stats.customer_pair_branches += 1
    elif branch_type == "launch_pad":
        stats.launch_pad_branches += 1
    elif branch_type == "conditioned_arc":
        stats.conditioned_arc_branches += 1
    else:
        raise RuntimeError(f"unknown branch type {branch_type}")

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
        "event_sequence": stats.progress_events_seen,
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
    history_path = progress_path.with_suffix(".jsonl")
    with history_path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(record, separators=(",", ":")) + "\n")
    stats.progress_events_written += 1
    stats.progress_serialization_time += time.time() - serialization_start

def _should_write_progress_event(solver_config: SolverConfig, stats: BPCStats, event: str) -> bool:
    if solver_config.logging_mode == "audit":
        return stats.progress_events_seen % solver_config.progress_snapshot_period == 0
    major_events = {
        "time_limit_node_unresolved",
        "farkas_certificate",
        "farkas_columns_added",
        "sr_cuts_added",
        "node_closed",
        "time_limit_pricing_unresolved",
        "pricing_timeout_with_columns",
    }
    return event in major_events or (
        solver_config.progress_snapshot_period > 1
        and stats.progress_events_seen % solver_config.progress_snapshot_period == 0
    )

def _atomic_write_json(path: Path, record: dict) -> None:
    temporary_path = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary_path.write_text(json.dumps(record, indent=2), encoding="utf-8")
    temporary_path.replace(path)


def _initialize_progress_history(solver_config: SolverConfig) -> None:
    if solver_config.gurobi_log_dir is None:
        return
    progress_dir = Path(solver_config.gurobi_log_dir).parent
    progress_dir.mkdir(parents=True, exist_ok=True)
    (progress_dir / "bpc_progress.jsonl").write_text("", encoding="utf-8")

def _extract_root_routes(
    instance: InstanceData,
    weights: ObjectiveWeights,
    solver_config: SolverConfig,
    graph,
    objective: ObjectiveData,
    routes: dict[tuple[str, ...], Route],
    stats: BPCStats,
    deadline: float,
) -> set[tuple[str, ...]]:
    del deadline
    if not solver_config.enable_root_compact_warm_start:
        stats.root_compact_status = "skipped"
        stats.root_compact_skipped_reason = "disabled"
        return set()
    solve_budget = solver_config.root_compact_solve_time_limit
    stats.root_compact_solve_budget_seconds = solve_budget
    if solve_budget <= 0.0:
        raise ValueError("root compact solve time limit must be positive when the warm start is enabled")
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
    stats.root_compact_objective_full = solution.objective_full
    stats.root_compact_bound_full = solution.objective_bound_full
    stats.root_compact_mip_gap = solution.mip_gap
    stats.root_compact_node_count = solution.node_count
    stats.root_compact_iteration_count = solution.iteration_count
    stats.root_compact_route_paths = solution.route_paths
    if solution.route_paths and solution.objective_full is not None:
        validate_route_cover(solution.route_paths, graph, objective)
        stats.root_compact_incumbent_validated = True
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
        route = routes[path]
        if (
            route.served
            and route.served.issubset(node.residual_customers)
            and node.restrictions.route_allowed(route)
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
            "together": 0,
            "separate": 0,
            "launch_pad": 0,
            "conditioned_arc": 0,
        }
    else:
        index_start = time.time()
        branch_index = build_branch_route_index(candidates, routes, graph, child.residual_customers, signature_cache)
        stats.child_branch_index_build_time += time.time() - index_start
        query_start = time.time()
        indexed_candidates, branch_rejections = query_branch_route_index(branch_index, child.restrictions)
        stats.child_branch_index_query_time += time.time() - query_start
    stats.child_branch_index_candidates_after += len(indexed_candidates)
    stats.child_branch_index_reject_together += branch_rejections["together"]
    stats.child_branch_index_reject_separate += branch_rejections["separate"]
    stats.child_branch_index_reject_launch_pad += branch_rejections["launch_pad"]
    stats.child_branch_index_reject_conditioned_arc += branch_rejections["conditioned_arc"]
    accepted: set[tuple[str, ...]] = set()
    for path in sorted(indexed_candidates):
        route = routes[path]
        stats.child_inherited_route_candidates += 1
        rejection = _child_inheritance_rejection_reason(
            graph,
            objective,
            child,
            route,
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
    query_start = time.time()
    candidates, rejections = query_branch_route_index(branch_index, node.restrictions)
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
    if not child.restrictions.route_allowed(route):
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
    return {
        path
        for path in node.column_paths
        if (
            routes[path].served
            and routes[path].served.issubset(node.residual_customers)
            and node.restrictions.route_allowed(routes[path])
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

def _merge_duplicate_column_paths_for_node(
    node: NodeState,
    routes: dict[tuple[str, ...], Route],
    graph,
    stats: BPCStats | None = None,
    signature_cache: RouteSignatureCache | None = None,
) -> set[tuple[str, ...]]:
    start = time.time()
    admissible_paths = _node_admissible_column_paths(graph, node, routes)
    merged = merge_duplicate_column_paths(
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

    fractional_conditioned_arcs = []
    for customer in residual:
        customer_paths = [
            (path, value)
            for path, value in z_values.items()
            if customer in routes[path].served and value > solver_config.integrality_tolerance
        ]
        for arc in sorted(graph.arcs):
            flow = sum(value for path, value in customer_paths if arc in routes[path].used_arcs)
            if _fractional(flow, solver_config.integrality_tolerance):
                fractional_conditioned_arcs.append((abs(flow - 0.5), customer, arc))
    if fractional_conditioned_arcs:
        _, customer, arc = min(fractional_conditioned_arcs)
        return BranchDecision(
            "conditioned_arc",
            node.copy_for_child(left_id, node.restrictions.with_conditioned_arc_forbidden(customer, arc)),
            node.copy_for_child(right_id, node.restrictions.with_conditioned_arc_required(customer, arc)),
        )
    raise RuntimeError(
        "fractional RMP has no fractional customer-pair, launch-pad, or customer-conditioned transformed-arc flow"
    )

def _fractional(value: float, tolerance: float) -> bool:
    return tolerance < value < 1.0 - tolerance and abs(value - round(value)) > tolerance
