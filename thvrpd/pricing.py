from __future__ import annotations

from concurrent.futures import FIRST_COMPLETED, ProcessPoolExecutor, ThreadPoolExecutor, wait
from dataclasses import dataclass, field, replace
from heapq import heappop, heappush
from itertools import count
from math import ceil, isfinite
import multiprocessing
from threading import Event
import time

import networkx as nx

from .branching import BranchRestrictions
from .columns import (
    COST_SIGNATURE_TOLERANCE,
    PAPER_DOMINANCE_TOLERANCE,
    RouteCoefficientSignature,
    RouteSignatureCache,
    route_coefficient_signature,
    route_signature,
    route_signature_from_resources,
)
from .objective import ObjectiveData
from .routes import PAYLOAD_TOLERANCE, Route, ServiceEnvelopeViolation, is_customer_representation, route_from_path
from .transform import TransformedGraph, duplicate_customer, duplicate_hub, is_duplicate, served_customer


PRICING_STATUS_NEGATIVE_BATCH = "NEGATIVE_BATCH"
PRICING_STATUS_EXHAUSTED_NO_NEGATIVE = "EXHAUSTED_NO_NEGATIVE"
PRICING_STATUS_TIME_LIMIT_WITH_COLUMNS = "TIME_LIMIT_WITH_COLUMNS"
PRICING_STATUS_TIME_LIMIT_NO_COLUMNS = "TIME_LIMIT_NO_COLUMNS"


@dataclass(frozen=True)
class PricingDuals:
    mu: dict[str, float]
    kappa: float
    nu: dict[tuple[str, str, str], float] = field(default_factory=dict)


@dataclass(frozen=True)
class PricingResult:
    routes: tuple[Route, ...]
    reduced_costs: tuple[float, ...]
    best_route: Route | None
    best_reduced_cost: float | None
    diagnostics: "PricingDiagnostics"
    side_pool_routes: tuple[Route, ...] = tuple()
    side_pool_reduced_costs: tuple[float, ...] = tuple()

    @property
    def route(self) -> Route | None:
        return self.routes[0] if self.routes else self.best_route

    @property
    def reduced_cost(self) -> float | None:
        return self.reduced_costs[0] if self.reduced_costs else self.best_reduced_cost


@dataclass(frozen=True)
class PricingDiagnostics:
    labels_generated: int
    labels_dominated: int
    labels_pruned: int
    max_queue_size: int
    complete_routes_generated: int
    returned_routes: int
    best_reduced_cost: float | None
    exact_completion: bool
    termination_reason: str = "unspecified"
    certification_mode: str = "not_certified"
    labels_purged: int = 0
    stale_labels_skipped: int = 0
    standard_bound_pruned: int = 0
    farkas_bound_pruned: int = 0
    elapsed_seconds: float = 0.0
    pricing_engine: str = "forward_labeling"
    forward_labels_generated: int = 0
    backward_labels_generated: int = 0
    backward_dominance_tests: int = 0
    backward_labels_dominated: int = 0
    backward_cost_function_build_time_seconds: float = 0.0
    backward_cost_function_eval_time_seconds: float = 0.0
    join_sr_correction_time_seconds: float = 0.0
    join_active_block_time_seconds: float = 0.0
    joined_reduced_cost_evaluations: int = 0
    backward_dominance_cost_tests: int = 0
    backward_dominance_cost_rejected: int = 0
    backward_exclusive_resource_violations: int = 0
    join_pairs_tested: int = 0
    joined_routes_accepted: int = 0
    forward_labeling_time_seconds: float = 0.0
    backward_labeling_time_seconds: float = 0.0
    join_time_seconds: float = 0.0
    parallel_labeling_used: bool = False
    parallel_workers: int = 1
    parallel_calls: int = 0
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
    dominance_prefilter_pairs: int = 0
    dominance_prefilter_rejected: int = 0
    dominance_bucket_pairs_considered: int = 0
    dominance_bucket_pairs_rejected: int = 0
    dominance_bucket_candidate_pairs: int = 0
    dominance_compatible_keys_generated: int = 0
    dominance_compatible_key_lookups: int = 0
    dominance_bucket_scans_avoided: int = 0
    dominance_key_generation_time_seconds: float = 0.0
    backward_full_dominance_tests: int = 0
    join_prefilter_pairs: int = 0
    join_prefilter_rejected: int = 0
    join_bucket_pairs_considered: int = 0
    join_bucket_pairs_rejected: int = 0
    join_bucket_candidate_pairs: int = 0
    join_compatible_keys_generated: int = 0
    join_compatible_key_lookups: int = 0
    join_bucket_scans_avoided: int = 0
    join_key_generation_time_seconds: float = 0.0
    join_key_cache_hits: int = 0
    join_key_cache_misses: int = 0
    join_graph_build_time_seconds: float = 0.0
    join_subbucket_pairs_considered: int = 0
    join_subbucket_pairs_rejected: int = 0
    join_small_bypass_calls: int = 0
    join_local_bypass_calls: int = 0
    join_cumulative_bypass_calls: int = 0
    join_indexed_activation_count: int = 0
    join_work_estimate: int = 0
    join_candidate_pairs_accepted: int = 0
    join_activation_mode: str = "none"
    join_label_pairs_materialized: int = 0
    join_full_decodes: int = 0
    lazy_rejected_before_decode: int = 0
    fully_decoded_routes: int = 0
    duplicate_equivalent_rejected: int = 0
    cost_dominated_rejected: int = 0
    signature_build_time_seconds: float = 0.0
    sr_coeff_build_time_seconds: float = 0.0
    duplicate_lookup_time_seconds: float = 0.0
    route_decode_time_seconds: float = 0.0
    reduced_cost_verification_time_seconds: float = 0.0
    dominance_key_cache_hits: int = 0
    dominance_key_cache_misses: int = 0
    dominance_small_bypass_calls: int = 0
    dominance_bypass_calls: int = 0
    dominance_indexed_activation_count: int = 0
    dominance_work_estimate: int = 0
    dominance_activation_mode: str = "none"
    sticky_indexed_join_activations: int = 0
    sticky_indexed_dominance_activations: int = 0
    join_stage_reject_key: int = 0
    join_stage_reject_branch: int = 0
    join_stage_reject_customer: int = 0
    join_stage_reject_truck_node: int = 0
    join_stage_reject_payload: int = 0
    join_stage_reject_block: int = 0
    join_stage_reject_reduced_cost: int = 0
    dominance_stage_reject_key: int = 0
    dominance_stage_reject_branch: int = 0
    dominance_stage_reject_customer: int = 0
    dominance_stage_reject_truck_node: int = 0
    dominance_stage_reject_payload: int = 0
    dominance_stage_reject_block: int = 0
    dominance_stage_reject_time: int = 0
    dominance_stage_reject_cost: int = 0
    pricing_mode: str = "productive"
    pricing_yield_ratio: float = 0.0
    side_pool_routes_returned: int = 0
    side_pool_reduced_cost_min: float | None = None
    side_pool_candidates_seen: int = 0
    side_pool_routes_retained: int = 0
    side_pool_routes_rejected_by_budget: int = 0
    join_pairs_key_compatible: int = 0
    join_pairs_after_bitset_filters: int = 0
    join_lower_envelope_rejects: int = 0
    join_bucket_lower_envelope_rejects: int = 0
    join_subbucket_lower_envelope_rejects: int = 0
    join_pair_lower_envelope_rejects: int = 0
    join_queue_pushes: int = 0
    join_queue_pops: int = 0
    join_generator_queue_pushes: int = 0
    join_generator_queue_pops: int = 0
    join_generator_splits: int = 0
    join_materialized_pairs: int = 0
    join_exact_rc_evals: int = 0
    join_exact_rc_time_seconds: float = 0.0
    interface_cache_hits: int = 0
    interface_cache_misses: int = 0
    suffix_profile_cache_hits: int = 0
    suffix_profile_cache_misses: int = 0
    interface_profile_cache_hits: int = 0
    interface_profile_cache_misses: int = 0
    pricing_status: str = "unspecified"
    productive_calls: int = 0
    certification_calls: int = 0
    negative_routes_verified: int = 0
    negative_routes_inserted: int = 0
    pricing_worker_backend: str = "serial"
    process_cpu_time_seconds: float = 0.0
    cpu_core_equivalent: float = 0.0
    worker_id: int | None = None
    source_neighbor_count: int = 0
    source_neighbor_block_sizes: tuple[int, ...] = tuple()
    first_hit_worker_id: int | None = None
    first_hit_exits: int = 0
    interrupted_worker_calls: int = 0
    certification_worker_calls: int = 0
    productive_worker_calls: int = 0
    per_worker_elapsed_seconds: tuple[tuple[int, float], ...] = tuple()
    per_worker_cpu_time_seconds: tuple[tuple[int, float], ...] = tuple()
    per_worker_labels_generated: tuple[tuple[int, int], ...] = tuple()
    per_worker_labels_dominated: tuple[tuple[int, int], ...] = tuple()
    per_worker_labels_pruned: tuple[tuple[int, int], ...] = tuple()
    per_worker_completed_labels: tuple[tuple[int, int], ...] = tuple()
    per_worker_verified_negative_routes: tuple[tuple[int, int], ...] = tuple()
    pricing_pool_startup_time_seconds: float = 0.0
    pricing_pool_startup_count: int = 0
    pricing_pool_reused_calls: int = 0
    pricing_pool_shutdown_time_seconds: float = 0.0
    pricing_task_submission_time_seconds: float = 0.0
    pricing_worker_payload_count: int = 0
    pricing_worker_response_count: int = 0
    pricing_candidate_paths_before_merge: int = 0
    pricing_candidate_paths_after_merge: int = 0
    pricing_decoded_routes_in_main: int = 0
    pricing_verified_routes_in_main: int = 0
    pricing_batch_target: int = 0
    pricing_returned_batch_size: int = 0
    pricing_first_hit_enabled: bool = False
    pricing_stale_response_rejections: int = 0
    pricing_worker_cpu_time_seconds: float = 0.0
    pricing_main_process_cpu_time_seconds: float = 0.0
    pricing_main_merge_time_seconds: float = 0.0
    productive_slice_seconds: float = 0.0
    productive_slice_deadline_used: bool = False
    productive_time_limit_with_columns: int = 0
    productive_time_limit_no_columns: int = 0
    adaptive_slice_seconds: float = 0.0
    productive_yield_window_rate: float = 0.0
    stabilized_dual_enabled: bool = False
    stabilized_candidates_returned: int = 0
    true_dual_rejected_candidates: int = 0
    mean_worker_rc_minus_true_rc: float = 0.0
    max_abs_worker_true_rc_discrepancy: float = 0.0
    prefix_task_depth: int = 1
    source_neighbor_task_count: int = 0
    source_neighbor_task_sizes: tuple[int, ...] = tuple()
    local_worker_candidate_quota: int = 0
    diversity_quota: int = 0
    diversity_selected_routes: int = 0
    diversity_selected_customers: int = 0
    verified_candidates_by_source_neighbor: tuple[tuple[str, int], ...] = tuple()
    selected_candidates_by_source_neighbor: tuple[tuple[str, int], ...] = tuple()
    extensions_attempted: int = 0
    extensions_rejected_by_deadline: int = 0
    deadline_reachability_removed: int = 0
    reward_set_size_before_deadline: int = 0
    reward_set_size_after_deadline: int = 0
    deadline_reward_bound_calls: int = 0
    deadline_dominance_prefilter_skips: int = 0
    routes_rejected_by_deadline_in_master: int = 0
    forward_dominance_tests: int = 0
    forward_same_node_dominance_tests: int = 0
    forward_physical_location_dominance_tests: int = 0
    forward_physical_location_dominance_rejections: int = 0
    forward_return_time_credit_checks: int = 0
    forward_return_time_credit_checks_skipped: int = 0
    forward_branch_language_failures: int = 0
    forward_mask_scalar_prefilter_failures: int = 0
    dom_gate_pairs_seen: int = 0
    dom_gate_mask_failures: int = 0
    dom_gate_scalar_failures: int = 0
    dom_gate_branch_failures: int = 0
    dom_gate_deadline_failures: int = 0
    labels_dominated_same_node: int = 0
    labels_dominated_physical: int = 0
    dom_prefilter_pairs: int = 0
    dom_prefilter_mask_fail: int = 0
    dom_prefilter_branch_fail: int = 0
    dom_prefilter_payload_fail: int = 0
    dom_prefilter_block_fail: int = 0
    dom_prefilter_return_credit_fail: int = 0
    dom_full_tests: int = 0
    dom_full_rejections: int = 0
    physical_location_full_tests: int = 0
    physical_location_rejections: int = 0


class PricingTimeLimitReached(RuntimeError):
    def __init__(self, diagnostics: PricingDiagnostics) -> None:
        super().__init__("pricing label search reached the global time limit before exact completion")
        self.diagnostics = diagnostics


def _validate_inequality_dual_signs(duals: PricingDuals, tolerance: float = 1e-9) -> None:
    sign_tolerance = max(tolerance, 1e-9)
    positive_sr = {triplet: value for triplet, value in duals.nu.items() if value > sign_tolerance}
    if duals.kappa > sign_tolerance or positive_sr:
        raise ValueError(
            "pricing inequality duals must be nonpositive under the paper dual convention "
            f"(kappa={duals.kappa}, positive_sr={positive_sr})"
        )


@dataclass(frozen=True)
class _RewardItem:
    customer: str
    demand: float
    reward: float
    density: float


@dataclass(frozen=True)
class _PricingBounds:
    reward_items_by_location: dict[str, tuple[_RewardItem, ...]]


@dataclass
class _DeadlinePricingCounters:
    extensions_attempted: int = 0
    extensions_rejected_by_deadline: int = 0
    deadline_reachability_removed: int = 0
    reward_set_size_before_deadline: int = 0
    reward_set_size_after_deadline: int = 0
    deadline_reward_bound_calls: int = 0
    deadline_dominance_prefilter_skips: int = 0
    routes_rejected_by_deadline_in_master: int = 0
    forward_dominance_tests: int = 0
    forward_same_node_dominance_tests: int = 0
    forward_physical_location_dominance_tests: int = 0
    forward_physical_location_dominance_rejections: int = 0
    forward_return_time_credit_checks: int = 0
    forward_return_time_credit_checks_skipped: int = 0
    forward_branch_language_failures: int = 0
    forward_mask_scalar_prefilter_failures: int = 0
    dom_gate_pairs_seen: int = 0
    dom_gate_mask_failures: int = 0
    dom_gate_scalar_failures: int = 0
    dom_gate_branch_failures: int = 0
    dom_gate_deadline_failures: int = 0
    labels_dominated_same_node: int = 0
    labels_dominated_physical: int = 0
    dom_prefilter_pairs: int = 0
    dom_prefilter_mask_fail: int = 0
    dom_prefilter_branch_fail: int = 0
    dom_prefilter_payload_fail: int = 0
    dom_prefilter_block_fail: int = 0
    dom_prefilter_return_credit_fail: int = 0
    dom_full_tests: int = 0
    dom_full_rejections: int = 0
    physical_location_full_tests: int = 0
    physical_location_rejections: int = 0
    shortest: dict[tuple[str, str], float] | None = None
    reachable_mask_cache: dict[_Label, int] = field(default_factory=dict)


@dataclass(frozen=True)
class _Label:
    path: tuple[str, ...]
    represented: frozenset[str]
    truck_visited: frozenset[str]
    truck_load: float
    active_pad: str | None
    active_pad_arrival: float
    active_wait: float
    block_count: int
    physical_time: float
    service_times: tuple[tuple[str, float], ...]
    sr_counts: tuple[tuple[tuple[str, str, str], int], ...]
    reduced_cost: float
    used_arcs: frozenset[tuple[str, str]]
    truck_served: frozenset[str] = frozenset()
    pad_served: frozenset[tuple[str, str]] = frozenset()
    represented_mask: int = 0
    truck_node_mask: int = 0
    branch_state: "_BranchState | None" = None


@dataclass(frozen=True)
class _WorkerPricingTask:
    call_id: int
    dual_id: int
    worker_id: int
    source_neighbor_block: tuple[str, ...]
    residual_customers: frozenset[str]
    restrictions: BranchRestrictions
    duals: PricingDuals
    next_route_id: int
    farkas: bool
    pricing_tolerance: float
    use_standard_acceleration: bool
    stop_at_first_negative: bool
    batch_size: int
    deadline: float | None
    known_signature_costs: dict[RouteCoefficientSignature, float]
    pricing_mode: str
    pricing_yield_ratio: float
    stop_event: object | None
    pricing_worker_backend: str
    source_neighbor_count: int
    source_neighbor_block_sizes: tuple[int, ...]
    parallel_workers: int
    source_neighbor_task_count: int
    source_neighbor_task_sizes: tuple[int, ...]
    local_worker_candidate_quota: int
    source_prefixes: tuple[tuple[str, ...], ...] = tuple()


@dataclass(frozen=True)
class _WorkerPricingResult:
    call_id: int
    dual_id: int
    worker_id: int
    route_paths: tuple[tuple[str, ...], ...]
    reduced_costs: tuple[float, ...]
    best_path: tuple[str, ...] | None
    best_reduced_cost: float | None
    diagnostics: PricingDiagnostics


_PROCESS_WORKER_GRAPH: TransformedGraph | None = None
_PROCESS_WORKER_OBJECTIVE: ObjectiveData | None = None


def _initialize_source_neighbor_process_worker(graph: TransformedGraph, objective: ObjectiveData) -> None:
    global _PROCESS_WORKER_GRAPH, _PROCESS_WORKER_OBJECTIVE
    _PROCESS_WORKER_GRAPH = graph
    _PROCESS_WORKER_OBJECTIVE = objective


def _process_worker_ready() -> int:
    if _PROCESS_WORKER_GRAPH is None or _PROCESS_WORKER_OBJECTIVE is None:
        raise RuntimeError("persistent pricing process worker was not initialized")
    return 1


class SourceNeighborPricingPool:
    def __init__(
        self,
        graph: TransformedGraph,
        objective: ObjectiveData,
        parallel_workers: int,
        source_neighbor_task_size: int = 1,
    ) -> None:
        if parallel_workers <= 0:
            raise ValueError("parallel_workers must be positive")
        if source_neighbor_task_size <= 0:
            raise ValueError("source_neighbor_task_size must be positive")
        self.graph = graph
        self.objective = objective
        self.source_neighbors = _admissible_source_neighbors(graph)
        self.parallel_workers = min(parallel_workers, max(len(self.source_neighbors), 1))
        self.source_neighbor_task_size = source_neighbor_task_size
        self.call_counter = count(1)
        self.startup_count = 0
        self.reused_calls = 0
        self.shutdown_time_seconds = 0.0
        self.shutdown_count = 0
        self._manager = multiprocessing.Manager()
        startup = time.time()
        self._executor = ProcessPoolExecutor(
            max_workers=self.parallel_workers,
            initializer=_initialize_source_neighbor_process_worker,
            initargs=(graph, objective),
        )
        ready = [self._executor.submit(_process_worker_ready) for _ in range(self.parallel_workers)]
        for future in ready:
            future.result()
        self.startup_time_seconds = time.time() - startup
        self.startup_count = 1
        self._closed = False

    def stop_event(self):
        return self._manager.Event()

    def shutdown(self) -> None:
        if self._closed:
            return
        start = time.time()
        self._executor.shutdown(wait=True)
        self._manager.shutdown()
        self.shutdown_time_seconds += time.time() - start
        self.shutdown_count += 1
        self._closed = True

    def price(
        self,
        *,
        residual_customers: frozenset[str],
        restrictions: BranchRestrictions,
        duals: PricingDuals,
        next_route_id: int,
        farkas: bool,
        pricing_tolerance: float,
        use_standard_acceleration: bool,
        stop_at_first_negative: bool,
        batch_size: int,
        deadline: float | None,
        existing_routes: dict[tuple[str, ...], Route] | None,
        existing_column_paths: set[tuple[str, ...]] | None,
        pricing_mode: str,
        pricing_yield_ratio: float,
        productive_candidate_multiplier: float = 1.5,
        pricing_diversity_batch_fraction: float = 0.5,
        source_neighbor_task_size: int | None = None,
        productive_slice_seconds: float = 0.0,
        productive_slice_deadline_used: bool = False,
        adaptive_slice_seconds: float = 0.0,
        productive_yield_window_rate: float = 0.0,
        search_duals: PricingDuals | None = None,
        prefix_task_depth: int = 1,
    ) -> PricingResult:
        if self._closed:
            raise RuntimeError("persistent pricing process pool is closed")
        if productive_candidate_multiplier <= 0:
            raise ValueError("productive_candidate_multiplier must be positive")
        if not 0.0 <= pricing_diversity_batch_fraction <= 1.0:
            raise ValueError("pricing_diversity_batch_fraction must be in [0, 1]")
        call_start_time = time.time()
        call_id = next(self.call_counter)
        dual_id = call_id
        worker_duals = search_duals or duals
        stabilized_dual_enabled = search_duals is not None
        self.reused_calls += 1
        known_signature_costs = _known_signature_costs(
            existing_routes,
            existing_column_paths,
            self.graph,
            residual_customers,
            RouteSignatureCache(),
        )
        first_hit_enabled = stop_at_first_negative or batch_size == 1
        task_size = self.source_neighbor_task_size if source_neighbor_task_size is None else source_neighbor_task_size
        if prefix_task_depth <= 1:
            task_blocks = _chunk_source_neighbors(self.source_neighbors, task_size)
            prefix_task_blocks: tuple[tuple[tuple[str, ...], ...], ...] = tuple(tuple() for _ in task_blocks)
            task_size_units = tuple(len(block_item) for block_item in task_blocks)
        else:
            prefixes = _source_neighbor_prefix_tasks(
                self.graph,
                self.objective,
                residual_customers,
                restrictions,
                prefix_task_depth,
            )
            prefix_task_blocks = _chunk_prefix_tasks(prefixes, task_size)
            task_blocks = tuple(
                tuple(sorted({prefix[0] for prefix in block if prefix}))
                for block in prefix_task_blocks
            )
            task_size_units = tuple(len(block_item) for block_item in prefix_task_blocks)
        local_candidate_quota = (
            1
            if first_hit_enabled
            else max(1, ceil(batch_size * productive_candidate_multiplier / max(len(task_blocks), 1)))
        )
        worker_batch_size = 1 if first_hit_enabled else local_candidate_quota
        stop_event = self.stop_event() if first_hit_enabled else None
        tasks = [
            _WorkerPricingTask(
                call_id=call_id,
                dual_id=dual_id,
                worker_id=task_id,
                source_neighbor_block=block,
                residual_customers=residual_customers,
                restrictions=restrictions,
                duals=worker_duals,
                next_route_id=next_route_id,
                farkas=farkas,
                pricing_tolerance=pricing_tolerance,
                use_standard_acceleration=use_standard_acceleration,
                stop_at_first_negative=first_hit_enabled,
                batch_size=worker_batch_size,
                deadline=deadline,
                known_signature_costs=known_signature_costs,
                pricing_mode=pricing_mode,
                pricing_yield_ratio=pricing_yield_ratio,
                stop_event=stop_event,
                pricing_worker_backend="process",
                source_neighbor_count=len(self.source_neighbors),
                source_neighbor_block_sizes=tuple(len(block_item) for block_item in task_blocks),
                parallel_workers=self.parallel_workers,
                source_neighbor_task_count=len(task_blocks),
                source_neighbor_task_sizes=task_size_units,
                local_worker_candidate_quota=local_candidate_quota,
                source_prefixes=prefix_task_blocks[task_id],
            )
            for task_id, block in enumerate(task_blocks)
        ]
        submit_start = time.time()
        futures = [self._executor.submit(_run_persistent_forward_pricing_worker, task) for task in tasks]
        submission_time = time.time() - submit_start
        results: list[_WorkerPricingResult] = []
        selected: _WorkerPricingResult | None = None
        if first_hit_enabled:
            pending = set(futures)
            while pending:
                done, pending = wait(pending, return_when=FIRST_COMPLETED)
                for future in sorted(done, key=lambda item: id(item)):
                    result = future.result()
                    results.append(result)
                    if result.route_paths:
                        selected = result
                        stop_event.set()
                        if pending:
                            done_remaining, pending = wait(pending)
                            results.extend(future_item.result() for future_item in done_remaining)
                        return _merge_compact_worker_results(
                            results,
                            self.graph,
                            self.objective,
                            residual_customers,
                            restrictions,
                            duals,
                            next_route_id,
                            farkas,
                            pricing_tolerance,
                            existing_routes,
                            existing_column_paths,
                            pricing_mode,
                            pricing_yield_ratio,
                            "process",
                            len(self.source_neighbors),
                            task_size_units,
                            self.parallel_workers,
                            selected=selected,
                            force_time_limit=False,
                            call_id=call_id,
                            dual_id=dual_id,
                            submission_time_seconds=submission_time,
                            pool_startup_time_seconds=self.startup_time_seconds if call_id == 1 else 0.0,
                            pool_startup_count=1 if call_id == 1 else 0,
                            pool_reused_calls=1,
                            first_hit_enabled=first_hit_enabled,
                            batch_target=batch_size,
                            productive_slice_seconds=productive_slice_seconds,
                            productive_slice_deadline_used=productive_slice_deadline_used,
                            adaptive_slice_seconds=adaptive_slice_seconds,
                            productive_yield_window_rate=productive_yield_window_rate,
                            stabilized_dual_enabled=stabilized_dual_enabled,
                            prefix_task_depth=prefix_task_depth,
                            pricing_diversity_batch_fraction=pricing_diversity_batch_fraction,
                            worker_reduced_costs_are_true=not stabilized_dual_enabled,
                            call_start_time=call_start_time,
                        )
        else:
            for future in futures:
                results.append(future.result())
        has_time_limit = any(result.diagnostics.pricing_status == PRICING_STATUS_TIME_LIMIT_NO_COLUMNS for result in results)
        merged = _merge_compact_worker_results(
            results,
            self.graph,
            self.objective,
            residual_customers,
            restrictions,
            duals,
            next_route_id,
            farkas,
            pricing_tolerance,
            existing_routes,
            existing_column_paths,
            pricing_mode,
            pricing_yield_ratio,
            "process",
            len(self.source_neighbors),
            task_size_units,
            self.parallel_workers,
            selected=None,
            force_time_limit=has_time_limit,
            call_id=call_id,
            dual_id=dual_id,
            submission_time_seconds=submission_time,
            pool_startup_time_seconds=self.startup_time_seconds if call_id == 1 else 0.0,
            pool_startup_count=1 if call_id == 1 else 0,
            pool_reused_calls=1,
            first_hit_enabled=first_hit_enabled,
            batch_target=batch_size,
            productive_slice_seconds=productive_slice_seconds,
            productive_slice_deadline_used=productive_slice_deadline_used,
            adaptive_slice_seconds=adaptive_slice_seconds,
            productive_yield_window_rate=productive_yield_window_rate,
            stabilized_dual_enabled=stabilized_dual_enabled,
            prefix_task_depth=prefix_task_depth,
            pricing_diversity_batch_fraction=pricing_diversity_batch_fraction,
            worker_reduced_costs_are_true=not stabilized_dual_enabled,
            call_start_time=call_start_time,
        )
        if has_time_limit and not merged.routes:
            raise PricingTimeLimitReached(merged.diagnostics)
        return merged


@dataclass(frozen=True)
class _TimeExpr:
    base: str
    offset: float
    wait_floor: float = 0.0


@dataclass(frozen=True)
class _BackwardProfile:
    service_times: tuple[tuple[str, _TimeExpr], ...]
    return_time: _TimeExpr
    drone_sorties: int


@dataclass(frozen=True)
class _BackwardInterface:
    physical_time: float
    pad_arrival: float
    active_wait: float


@dataclass(frozen=True)
class _BackwardCostFunction:
    profile: _BackwardProfile
    represented: frozenset[str]
    sr_counts: tuple[tuple[tuple[str, str, str], int], ...]

    def evaluate(
        self,
        interface: _BackwardInterface,
        graph: TransformedGraph,
        objective: ObjectiveData,
        duals: PricingDuals,
        farkas: bool,
    ) -> float:
        if farkas:
            return (
                -sum(duals.mu[customer] for customer in self.represented)
                - _suffix_sr_contribution(dict(self.sr_counts), duals)
            )
        value = objective.coeffs.cost * graph.instance.drone_cost * self.profile.drone_sorties
        for customer, expr in self.profile.service_times:
            service_time = _evaluate_time_expr_at_interface(expr, interface)
            value += objective.coeffs.delay * (service_time - objective.bounds.arrival_lb[customer]) ** 2
            value -= duals.mu[customer]
        value += objective.coeffs.return_time * _evaluate_time_expr_at_interface(self.profile.return_time, interface)
        value -= _suffix_sr_contribution(dict(self.sr_counts), duals)
        return value


@dataclass(frozen=True)
class _BackwardCostLowerEnvelope:
    represented: frozenset[str]

    def evaluate(self, duals: PricingDuals, farkas: bool) -> float:
        if farkas:
            raise ValueError("standard-pricing backward lower envelope is disabled in Farkas pricing")
        return -sum(max(duals.mu[customer], 0.0) for customer in self.represented)


@dataclass(frozen=True)
class _JoinEvalCacheKey:
    backward_path: tuple[str, ...]
    meet_node: str
    physical_time: float
    pad_arrival: float
    active_wait: float
    active_sr_version: int
    dual_solution_key: tuple[object, ...]
    farkas: bool
    prefix_sr_counts: tuple[tuple[tuple[str, str, str], int], ...]
    suffix_sr_counts: tuple[tuple[tuple[str, str, str], int], ...]


@dataclass
class _JoinEvalCache:
    values: dict[_JoinEvalCacheKey, tuple[float, float]] = field(default_factory=dict)
    hits: int = 0
    misses: int = 0
    suffix_profile_hits: int = 0
    suffix_profile_misses: int = 0
    interface_profile_hits: int = 0
    interface_profile_misses: int = 0


@dataclass(frozen=True)
class _BackwardLabel:
    path: tuple[str, ...]
    represented: frozenset[str]
    truck_visited: frozenset[str]
    truck_load: float
    used_arcs: frozenset[tuple[str, str]]
    truck_served: frozenset[str] = frozenset()
    pad_served: frozenset[tuple[str, str]] = frozenset()
    sr_counts: tuple[tuple[tuple[str, str, str], int], ...] = tuple()
    branch_state: _BranchState | None = None
    profile: _BackwardProfile | None = None
    cost_function: _BackwardCostFunction | None = None
    lower_envelope: _BackwardCostLowerEnvelope | None = None
    leading_block_hub: str | None = None
    leading_block_count: int = 0
    leading_block_wait: float = 0.0
    represented_mask: int = 0
    truck_node_mask: int = 0


@dataclass(frozen=True)
class _BranchState:
    together: tuple[tuple[bool, bool], ...]
    required_arcs: tuple[tuple[tuple[str, str], bool, bool], ...]


def _mask_for_values(values: frozenset[str] | set[str], universe: tuple[str, ...]) -> int:
    bit_by_value = {value: 1 << index for index, value in enumerate(universe)}
    mask = 0
    for value in values:
        mask |= bit_by_value.get(value, 0)
    return mask


def _customer_mask(values: frozenset[str] | set[str], graph: TransformedGraph) -> int:
    return _mask_for_values(values, tuple(graph.instance.customers))


def _truck_node_mask(values: frozenset[str] | set[str], graph: TransformedGraph) -> int:
    instance = graph.instance
    return _mask_for_values(values, (instance.depot_source, *instance.hubs, *instance.customers, instance.depot_sink))


@dataclass(frozen=True)
class _ForwardExpansion:
    labels: tuple[_Label, ...]
    complete_labels: tuple[_Label, ...]
    generated: int
    elapsed_seconds: float


@dataclass(frozen=True)
class _BackwardExpansion:
    labels: tuple[_BackwardLabel, ...]
    generated: int
    elapsed_seconds: float
    cost_function_build_time_seconds: float = 0.0
    exclusive_resource_violations: int = 0


@dataclass
class _DominanceCounter:
    prefilter_pairs: int = 0
    prefilter_rejected: int = 0
    full_tests: int = 0
    bucket_pairs_considered: int = 0
    bucket_pairs_rejected: int = 0
    bucket_candidate_pairs: int = 0
    compatible_keys_generated: int = 0
    compatible_key_lookups: int = 0
    bucket_scans_avoided: int = 0
    key_generation_time_seconds: float = 0.0
    key_cache_hits: int = 0
    key_cache_misses: int = 0
    small_bypass_calls: int = 0
    indexed_activation_count: int = 0
    work_estimate: int = 0
    sticky_indexed: bool = False
    sticky_indexed_activations: int = 0
    stage_reject_key: int = 0
    stage_reject_branch: int = 0
    stage_reject_customer: int = 0
    stage_reject_truck_node: int = 0
    stage_reject_payload: int = 0
    stage_reject_block: int = 0
    stage_reject_time: int = 0
    stage_reject_cost: int = 0
    cost_function_tests: int = 0
    cost_function_rejected: int = 0


@dataclass(frozen=True)
class _RouteCandidate:
    path: tuple[str, ...]
    reduced_cost: float
    joined: bool
    served: frozenset[str]
    truck_served: frozenset[str]
    pad_served: frozenset[tuple[str, str]]
    used_arcs: frozenset[tuple[str, str]]


@dataclass
class _JoinStageCounter:
    reject_key: int = 0
    reject_branch: int = 0
    reject_customer: int = 0
    reject_truck_node: int = 0
    reject_payload: int = 0
    reject_block: int = 0
    reject_reduced_cost: int = 0
    sticky_indexed_activations: int = 0


@dataclass(frozen=True)
class _JoinBucketKey:
    meet_node: str
    physical_loc: str
    active_pad: str | None
    block_pos_class: int
    branch_state_hash: int
    payload_bucket: int


@dataclass(frozen=True)
class _JoinLookupKey:
    meet_node: str
    physical_loc: str
    active_pad: str | None
    block_pos_class: int


@dataclass
class _JoinBucket:
    key: _JoinBucketKey
    labels: list[_Label] | list[_BackwardLabel]
    labels_by_payload: list[_Label] | list[_BackwardLabel]
    union_customer_set: frozenset[str]
    intersection_customer_set: frozenset[str]
    union_truck_node_set: frozenset[str]
    intersection_truck_node_set: frozenset[str]
    min_payload: float
    max_payload: float


@dataclass(frozen=True)
class _JoinSubbucketKey:
    payload_bin: int
    active_pad: str | None
    block_pos_class: int
    branch_state_hash: int
    physical_mask_class: int


@dataclass
class _JoinSubbucket:
    key: _JoinSubbucketKey
    labels: list[_Label] | list[_BackwardLabel]
    labels_by_payload: list[_Label] | list[_BackwardLabel]
    min_payload: float
    max_payload: float


@dataclass(frozen=True)
class _JoinGenerator:
    level: str
    lower_bound: float
    forward_labels: tuple[_Label, ...]
    backward_labels: tuple[_BackwardLabel, ...]
    estimated_pair_count: int


@dataclass(frozen=True)
class _DominanceBucketKey:
    end_node: str
    physical_loc: str
    active_pad: str | None
    block_pos: int
    branch_state_hash: int


def price_route(
    graph: TransformedGraph,
    objective: ObjectiveData,
    residual_customers: frozenset[str],
    restrictions: BranchRestrictions,
    duals: PricingDuals,
    next_route_id: int,
    farkas: bool = False,
    pricing_tolerance: float = 0.0,
    use_standard_acceleration: bool = True,
    stop_at_first_negative: bool = False,
    batch_size: int = 1,
    deadline: float | None = None,
    enable_bidirectional: bool = True,
    parallel_workers: int = 2,
    existing_routes: dict[tuple[str, ...], Route] | None = None,
    existing_column_paths: set[tuple[str, ...]] | None = None,
    small_join_pair_threshold: int = 5_000,
    small_join_cumulative_threshold: int = 250_000,
    max_join_bypass_calls: int = 1_000,
    small_dom_bucket_threshold: int = 100,
    small_dom_cumulative_threshold: int = 500_000,
    max_dom_bypass_calls: int = 2_000,
    join_payload_bin_width: float = 1.0,
    side_pool_batch_size: int = 0,
    pricing_mode: str = "productive",
    pricing_yield_ratio: float = 0.0,
    join_eval_budget: int = 0,
    pricing_certification_slice_seconds: float = 0.0,
    enable_join_lower_envelope: bool = True,
    join_generator_split_threshold: int = 50_000,
    join_generator_pair_batch_size: int = 10_000,
    enable_bucket_join_envelope: bool = True,
    enable_join_profile_cache: bool = True,
    pricing_worker_backend: str = "thread",
    pricing_process_pool: SourceNeighborPricingPool | None = None,
    productive_candidate_multiplier: float = 1.5,
    source_neighbor_task_size: int = 1,
    pricing_diversity_batch_fraction: float = 0.5,
    productive_slice_seconds: float = 0.0,
    productive_slice_deadline_used: bool = False,
    adaptive_slice_seconds: float = 0.0,
    productive_yield_window_rate: float = 0.0,
    search_duals: PricingDuals | None = None,
    prefix_task_depth: int = 1,
) -> PricingResult:
    if parallel_workers <= 0:
        raise ValueError("parallel_workers must be positive")
    if pricing_worker_backend not in {"thread", "process"}:
        raise ValueError("pricing_worker_backend must be 'thread' or 'process'")
    if search_duals is not None and pricing_worker_backend != "process":
        raise ValueError("dual-stabilized productive search requires the process pricing backend")
    if pricing_worker_backend == "process" and pricing_process_pool is not None:
        return pricing_process_pool.price(
            residual_customers=residual_customers,
            restrictions=restrictions,
            duals=duals,
            next_route_id=next_route_id,
            farkas=farkas,
            pricing_tolerance=pricing_tolerance,
            use_standard_acceleration=use_standard_acceleration,
            stop_at_first_negative=stop_at_first_negative,
            batch_size=batch_size,
            deadline=deadline,
            existing_routes=existing_routes,
            existing_column_paths=existing_column_paths,
            pricing_mode=pricing_mode,
            pricing_yield_ratio=pricing_yield_ratio,
            productive_candidate_multiplier=productive_candidate_multiplier,
            pricing_diversity_batch_fraction=pricing_diversity_batch_fraction,
            source_neighbor_task_size=source_neighbor_task_size,
            productive_slice_seconds=productive_slice_seconds,
            productive_slice_deadline_used=productive_slice_deadline_used,
            adaptive_slice_seconds=adaptive_slice_seconds,
            productive_yield_window_rate=productive_yield_window_rate,
            search_duals=search_duals,
            prefix_task_depth=prefix_task_depth,
        )
    if pricing_worker_backend == "process" and parallel_workers > 1:
        temporary_pool = SourceNeighborPricingPool(graph, objective, parallel_workers, source_neighbor_task_size)
        try:
            return temporary_pool.price(
                residual_customers=residual_customers,
                restrictions=restrictions,
                duals=duals,
                next_route_id=next_route_id,
                farkas=farkas,
                pricing_tolerance=pricing_tolerance,
                use_standard_acceleration=use_standard_acceleration,
                stop_at_first_negative=stop_at_first_negative,
                batch_size=batch_size,
                deadline=deadline,
                existing_routes=existing_routes,
                existing_column_paths=existing_column_paths,
                pricing_mode=pricing_mode,
                pricing_yield_ratio=pricing_yield_ratio,
                productive_candidate_multiplier=productive_candidate_multiplier,
                pricing_diversity_batch_fraction=pricing_diversity_batch_fraction,
                source_neighbor_task_size=source_neighbor_task_size,
                productive_slice_seconds=productive_slice_seconds,
                productive_slice_deadline_used=productive_slice_deadline_used,
                adaptive_slice_seconds=adaptive_slice_seconds,
                productive_yield_window_rate=productive_yield_window_rate,
                search_duals=search_duals,
                prefix_task_depth=prefix_task_depth,
            )
        finally:
            temporary_pool.shutdown()
    if parallel_workers > 1:
        return _price_route_source_neighbor_parallel(
            graph=graph,
            objective=objective,
            residual_customers=residual_customers,
            restrictions=restrictions,
            duals=duals,
            next_route_id=next_route_id,
            farkas=farkas,
            pricing_tolerance=pricing_tolerance,
            use_standard_acceleration=use_standard_acceleration,
            stop_at_first_negative=stop_at_first_negative,
            batch_size=batch_size,
            deadline=deadline,
            existing_routes=existing_routes,
            existing_column_paths=existing_column_paths,
            side_pool_batch_size=side_pool_batch_size,
            pricing_mode=pricing_mode,
            pricing_yield_ratio=pricing_yield_ratio,
            parallel_workers=parallel_workers,
            pricing_worker_backend=pricing_worker_backend,
        )
    cpu_start = time.process_time()
    result = _price_route_forward_only(
        graph=graph,
        objective=objective,
        residual_customers=residual_customers,
        restrictions=restrictions,
        duals=duals,
        next_route_id=next_route_id,
        farkas=farkas,
        pricing_tolerance=pricing_tolerance,
        use_standard_acceleration=use_standard_acceleration,
        stop_at_first_negative=stop_at_first_negative,
        batch_size=batch_size,
        deadline=deadline,
        existing_routes=existing_routes,
        existing_column_paths=existing_column_paths,
        pricing_mode=pricing_mode,
        pricing_yield_ratio=pricing_yield_ratio,
    )
    result = replace(
        result,
        diagnostics=replace(
            result.diagnostics,
            adaptive_slice_seconds=adaptive_slice_seconds,
            productive_yield_window_rate=productive_yield_window_rate,
            prefix_task_depth=prefix_task_depth,
        ),
    )
    return _with_runtime_diagnostics(
        result,
        pricing_worker_backend="serial" if pricing_worker_backend == "thread" else pricing_worker_backend,
        process_cpu_time_seconds=time.process_time() - cpu_start,
        worker_id=None,
        source_neighbor_count=len(_admissible_source_neighbors(graph)),
        source_neighbor_block_sizes=(len(_admissible_source_neighbors(graph)),),
        parallel_workers=1,
    )


def run_source_neighbor_parallel_forward_pricing(
    graph: TransformedGraph,
    objective: ObjectiveData,
    residual_customers: frozenset[str],
    restrictions: BranchRestrictions,
    duals: PricingDuals,
    next_route_id: int,
    *,
    farkas: bool = False,
    pricing_tolerance: float = 0.0,
    use_standard_acceleration: bool = True,
    batch_size: int = 1,
    deadline: float | None = None,
    existing_routes: dict[tuple[str, ...], Route] | None = None,
    existing_column_paths: set[tuple[str, ...]] | None = None,
    side_pool_batch_size: int = 0,
    pricing_mode: str = "productive",
    pricing_yield_ratio: float = 0.0,
    parallel_workers: int = 2,
    pricing_worker_backend: str = "thread",
    adaptive_slice_seconds: float = 0.0,
    productive_yield_window_rate: float = 0.0,
    prefix_task_depth: int = 1,
) -> PricingResult:
    return _price_route_source_neighbor_parallel(
        graph=graph,
        objective=objective,
        residual_customers=residual_customers,
        restrictions=restrictions,
        duals=duals,
        next_route_id=next_route_id,
        farkas=farkas,
        pricing_tolerance=pricing_tolerance,
        use_standard_acceleration=use_standard_acceleration,
        stop_at_first_negative=True,
        batch_size=batch_size,
        deadline=deadline,
        existing_routes=existing_routes,
        existing_column_paths=existing_column_paths,
        side_pool_batch_size=side_pool_batch_size,
        pricing_mode=pricing_mode,
        pricing_yield_ratio=pricing_yield_ratio,
        parallel_workers=parallel_workers,
        pricing_worker_backend=pricing_worker_backend,
        adaptive_slice_seconds=adaptive_slice_seconds,
        productive_yield_window_rate=productive_yield_window_rate,
        prefix_task_depth=prefix_task_depth,
    )


def _price_route_source_neighbor_parallel(
    graph: TransformedGraph,
    objective: ObjectiveData,
    residual_customers: frozenset[str],
    restrictions: BranchRestrictions,
    duals: PricingDuals,
    next_route_id: int,
    farkas: bool,
    pricing_tolerance: float,
    use_standard_acceleration: bool,
    stop_at_first_negative: bool,
    batch_size: int,
    deadline: float | None,
    existing_routes: dict[tuple[str, ...], Route] | None,
    existing_column_paths: set[tuple[str, ...]] | None,
    side_pool_batch_size: int,
    pricing_mode: str,
    pricing_yield_ratio: float,
    parallel_workers: int,
    pricing_worker_backend: str,
    adaptive_slice_seconds: float = 0.0,
    productive_yield_window_rate: float = 0.0,
    prefix_task_depth: int = 1,
) -> PricingResult:
    if pricing_worker_backend not in {"thread", "process"}:
        raise ValueError("pricing_worker_backend must be 'thread' or 'process'")
    source_neighbors = _admissible_source_neighbors(graph)
    blocks = _partition_source_neighbors(source_neighbors, parallel_workers)
    if len(blocks) <= 1:
        cpu_start = time.process_time()
        result = _price_route_forward_only(
            graph=graph,
            objective=objective,
            residual_customers=residual_customers,
            restrictions=restrictions,
            duals=duals,
            next_route_id=next_route_id,
            farkas=farkas,
            pricing_tolerance=pricing_tolerance,
            use_standard_acceleration=use_standard_acceleration,
            stop_at_first_negative=stop_at_first_negative,
            batch_size=batch_size,
            deadline=deadline,
            existing_routes=existing_routes,
            existing_column_paths=existing_column_paths,
            pricing_mode=pricing_mode,
            pricing_yield_ratio=pricing_yield_ratio,
            source_neighbor_block=blocks[0] if blocks else tuple(),
            worker_id=0,
        )
        result = replace(
            result,
            diagnostics=replace(
                result.diagnostics,
                adaptive_slice_seconds=adaptive_slice_seconds,
                productive_yield_window_rate=productive_yield_window_rate,
                prefix_task_depth=prefix_task_depth,
            ),
        )
        return _with_runtime_diagnostics(
            result,
            pricing_worker_backend="serial",
            process_cpu_time_seconds=time.process_time() - cpu_start,
            worker_id=0,
            source_neighbor_count=len(source_neighbors),
            source_neighbor_block_sizes=tuple(len(block) for block in blocks),
            parallel_workers=1,
        )

    manager = None
    stop_event = Event()
    executor_class = ThreadPoolExecutor
    if pricing_worker_backend == "process":
        manager = multiprocessing.Manager()
        stop_event = manager.Event()
        executor_class = ProcessPoolExecutor
    first_hit_enabled = pricing_worker_backend == "thread" or stop_at_first_negative or batch_size == 1
    worker_batch_size = 1 if first_hit_enabled else batch_size
    known_signature_costs = _known_signature_costs(
        existing_routes,
        existing_column_paths,
        graph,
        residual_customers,
        RouteSignatureCache(),
    )
    worker_args = [
        (
            worker_id,
            block,
            graph,
            objective,
            residual_customers,
            restrictions,
            duals,
            next_route_id,
            farkas,
            pricing_tolerance,
            use_standard_acceleration,
            first_hit_enabled,
            worker_batch_size,
            deadline,
            existing_routes,
            existing_column_paths,
            pricing_mode,
            pricing_yield_ratio,
            stop_event,
            pricing_worker_backend,
            len(source_neighbors),
            tuple(len(item) for item in blocks),
            len(blocks),
            known_signature_costs,
        )
        for worker_id, block in enumerate(blocks)
    ]
    results: list[PricingResult] = []
    parallel_cpu_start = time.process_time()
    try:
        with executor_class(max_workers=len(blocks)) as executor:
            pending = {executor.submit(_run_forward_pricing_worker, args) for args in worker_args}
            while pending:
                done, pending = wait(pending, return_when=FIRST_COMPLETED)
                for future in sorted(done, key=lambda item: id(item)):
                    result = future.result()
                    results.append(result)
                    if first_hit_enabled and result.routes:
                        stop_event.set()
                        if pending:
                            done_remaining, pending = wait(pending)
                            results.extend(future.result() for future in done_remaining)
                        return _merge_parallel_forward_results(
                            results,
                            graph,
                            objective,
                            next_route_id,
                            pricing_mode,
                            pricing_yield_ratio,
                            pricing_worker_backend,
                            len(source_neighbors),
                            tuple(len(block) for block in blocks),
                            len(blocks),
                            selected=result,
                            process_cpu_time_seconds=time.process_time() - parallel_cpu_start,
                            adaptive_slice_seconds=adaptive_slice_seconds,
                            productive_yield_window_rate=productive_yield_window_rate,
                            prefix_task_depth=prefix_task_depth,
                        )
    finally:
        if manager is not None:
            manager.shutdown()
    if any(result.diagnostics.pricing_status == PRICING_STATUS_TIME_LIMIT_NO_COLUMNS for result in results):
        diagnostic_result = _merge_parallel_forward_results(
            results,
            graph,
            objective,
            next_route_id,
            pricing_mode,
            pricing_yield_ratio,
            pricing_worker_backend,
            len(source_neighbors),
            tuple(len(block) for block in blocks),
            len(blocks),
            selected=None,
            force_time_limit=True,
            process_cpu_time_seconds=time.process_time() - parallel_cpu_start,
            adaptive_slice_seconds=adaptive_slice_seconds,
            productive_yield_window_rate=productive_yield_window_rate,
            prefix_task_depth=prefix_task_depth,
        )
        raise PricingTimeLimitReached(diagnostic_result.diagnostics)
    return _merge_parallel_forward_results(
        results,
        graph,
        objective,
        next_route_id,
        pricing_mode,
        pricing_yield_ratio,
        pricing_worker_backend,
        len(source_neighbors),
        tuple(len(block) for block in blocks),
        len(blocks),
        selected=None,
        process_cpu_time_seconds=time.process_time() - parallel_cpu_start,
        adaptive_slice_seconds=adaptive_slice_seconds,
        productive_yield_window_rate=productive_yield_window_rate,
        prefix_task_depth=prefix_task_depth,
    )


def _run_forward_pricing_worker(args: tuple) -> PricingResult:
    (
        worker_id,
        source_neighbor_block,
        graph,
        objective,
        residual_customers,
        restrictions,
        duals,
        next_route_id,
        farkas,
        pricing_tolerance,
        use_standard_acceleration,
        stop_at_first_negative,
        batch_size,
        deadline,
        existing_routes,
        existing_column_paths,
        pricing_mode,
        pricing_yield_ratio,
        stop_event,
        pricing_worker_backend,
        source_neighbor_count,
        source_neighbor_block_sizes,
        parallel_workers,
        known_signature_costs,
    ) = args
    cpu_start = time.process_time()
    try:
        result = _price_route_forward_only(
            graph=graph,
            objective=objective,
            residual_customers=residual_customers,
            restrictions=restrictions,
            duals=duals,
            next_route_id=next_route_id,
            farkas=farkas,
            pricing_tolerance=pricing_tolerance,
            use_standard_acceleration=use_standard_acceleration,
            stop_at_first_negative=stop_at_first_negative,
            batch_size=batch_size,
            deadline=deadline,
            existing_routes=existing_routes,
            existing_column_paths=existing_column_paths,
            pricing_mode=pricing_mode,
            pricing_yield_ratio=pricing_yield_ratio,
            source_neighbor_block=tuple(source_neighbor_block),
            worker_id=worker_id,
            stop_event=stop_event,
            known_signature_costs_snapshot=known_signature_costs,
        )
    except PricingTimeLimitReached as exc:
        result = PricingResult(tuple(), tuple(), None, None, exc.diagnostics)
    return _with_runtime_diagnostics(
        result,
        pricing_worker_backend=pricing_worker_backend,
        process_cpu_time_seconds=time.process_time() - cpu_start,
        worker_id=worker_id,
        source_neighbor_count=source_neighbor_count,
        source_neighbor_block_sizes=tuple(source_neighbor_block_sizes),
        parallel_workers=parallel_workers,
    )


def _run_persistent_forward_pricing_worker(task: _WorkerPricingTask) -> _WorkerPricingResult:
    if _PROCESS_WORKER_GRAPH is None or _PROCESS_WORKER_OBJECTIVE is None:
        raise RuntimeError("persistent pricing process worker was not initialized")
    cpu_start = time.process_time()
    try:
        result = _price_route_forward_only(
            graph=_PROCESS_WORKER_GRAPH,
            objective=_PROCESS_WORKER_OBJECTIVE,
            residual_customers=task.residual_customers,
            restrictions=task.restrictions,
            duals=task.duals,
            next_route_id=task.next_route_id,
            farkas=task.farkas,
            pricing_tolerance=task.pricing_tolerance,
            use_standard_acceleration=task.use_standard_acceleration,
            stop_at_first_negative=task.stop_at_first_negative,
            batch_size=task.batch_size,
            deadline=task.deadline,
            existing_routes=None,
            existing_column_paths=None,
            pricing_mode=task.pricing_mode,
            pricing_yield_ratio=task.pricing_yield_ratio,
            source_neighbor_block=tuple(task.source_neighbor_block),
            worker_id=task.worker_id,
            stop_event=task.stop_event,
            known_signature_costs_snapshot=task.known_signature_costs,
            source_prefixes=task.source_prefixes,
        )
    except PricingTimeLimitReached as exc:
        result = PricingResult(tuple(), tuple(), None, None, exc.diagnostics)
    result = _with_runtime_diagnostics(
        result,
        pricing_worker_backend=task.pricing_worker_backend,
        process_cpu_time_seconds=time.process_time() - cpu_start,
        worker_id=task.worker_id,
        source_neighbor_count=task.source_neighbor_count,
        source_neighbor_block_sizes=tuple(task.source_neighbor_block_sizes),
        parallel_workers=task.parallel_workers,
        source_neighbor_task_count=task.source_neighbor_task_count,
        source_neighbor_task_sizes=tuple(task.source_neighbor_task_sizes),
        local_worker_candidate_quota=task.local_worker_candidate_quota,
    )
    return _WorkerPricingResult(
        call_id=task.call_id,
        dual_id=task.dual_id,
        worker_id=task.worker_id,
        route_paths=tuple(route.path for route in result.routes),
        reduced_costs=tuple(result.reduced_costs),
        best_path=None if result.best_route is None else result.best_route.path,
        best_reduced_cost=result.best_reduced_cost,
        diagnostics=result.diagnostics,
    )


def _admissible_source_neighbors(graph: TransformedGraph) -> tuple[str, ...]:
    source = graph.instance.depot_source
    sink = graph.instance.depot_sink
    return tuple(sorted(node for node in graph.out_arcs[source] if node != sink))


def _partition_source_neighbors(neighbors: tuple[str, ...], workers: int) -> tuple[tuple[str, ...], ...]:
    if workers <= 0:
        raise ValueError("workers must be positive")
    if not neighbors:
        return (tuple(),)
    block_count = min(workers, len(neighbors))
    base, remainder = divmod(len(neighbors), block_count)
    blocks = []
    start = 0
    for index in range(block_count):
        size = base + (1 if index < remainder else 0)
        blocks.append(tuple(neighbors[start:start + size]))
        start += size
    return tuple(blocks)


def _chunk_source_neighbors(neighbors: tuple[str, ...], task_size: int) -> tuple[tuple[str, ...], ...]:
    if task_size <= 0:
        raise ValueError("source neighbor task size must be positive")
    if not neighbors:
        return (tuple(),)
    return tuple(tuple(neighbors[index:index + task_size]) for index in range(0, len(neighbors), task_size))


def _source_neighbor_prefix_tasks(
    graph: TransformedGraph,
    objective: ObjectiveData,
    residual_customers: frozenset[str],
    restrictions: BranchRestrictions,
    prefix_task_depth: int,
) -> tuple[tuple[str, ...], ...]:
    if prefix_task_depth <= 0:
        raise ValueError("prefix task depth must be positive")
    if prefix_task_depth == 1:
        return tuple((neighbor,) for neighbor in _admissible_source_neighbors(graph))
    instance = graph.instance
    source = instance.depot_source
    sink = instance.depot_sink
    zero_duals = PricingDuals(mu={customer: 0.0 for customer in instance.customers}, kappa=0.0, nu={})
    arc_customer_sets = {arc: graph.arc_customer_set(arc) for arc in graph.arcs}
    source_label = _Label(
        path=(source,),
        represented=frozenset(),
        truck_visited=frozenset({source}),
        truck_load=0.0,
        active_pad=None,
        active_pad_arrival=0.0,
        active_wait=0.0,
        block_count=0,
        physical_time=0.0,
        service_times=tuple(),
        sr_counts=tuple(),
        reduced_cost=0.0,
        used_arcs=frozenset(),
        represented_mask=0,
        truck_node_mask=_truck_node_mask(frozenset({source}), graph),
    )
    prefixes: list[tuple[str, ...]] = []
    stack = [source_label]
    while stack:
        label = stack.pop()
        depth = len(label.path) - 1
        node = label.path[-1]
        if depth >= prefix_task_depth or node == sink:
            if depth > 0:
                prefixes.append(label.path[1:])
            continue
        for next_node in sorted(graph.out_arcs[node], reverse=True):
            if node == source and next_node == sink:
                continue
            if not _extension_allowed(label, next_node, graph, residual_customers, restrictions):
                continue
            if _extension_rejected_by_service_deadline(label, next_node, graph, objective):
                continue
            new_label = _extend(
                label,
                next_node,
                graph,
                objective,
                zero_duals,
                tuple(),
                False,
                restrictions,
                arc_customer_sets,
            )
            if next_node == sink:
                if new_label.represented and _complete_allowed(new_label, graph, restrictions, arc_customer_sets):
                    prefixes.append(new_label.path[1:])
                continue
            stack.append(new_label)
    return tuple(sorted(set(prefixes)))


def _chunk_prefix_tasks(prefixes: tuple[tuple[str, ...], ...], task_size: int) -> tuple[tuple[tuple[str, ...], ...], ...]:
    if task_size <= 0:
        raise ValueError("source neighbor task size must be positive")
    if not prefixes:
        return (tuple(),)
    return tuple(tuple(prefixes[index:index + task_size]) for index in range(0, len(prefixes), task_size))


def _with_runtime_diagnostics(
    result: PricingResult,
    *,
    pricing_worker_backend: str,
    process_cpu_time_seconds: float,
    worker_id: int | None,
    source_neighbor_count: int,
    source_neighbor_block_sizes: tuple[int, ...],
    parallel_workers: int,
    source_neighbor_task_count: int = 0,
    source_neighbor_task_sizes: tuple[int, ...] = tuple(),
    local_worker_candidate_quota: int = 0,
) -> PricingResult:
    elapsed = result.diagnostics.elapsed_seconds
    core_equivalent = process_cpu_time_seconds / elapsed if elapsed > 0.0 else 0.0
    diagnostics = replace(
        result.diagnostics,
        pricing_worker_backend=pricing_worker_backend,
        process_cpu_time_seconds=process_cpu_time_seconds,
        cpu_core_equivalent=core_equivalent,
        worker_id=worker_id,
        source_neighbor_count=source_neighbor_count,
        source_neighbor_block_sizes=source_neighbor_block_sizes,
        parallel_labeling_used=parallel_workers > 1,
        parallel_workers=parallel_workers,
        source_neighbor_task_count=source_neighbor_task_count,
        source_neighbor_task_sizes=source_neighbor_task_sizes,
        local_worker_candidate_quota=local_worker_candidate_quota,
    )
    return PricingResult(
        result.routes,
        result.reduced_costs,
        result.best_route,
        result.best_reduced_cost,
        diagnostics,
        result.side_pool_routes,
        result.side_pool_reduced_costs,
    )


def _merge_parallel_forward_results(
    results: list[PricingResult],
    graph: TransformedGraph,
    objective: ObjectiveData,
    next_route_id: int,
    pricing_mode: str,
    pricing_yield_ratio: float,
    pricing_worker_backend: str,
    source_neighbor_count: int,
    source_neighbor_block_sizes: tuple[int, ...],
    parallel_workers: int,
    *,
    selected: PricingResult | None,
    force_time_limit: bool = False,
    process_cpu_time_seconds: float | None = None,
    adaptive_slice_seconds: float = 0.0,
    productive_yield_window_rate: float = 0.0,
    prefix_task_depth: int = 1,
) -> PricingResult:
    diagnostics = [result.diagnostics for result in results]
    verified_candidates: list[tuple[Route, float, int]] = []
    selected_route_paths: list[tuple[str, ...]] = []
    returned_routes: list[Route] = []
    returned_costs: list[float] = []
    if selected is not None:
        selected_route_paths = [route.path for route in selected.routes]
        returned_costs = list(selected.reduced_costs)
        returned_routes = [
            route_from_path(next_route_id + index, path, graph, objective)
            for index, path in enumerate(selected_route_paths)
        ]
    best_result = min(
        (result for result in results if result.best_reduced_cost is not None),
        key=lambda item: (item.best_reduced_cost, item.best_route.path if item.best_route else tuple()),
        default=None,
    )
    best_cost = None if best_result is None else best_result.best_reduced_cost
    best_route = None
    if best_result is not None and best_result.best_route is not None:
        best_route = route_from_path(next_route_id + len(returned_routes), best_result.best_route.path, graph, objective)
    has_negative = bool(returned_routes)
    exact_completion = bool(diagnostics) and all(item.exact_completion for item in diagnostics) and not has_negative and not force_time_limit
    if force_time_limit:
        termination_reason = "time_limit_unresolved"
        certification_mode = "not_certified_time_limit"
    elif has_negative:
        termination_reason = "closure_negative_batch_found" if pricing_mode == "closure" else "productive_batch_found"
        certification_mode = "not_certified_closure_returned_columns" if pricing_mode == "closure" else "not_certified_productive"
    else:
        termination_reason = "exact_pricing_complete"
        certification_mode = "source_neighbor_partitions_closed"
    elapsed = max((item.elapsed_seconds for item in diagnostics), default=0.0)
    worker_cpu_sum = sum(item.process_cpu_time_seconds for item in diagnostics)
    cpu_time = process_cpu_time_seconds if pricing_worker_backend == "thread" and process_cpu_time_seconds is not None else worker_cpu_sum
    per_worker_elapsed = tuple(sorted((item.worker_id, item.elapsed_seconds) for item in diagnostics if item.worker_id is not None))
    per_worker_cpu = tuple(sorted((item.worker_id, item.process_cpu_time_seconds) for item in diagnostics if item.worker_id is not None))
    per_worker_generated = tuple(sorted((item.worker_id, item.labels_generated) for item in diagnostics if item.worker_id is not None))
    per_worker_dominated = tuple(sorted((item.worker_id, item.labels_dominated) for item in diagnostics if item.worker_id is not None))
    per_worker_pruned = tuple(sorted((item.worker_id, item.labels_pruned) for item in diagnostics if item.worker_id is not None))
    per_worker_complete = tuple(sorted((item.worker_id, item.complete_routes_generated) for item in diagnostics if item.worker_id is not None))
    per_worker_negative = tuple(sorted((item.worker_id, item.returned_routes) for item in diagnostics if item.worker_id is not None))
    return _pricing_result(
        returned_routes,
        returned_costs,
        best_route,
        best_cost,
        labels_generated=sum(item.labels_generated for item in diagnostics),
        labels_dominated=sum(item.labels_dominated for item in diagnostics),
        labels_pruned=sum(item.labels_pruned for item in diagnostics),
        labels_purged=sum(item.labels_purged for item in diagnostics),
        stale_labels_skipped=sum(item.stale_labels_skipped for item in diagnostics),
        standard_bound_pruned=sum(item.standard_bound_pruned for item in diagnostics),
        farkas_bound_pruned=sum(item.farkas_bound_pruned for item in diagnostics),
        max_queue_size=max((item.max_queue_size for item in diagnostics), default=0),
        complete_routes_generated=sum(item.complete_routes_generated for item in diagnostics),
        exact_completion=exact_completion,
        termination_reason=termination_reason,
        certification_mode=certification_mode,
        elapsed_seconds=elapsed,
        pricing_engine="source_neighbor_parallel_forward",
        forward_labels_generated=sum(item.forward_labels_generated for item in diagnostics),
        forward_labeling_time_seconds=sum(item.forward_labeling_time_seconds for item in diagnostics),
        parallel_labeling_used=True,
        parallel_workers=parallel_workers,
        parallel_calls=1,
        pricing_mode=pricing_mode,
        pricing_yield_ratio=pricing_yield_ratio,
        pricing_worker_backend=pricing_worker_backend,
        process_cpu_time_seconds=cpu_time,
        cpu_core_equivalent=cpu_time / elapsed if elapsed > 0.0 else 0.0,
        source_neighbor_count=source_neighbor_count,
        source_neighbor_block_sizes=source_neighbor_block_sizes,
        first_hit_worker_id=None if selected is None else selected.diagnostics.worker_id,
        first_hit_exits=1 if selected is not None else 0,
        interrupted_worker_calls=sum(1 for item in diagnostics if item.termination_reason == "interrupted_after_first_hit"),
        certification_worker_calls=sum(1 for item in diagnostics if item.exact_completion and not item.returned_routes),
        productive_worker_calls=sum(1 for item in diagnostics if item.returned_routes),
        per_worker_elapsed_seconds=per_worker_elapsed,
        per_worker_cpu_time_seconds=per_worker_cpu,
        per_worker_labels_generated=per_worker_generated,
        per_worker_labels_dominated=per_worker_dominated,
        per_worker_labels_pruned=per_worker_pruned,
        per_worker_completed_labels=per_worker_complete,
        per_worker_verified_negative_routes=per_worker_negative,
        negative_routes_verified=len(returned_routes),
        negative_routes_inserted=len(returned_routes),
        adaptive_slice_seconds=adaptive_slice_seconds,
        productive_yield_window_rate=productive_yield_window_rate,
        prefix_task_depth=prefix_task_depth,
        extensions_attempted=sum(item.extensions_attempted for item in diagnostics),
        extensions_rejected_by_deadline=sum(item.extensions_rejected_by_deadline for item in diagnostics),
        deadline_reachability_removed=sum(item.deadline_reachability_removed for item in diagnostics),
        reward_set_size_before_deadline=sum(item.reward_set_size_before_deadline for item in diagnostics),
        reward_set_size_after_deadline=sum(item.reward_set_size_after_deadline for item in diagnostics),
        deadline_reward_bound_calls=sum(item.deadline_reward_bound_calls for item in diagnostics),
        deadline_dominance_prefilter_skips=sum(item.deadline_dominance_prefilter_skips for item in diagnostics),
        routes_rejected_by_deadline_in_master=sum(item.routes_rejected_by_deadline_in_master for item in diagnostics),
        forward_dominance_tests=sum(item.forward_dominance_tests for item in diagnostics),
        forward_same_node_dominance_tests=sum(item.forward_same_node_dominance_tests for item in diagnostics),
        forward_physical_location_dominance_tests=sum(item.forward_physical_location_dominance_tests for item in diagnostics),
        forward_physical_location_dominance_rejections=sum(
            item.forward_physical_location_dominance_rejections for item in diagnostics
        ),
        forward_return_time_credit_checks=sum(item.forward_return_time_credit_checks for item in diagnostics),
        forward_return_time_credit_checks_skipped=sum(
            item.forward_return_time_credit_checks_skipped for item in diagnostics
        ),
        forward_branch_language_failures=sum(item.forward_branch_language_failures for item in diagnostics),
        forward_mask_scalar_prefilter_failures=sum(item.forward_mask_scalar_prefilter_failures for item in diagnostics),
        dom_gate_pairs_seen=sum(item.dom_gate_pairs_seen for item in diagnostics),
        dom_gate_mask_failures=sum(item.dom_gate_mask_failures for item in diagnostics),
        dom_gate_scalar_failures=sum(item.dom_gate_scalar_failures for item in diagnostics),
        dom_gate_branch_failures=sum(item.dom_gate_branch_failures for item in diagnostics),
        dom_gate_deadline_failures=sum(item.dom_gate_deadline_failures for item in diagnostics),
        labels_dominated_same_node=sum(item.labels_dominated_same_node for item in diagnostics),
        labels_dominated_physical=sum(item.labels_dominated_physical for item in diagnostics),
        dom_prefilter_pairs=sum(item.dom_prefilter_pairs for item in diagnostics),
        dom_prefilter_mask_fail=sum(item.dom_prefilter_mask_fail for item in diagnostics),
        dom_prefilter_branch_fail=sum(item.dom_prefilter_branch_fail for item in diagnostics),
        dom_prefilter_payload_fail=sum(item.dom_prefilter_payload_fail for item in diagnostics),
        dom_prefilter_block_fail=sum(item.dom_prefilter_block_fail for item in diagnostics),
        dom_prefilter_return_credit_fail=sum(item.dom_prefilter_return_credit_fail for item in diagnostics),
        dom_full_tests=sum(item.dom_full_tests for item in diagnostics),
        dom_full_rejections=sum(item.dom_full_rejections for item in diagnostics),
        physical_location_full_tests=sum(item.physical_location_full_tests for item in diagnostics),
        physical_location_rejections=sum(item.physical_location_rejections for item in diagnostics),
    )


def _candidate_source_neighbor(route: Route) -> str:
    if len(route.path) < 2:
        raise RuntimeError("pricing candidate path has no source-neighbor arc")
    return route.path[1]


def _candidate_counts_by_source(candidates: list[tuple[Route, float, int]]) -> tuple[tuple[str, int], ...]:
    counts: dict[str, int] = {}
    for route, _, _ in candidates:
        source_neighbor = _candidate_source_neighbor(route)
        counts[source_neighbor] = counts.get(source_neighbor, 0) + 1
    return tuple(sorted(counts.items()))


def _select_diverse_pricing_candidates(
    candidates: list[tuple[Route, float, int]],
    residual_customers: frozenset[str],
    batch_target: int,
    diversity_fraction: float,
) -> tuple[list[tuple[Route, float, int]], int]:
    if batch_target <= 0:
        raise ValueError("batch target must be positive")
    if not 0.0 <= diversity_fraction <= 1.0:
        raise ValueError("pricing diversity batch fraction must be in [0, 1]")
    if not candidates:
        return [], 0
    diversity_quota = min(batch_target, ceil(batch_target * diversity_fraction))
    selected_indices: set[int] = set()
    selected: list[tuple[Route, float, int]] = []
    if diversity_quota > 0:
        groups: dict[str, list[int]] = {}
        for index, (route, _, _) in enumerate(candidates):
            groups.setdefault(_candidate_source_neighbor(route), []).append(index)
        covered: set[str] = set()
        while len(selected) < diversity_quota:
            progressed = False
            for source_neighbor in sorted(groups):
                available = [index for index in groups[source_neighbor] if index not in selected_indices]
                if not available:
                    continue
                best_index = min(
                    available,
                    key=lambda index: (
                        -len(candidates[index][0].served.intersection(residual_customers - covered)),
                        candidates[index][1],
                        candidates[index][0].path,
                    ),
                )
                selected_indices.add(best_index)
                route, cost, worker_id = candidates[best_index]
                selected.append((route, cost, worker_id))
                covered.update(route.served.intersection(residual_customers))
                progressed = True
                if len(selected) >= diversity_quota:
                    break
            if not progressed:
                break
    for index, candidate in enumerate(candidates):
        if len(selected) >= batch_target:
            break
        if index in selected_indices:
            continue
        selected_indices.add(index)
        selected.append(candidate)
    return selected, diversity_quota


def _merge_compact_worker_results(
    results: list[_WorkerPricingResult],
    graph: TransformedGraph,
    objective: ObjectiveData,
    residual_customers: frozenset[str],
    restrictions: BranchRestrictions,
    duals: PricingDuals,
    next_route_id: int,
    farkas: bool,
    pricing_tolerance: float,
    existing_routes: dict[tuple[str, ...], Route] | None,
    existing_column_paths: set[tuple[str, ...]] | None,
    pricing_mode: str,
    pricing_yield_ratio: float,
    pricing_worker_backend: str,
    source_neighbor_count: int,
    source_neighbor_block_sizes: tuple[int, ...],
    parallel_workers: int,
    *,
    selected: _WorkerPricingResult | None,
    force_time_limit: bool,
    call_id: int,
    dual_id: int,
    submission_time_seconds: float,
    pool_startup_time_seconds: float,
    pool_startup_count: int,
    pool_reused_calls: int,
    first_hit_enabled: bool,
    batch_target: int,
    productive_slice_seconds: float = 0.0,
    productive_slice_deadline_used: bool = False,
    adaptive_slice_seconds: float = 0.0,
    productive_yield_window_rate: float = 0.0,
    stabilized_dual_enabled: bool = False,
    prefix_task_depth: int = 1,
    pricing_diversity_batch_fraction: float = 0.5,
    worker_reduced_costs_are_true: bool = True,
    call_start_time: float | None = None,
) -> PricingResult:
    stale_rejections = 0
    for result in results:
        if result.call_id != call_id or result.dual_id != dual_id:
            stale_rejections += 1
    if stale_rejections:
        raise RuntimeError("persistent pricing worker returned a stale call or dual snapshot")
    diagnostics = [result.diagnostics for result in results]
    merge_start = time.time()
    merge_cpu_start = time.process_time()
    route_decode_time = 0.0
    verification_time = 0.0
    duplicate_lookup_time = 0.0
    decoded_routes = 0
    verified_routes = 0
    candidate_sources = [selected] if selected is not None else sorted(results, key=lambda item: item.worker_id)
    raw_candidates: list[tuple[float, tuple[str, ...], int]] = []
    for result in candidate_sources:
        if result is None:
            continue
        for path, cost in zip(result.route_paths, result.reduced_costs):
            raw_candidates.append((cost, path, result.worker_id))
    raw_candidate_count = len(raw_candidates)
    unique_candidates: list[tuple[float, tuple[str, ...], int]] = []
    seen_paths: set[tuple[str, ...]] = set()
    for cost, path, worker_id in sorted(raw_candidates, key=lambda item: (item[0], item[1], item[2])):
        if path in seen_paths:
            continue
        seen_paths.add(path)
        unique_candidates.append((cost, path, worker_id))
    known_signature_costs = _known_signature_costs(
        existing_routes,
        existing_column_paths,
        graph,
        residual_customers,
        RouteSignatureCache(),
    )
    signature_cache = RouteSignatureCache()
    arc_customer_sets = {arc: graph.arc_customer_set(arc) for arc in graph.arcs}
    verified_candidates: list[tuple[Route, float, int]] = []
    returned_routes: list[Route] = []
    returned_costs: list[float] = []
    true_dual_rejected_candidates = 0
    worker_true_diffs: list[float] = []
    routes_rejected_by_deadline = 0
    for worker_cost, path, _ in unique_candidates:
        decode_start = time.time()
        try:
            route = route_from_path(next_route_id + len(verified_candidates), path, graph, objective)
        except ServiceEnvelopeViolation:
            route_decode_time += time.time() - decode_start
            routes_rejected_by_deadline += 1
            continue
        route_decode_time += time.time() - decode_start
        decoded_routes += 1
        if not restrictions.route_allowed(route, arc_customer_sets):
            raise RuntimeError("compact worker pricing candidate failed route-level branch validation")
        verify_start = time.time()
        direct_cost = route_farkas_reduced_cost(route, duals) if farkas else route_reduced_cost(route, duals)
        verification_time += time.time() - verify_start
        verified_routes += 1
        diff = worker_cost - direct_cost
        worker_true_diffs.append(diff)
        if worker_reduced_costs_are_true and abs(diff) > max(1e-8, pricing_tolerance * 10.0):
            raise RuntimeError("worker reduced-cost report disagrees with direct main-process verification")
        if direct_cost >= -pricing_tolerance:
            if stabilized_dual_enabled:
                true_dual_rejected_candidates += 1
            continue
        duplicate_start = time.time()
        duplicate, signature = _duplicate_dominated_by_existing(
            route,
            known_signature_costs,
            graph,
            residual_customers,
            signature_cache,
        )
        duplicate_lookup_time += time.time() - duplicate_start
        if duplicate:
            continue
        verified_candidates.append((route, direct_cost, worker_id))
        coeff_signature = route_coefficient_signature(signature)
        known_signature_costs[coeff_signature] = min(
            known_signature_costs.get(coeff_signature, float("inf")),
            route.cost,
        )
    selected_candidates, diversity_quota = _select_diverse_pricing_candidates(
        verified_candidates,
        residual_customers,
        batch_target,
        pricing_diversity_batch_fraction,
    )
    returned_routes = [route for route, _, _ in selected_candidates]
    returned_costs = [cost for _, cost, _ in selected_candidates]
    best_result = min(
        (result for result in results if result.best_reduced_cost is not None),
        key=lambda item: (item.best_reduced_cost, item.best_path or tuple()),
        default=None,
    )
    best_cost = None if best_result is None else best_result.best_reduced_cost
    best_route = None
    if best_result is not None and best_result.best_path is not None:
        try:
            best_route = route_from_path(next_route_id + len(returned_routes), best_result.best_path, graph, objective)
        except ServiceEnvelopeViolation:
            routes_rejected_by_deadline += 1
    has_negative = bool(returned_routes)
    any_time_limit = force_time_limit or any(
        item.pricing_status == PRICING_STATUS_TIME_LIMIT_NO_COLUMNS for item in diagnostics
    )
    exact_completion = bool(diagnostics) and all(item.exact_completion for item in diagnostics) and not has_negative and not any_time_limit
    if any_time_limit and has_negative:
        termination_reason = "time_limit_with_columns"
        certification_mode = "not_certified_time_limit"
    elif any_time_limit:
        termination_reason = "time_limit_unresolved"
        certification_mode = "not_certified_time_limit"
    elif has_negative:
        termination_reason = "closure_negative_batch_found" if pricing_mode == "closure" else "productive_batch_found"
        certification_mode = "not_certified_closure_returned_columns" if pricing_mode == "closure" else "not_certified_productive"
    else:
        termination_reason = "exact_pricing_complete"
        certification_mode = "source_neighbor_partitions_closed"
    merge_time = time.time() - merge_start
    main_cpu_time = time.process_time() - merge_cpu_start
    if call_start_time is None:
        worker_elapsed = max((item.elapsed_seconds for item in diagnostics), default=0.0)
        elapsed = worker_elapsed + submission_time_seconds + merge_time
    else:
        elapsed = time.time() - call_start_time
    worker_cpu_sum = sum(item.process_cpu_time_seconds for item in diagnostics)
    total_cpu = worker_cpu_sum + main_cpu_time
    per_worker_elapsed = tuple(sorted((item.worker_id, item.elapsed_seconds) for item in diagnostics if item.worker_id is not None))
    per_worker_cpu = tuple(sorted((item.worker_id, item.process_cpu_time_seconds) for item in diagnostics if item.worker_id is not None))
    per_worker_generated = tuple(sorted((item.worker_id, item.labels_generated) for item in diagnostics if item.worker_id is not None))
    per_worker_dominated = tuple(sorted((item.worker_id, item.labels_dominated) for item in diagnostics if item.worker_id is not None))
    per_worker_pruned = tuple(sorted((item.worker_id, item.labels_pruned) for item in diagnostics if item.worker_id is not None))
    per_worker_complete = tuple(sorted((item.worker_id, item.complete_routes_generated) for item in diagnostics if item.worker_id is not None))
    per_worker_negative = tuple(sorted((item.worker_id, item.returned_routes) for item in diagnostics if item.worker_id is not None))
    verified_by_source = _candidate_counts_by_source([(route, cost, worker_id) for route, cost, worker_id in verified_candidates])
    selected_by_source = _candidate_counts_by_source(selected_candidates)
    selected_customers = frozenset().union(*(route.served for route in returned_routes)) if returned_routes else frozenset()
    mean_worker_diff = sum(worker_true_diffs) / len(worker_true_diffs) if worker_true_diffs else 0.0
    max_abs_worker_diff = max((abs(value) for value in worker_true_diffs), default=0.0)
    return _pricing_result(
        returned_routes,
        returned_costs,
        best_route,
        best_cost,
        labels_generated=sum(item.labels_generated for item in diagnostics),
        labels_dominated=sum(item.labels_dominated for item in diagnostics),
        labels_pruned=sum(item.labels_pruned for item in diagnostics),
        labels_purged=sum(item.labels_purged for item in diagnostics),
        stale_labels_skipped=sum(item.stale_labels_skipped for item in diagnostics),
        standard_bound_pruned=sum(item.standard_bound_pruned for item in diagnostics),
        farkas_bound_pruned=sum(item.farkas_bound_pruned for item in diagnostics),
        max_queue_size=max((item.max_queue_size for item in diagnostics), default=0),
        complete_routes_generated=sum(item.complete_routes_generated for item in diagnostics),
        exact_completion=exact_completion,
        termination_reason=termination_reason,
        certification_mode=certification_mode,
        elapsed_seconds=elapsed,
        pricing_engine="source_neighbor_parallel_forward",
        forward_labels_generated=sum(item.forward_labels_generated for item in diagnostics),
        forward_labeling_time_seconds=sum(item.forward_labeling_time_seconds for item in diagnostics),
        parallel_labeling_used=True,
        parallel_workers=parallel_workers,
        parallel_calls=1,
        signature_cache_hits=signature_cache.stats.signature_cache_hits,
        signature_cache_misses=signature_cache.stats.signature_cache_misses,
        core_signature_cache_hits=signature_cache.stats.core_signature_cache_hits,
        core_signature_cache_misses=signature_cache.stats.core_signature_cache_misses,
        active_signature_cache_hits=signature_cache.stats.active_signature_cache_hits,
        active_signature_cache_misses=signature_cache.stats.active_signature_cache_misses,
        sr_coeff_cache_hits=signature_cache.stats.sr_coeff_cache_hits,
        sr_coeff_cache_misses=signature_cache.stats.sr_coeff_cache_misses,
        active_sr_key_cache_hits=signature_cache.stats.active_sr_key_cache_hits,
        active_sr_key_cache_misses=signature_cache.stats.active_sr_key_cache_misses,
        active_sr_coeffs_computed=signature_cache.stats.active_sr_coeffs_computed,
        triplet_masks_built=signature_cache.stats.triplet_masks_built,
        duplicate_equivalent_rejected=signature_cache.stats.duplicate_equivalent_rejected,
        cost_dominated_rejected=signature_cache.stats.cost_dominated_rejected,
        signature_build_time_seconds=signature_cache.stats.signature_build_time,
        sr_coeff_build_time_seconds=signature_cache.stats.sr_coeff_build_time,
        duplicate_lookup_time_seconds=duplicate_lookup_time,
        route_decode_time_seconds=route_decode_time,
        reduced_cost_verification_time_seconds=verification_time,
        pricing_mode=pricing_mode,
        pricing_yield_ratio=pricing_yield_ratio,
        pricing_status=_pricing_status_from_reason(termination_reason, has_negative),
        pricing_worker_backend=pricing_worker_backend,
        process_cpu_time_seconds=total_cpu,
        cpu_core_equivalent=total_cpu / elapsed if elapsed > 0.0 else 0.0,
        source_neighbor_count=source_neighbor_count,
        source_neighbor_block_sizes=source_neighbor_block_sizes,
        first_hit_worker_id=None if selected is None else selected.worker_id,
        first_hit_exits=1 if selected is not None else 0,
        interrupted_worker_calls=sum(1 for item in diagnostics if item.termination_reason == "interrupted_after_first_hit"),
        certification_worker_calls=sum(1 for item in diagnostics if item.exact_completion and not item.returned_routes),
        productive_worker_calls=sum(1 for item in diagnostics if item.returned_routes),
        per_worker_elapsed_seconds=per_worker_elapsed,
        per_worker_cpu_time_seconds=per_worker_cpu,
        per_worker_labels_generated=per_worker_generated,
        per_worker_labels_dominated=per_worker_dominated,
        per_worker_labels_pruned=per_worker_pruned,
        per_worker_completed_labels=per_worker_complete,
        per_worker_verified_negative_routes=per_worker_negative,
        negative_routes_verified=verified_routes,
        negative_routes_inserted=len(returned_routes),
        pricing_pool_startup_time_seconds=pool_startup_time_seconds,
        pricing_pool_startup_count=pool_startup_count,
        pricing_pool_reused_calls=pool_reused_calls,
        pricing_task_submission_time_seconds=submission_time_seconds,
        pricing_worker_payload_count=len(results),
        pricing_worker_response_count=len(results),
        pricing_candidate_paths_before_merge=raw_candidate_count,
        pricing_candidate_paths_after_merge=len(unique_candidates),
        pricing_decoded_routes_in_main=decoded_routes,
        pricing_verified_routes_in_main=verified_routes,
        pricing_batch_target=batch_target,
        pricing_returned_batch_size=len(returned_routes),
        pricing_first_hit_enabled=first_hit_enabled,
        pricing_stale_response_rejections=stale_rejections,
        pricing_worker_cpu_time_seconds=worker_cpu_sum,
        pricing_main_process_cpu_time_seconds=main_cpu_time,
        pricing_main_merge_time_seconds=merge_time,
        productive_slice_seconds=productive_slice_seconds,
        productive_slice_deadline_used=productive_slice_deadline_used,
        adaptive_slice_seconds=adaptive_slice_seconds,
        productive_yield_window_rate=productive_yield_window_rate,
        stabilized_dual_enabled=stabilized_dual_enabled,
        stabilized_candidates_returned=raw_candidate_count if stabilized_dual_enabled else 0,
        true_dual_rejected_candidates=true_dual_rejected_candidates,
        mean_worker_rc_minus_true_rc=mean_worker_diff,
        max_abs_worker_true_rc_discrepancy=max_abs_worker_diff,
        prefix_task_depth=prefix_task_depth,
        productive_time_limit_with_columns=1 if termination_reason == "time_limit_with_columns" and has_negative else 0,
        productive_time_limit_no_columns=1 if termination_reason == "time_limit_unresolved" and pricing_mode == "productive" else 0,
        source_neighbor_task_count=len(diagnostics),
        source_neighbor_task_sizes=source_neighbor_block_sizes,
        local_worker_candidate_quota=max((item.local_worker_candidate_quota for item in diagnostics), default=0),
        diversity_quota=diversity_quota,
        diversity_selected_routes=len(returned_routes),
        diversity_selected_customers=len(selected_customers.intersection(residual_customers)),
        verified_candidates_by_source_neighbor=verified_by_source,
        selected_candidates_by_source_neighbor=selected_by_source,
        routes_rejected_by_deadline_in_master=routes_rejected_by_deadline
        + sum(item.routes_rejected_by_deadline_in_master for item in diagnostics),
        extensions_attempted=sum(item.extensions_attempted for item in diagnostics),
        extensions_rejected_by_deadline=sum(item.extensions_rejected_by_deadline for item in diagnostics),
        deadline_reachability_removed=sum(item.deadline_reachability_removed for item in diagnostics),
        reward_set_size_before_deadline=sum(item.reward_set_size_before_deadline for item in diagnostics),
        reward_set_size_after_deadline=sum(item.reward_set_size_after_deadline for item in diagnostics),
        deadline_reward_bound_calls=sum(item.deadline_reward_bound_calls for item in diagnostics),
        deadline_dominance_prefilter_skips=sum(item.deadline_dominance_prefilter_skips for item in diagnostics),
        forward_dominance_tests=sum(item.forward_dominance_tests for item in diagnostics),
        forward_same_node_dominance_tests=sum(item.forward_same_node_dominance_tests for item in diagnostics),
        forward_physical_location_dominance_tests=sum(item.forward_physical_location_dominance_tests for item in diagnostics),
        forward_physical_location_dominance_rejections=sum(
            item.forward_physical_location_dominance_rejections for item in diagnostics
        ),
        forward_return_time_credit_checks=sum(item.forward_return_time_credit_checks for item in diagnostics),
        forward_return_time_credit_checks_skipped=sum(
            item.forward_return_time_credit_checks_skipped for item in diagnostics
        ),
        forward_branch_language_failures=sum(item.forward_branch_language_failures for item in diagnostics),
        forward_mask_scalar_prefilter_failures=sum(item.forward_mask_scalar_prefilter_failures for item in diagnostics),
        dom_gate_pairs_seen=sum(item.dom_gate_pairs_seen for item in diagnostics),
        dom_gate_mask_failures=sum(item.dom_gate_mask_failures for item in diagnostics),
        dom_gate_scalar_failures=sum(item.dom_gate_scalar_failures for item in diagnostics),
        dom_gate_branch_failures=sum(item.dom_gate_branch_failures for item in diagnostics),
        dom_gate_deadline_failures=sum(item.dom_gate_deadline_failures for item in diagnostics),
        labels_dominated_same_node=sum(item.labels_dominated_same_node for item in diagnostics),
        labels_dominated_physical=sum(item.labels_dominated_physical for item in diagnostics),
        dom_prefilter_pairs=sum(item.dom_prefilter_pairs for item in diagnostics),
        dom_prefilter_mask_fail=sum(item.dom_prefilter_mask_fail for item in diagnostics),
        dom_prefilter_branch_fail=sum(item.dom_prefilter_branch_fail for item in diagnostics),
        dom_prefilter_payload_fail=sum(item.dom_prefilter_payload_fail for item in diagnostics),
        dom_prefilter_block_fail=sum(item.dom_prefilter_block_fail for item in diagnostics),
        dom_prefilter_return_credit_fail=sum(item.dom_prefilter_return_credit_fail for item in diagnostics),
        dom_full_tests=sum(item.dom_full_tests for item in diagnostics),
        dom_full_rejections=sum(item.dom_full_rejections for item in diagnostics),
        physical_location_full_tests=sum(item.physical_location_full_tests for item in diagnostics),
        physical_location_rejections=sum(item.physical_location_rejections for item in diagnostics),
    )


def _price_route_forward_only(
    graph: TransformedGraph,
    objective: ObjectiveData,
    residual_customers: frozenset[str],
    restrictions: BranchRestrictions,
    duals: PricingDuals,
    next_route_id: int,
    farkas: bool = False,
    pricing_tolerance: float = 0.0,
    use_standard_acceleration: bool = True,
    stop_at_first_negative: bool = False,
    batch_size: int = 1,
    deadline: float | None = None,
    existing_routes: dict[tuple[str, ...], Route] | None = None,
    existing_column_paths: set[tuple[str, ...]] | None = None,
    pricing_mode: str = "productive",
    pricing_yield_ratio: float = 0.0,
    source_neighbor_block: tuple[str, ...] | None = None,
    worker_id: int | None = None,
    stop_event: object | None = None,
    known_signature_costs_snapshot: dict[RouteCoefficientSignature, float] | None = None,
    source_prefixes: tuple[tuple[str, ...], ...] = tuple(),
) -> PricingResult:
    if batch_size <= 0:
        raise ValueError("pricing batch size must be positive")
    _validate_inequality_dual_signs(duals, pricing_tolerance)
    pricing_start = time.time()
    instance = graph.instance
    source = instance.depot_source
    sink = instance.depot_sink
    active_sr = tuple(sorted(duals.nu))
    active_sr_version = len(active_sr)
    arc_customer_sets = {arc: graph.arc_customer_set(arc) for arc in graph.arcs}
    shortest = _shortest_truck_times(graph)
    bounds = _build_pricing_bounds(graph, duals, residual_customers, shortest)
    deadline_counters = _DeadlinePricingCounters(shortest=shortest)
    signature_cache = RouteSignatureCache()
    known_signature_costs = (
        dict(known_signature_costs_snapshot)
        if known_signature_costs_snapshot is not None
        else _known_signature_costs(
            existing_routes,
            existing_column_paths,
            graph,
            residual_customers,
            signature_cache,
            active_sr,
            active_sr_version,
        )
    )
    counter = count()
    source_cost = -duals.kappa if farkas else objective.coeffs.cost * instance.truck_cost - duals.kappa
    source_label = _Label(
        path=(source,),
        represented=frozenset(),
        truck_visited=frozenset({source}),
        truck_load=0.0,
        active_pad=None,
        active_pad_arrival=0.0,
        active_wait=0.0,
        block_count=0,
        physical_time=0.0,
        service_times=tuple(),
        sr_counts=tuple((triplet, 0) for triplet in active_sr),
        reduced_cost=source_cost,
        used_arcs=frozenset(),
        represented_mask=0,
        truck_node_mask=_truck_node_mask(frozenset({source}), graph),
    )
    queue: list[tuple[float, int, _Label]] = []
    best_route: Route | None = None
    best_path: tuple[str, ...] | None = None
    best_cost = float("inf")
    returned_routes: list[Route] = []
    returned_costs: list[float] = []
    returned_paths: set[tuple[str, ...]] = set()
    labels_generated = 1
    labels_dominated = 0
    labels_pruned = 0
    labels_purged = 0
    stale_labels_skipped = 0
    standard_bound_pruned = 0
    farkas_bound_pruned = 0
    complete_routes_generated = 0
    max_queue_size = 0
    kept_standard_labels: dict[str, list[_Label]] = {}
    kept_farkas_labels: dict[str, list[_Label]] = {}
    effective_batch_size = 1 if stop_at_first_negative else batch_size
    source_neighbor_set = None if source_neighbor_block is None else frozenset(source_neighbor_block)

    def deadline_diag_kwargs() -> dict[str, int]:
        return {
            "extensions_attempted": deadline_counters.extensions_attempted,
            "extensions_rejected_by_deadline": deadline_counters.extensions_rejected_by_deadline,
            "deadline_reachability_removed": deadline_counters.deadline_reachability_removed,
            "reward_set_size_before_deadline": deadline_counters.reward_set_size_before_deadline,
            "reward_set_size_after_deadline": deadline_counters.reward_set_size_after_deadline,
            "deadline_reward_bound_calls": deadline_counters.deadline_reward_bound_calls,
            "deadline_dominance_prefilter_skips": deadline_counters.deadline_dominance_prefilter_skips,
            "routes_rejected_by_deadline_in_master": deadline_counters.routes_rejected_by_deadline_in_master,
            "forward_dominance_tests": deadline_counters.forward_dominance_tests,
            "forward_same_node_dominance_tests": deadline_counters.forward_same_node_dominance_tests,
            "forward_physical_location_dominance_tests": deadline_counters.forward_physical_location_dominance_tests,
            "forward_physical_location_dominance_rejections": deadline_counters.forward_physical_location_dominance_rejections,
            "forward_return_time_credit_checks": deadline_counters.forward_return_time_credit_checks,
            "forward_return_time_credit_checks_skipped": deadline_counters.forward_return_time_credit_checks_skipped,
            "forward_branch_language_failures": deadline_counters.forward_branch_language_failures,
            "forward_mask_scalar_prefilter_failures": deadline_counters.forward_mask_scalar_prefilter_failures,
            "dom_gate_pairs_seen": deadline_counters.dom_gate_pairs_seen,
            "dom_gate_mask_failures": deadline_counters.dom_gate_mask_failures,
            "dom_gate_scalar_failures": deadline_counters.dom_gate_scalar_failures,
            "dom_gate_branch_failures": deadline_counters.dom_gate_branch_failures,
            "dom_gate_deadline_failures": deadline_counters.dom_gate_deadline_failures,
            "labels_dominated_same_node": deadline_counters.labels_dominated_same_node,
            "labels_dominated_physical": deadline_counters.labels_dominated_physical,
            "dom_prefilter_pairs": deadline_counters.dom_prefilter_pairs,
            "dom_prefilter_mask_fail": deadline_counters.dom_prefilter_mask_fail,
            "dom_prefilter_branch_fail": deadline_counters.dom_prefilter_branch_fail,
            "dom_prefilter_payload_fail": deadline_counters.dom_prefilter_payload_fail,
            "dom_prefilter_block_fail": deadline_counters.dom_prefilter_block_fail,
            "dom_prefilter_return_credit_fail": deadline_counters.dom_prefilter_return_credit_fail,
            "dom_full_tests": deadline_counters.dom_full_tests,
            "dom_full_rejections": deadline_counters.dom_full_rejections,
            "physical_location_full_tests": deadline_counters.physical_location_full_tests,
            "physical_location_rejections": deadline_counters.physical_location_rejections,
        }

    def accept_complete_label(new_label: _Label) -> PricingResult | None:
        nonlocal best_route, best_path, best_cost, complete_routes_generated
        if not new_label.represented or not _complete_allowed(new_label, graph, restrictions, arc_customer_sets):
            return None
        try:
            route = route_from_path(next_route_id, new_label.path, graph, objective)
        except ServiceEnvelopeViolation:
            deadline_counters.routes_rejected_by_deadline_in_master += 1
            return None
        if not restrictions.route_allowed(route, arc_customer_sets):
            raise RuntimeError("complete pricing label failed route-level branch validation")
        duplicate, signature = _duplicate_dominated_by_existing(
            route,
            known_signature_costs,
            graph,
            residual_customers,
            signature_cache,
            active_sr,
            active_sr_version,
        )
        if duplicate:
            return None
        reduced_cost = route_farkas_reduced_cost(route, duals) if farkas else route_reduced_cost(route, duals)
        complete_routes_generated += 1
        if reduced_cost < best_cost:
            best_route = None
            best_path = new_label.path
            best_cost = reduced_cost
        if reduced_cost < -pricing_tolerance and new_label.path not in returned_paths:
            try:
                route = route_from_path(next_route_id + len(returned_routes), new_label.path, graph, objective)
            except ServiceEnvelopeViolation:
                deadline_counters.routes_rejected_by_deadline_in_master += 1
                return None
            returned_routes.append(route)
            returned_costs.append(reduced_cost)
            returned_paths.add(route.path)
            coeff_signature = route_coefficient_signature(signature)
            known_signature_costs[coeff_signature] = min(
                known_signature_costs.get(coeff_signature, float("inf")),
                route.cost,
            )
            if route.path == best_path:
                best_route = route
            if len(returned_routes) >= effective_batch_size:
                best_route = _materialize_best_route(best_route, best_path, next_route_id, graph, objective)
                negative_batch_reason = (
                    "closure_negative_batch_found"
                    if pricing_mode == "closure"
                    else "productive_batch_found"
                )
                negative_batch_certification = (
                    "not_certified_closure_returned_columns"
                    if pricing_mode == "closure"
                    else "not_certified_productive"
                )
                return _pricing_result(
                    returned_routes,
                    returned_costs,
                    best_route,
                    best_cost,
                    labels_generated,
                    labels_dominated,
                    labels_pruned,
                    labels_purged,
                    stale_labels_skipped,
                    standard_bound_pruned,
                    farkas_bound_pruned,
                    max_queue_size,
                    complete_routes_generated,
                    exact_completion=False,
                    termination_reason=negative_batch_reason,
                    certification_mode=negative_batch_certification,
                    elapsed_seconds=time.time() - pricing_start,
                    pricing_mode=pricing_mode,
                    pricing_yield_ratio=pricing_yield_ratio,
                    **deadline_diag_kwargs(),
                )
        return None

    def insert_open_label(new_label: _Label) -> bool:
        nonlocal labels_dominated, labels_purged
        if farkas:
            accepted, rejected_count, purged_count = _insert_nondominated_farkas_label(
                kept_farkas_labels,
                new_label,
                graph,
                restrictions,
                arc_customer_sets,
            )
        else:
            accepted, rejected_count, purged_count = _insert_nondominated_standard_label(
                kept_standard_labels,
                new_label,
                graph,
                objective,
                duals,
                restrictions,
                arc_customer_sets,
                deadline_counters,
            )
        labels_dominated += rejected_count + purged_count
        labels_purged += purged_count
        return accepted

    source_prefix_tuple = tuple(source_prefixes or tuple())
    if source_prefix_tuple:
        for prefix in source_prefix_tuple:
            if not prefix:
                raise RuntimeError("empty source prefix task")
            prefix_label = source_label
            for next_node in prefix:
                if prefix_label.path[-1] == source and next_node == sink:
                    raise RuntimeError("prefix task cannot be the empty source-sink route")
                if not _extension_allowed(prefix_label, next_node, graph, residual_customers, restrictions):
                    raise RuntimeError("prefix task failed extension feasibility")
                deadline_counters.extensions_attempted += 1
                if _extension_rejected_by_service_deadline(prefix_label, next_node, graph, objective):
                    deadline_counters.extensions_rejected_by_deadline += 1
                    raise RuntimeError("prefix task failed service-envelope feasibility")
                prefix_label = _extend(
                    prefix_label,
                    next_node,
                    graph,
                    objective,
                    duals,
                    active_sr,
                    farkas,
                    restrictions,
                    arc_customer_sets,
                )
                labels_generated += 1
            if prefix_label.path[-1] == sink:
                result = accept_complete_label(prefix_label)
                if result is not None:
                    return result
                continue
            if not insert_open_label(prefix_label):
                continue
            prefix_key = _queue_key(prefix_label, graph, objective, shortest, bounds, farkas, use_standard_acceleration, deadline_counters)
            if not farkas and use_standard_acceleration and prefix_key >= -pricing_tolerance:
                labels_pruned += 1
                standard_bound_pruned += 1
                continue
            heappush(queue, (prefix_key, next(counter), prefix_label))
        max_queue_size = max(max_queue_size, len(queue))
    else:
        source_key = _queue_key(source_label, graph, objective, shortest, bounds, farkas, use_standard_acceleration, deadline_counters)
        heappush(queue, (source_key, next(counter), source_label))
        max_queue_size = 1

    while queue:
        if stop_event is not None and stop_event.is_set():
            best_route = _materialize_best_route(best_route, best_path, next_route_id, graph, objective)
            return _pricing_result(
                returned_routes,
                returned_costs,
                best_route,
                None if best_cost == float("inf") else best_cost,
                labels_generated,
                labels_dominated,
                labels_pruned,
                labels_purged,
                stale_labels_skipped,
                standard_bound_pruned,
                farkas_bound_pruned,
                max_queue_size,
                complete_routes_generated,
                exact_completion=False,
                termination_reason="interrupted_after_first_hit",
                certification_mode="not_certified_interrupted",
                elapsed_seconds=time.time() - pricing_start,
                pricing_engine="source_neighbor_forward_labeling",
                pricing_mode=pricing_mode,
                pricing_yield_ratio=pricing_yield_ratio,
                **deadline_diag_kwargs(),
            )
        if deadline is not None and time.time() >= deadline:
            if returned_routes:
                best_route = _materialize_best_route(best_route, best_path, next_route_id, graph, objective)
                return _pricing_result(
                    returned_routes,
                    returned_costs,
                    best_route,
                    best_cost,
                    labels_generated,
                    labels_dominated,
                    labels_pruned,
                    labels_purged,
                    stale_labels_skipped,
                    standard_bound_pruned,
                    farkas_bound_pruned,
                    max_queue_size,
                    complete_routes_generated,
                    exact_completion=False,
                    termination_reason="time_limit_with_columns",
                    certification_mode="not_certified_time_limit",
                    elapsed_seconds=time.time() - pricing_start,
                    pricing_mode=pricing_mode,
                    pricing_yield_ratio=pricing_yield_ratio,
                    **deadline_diag_kwargs(),
                )
            raise PricingTimeLimitReached(
                PricingDiagnostics(
                    labels_generated=labels_generated,
                    labels_dominated=labels_dominated,
                    labels_pruned=labels_pruned,
                    labels_purged=labels_purged,
                    stale_labels_skipped=stale_labels_skipped,
                    standard_bound_pruned=standard_bound_pruned,
                    farkas_bound_pruned=farkas_bound_pruned,
                    max_queue_size=max_queue_size,
                    complete_routes_generated=complete_routes_generated,
                    returned_routes=0,
                    best_reduced_cost=None if best_cost == float("inf") else best_cost,
                    exact_completion=False,
                    termination_reason="time_limit_unresolved",
                    certification_mode="not_certified_time_limit",
                    elapsed_seconds=time.time() - pricing_start,
                    forward_labels_generated=labels_generated,
                    forward_labeling_time_seconds=time.time() - pricing_start,
                    pricing_mode=pricing_mode,
                    pricing_yield_ratio=pricing_yield_ratio,
                    pricing_status=PRICING_STATUS_TIME_LIMIT_NO_COLUMNS,
                    **deadline_diag_kwargs(),
                )
            )
        key, _, label = heappop(queue)
        if not farkas and use_standard_acceleration:
            if key >= -pricing_tolerance:
                break
        node = label.path[-1]
        if node != source:
            kept_labels = kept_farkas_labels if farkas else kept_standard_labels
            if label not in kept_labels.get(_physical_location(label), []):
                stale_labels_skipped += 1
                continue
        if node == sink:
            continue
        for next_node in graph.out_arcs[node]:
            if node == source:
                if next_node == sink:
                    continue
                if source_neighbor_set is not None and next_node not in source_neighbor_set:
                    continue
            if not _extension_allowed(label, next_node, graph, residual_customers, restrictions):
                continue
            deadline_counters.extensions_attempted += 1
            if _extension_rejected_by_service_deadline(label, next_node, graph, objective):
                deadline_counters.extensions_rejected_by_deadline += 1
                continue
            new_label = _extend(label, next_node, graph, objective, duals, active_sr, farkas, restrictions, arc_customer_sets)
            labels_generated += 1
            if next_node == sink:
                result = accept_complete_label(new_label)
                if result is not None:
                    return result
                continue
            if farkas:
                accepted, rejected_count, purged_count = _insert_nondominated_farkas_label(
                    kept_farkas_labels,
                    new_label,
                    graph,
                    restrictions,
                    arc_customer_sets,
                )
                labels_dominated += rejected_count + purged_count
                labels_purged += purged_count
                if not accepted:
                    continue
            else:
                accepted, rejected_count, purged_count = _insert_nondominated_standard_label(
                    kept_standard_labels,
                    new_label,
                    graph,
                    objective,
                    duals,
                    restrictions,
                    arc_customer_sets,
                    deadline_counters,
                )
                labels_dominated += rejected_count + purged_count
                labels_purged += purged_count
                if not accepted:
                    continue
            new_key = _queue_key(new_label, graph, objective, shortest, bounds, farkas, use_standard_acceleration, deadline_counters)
            if not farkas and use_standard_acceleration:
                if new_key >= -pricing_tolerance:
                    labels_pruned += 1
                    standard_bound_pruned += 1
                    continue
            heappush(queue, (new_key, next(counter), new_label))
            max_queue_size = max(max_queue_size, len(queue))

    if returned_routes:
        best_route = _materialize_best_route(best_route, best_path, next_route_id, graph, objective)
        return _pricing_result(
            returned_routes,
            returned_costs,
            best_route,
            best_cost,
            labels_generated,
            labels_dominated,
            labels_pruned,
            labels_purged,
            stale_labels_skipped,
            standard_bound_pruned,
            farkas_bound_pruned,
            max_queue_size,
            complete_routes_generated,
            exact_completion=True,
            elapsed_seconds=time.time() - pricing_start,
                pricing_mode=pricing_mode,
                pricing_yield_ratio=pricing_yield_ratio,
                **deadline_diag_kwargs(),
            )
    if farkas:
        best_route = _materialize_best_route(best_route, best_path, next_route_id, graph, objective)
        best_cost_value = None if best_cost == float("inf") else best_cost
        return _pricing_result(
            [],
            [],
            best_route,
            best_cost_value,
            labels_generated,
            labels_dominated,
            labels_pruned,
            labels_purged,
            stale_labels_skipped,
            standard_bound_pruned,
            farkas_bound_pruned,
            max_queue_size,
            complete_routes_generated,
            exact_completion=True,
            elapsed_seconds=time.time() - pricing_start,
            pricing_mode=pricing_mode,
            pricing_yield_ratio=pricing_yield_ratio,
            **deadline_diag_kwargs(),
        )
    if best_route is None:
        best_route = _materialize_best_route(best_route, best_path, next_route_id, graph, objective)
        best_cost_value = None if best_cost == float("inf") else best_cost
        return _pricing_result(
            [],
            [],
            best_route,
            best_cost_value,
            labels_generated,
            labels_dominated,
            labels_pruned,
            labels_purged,
            stale_labels_skipped,
            standard_bound_pruned,
            farkas_bound_pruned,
            max_queue_size,
            complete_routes_generated,
            exact_completion=True,
            elapsed_seconds=time.time() - pricing_start,
            pricing_mode=pricing_mode,
            pricing_yield_ratio=pricing_yield_ratio,
            **deadline_diag_kwargs(),
        )
    best_route = _materialize_best_route(best_route, best_path, next_route_id, graph, objective)
    return _pricing_result(
        [],
        [],
        best_route,
        best_cost,
        labels_generated,
        labels_dominated,
        labels_pruned,
        labels_purged,
        stale_labels_skipped,
        standard_bound_pruned,
        farkas_bound_pruned,
        max_queue_size,
        complete_routes_generated,
        exact_completion=True,
        elapsed_seconds=time.time() - pricing_start,
        pricing_mode=pricing_mode,
        pricing_yield_ratio=pricing_yield_ratio,
        **deadline_diag_kwargs(),
    )


def _price_route_bidirectional(
    graph: TransformedGraph,
    objective: ObjectiveData,
    residual_customers: frozenset[str],
    restrictions: BranchRestrictions,
    duals: PricingDuals,
    next_route_id: int,
    farkas: bool = False,
    pricing_tolerance: float = 0.0,
    use_standard_acceleration: bool = True,
    stop_at_first_negative: bool = False,
    batch_size: int = 1,
    deadline: float | None = None,
    parallel_workers: int = 2,
    existing_routes: dict[tuple[str, ...], Route] | None = None,
    existing_column_paths: set[tuple[str, ...]] | None = None,
    small_join_pair_threshold: int = 5_000,
    small_join_cumulative_threshold: int = 250_000,
    max_join_bypass_calls: int = 1_000,
    small_dom_bucket_threshold: int = 100,
    small_dom_cumulative_threshold: int = 500_000,
    max_dom_bypass_calls: int = 2_000,
    join_payload_bin_width: float = 1.0,
    side_pool_batch_size: int = 0,
    pricing_mode: str = "productive",
    pricing_yield_ratio: float = 0.0,
    join_eval_budget: int = 0,
    pricing_certification_slice_seconds: float = 0.0,
    enable_join_lower_envelope: bool = True,
    join_generator_split_threshold: int = 50_000,
    join_generator_pair_batch_size: int = 10_000,
    enable_bucket_join_envelope: bool = True,
    enable_join_profile_cache: bool = True,
) -> PricingResult:
    if batch_size <= 0:
        raise ValueError("pricing batch size must be positive")
    if min(
        small_join_pair_threshold,
        small_join_cumulative_threshold,
        max_join_bypass_calls,
        small_dom_bucket_threshold,
        small_dom_cumulative_threshold,
        max_dom_bypass_calls,
        join_generator_split_threshold,
        join_generator_pair_batch_size,
    ) <= 0:
        raise ValueError("candidate-management thresholds must be positive")
    if join_payload_bin_width <= 0:
        raise ValueError("join payload bin width must be positive")
    if side_pool_batch_size < 0:
        raise ValueError("side-pool batch size must be nonnegative")
    if join_eval_budget < 0 or pricing_certification_slice_seconds < 0:
        raise ValueError("join evaluation budget and certification slice must be nonnegative")
    _validate_inequality_dual_signs(duals, pricing_tolerance)
    pricing_start = time.time()
    effective_deadline = deadline
    if pricing_mode == "closure" and pricing_certification_slice_seconds > 0.0:
        slice_deadline = pricing_start + pricing_certification_slice_seconds
        effective_deadline = slice_deadline if deadline is None else min(deadline, slice_deadline)
    instance = graph.instance
    source = instance.depot_source
    sink = instance.depot_sink
    active_sr = tuple(sorted(duals.nu))
    active_sr_version = len(active_sr)
    dual_solution_key = _dual_solution_key(duals)
    arc_customer_sets = {arc: graph.arc_customer_set(arc) for arc in graph.arcs}
    shortest = _shortest_truck_times(graph)
    bounds = _build_pricing_bounds(graph, duals, residual_customers, shortest)
    signature_cache = RouteSignatureCache()
    known_signature_costs = _known_signature_costs(
        existing_routes,
        existing_column_paths,
        graph,
        residual_customers,
        signature_cache,
        active_sr,
        active_sr_version,
    )
    source_cost = -duals.kappa if farkas else objective.coeffs.cost * instance.truck_cost - duals.kappa
    source_label = _Label(
        path=(source,),
        represented=frozenset(),
        truck_visited=frozenset({source}),
        truck_load=0.0,
        active_pad=None,
        active_pad_arrival=0.0,
        active_wait=0.0,
        block_count=0,
        physical_time=0.0,
        service_times=tuple(),
        sr_counts=tuple((triplet, 0) for triplet in active_sr),
        reduced_cost=source_cost,
        used_arcs=frozenset(),
        represented_mask=0,
        truck_node_mask=_truck_node_mask(frozenset({source}), graph),
    )
    sink_label = _build_backward_label(
        (sink,),
        graph,
        residual_customers,
        active_sr,
        restrictions,
        arc_customer_sets,
    )

    forward_counter = count()
    backward_counter = count()
    forward_queue: list[tuple[float, int, _Label]] = []
    backward_queue: list[tuple[int, int, _BackwardLabel]] = []
    source_key = _queue_key(source_label, graph, objective, shortest, bounds, farkas, use_standard_acceleration)
    heappush(forward_queue, (source_key, next(forward_counter), source_label))
    heappush(backward_queue, (0, next(backward_counter), sink_label))
    kept_standard: dict[str, list[_Label]] = {}
    kept_farkas: dict[str, list[_Label]] = {}
    loc = _physical_location(source_label)
    kept_standard[loc] = [source_label]
    kept_farkas[loc] = [source_label]
    forward_by_node: dict[str, list[_Label]] = {source: [source_label]}
    backward_by_node: dict[str, list[_BackwardLabel]] = {sink: [sink_label]}
    backward_dominance_index: dict[_DominanceBucketKey, list[_BackwardLabel]] = {
        _dominance_bucket_key(sink_label): [sink_label],
    }
    backward_paths = {sink_label.path}

    returned_routes: list[Route] = []
    returned_costs: list[float] = []
    returned_paths: set[tuple[str, ...]] = set()
    side_pool_routes: list[Route] = []
    side_pool_costs: list[float] = []
    side_pool_paths: set[tuple[str, ...]] = set()
    best_route: Route | None = None
    best_path: tuple[str, ...] | None = None
    best_cost = float("inf")
    forward_labels_generated = 1
    backward_labels_generated = 1
    backward_dominance_tests = 0
    backward_labels_dominated = 0
    backward_cost_function_build_time = 0.0
    backward_cost_function_eval_time = 0.0
    join_sr_correction_time = 0.0
    join_active_block_time = 0.0
    joined_reduced_cost_evaluations = 0
    backward_exclusive_resource_violations = 0
    labels_dominated = 0
    labels_pruned = 0
    labels_purged = 0
    stale_labels_skipped = 0
    standard_bound_pruned = 0
    farkas_bound_pruned = 0
    max_queue_size = 2
    complete_routes_generated = 0
    join_pairs_tested = 0
    joined_routes_accepted = 0
    forward_labeling_time = 0.0
    backward_labeling_time = 0.0
    join_time = 0.0
    parallel_calls = 0
    parallel_labeling_used = False
    dominance_counter = _DominanceCounter()
    join_prefilter_pairs = 0
    join_prefilter_rejected = 0
    join_bucket_pairs_considered = 0
    join_bucket_pairs_rejected = 0
    join_bucket_candidate_pairs = 0
    join_compatible_keys_generated = 0
    join_compatible_key_lookups = 0
    join_bucket_scans_avoided = 0
    join_key_generation_time = 0.0
    join_key_cache_hits = 0
    join_key_cache_misses = 0
    join_graph_build_time = 0.0
    join_subbucket_pairs_considered = 0
    join_subbucket_pairs_rejected = 0
    join_small_bypass_calls = 0
    join_local_bypass_calls = 0
    join_cumulative_bypass_calls = 0
    join_indexed_activation_count = 0
    join_work_estimate = 0
    join_candidate_pairs_accepted = 0
    join_label_pairs_materialized = 0
    join_full_decodes = 0
    lazy_rejected_before_decode = 0
    fully_decoded_routes = 0
    duplicate_equivalent_rejected = 0
    cost_dominated_rejected = 0
    duplicate_lookup_time = 0.0
    route_decode_time = 0.0
    reduced_cost_verification_time = 0.0
    side_pool_candidates_seen = 0
    side_pool_routes_retained = 0
    side_pool_routes_rejected_by_budget = 0
    join_key_cache: dict[_JoinLookupKey, tuple[_JoinLookupKey, ...]] = {}
    dominance_key_cache: dict[tuple[_DominanceBucketKey, bool], tuple[_DominanceBucketKey, ...]] = {}
    join_stage_counter = _JoinStageCounter()
    sticky_indexed_join = False
    join_eval_cache = _JoinEvalCache()
    join_pairs_key_compatible = 0
    join_pairs_after_bitset_filters = 0
    join_lower_envelope_rejects = 0
    join_bucket_lower_envelope_rejects = 0
    join_subbucket_lower_envelope_rejects = 0
    join_pair_lower_envelope_rejects = 0
    join_queue_pushes = 0
    join_queue_pops = 0
    join_generator_splits = 0
    join_materialized_pairs = 0
    join_exact_rc_evals = 0
    join_exact_rc_time = 0.0
    negative_routes_verified = 0
    join_eval_budget_exhausted = False

    def finish(
        exact_completion: bool,
        termination_reason: str | None = None,
        certification_mode: str | None = None,
    ) -> PricingResult:
        nonlocal best_route
        best_route = _materialize_best_route(best_route, best_path, next_route_id, graph, objective)
        best_cost_value = None if best_cost == float("inf") else best_cost
        return _pricing_result(
            returned_routes,
            returned_costs,
            best_route,
            best_cost_value,
            forward_labels_generated + backward_labels_generated,
            labels_dominated,
            labels_pruned,
            labels_purged,
            stale_labels_skipped,
            standard_bound_pruned,
            farkas_bound_pruned,
            max_queue_size,
            complete_routes_generated,
            exact_completion=exact_completion,
            termination_reason=termination_reason,
            certification_mode=certification_mode,
            elapsed_seconds=time.time() - pricing_start,
            pricing_engine="bidirectional_forward_backward",
            forward_labels_generated=forward_labels_generated,
            backward_labels_generated=backward_labels_generated,
            backward_dominance_tests=backward_dominance_tests,
            backward_labels_dominated=backward_labels_dominated,
            backward_cost_function_build_time_seconds=backward_cost_function_build_time,
            backward_cost_function_eval_time_seconds=backward_cost_function_eval_time,
            join_sr_correction_time_seconds=join_sr_correction_time,
            join_active_block_time_seconds=join_active_block_time,
            joined_reduced_cost_evaluations=joined_reduced_cost_evaluations,
            backward_dominance_cost_tests=dominance_counter.cost_function_tests,
            backward_dominance_cost_rejected=dominance_counter.cost_function_rejected,
            backward_exclusive_resource_violations=backward_exclusive_resource_violations,
            join_pairs_tested=join_pairs_tested,
            joined_routes_accepted=joined_routes_accepted,
            forward_labeling_time_seconds=forward_labeling_time,
            backward_labeling_time_seconds=backward_labeling_time,
            join_time_seconds=join_time,
            parallel_labeling_used=parallel_labeling_used,
            parallel_workers=parallel_workers,
            parallel_calls=parallel_calls,
            signature_cache_hits=signature_cache.stats.signature_cache_hits,
            signature_cache_misses=signature_cache.stats.signature_cache_misses,
            core_signature_cache_hits=signature_cache.stats.core_signature_cache_hits,
            core_signature_cache_misses=signature_cache.stats.core_signature_cache_misses,
            active_signature_cache_hits=signature_cache.stats.active_signature_cache_hits,
            active_signature_cache_misses=signature_cache.stats.active_signature_cache_misses,
            sr_coeff_cache_hits=signature_cache.stats.sr_coeff_cache_hits,
            sr_coeff_cache_misses=signature_cache.stats.sr_coeff_cache_misses,
            active_sr_key_cache_hits=signature_cache.stats.active_sr_key_cache_hits,
            active_sr_key_cache_misses=signature_cache.stats.active_sr_key_cache_misses,
            active_sr_coeffs_computed=signature_cache.stats.active_sr_coeffs_computed,
            triplet_masks_built=signature_cache.stats.triplet_masks_built,
            dominance_prefilter_pairs=dominance_counter.prefilter_pairs,
            dominance_prefilter_rejected=dominance_counter.prefilter_rejected,
            dominance_bucket_pairs_considered=dominance_counter.bucket_pairs_considered,
            dominance_bucket_pairs_rejected=dominance_counter.bucket_pairs_rejected,
            dominance_bucket_candidate_pairs=dominance_counter.bucket_candidate_pairs,
            dominance_compatible_keys_generated=dominance_counter.compatible_keys_generated,
            dominance_compatible_key_lookups=dominance_counter.compatible_key_lookups,
            dominance_bucket_scans_avoided=dominance_counter.bucket_scans_avoided,
            dominance_key_generation_time_seconds=dominance_counter.key_generation_time_seconds,
            backward_full_dominance_tests=dominance_counter.full_tests,
            join_prefilter_pairs=join_prefilter_pairs,
            join_prefilter_rejected=join_prefilter_rejected,
            join_bucket_pairs_considered=join_bucket_pairs_considered,
            join_bucket_pairs_rejected=join_bucket_pairs_rejected,
            join_bucket_candidate_pairs=join_bucket_candidate_pairs,
            join_compatible_keys_generated=join_compatible_keys_generated,
            join_compatible_key_lookups=join_compatible_key_lookups,
            join_bucket_scans_avoided=join_bucket_scans_avoided,
            join_key_generation_time_seconds=join_key_generation_time,
            join_key_cache_hits=join_key_cache_hits,
            join_key_cache_misses=join_key_cache_misses,
            join_graph_build_time_seconds=join_graph_build_time,
            join_subbucket_pairs_considered=join_subbucket_pairs_considered,
            join_subbucket_pairs_rejected=join_subbucket_pairs_rejected,
            join_small_bypass_calls=join_small_bypass_calls,
            join_local_bypass_calls=join_local_bypass_calls,
            join_cumulative_bypass_calls=join_cumulative_bypass_calls,
            join_indexed_activation_count=join_indexed_activation_count,
            join_work_estimate=join_work_estimate,
            join_candidate_pairs_accepted=join_candidate_pairs_accepted,
            join_activation_mode=_activation_mode(join_small_bypass_calls, join_indexed_activation_count),
            join_label_pairs_materialized=join_label_pairs_materialized,
            join_full_decodes=join_full_decodes,
            lazy_rejected_before_decode=lazy_rejected_before_decode,
            fully_decoded_routes=fully_decoded_routes,
            duplicate_equivalent_rejected=duplicate_equivalent_rejected + signature_cache.stats.duplicate_equivalent_rejected,
            cost_dominated_rejected=cost_dominated_rejected + signature_cache.stats.cost_dominated_rejected,
            signature_build_time_seconds=signature_cache.stats.signature_build_time,
            sr_coeff_build_time_seconds=signature_cache.stats.sr_coeff_build_time,
            duplicate_lookup_time_seconds=duplicate_lookup_time,
            route_decode_time_seconds=route_decode_time,
            reduced_cost_verification_time_seconds=reduced_cost_verification_time,
            dominance_key_cache_hits=dominance_counter.key_cache_hits,
            dominance_key_cache_misses=dominance_counter.key_cache_misses,
            dominance_small_bypass_calls=dominance_counter.small_bypass_calls,
            dominance_bypass_calls=dominance_counter.small_bypass_calls,
            dominance_indexed_activation_count=dominance_counter.indexed_activation_count,
            dominance_work_estimate=dominance_counter.work_estimate,
            dominance_activation_mode=_activation_mode(dominance_counter.small_bypass_calls, dominance_counter.indexed_activation_count),
            sticky_indexed_join_activations=join_stage_counter.sticky_indexed_activations,
            sticky_indexed_dominance_activations=dominance_counter.sticky_indexed_activations,
            join_stage_reject_key=join_stage_counter.reject_key,
            join_stage_reject_branch=join_stage_counter.reject_branch,
            join_stage_reject_customer=join_stage_counter.reject_customer,
            join_stage_reject_truck_node=join_stage_counter.reject_truck_node,
            join_stage_reject_payload=join_stage_counter.reject_payload,
            join_stage_reject_block=join_stage_counter.reject_block,
            join_stage_reject_reduced_cost=join_stage_counter.reject_reduced_cost,
            dominance_stage_reject_key=dominance_counter.stage_reject_key,
            dominance_stage_reject_branch=dominance_counter.stage_reject_branch,
            dominance_stage_reject_customer=dominance_counter.stage_reject_customer,
            dominance_stage_reject_truck_node=dominance_counter.stage_reject_truck_node,
            dominance_stage_reject_payload=dominance_counter.stage_reject_payload,
            dominance_stage_reject_block=dominance_counter.stage_reject_block,
            dominance_stage_reject_time=dominance_counter.stage_reject_time,
            dominance_stage_reject_cost=dominance_counter.stage_reject_cost,
            pricing_mode=pricing_mode,
            pricing_yield_ratio=pricing_yield_ratio,
            side_pool_routes=side_pool_routes,
            side_pool_reduced_costs=side_pool_costs,
            side_pool_candidates_seen=side_pool_candidates_seen,
            side_pool_routes_retained=side_pool_routes_retained,
            side_pool_routes_rejected_by_budget=side_pool_routes_rejected_by_budget,
            join_pairs_key_compatible=join_pairs_key_compatible,
            join_pairs_after_bitset_filters=join_pairs_after_bitset_filters,
            join_lower_envelope_rejects=join_lower_envelope_rejects,
            join_bucket_lower_envelope_rejects=join_bucket_lower_envelope_rejects,
            join_subbucket_lower_envelope_rejects=join_subbucket_lower_envelope_rejects,
            join_pair_lower_envelope_rejects=join_pair_lower_envelope_rejects,
            join_queue_pushes=join_queue_pushes,
            join_queue_pops=join_queue_pops,
            join_generator_queue_pushes=join_queue_pushes,
            join_generator_queue_pops=join_queue_pops,
            join_generator_splits=join_generator_splits,
            join_materialized_pairs=join_materialized_pairs,
            join_exact_rc_evals=join_exact_rc_evals,
            join_exact_rc_time_seconds=join_exact_rc_time,
            interface_cache_hits=join_eval_cache.hits,
            interface_cache_misses=join_eval_cache.misses,
            suffix_profile_cache_hits=join_eval_cache.suffix_profile_hits,
            suffix_profile_cache_misses=join_eval_cache.suffix_profile_misses,
            interface_profile_cache_hits=join_eval_cache.interface_profile_hits,
            interface_profile_cache_misses=join_eval_cache.interface_profile_misses,
            productive_calls=1 if pricing_mode == "productive" else 0,
            certification_calls=1 if pricing_mode == "closure" else 0,
            negative_routes_verified=negative_routes_verified,
            negative_routes_inserted=len(returned_routes),
        )

    def accept_candidates(candidates: list[_RouteCandidate]) -> bool:
        nonlocal best_route, best_path, best_cost, complete_routes_generated, joined_routes_accepted
        nonlocal lazy_rejected_before_decode, fully_decoded_routes, duplicate_equivalent_rejected, cost_dominated_rejected
        nonlocal duplicate_lookup_time, route_decode_time, reduced_cost_verification_time, join_full_decodes
        nonlocal side_pool_candidates_seen, side_pool_routes_retained, side_pool_routes_rejected_by_budget
        nonlocal negative_routes_verified
        batch_ready = False
        for candidate in sorted(candidates, key=lambda item: (item.reduced_cost, item.path)):
            if candidate.reduced_cost < best_cost:
                best_cost = candidate.reduced_cost
                best_path = candidate.path
                best_route = None
            complete_routes_generated += 1
            if candidate.reduced_cost >= -pricing_tolerance or candidate.path in returned_paths:
                lazy_rejected_before_decode += 1
                continue
            lookup_start = time.time()
            estimated_cost = _candidate_route_cost_from_reduced_cost(candidate, duals, farkas)
            signature = route_signature_from_resources(
                candidate.served,
                candidate.truck_served,
                candidate.pad_served,
                candidate.used_arcs,
                graph,
                residual_customers,
                signature_cache,
                active_sr=active_sr,
                active_sr_version=active_sr_version,
                route_cost=estimated_cost,
            )
            coeff_signature = route_coefficient_signature(signature)
            incumbent_cost = known_signature_costs.get(coeff_signature)
            duplicate_lookup_time += time.time() - lookup_start
            if estimated_cost is not None and incumbent_cost is not None and incumbent_cost <= estimated_cost + PAPER_DOMINANCE_TOLERANCE:
                if abs(incumbent_cost - estimated_cost) <= COST_SIGNATURE_TOLERANCE:
                    duplicate_equivalent_rejected += 1
                else:
                    cost_dominated_rejected += 1
                lazy_rejected_before_decode += 1
                continue
            decode_start = time.time()
            route = route_from_path(next_route_id + len(returned_routes), candidate.path, graph, objective)
            route_decode_time += time.time() - decode_start
            fully_decoded_routes += 1
            if candidate.joined:
                join_full_decodes += 1
            verify_start = time.time()
            if not route.served or not route.served.issubset(residual_customers):
                raise RuntimeError("decoded pricing route violates residual-customer scope")
            if not restrictions.route_allowed(route, arc_customer_sets):
                raise RuntimeError("decoded pricing route violates branch restrictions")
            direct_reduced_cost = route_farkas_reduced_cost(route, duals) if farkas else route_reduced_cost(route, duals)
            if abs(direct_reduced_cost - candidate.reduced_cost) > max(1e-7, 10.0 * pricing_tolerance):
                raise RuntimeError("label-computed reduced cost disagrees with direct decoded route cost")
            if direct_reduced_cost < -pricing_tolerance:
                negative_routes_verified += 1
            duplicate, signature = _duplicate_dominated_by_existing(
                route,
                known_signature_costs,
                graph,
                residual_customers,
                signature_cache,
                active_sr,
                active_sr_version,
            )
            reduced_cost_verification_time += time.time() - verify_start
            if duplicate:
                lazy_rejected_before_decode += 1
                continue
            coeff_signature = route_coefficient_signature(signature)
            if candidate.path not in returned_paths and len(returned_routes) < batch_size:
                returned_routes.append(route)
                returned_costs.append(direct_reduced_cost)
                returned_paths.add(candidate.path)
                known_signature_costs[coeff_signature] = min(
                    known_signature_costs.get(coeff_signature, float("inf")),
                    route.cost,
                )
                if candidate.joined:
                    joined_routes_accepted += 1
                if stop_at_first_negative or len(returned_routes) >= batch_size:
                    batch_ready = True
                    if stop_at_first_negative and side_pool_batch_size == 0:
                        return True
                continue
            if (
                side_pool_batch_size > 0
                and candidate.path not in returned_paths
                and candidate.path not in side_pool_paths
            ):
                side_pool_candidates_seen += 1
                if len(side_pool_routes) < side_pool_batch_size:
                    side_pool_routes.append(route)
                    side_pool_costs.append(direct_reduced_cost)
                    side_pool_paths.add(candidate.path)
                    side_pool_routes_retained += 1
                else:
                    side_pool_routes_rejected_by_budget += 1
        return batch_ready

    def join_candidates(
        forward_labels: list[_Label],
        backward_labels: list[_BackwardLabel],
    ) -> list[_RouteCandidate]:
        nonlocal join_pairs_tested, join_time, parallel_calls, parallel_labeling_used
        nonlocal join_prefilter_pairs, join_prefilter_rejected
        nonlocal join_bucket_pairs_considered, join_bucket_pairs_rejected, join_bucket_candidate_pairs, join_label_pairs_materialized
        nonlocal join_compatible_keys_generated, join_compatible_key_lookups, join_bucket_scans_avoided, join_key_generation_time
        nonlocal join_key_cache_hits, join_key_cache_misses, join_graph_build_time
        nonlocal join_subbucket_pairs_considered, join_subbucket_pairs_rejected, join_small_bypass_calls
        nonlocal join_local_bypass_calls, join_cumulative_bypass_calls, join_indexed_activation_count
        nonlocal join_work_estimate, join_candidate_pairs_accepted
        nonlocal sticky_indexed_join
        nonlocal backward_cost_function_eval_time, joined_reduced_cost_evaluations, join_sr_correction_time, join_active_block_time
        nonlocal join_pairs_key_compatible, join_pairs_after_bitset_filters, join_lower_envelope_rejects
        nonlocal join_bucket_lower_envelope_rejects, join_subbucket_lower_envelope_rejects, join_pair_lower_envelope_rejects
        nonlocal join_queue_pushes, join_queue_pops, join_generator_splits, join_materialized_pairs
        nonlocal join_exact_rc_evals, join_exact_rc_time
        nonlocal join_eval_budget_exhausted
        if not forward_labels or not backward_labels:
            return []
        join_start = time.time()
        current_join_work = _estimate_join_work(forward_by_node, backward_by_node)
        join_work_estimate = max(join_work_estimate, current_join_work)
        local_pair_count = len(forward_labels) * len(backward_labels)
        if (
            not sticky_indexed_join
            and (
                local_pair_count > small_join_pair_threshold
                or current_join_work > small_join_cumulative_threshold
                or join_small_bypass_calls >= max_join_bypass_calls
            )
        ):
            sticky_indexed_join = True
            join_stage_counter.sticky_indexed_activations += 1
        bucketed = _bucketed_join_generators(
            forward_labels,
            backward_labels,
            graph,
            duals,
            farkas,
            compatible_key_cache=join_key_cache,
            small_join_pair_threshold=small_join_pair_threshold,
            small_join_cumulative_threshold=small_join_cumulative_threshold,
            max_join_bypass_calls=max_join_bypass_calls,
            previous_join_bypass_calls=join_small_bypass_calls,
            cumulative_pair_count=current_join_work,
            payload_bin_width=join_payload_bin_width,
            force_indexed=sticky_indexed_join,
            stage_counter=join_stage_counter,
            pricing_tolerance=pricing_tolerance,
            enable_join_lower_envelope=enable_join_lower_envelope,
            enable_bucket_join_envelope=enable_bucket_join_envelope,
        )
        join_bucket_pairs_considered += bucketed["bucket_pairs_considered"]
        join_bucket_pairs_rejected += bucketed["bucket_pairs_rejected"]
        join_bucket_candidate_pairs += bucketed["bucket_candidate_pairs"]
        join_bucket_lower_envelope_rejects += bucketed["bucket_lower_envelope_rejects"]
        join_lower_envelope_rejects += bucketed["bucket_lower_envelope_rejects"]
        join_compatible_keys_generated += bucketed["compatible_keys_generated"]
        join_compatible_key_lookups += bucketed["compatible_key_lookups"]
        join_bucket_scans_avoided += bucketed["bucket_scans_avoided"]
        join_key_generation_time += bucketed["key_generation_time_seconds"]
        join_key_cache_hits += bucketed["key_cache_hits"]
        join_key_cache_misses += bucketed["key_cache_misses"]
        join_graph_build_time += bucketed["join_graph_build_time_seconds"]
        join_subbucket_pairs_considered += bucketed["subbucket_pairs_considered"]
        join_subbucket_pairs_rejected += bucketed["subbucket_pairs_rejected"]
        join_small_bypass_calls += bucketed["small_bypass_calls"]
        join_local_bypass_calls += bucketed["local_bypass_calls"]
        join_cumulative_bypass_calls += bucketed["cumulative_bypass_calls"]
        join_indexed_activation_count += bucketed["indexed_activation_count"]
        join_work_estimate = max(join_work_estimate, int(bucketed["join_work_estimate"]))
        join_candidate_pairs_accepted += bucketed["candidate_pairs_accepted"]
        join_prefilter_rejected += bucketed["label_pairs_rejected"]
        generators = bucketed["generators"]
        if not generators:
            join_time += time.time() - join_start
            return []
        join_pairs_key_compatible += int(bucketed["bucket_candidate_pairs"])
        queue: list[tuple[float, int, _JoinGenerator]] = []
        queue_counter = count()
        for generator in generators:
            heappush(queue, (generator.lower_bound, next(queue_counter), generator))
            join_queue_pushes += 1

        candidates: list[_RouteCandidate] = []
        negative_candidates = 0
        while queue:
            if effective_deadline is not None and time.time() >= effective_deadline:
                break
            if join_eval_budget > 0 and join_exact_rc_evals >= join_eval_budget and pricing_mode != "closure":
                join_eval_budget_exhausted = True
                break
            _, _, generator = heappop(queue)
            join_queue_pops += 1
            split_threshold = min(join_generator_split_threshold, join_generator_pair_batch_size)
            if generator.level == "bucket" and generator.estimated_pair_count > split_threshold:
                subgenerators = _split_join_generator(
                    generator,
                    graph,
                    duals,
                    farkas,
                    join_payload_bin_width,
                    pricing_tolerance,
                    enable_join_lower_envelope,
                    enable_bucket_join_envelope,
                )
                if (
                    int(subgenerators["subbucket_pairs_considered"]) > 1
                    or int(subgenerators["subbucket_lower_envelope_rejects"]) > 0
                    or int(subgenerators["subbucket_pairs_rejected"]) > 0
                ):
                    join_generator_splits += 1
                    join_subbucket_pairs_considered += subgenerators["subbucket_pairs_considered"]
                    join_subbucket_pairs_rejected += subgenerators["subbucket_pairs_rejected"]
                    join_subbucket_lower_envelope_rejects += subgenerators["subbucket_lower_envelope_rejects"]
                    join_lower_envelope_rejects += subgenerators["subbucket_lower_envelope_rejects"]
                    _increment_join_stage(join_stage_counter, "reduced_cost", int(subgenerators["subbucket_lower_envelope_rejects"]))
                    for subgenerator in subgenerators["generators"]:
                        heappush(queue, (subgenerator.lower_bound, next(queue_counter), subgenerator))
                        join_queue_pushes += 1
                    continue
            for forward_label, backward_label in _iter_join_generator_pairs(generator, graph):
                if effective_deadline is not None and time.time() >= effective_deadline:
                    break
                if join_eval_budget > 0 and join_exact_rc_evals >= join_eval_budget and pricing_mode != "closure":
                    join_eval_budget_exhausted = True
                    break
                join_materialized_pairs += 1
                join_label_pairs_materialized += 1
                join_pairs_tested += 1
                join_prefilter_pairs += 1
                rejection_stage = _join_rejection_stage(forward_label, backward_label, graph)
                if rejection_stage is not None:
                    join_prefilter_rejected += 1
                    _increment_join_stage(join_stage_counter, rejection_stage)
                    continue
                join_pairs_after_bitset_filters += 1
                join_lb = _join_lower_bound(
                    forward_label,
                    backward_label,
                    graph,
                    duals,
                    farkas,
                    enable_join_lower_envelope=enable_join_lower_envelope,
                )
                if not farkas and enable_join_lower_envelope and join_lb >= -pricing_tolerance:
                    join_lower_envelope_rejects += 1
                    join_pair_lower_envelope_rejects += 1
                    _increment_join_stage(join_stage_counter, "reduced_cost")
                    continue
                eval_start = time.time()
                reduced_cost = _joined_reduced_cost_cached(
                    forward_label,
                    backward_label,
                    graph,
                    objective,
                    duals,
                    farkas,
                    join_eval_cache,
                    active_sr_version,
                    dual_solution_key,
                    enable_cache=enable_join_profile_cache,
                )
                eval_elapsed = time.time() - eval_start
                join_exact_rc_time += eval_elapsed
                backward_cost_function_eval_time += eval_elapsed
                join_sr_correction_time += eval_elapsed
                join_active_block_time += eval_elapsed
                join_exact_rc_evals += 1
                joined_reduced_cost_evaluations += 1
                candidate = _join_forward_backward(
                    forward_label,
                    backward_label,
                    graph,
                    objective,
                    residual_customers,
                    restrictions,
                    arc_customer_sets,
                    duals,
                    farkas,
                    reduced_cost=reduced_cost,
                )
                if candidate is None:
                    continue
                candidates.append(candidate)
                if reduced_cost < -pricing_tolerance:
                    negative_candidates += 1
                    if negative_candidates >= batch_size:
                        break
            if negative_candidates >= batch_size or join_eval_budget_exhausted:
                break
        join_time += time.time() - join_start
        return candidates

    def _evaluate_join_pair(pair: tuple[_Label, _BackwardLabel]) -> _RouteCandidate | None:
        forward_label, backward_label = pair
        candidate = _join_forward_backward(
            forward_label,
            backward_label,
            graph,
            objective,
            residual_customers,
            restrictions,
            arc_customer_sets,
            duals,
            farkas,
        )
        if candidate is None:
            return None
        return candidate

    with ThreadPoolExecutor(max_workers=parallel_workers) as executor:
        while forward_queue or backward_queue:
            if effective_deadline is not None and time.time() >= effective_deadline:
                if returned_routes:
                    return finish(
                        exact_completion=False,
                        termination_reason="time_limit_with_columns",
                        certification_mode="not_certified_time_limit",
                    )
                raise PricingTimeLimitReached(
                    finish(
                        exact_completion=False,
                        termination_reason="time_limit_unresolved",
                        certification_mode="not_certified_time_limit",
                    ).diagnostics
                )

            forward_label: _Label | None = None
            while forward_queue and forward_label is None:
                _, _, candidate = heappop(forward_queue)
                key = _physical_location(candidate)
                kept = kept_farkas if farkas else kept_standard
                if candidate not in kept.get(key, []):
                    stale_labels_skipped += 1
                    continue
                forward_label = candidate
            backward_label: _BackwardLabel | None = None
            while backward_queue and backward_label is None:
                _, _, candidate = heappop(backward_queue)
                if candidate not in backward_by_node.get(candidate.path[0], []):
                    stale_labels_skipped += 1
                    continue
                backward_label = candidate

            if forward_label is None and backward_label is None:
                continue
            futures = []
            if forward_label is not None and backward_label is not None and parallel_workers > 1:
                parallel_calls += 1
                parallel_labeling_used = True
                futures.append(("forward", executor.submit(
                    _expand_forward_label,
                    forward_label,
                    graph,
                    objective,
                    residual_customers,
                    restrictions,
                    arc_customer_sets,
                    duals,
                    active_sr,
                    farkas,
                )))
                futures.append(("backward", executor.submit(
                    _expand_backward_label,
                    backward_label,
                    graph,
                    residual_customers,
                    active_sr,
                    restrictions,
                    arc_customer_sets,
                )))
                forward_expansion: _ForwardExpansion | None = None
                backward_expansion: _BackwardExpansion | None = None
                for kind, future in futures:
                    if kind == "forward":
                        forward_expansion = future.result()
                    else:
                        backward_expansion = future.result()
            else:
                forward_expansion = (
                    _expand_forward_label(
                        forward_label,
                        graph,
                        objective,
                        residual_customers,
                        restrictions,
                        arc_customer_sets,
                        duals,
                        active_sr,
                        farkas,
                    )
                    if forward_label is not None
                    else None
                )
                backward_expansion = (
                    _expand_backward_label(
                        backward_label,
                        graph,
                        residual_customers,
                        active_sr,
                        restrictions,
                        arc_customer_sets,
                    )
                    if backward_label is not None
                    else None
                )

            candidate_routes: list[_RouteCandidate] = []
            if forward_expansion is not None:
                forward_labels_generated += forward_expansion.generated
                forward_labeling_time += forward_expansion.elapsed_seconds
                for complete_label in forward_expansion.complete_labels:
                    candidate_routes.append(_candidate_from_forward_label(complete_label, graph, objective, duals, farkas))
                for new_label in sorted(forward_expansion.labels, key=lambda item: (item.path, item.reduced_cost)):
                    insert_ok = True
                    if farkas:
                        inserted, rejected_count, purged = _insert_nondominated_farkas_label(
                            kept_farkas,
                            new_label,
                            graph,
                            restrictions,
                            arc_customer_sets,
                        )
                    else:
                        inserted, rejected_count, purged = _insert_nondominated_standard_label(
                            kept_standard,
                            new_label,
                            graph,
                            objective,
                            duals,
                            restrictions,
                            arc_customer_sets,
                        )
                    if not inserted:
                        labels_dominated += rejected_count + purged
                        insert_ok = False
                    else:
                        labels_dominated += purged
                    labels_purged += purged
                    if insert_ok:
                        key = _queue_key(new_label, graph, objective, shortest, bounds, farkas, use_standard_acceleration)
                        if not farkas and use_standard_acceleration and key >= -pricing_tolerance:
                            standard_bound_pruned += 1
                            labels_pruned += 1
                            insert_ok = False
                    if not insert_ok:
                        continue
                    heappush(forward_queue, (key, next(forward_counter), new_label))
                    forward_by_node.setdefault(new_label.path[-1], []).append(new_label)
                    candidate_routes.extend(join_candidates([new_label], list(backward_by_node.get(new_label.path[-1], ()))))

            if backward_expansion is not None:
                backward_labels_generated += backward_expansion.generated
                backward_labeling_time += backward_expansion.elapsed_seconds
                backward_cost_function_build_time += backward_expansion.cost_function_build_time_seconds
                backward_exclusive_resource_violations += backward_expansion.exclusive_resource_violations
                for suffix in sorted(backward_expansion.labels, key=lambda item: item.path):
                    inserted, rejected_count, purged_count, dominance_tests = _insert_nondominated_backward_label(
                        backward_by_node,
                        backward_paths,
                        suffix,
                        graph,
                        objective,
                        restrictions,
                        arc_customer_sets,
                        duals,
                        farkas,
                        dominance_counter,
                        dominance_index=backward_dominance_index,
                        dominance_key_cache=dominance_key_cache,
                        small_dom_bucket_threshold=small_dom_bucket_threshold,
                        small_dom_cumulative_threshold=small_dom_cumulative_threshold,
                        max_dom_bypass_calls=max_dom_bypass_calls,
                    )
                    backward_dominance_tests += dominance_tests
                    backward_labels_dominated += rejected_count + purged_count
                    if not inserted:
                        continue
                    heappush(backward_queue, (len(suffix.path), next(backward_counter), suffix))
                    candidate_routes.extend(join_candidates(list(forward_by_node.get(suffix.path[0], ())), [suffix]))

            if accept_candidates(candidate_routes):
                negative_batch_reason = (
                    "closure_negative_batch_found"
                    if pricing_mode == "closure"
                    else "productive_batch_found"
                )
                negative_batch_certification = (
                    "not_certified_closure_returned_columns"
                    if pricing_mode == "closure"
                    else "not_certified_productive"
                )
                return finish(
                    exact_completion=False,
                    termination_reason=negative_batch_reason,
                    certification_mode=negative_batch_certification,
                )
            max_queue_size = max(max_queue_size, len(forward_queue) + len(backward_queue))

    if join_eval_budget_exhausted:
        raise PricingTimeLimitReached(
            finish(
                exact_completion=False,
                termination_reason="time_limit_unresolved",
                certification_mode="not_certified_join_eval_budget",
            ).diagnostics
        )
    return finish(
        exact_completion=True,
        termination_reason="complete_label_and_join_exhaustion",
        certification_mode="bidirectional_complete_meet_node",
    )


def _expand_forward_label(
    label: _Label,
    graph: TransformedGraph,
    objective: ObjectiveData,
    residual_customers: frozenset[str],
    restrictions: BranchRestrictions,
    arc_customer_sets: dict[tuple[str, str], frozenset[str]],
    duals: PricingDuals,
    active_sr: tuple[tuple[str, str, str], ...],
    farkas: bool,
) -> _ForwardExpansion:
    start = time.time()
    instance = graph.instance
    sink = instance.depot_sink
    labels: list[_Label] = []
    complete_labels: list[_Label] = []
    generated = 0
    for next_node in graph.out_arcs[label.path[-1]]:
        if not _extension_allowed(label, next_node, graph, residual_customers, restrictions):
            continue
        new_label = _extend(label, next_node, graph, objective, duals, active_sr, farkas, restrictions, arc_customer_sets)
        generated += 1
        if next_node == sink:
            if new_label.represented and _complete_allowed(new_label, graph, restrictions, arc_customer_sets):
                complete_labels.append(new_label)
        else:
            labels.append(new_label)
    return _ForwardExpansion(tuple(labels), tuple(complete_labels), generated, time.time() - start)


def _expand_backward_label(
    label: _BackwardLabel,
    graph: TransformedGraph,
    residual_customers: frozenset[str],
    active_sr: tuple[tuple[str, str, str], ...],
    restrictions: BranchRestrictions,
    arc_customer_sets: dict[tuple[str, str], frozenset[str]],
) -> _BackwardExpansion:
    start = time.time()
    source = graph.instance.depot_source
    if label.path[0] == source:
        return _BackwardExpansion(tuple(), 0, time.time() - start)
    labels: list[_BackwardLabel] = []
    cost_function_build_time = 0.0
    exclusive_resource_violations = 0
    for prev_node in graph.in_arcs[label.path[0]]:
        extension_start = time.time()
        suffix = _extend_backward_label(prev_node, label, graph, residual_customers, active_sr, restrictions, arc_customer_sets)
        extension_elapsed = time.time() - extension_start
        if suffix is not None:
            labels.append(suffix)
            cost_function_build_time += extension_elapsed
        else:
            exclusive_resource_violations += 1
    return _BackwardExpansion(
        tuple(labels),
        len(labels),
        time.time() - start,
        cost_function_build_time_seconds=cost_function_build_time,
        exclusive_resource_violations=exclusive_resource_violations,
    )


def _extend_backward_label(
    predecessor: str,
    label: _BackwardLabel,
    graph: TransformedGraph,
    residual_customers: frozenset[str],
    active_sr: tuple[tuple[str, str, str], ...],
    restrictions: BranchRestrictions,
    arc_customer_sets: dict[tuple[str, str], frozenset[str]],
) -> _BackwardLabel | None:
    instance = graph.instance
    if not label.path:
        return None
    suffix_start = label.path[0]
    arc = (predecessor, suffix_start)
    if arc not in graph.arcs:
        return None
    if predecessor == instance.depot_sink or instance.depot_source in label.path[1:]:
        return None

    represented = set(label.represented)
    truck_visited = set(label.truck_visited)
    truck_load = label.truck_load

    if suffix_start in instance.customers + instance.hubs:
        if suffix_start in truck_visited:
            return None
        truck_visited.add(suffix_start)
    if predecessor in instance.customers + instance.hubs and predecessor in truck_visited:
        return None

    truck_served = set(label.truck_served)
    pad_served = set(label.pad_served)
    if is_customer_representation(suffix_start, instance):
        customer = served_customer(suffix_start)
        if customer not in residual_customers or customer in represented:
            return None
        if customer in restrictions.truck_service and is_duplicate(suffix_start):
            return None
        if customer in restrictions.drone_service and suffix_start == customer:
            return None
        required_pads = [hub for hub, c in restrictions.pad_required if c == customer]
        if required_pads and (not is_duplicate(suffix_start) or duplicate_hub(suffix_start) not in required_pads):
            return None
        if is_duplicate(suffix_start) and (duplicate_hub(suffix_start), customer) in restrictions.pad_forbidden:
            return None
        for p, q in restrictions.separate_pairs:
            if customer == p and q in represented:
                return None
            if customer == q and p in represented:
                return None
        if is_duplicate(suffix_start):
            hub = duplicate_hub(suffix_start)
            if (hub, customer) not in instance.drone_arcs:
                return None
            pad_served.add((hub, customer))
            if label.leading_block_hub is not None and label.leading_block_hub != hub:
                return None
            leading_block_hub = hub
            leading_block_count = label.leading_block_count + 1
            leading_block_wait = max(label.leading_block_wait, instance.drone_trip_time[(hub, customer)])
        else:
            truck_served.add(customer)
            leading_block_hub = None
            leading_block_count = 0
            leading_block_wait = 0.0
        represented.add(customer)
        truck_load += instance.demand[customer]
        if truck_load > instance.truck_payload + PAYLOAD_TOLERANCE:
            return None
    else:
        leading_block_hub = None
        leading_block_count = 0
        leading_block_wait = 0.0

    if leading_block_count > instance.drones_per_truck:
        return None

    path = (predecessor,) + label.path
    profile, profile_truck_served, profile_pad_served = _backward_profile(path, graph)
    truck_served = set(profile_truck_served)
    pad_served = set(profile_pad_served)
    represented_frozen = frozenset(represented)
    truck_visited_frozen = frozenset(truck_visited)
    sr_counts = tuple(
        (triplet, len(represented_frozen.intersection(triplet)))
        for triplet in active_sr
    )
    used_arcs = label.used_arcs | {arc}
    branch_state = _backward_branch_state(represented_frozen, used_arcs, restrictions, arc_customer_sets)
    return _BackwardLabel(
        path=path,
        represented=represented_frozen,
        truck_visited=truck_visited_frozen,
        truck_load=truck_load,
        used_arcs=used_arcs,
        truck_served=frozenset(truck_served),
        pad_served=frozenset(pad_served),
        sr_counts=sr_counts,
        branch_state=branch_state,
        profile=profile,
        cost_function=_BackwardCostFunction(profile, represented_frozen, sr_counts),
        lower_envelope=_BackwardCostLowerEnvelope(represented_frozen),
        leading_block_hub=leading_block_hub,
        leading_block_count=leading_block_count,
        leading_block_wait=leading_block_wait,
        represented_mask=_customer_mask(represented_frozen, graph),
        truck_node_mask=_truck_node_mask(truck_visited_frozen, graph),
    )


def _try_build_backward_label(
    path: tuple[str, ...],
    graph: TransformedGraph,
    residual_customers: frozenset[str],
    active_sr: tuple[tuple[str, str, str], ...] = tuple(),
    restrictions: BranchRestrictions | None = None,
    arc_customer_sets: dict[tuple[str, str], frozenset[str]] | None = None,
) -> _BackwardLabel | None:
    if restrictions is None:
        restrictions = BranchRestrictions()
    if arc_customer_sets is None:
        arc_customer_sets = {arc: graph.arc_customer_set(arc) for arc in graph.arcs}
    instance = graph.instance
    if not path or path[-1] != instance.depot_sink:
        return None
    if instance.depot_source in path[1:]:
        return None
    represented: set[str] = set()
    truck_visited: set[str] = set()
    load = 0.0
    active_block_hub: str | None = None
    active_block_count = 0
    for arc in zip(path, path[1:]):
        if arc not in graph.arcs:
            return None
        i, j = arc
        if arc in graph.hub_duplicate_arcs:
            active_block_hub = i
            active_block_count = 1
        elif arc in graph.duplicate_duplicate_arcs:
            hub = duplicate_hub(j)
            if active_block_hub is None and is_duplicate(i):
                active_block_hub = duplicate_hub(i)
            if active_block_hub != hub:
                return None
            active_block_count += 1
        elif arc in graph.duplicate_regular_arcs:
            hub = duplicate_hub(i)
            if active_block_hub is not None and active_block_hub != hub:
                return None
            active_block_hub = None
            active_block_count = 0
        elif arc in graph.truck_arcs:
            active_block_hub = None
            active_block_count = 0
        if active_block_count > instance.drones_per_truck:
            return None
    for node in path[1:]:
        if node in instance.customers + instance.hubs:
            if node in truck_visited:
                return None
            truck_visited.add(node)
        if is_customer_representation(node, instance):
            customer = served_customer(node)
            if customer not in residual_customers or customer in represented:
                return None
            if is_duplicate(node) and (duplicate_hub(node), customer) not in instance.drone_arcs:
                return None
            represented.add(customer)
            load += instance.demand[customer]
            if load > instance.truck_payload + PAYLOAD_TOLERANCE:
                return None
    leading_block_hub, leading_block_count, leading_block_wait = _backward_leading_block(path, graph)
    profile, truck_served, pad_served = _backward_profile(path, graph)
    sr_counts = tuple(
        (triplet, len(frozenset(represented).intersection(triplet)))
        for triplet in active_sr
    )
    branch_state = _backward_branch_state(
        frozenset(represented),
        frozenset(zip(path, path[1:])),
        restrictions,
        arc_customer_sets,
    )
    represented_frozen = frozenset(represented)
    truck_visited_frozen = frozenset(truck_visited)
    return _BackwardLabel(
        path=path,
        represented=represented_frozen,
        truck_visited=truck_visited_frozen,
        truck_load=load,
        used_arcs=frozenset(zip(path, path[1:])),
        truck_served=truck_served,
        pad_served=pad_served,
        sr_counts=sr_counts,
        branch_state=branch_state,
        profile=profile,
        cost_function=_BackwardCostFunction(profile, represented_frozen, sr_counts),
        lower_envelope=_BackwardCostLowerEnvelope(represented_frozen),
        leading_block_hub=leading_block_hub,
        leading_block_count=leading_block_count,
        leading_block_wait=leading_block_wait,
        represented_mask=_customer_mask(represented_frozen, graph),
        truck_node_mask=_truck_node_mask(truck_visited_frozen, graph),
    )


def _backward_leading_block(path: tuple[str, ...], graph: TransformedGraph) -> tuple[str | None, int, float]:
    hub: str | None = None
    count = 0
    wait = 0.0
    instance = graph.instance
    for i, j in zip(path, path[1:]):
        arc = (i, j)
        if arc in graph.hub_duplicate_arcs:
            hub = i
            count += 1
            wait = max(wait, instance.drone_trip_time[(hub, duplicate_customer(j))])
        elif arc in graph.duplicate_duplicate_arcs:
            if hub is None and is_duplicate(i):
                hub = duplicate_hub(i)
            if hub != duplicate_hub(j):
                return None, 0, 0.0
            count += 1
            wait = max(wait, instance.drone_trip_time[(hub, duplicate_customer(j))])
        else:
            break
    if count == 0:
        return None, 0, 0.0
    return hub, count, wait


def _build_backward_label(
    path: tuple[str, ...],
    graph: TransformedGraph,
    residual_customers: frozenset[str],
    active_sr: tuple[tuple[str, str, str], ...] = tuple(),
    restrictions: BranchRestrictions | None = None,
    arc_customer_sets: dict[tuple[str, str], frozenset[str]] | None = None,
) -> _BackwardLabel:
    label = _try_build_backward_label(path, graph, residual_customers, active_sr, restrictions, arc_customer_sets)
    if label is None:
        raise ValueError(f"invalid backward suffix path: {path}")
    return label


def _insert_nondominated_backward_label(
    backward_by_node: dict[str, list[_BackwardLabel]],
    backward_paths: set[tuple[str, ...]],
    label: _BackwardLabel,
    graph: TransformedGraph,
    objective: ObjectiveData,
    restrictions: BranchRestrictions,
    arc_customer_sets: dict[tuple[str, str], frozenset[str]],
    duals: PricingDuals,
    farkas: bool,
    counter: _DominanceCounter | None = None,
    dominance_index: dict[_DominanceBucketKey, list[_BackwardLabel]] | None = None,
    dominance_key_cache: dict[tuple[_DominanceBucketKey, bool], tuple[_DominanceBucketKey, ...]] | None = None,
    small_dom_bucket_threshold: int = 100,
    small_dom_cumulative_threshold: int = 500_000,
    max_dom_bypass_calls: int = 2_000,
) -> tuple[bool, int, int, int]:
    if dominance_index is None:
        dominance_index = {}
        for labels in backward_by_node.values():
            for incumbent in labels:
                dominance_index.setdefault(_dominance_bucket_key(incumbent), []).append(incumbent)
    if label.path in backward_paths:
        return False, 1, 0, 0
    meet = label.path[0]
    comparable = backward_by_node.setdefault(meet, [])
    dominance_tests = 0
    for incumbent in _dominance_candidate_labels(
        label,
        dominance_index,
        graph,
        counter,
        incumbent_may_dominate_label=True,
        dominance_key_cache=dominance_key_cache,
        small_dom_bucket_threshold=small_dom_bucket_threshold,
        small_dom_cumulative_threshold=small_dom_cumulative_threshold,
        max_dom_bypass_calls=max_dom_bypass_calls,
    ):
        dominance_tests += 1
        if not _backward_dominance_prefilter(incumbent, label, counter):
            continue
        if _backward_dominates(incumbent, label, graph, objective, restrictions, arc_customer_sets, duals, farkas, counter):
            return False, 1, 0, dominance_tests
    survivors: list[_BackwardLabel] = []
    purged_count = 0
    purge_candidates = set(
        _dominance_candidate_labels(
            label,
            dominance_index,
            graph,
            counter,
            incumbent_may_dominate_label=False,
            dominance_key_cache=dominance_key_cache,
            small_dom_bucket_threshold=small_dom_bucket_threshold,
            small_dom_cumulative_threshold=small_dom_cumulative_threshold,
            max_dom_bypass_calls=max_dom_bypass_calls,
        )
    )
    for incumbent in comparable:
        if incumbent not in purge_candidates:
            survivors.append(incumbent)
            continue
        dominance_tests += 1
        if not _backward_dominance_prefilter(label, incumbent, counter):
            survivors.append(incumbent)
            continue
        if _backward_dominates(label, incumbent, graph, objective, restrictions, arc_customer_sets, duals, farkas, counter):
            purged_count += 1
            backward_paths.discard(incumbent.path)
            _remove_from_dominance_index(dominance_index, incumbent)
        else:
            survivors.append(incumbent)
    survivors.append(label)
    backward_by_node[meet] = survivors
    backward_paths.add(label.path)
    dominance_index.setdefault(_dominance_bucket_key(label), []).append(label)
    return True, 0, purged_count, dominance_tests


def _dominance_bucket_key(label: _BackwardLabel) -> _DominanceBucketKey:
    meet = label.path[0]
    physical_loc = duplicate_hub(meet) if is_duplicate(meet) else meet
    return _DominanceBucketKey(
        end_node=meet,
        physical_loc=physical_loc,
        active_pad=label.leading_block_hub,
        block_pos=label.leading_block_count,
        branch_state_hash=hash(label.branch_state),
    )


def _dominance_candidate_labels(
    label: _BackwardLabel,
    dominance_index: dict[_DominanceBucketKey, list[_BackwardLabel]],
    graph: TransformedGraph,
    counter: _DominanceCounter | None,
    incumbent_may_dominate_label: bool,
    dominance_key_cache: dict[tuple[_DominanceBucketKey, bool], tuple[_DominanceBucketKey, ...]] | None = None,
    small_dom_bucket_threshold: int = 100,
    small_dom_cumulative_threshold: int = 500_000,
    max_dom_bypass_calls: int = 2_000,
) -> list[_BackwardLabel]:
    label_key = _dominance_bucket_key(label)
    local_dom_size = len(dominance_index.get(label_key, ()))
    dom_work_estimate = _estimate_dominance_work(
        dominance_index,
        graph,
        incumbent_may_dominate_label,
        dominance_key_cache,
        counter,
    )
    if counter is not None:
        counter.work_estimate = max(counter.work_estimate, dom_work_estimate)
        if (
            not counter.sticky_indexed
            and (
                local_dom_size > small_dom_bucket_threshold
                or dom_work_estimate > small_dom_cumulative_threshold
                or counter.small_bypass_calls >= max_dom_bypass_calls
            )
        ):
            counter.sticky_indexed = True
            counter.sticky_indexed_activations += 1
    use_direct = (
        (counter is None or not counter.sticky_indexed)
        and
        small_dom_bucket_threshold > 0
        and local_dom_size <= small_dom_bucket_threshold
        and dom_work_estimate <= small_dom_cumulative_threshold
        and (counter is None or counter.small_bypass_calls < max_dom_bypass_calls)
    )
    if use_direct:
        if counter is not None:
            counter.small_bypass_calls += 1
        keys = tuple(sorted(dominance_index, key=str))
    else:
        if counter is not None:
            counter.indexed_activation_count += 1
        cache_key = (label_key, incumbent_may_dominate_label)
        if dominance_key_cache is not None and cache_key in dominance_key_cache:
            keys = dominance_key_cache[cache_key]
            if counter is not None:
                counter.key_cache_hits += 1
        else:
            key_start = time.time()
            keys = _compatible_dominance_keys(label_key, graph, incumbent_may_dominate_label)
            if dominance_key_cache is not None:
                dominance_key_cache[cache_key] = keys
            if counter is not None:
                counter.key_generation_time_seconds += time.time() - key_start
                counter.key_cache_misses += 1
        if counter is not None:
            counter.compatible_keys_generated += len(keys)
            counter.bucket_scans_avoided += max(len(dominance_index) - len(keys), 0)
    candidates: list[_BackwardLabel] = []
    for key in keys:
        if counter is not None:
            counter.compatible_key_lookups += 1
        labels = dominance_index.get(key)
        if not labels:
            continue
        pair_count = len(labels)
        if counter is not None:
            counter.bucket_pairs_considered += pair_count
        if not _dominance_buckets_compatible(key, label_key, incumbent_may_dominate_label):
            if counter is not None:
                counter.bucket_pairs_rejected += pair_count
                counter.stage_reject_key += pair_count
            continue
        if counter is not None:
            counter.bucket_candidate_pairs += pair_count
        candidates.extend(labels)
    return candidates


def _estimate_dominance_work(
    dominance_index: dict[_DominanceBucketKey, list[_BackwardLabel]],
    graph: TransformedGraph,
    incumbent_may_dominate_label: bool,
    dominance_key_cache: dict[tuple[_DominanceBucketKey, bool], tuple[_DominanceBucketKey, ...]] | None,
    counter: _DominanceCounter | None,
) -> int:
    label_count = sum(len(labels) for labels in dominance_index.values())
    return label_count * label_count


def _compatible_dominance_keys(
    label_key: _DominanceBucketKey,
    graph: TransformedGraph,
    incumbent_may_dominate_label: bool,
) -> tuple[_DominanceBucketKey, ...]:
    keys: set[_DominanceBucketKey] = set()
    if incumbent_may_dominate_label:
        if label_key.active_pad is None:
            keys.add(
                _DominanceBucketKey(
                    label_key.end_node,
                    label_key.physical_loc,
                    None,
                    0,
                    label_key.branch_state_hash,
                )
            )
        else:
            keys.add(
                _DominanceBucketKey(
                    label_key.end_node,
                    label_key.physical_loc,
                    None,
                    0,
                    label_key.branch_state_hash,
                )
            )
            for block_pos in range(label_key.block_pos + 1):
                keys.add(
                    _DominanceBucketKey(
                        label_key.end_node,
                        label_key.physical_loc,
                        label_key.active_pad,
                        block_pos,
                        label_key.branch_state_hash,
                    )
                )
    elif label_key.active_pad is None:
        keys.add(
            _DominanceBucketKey(
                label_key.end_node,
                label_key.physical_loc,
                None,
                0,
                label_key.branch_state_hash,
            )
        )
        for hub in graph.instance.hubs:
            for block_pos in range(graph.instance.drones_per_truck + 1):
                keys.add(
                    _DominanceBucketKey(
                        label_key.end_node,
                        label_key.physical_loc,
                        hub,
                        block_pos,
                        label_key.branch_state_hash,
                    )
                )
    else:
        for block_pos in range(label_key.block_pos, graph.instance.drones_per_truck + 1):
            keys.add(
                _DominanceBucketKey(
                    label_key.end_node,
                    label_key.physical_loc,
                    label_key.active_pad,
                    block_pos,
                    label_key.branch_state_hash,
                )
            )
    return tuple(sorted(keys, key=str))


def _dominance_buckets_compatible(
    incumbent_key: _DominanceBucketKey,
    label_key: _DominanceBucketKey,
    incumbent_may_dominate_label: bool,
) -> bool:
    if incumbent_key.end_node != label_key.end_node:
        return False
    if incumbent_key.physical_loc != label_key.physical_loc:
        return False
    if incumbent_key.branch_state_hash != label_key.branch_state_hash:
        return False
    dominator = incumbent_key if incumbent_may_dominate_label else label_key
    dominated = label_key if incumbent_may_dominate_label else incumbent_key
    if dominated.active_pad is None:
        return dominator.active_pad is None
    if dominator.active_pad is None:
        return True
    return dominator.active_pad == dominated.active_pad and dominator.block_pos <= dominated.block_pos


def _remove_from_dominance_index(
    dominance_index: dict[_DominanceBucketKey, list[_BackwardLabel]],
    label: _BackwardLabel,
) -> None:
    key = _dominance_bucket_key(label)
    labels = dominance_index.get(key)
    if labels is None:
        return
    dominance_index[key] = [incumbent for incumbent in labels if incumbent.path != label.path]
    if not dominance_index[key]:
        del dominance_index[key]


def _backward_dominance_prefilter(
    a: _BackwardLabel,
    b: _BackwardLabel,
    counter: _DominanceCounter | None = None,
) -> bool:
    if counter is not None:
        counter.prefilter_pairs += 1
    if a.path[0] != b.path[0]:
        if counter is not None:
            counter.prefilter_rejected += 1
            counter.stage_reject_key += 1
        return False
    if a.branch_state is not None and b.branch_state is not None and a.branch_state != b.branch_state:
        if counter is not None:
            counter.prefilter_rejected += 1
            counter.stage_reject_branch += 1
        return False
    if a.represented_mask and b.represented_mask:
        customer_included = (a.represented_mask & ~b.represented_mask) == 0
    else:
        customer_included = a.represented.issubset(b.represented)
    if not customer_included:
        if counter is not None:
            counter.prefilter_rejected += 1
            counter.stage_reject_customer += 1
        return False
    if not a.represented and a.represented != b.represented:
        if counter is not None:
            counter.prefilter_rejected += 1
            counter.stage_reject_customer += 1
        return False
    if a.truck_node_mask and b.truck_node_mask:
        truck_included = (a.truck_node_mask & ~b.truck_node_mask) == 0
    else:
        truck_included = a.truck_visited.issubset(b.truck_visited)
    if not truck_included:
        if counter is not None:
            counter.prefilter_rejected += 1
            counter.stage_reject_truck_node += 1
        return False
    if a.truck_load > b.truck_load + PAYLOAD_TOLERANCE:
        if counter is not None:
            counter.prefilter_rejected += 1
            counter.stage_reject_payload += 1
        return False
    if a.leading_block_hub is not None and b.leading_block_hub is not None:
        if a.leading_block_hub != b.leading_block_hub or a.leading_block_count > b.leading_block_count:
            if counter is not None:
                counter.prefilter_rejected += 1
                counter.stage_reject_block += 1
            return False
    elif a.leading_block_hub is not None and b.leading_block_hub is None:
        if counter is not None:
            counter.prefilter_rejected += 1
            counter.stage_reject_block += 1
        return False
    if counter is not None:
        counter.full_tests += 1
    return True


def _backward_dominates(
    a: _BackwardLabel,
    b: _BackwardLabel,
    graph: TransformedGraph,
    objective: ObjectiveData,
    restrictions: BranchRestrictions,
    arc_customer_sets: dict[tuple[str, str], frozenset[str]],
    duals: PricingDuals,
    farkas: bool,
    counter: _DominanceCounter | None = None,
) -> bool:
    if a.path[0] != b.path[0]:
        return False
    if not a.represented.issubset(b.represented):
        return False
    if not a.represented and a.represented != b.represented:
        return False
    if not a.truck_visited.issubset(b.truck_visited):
        return False
    if a.truck_load > b.truck_load + PAYLOAD_TOLERANCE:
        return False
    if not _backward_service_representation_includes(a, b):
        return False
    if not _leading_block_includes(a, b):
        return False
    if not _backward_branch_language_includes(a, b, restrictions, arc_customer_sets):
        return False
    if not _suffix_dual_contribution_nonincreasing(a, b, duals):
        return False
    if farkas and a.sr_counts != b.sr_counts:
        return False
    if not farkas and not _backward_profile_dominates(a, b):
        return False
    if counter is not None:
        counter.cost_function_tests += 1
    if not _backward_cost_function_dominates(a, b, graph, objective, duals, farkas):
        if counter is not None:
            counter.cost_function_rejected += 1
            counter.stage_reject_cost += 1
        return False
    return (
        a.path != b.path
        or a.represented != b.represented
        or a.truck_visited != b.truck_visited
        or a.truck_load < b.truck_load
        or a.leading_block_hub != b.leading_block_hub
        or a.leading_block_count < b.leading_block_count
        or a.cost_function != b.cost_function
    )


def _backward_service_representation_includes(a: _BackwardLabel, b: _BackwardLabel) -> bool:
    if not a.truck_served.issubset(b.truck_served):
        return False
    if not a.pad_served.issubset(b.pad_served):
        return False
    if a.truck_served | frozenset(customer for _, customer in a.pad_served) != a.represented:
        return False
    for customer in a.represented:
        if (customer in a.truck_served) != (customer in b.truck_served):
            return False
        a_pad = tuple(sorted(hub for hub, served in a.pad_served if served == customer))
        b_pad = tuple(sorted(hub for hub, served in b.pad_served if served == customer))
        if a_pad != b_pad:
            return False
    return True


def _leading_block_includes(a: _BackwardLabel, b: _BackwardLabel) -> bool:
    if b.leading_block_hub is None:
        return a.leading_block_hub is None
    if a.leading_block_hub is None:
        return True
    return a.leading_block_hub == b.leading_block_hub and a.leading_block_count <= b.leading_block_count


def _backward_branch_language_includes(
    a: _BackwardLabel,
    b: _BackwardLabel,
    restrictions: BranchRestrictions,
    arc_customer_sets: dict[tuple[str, str], frozenset[str]],
) -> bool:
    if a.branch_state is None or b.branch_state is None:
        return False
    if a.branch_state != b.branch_state:
        return False
    if restrictions.route_forbidden and a.path != b.path:
        return False
    for p, q in restrictions.together_pairs:
        a_pair = (p in a.represented, q in a.represented)
        b_pair = (p in b.represented, q in b.represented)
        if b_pair in {(True, False), (False, True)}:
            if a_pair != b_pair:
                return False
        elif b_pair == (True, True) and a_pair not in {(True, True), (False, False)}:
            return False
        elif b_pair == (False, False) and a_pair != (False, False):
            return False
    if a.represented.intersection(restrictions.truck_service) != a.truck_served.intersection(restrictions.truck_service):
        return False
    if a.represented.intersection(restrictions.drone_service).intersection(a.truck_served):
        return False
    if a.pad_served.intersection(restrictions.pad_forbidden):
        return False
    for hub, customer in restrictions.pad_required:
        if customer in a.represented and (hub, customer) not in a.pad_served:
            return False
    if a.used_arcs.intersection(restrictions.trans_arc_forbidden):
        return False
    for arc in restrictions.trans_arc_required:
        if a.represented.intersection(arc_customer_sets[arc]) and arc not in a.used_arcs:
            return False
    return True


def _suffix_dual_contribution_nonincreasing(a: _BackwardLabel, b: _BackwardLabel, duals: PricingDuals) -> bool:
    if a.sr_counts and b.sr_counts:
        a_counts = dict(a.sr_counts)
        b_counts = dict(b.sr_counts)
        for triplet in duals.nu:
            if a_counts.get(triplet, 0) > b_counts.get(triplet, 0):
                return False
    for customer in b.represented - a.represented:
        if duals.mu[customer] > PAPER_DOMINANCE_TOLERANCE:
            return False
    return True


def _backward_profile_dominates(a: _BackwardLabel, b: _BackwardLabel) -> bool:
    if a.profile is None or b.profile is None:
        return False
    if a.profile.drone_sorties > b.profile.drone_sorties:
        return False
    a_service = dict(a.profile.service_times)
    b_service = dict(b.profile.service_times)
    if not frozenset(a_service).issubset(b_service):
        return False
    for customer, expr in a_service.items():
        if not _time_expr_leq_for_all(expr, b_service[customer]):
            return False
    return _time_expr_leq_for_all(a.profile.return_time, b.profile.return_time)


def _backward_cost_function_dominates(
    a: _BackwardLabel,
    b: _BackwardLabel,
    graph: TransformedGraph,
    objective: ObjectiveData,
    duals: PricingDuals,
    farkas: bool,
) -> bool:
    if a.cost_function is None or b.cost_function is None:
        return False
    boundary_correction = 0.0 if farkas else _worst_case_backward_sr_boundary_correction(a, b, duals)
    for interface in _backward_dominance_interfaces(a, b, graph, objective):
        a_value = a.cost_function.evaluate(interface, graph, objective, duals, farkas) + boundary_correction
        b_value = b.cost_function.evaluate(interface, graph, objective, duals, farkas)
        if a_value > b_value + PAPER_DOMINANCE_TOLERANCE:
            return False
    return True


def _worst_case_backward_sr_boundary_correction(
    a: _BackwardLabel,
    b: _BackwardLabel,
    duals: PricingDuals,
) -> float:
    a_counts = dict(a.sr_counts)
    b_counts = dict(b.sr_counts)
    worst = 0.0
    for triplet, dual in duals.nu.items():
        a_suffix = a_counts.get(triplet, 0)
        b_suffix = b_counts.get(triplet, 0)
        triplet_worst = max(
            -dual * (
                ((prefix + a_suffix) // 2 - prefix // 2 - a_suffix // 2)
                - ((prefix + b_suffix) // 2 - prefix // 2 - b_suffix // 2)
            )
            for prefix in range(len(triplet) + 1)
        )
        worst += triplet_worst
    return worst


def _backward_dominance_interfaces(
    a: _BackwardLabel,
    b: _BackwardLabel,
    graph: TransformedGraph,
    objective: ObjectiveData,
) -> tuple[_BackwardInterface, ...]:
    upper = objective.bounds.return_ub
    service_exprs: list[tuple[str, _TimeExpr]] = []
    for profile in (a.profile, b.profile):
        if profile is None:
            return tuple()
        service_exprs.extend(profile.service_times)
    wait_candidates = {
        0.0,
        a.leading_block_wait,
        b.leading_block_wait,
    }
    wait_candidates.update(graph.instance.drone_trip_time.values())
    time_candidates = {0.0, upper}
    for customer, expr in service_exprs:
        target = objective.bounds.arrival_lb[customer]
        if expr.base in {"physical", "pad_arrival"}:
            time_candidates.add(target - expr.offset)
        elif expr.base == "wait":
            for wait_value in wait_candidates:
                time_candidates.add(target - max(wait_value, expr.wait_floor) - expr.offset)
        else:
            raise ValueError(f"unknown time expression base: {expr.base}")
    bounded_times = sorted(value for value in time_candidates if 0.0 <= value <= upper)
    bounded_waits = sorted(value for value in wait_candidates if 0.0 <= value <= upper)
    interfaces: list[_BackwardInterface] = []
    for physical_time in bounded_times:
        for pad_arrival in bounded_times:
            if pad_arrival > physical_time + PAPER_DOMINANCE_TOLERANCE:
                continue
            for wait_value in bounded_waits:
                interfaces.append(
                    _BackwardInterface(
                        physical_time=physical_time,
                        pad_arrival=pad_arrival,
                        active_wait=wait_value,
                    )
                )
    if not interfaces:
        return (
            _BackwardInterface(0.0, 0.0, 0.0),
            _BackwardInterface(upper, upper, 0.0),
        )
    return tuple(interfaces)


def _backward_profile(
    path: tuple[str, ...],
    graph: TransformedGraph,
) -> tuple[_BackwardProfile, frozenset[str], frozenset[tuple[str, str]]]:
    instance = graph.instance
    physical_time = _TimeExpr("physical", 0.0)
    active_pad: str | None = None
    active_pad_arrival = physical_time
    active_wait_floor = 0.0
    if path and path[0] in instance.hubs:
        active_pad = path[0]
        active_pad_arrival = physical_time
    elif path and is_duplicate(path[0]):
        active_pad = duplicate_hub(path[0])
        active_pad_arrival = _TimeExpr("pad_arrival", 0.0)

    service_times: dict[str, _TimeExpr] = {}
    truck_served: set[str] = set()
    pad_served: set[tuple[str, str]] = set()

    for i, j in zip(path, path[1:]):
        arc = (i, j)
        if arc in graph.truck_arcs:
            physical_time = _time_expr_add(physical_time, instance.truck_time[arc])
            active_pad = j if j in instance.hubs else active_pad
            active_pad_arrival = physical_time if j in instance.hubs else active_pad_arrival
            active_wait_floor = 0.0
            if j in instance.customers:
                truck_served.add(j)
                service_times[j] = physical_time
            continue
        if arc in graph.hub_duplicate_arcs:
            active_pad = i
            active_pad_arrival = physical_time
            active_wait_floor = 0.0
            customer = duplicate_customer(j)
            pad_served.add((active_pad, customer))
            service_times[customer] = _time_expr_add(active_pad_arrival, instance.drone_time[(active_pad, customer)])
            active_wait_floor = max(active_wait_floor, instance.drone_trip_time[(active_pad, customer)])
            continue
        if arc in graph.duplicate_duplicate_arcs:
            hub = duplicate_hub(j)
            if active_pad != hub:
                raise ValueError("duplicate block hub mismatch in backward profile")
            customer = duplicate_customer(j)
            pad_served.add((active_pad, customer))
            service_times[customer] = _time_expr_add(active_pad_arrival, instance.drone_time[(active_pad, customer)])
            active_wait_floor = max(active_wait_floor, instance.drone_trip_time[(active_pad, customer)])
            continue
        if arc in graph.duplicate_regular_arcs:
            hub = duplicate_hub(i)
            if active_pad != hub:
                raise ValueError("duplicate continuation hub mismatch in backward profile")
            physical_time = _depart_after_wait(active_pad_arrival, active_wait_floor, instance.truck_time[(hub, j)])
            active_pad = j if j in instance.hubs else active_pad
            active_pad_arrival = physical_time if j in instance.hubs else active_pad_arrival
            active_wait_floor = 0.0
            if j in instance.customers:
                truck_served.add(j)
                service_times[j] = physical_time
            continue
        raise ValueError(f"unclassified transformed arc in backward profile: {arc}")

    return (
        _BackwardProfile(
            service_times=tuple(sorted(service_times.items())),
            return_time=physical_time,
            drone_sorties=len(pad_served),
        ),
        frozenset(truck_served),
        frozenset(pad_served),
    )


def _backward_branch_state(
    represented: frozenset[str],
    used_arcs: frozenset[tuple[str, str]],
    restrictions: BranchRestrictions,
    arc_customer_sets: dict[tuple[str, str], frozenset[str]],
) -> _BranchState:
    together = tuple(
        (p in represented, q in represented)
        for p, q in sorted(restrictions.together_pairs)
    )
    required_arcs = tuple(
        (arc, bool(represented.intersection(arc_customer_sets[arc])), arc in used_arcs)
        for arc in sorted(restrictions.trans_arc_required)
    )
    return _BranchState(together=together, required_arcs=required_arcs)


def _time_expr_add(expr: _TimeExpr, offset: float) -> _TimeExpr:
    return _TimeExpr(expr.base, expr.offset + offset, expr.wait_floor)


def _depart_after_wait(arrival: _TimeExpr, wait_floor: float, travel_time: float) -> _TimeExpr:
    if arrival.base == "physical":
        return _TimeExpr("physical", arrival.offset + wait_floor + travel_time)
    if arrival.base == "pad_arrival":
        return _TimeExpr("wait", arrival.offset + travel_time, wait_floor)
    if arrival.base == "wait":
        return _TimeExpr("wait", arrival.offset + wait_floor + travel_time, arrival.wait_floor)
    raise ValueError(f"unknown time expression base: {arrival.base}")


def _time_expr_leq_for_all(a: _TimeExpr, b: _TimeExpr) -> bool:
    if a.base != b.base:
        return False
    tolerance = PAPER_DOMINANCE_TOLERANCE
    if a.base in {"physical", "pad_arrival"}:
        return a.offset <= b.offset + tolerance
    if a.base == "wait":
        for wait_value in sorted({0.0, a.wait_floor, b.wait_floor}):
            a_value = max(wait_value, a.wait_floor) + a.offset
            b_value = max(wait_value, b.wait_floor) + b.offset
            if a_value > b_value + tolerance:
                return False
        return a.offset <= b.offset + tolerance
    raise ValueError(f"unknown time expression base: {a.base}")


def _evaluate_time_expr(expr: _TimeExpr, label: _Label) -> float:
    if expr.base == "physical":
        return label.physical_time + expr.offset
    if expr.base == "pad_arrival":
        return label.active_pad_arrival + expr.offset
    if expr.base == "wait":
        return label.active_pad_arrival + max(label.active_wait, expr.wait_floor) + expr.offset
    raise ValueError(f"unknown time expression base: {expr.base}")


def _evaluate_time_expr_at_interface(expr: _TimeExpr, interface: _BackwardInterface) -> float:
    if expr.base == "physical":
        return interface.physical_time + expr.offset
    if expr.base == "pad_arrival":
        return interface.pad_arrival + expr.offset
    if expr.base == "wait":
        return interface.pad_arrival + max(interface.active_wait, expr.wait_floor) + expr.offset
    raise ValueError(f"unknown time expression base: {expr.base}")


def _backward_join_interface(
    forward_label: _Label,
    backward_label: _BackwardLabel,
    graph: TransformedGraph,
) -> _BackwardInterface:
    meet = forward_label.path[-1]
    if is_duplicate(meet):
        if backward_label.leading_block_hub is None:
            return _BackwardInterface(
                physical_time=forward_label.physical_time,
                pad_arrival=forward_label.active_pad_arrival,
                active_wait=forward_label.active_wait,
            )
        return _BackwardInterface(
            physical_time=forward_label.physical_time,
            pad_arrival=forward_label.active_pad_arrival,
            active_wait=max(forward_label.active_wait, backward_label.leading_block_wait),
        )
    if meet in graph.instance.hubs:
        return _BackwardInterface(
            physical_time=forward_label.physical_time,
            pad_arrival=forward_label.physical_time,
            active_wait=0.0,
        )
    return _BackwardInterface(
        physical_time=forward_label.physical_time,
        pad_arrival=forward_label.active_pad_arrival,
        active_wait=0.0,
    )


def _suffix_sr_contribution(sr_counts: dict[tuple[str, str, str], int], duals: PricingDuals) -> float:
    return sum(dual * (sr_counts.get(triplet, 0) // 2) for triplet, dual in duals.nu.items())


def _sr_join_correction(
    prefix_counts: dict[tuple[str, str, str], int],
    suffix_counts: dict[tuple[str, str, str], int],
    duals: PricingDuals,
) -> float:
    correction = 0.0
    for triplet, dual in duals.nu.items():
        prefix = prefix_counts.get(triplet, 0)
        suffix = suffix_counts.get(triplet, 0)
        correction -= dual * (((prefix + suffix) // 2) - (prefix // 2) - (suffix // 2))
    return correction


def _dual_solution_key(duals: PricingDuals) -> tuple[object, ...]:
    return (
        tuple(sorted(duals.mu.items())),
        duals.kappa,
        tuple(sorted(duals.nu.items())),
    )


def _candidate_from_forward_label(
    label: _Label,
    graph: TransformedGraph,
    objective: ObjectiveData,
    duals: PricingDuals,
    farkas: bool,
) -> _RouteCandidate:
    return _RouteCandidate(
        path=label.path,
        reduced_cost=label.reduced_cost,
        joined=False,
        served=label.represented,
        truck_served=label.truck_served,
        pad_served=label.pad_served,
        used_arcs=label.used_arcs,
    )


def _join_forward_backward(
    forward_label: _Label,
    backward_label: _BackwardLabel,
    graph: TransformedGraph,
    objective: ObjectiveData,
    residual_customers: frozenset[str],
    restrictions: BranchRestrictions,
    arc_customer_sets: dict[tuple[str, str], frozenset[str]],
    duals: PricingDuals,
    farkas: bool,
    reduced_cost: float | None = None,
) -> _RouteCandidate | None:
    if forward_label.path[-1] != backward_label.path[0]:
        return None
    if forward_label.represented.intersection(backward_label.represented):
        return None
    if not forward_label.represented and not backward_label.represented:
        return None
    meet_node = forward_label.path[-1]
    shared_physical_meet = duplicate_hub(meet_node) if is_duplicate(meet_node) else meet_node
    if forward_label.truck_visited.intersection(backward_label.truck_visited) - {shared_physical_meet}:
        return None
    instance = graph.instance
    if forward_label.truck_load + backward_label.truck_load > instance.truck_payload + PAYLOAD_TOLERANCE:
        return None
    if backward_label.leading_block_hub is not None:
        if forward_label.active_pad not in {None, backward_label.leading_block_hub}:
            return None
        if forward_label.block_count + backward_label.leading_block_count > instance.drones_per_truck:
            return None
    path = forward_label.path + backward_label.path[1:]
    if not path or path[0] != instance.depot_source or path[-1] != instance.depot_sink:
        return None
    if not _path_physical_elementary(path, graph):
        return None
    served = forward_label.represented | backward_label.represented
    truck_served = forward_label.truck_served | backward_label.truck_served
    pad_served = forward_label.pad_served | backward_label.pad_served
    used_arcs = forward_label.used_arcs | backward_label.used_arcs
    if not served or not served.issubset(residual_customers):
        return None
    if not _candidate_branch_allowed(path, served, truck_served, pad_served, used_arcs, restrictions, arc_customer_sets):
        return None
    if reduced_cost is None:
        reduced_cost = _joined_reduced_cost(forward_label, backward_label, graph, objective, duals, farkas)
    return _RouteCandidate(
        path=path,
        reduced_cost=reduced_cost,
        joined=True,
        served=served,
        truck_served=truck_served,
        pad_served=pad_served,
        used_arcs=used_arcs,
    )


def _joined_reduced_cost(
    forward_label: _Label,
    backward_label: _BackwardLabel,
    graph: TransformedGraph,
    objective: ObjectiveData,
    duals: PricingDuals,
    farkas: bool,
) -> float:
    if backward_label.cost_function is None:
        raise ValueError("backward label is missing its suffix cost function")
    interface = _backward_join_interface(forward_label, backward_label, graph)
    reduced_cost = forward_label.reduced_cost + backward_label.cost_function.evaluate(
        interface,
        graph,
        objective,
        duals,
        farkas,
    )
    suffix_sr_counts = dict(backward_label.sr_counts)
    prefix_sr_counts = dict(forward_label.sr_counts)
    return reduced_cost + _sr_join_correction(prefix_sr_counts, suffix_sr_counts, duals)


def _join_lower_bound(
    forward_label: _Label,
    backward_label: _BackwardLabel,
    graph: TransformedGraph,
    duals: PricingDuals,
    farkas: bool,
    enable_join_lower_envelope: bool = True,
) -> float:
    if farkas or not enable_join_lower_envelope:
        return float("-inf")
    if backward_label.lower_envelope is None:
        raise ValueError("backward label is missing its suffix lower envelope")
    _backward_join_interface(forward_label, backward_label, graph)
    return forward_label.reduced_cost + backward_label.lower_envelope.evaluate(duals, farkas=False)


def _joined_reduced_cost_cached(
    forward_label: _Label,
    backward_label: _BackwardLabel,
    graph: TransformedGraph,
    objective: ObjectiveData,
    duals: PricingDuals,
    farkas: bool,
    cache: _JoinEvalCache,
    active_sr_version: int,
    dual_solution_key: tuple[object, ...],
    enable_cache: bool = True,
) -> float:
    if backward_label.cost_function is None:
        raise ValueError("backward label is missing its suffix cost function")
    interface = _backward_join_interface(forward_label, backward_label, graph)
    prefix_sr_counts = tuple(sorted(forward_label.sr_counts))
    suffix_sr_counts = tuple(sorted(backward_label.sr_counts))
    key = _JoinEvalCacheKey(
        backward_path=backward_label.path,
        meet_node=forward_label.path[-1],
        physical_time=interface.physical_time,
        pad_arrival=interface.pad_arrival,
        active_wait=interface.active_wait,
        active_sr_version=active_sr_version,
        dual_solution_key=dual_solution_key,
        farkas=farkas,
        prefix_sr_counts=prefix_sr_counts,
        suffix_sr_counts=suffix_sr_counts,
    )
    cached = cache.values.get(key) if enable_cache else None
    if cached is None:
        if enable_cache:
            cache.misses += 1
            cache.suffix_profile_misses += 1
            cache.interface_profile_misses += 1
        suffix_value = backward_label.cost_function.evaluate(interface, graph, objective, duals, farkas)
        sr_correction = _sr_join_correction(dict(prefix_sr_counts), dict(suffix_sr_counts), duals)
        if enable_cache:
            cache.values[key] = (suffix_value, sr_correction)
    else:
        cache.hits += 1
        cache.suffix_profile_hits += 1
        cache.interface_profile_hits += 1
        suffix_value, sr_correction = cached
    return forward_label.reduced_cost + suffix_value + sr_correction


def _candidate_branch_allowed(
    path: tuple[str, ...],
    served: frozenset[str],
    truck_served: frozenset[str],
    pad_served: frozenset[tuple[str, str]],
    used_arcs: frozenset[tuple[str, str]],
    restrictions: BranchRestrictions,
    arc_customer_sets: dict[tuple[str, str], frozenset[str]],
) -> bool:
    if path in restrictions.route_forbidden:
        return False
    for p, q in restrictions.together_pairs:
        if (p in served) != (q in served):
            return False
    for p, q in restrictions.separate_pairs:
        if p in served and q in served:
            return False
    for customer in restrictions.truck_service:
        if customer in served and customer not in truck_served:
            return False
    for customer in restrictions.drone_service:
        if customer in served and customer in truck_served:
            return False
    if pad_served.intersection(restrictions.pad_forbidden):
        return False
    for hub, customer in restrictions.pad_required:
        if customer in served and (hub, customer) not in pad_served:
            return False
    if used_arcs.intersection(restrictions.trans_arc_forbidden):
        return False
    for arc in restrictions.trans_arc_required:
        if served.intersection(arc_customer_sets[arc]) and arc not in used_arcs:
            return False
    return True


def _join_rejection_stage(forward_label: _Label, backward_label: _BackwardLabel, graph: TransformedGraph) -> str | None:
    if forward_label.path[-1] != backward_label.path[0]:
        return "key"
    if forward_label.represented_mask and backward_label.represented_mask:
        if forward_label.represented_mask & backward_label.represented_mask:
            return "customer"
    elif forward_label.represented.intersection(backward_label.represented):
        return "customer"
    if not forward_label.represented and not backward_label.represented:
        return "customer"
    meet_node = forward_label.path[-1]
    physical_loc = duplicate_hub(meet_node) if is_duplicate(meet_node) else meet_node
    if forward_label.truck_visited.intersection(backward_label.truck_visited) - {physical_loc}:
        return "truck_node"
    if forward_label.truck_load + backward_label.truck_load > graph.instance.truck_payload + PAYLOAD_TOLERANCE:
        return "payload"
    if backward_label.leading_block_hub is not None:
        if forward_label.active_pad not in {None, backward_label.leading_block_hub}:
            return "block"
        if forward_label.block_count + backward_label.leading_block_count > graph.instance.drones_per_truck:
            return "block"
    return None


def _join_prefilter(forward_label: _Label, backward_label: _BackwardLabel, graph: TransformedGraph) -> bool:
    return _join_rejection_stage(forward_label, backward_label, graph) is None


def _path_physical_elementary(path: tuple[str, ...], graph: TransformedGraph) -> bool:
    seen = {graph.instance.depot_source}
    for node in path[1:]:
        if node in graph.instance.customers + graph.instance.hubs:
            if node in seen:
                return False
            seen.add(node)
    return True


def _increment_join_stage(counter: _JoinStageCounter | None, stage: str, amount: int = 1) -> None:
    if counter is None:
        return
    if stage == "key":
        counter.reject_key += amount
    elif stage == "branch":
        counter.reject_branch += amount
    elif stage == "customer":
        counter.reject_customer += amount
    elif stage == "truck_node":
        counter.reject_truck_node += amount
    elif stage == "payload":
        counter.reject_payload += amount
    elif stage == "block":
        counter.reject_block += amount
    elif stage == "reduced_cost":
        counter.reject_reduced_cost += amount
    else:
        raise RuntimeError(f"unknown join rejection stage {stage}")


def _bucketed_join_pairs(
    forward_labels: list[_Label],
    backward_labels: list[_BackwardLabel],
    graph: TransformedGraph,
    compatible_key_cache: dict[_JoinLookupKey, tuple[_JoinLookupKey, ...]] | None = None,
    small_join_pair_threshold: int = 5_000,
    small_join_cumulative_threshold: int = 250_000,
    max_join_bypass_calls: int = 1_000,
    previous_join_bypass_calls: int = 0,
    cumulative_pair_count: int | None = None,
    payload_bin_width: float = 1.0,
    force_indexed: bool = False,
    stage_counter: _JoinStageCounter | None = None,
) -> dict[str, object]:
    total_pair_count = len(forward_labels) * len(backward_labels)
    join_work_estimate = total_pair_count if cumulative_pair_count is None else cumulative_pair_count
    use_direct = (
        not force_indexed
        and
        total_pair_count <= small_join_pair_threshold
        and join_work_estimate <= small_join_cumulative_threshold
        and previous_join_bypass_calls < max_join_bypass_calls
    )
    if use_direct:
        pairs: list[tuple[_Label, _BackwardLabel]] = []
        label_pairs_rejected = 0
        for forward_label in forward_labels:
            for backward_label in backward_labels:
                rejection_stage = _join_rejection_stage(forward_label, backward_label, graph)
                if rejection_stage is None:
                    pairs.append((forward_label, backward_label))
                else:
                    label_pairs_rejected += 1
                    _increment_join_stage(stage_counter, rejection_stage)
        return {
            "pairs": pairs,
            "bucket_pairs_considered": 0,
            "bucket_pairs_rejected": 0,
            "bucket_candidate_pairs": total_pair_count,
            "compatible_keys_generated": 0,
            "compatible_key_lookups": 0,
            "bucket_scans_avoided": 0,
            "key_generation_time_seconds": 0.0,
            "key_cache_hits": 0,
            "key_cache_misses": 0,
            "join_graph_build_time_seconds": 0.0,
            "subbucket_pairs_considered": 0,
            "subbucket_pairs_rejected": 0,
            "small_bypass_calls": 1,
            "local_bypass_calls": 1,
            "cumulative_bypass_calls": 1,
            "indexed_activation_count": 0,
            "join_work_estimate": join_work_estimate,
            "label_pairs_materialized": total_pair_count,
            "label_pairs_rejected": label_pairs_rejected,
            "candidate_pairs_accepted": len(pairs),
        }
    forward_buckets = _join_buckets(forward_labels, graph, forward=True)
    backward_buckets = _join_buckets(backward_labels, graph, forward=False)
    backward_by_lookup: dict[_JoinLookupKey, list[_JoinBucket]] = {}
    for backward_bucket in backward_buckets:
        backward_by_lookup.setdefault(_join_lookup_key(backward_bucket.key), []).append(backward_bucket)
    pairs: list[tuple[_Label, _BackwardLabel]] = []
    bucket_pairs_considered = 0
    bucket_pairs_rejected = 0
    bucket_candidate_pairs = 0
    compatible_keys_generated = 0
    compatible_key_lookups = 0
    bucket_scans_avoided = 0
    key_generation_time = 0.0
    key_cache_hits = 0
    key_cache_misses = 0
    graph_build_start = time.time()
    join_graph: dict[int, tuple[_JoinBucket, ...]] = {}
    label_pairs_materialized = 0
    label_pairs_rejected = 0
    for index, forward_bucket in enumerate(forward_buckets):
        forward_lookup_key = _join_lookup_key(forward_bucket.key)
        if compatible_key_cache is not None and forward_lookup_key in compatible_key_cache:
            compatible_keys = compatible_key_cache[forward_lookup_key]
            key_cache_hits += 1
        else:
            key_start = time.time()
            compatible_keys = _compatible_join_lookup_keys(forward_bucket.key, graph)
            key_generation_time += time.time() - key_start
            if compatible_key_cache is not None:
                compatible_key_cache[forward_lookup_key] = compatible_keys
            key_cache_misses += 1
        compatible_keys_generated += len(compatible_keys)
        probed_bucket_count = sum(len(backward_by_lookup[lookup_key]) for lookup_key in compatible_keys if lookup_key in backward_by_lookup)
        bucket_scans_avoided += max(len(backward_buckets) - probed_bucket_count, 0)
        compatible_buckets: list[_JoinBucket] = []
        for lookup_key in compatible_keys:
            compatible_key_lookups += 1
            compatible_buckets.extend(backward_by_lookup.get(lookup_key, ()))
        join_graph[index] = tuple(compatible_buckets)
    join_graph_build_time = time.time() - graph_build_start
    subbucket_pairs_considered = 0
    subbucket_pairs_rejected = 0
    for index, forward_bucket in enumerate(forward_buckets):
        forward_subbuckets = _join_subbuckets(forward_bucket, payload_bin_width)
        for backward_bucket in join_graph[index]:
            bucket_pairs_considered += 1
            pair_count = len(forward_bucket.labels) * len(backward_bucket.labels)
            if not _join_buckets_compatible(forward_bucket, backward_bucket, graph):
                bucket_pairs_rejected += pair_count
                _increment_join_stage(
                    stage_counter,
                    _join_bucket_rejection_stage(forward_bucket, backward_bucket, graph) or "key",
                    pair_count,
                )
                continue
            bucket_candidate_pairs += pair_count
            backward_subbuckets = _join_subbuckets(backward_bucket, payload_bin_width)
            for forward_subbucket in forward_subbuckets:
                for backward_subbucket in backward_subbuckets:
                    subbucket_pair_count = len(forward_subbucket.labels) * len(backward_subbucket.labels)
                    subbucket_pairs_considered += 1
                    if forward_subbucket.min_payload + backward_subbucket.min_payload > graph.instance.truck_payload + PAYLOAD_TOLERANCE:
                        subbucket_pairs_rejected += subbucket_pair_count
                        _increment_join_stage(stage_counter, "payload", subbucket_pair_count)
                        continue
                    for forward_label in forward_subbucket.labels:
                        max_backward_payload = graph.instance.truck_payload - forward_label.truck_load
                        for backward_label in backward_subbucket.labels_by_payload:
                            if backward_label.truck_load > max_backward_payload + PAYLOAD_TOLERANCE:
                                break
                            label_pairs_materialized += 1
                            rejection_stage = _join_rejection_stage(forward_label, backward_label, graph)
                            if rejection_stage is None:
                                pairs.append((forward_label, backward_label))
                            else:
                                label_pairs_rejected += 1
                                _increment_join_stage(stage_counter, rejection_stage)
    return {
        "pairs": pairs,
        "bucket_pairs_considered": bucket_pairs_considered,
        "bucket_pairs_rejected": bucket_pairs_rejected,
        "bucket_candidate_pairs": bucket_candidate_pairs,
        "compatible_keys_generated": compatible_keys_generated,
        "compatible_key_lookups": compatible_key_lookups,
        "bucket_scans_avoided": bucket_scans_avoided,
        "key_generation_time_seconds": key_generation_time,
        "key_cache_hits": key_cache_hits,
        "key_cache_misses": key_cache_misses,
        "join_graph_build_time_seconds": join_graph_build_time,
        "subbucket_pairs_considered": subbucket_pairs_considered,
        "subbucket_pairs_rejected": subbucket_pairs_rejected,
        "small_bypass_calls": 0,
        "local_bypass_calls": 0,
        "cumulative_bypass_calls": 0,
        "indexed_activation_count": 1,
        "join_work_estimate": join_work_estimate,
        "label_pairs_materialized": label_pairs_materialized,
        "label_pairs_rejected": label_pairs_rejected,
        "candidate_pairs_accepted": len(pairs),
    }


def _bucketed_join_generators(
    forward_labels: list[_Label],
    backward_labels: list[_BackwardLabel],
    graph: TransformedGraph,
    duals: PricingDuals,
    farkas: bool,
    compatible_key_cache: dict[_JoinLookupKey, tuple[_JoinLookupKey, ...]] | None = None,
    small_join_pair_threshold: int = 5_000,
    small_join_cumulative_threshold: int = 250_000,
    max_join_bypass_calls: int = 1_000,
    previous_join_bypass_calls: int = 0,
    cumulative_pair_count: int | None = None,
    payload_bin_width: float = 1.0,
    force_indexed: bool = False,
    stage_counter: _JoinStageCounter | None = None,
    pricing_tolerance: float = 0.0,
    enable_join_lower_envelope: bool = True,
    enable_bucket_join_envelope: bool = True,
) -> dict[str, object]:
    total_pair_count = len(forward_labels) * len(backward_labels)
    join_work_estimate = total_pair_count if cumulative_pair_count is None else cumulative_pair_count
    use_direct = (
        not force_indexed
        and total_pair_count <= small_join_pair_threshold
        and join_work_estimate <= small_join_cumulative_threshold
        and previous_join_bypass_calls < max_join_bypass_calls
    )
    if use_direct:
        generator = _make_join_generator(
            "pair_batch",
            tuple(forward_labels),
            tuple(backward_labels),
            duals,
            farkas,
            enable_join_lower_envelope,
        )
        return {
            "generators": [generator],
            "bucket_pairs_considered": 0,
            "bucket_pairs_rejected": 0,
            "bucket_candidate_pairs": total_pair_count,
            "bucket_lower_envelope_rejects": 0,
            "compatible_keys_generated": 0,
            "compatible_key_lookups": 0,
            "bucket_scans_avoided": 0,
            "key_generation_time_seconds": 0.0,
            "key_cache_hits": 0,
            "key_cache_misses": 0,
            "join_graph_build_time_seconds": 0.0,
            "subbucket_pairs_considered": 0,
            "subbucket_pairs_rejected": 0,
            "small_bypass_calls": 1,
            "local_bypass_calls": 1,
            "cumulative_bypass_calls": 1,
            "indexed_activation_count": 0,
            "join_work_estimate": join_work_estimate,
            "label_pairs_rejected": 0,
            "candidate_pairs_accepted": total_pair_count,
        }

    forward_buckets = _join_buckets(forward_labels, graph, forward=True)
    backward_buckets = _join_buckets(backward_labels, graph, forward=False)
    backward_by_lookup: dict[_JoinLookupKey, list[_JoinBucket]] = {}
    for backward_bucket in backward_buckets:
        backward_by_lookup.setdefault(_join_lookup_key(backward_bucket.key), []).append(backward_bucket)
    generators: list[_JoinGenerator] = []
    bucket_pairs_considered = 0
    bucket_pairs_rejected = 0
    bucket_candidate_pairs = 0
    bucket_lower_envelope_rejects = 0
    compatible_keys_generated = 0
    compatible_key_lookups = 0
    bucket_scans_avoided = 0
    key_generation_time = 0.0
    key_cache_hits = 0
    key_cache_misses = 0
    graph_build_start = time.time()
    join_graph: dict[int, tuple[_JoinBucket, ...]] = {}
    for index, forward_bucket in enumerate(forward_buckets):
        forward_lookup_key = _join_lookup_key(forward_bucket.key)
        if compatible_key_cache is not None and forward_lookup_key in compatible_key_cache:
            compatible_keys = compatible_key_cache[forward_lookup_key]
            key_cache_hits += 1
        else:
            key_start = time.time()
            compatible_keys = _compatible_join_lookup_keys(forward_bucket.key, graph)
            key_generation_time += time.time() - key_start
            if compatible_key_cache is not None:
                compatible_key_cache[forward_lookup_key] = compatible_keys
            key_cache_misses += 1
        compatible_keys_generated += len(compatible_keys)
        probed_bucket_count = sum(len(backward_by_lookup[lookup_key]) for lookup_key in compatible_keys if lookup_key in backward_by_lookup)
        bucket_scans_avoided += max(len(backward_buckets) - probed_bucket_count, 0)
        compatible_buckets: list[_JoinBucket] = []
        for lookup_key in compatible_keys:
            compatible_key_lookups += 1
            compatible_buckets.extend(backward_by_lookup.get(lookup_key, ()))
        join_graph[index] = tuple(compatible_buckets)
    join_graph_build_time = time.time() - graph_build_start

    for index, forward_bucket in enumerate(forward_buckets):
        for backward_bucket in join_graph[index]:
            bucket_pairs_considered += 1
            pair_count = len(forward_bucket.labels) * len(backward_bucket.labels)
            if not _join_buckets_compatible(forward_bucket, backward_bucket, graph):
                bucket_pairs_rejected += pair_count
                _increment_join_stage(
                    stage_counter,
                    _join_bucket_rejection_stage(forward_bucket, backward_bucket, graph) or "key",
                    pair_count,
                )
                continue
            bucket_candidate_pairs += pair_count
            generator = _make_join_generator(
                "bucket",
                tuple(forward_bucket.labels),  # type: ignore[arg-type]
                tuple(backward_bucket.labels),  # type: ignore[arg-type]
                duals,
                farkas,
                enable_join_lower_envelope,
            )
            if (
                enable_bucket_join_envelope
                and not farkas
                and enable_join_lower_envelope
                and generator.lower_bound >= -pricing_tolerance
            ):
                bucket_lower_envelope_rejects += pair_count
                _increment_join_stage(stage_counter, "reduced_cost", pair_count)
                continue
            generators.append(generator)

    return {
        "generators": generators,
        "bucket_pairs_considered": bucket_pairs_considered,
        "bucket_pairs_rejected": bucket_pairs_rejected,
        "bucket_candidate_pairs": bucket_candidate_pairs,
        "bucket_lower_envelope_rejects": bucket_lower_envelope_rejects,
        "compatible_keys_generated": compatible_keys_generated,
        "compatible_key_lookups": compatible_key_lookups,
        "bucket_scans_avoided": bucket_scans_avoided,
        "key_generation_time_seconds": key_generation_time,
        "key_cache_hits": key_cache_hits,
        "key_cache_misses": key_cache_misses,
        "join_graph_build_time_seconds": join_graph_build_time,
        "subbucket_pairs_considered": 0,
        "subbucket_pairs_rejected": 0,
        "small_bypass_calls": 0,
        "local_bypass_calls": 0,
        "cumulative_bypass_calls": 0,
        "indexed_activation_count": 1,
        "join_work_estimate": join_work_estimate,
        "label_pairs_rejected": 0,
        "candidate_pairs_accepted": bucket_candidate_pairs,
    }


def _make_join_generator(
    level: str,
    forward_labels: tuple[_Label, ...],
    backward_labels: tuple[_BackwardLabel, ...],
    duals: PricingDuals,
    farkas: bool,
    enable_join_lower_envelope: bool,
) -> _JoinGenerator:
    return _JoinGenerator(
        level=level,
        lower_bound=_join_group_lower_bound(forward_labels, backward_labels, duals, farkas, enable_join_lower_envelope),
        forward_labels=forward_labels,
        backward_labels=backward_labels,
        estimated_pair_count=len(forward_labels) * len(backward_labels),
    )


def _join_group_lower_bound(
    forward_labels: tuple[_Label, ...],
    backward_labels: tuple[_BackwardLabel, ...],
    duals: PricingDuals,
    farkas: bool,
    enable_join_lower_envelope: bool = True,
) -> float:
    if farkas or not enable_join_lower_envelope:
        return float("-inf")
    if not forward_labels or not backward_labels:
        return float("inf")
    min_forward = min(label.reduced_cost for label in forward_labels)
    min_suffix = float("inf")
    for label in backward_labels:
        if label.lower_envelope is None:
            raise ValueError("backward label is missing its suffix lower envelope")
        min_suffix = min(min_suffix, label.lower_envelope.evaluate(duals, farkas=False))
    return min_forward + min_suffix


def _split_join_generator(
    generator: _JoinGenerator,
    graph: TransformedGraph,
    duals: PricingDuals,
    farkas: bool,
    payload_bin_width: float,
    pricing_tolerance: float,
    enable_join_lower_envelope: bool,
    enable_bucket_join_envelope: bool,
) -> dict[str, object]:
    forward_bucket = _make_join_bucket(_join_bucket_key(generator.forward_labels[0], graph, True), list(generator.forward_labels))
    backward_bucket = _make_join_bucket(_join_bucket_key(generator.backward_labels[0], graph, False), list(generator.backward_labels))
    generators: list[_JoinGenerator] = []
    subbucket_pairs_considered = 0
    subbucket_pairs_rejected = 0
    subbucket_lower_envelope_rejects = 0
    for forward_subbucket in _join_subbuckets(forward_bucket, payload_bin_width):
        for backward_subbucket in _join_subbuckets(backward_bucket, payload_bin_width):
            pair_count = len(forward_subbucket.labels) * len(backward_subbucket.labels)
            subbucket_pairs_considered += 1
            if forward_subbucket.min_payload + backward_subbucket.min_payload > graph.instance.truck_payload + PAYLOAD_TOLERANCE:
                subbucket_pairs_rejected += pair_count
                continue
            subgenerator = _make_join_generator(
                "subbucket",
                tuple(forward_subbucket.labels),  # type: ignore[arg-type]
                tuple(backward_subbucket.labels),  # type: ignore[arg-type]
                duals,
                farkas,
                enable_join_lower_envelope,
            )
            if (
                enable_bucket_join_envelope
                and not farkas
                and enable_join_lower_envelope
                and subgenerator.lower_bound >= -pricing_tolerance
            ):
                subbucket_lower_envelope_rejects += pair_count
                continue
            generators.append(subgenerator)
    return {
        "generators": generators,
        "subbucket_pairs_considered": subbucket_pairs_considered,
        "subbucket_pairs_rejected": subbucket_pairs_rejected,
        "subbucket_lower_envelope_rejects": subbucket_lower_envelope_rejects,
    }


def _iter_join_generator_pairs(
    generator: _JoinGenerator,
    graph: TransformedGraph,
):
    backward_by_payload = sorted(generator.backward_labels, key=lambda label: (label.truck_load, label.path))
    for forward_label in sorted(generator.forward_labels, key=lambda label: (label.reduced_cost, label.truck_load, label.path)):
        max_backward_payload = graph.instance.truck_payload - forward_label.truck_load
        for backward_label in backward_by_payload:
            if backward_label.truck_load > max_backward_payload + PAYLOAD_TOLERANCE:
                break
            yield forward_label, backward_label


def _join_buckets(
    labels: list[_Label] | list[_BackwardLabel],
    graph: TransformedGraph,
    forward: bool,
) -> list[_JoinBucket]:
    grouped: dict[_JoinBucketKey, list[_Label] | list[_BackwardLabel]] = {}
    for label in labels:
        key = _join_bucket_key(label, graph, forward)
        grouped.setdefault(key, []).append(label)
    return [_make_join_bucket(key, grouped[key]) for key in sorted(grouped, key=str)]


def _estimate_join_work(
    forward_by_node: dict[str, list[_Label]],
    backward_by_node: dict[str, list[_BackwardLabel]],
) -> int:
    return sum(
        len(forward_by_node[node]) * len(backward_by_node[node])
        for node in forward_by_node.keys() & backward_by_node.keys()
    )


def _join_bucket_key(
    label: _Label | _BackwardLabel,
    graph: TransformedGraph,
    forward: bool,
) -> _JoinBucketKey:
    meet_node = label.path[-1] if forward else label.path[0]
    physical_loc = duplicate_hub(meet_node) if is_duplicate(meet_node) else meet_node
    if forward:
        active_pad = label.active_pad  # type: ignore[union-attr]
        block_pos = _block_position(label, graph)  # type: ignore[arg-type]
    else:
        active_pad = label.leading_block_hub  # type: ignore[union-attr]
        block_pos = label.leading_block_count  # type: ignore[union-attr]
    return _JoinBucketKey(
        meet_node=meet_node,
        physical_loc=physical_loc,
        active_pad=active_pad,
        block_pos_class=block_pos,
        branch_state_hash=hash(label.branch_state) if label.branch_state is not None else 0,
        payload_bucket=0,
    )


def _make_join_bucket(
    key: _JoinBucketKey,
    labels: list[_Label] | list[_BackwardLabel],
) -> _JoinBucket:
    if not labels:
        raise ValueError("join bucket cannot be empty")
    union_customers = frozenset().union(*(label.represented for label in labels))
    intersection_customers = frozenset(labels[0].represented)
    union_truck_nodes = frozenset().union(*(label.truck_visited for label in labels))
    intersection_truck_nodes = frozenset(labels[0].truck_visited)
    for label in labels[1:]:
        intersection_customers = intersection_customers.intersection(label.represented)
        intersection_truck_nodes = intersection_truck_nodes.intersection(label.truck_visited)
    return _JoinBucket(
        key=key,
        labels=labels,
        labels_by_payload=sorted(labels, key=lambda label: (label.truck_load, label.path)),
        union_customer_set=union_customers,
        intersection_customer_set=intersection_customers,
        union_truck_node_set=union_truck_nodes,
        intersection_truck_node_set=intersection_truck_nodes,
        min_payload=min(label.truck_load for label in labels),
        max_payload=max(label.truck_load for label in labels),
    )


def _join_subbuckets(bucket: _JoinBucket, payload_bin_width: float) -> list[_JoinSubbucket]:
    grouped: dict[_JoinSubbucketKey, list[_Label] | list[_BackwardLabel]] = {}
    for label in bucket.labels:
        key = _JoinSubbucketKey(
            payload_bin=int(label.truck_load // payload_bin_width),
            active_pad=bucket.key.active_pad,
            block_pos_class=bucket.key.block_pos_class,
            branch_state_hash=bucket.key.branch_state_hash,
            physical_mask_class=hash(frozenset(label.truck_visited)),
        )
        grouped.setdefault(key, []).append(label)
    subbuckets: list[_JoinSubbucket] = []
    for key in sorted(grouped, key=str):
        labels = grouped[key]
        subbuckets.append(
            _JoinSubbucket(
                key=key,
                labels=labels,
                labels_by_payload=sorted(labels, key=lambda item: (item.truck_load, item.path)),
                min_payload=min(label.truck_load for label in labels),
                max_payload=max(label.truck_load for label in labels),
            )
        )
    return subbuckets


def _join_lookup_key(key: _JoinBucketKey) -> _JoinLookupKey:
    return _JoinLookupKey(
        meet_node=key.meet_node,
        physical_loc=key.physical_loc,
        active_pad=key.active_pad,
        block_pos_class=key.block_pos_class,
    )


def _compatible_join_lookup_keys(forward_key: _JoinBucketKey, graph: TransformedGraph) -> tuple[_JoinLookupKey, ...]:
    keys: set[_JoinLookupKey] = {
        _JoinLookupKey(forward_key.meet_node, forward_key.physical_loc, None, 0)
    }
    remaining_block_slots = graph.instance.drones_per_truck - forward_key.block_pos_class
    if remaining_block_slots >= 0:
        if forward_key.active_pad is None:
            for hub in graph.instance.hubs:
                for block_pos in range(remaining_block_slots + 1):
                    keys.add(_JoinLookupKey(forward_key.meet_node, forward_key.physical_loc, hub, block_pos))
        else:
            for block_pos in range(remaining_block_slots + 1):
                keys.add(_JoinLookupKey(forward_key.meet_node, forward_key.physical_loc, forward_key.active_pad, block_pos))
    return tuple(sorted(keys, key=str))


def _join_buckets_compatible(
    forward_bucket: _JoinBucket,
    backward_bucket: _JoinBucket,
    graph: TransformedGraph,
) -> bool:
    return _join_bucket_rejection_stage(forward_bucket, backward_bucket, graph) is None


def _join_bucket_rejection_stage(
    forward_bucket: _JoinBucket,
    backward_bucket: _JoinBucket,
    graph: TransformedGraph,
) -> str | None:
    if forward_bucket.key.meet_node != backward_bucket.key.meet_node:
        return "key"
    if forward_bucket.key.physical_loc != backward_bucket.key.physical_loc:
        return "key"
    if forward_bucket.intersection_customer_set.intersection(backward_bucket.intersection_customer_set):
        return "customer"
    repeated_truck_nodes = forward_bucket.intersection_truck_node_set.intersection(
        backward_bucket.intersection_truck_node_set
    ) - {forward_bucket.key.physical_loc}
    if repeated_truck_nodes:
        return "truck_node"
    if forward_bucket.min_payload + backward_bucket.min_payload > graph.instance.truck_payload + PAYLOAD_TOLERANCE:
        return "payload"
    backward_pad = backward_bucket.key.active_pad
    if backward_pad is not None:
        forward_pad = forward_bucket.key.active_pad
        if forward_pad is not None and forward_pad != backward_pad:
            return "block"
        if forward_bucket.key.block_pos_class + backward_bucket.key.block_pos_class > graph.instance.drones_per_truck:
            return "block"
    return None


def _candidate_route_cost_from_reduced_cost(
    candidate: _RouteCandidate,
    duals: PricingDuals,
    farkas: bool,
) -> float | None:
    if farkas:
        return None
    return (
        candidate.reduced_cost
        + sum(duals.mu[customer] for customer in candidate.served)
        + sum(dual * (len(candidate.served.intersection(triplet)) // 2) for triplet, dual in duals.nu.items())
        + duals.kappa
    )


def _activation_mode(bypass_calls: int, indexed_calls: int) -> str:
    if bypass_calls and indexed_calls:
        return "mixed"
    if indexed_calls:
        return "indexed_dominated"
    if bypass_calls:
        return "direct_dominated"
    return "none"


def _pricing_status_from_reason(termination_reason: str, has_columns: bool) -> str:
    if termination_reason in {"productive_batch_found", "closure_negative_batch_found"}:
        return PRICING_STATUS_NEGATIVE_BATCH
    if termination_reason in {"complete_label_and_join_exhaustion", "exact_pricing_complete"}:
        return PRICING_STATUS_EXHAUSTED_NO_NEGATIVE
    if termination_reason == "time_limit_with_columns":
        return PRICING_STATUS_TIME_LIMIT_WITH_COLUMNS
    if termination_reason == "time_limit_unresolved":
        return (
            PRICING_STATUS_TIME_LIMIT_WITH_COLUMNS
            if has_columns
            else PRICING_STATUS_TIME_LIMIT_NO_COLUMNS
        )
    if termination_reason == "interrupted_after_first_hit":
        return "INTERRUPTED"
    return "UNSPECIFIED"


def _known_signature_costs(
    existing_routes: dict[tuple[str, ...], Route] | None,
    existing_column_paths: set[tuple[str, ...]] | None,
    graph: TransformedGraph,
    residual_customers: frozenset[str],
    cache: RouteSignatureCache | None = None,
    active_sr: tuple[tuple[str, str, str], ...] = tuple(),
    active_sr_version: int = 0,
) -> dict[RouteCoefficientSignature, float]:
    if existing_routes is None or existing_column_paths is None:
        return {}
    costs: dict[RouteCoefficientSignature, float] = {}
    for path in existing_column_paths:
        route = existing_routes[path]
        signature = route_signature(route, graph, residual_customers, cache, active_sr, active_sr_version)
        coeff_signature = route_coefficient_signature(signature)
        costs[coeff_signature] = min(costs.get(coeff_signature, float("inf")), route.cost)
    return costs


def _duplicate_dominated_by_existing(
    route: Route,
    known_signature_costs: dict[RouteCoefficientSignature, float],
    graph: TransformedGraph,
    residual_customers: frozenset[str],
    cache: RouteSignatureCache | None = None,
    active_sr: tuple[tuple[str, str, str], ...] = tuple(),
    active_sr_version: int = 0,
) -> tuple[bool, tuple[object, ...]]:
    signature = route_signature(route, graph, residual_customers, cache, active_sr, active_sr_version)
    coeff_signature = route_coefficient_signature(signature)
    incumbent_cost = known_signature_costs.get(coeff_signature)
    if incumbent_cost is not None and incumbent_cost <= route.cost + PAPER_DOMINANCE_TOLERANCE:
        if cache is not None:
            if abs(incumbent_cost - route.cost) <= COST_SIGNATURE_TOLERANCE:
                cache.stats.duplicate_equivalent_rejected += 1
            else:
                cache.stats.cost_dominated_rejected += 1
        return True, signature
    return False, signature


def _pricing_result(
    routes: list[Route],
    reduced_costs: list[float],
    best_route: Route | None,
    best_cost: float | None,
    labels_generated: int,
    labels_dominated: int,
    labels_pruned: int,
    labels_purged: int,
    stale_labels_skipped: int,
    standard_bound_pruned: int,
    farkas_bound_pruned: int,
    max_queue_size: int,
    complete_routes_generated: int,
    exact_completion: bool,
    termination_reason: str | None = None,
    certification_mode: str | None = None,
    elapsed_seconds: float = 0.0,
    pricing_engine: str = "forward_labeling",
    forward_labels_generated: int | None = None,
    backward_labels_generated: int = 0,
    backward_dominance_tests: int = 0,
    backward_labels_dominated: int = 0,
    backward_cost_function_build_time_seconds: float = 0.0,
    backward_cost_function_eval_time_seconds: float = 0.0,
    join_sr_correction_time_seconds: float = 0.0,
    join_active_block_time_seconds: float = 0.0,
    joined_reduced_cost_evaluations: int = 0,
    backward_dominance_cost_tests: int = 0,
    backward_dominance_cost_rejected: int = 0,
    backward_exclusive_resource_violations: int = 0,
    join_pairs_tested: int = 0,
    joined_routes_accepted: int = 0,
    forward_labeling_time_seconds: float | None = None,
    backward_labeling_time_seconds: float = 0.0,
    join_time_seconds: float = 0.0,
    parallel_labeling_used: bool = False,
    parallel_workers: int = 1,
    parallel_calls: int = 0,
    signature_cache_hits: int = 0,
    signature_cache_misses: int = 0,
    core_signature_cache_hits: int = 0,
    core_signature_cache_misses: int = 0,
    active_signature_cache_hits: int = 0,
    active_signature_cache_misses: int = 0,
    sr_coeff_cache_hits: int = 0,
    sr_coeff_cache_misses: int = 0,
    active_sr_key_cache_hits: int = 0,
    active_sr_key_cache_misses: int = 0,
    active_sr_coeffs_computed: int = 0,
    triplet_masks_built: int = 0,
    dominance_prefilter_pairs: int = 0,
    dominance_prefilter_rejected: int = 0,
    dominance_bucket_pairs_considered: int = 0,
    dominance_bucket_pairs_rejected: int = 0,
    dominance_bucket_candidate_pairs: int = 0,
    dominance_compatible_keys_generated: int = 0,
    dominance_compatible_key_lookups: int = 0,
    dominance_bucket_scans_avoided: int = 0,
    dominance_key_generation_time_seconds: float = 0.0,
    backward_full_dominance_tests: int = 0,
    join_prefilter_pairs: int = 0,
    join_prefilter_rejected: int = 0,
    join_bucket_pairs_considered: int = 0,
    join_bucket_pairs_rejected: int = 0,
    join_bucket_candidate_pairs: int = 0,
    join_compatible_keys_generated: int = 0,
    join_compatible_key_lookups: int = 0,
    join_bucket_scans_avoided: int = 0,
    join_key_generation_time_seconds: float = 0.0,
    join_key_cache_hits: int = 0,
    join_key_cache_misses: int = 0,
    join_graph_build_time_seconds: float = 0.0,
    join_subbucket_pairs_considered: int = 0,
    join_subbucket_pairs_rejected: int = 0,
    join_small_bypass_calls: int = 0,
    join_local_bypass_calls: int = 0,
    join_cumulative_bypass_calls: int = 0,
    join_indexed_activation_count: int = 0,
    join_work_estimate: int = 0,
    join_candidate_pairs_accepted: int = 0,
    join_activation_mode: str = "none",
    join_label_pairs_materialized: int = 0,
    join_full_decodes: int = 0,
    lazy_rejected_before_decode: int = 0,
    fully_decoded_routes: int = 0,
    duplicate_equivalent_rejected: int = 0,
    cost_dominated_rejected: int = 0,
    signature_build_time_seconds: float = 0.0,
    sr_coeff_build_time_seconds: float = 0.0,
    duplicate_lookup_time_seconds: float = 0.0,
    route_decode_time_seconds: float = 0.0,
    reduced_cost_verification_time_seconds: float = 0.0,
    dominance_key_cache_hits: int = 0,
    dominance_key_cache_misses: int = 0,
    dominance_small_bypass_calls: int = 0,
    dominance_bypass_calls: int = 0,
    dominance_indexed_activation_count: int = 0,
    dominance_work_estimate: int = 0,
    dominance_activation_mode: str = "none",
    sticky_indexed_join_activations: int = 0,
    sticky_indexed_dominance_activations: int = 0,
    join_stage_reject_key: int = 0,
    join_stage_reject_branch: int = 0,
    join_stage_reject_customer: int = 0,
    join_stage_reject_truck_node: int = 0,
    join_stage_reject_payload: int = 0,
    join_stage_reject_block: int = 0,
    join_stage_reject_reduced_cost: int = 0,
    dominance_stage_reject_key: int = 0,
    dominance_stage_reject_branch: int = 0,
    dominance_stage_reject_customer: int = 0,
    dominance_stage_reject_truck_node: int = 0,
    dominance_stage_reject_payload: int = 0,
    dominance_stage_reject_block: int = 0,
    dominance_stage_reject_time: int = 0,
    dominance_stage_reject_cost: int = 0,
    pricing_mode: str = "productive",
    pricing_yield_ratio: float = 0.0,
    side_pool_routes: list[Route] | None = None,
    side_pool_reduced_costs: list[float] | None = None,
    side_pool_candidates_seen: int = 0,
    side_pool_routes_retained: int = 0,
    side_pool_routes_rejected_by_budget: int = 0,
    join_pairs_key_compatible: int = 0,
    join_pairs_after_bitset_filters: int = 0,
    join_lower_envelope_rejects: int = 0,
    join_bucket_lower_envelope_rejects: int = 0,
    join_subbucket_lower_envelope_rejects: int = 0,
    join_pair_lower_envelope_rejects: int = 0,
    join_queue_pushes: int = 0,
    join_queue_pops: int = 0,
    join_generator_queue_pushes: int = 0,
    join_generator_queue_pops: int = 0,
    join_generator_splits: int = 0,
    join_materialized_pairs: int = 0,
    join_exact_rc_evals: int = 0,
    join_exact_rc_time_seconds: float = 0.0,
    interface_cache_hits: int = 0,
    interface_cache_misses: int = 0,
    suffix_profile_cache_hits: int = 0,
    suffix_profile_cache_misses: int = 0,
    interface_profile_cache_hits: int = 0,
    interface_profile_cache_misses: int = 0,
    pricing_status: str | None = None,
    productive_calls: int = 0,
    certification_calls: int = 0,
    negative_routes_verified: int = 0,
    negative_routes_inserted: int | None = None,
    pricing_worker_backend: str = "serial",
    process_cpu_time_seconds: float = 0.0,
    cpu_core_equivalent: float = 0.0,
    worker_id: int | None = None,
    source_neighbor_count: int = 0,
    source_neighbor_block_sizes: tuple[int, ...] = tuple(),
    first_hit_worker_id: int | None = None,
    first_hit_exits: int = 0,
    interrupted_worker_calls: int = 0,
    certification_worker_calls: int = 0,
    productive_worker_calls: int = 0,
    per_worker_elapsed_seconds: tuple[tuple[int, float], ...] = tuple(),
    per_worker_cpu_time_seconds: tuple[tuple[int, float], ...] = tuple(),
    per_worker_labels_generated: tuple[tuple[int, int], ...] = tuple(),
    per_worker_labels_dominated: tuple[tuple[int, int], ...] = tuple(),
    per_worker_labels_pruned: tuple[tuple[int, int], ...] = tuple(),
    per_worker_completed_labels: tuple[tuple[int, int], ...] = tuple(),
    per_worker_verified_negative_routes: tuple[tuple[int, int], ...] = tuple(),
    pricing_pool_startup_time_seconds: float = 0.0,
    pricing_pool_startup_count: int = 0,
    pricing_pool_reused_calls: int = 0,
    pricing_pool_shutdown_time_seconds: float = 0.0,
    pricing_task_submission_time_seconds: float = 0.0,
    pricing_worker_payload_count: int = 0,
    pricing_worker_response_count: int = 0,
    pricing_candidate_paths_before_merge: int = 0,
    pricing_candidate_paths_after_merge: int = 0,
    pricing_decoded_routes_in_main: int = 0,
    pricing_verified_routes_in_main: int = 0,
    pricing_batch_target: int = 0,
    pricing_returned_batch_size: int = 0,
    pricing_first_hit_enabled: bool = False,
    pricing_stale_response_rejections: int = 0,
    pricing_worker_cpu_time_seconds: float = 0.0,
    pricing_main_process_cpu_time_seconds: float = 0.0,
    pricing_main_merge_time_seconds: float = 0.0,
    productive_slice_seconds: float = 0.0,
    productive_slice_deadline_used: bool = False,
    adaptive_slice_seconds: float = 0.0,
    productive_yield_window_rate: float = 0.0,
    stabilized_dual_enabled: bool = False,
    stabilized_candidates_returned: int = 0,
    true_dual_rejected_candidates: int = 0,
    mean_worker_rc_minus_true_rc: float = 0.0,
    max_abs_worker_true_rc_discrepancy: float = 0.0,
    prefix_task_depth: int = 1,
    productive_time_limit_with_columns: int = 0,
    productive_time_limit_no_columns: int = 0,
    source_neighbor_task_count: int = 0,
    source_neighbor_task_sizes: tuple[int, ...] = tuple(),
    local_worker_candidate_quota: int = 0,
    diversity_quota: int = 0,
    diversity_selected_routes: int = 0,
    diversity_selected_customers: int = 0,
    verified_candidates_by_source_neighbor: tuple[tuple[str, int], ...] = tuple(),
    selected_candidates_by_source_neighbor: tuple[tuple[str, int], ...] = tuple(),
    extensions_attempted: int = 0,
    extensions_rejected_by_deadline: int = 0,
    deadline_reachability_removed: int = 0,
    reward_set_size_before_deadline: int = 0,
    reward_set_size_after_deadline: int = 0,
    deadline_reward_bound_calls: int = 0,
    deadline_dominance_prefilter_skips: int = 0,
    routes_rejected_by_deadline_in_master: int = 0,
    forward_dominance_tests: int = 0,
    forward_same_node_dominance_tests: int = 0,
    forward_physical_location_dominance_tests: int = 0,
    forward_physical_location_dominance_rejections: int = 0,
    forward_return_time_credit_checks: int = 0,
    forward_return_time_credit_checks_skipped: int = 0,
    forward_branch_language_failures: int = 0,
    forward_mask_scalar_prefilter_failures: int = 0,
    dom_gate_pairs_seen: int = 0,
    dom_gate_mask_failures: int = 0,
    dom_gate_scalar_failures: int = 0,
    dom_gate_branch_failures: int = 0,
    dom_gate_deadline_failures: int = 0,
    labels_dominated_same_node: int = 0,
    labels_dominated_physical: int = 0,
    dom_prefilter_pairs: int = 0,
    dom_prefilter_mask_fail: int = 0,
    dom_prefilter_branch_fail: int = 0,
    dom_prefilter_payload_fail: int = 0,
    dom_prefilter_block_fail: int = 0,
    dom_prefilter_return_credit_fail: int = 0,
    dom_full_tests: int = 0,
    dom_full_rejections: int = 0,
    physical_location_full_tests: int = 0,
    physical_location_rejections: int = 0,
) -> PricingResult:
    if forward_labels_generated is None:
        forward_labels_generated = labels_generated
    if forward_labeling_time_seconds is None:
        forward_labeling_time_seconds = elapsed_seconds
    if termination_reason is None:
        termination_reason = "exact_pricing_complete" if exact_completion else "productive_batch_found"
    if pricing_status is None:
        pricing_status = _pricing_status_from_reason(termination_reason, bool(routes))
    if negative_routes_inserted is None:
        negative_routes_inserted = len(routes)
    if productive_calls == 0 and certification_calls == 0:
        if pricing_mode == "closure":
            certification_calls = 1
        else:
            productive_calls = 1
    if certification_mode is None:
        if exact_completion:
            certification_mode = (
                "bidirectional_complete_meet_node"
                if pricing_engine == "bidirectional_forward_backward"
                else "complete_forward_fallback"
            )
        else:
            certification_mode = "not_certified_productive"
    diagnostics = PricingDiagnostics(
        labels_generated=labels_generated,
        labels_dominated=labels_dominated,
        labels_pruned=labels_pruned,
        labels_purged=labels_purged,
        stale_labels_skipped=stale_labels_skipped,
        standard_bound_pruned=standard_bound_pruned,
        farkas_bound_pruned=farkas_bound_pruned,
        max_queue_size=max_queue_size,
        complete_routes_generated=complete_routes_generated,
        returned_routes=len(routes),
        best_reduced_cost=best_cost,
        exact_completion=exact_completion,
        termination_reason=termination_reason,
        certification_mode=certification_mode,
        elapsed_seconds=elapsed_seconds,
        pricing_engine=pricing_engine,
        forward_labels_generated=forward_labels_generated,
        backward_labels_generated=backward_labels_generated,
        backward_dominance_tests=backward_dominance_tests,
        backward_labels_dominated=backward_labels_dominated,
        backward_cost_function_build_time_seconds=backward_cost_function_build_time_seconds,
        backward_cost_function_eval_time_seconds=backward_cost_function_eval_time_seconds,
        join_sr_correction_time_seconds=join_sr_correction_time_seconds,
        join_active_block_time_seconds=join_active_block_time_seconds,
        joined_reduced_cost_evaluations=joined_reduced_cost_evaluations,
        backward_dominance_cost_tests=backward_dominance_cost_tests,
        backward_dominance_cost_rejected=backward_dominance_cost_rejected,
        backward_exclusive_resource_violations=backward_exclusive_resource_violations,
        join_pairs_tested=join_pairs_tested,
        joined_routes_accepted=joined_routes_accepted,
        forward_labeling_time_seconds=forward_labeling_time_seconds,
        backward_labeling_time_seconds=backward_labeling_time_seconds,
        join_time_seconds=join_time_seconds,
        parallel_labeling_used=parallel_labeling_used,
        parallel_workers=parallel_workers,
        parallel_calls=parallel_calls,
        signature_cache_hits=signature_cache_hits,
        signature_cache_misses=signature_cache_misses,
        core_signature_cache_hits=core_signature_cache_hits,
        core_signature_cache_misses=core_signature_cache_misses,
        active_signature_cache_hits=active_signature_cache_hits,
        active_signature_cache_misses=active_signature_cache_misses,
        sr_coeff_cache_hits=sr_coeff_cache_hits,
        sr_coeff_cache_misses=sr_coeff_cache_misses,
        active_sr_key_cache_hits=active_sr_key_cache_hits,
        active_sr_key_cache_misses=active_sr_key_cache_misses,
        active_sr_coeffs_computed=active_sr_coeffs_computed,
        triplet_masks_built=triplet_masks_built,
        dominance_prefilter_pairs=dominance_prefilter_pairs,
        dominance_prefilter_rejected=dominance_prefilter_rejected,
        dominance_bucket_pairs_considered=dominance_bucket_pairs_considered,
        dominance_bucket_pairs_rejected=dominance_bucket_pairs_rejected,
        dominance_bucket_candidate_pairs=dominance_bucket_candidate_pairs,
        dominance_compatible_keys_generated=dominance_compatible_keys_generated,
        dominance_compatible_key_lookups=dominance_compatible_key_lookups,
        dominance_bucket_scans_avoided=dominance_bucket_scans_avoided,
        dominance_key_generation_time_seconds=dominance_key_generation_time_seconds,
        backward_full_dominance_tests=backward_full_dominance_tests,
        join_prefilter_pairs=join_prefilter_pairs,
        join_prefilter_rejected=join_prefilter_rejected,
        join_bucket_pairs_considered=join_bucket_pairs_considered,
        join_bucket_pairs_rejected=join_bucket_pairs_rejected,
        join_bucket_candidate_pairs=join_bucket_candidate_pairs,
        join_compatible_keys_generated=join_compatible_keys_generated,
        join_compatible_key_lookups=join_compatible_key_lookups,
        join_bucket_scans_avoided=join_bucket_scans_avoided,
        join_key_generation_time_seconds=join_key_generation_time_seconds,
        join_key_cache_hits=join_key_cache_hits,
        join_key_cache_misses=join_key_cache_misses,
        join_graph_build_time_seconds=join_graph_build_time_seconds,
        join_subbucket_pairs_considered=join_subbucket_pairs_considered,
        join_subbucket_pairs_rejected=join_subbucket_pairs_rejected,
        join_small_bypass_calls=join_small_bypass_calls,
        join_local_bypass_calls=join_local_bypass_calls,
        join_cumulative_bypass_calls=join_cumulative_bypass_calls,
        join_indexed_activation_count=join_indexed_activation_count,
        join_work_estimate=join_work_estimate,
        join_candidate_pairs_accepted=join_candidate_pairs_accepted,
        join_activation_mode=join_activation_mode,
        join_label_pairs_materialized=join_label_pairs_materialized,
        join_full_decodes=join_full_decodes,
        lazy_rejected_before_decode=lazy_rejected_before_decode,
        fully_decoded_routes=fully_decoded_routes,
        duplicate_equivalent_rejected=duplicate_equivalent_rejected,
        cost_dominated_rejected=cost_dominated_rejected,
        signature_build_time_seconds=signature_build_time_seconds,
        sr_coeff_build_time_seconds=sr_coeff_build_time_seconds,
        duplicate_lookup_time_seconds=duplicate_lookup_time_seconds,
        route_decode_time_seconds=route_decode_time_seconds,
        reduced_cost_verification_time_seconds=reduced_cost_verification_time_seconds,
        dominance_key_cache_hits=dominance_key_cache_hits,
        dominance_key_cache_misses=dominance_key_cache_misses,
        dominance_small_bypass_calls=dominance_small_bypass_calls,
        dominance_bypass_calls=dominance_bypass_calls,
        dominance_indexed_activation_count=dominance_indexed_activation_count,
        dominance_work_estimate=dominance_work_estimate,
        dominance_activation_mode=dominance_activation_mode,
        sticky_indexed_join_activations=sticky_indexed_join_activations,
        sticky_indexed_dominance_activations=sticky_indexed_dominance_activations,
        join_stage_reject_key=join_stage_reject_key,
        join_stage_reject_branch=join_stage_reject_branch,
        join_stage_reject_customer=join_stage_reject_customer,
        join_stage_reject_truck_node=join_stage_reject_truck_node,
        join_stage_reject_payload=join_stage_reject_payload,
        join_stage_reject_block=join_stage_reject_block,
        join_stage_reject_reduced_cost=join_stage_reject_reduced_cost,
        dominance_stage_reject_key=dominance_stage_reject_key,
        dominance_stage_reject_branch=dominance_stage_reject_branch,
        dominance_stage_reject_customer=dominance_stage_reject_customer,
        dominance_stage_reject_truck_node=dominance_stage_reject_truck_node,
        dominance_stage_reject_payload=dominance_stage_reject_payload,
        dominance_stage_reject_block=dominance_stage_reject_block,
        dominance_stage_reject_time=dominance_stage_reject_time,
        dominance_stage_reject_cost=dominance_stage_reject_cost,
        pricing_mode=pricing_mode,
        pricing_yield_ratio=pricing_yield_ratio,
        side_pool_routes_returned=0 if side_pool_routes is None else len(side_pool_routes),
        side_pool_reduced_cost_min=(
            None
            if not side_pool_reduced_costs
            else min(side_pool_reduced_costs)
        ),
        side_pool_candidates_seen=side_pool_candidates_seen,
        side_pool_routes_retained=side_pool_routes_retained,
        side_pool_routes_rejected_by_budget=side_pool_routes_rejected_by_budget,
        join_pairs_key_compatible=join_pairs_key_compatible,
        join_pairs_after_bitset_filters=join_pairs_after_bitset_filters,
        join_lower_envelope_rejects=join_lower_envelope_rejects,
        join_bucket_lower_envelope_rejects=join_bucket_lower_envelope_rejects,
        join_subbucket_lower_envelope_rejects=join_subbucket_lower_envelope_rejects,
        join_pair_lower_envelope_rejects=join_pair_lower_envelope_rejects,
        join_queue_pushes=join_queue_pushes,
        join_queue_pops=join_queue_pops,
        join_generator_queue_pushes=join_generator_queue_pushes,
        join_generator_queue_pops=join_generator_queue_pops,
        join_generator_splits=join_generator_splits,
        join_materialized_pairs=join_materialized_pairs,
        join_exact_rc_evals=join_exact_rc_evals,
        join_exact_rc_time_seconds=join_exact_rc_time_seconds,
        interface_cache_hits=interface_cache_hits,
        interface_cache_misses=interface_cache_misses,
        suffix_profile_cache_hits=suffix_profile_cache_hits,
        suffix_profile_cache_misses=suffix_profile_cache_misses,
        interface_profile_cache_hits=interface_profile_cache_hits,
        interface_profile_cache_misses=interface_profile_cache_misses,
        pricing_status=pricing_status,
        productive_calls=productive_calls,
        certification_calls=certification_calls,
        negative_routes_verified=negative_routes_verified,
        negative_routes_inserted=negative_routes_inserted,
        pricing_worker_backend=pricing_worker_backend,
        process_cpu_time_seconds=process_cpu_time_seconds,
        cpu_core_equivalent=cpu_core_equivalent,
        worker_id=worker_id,
        source_neighbor_count=source_neighbor_count,
        source_neighbor_block_sizes=source_neighbor_block_sizes,
        first_hit_worker_id=first_hit_worker_id,
        first_hit_exits=first_hit_exits,
        interrupted_worker_calls=interrupted_worker_calls,
        certification_worker_calls=certification_worker_calls,
        productive_worker_calls=productive_worker_calls,
        per_worker_elapsed_seconds=per_worker_elapsed_seconds,
        per_worker_cpu_time_seconds=per_worker_cpu_time_seconds,
        per_worker_labels_generated=per_worker_labels_generated,
        per_worker_labels_dominated=per_worker_labels_dominated,
        per_worker_labels_pruned=per_worker_labels_pruned,
        per_worker_completed_labels=per_worker_completed_labels,
        per_worker_verified_negative_routes=per_worker_verified_negative_routes,
        pricing_pool_startup_time_seconds=pricing_pool_startup_time_seconds,
        pricing_pool_startup_count=pricing_pool_startup_count,
        pricing_pool_reused_calls=pricing_pool_reused_calls,
        pricing_pool_shutdown_time_seconds=pricing_pool_shutdown_time_seconds,
        pricing_task_submission_time_seconds=pricing_task_submission_time_seconds,
        pricing_worker_payload_count=pricing_worker_payload_count,
        pricing_worker_response_count=pricing_worker_response_count,
        pricing_candidate_paths_before_merge=pricing_candidate_paths_before_merge,
        pricing_candidate_paths_after_merge=pricing_candidate_paths_after_merge,
        pricing_decoded_routes_in_main=pricing_decoded_routes_in_main,
        pricing_verified_routes_in_main=pricing_verified_routes_in_main,
        pricing_batch_target=pricing_batch_target,
        pricing_returned_batch_size=pricing_returned_batch_size,
        pricing_first_hit_enabled=pricing_first_hit_enabled,
        pricing_stale_response_rejections=pricing_stale_response_rejections,
        pricing_worker_cpu_time_seconds=pricing_worker_cpu_time_seconds,
        pricing_main_process_cpu_time_seconds=pricing_main_process_cpu_time_seconds,
        pricing_main_merge_time_seconds=pricing_main_merge_time_seconds,
        productive_slice_seconds=productive_slice_seconds,
        productive_slice_deadline_used=productive_slice_deadline_used,
        adaptive_slice_seconds=adaptive_slice_seconds,
        productive_yield_window_rate=productive_yield_window_rate,
        stabilized_dual_enabled=stabilized_dual_enabled,
        stabilized_candidates_returned=stabilized_candidates_returned,
        true_dual_rejected_candidates=true_dual_rejected_candidates,
        mean_worker_rc_minus_true_rc=mean_worker_rc_minus_true_rc,
        max_abs_worker_true_rc_discrepancy=max_abs_worker_true_rc_discrepancy,
        prefix_task_depth=prefix_task_depth,
        productive_time_limit_with_columns=productive_time_limit_with_columns,
        productive_time_limit_no_columns=productive_time_limit_no_columns,
        source_neighbor_task_count=source_neighbor_task_count,
        source_neighbor_task_sizes=source_neighbor_task_sizes,
        local_worker_candidate_quota=local_worker_candidate_quota,
        diversity_quota=diversity_quota,
        diversity_selected_routes=diversity_selected_routes,
        diversity_selected_customers=diversity_selected_customers,
        verified_candidates_by_source_neighbor=verified_candidates_by_source_neighbor,
        selected_candidates_by_source_neighbor=selected_candidates_by_source_neighbor,
        extensions_attempted=extensions_attempted,
        extensions_rejected_by_deadline=extensions_rejected_by_deadline,
        deadline_reachability_removed=deadline_reachability_removed,
        reward_set_size_before_deadline=reward_set_size_before_deadline,
        reward_set_size_after_deadline=reward_set_size_after_deadline,
        deadline_reward_bound_calls=deadline_reward_bound_calls,
        deadline_dominance_prefilter_skips=deadline_dominance_prefilter_skips,
        routes_rejected_by_deadline_in_master=routes_rejected_by_deadline_in_master,
        forward_dominance_tests=forward_dominance_tests,
        forward_same_node_dominance_tests=forward_same_node_dominance_tests,
        forward_physical_location_dominance_tests=forward_physical_location_dominance_tests,
        forward_physical_location_dominance_rejections=forward_physical_location_dominance_rejections,
        forward_return_time_credit_checks=forward_return_time_credit_checks,
        forward_return_time_credit_checks_skipped=forward_return_time_credit_checks_skipped,
        forward_branch_language_failures=forward_branch_language_failures,
        forward_mask_scalar_prefilter_failures=forward_mask_scalar_prefilter_failures,
        dom_gate_pairs_seen=dom_gate_pairs_seen,
        dom_gate_mask_failures=dom_gate_mask_failures,
        dom_gate_scalar_failures=dom_gate_scalar_failures,
        dom_gate_branch_failures=dom_gate_branch_failures,
        dom_gate_deadline_failures=dom_gate_deadline_failures,
        labels_dominated_same_node=labels_dominated_same_node,
        labels_dominated_physical=labels_dominated_physical,
        dom_prefilter_pairs=dom_prefilter_pairs,
        dom_prefilter_mask_fail=dom_prefilter_mask_fail,
        dom_prefilter_branch_fail=dom_prefilter_branch_fail,
        dom_prefilter_payload_fail=dom_prefilter_payload_fail,
        dom_prefilter_block_fail=dom_prefilter_block_fail,
        dom_prefilter_return_credit_fail=dom_prefilter_return_credit_fail,
        dom_full_tests=dom_full_tests,
        dom_full_rejections=dom_full_rejections,
        physical_location_full_tests=physical_location_full_tests,
        physical_location_rejections=physical_location_rejections,
    )
    return PricingResult(
        tuple(routes),
        tuple(reduced_costs),
        best_route,
        best_cost,
        diagnostics,
        tuple(side_pool_routes or ()),
        tuple(side_pool_reduced_costs or ()),
    )


def route_reduced_cost(route: Route, duals: PricingDuals) -> float:
    return (
        route.cost
        - sum(duals.mu[customer] for customer in route.served)
        - sum(dual * route.sr_coeff(triplet) for triplet, dual in duals.nu.items())
        - duals.kappa
    )


def route_farkas_reduced_cost(route: Route, duals: PricingDuals) -> float:
    return (
        -duals.kappa
        - sum(duals.mu[customer] for customer in route.served)
        - sum(dual * route.sr_coeff(triplet) for triplet, dual in duals.nu.items())
    )


def _extension_allowed(
    label: _Label,
    next_node: str,
    graph: TransformedGraph,
    residual_customers: frozenset[str],
    restrictions: BranchRestrictions,
) -> bool:
    instance = graph.instance
    node = label.path[-1]
    arc = (node, next_node)
    if next_node == instance.depot_source:
        return False
    if arc in restrictions.trans_arc_forbidden:
        return False
    if next_node in instance.customers + instance.hubs and next_node in label.truck_visited:
        return False
    if is_customer_representation(next_node, instance):
        customer = served_customer(next_node)
        if customer not in residual_customers or customer in label.represented:
            return False
        if label.truck_load + instance.demand[customer] > instance.truck_payload + PAYLOAD_TOLERANCE:
            return False
        if customer in restrictions.truck_service and is_duplicate(next_node):
            return False
        if customer in restrictions.drone_service and next_node == customer:
            return False
        required_pads = [hub for hub, c in restrictions.pad_required if c == customer]
        if required_pads and (not is_duplicate(next_node) or duplicate_hub(next_node) not in required_pads):
            return False
        if is_duplicate(next_node) and (duplicate_hub(next_node), customer) in restrictions.pad_forbidden:
            return False
        for p, q in restrictions.separate_pairs:
            if customer == p and q in label.represented:
                return False
            if customer == q and p in label.represented:
                return False
    if is_duplicate(next_node):
        hub = duplicate_hub(next_node)
        customer = duplicate_customer(next_node)
        if label.active_pad not in {None, hub}:
            return False
        if label.block_count + 1 > instance.drones_per_truck:
            return False
        if (hub, customer) not in instance.drone_arcs:
            return False
    return True


def _extension_rejected_by_service_deadline(
    label: _Label,
    next_node: str,
    graph: TransformedGraph,
    objective: ObjectiveData,
) -> bool:
    customer = served_customer(next_node) if is_customer_representation(next_node, graph.instance) else None
    if customer is None:
        return False
    service_time = _tentative_extension_service_time(label, next_node, graph)
    if service_time is None:
        return False
    return service_time > objective.bounds.service_ub[customer] + 1e-9


def _tentative_extension_service_time(label: _Label, next_node: str, graph: TransformedGraph) -> float | None:
    instance = graph.instance
    node = label.path[-1]
    arc = (node, next_node)
    if arc in graph.truck_arcs and next_node in instance.customers:
        return label.physical_time + instance.truck_time[arc]
    if arc in graph.hub_duplicate_arcs:
        hub = duplicate_hub(next_node)
        customer = duplicate_customer(next_node)
        return label.physical_time + instance.drone_time[(hub, customer)]
    if arc in graph.duplicate_duplicate_arcs:
        hub = duplicate_hub(next_node)
        customer = duplicate_customer(next_node)
        return label.active_pad_arrival + instance.drone_time[(hub, customer)]
    if arc in graph.duplicate_regular_arcs and next_node in instance.customers:
        hub = duplicate_hub(node)
        return label.active_pad_arrival + label.active_wait + instance.truck_time[(hub, next_node)]
    return None


def _extend(
    label: _Label,
    next_node: str,
    graph: TransformedGraph,
    objective: ObjectiveData,
    duals: PricingDuals,
    active_sr: tuple[tuple[str, str, str], ...],
    farkas: bool,
    restrictions: BranchRestrictions | None = None,
    arc_customer_sets: dict[tuple[str, str], frozenset[str]] | None = None,
) -> _Label:
    instance = graph.instance
    node = label.path[-1]
    arc = (node, next_node)
    represented = set(label.represented)
    truck_visited = set(label.truck_visited)
    truck_load = label.truck_load
    active_pad = label.active_pad
    active_pad_arrival = label.active_pad_arrival
    active_wait = label.active_wait
    block_count = label.block_count
    physical_time = label.physical_time
    service_times = dict(label.service_times)
    sr_counts = dict(label.sr_counts)
    reduced_cost = label.reduced_cost
    truck_served = set(label.truck_served)
    pad_served = set(label.pad_served)

    if arc in graph.truck_arcs:
        physical_time += instance.truck_time[arc]
        if next_node in instance.customers + instance.hubs:
            truck_visited.add(next_node)
        active_pad = next_node if next_node in instance.hubs else active_pad
        active_pad_arrival = physical_time if next_node in instance.hubs else active_pad_arrival
        active_wait = 0.0
        block_count = 0
        if next_node in instance.customers:
            reduced_cost = _add_customer_cost(next_node, physical_time, reduced_cost, objective, duals, sr_counts, active_sr, farkas)
            represented.add(next_node)
            truck_served.add(next_node)
            truck_load += instance.demand[next_node]
            service_times[next_node] = physical_time
        elif next_node == instance.depot_sink and not farkas:
            reduced_cost += objective.coeffs.return_time * physical_time
    elif arc in graph.hub_duplicate_arcs or arc in graph.duplicate_duplicate_arcs:
        hub = duplicate_hub(next_node)
        customer = duplicate_customer(next_node)
        if arc in graph.hub_duplicate_arcs:
            active_pad = hub
            active_pad_arrival = physical_time
            active_wait = 0.0
            block_count = 0
        service_time = active_pad_arrival + instance.drone_time[(hub, customer)]
        active_wait = max(active_wait, instance.drone_trip_time[(hub, customer)])
        block_count += 1
        reduced_cost = _add_customer_cost(customer, service_time, reduced_cost, objective, duals, sr_counts, active_sr, farkas)
        if not farkas:
            reduced_cost += objective.coeffs.cost * instance.drone_cost
        represented.add(customer)
        pad_served.add((hub, customer))
        truck_load += instance.demand[customer]
        service_times[customer] = service_time
    elif arc in graph.duplicate_regular_arcs:
        hub = duplicate_hub(node)
        physical_time = active_pad_arrival + active_wait + instance.truck_time[(hub, next_node)]
        if next_node in instance.customers + instance.hubs:
            truck_visited.add(next_node)
        active_pad = next_node if next_node in instance.hubs else active_pad
        active_pad_arrival = physical_time if next_node in instance.hubs else active_pad_arrival
        active_wait = 0.0
        block_count = 0
        if next_node in instance.customers:
            reduced_cost = _add_customer_cost(next_node, physical_time, reduced_cost, objective, duals, sr_counts, active_sr, farkas)
            represented.add(next_node)
            truck_served.add(next_node)
            truck_load += instance.demand[next_node]
            service_times[next_node] = physical_time
        elif next_node == instance.depot_sink and not farkas:
            reduced_cost += objective.coeffs.return_time * physical_time
    else:
        raise ValueError(f"unclassified transformed arc: {arc}")

    represented_frozen = frozenset(represented)
    truck_visited_frozen = frozenset(truck_visited)
    used_arcs = label.used_arcs | {arc}
    branch_state = None
    if restrictions is not None and arc_customer_sets is not None:
        branch_state = _branch_state_from_resources(represented_frozen, used_arcs, restrictions, arc_customer_sets)
    return _Label(
        path=label.path + (next_node,),
        represented=represented_frozen,
        truck_visited=truck_visited_frozen,
        truck_load=truck_load,
        active_pad=active_pad,
        active_pad_arrival=active_pad_arrival,
        active_wait=active_wait,
        block_count=block_count,
        physical_time=physical_time,
        service_times=tuple(sorted(service_times.items())),
        sr_counts=tuple(sorted(sr_counts.items())),
        reduced_cost=reduced_cost,
        used_arcs=used_arcs,
        truck_served=frozenset(truck_served),
        pad_served=frozenset(pad_served),
        represented_mask=_customer_mask(represented_frozen, graph),
        truck_node_mask=_truck_node_mask(truck_visited_frozen, graph),
        branch_state=branch_state,
    )


def _add_customer_cost(
    customer: str,
    service_time: float,
    reduced_cost: float,
    objective: ObjectiveData,
    duals: PricingDuals,
    sr_counts: dict[tuple[str, str, str], int],
    active_sr: tuple[tuple[str, str, str], ...],
    farkas: bool,
) -> float:
    if not farkas:
        reduced_cost += objective.coeffs.delay * (service_time - objective.bounds.arrival_lb[customer]) ** 2
    reduced_cost -= duals.mu[customer]
    for triplet in active_sr:
        old = sr_counts[triplet]
        if customer in triplet:
            sr_counts[triplet] = old + 1
            if old < 2 <= sr_counts[triplet]:
                reduced_cost -= duals.nu[triplet]
    return reduced_cost


def _complete_allowed(
    label: _Label,
    graph: TransformedGraph,
    restrictions: BranchRestrictions,
    arc_customer_sets: dict[tuple[str, str], frozenset[str]],
) -> bool:
    if label.path in restrictions.route_forbidden:
        return False
    for p, q in restrictions.together_pairs:
        if (p in label.represented) != (q in label.represented):
            return False
    for arc in restrictions.trans_arc_required:
        if label.represented.intersection(arc_customer_sets[arc]) and arc not in label.used_arcs:
            return False
    return True


def _materialize_best_route(
    best_route: Route | None,
    best_path: tuple[str, ...] | None,
    next_route_id: int,
    graph: TransformedGraph,
    objective: ObjectiveData,
) -> Route | None:
    if best_route is not None or best_path is None:
        return best_route
    return route_from_path(next_route_id, best_path, graph, objective)


def _insert_nondominated_standard_label(
    kept_labels: dict[str, list[_Label]],
    label: _Label,
    graph: TransformedGraph,
    objective: ObjectiveData,
    duals: PricingDuals,
    restrictions: BranchRestrictions,
    arc_customer_sets: dict[tuple[str, str], frozenset[str]],
    deadline_counters: _DeadlinePricingCounters | None = None,
) -> tuple[bool, int, int]:
    location = _physical_location(label)
    comparable = kept_labels.setdefault(location, [])
    if any(
        _paper_dominates(incumbent, label, graph, objective, duals, restrictions, arc_customer_sets, deadline_counters)
        for incumbent in comparable
    ):
        return False, 1, 0
    survivors = []
    dominated_count = 0
    for incumbent in comparable:
        if _paper_dominates(label, incumbent, graph, objective, duals, restrictions, arc_customer_sets, deadline_counters):
            dominated_count += 1
        else:
            survivors.append(incumbent)
    survivors.append(label)
    kept_labels[location] = survivors
    return True, 0, dominated_count


def _insert_nondominated_farkas_label(
    kept_labels: dict[str, list[_Label]],
    label: _Label,
    graph: TransformedGraph,
    restrictions: BranchRestrictions,
    arc_customer_sets: dict[tuple[str, str], frozenset[str]],
) -> tuple[bool, int, int]:
    location = _physical_location(label)
    comparable = kept_labels.setdefault(location, [])
    if any(_farkas_dominates(incumbent, label, graph, restrictions, arc_customer_sets) for incumbent in comparable):
        return False, 1, 0
    survivors = []
    dominated_count = 0
    for incumbent in comparable:
        if _farkas_dominates(label, incumbent, graph, restrictions, arc_customer_sets):
            dominated_count += 1
        else:
            survivors.append(incumbent)
    survivors.append(label)
    kept_labels[location] = survivors
    return True, 0, dominated_count


def _paper_dominates(
    a: _Label,
    b: _Label,
    graph: TransformedGraph,
    objective: ObjectiveData,
    duals: PricingDuals,
    restrictions: BranchRestrictions,
    arc_customer_sets: dict[tuple[str, str], frozenset[str]],
    deadline_counters: _DeadlinePricingCounters | None = None,
) -> bool:
    if deadline_counters is not None:
        deadline_counters.forward_dominance_tests += 1
        deadline_counters.dom_gate_pairs_seen += 1
        deadline_counters.dom_prefilter_pairs += 1
    comparable, return_credit = _block_comparable_return_credit(a, b, graph)
    if not comparable:
        if deadline_counters is not None:
            deadline_counters.forward_mask_scalar_prefilter_failures += 1
            deadline_counters.dom_gate_scalar_failures += 1
            deadline_counters.dom_prefilter_return_credit_fail += 1
            deadline_counters.forward_return_time_credit_checks_skipped += 1
        return False
    same_original_node = _same_original_node(a, b, graph)
    if not a.represented.issubset(b.represented):
        if deadline_counters is not None:
            deadline_counters.forward_mask_scalar_prefilter_failures += 1
            deadline_counters.dom_gate_mask_failures += 1
            deadline_counters.dom_prefilter_mask_fail += 1
            deadline_counters.forward_return_time_credit_checks_skipped += 1
        return False
    if not a.truck_visited.issubset(b.truck_visited):
        if deadline_counters is not None:
            deadline_counters.forward_mask_scalar_prefilter_failures += 1
            deadline_counters.dom_gate_mask_failures += 1
            deadline_counters.dom_prefilter_mask_fail += 1
            deadline_counters.forward_return_time_credit_checks_skipped += 1
        return False
    if not a.represented and a.represented != b.represented:
        if deadline_counters is not None:
            deadline_counters.forward_mask_scalar_prefilter_failures += 1
            deadline_counters.dom_gate_mask_failures += 1
            deadline_counters.dom_prefilter_mask_fail += 1
            deadline_counters.forward_return_time_credit_checks_skipped += 1
        return False
    if _branch_state(a, restrictions, arc_customer_sets) != _branch_state(b, restrictions, arc_customer_sets):
        if deadline_counters is not None:
            deadline_counters.forward_branch_language_failures += 1
            deadline_counters.dom_gate_branch_failures += 1
            deadline_counters.dom_prefilter_branch_fail += 1
            deadline_counters.forward_return_time_credit_checks_skipped += 1
        return False
    if deadline_counters is not None:
        a_mask = _cached_deadline_reachable_mask(a, graph, objective, deadline_counters)
        b_mask = _cached_deadline_reachable_mask(b, graph, objective, deadline_counters)
        if b_mask & ~a_mask:
            deadline_counters.deadline_dominance_prefilter_skips += 1
            deadline_counters.dom_gate_deadline_failures += 1
            deadline_counters.dom_prefilter_mask_fail += 1
            deadline_counters.forward_return_time_credit_checks_skipped += 1
            return False
    if a.truck_load > b.truck_load or a.block_count > b.block_count:
        if deadline_counters is not None:
            deadline_counters.forward_mask_scalar_prefilter_failures += 1
            deadline_counters.dom_gate_scalar_failures += 1
            if a.truck_load > b.truck_load:
                deadline_counters.dom_prefilter_payload_fail += 1
            else:
                deadline_counters.dom_prefilter_block_fail += 1
            deadline_counters.forward_return_time_credit_checks_skipped += 1
        return False
    if same_original_node:
        if a.physical_time > b.physical_time:
            if deadline_counters is not None:
                deadline_counters.forward_mask_scalar_prefilter_failures += 1
                deadline_counters.dom_gate_scalar_failures += 1
                deadline_counters.dom_prefilter_return_credit_fail += 1
                deadline_counters.forward_return_time_credit_checks_skipped += 1
            return False
    else:
        if a.active_pad_arrival > b.active_pad_arrival or a.active_wait > b.active_wait:
            if deadline_counters is not None:
                deadline_counters.forward_mask_scalar_prefilter_failures += 1
                deadline_counters.dom_gate_scalar_failures += 1
                deadline_counters.dom_prefilter_block_fail += 1
                deadline_counters.forward_return_time_credit_checks_skipped += 1
            return False
    if deadline_counters is not None:
        deadline_counters.forward_return_time_credit_checks += 1
    adjusted_cost = a.reduced_cost - _sr_extra_penalty_bound(a, b, duals)
    if adjusted_cost > b.reduced_cost + objective.coeffs.return_time * return_credit:
        if deadline_counters is not None:
            deadline_counters.forward_mask_scalar_prefilter_failures += 1
            deadline_counters.dom_gate_scalar_failures += 1
            deadline_counters.dom_prefilter_return_credit_fail += 1
        return False
    if deadline_counters is not None:
        deadline_counters.dom_full_tests += 1
        if same_original_node:
            deadline_counters.forward_same_node_dominance_tests += 1
        else:
            deadline_counters.forward_physical_location_dominance_tests += 1
            deadline_counters.physical_location_full_tests += 1
    dominated = (
        a.represented != b.represented
        or a.truck_visited != b.truck_visited
        or a.truck_load < b.truck_load
        or a.block_count < b.block_count
        or a.physical_time < b.physical_time
        or (not same_original_node and a.active_pad_arrival < b.active_pad_arrival)
        or (not same_original_node and a.active_wait < b.active_wait)
        or adjusted_cost < b.reduced_cost + objective.coeffs.return_time * return_credit
    )
    if dominated and deadline_counters is not None and not same_original_node:
        deadline_counters.forward_physical_location_dominance_rejections += 1
        deadline_counters.physical_location_rejections += 1
        deadline_counters.labels_dominated_physical += 1
    if dominated and deadline_counters is not None and same_original_node:
        deadline_counters.labels_dominated_same_node += 1
    if dominated and deadline_counters is not None:
        deadline_counters.dom_full_rejections += 1
    return dominated


def _farkas_dominates(
    a: _Label,
    b: _Label,
    graph: TransformedGraph,
    restrictions: BranchRestrictions,
    arc_customer_sets: dict[tuple[str, str], frozenset[str]],
) -> bool:
    comparable, _ = _block_comparable_return_credit(a, b, graph)
    if not comparable:
        return False
    if not a.represented.issubset(b.represented):
        return False
    if not a.truck_visited.issubset(b.truck_visited):
        return False
    if not a.represented and a.represented != b.represented:
        return False
    if a.truck_load > b.truck_load or a.block_count > b.block_count:
        return False
    if _branch_state(a, restrictions, arc_customer_sets) != _branch_state(b, restrictions, arc_customer_sets):
        return False
    if a.sr_counts != b.sr_counts:
        return False
    same_original_node = _same_original_node(a, b, graph)
    if same_original_node:
        if a.physical_time > b.physical_time:
            return False
    elif a.active_pad_arrival > b.active_pad_arrival or a.active_wait > b.active_wait:
        return False
    if a.reduced_cost > b.reduced_cost:
        return False
    return (
        a.represented != b.represented
        or a.truck_visited != b.truck_visited
        or a.truck_load < b.truck_load
        or a.block_count < b.block_count
        or a.physical_time < b.physical_time
        or (not same_original_node and a.active_pad_arrival < b.active_pad_arrival)
        or (not same_original_node and a.active_wait < b.active_wait)
        or a.reduced_cost < b.reduced_cost
    )


def _block_comparable_return_credit(a: _Label, b: _Label, graph: TransformedGraph) -> tuple[bool, float]:
    a_node = a.path[-1]
    b_node = b.path[-1]
    if a_node == b_node and a_node in graph.instance.nodes:
        return True, max(b.physical_time - a.physical_time, 0.0)
    if a_node == b_node and is_duplicate(a_node):
        if a.active_pad != b.active_pad:
            return False, 0.0
        return True, max(b.active_pad_arrival - a.active_pad_arrival, 0.0)
    a_loc = _physical_location(a)
    b_loc = _physical_location(b)
    if a_loc not in graph.instance.hubs or a_loc != b_loc or a.active_pad != b.active_pad:
        return False, 0.0
    if _block_position(a, graph) > _block_position(b, graph):
        return False, 0.0
    return True, max(b.active_pad_arrival - a.active_pad_arrival, 0.0)


def _same_original_node(a: _Label, b: _Label, graph: TransformedGraph) -> bool:
    node = a.path[-1]
    return node == b.path[-1] and node in graph.instance.nodes


def _physical_location(label: _Label) -> str:
    node = label.path[-1]
    return duplicate_hub(node) if is_duplicate(node) else node


def _block_position(label: _Label, graph: TransformedGraph) -> int:
    node = label.path[-1]
    if node in graph.instance.hubs:
        return 0
    if is_duplicate(node):
        return graph.order[(duplicate_hub(node), duplicate_customer(node))] + 1
    return 0


def _branch_state(
    label: _Label,
    restrictions: BranchRestrictions,
    arc_customer_sets: dict[tuple[str, str], frozenset[str]],
) -> _BranchState:
    return _branch_state_from_resources(label.represented, label.used_arcs, restrictions, arc_customer_sets)


def _branch_state_from_resources(
    represented: frozenset[str],
    used_arcs: frozenset[tuple[str, str]],
    restrictions: BranchRestrictions,
    arc_customer_sets: dict[tuple[str, str], frozenset[str]],
) -> _BranchState:
    together = tuple(
        (p in represented, q in represented)
        for p, q in sorted(restrictions.together_pairs)
    )
    required_arcs = tuple(
        (arc, bool(represented.intersection(arc_customer_sets[arc])), arc in used_arcs)
        for arc in sorted(restrictions.trans_arc_required)
    )
    return _BranchState(together=together, required_arcs=required_arcs)


def _sr_extra_penalty_bound(a: _Label, b: _Label, duals: PricingDuals) -> float:
    a_counts = dict(a.sr_counts)
    b_counts = dict(b.sr_counts)
    penalty = 0.0
    for triplet, dual in duals.nu.items():
        if dual < 0.0 and a_counts.get(triplet, 0) in {1, 3} and b_counts.get(triplet, 0) in {0, 2}:
            penalty += dual
    return penalty


def _shortest_truck_times(graph: TransformedGraph) -> dict[tuple[str, str], float]:
    shortest = dict(nx.all_pairs_dijkstra_path_length(graph.instance.truck_graph(), weight="weight"))
    return {
        (i, j): float(shortest.get(i, {}).get(j, float("inf")))
        for i in graph.instance.nodes
        for j in graph.instance.nodes
    }


def _queue_key(
    label: _Label,
    graph: TransformedGraph,
    objective: ObjectiveData,
    shortest: dict[tuple[str, str], float],
    bounds: _PricingBounds,
    farkas: bool,
    use_standard_acceleration: bool,
    deadline_counters: _DeadlinePricingCounters | None = None,
) -> float:
    if farkas:
        return label.reduced_cost
    if not use_standard_acceleration:
        return label.reduced_cost
    return _knapsack_reduced_cost_lower_bound(label, graph, objective, shortest, bounds, deadline_counters)


def _knapsack_reduced_cost_lower_bound(
    label: _Label,
    graph: TransformedGraph,
    objective: ObjectiveData,
    shortest: dict[tuple[str, str], float],
    bounds: _PricingBounds,
    deadline_counters: _DeadlinePricingCounters | None = None,
) -> float:
    completion_return = _completion_return_lb(label, graph, shortest)
    reward = _dual_reward_bound(label, graph, bounds, objective, shortest, deadline_counters)
    return label.reduced_cost + objective.coeffs.return_time * completion_return - reward


def _farkas_reduced_cost_lower_bound(
    label: _Label,
    graph: TransformedGraph,
    bounds: _PricingBounds,
) -> float:
    reward = _dual_reward_bound(label, graph, bounds)
    return label.reduced_cost - reward


def _completion_return_lb(
    label: _Label,
    graph: TransformedGraph,
    shortest: dict[tuple[str, str], float],
) -> float:
    instance = graph.instance
    node = label.path[-1]
    if is_duplicate(node):
        hub = duplicate_hub(node)
        return label.active_pad_arrival + label.active_wait + shortest[(hub, instance.depot_sink)]
    return label.physical_time + shortest[(node, instance.depot_sink)]


def _dual_reward_bound(
    label: _Label,
    graph: TransformedGraph,
    bounds: _PricingBounds,
    objective: ObjectiveData | None = None,
    shortest: dict[tuple[str, str], float] | None = None,
    deadline_counters: _DeadlinePricingCounters | None = None,
) -> float:
    instance = graph.instance
    residual_payload = instance.truck_payload - label.truck_load + PAYLOAD_TOLERANCE
    current = duplicate_hub(label.path[-1]) if is_duplicate(label.path[-1]) else label.path[-1]
    items_before_deadline = tuple(
        item
        for item in bounds.reward_items_by_location.get(current, tuple())
        if item.customer not in label.represented and item.demand <= residual_payload
    )
    if objective is None or shortest is None:
        items = items_before_deadline
    else:
        items = tuple(
            item
            for item in items_before_deadline
            if _customer_deadline_reachable_from_label(label, item.customer, graph, objective, shortest)
        )
        if deadline_counters is not None:
            deadline_counters.deadline_reward_bound_calls += 1
            deadline_counters.reward_set_size_before_deadline += len(items_before_deadline)
            deadline_counters.reward_set_size_after_deadline += len(items)
            deadline_counters.deadline_reachability_removed += len(items_before_deadline) - len(items)
    if not items:
        return 0.0
    payload_reward = _payload_reward_bound(items, residual_payload)
    cardinality_limit = _max_additional_reward_customers(label, graph, items)
    cardinality_reward = _cardinality_reward_bound(items, cardinality_limit)
    return min(payload_reward, cardinality_reward)


def _payload_reward_bound(items: tuple[_RewardItem, ...], residual_payload: float) -> float:
    remaining = residual_payload
    reward = 0.0
    for item in items:
        if remaining <= 0.0:
            break
        demand = item.demand
        value = item.reward
        if demand <= remaining:
            reward += value
            remaining -= demand
        else:
            reward += value * remaining / demand
            break
    return reward


def _max_additional_reward_customers(
    label: _Label,
    graph: TransformedGraph,
    items: tuple[_RewardItem, ...],
) -> int:
    instance = graph.instance
    candidate_customers = frozenset(item.customer for item in items)
    direct_capacity = sum(1 for customer in candidate_customers if customer not in label.truck_visited)
    current_block_capacity = 0
    if is_duplicate(label.path[-1]):
        current_block_capacity = instance.drones_per_truck - label.block_count
    unvisited_hub_capacity = instance.drones_per_truck * sum(
        1 for hub in instance.hubs if hub not in label.truck_visited
    )
    structural_capacity = direct_capacity + current_block_capacity + unvisited_hub_capacity
    return min(len(items), max(0, structural_capacity))


def _cardinality_reward_bound(items: tuple[_RewardItem, ...], cardinality_limit: int) -> float:
    if cardinality_limit <= 0:
        return 0.0
    reward = 0.0
    remaining = float(cardinality_limit)
    for item in sorted(items, key=lambda item: (-item.reward, item.customer)):
        if remaining <= 0.0:
            break
        if remaining >= 1.0:
            reward += item.reward
            remaining -= 1.0
        else:
            reward += item.reward * remaining
            break
    return reward


def _build_pricing_bounds(
    graph: TransformedGraph,
    duals: PricingDuals,
    residual_customers: frozenset[str],
    shortest: dict[tuple[str, str], float],
) -> _PricingBounds:
    instance = graph.instance
    reward_items_by_location: dict[str, tuple[_RewardItem, ...]] = {}
    for location in instance.nodes:
        items = []
        for customer in sorted(residual_customers):
            reward = max(duals.mu.get(customer, 0.0), 0.0)
            if reward <= 0.0:
                continue
            if not _customer_reachable_from_location(customer, location, graph, shortest):
                continue
            demand = instance.demand[customer]
            items.append(_RewardItem(customer, demand, reward, reward / demand))
        reward_items_by_location[location] = tuple(sorted(items, key=lambda item: (-item.density, item.customer)))
    return _PricingBounds(reward_items_by_location=reward_items_by_location)


def _customer_reachable_from_location(
    customer: str,
    location: str,
    graph: TransformedGraph,
    shortest: dict[tuple[str, str], float],
) -> bool:
    instance = graph.instance
    truck_reachable = isfinite(shortest[(location, customer)]) and isfinite(shortest[(customer, instance.depot_sink)])
    if truck_reachable:
        return True
    return any(
        isfinite(shortest[(location, hub)])
        and isfinite(shortest[(hub, instance.depot_sink)])
        and (hub, customer) in instance.drone_arcs
        and instance.demand[customer] <= instance.drone_payload
        and instance.drone_trip_time[(hub, customer)] <= instance.drone_endurance
        for hub in instance.hubs
    )


def _customer_deadline_reachable_from_label(
    label: _Label,
    customer: str,
    graph: TransformedGraph,
    objective: ObjectiveData,
    shortest: dict[tuple[str, str], float],
) -> bool:
    if customer in label.represented:
        return False
    upper = objective.bounds.service_ub[customer]
    if not isfinite(upper):
        return True
    instance = graph.instance
    node = label.path[-1]
    location = duplicate_hub(node) if is_duplicate(node) else node
    departure = label.active_pad_arrival + label.active_wait if is_duplicate(node) else label.physical_time
    best = float("inf")
    if customer not in label.truck_visited and isfinite(shortest[(location, customer)]):
        best = min(best, departure + shortest[(location, customer)])
    active_block_available = label.active_pad is not None and (location == label.active_pad or is_duplicate(node))
    if active_block_available and label.block_count < instance.drones_per_truck and (label.active_pad, customer) in instance.drone_arcs:
        best = min(best, label.active_pad_arrival + instance.drone_time[(label.active_pad, customer)])
    for hub in instance.hubs:
        if (hub, customer) not in instance.drone_arcs:
            continue
        if isfinite(shortest[(location, hub)]):
            best = min(best, departure + shortest[(location, hub)] + instance.drone_time[(hub, customer)])
    return best <= upper + 1e-9


def _deadline_reachable_mask(
    label: _Label,
    graph: TransformedGraph,
    objective: ObjectiveData,
    shortest: dict[tuple[str, str], float],
) -> int:
    mask = 0
    for index, customer in enumerate(graph.instance.customers):
        if _customer_deadline_reachable_from_label(label, customer, graph, objective, shortest):
            mask |= 1 << index
    return mask


def _cached_deadline_reachable_mask(
    label: _Label,
    graph: TransformedGraph,
    objective: ObjectiveData,
    counter: _DeadlinePricingCounters,
) -> int:
    if counter.shortest is None:
        raise RuntimeError("deadline dominance prefilter requires shortest-time pricing bounds")
    if label not in counter.reachable_mask_cache:
        counter.reachable_mask_cache[label] = _deadline_reachable_mask(label, graph, objective, counter.shortest)
    return counter.reachable_mask_cache[label]
