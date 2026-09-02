from __future__ import annotations
from concurrent.futures import ThreadPoolExecutor

from dataclasses import dataclass, field, replace

from heapq import heappop, heappush

from itertools import count

from math import ceil, isfinite

import multiprocessing as mp
import os
import pickle
from queue import Empty
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
)

from .objective import ObjectiveData

from .routes import PAYLOAD_TOLERANCE, Route, ServiceEnvelopeViolation, is_customer_representation, route_from_path

from .transform import TransformedGraph, duplicate_customer, is_duplicate, served_customer

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
class PricingEpoch:
    dual_signature: tuple
    active_sr_ids: tuple
    sr_version: int
    residual_customer_mask: int
    branch_signature: tuple
    fixed_route_signature: tuple
    active_column_version: tuple
    rmp_structure_version: tuple
    objective_scale_version: tuple[object, ...]
    service_window_version: tuple[object, ...]

@dataclass(frozen=True)
class PricingEpochContext:
    active_sr_version: int = 0
    fixed_route_signature: tuple[tuple[str, ...], ...] = tuple()
    active_column_version: tuple = tuple()
    rmp_structure_version: tuple = tuple()

@dataclass(frozen=True)
class PricingSchedulerConfig:
    customer_weight: float = 1.0
    out_degree_weight: float = 0.25
    drone_pad_weight: float = 0.5
    deadline_weight: float = 0.5
    split_open_labels_min: int = 2000
    split_gap_factor: float = 1.0
    split_elapsed_min: float = 5.0
    split_work_min: int = 2000
    refinement_depth: int = 2
    checkpoint_extension_period: int = 5000


def _task_closure_gap(completion_lower_bound: float) -> float:
    return max(0.0, -completion_lower_bound)


def _closure_gap_exceeds_split_threshold(
    closure_gap: float,
    pricing_tolerance: float,
    split_gap_factor: float,
) -> bool:
    return closure_gap > split_gap_factor * pricing_tolerance


@dataclass(frozen=True)
class PricingResult:
    routes: tuple[Route, ...]
    reduced_costs: tuple[float, ...]
    best_route: Route | None
    best_reduced_cost: float | None
    diagnostics: "PricingDiagnostics"

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
    pricing_engine: str = "source_neighbor_parallel_forward"
    forward_labels_generated: int = 0
    forward_labeling_time_seconds: float = 0.0
    parallel_labeling_used: bool = False
    parallel_workers: int = 1
    parallel_calls: int = 0
    pricing_mode: str = "productive"
    pricing_status: str = PRICING_STATUS_EXHAUSTED_NO_NEGATIVE
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
    source_neighbor_task_count: int = 0
    source_neighbor_task_sizes: tuple[int, ...] = tuple()
    core_subspace_count: int = 0
    core_empty_blocks: int = 0
    min_core_reduced_cost: float | None = None
    root_closed_by_all_cores: bool = False
    certification_worker_calls: int = 0
    productive_worker_calls: int = 0
    certification_core_closed_count: int = 0
    certification_core_unresolved_count: int = 0
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
    pricing_task_submission_time_seconds: float = 0.0
    pricing_worker_payload_count: int = 0
    pricing_worker_response_count: int = 0
    pricing_candidate_paths_before_merge: int = 0
    pricing_candidate_paths_after_merge: int = 0
    pricing_decoded_routes_in_main: int = 0
    pricing_verified_routes_in_main: int = 0
    pricing_batch_target: int = 0
    pricing_returned_batch_size: int = 0
    prefix_task_depth: int = 1
    extensions_attempted: int = 0
    extensions_rejected_by_deadline: int = 0
    together_branch_reachability_pruned: int = 0
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
    forward_branch_interface_failures: int = 0
    forward_mask_scalar_prefilter_failures: int = 0
    dom_gate_pairs_seen: int = 0
    dom_gate_mask_failures: int = 0
    dom_gate_scalar_failures: int = 0
    dom_gate_branch_failures: int = 0
    dom_gate_deadline_failures: int = 0
    dominance_bucket_pairs_considered: int = 0
    dominance_bucket_pairs_rejected: int = 0
    dominance_bucket_candidate_pairs: int = 0
    dominance_bucket_scans_avoided: int = 0
    dominance_bucket_queries: int = 0
    dominance_bucket_skipped_by_mask: int = 0
    dominance_bucket_skipped_by_scalar: int = 0
    dominance_bucket_skipped_by_branch: int = 0
    dominance_bucket_skipped_by_deadline: int = 0
    dominance_bucket_skipped_by_return_credit: int = 0
    dom_frontier_queries: int = 0
    dom_frontier_keys_scanned: int = 0
    dom_frontier_keys_skipped_by_mask: int = 0
    dom_frontier_keys_skipped_by_branch: int = 0
    dom_frontier_keys_skipped_by_deadline: int = 0
    dom_frontier_keys_skipped_by_return_credit: int = 0
    frontier_cells_created: int = 0
    frontier_cells_split: int = 0
    mask_trie_subset_queries: int = 0
    mask_trie_superset_queries: int = 0
    mask_trie_returned_items: int = 0
    mask_subset_queries: int = 0
    mask_superset_queries: int = 0
    mask_query_cache_hits: int = 0
    mask_query_cache_misses: int = 0
    cell_splits: int = 0
    cell_pair_products_before_split: int = 0
    cell_pairs_considered: int = 0
    cell_pairs_rejected_by_mask: int = 0
    cell_pairs_rejected_by_envelope: int = 0
    label_pairs_materialized: int = 0
    full_same_node_tests: int = 0
    full_physical_location_tests: int = 0
    labels_deleted_same_node: int = 0
    labels_deleted_physical_location: int = 0
    certification_tasks_exhausted_by_label_search: int = 0
    physdom_cell_pairs_considered: int = 0
    physdom_cell_pairs_rejected_by_mask: int = 0
    physdom_cell_pairs_rejected_by_envelope: int = 0
    physdom_label_pairs_materialized: int = 0
    physdom_full_tests: int = 0
    physdom_deletions: int = 0
    physdom_time: float = 0.0
    physdom_time_per_deletion: float = 0.0
    return_credit_incompatible_pairs: int = 0
    dom_pairs_avoided_before_materialization: int = 0
    dom_candidate_pairs_materialized: int = 0
    dom_full_tests_same_node: int = 0
    dom_full_tests_physical_location: int = 0
    dom_labels_deleted_same_node: int = 0
    dom_labels_deleted_physical_location: int = 0
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
    balanced_process_dynamic: bool = False
    initial_block_scores: tuple[float, ...] = tuple()
    initial_load_imbalance_max_mean: float = 0.0
    per_worker_busy_seconds: tuple[tuple[int, float], ...] = tuple()
    per_worker_idle_seconds: tuple[tuple[int, float], ...] = tuple()
    per_worker_task_counts: tuple[tuple[int, int], ...] = tuple()
    idle_work_requests: int = 0
    dynamic_split_candidates: int = 0
    dynamic_splits_performed: int = 0
    dynamic_split_rejected_near_closure: int = 0
    dynamic_split_rejected_small_frontier: int = 0
    dynamic_split_rejected_elapsed: int = 0
    dynamic_split_rejected_low_work: int = 0
    dynamic_child_tasks_created: int = 0
    dynamic_labels_transferred: int = 0
    dynamic_bytes_transferred: int = 0
    dynamic_split_control_seconds: float = 0.0
    leaf_tasks_created: int = 0
    leaf_tasks_closed: int = 0
    pending_transfer_peak: int = 0
    stale_worker_results_discarded: int = 0
    pricing_epoch_invalidations: int = 0
    productive_first_hit_worker: int | None = None
    candidate_verification_seconds: float = 0.0
    master_control_seconds: float = 0.0
    candidate_checkpoints: int = 0
    candidate_worker_resumptions: int = 0
    global_verified_candidates: int = 0
    global_batch_limit_cancellations: int = 0

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
    together_branch_reachability_pruned: int = 0
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
    forward_branch_interface_failures: int = 0
    forward_mask_scalar_prefilter_failures: int = 0
    dom_gate_pairs_seen: int = 0
    dom_gate_mask_failures: int = 0
    dom_gate_scalar_failures: int = 0
    dom_gate_branch_failures: int = 0
    dom_gate_deadline_failures: int = 0
    dominance_bucket_pairs_considered: int = 0
    dominance_bucket_pairs_rejected: int = 0
    dominance_bucket_candidate_pairs: int = 0
    dominance_bucket_scans_avoided: int = 0
    dominance_bucket_queries: int = 0
    dominance_bucket_skipped_by_mask: int = 0
    dominance_bucket_skipped_by_scalar: int = 0
    dominance_bucket_skipped_by_branch: int = 0
    dominance_bucket_skipped_by_deadline: int = 0
    dominance_bucket_skipped_by_return_credit: int = 0
    dom_frontier_queries: int = 0
    dom_frontier_keys_scanned: int = 0
    dom_frontier_keys_skipped_by_mask: int = 0
    dom_frontier_keys_skipped_by_branch: int = 0
    dom_frontier_keys_skipped_by_deadline: int = 0
    dom_frontier_keys_skipped_by_return_credit: int = 0
    frontier_cells_created: int = 0
    frontier_cells_split: int = 0
    mask_trie_subset_queries: int = 0
    mask_trie_superset_queries: int = 0
    mask_trie_returned_items: int = 0
    mask_subset_queries: int = 0
    mask_superset_queries: int = 0
    mask_query_cache_hits: int = 0
    mask_query_cache_misses: int = 0
    cell_splits: int = 0
    cell_pair_products_before_split: int = 0
    cell_pairs_considered: int = 0
    cell_pairs_rejected_by_mask: int = 0
    cell_pairs_rejected_by_envelope: int = 0
    label_pairs_materialized: int = 0
    full_same_node_tests: int = 0
    full_physical_location_tests: int = 0
    labels_deleted_same_node: int = 0
    labels_deleted_physical_location: int = 0
    certification_tasks_exhausted_by_label_search: int = 0
    pricing_mode_productive_or_certification: str = "productive"
    physdom_cell_pairs_considered: int = 0
    physdom_cell_pairs_rejected_by_mask: int = 0
    physdom_cell_pairs_rejected_by_envelope: int = 0
    physdom_label_pairs_materialized: int = 0
    physdom_full_tests: int = 0
    physdom_deletions: int = 0
    physdom_time: float = 0.0
    physdom_time_per_deletion: float = 0.0
    return_credit_incompatible_pairs: int = 0
    dom_pairs_avoided_before_materialization: int = 0
    dom_candidate_pairs_materialized: int = 0
    dom_full_tests_same_node: int = 0
    dom_full_tests_physical_location: int = 0
    dom_labels_deleted_same_node: int = 0
    dom_labels_deleted_physical_location: int = 0
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
    frontier_index: dict[str, "_ForwardFrontierLocationIndex"] = field(default_factory=dict)
    max_frontier_cell_size: int = 512
    max_frontier_pair_product: int = 2000
    max_frontier_split_depth: int = 6

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


@dataclass
class _DominanceExtensionContext:
    objective: ObjectiveData
    residual_customers: frozenset[str]
    restrictions: BranchRestrictions
    feasible_first_successors: dict[_Label, tuple[str, ...]] = field(default_factory=dict)


@dataclass(frozen=True)
class _ForwardSearchCheckpoint:
    open_labels: tuple[_Label, ...]
    completion_lower_bound: float
    best_path: tuple[str, ...] | None
    best_reduced_cost: float
    result: PricingResult


@dataclass(frozen=True)
class _ForwardCandidateCheckpoint:
    open_labels: tuple[_Label, ...]
    completion_lower_bound: float
    best_path: tuple[str, ...] | None
    best_reduced_cost: float
    result: PricingResult

@dataclass(frozen=True)
class _ForwardDomKey:
    endpoint: str
    physical_location: str
    active_pad: str | None
    block_position: int
    branch_state_key: "_BranchState"

@dataclass(frozen=True)
class _ForwardFrontierKey:
    dom_key: _ForwardDomKey
    customer_mask: int
    truck_node_mask: int
    deadline_reachable_mask: int
    branch_language_key: "_BranchState"
    active_block_resource_class: int
    payload_bin: int
    drone_count_bin: int
    time_bin: int
    wait_bin: int

@dataclass
class _MaskTrieNode:
    zero: "_MaskTrieNode | None" = None
    one: "_MaskTrieNode | None" = None
    payloads: list[_ForwardFrontierKey] = field(default_factory=list)

class MaskContainmentTrie:
    def __init__(self, bit_count: int) -> None:
        if bit_count < 0:
            raise ValueError("mask trie bit count must be nonnegative")
        self.bit_count = bit_count
        self.root = _MaskTrieNode()

    def insert(self, mask: int, payload: _ForwardFrontierKey) -> None:
        node = self.root
        for bit in range(self.bit_count):
            if mask & (1 << bit):
                if node.one is None:
                    node.one = _MaskTrieNode()
                node = node.one
            else:
                if node.zero is None:
                    node.zero = _MaskTrieNode()
                node = node.zero
        node.payloads.append(payload)

    def query_subsets(self, mask: int) -> tuple[_ForwardFrontierKey, ...]:
        out: list[_ForwardFrontierKey] = []

        def visit(node: _MaskTrieNode | None, bit: int) -> None:
            if node is None:
                return
            if bit == self.bit_count:
                out.extend(node.payloads)
                return
            visit(node.zero, bit + 1)
            if mask & (1 << bit):
                visit(node.one, bit + 1)

        visit(self.root, 0)
        return tuple(out)

    def query_supersets(self, mask: int) -> tuple[_ForwardFrontierKey, ...]:
        out: list[_ForwardFrontierKey] = []

        def visit(node: _MaskTrieNode | None, bit: int) -> None:
            if node is None:
                return
            if bit == self.bit_count:
                out.extend(node.payloads)
                return
            if mask & (1 << bit):
                visit(node.one, bit + 1)
            else:
                visit(node.zero, bit + 1)
                visit(node.one, bit + 1)

        visit(self.root, 0)
        return tuple(out)

@dataclass
class _ForwardFrontierCell:
    labels: list[_Label] = field(default_factory=list)
    min_payload: float = float("inf")
    max_payload: float = float("-inf")
    min_active_drone_count: int = 10**9
    max_active_drone_count: int = -1
    min_time: float = float("inf")
    max_time: float = float("-inf")
    min_wait: float = float("inf")
    max_wait: float = float("-inf")
    min_reduced_cost: float = float("inf")
    max_reduced_cost: float = float("-inf")

    def add(self, label: _Label) -> None:
        resource_time = _frontier_time_resource(label)
        self.labels.append(label)
        self.min_payload = min(self.min_payload, label.truck_load)
        self.max_payload = max(self.max_payload, label.truck_load)
        self.min_active_drone_count = min(self.min_active_drone_count, label.block_count)
        self.max_active_drone_count = max(self.max_active_drone_count, label.block_count)
        self.min_time = min(self.min_time, resource_time)
        self.max_time = max(self.max_time, resource_time)
        self.min_wait = min(self.min_wait, label.active_wait)
        self.max_wait = max(self.max_wait, label.active_wait)
        self.min_reduced_cost = min(self.min_reduced_cost, label.reduced_cost)
        self.max_reduced_cost = max(self.max_reduced_cost, label.reduced_cost)

@dataclass
class _ForwardFrontierLocationIndex:
    buckets: dict[_ForwardFrontierKey, list[_Label]] = field(default_factory=dict)
    cells: dict[_ForwardFrontierKey, _ForwardFrontierCell] = field(default_factory=dict)
    by_branch_customer_mask: dict[_BranchState, dict[int, list[_ForwardFrontierKey]]] = field(default_factory=dict)
    query_cache: dict[tuple[_BranchState, int, int, int, bool], tuple[_ForwardFrontierKey, ...]] = field(default_factory=dict)
    customer_tries_by_branch: dict[_BranchState, MaskContainmentTrie] = field(default_factory=dict)
    truck_tries_by_branch: dict[_BranchState, MaskContainmentTrie] = field(default_factory=dict)
    deadline_tries_by_branch: dict[_BranchState, MaskContainmentTrie] = field(default_factory=dict)
    labels_count: int = 0

    @property
    def key_count(self) -> int:
        return len(self.buckets)

@dataclass(frozen=True)
class _WorkerPricingTask:
    call_id: int
    dual_id: int
    epoch: PricingEpoch
    worker_id: int
    task_id: int
    generation: int
    source_prefixes: tuple[tuple[str, ...], ...]
    task_root_prefix: tuple[str, ...]
    initial_open_labels: tuple[_Label, ...] | None
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

@dataclass(frozen=True)
class _WorkerPricingResult:
    call_id: int
    dual_id: int
    epoch: PricingEpoch
    worker_id: int
    task_id: int
    generation: int
    route_paths: tuple[tuple[str, ...], ...]
    reduced_costs: tuple[float, ...]
    best_path: tuple[str, ...] | None
    best_reduced_cost: float | None
    diagnostics: PricingDiagnostics

@dataclass(frozen=True)
class _ProcessWorkerCommand:
    kind: str
    task: _WorkerPricingTask | None = None
    epoch: PricingEpoch | None = None
    task_id: int | None = None
    generation: int | None = None
    child_limit: int = 0

@dataclass(frozen=True)
class _ProcessWorkerEvent:
    kind: str
    worker_id: int
    epoch: PricingEpoch | None = None
    task_id: int | None = None
    generation: int | None = None
    result: _WorkerPricingResult | None = None
    open_label_count: int = 0
    completion_lower_bound: float | None = None
    closure_gap: float | None = None
    remaining_work: int = 0
    task_elapsed_seconds: float = 0.0
    child_groups: tuple[tuple[tuple[str, ...], tuple[_Label, ...]], ...] = tuple()
    retained_label_count: int = 0
    process_id: int = 0

@dataclass
class _ProcessLocalTaskState:
    task: _WorkerPricingTask
    open_labels: tuple[_Label, ...] | None
    best_path: tuple[str, ...] | None
    best_reduced_cost: float
    started_at: float
    initialized: bool = False
    recent_extensions: int = 0
    recent_dominance_tests: int = 0

@dataclass(frozen=True)
class _PricingTaskPlan:
    prefixes: tuple[tuple[str, ...], ...]
    source_neighbors: tuple[str, ...]

@dataclass(frozen=True)
class _BalancedTaskPlan:
    source_neighbors: tuple[str, ...]
    blocks: tuple[tuple[str, ...], ...]
    block_scores: tuple[float, ...]
    successor_scores: tuple[tuple[str, float], ...]

def _worker_pricing_result(
    task: _WorkerPricingTask,
    result: PricingResult,
    worker_id: int,
    cpu_seconds: float,
) -> _WorkerPricingResult:
    result = _with_runtime_diagnostics(
        result,
        pricing_worker_backend="worker_process",
        process_cpu_time_seconds=cpu_seconds,
        worker_id=worker_id,
        source_neighbor_count=len({prefix[0] for prefix in task.source_prefixes if prefix}),
        source_neighbor_block_sizes=(len(task.source_prefixes),),
        parallel_workers=1,
        source_neighbor_task_count=1,
        source_neighbor_task_sizes=(len(task.source_prefixes),),
    )
    return _WorkerPricingResult(
        call_id=task.call_id,
        dual_id=task.dual_id,
        epoch=task.epoch,
        worker_id=worker_id,
        task_id=task.task_id,
        generation=task.generation,
        route_paths=tuple(route.path for route in result.routes),
        reduced_costs=result.reduced_costs,
        best_path=None if result.best_route is None else result.best_route.path,
        best_reduced_cost=result.best_reduced_cost,
        diagnostics=result.diagnostics,
    )

def _run_balanced_process_worker(
    worker_id: int,
    command_queue,
    result_queue,
    graph: TransformedGraph,
    objective: ObjectiveData,
    scheduler: PricingSchedulerConfig,
) -> None:
    result_queue.put(_ProcessWorkerEvent(kind="ready", worker_id=worker_id, process_id=os.getpid()))
    state: _ProcessLocalTaskState | None = None
    while True:
        if state is None:
            command: _ProcessWorkerCommand = command_queue.get()
            if command.kind == "shutdown":
                result_queue.put(_ProcessWorkerEvent(kind="shutdown", worker_id=worker_id, process_id=os.getpid()))
                return
            if command.kind == "cancel":
                result_queue.put(
                    _ProcessWorkerEvent(
                        kind="cancelled",
                        worker_id=worker_id,
                        epoch=command.epoch,
                        task_id=command.task_id,
                        generation=command.generation,
                        process_id=os.getpid(),
                    )
                )
                continue
            if command.kind == "split":
                result_queue.put(
                    _ProcessWorkerEvent(
                        kind="split_rejected",
                        worker_id=worker_id,
                        epoch=command.epoch,
                        task_id=command.task_id,
                        generation=command.generation,
                        process_id=os.getpid(),
                    )
                )
                continue
            if command.kind != "start" or command.task is None:
                raise RuntimeError("idle pricing worker received an invalid command")
            task = command.task
            state = _ProcessLocalTaskState(
                task=task,
                open_labels=task.initial_open_labels,
                best_path=None,
                best_reduced_cost=float("inf"),
                started_at=time.time(),
                initialized=task.initial_open_labels is not None,
            )
            result_queue.put(
                _ProcessWorkerEvent(
                    kind="started",
                    worker_id=worker_id,
                    epoch=task.epoch,
                    task_id=task.task_id,
                    generation=task.generation,
                    process_id=os.getpid(),
                )
            )

        task = state.task
        cpu_start = time.process_time()
        try:
            outcome = _price_route_forward_only(
                graph=graph,
                objective=objective,
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
                source_neighbor_block=tuple(sorted({prefix[0] for prefix in task.source_prefixes if prefix})),
                worker_id=worker_id,
                known_signature_costs_snapshot=task.known_signature_costs,
                source_prefixes=tuple() if state.initialized else task.source_prefixes,
                initial_open_labels=state.open_labels if state.initialized else None,
                extension_budget=scheduler.checkpoint_extension_period,
                prior_best_path=state.best_path,
                prior_best_reduced_cost=state.best_reduced_cost,
            )
        except PricingTimeLimitReached as exc:
            result = PricingResult(tuple(), tuple(), None, exc.diagnostics.best_reduced_cost, exc.diagnostics)
            worker_result = _worker_pricing_result(task, result, worker_id, time.process_time() - cpu_start)
            result_queue.put(
                _ProcessWorkerEvent(
                    kind="time_limit",
                    worker_id=worker_id,
                    epoch=task.epoch,
                    task_id=task.task_id,
                    generation=task.generation,
                    result=worker_result,
                    task_elapsed_seconds=time.time() - state.started_at,
                    process_id=os.getpid(),
                )
            )
            state = None
            continue

        cpu_seconds = time.process_time() - cpu_start
        if isinstance(outcome, (_ForwardSearchCheckpoint, _ForwardCandidateCheckpoint)):
            state.open_labels = outcome.open_labels
            state.best_path = outcome.best_path
            state.best_reduced_cost = outcome.best_reduced_cost
            state.initialized = True
            state.recent_extensions = outcome.result.diagnostics.extensions_attempted
            state.recent_dominance_tests = outcome.result.diagnostics.forward_dominance_tests
            if isinstance(outcome, _ForwardCandidateCheckpoint):
                known_signature_costs = dict(state.task.known_signature_costs)
                signature_cache = RouteSignatureCache()
                active_sr = tuple(sorted(task.duals.nu))
                for route in outcome.result.routes:
                    signature = route_signature(
                        route,
                        graph,
                        task.residual_customers,
                        signature_cache,
                        active_sr,
                        len(active_sr),
                    )
                    coefficient_signature = route_coefficient_signature(signature)
                    known_signature_costs[coefficient_signature] = min(
                        known_signature_costs.get(coefficient_signature, float("inf")),
                        route.cost,
                    )
                state.task = replace(state.task, known_signature_costs=known_signature_costs)
            worker_result = _worker_pricing_result(task, outcome.result, worker_id, cpu_seconds)
            closure_gap = _task_closure_gap(outcome.completion_lower_bound)
            remaining_work = len(outcome.open_labels) + state.recent_extensions + state.recent_dominance_tests
            result_queue.put(
                _ProcessWorkerEvent(
                    kind=(
                        "candidate_checkpoint"
                        if isinstance(outcome, _ForwardCandidateCheckpoint)
                        else "checkpoint"
                    ),
                    worker_id=worker_id,
                    epoch=task.epoch,
                    task_id=task.task_id,
                    generation=task.generation,
                    result=worker_result,
                    open_label_count=len(outcome.open_labels),
                    completion_lower_bound=outcome.completion_lower_bound,
                    closure_gap=closure_gap,
                    remaining_work=remaining_work,
                    task_elapsed_seconds=time.time() - state.started_at,
                    process_id=os.getpid(),
                )
            )
            commands: list[_ProcessWorkerCommand] = []
            if isinstance(outcome, _ForwardCandidateCheckpoint):
                while True:
                    command = command_queue.get()
                    if command.kind == "split":
                        result_queue.put(
                            _ProcessWorkerEvent(
                                kind="split_rejected",
                                worker_id=worker_id,
                                epoch=task.epoch,
                                task_id=task.task_id,
                                generation=task.generation,
                                process_id=os.getpid(),
                            )
                        )
                        continue
                    if command.kind not in {"resume", "cancel"}:
                        raise RuntimeError("candidate-paused pricing worker received an invalid command")
                    commands.append(command)
                    break
            else:
                while True:
                    try:
                        commands.append(command_queue.get_nowait())
                    except Empty:
                        break
            cancel = next((item for item in commands if item.kind == "cancel"), None)
            if cancel is not None:
                result_queue.put(
                    _ProcessWorkerEvent(
                        kind="cancelled",
                        worker_id=worker_id,
                        epoch=task.epoch,
                        task_id=task.task_id,
                        generation=task.generation,
                        process_id=os.getpid(),
                    )
                )
                state = None
                continue
            if any(item.kind == "resume" for item in commands):
                continue
            split = next((item for item in commands if item.kind == "split"), None)
            if split is not None:
                retained, children = _split_open_label_frontier(
                    state.open_labels,
                    task_root_prefix=task.task_root_prefix,
                    refinement_depth=scheduler.refinement_depth,
                    child_limit=split.child_limit,
                )
                if not children:
                    result_queue.put(
                        _ProcessWorkerEvent(
                            kind="split_rejected",
                            worker_id=worker_id,
                            epoch=task.epoch,
                            task_id=task.task_id,
                            generation=task.generation,
                            process_id=os.getpid(),
                        )
                    )
                else:
                    state.open_labels = retained
                    result_queue.put(
                        _ProcessWorkerEvent(
                            kind="split_offer",
                            worker_id=worker_id,
                            epoch=task.epoch,
                            task_id=task.task_id,
                            generation=task.generation,
                            child_groups=children,
                            retained_label_count=len(retained),
                            process_id=os.getpid(),
                        )
                    )
            continue

        worker_result = _worker_pricing_result(task, outcome, worker_id, cpu_seconds)
        event_kind = "candidate" if outcome.routes else "closed"
        result_queue.put(
            _ProcessWorkerEvent(
                kind=event_kind,
                worker_id=worker_id,
                epoch=task.epoch,
                task_id=task.task_id,
                generation=task.generation,
                result=worker_result,
                task_elapsed_seconds=time.time() - state.started_at,
                process_id=os.getpid(),
            )
        )
        state = None

class SourceNeighborPricingPool:
    def __init__(
        self,
        graph: TransformedGraph,
        objective: ObjectiveData,
        parallel_workers: int,
        scheduler: PricingSchedulerConfig | None = None,
    ) -> None:
        if parallel_workers <= 0:
            raise ValueError("parallel_workers must be positive")
        self.graph = graph
        self.objective = objective
        self.parallel_workers = parallel_workers
        self.scheduler = scheduler or PricingSchedulerConfig()
        self.call_counter = count(1)
        self.startup_count = 0
        self.reused_calls = 0
        self.shutdown_time_seconds = 0.0
        self.shutdown_count = 0
        startup = time.time()
        context = mp.get_context("spawn")
        self._result_queue = context.Queue()
        self._command_queues = [context.Queue() for _ in range(self.parallel_workers)]
        self._processes = [
            context.Process(
                target=_run_balanced_process_worker,
                args=(worker_id, self._command_queues[worker_id], self._result_queue, graph, objective, self.scheduler),
                name=f"thvrpd-pricing-{worker_id}",
            )
            for worker_id in range(self.parallel_workers)
        ]
        for process in self._processes:
            process.start()
        ready_workers: set[int] = set()
        while len(ready_workers) < self.parallel_workers:
            try:
                event: _ProcessWorkerEvent = self._result_queue.get(timeout=0.5)
            except Empty:
                failed = [process for process in self._processes if process.exitcode not in {None, 0}]
                if failed:
                    raise RuntimeError(f"pricing worker process failed: {[process.exitcode for process in failed]}")
                continue
            if event.kind != "ready":
                raise RuntimeError("pricing process emitted work before readiness")
            ready_workers.add(event.worker_id)
        self.startup_time_seconds = time.time() - startup
        self.startup_count = 1
        self._closed = False

    def shutdown(self) -> None:
        if self._closed:
            return
        start = time.time()
        for command_queue in self._command_queues:
            command_queue.put(_ProcessWorkerCommand(kind="shutdown"))
        shutdown_workers: set[int] = set()
        while len(shutdown_workers) < self.parallel_workers:
            event: _ProcessWorkerEvent = self._result_queue.get()
            if event.kind == "shutdown":
                shutdown_workers.add(event.worker_id)
        for process in self._processes:
            process.join()
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
        epoch_context: PricingEpochContext | None = None,
    ) -> PricingResult:
        if self._closed:
            raise RuntimeError("persistent pricing process pool is closed")
        call_id = next(self.call_counter)
        dual_id = call_id
        pricing_epoch = _pricing_epoch(
            self.objective,
            residual_customers,
            restrictions,
            duals,
            existing_routes,
            existing_column_paths,
            epoch_context,
        )
        self.reused_calls += 1
        known_signature_costs = _known_signature_costs(
            existing_routes,
            existing_column_paths,
            self.graph,
            residual_customers,
            RouteSignatureCache(),
        )
        task_plan = _balanced_source_neighbor_plan(
            self.graph,
            self.objective,
            residual_customers,
            restrictions,
            duals,
            self.parallel_workers,
            self.scheduler,
        )
        worker_batch_size = (
            1
            if stop_at_first_negative
            else max(1, ceil(batch_size / self.parallel_workers))
        )
        tasks: dict[int, _WorkerPricingTask] = {}
        for worker_id, block in enumerate(task_plan.blocks):
            if not block:
                continue
            tasks[worker_id] = _WorkerPricingTask(
                call_id=call_id,
                dual_id=dual_id,
                epoch=pricing_epoch,
                worker_id=worker_id,
                task_id=worker_id,
                generation=0,
                source_prefixes=tuple((neighbor,) for neighbor in block),
                task_root_prefix=(block[0],) if len(block) == 1 else tuple(),
                initial_open_labels=None,
                residual_customers=residual_customers,
                restrictions=restrictions,
                duals=duals,
                next_route_id=next_route_id,
                farkas=farkas,
                pricing_tolerance=pricing_tolerance,
                use_standard_acceleration=use_standard_acceleration,
                stop_at_first_negative=stop_at_first_negative,
                batch_size=worker_batch_size,
                deadline=deadline,
                known_signature_costs=known_signature_costs,
                pricing_mode=pricing_mode,
            )
        call_start = time.time()
        master_cpu_start = time.process_time()
        leaf_status = {worker_id: ("running" if worker_id in tasks else "closed") for worker_id in range(self.parallel_workers)}
        worker_task = {worker_id: (worker_id if worker_id in tasks else None) for worker_id in range(self.parallel_workers)}
        idle_workers = {worker_id for worker_id in range(self.parallel_workers) if worker_id not in tasks}
        task_by_id = dict(tasks)
        next_task_id = self.parallel_workers
        all_results: list[_WorkerPricingResult] = []
        summaries: dict[int, _ProcessWorkerEvent] = {}
        pending_split_donors: set[int] = set()
        pending_children: set[int] = set()
        worker_busy = {worker_id: 0.0 for worker_id in range(self.parallel_workers)}
        worker_idle_started = {worker_id: call_start for worker_id in idle_workers}
        worker_idle = {worker_id: 0.0 for worker_id in range(self.parallel_workers)}
        worker_task_counts = {worker_id: 0 for worker_id in range(self.parallel_workers)}
        idle_work_requests = 0
        split_candidates = 0
        splits_performed = 0
        rejected_near = 0
        rejected_small = 0
        rejected_elapsed = 0
        rejected_work = 0
        labels_transferred = 0
        bytes_transferred = 0
        split_control_seconds = 0.0
        stale_results = 0
        pending_peak = 0
        first_hit_worker: int | None = None
        force_time_limit = False
        batch_target_reached = False
        cancel_waiting: set[int] = set()
        verified_candidate_routes: dict[tuple[str, ...], Route] = {}
        verification_routes = dict(existing_routes or {})
        verification_paths = set(existing_column_paths or set())
        candidate_verification_seconds = 0.0
        candidate_checkpoints = 0
        candidate_worker_resumptions = 0
        global_batch_limit_cancellations = 0

        submission_start = time.time()
        for worker_id, task in tasks.items():
            self._command_queues[worker_id].put(_ProcessWorkerCommand(kind="start", task=task))
            worker_task_counts[worker_id] += 1
        submission_time = time.time() - submission_start

        while True:
            if batch_target_reached:
                break
            if (
                all(status == "closed" for status in leaf_status.values())
                and not pending_children
                and not pending_split_donors
            ):
                break
            if deadline is not None and time.time() >= deadline:
                force_time_limit = True
                for worker_id, task_id in worker_task.items():
                    if task_id is not None:
                        task = task_by_id[task_id]
                        self._command_queues[worker_id].put(
                            _ProcessWorkerCommand(
                                kind="cancel",
                                epoch=pricing_epoch,
                                task_id=task.task_id,
                                generation=task.generation,
                            )
                        )
                        cancel_waiting.add(worker_id)
                break
            event: _ProcessWorkerEvent = self._result_queue.get()
            if event.epoch is not None and event.epoch != pricing_epoch:
                stale_results += 1
                continue
            if event.kind == "started":
                if event.worker_id in worker_idle_started:
                    worker_idle[event.worker_id] += time.time() - worker_idle_started.pop(event.worker_id)
                if event.task_id in pending_children:
                    pending_children.remove(event.task_id)
                    leaf_status[event.task_id] = "running"
                continue
            if event.result is not None:
                all_results.append(event.result)
                worker_busy[event.worker_id] += event.result.diagnostics.elapsed_seconds
            if event.kind == "checkpoint":
                if event.task_id is None:
                    raise RuntimeError("checkpoint missing task id")
                summaries[event.task_id] = event
                available = sorted(idle_workers)
                if available and not pending_split_donors:
                    idle_work_requests += len(available)
                    eligible: list[tuple[int, _ProcessWorkerEvent]] = []
                    for task_id, summary in summaries.items():
                        if leaf_status.get(task_id) != "running":
                            continue
                        if summary.open_label_count < self.scheduler.split_open_labels_min:
                            rejected_small += 1
                            continue
                        if summary.task_elapsed_seconds < self.scheduler.split_elapsed_min:
                            rejected_elapsed += 1
                            continue
                        if summary.remaining_work < self.scheduler.split_work_min:
                            rejected_work += 1
                            continue
                        if (
                            summary.closure_gap is None
                            or not _closure_gap_exceeds_split_threshold(
                                summary.closure_gap,
                                pricing_tolerance,
                                self.scheduler.split_gap_factor,
                            )
                        ):
                            rejected_near += 1
                            continue
                        eligible.append((task_id, summary))
                    if eligible:
                        split_candidates += len(eligible)
                        donor_id, donor_summary = max(
                            eligible,
                            key=lambda item: (
                                item[1].remaining_work,
                                float("inf") if item[1].closure_gap is None else item[1].closure_gap,
                                item[1].open_label_count,
                                -item[0],
                            ),
                        )
                        donor_worker = next(worker for worker, owned in worker_task.items() if owned == donor_id)
                        donor = task_by_id[donor_id]
                        self._command_queues[donor_worker].put(
                            _ProcessWorkerCommand(
                                kind="split",
                                epoch=pricing_epoch,
                                task_id=donor_id,
                                generation=donor.generation,
                                child_limit=len(available),
                            )
                        )
                        pending_split_donors.add(donor_id)
                continue
            if event.kind == "split_rejected":
                if event.task_id is not None:
                    pending_split_donors.discard(event.task_id)
                continue
            if event.kind == "split_offer":
                if event.task_id is None:
                    raise RuntimeError("split offer missing task id")
                control_start = time.time()
                pending_split_donors.discard(event.task_id)
                parent = task_by_id[event.task_id]
                available = sorted(idle_workers)
                if len(event.child_groups) > len(available):
                    raise RuntimeError("split offer exceeds reserved idle-worker capacity")
                for (root_prefix, labels), child_worker in zip(event.child_groups, available):
                    child_id = next_task_id
                    next_task_id += 1
                    child = replace(
                        parent,
                        worker_id=child_worker,
                        task_id=child_id,
                        generation=parent.generation + 1,
                        source_prefixes=tuple(),
                        task_root_prefix=root_prefix,
                        initial_open_labels=labels,
                    )
                    task_by_id[child_id] = child
                    leaf_status[child_id] = "pending"
                    pending_children.add(child_id)
                    worker_task[child_worker] = child_id
                    idle_workers.remove(child_worker)
                    self._command_queues[child_worker].put(_ProcessWorkerCommand(kind="start", task=child))
                    worker_task_counts[child_worker] += 1
                    labels_transferred += len(labels)
                    bytes_transferred += len(pickle.dumps(labels, protocol=pickle.HIGHEST_PROTOCOL))
                splits_performed += 1
                pending_peak = max(pending_peak, len(pending_children))
                split_control_seconds += time.time() - control_start
                continue
            if event.kind in {"closed", "time_limit"}:
                if event.task_id is None:
                    raise RuntimeError("terminal worker event missing task id")
                leaf_status[event.task_id] = "closed" if event.kind == "closed" else "unresolved"
                worker_task[event.worker_id] = None
                idle_workers.add(event.worker_id)
                worker_idle_started[event.worker_id] = time.time()
                if event.kind == "time_limit":
                    force_time_limit = True
                    break
                continue
            if event.kind in {"candidate", "candidate_checkpoint"}:
                if event.result is None:
                    raise RuntimeError("candidate event missing pricing result")
                if event.kind == "candidate_checkpoint":
                    candidate_checkpoints += 1
                verification_start = time.time()
                candidate_check = _merge_worker_results(
                    [event.result], self.graph, self.objective, residual_customers, restrictions, duals,
                    next_route_id + len(verified_candidate_routes), farkas, pricing_tolerance,
                    verification_routes, verification_paths,
                    pricing_mode, "process", len(task_plan.source_neighbors),
                    tuple(len(block) for block in task_plan.blocks), self.parallel_workers,
                    force_time_limit=False, call_id=call_id, dual_id=dual_id,
                    expected_epoch=pricing_epoch, submission_time_seconds=0.0,
                    pool_startup_time_seconds=0.0, pool_startup_count=0, pool_reused_calls=0,
                    batch_target=max(1, batch_size - len(verified_candidate_routes)), prefix_task_depth=1,
                )
                candidate_verification_seconds += time.time() - verification_start
                for route in candidate_check.routes:
                    verified_candidate_routes[route.path] = route
                    verification_routes[route.path] = route
                    verification_paths.add(route.path)
                if candidate_check.routes and first_hit_worker is None:
                    first_hit_worker = event.worker_id
                if len(verified_candidate_routes) >= batch_size:
                    batch_target_reached = True
                    global_batch_limit_cancellations += 1
                    for worker_id, task_id in worker_task.items():
                        if task_id is None:
                            continue
                        task = task_by_id[task_id]
                        self._command_queues[worker_id].put(
                            _ProcessWorkerCommand(
                                kind="cancel", epoch=pricing_epoch,
                                task_id=task.task_id, generation=task.generation,
                            )
                        )
                        cancel_waiting.add(worker_id)
                    break
                if event.kind == "candidate_checkpoint":
                    self._command_queues[event.worker_id].put(
                        _ProcessWorkerCommand(
                            kind="resume",
                            epoch=pricing_epoch,
                            task_id=event.task_id,
                            generation=event.generation,
                        )
                    )
                    candidate_worker_resumptions += 1
                else:
                    if event.task_id is None:
                        raise RuntimeError("terminal candidate event missing task id")
                    leaf_status[event.task_id] = "closed"
                    worker_task[event.worker_id] = None
                    idle_workers.add(event.worker_id)
                    worker_idle_started[event.worker_id] = time.time()
                continue
            if event.kind == "cancelled":
                worker_task[event.worker_id] = None
                idle_workers.add(event.worker_id)
                worker_idle_started[event.worker_id] = time.time()
                continue

        while cancel_waiting:
            try:
                event = self._result_queue.get(timeout=0.5)
            except Empty:
                failed = [process for process in self._processes if process.exitcode not in {None, 0}]
                if failed:
                    raise RuntimeError(f"pricing worker process failed during cancellation: {[process.exitcode for process in failed]}")
                continue
            if event.result is not None:
                all_results.append(event.result)
                worker_busy[event.worker_id] += event.result.diagnostics.elapsed_seconds
            if event.kind == "cancelled":
                cancel_waiting.discard(event.worker_id)
                worker_task[event.worker_id] = None
                idle_workers.add(event.worker_id)
                worker_idle_started[event.worker_id] = time.time()

        wall_elapsed = time.time() - call_start
        for worker_id, idle_start in worker_idle_started.items():
            worker_idle[worker_id] += time.time() - idle_start
        all_leaf_closed = (
            all(status == "closed" for status in leaf_status.values())
            and not pending_children
            and not pending_split_donors
        )
        merged = _merge_worker_results(
            all_results,
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
            "process",
            len(task_plan.source_neighbors),
            tuple(len(block) for block in task_plan.blocks),
            self.parallel_workers,
            force_time_limit=force_time_limit,
            call_id=call_id,
            dual_id=dual_id,
            expected_epoch=pricing_epoch,
            submission_time_seconds=submission_time,
            pool_startup_time_seconds=self.startup_time_seconds if call_id == 1 else 0.0,
            pool_startup_count=1 if call_id == 1 else 0,
            pool_reused_calls=1,
            batch_target=batch_size,
            prefix_task_depth=1,
            all_leaf_tasks_closed=all_leaf_closed,
            leaf_task_count=len(leaf_status),
            wall_elapsed_seconds=wall_elapsed,
            dynamic_diagnostics={
                "balanced_process_dynamic": True,
                "initial_block_scores": task_plan.block_scores,
                "initial_load_imbalance_max_mean": (
                    max(task_plan.block_scores) / (sum(task_plan.block_scores) / len(task_plan.block_scores))
                    if task_plan.block_scores and sum(task_plan.block_scores) > 0.0 else 0.0
                ),
                "per_worker_busy_seconds": tuple(sorted(worker_busy.items())),
                "per_worker_idle_seconds": tuple(sorted(worker_idle.items())),
                "per_worker_task_counts": tuple(sorted(worker_task_counts.items())),
                "idle_work_requests": idle_work_requests,
                "dynamic_split_candidates": split_candidates,
                "dynamic_splits_performed": splits_performed,
                "dynamic_split_rejected_near_closure": rejected_near,
                "dynamic_split_rejected_small_frontier": rejected_small,
                "dynamic_split_rejected_elapsed": rejected_elapsed,
                "dynamic_split_rejected_low_work": rejected_work,
                "dynamic_child_tasks_created": max(0, len(leaf_status) - self.parallel_workers),
                "dynamic_labels_transferred": labels_transferred,
                "dynamic_bytes_transferred": bytes_transferred,
                "dynamic_split_control_seconds": split_control_seconds,
                "leaf_tasks_created": len(leaf_status),
                "leaf_tasks_closed": sum(status == "closed" for status in leaf_status.values()),
                "pending_transfer_peak": pending_peak,
                "stale_worker_results_discarded": stale_results,
                "pricing_epoch_invalidations": 1 if verified_candidate_routes else 0,
                "productive_first_hit_worker": first_hit_worker,
                "candidate_verification_seconds": candidate_verification_seconds,
                "master_control_seconds": time.process_time() - master_cpu_start,
                "candidate_checkpoints": candidate_checkpoints,
                "candidate_worker_resumptions": candidate_worker_resumptions,
                "global_verified_candidates": len(verified_candidate_routes),
                "global_batch_limit_cancellations": global_batch_limit_cancellations,
            },
        )
        if force_time_limit and not merged.routes:
            raise PricingTimeLimitReached(merged.diagnostics)
        return merged

@dataclass(frozen=True)
class _BranchState:
    together: tuple[tuple[bool, bool], ...]
    conditioned_arcs: tuple[tuple[str, tuple[str, str], bool, bool, bool], ...]

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
    parallel_workers: int = 1,
    existing_routes: dict[tuple[str, ...], Route] | None = None,
    existing_column_paths: set[tuple[str, ...]] | None = None,
    pricing_mode: str = "productive",
    pricing_worker_backend: str = "process",
    pricing_process_pool: SourceNeighborPricingPool | None = None,
    prefix_task_depth: int = 1,
    scheduler_config: PricingSchedulerConfig | None = None,
    epoch_context: PricingEpochContext | None = None,
) -> PricingResult:
    if parallel_workers <= 0:
        raise ValueError("parallel_workers must be positive")
    if pricing_worker_backend not in {"thread", "process"}:
        raise ValueError("pricing_worker_backend must be 'thread' or 'process'")
    if pricing_mode not in {"productive", "closure"}:
        raise ValueError("pricing_mode must be 'productive' or 'closure'")
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
            epoch_context=epoch_context,
        )
    if pricing_worker_backend == "process" and parallel_workers > 1:
        temporary_pool = SourceNeighborPricingPool(
            graph,
            objective,
            parallel_workers,
            scheduler_config,
        )
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
                epoch_context=epoch_context,
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
            pricing_mode=pricing_mode,
            parallel_workers=parallel_workers,
            pricing_worker_backend=pricing_worker_backend,
            prefix_task_depth=prefix_task_depth,
            epoch_context=epoch_context,
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
    )
    if isinstance(result, _ForwardSearchCheckpoint):
        raise RuntimeError("serial pricing returned an unexpected checkpoint")
    if pricing_mode == "productive":
        result = replace(
            result,
            diagnostics=replace(
                result.diagnostics,
                exact_completion=False,
                termination_reason="productive_batch_found" if result.routes else "productive_no_columns",
                certification_mode="not_certified_productive",
                pricing_status=(
                    PRICING_STATUS_NEGATIVE_BATCH if result.routes else PRICING_STATUS_EXHAUSTED_NO_NEGATIVE
                ),
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
    pricing_mode: str = "productive",
    parallel_workers: int = 1,
    pricing_worker_backend: str = "thread",
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
        pricing_mode=pricing_mode,
        parallel_workers=parallel_workers,
        pricing_worker_backend=pricing_worker_backend,
        prefix_task_depth=prefix_task_depth,
    )

def _admissible_source_neighbors(graph: TransformedGraph) -> tuple[str, ...]:
    source = graph.instance.depot_source
    sink = graph.instance.depot_sink
    return tuple(sorted(node for node in graph.out_arcs[source] if node != sink))

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
            )
            if next_node == sink:
                if new_label.represented and _complete_allowed(new_label, graph, restrictions):
                    prefixes.append(new_label.path[1:])
                continue
            stack.append(new_label)
    return tuple(sorted(set(prefixes)))

def _build_deterministic_task_plan(
    graph: TransformedGraph,
    objective: ObjectiveData,
    residual_customers: frozenset[str],
    restrictions: BranchRestrictions,
    *,
    prefix_task_depth: int,
) -> _PricingTaskPlan:
    prefixes = _source_neighbor_prefix_tasks(
        graph,
        objective,
        residual_customers,
        restrictions,
        prefix_task_depth,
    )
    if not prefixes:
        return _PricingTaskPlan(prefixes=(tuple(),), source_neighbors=tuple())
    return _PricingTaskPlan(
        prefixes=prefixes,
        source_neighbors=tuple(sorted({prefix[0] for prefix in prefixes})),
    )

def _pricing_source_label(
    graph: TransformedGraph,
    objective: ObjectiveData,
    duals: PricingDuals,
    farkas: bool,
) -> _Label:
    instance = graph.instance
    active_sr = tuple(sorted(duals.nu))
    source_cost = -duals.kappa if farkas else objective.coeffs.cost * instance.truck_cost - duals.kappa
    return _Label(
        path=(instance.depot_source,),
        represented=frozenset(),
        truck_visited=frozenset({instance.depot_source}),
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
        truck_node_mask=_truck_node_mask(frozenset({instance.depot_source}), graph),
    )

def _balanced_source_neighbor_plan(
    graph: TransformedGraph,
    objective: ObjectiveData,
    residual_customers: frozenset[str],
    restrictions: BranchRestrictions,
    duals: PricingDuals,
    worker_count: int,
    scheduler: PricingSchedulerConfig,
) -> _BalancedTaskPlan:
    if worker_count <= 0:
        raise ValueError("worker_count must be positive")
    shortest = _shortest_truck_times(graph)
    source_label = _pricing_source_label(graph, objective, duals, False)
    scored: list[tuple[str, float]] = []
    for neighbor in _admissible_source_neighbors(graph):
        if not _extension_allowed(source_label, neighbor, graph, residual_customers, restrictions):
            continue
        if _extension_rejected_by_service_deadline(source_label, neighbor, graph, objective):
            continue
        first_label = _extend(
            source_label,
            neighbor,
            graph,
            objective,
            duals,
            tuple(sorted(duals.nu)),
            False,
            restrictions,
        )
        location = _physical_location(first_label)
        reachable = tuple(
            customer
            for customer in residual_customers - first_label.represented
            if _customer_reachable_from_location(customer, location, graph, shortest)
        )
        deadline_compatible = tuple(
            customer
            for customer in reachable
            if _customer_deadline_reachable_from_label(first_label, customer, graph, objective, shortest)
        )
        drone_pads = {
            hub
            for hub in graph.instance.hubs
            if isfinite(shortest[(location, hub)])
            and isfinite(shortest[(hub, graph.instance.depot_sink)])
            and any(
                (hub, customer) in graph.instance.drone_arcs
                for customer in residual_customers - first_label.represented
            )
        }
        score = (
            1.0
            + scheduler.customer_weight * len(reachable)
            + scheduler.out_degree_weight * len(graph.out_arcs[neighbor])
            + scheduler.drone_pad_weight * len(drone_pads)
            + scheduler.deadline_weight * len(deadline_compatible)
        )
        scored.append((neighbor, score))
    scored.sort(key=lambda item: (-item[1], item[0]))
    blocks: list[list[str]] = [[] for _ in range(worker_count)]
    loads = [0.0 for _ in range(worker_count)]
    for neighbor, score in scored:
        worker = min(range(worker_count), key=lambda index: (loads[index], index))
        blocks[worker].append(neighbor)
        loads[worker] += score
    return _BalancedTaskPlan(
        source_neighbors=tuple(sorted(neighbor for neighbor, _ in scored)),
        blocks=tuple(tuple(block) for block in blocks),
        block_scores=tuple(loads),
        successor_scores=tuple(sorted(scored)),
    )

def _split_open_label_frontier(
    labels: tuple[_Label, ...],
    *,
    task_root_prefix: tuple[str, ...],
    refinement_depth: int,
    child_limit: int,
) -> tuple[tuple[_Label, ...], tuple[tuple[tuple[str, ...], tuple[_Label, ...]], ...]]:
    if refinement_depth <= 0 or child_limit <= 0:
        raise ValueError("refinement depth and child limit must be positive")
    groups: dict[tuple[str, ...], list[_Label]] = {}
    retained: list[_Label] = []
    root_length = len(task_root_prefix)
    for label in labels:
        encoded_prefix = label.path[1:]
        if task_root_prefix and encoded_prefix[:root_length] != task_root_prefix:
            raise RuntimeError("open label does not belong to its task root prefix")
        split_start = root_length
        if len(encoded_prefix) < split_start + refinement_depth:
            retained.append(label)
            continue
        signature = encoded_prefix[: split_start + refinement_depth]
        groups.setdefault(signature, []).append(label)
    ranked = sorted(groups.items(), key=lambda item: (-len(item[1]), item[0]))
    if len(ranked) < 2:
        return labels, tuple()
    transferred = ranked[: min(child_limit, len(ranked) - 1)]
    transferred_keys = {key for key, _ in transferred}
    for key, group in ranked:
        if key not in transferred_keys:
            retained.extend(group)
    children = tuple((key, tuple(group)) for key, group in transferred)
    return tuple(retained), children

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
        core_subspace_count=len(source_neighbor_block_sizes),
        core_empty_blocks=sum(1 for size in source_neighbor_block_sizes if size == 0),
        min_core_reduced_cost=result.diagnostics.best_reduced_cost,
        root_closed_by_all_cores=result.diagnostics.exact_completion,
        parallel_labeling_used=parallel_workers > 1,
        parallel_workers=parallel_workers,
        source_neighbor_task_count=source_neighbor_task_count,
        source_neighbor_task_sizes=source_neighbor_task_sizes,
    )
    return PricingResult(
        result.routes,
        result.reduced_costs,
        result.best_route,
        result.best_reduced_cost,
        diagnostics,
    )

def _execute_forward_pricing_task(
    task: _WorkerPricingTask,
    graph: TransformedGraph,
    objective: ObjectiveData,
) -> _WorkerPricingResult:
    cpu_start = time.process_time()
    result = _price_route_forward_only(
        graph=graph,
        objective=objective,
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
        source_neighbor_block=tuple(sorted({prefix[0] for prefix in task.source_prefixes if prefix})),
        worker_id=task.worker_id,
        known_signature_costs_snapshot=task.known_signature_costs,
        source_prefixes=task.source_prefixes,
        initial_open_labels=task.initial_open_labels,
    )
    if isinstance(result, _ForwardSearchCheckpoint):
        raise RuntimeError("non-resumable pricing task returned a checkpoint")
    result = _with_runtime_diagnostics(
        result,
        pricing_worker_backend="worker",
        process_cpu_time_seconds=time.process_time() - cpu_start,
        worker_id=task.worker_id,
        source_neighbor_count=len({prefix[0] for prefix in task.source_prefixes if prefix}),
        source_neighbor_block_sizes=(len(task.source_prefixes),),
        parallel_workers=1,
        source_neighbor_task_count=1,
        source_neighbor_task_sizes=(len(task.source_prefixes),),
    )
    return _WorkerPricingResult(
        call_id=task.call_id,
        dual_id=task.dual_id,
        epoch=task.epoch,
        worker_id=task.worker_id,
        task_id=task.task_id,
        generation=task.generation,
        route_paths=tuple(route.path for route in result.routes),
        reduced_costs=result.reduced_costs,
        best_path=None if result.best_route is None else result.best_route.path,
        best_reduced_cost=result.best_reduced_cost,
        diagnostics=result.diagnostics,
    )

def _sum_worker_diagnostic(results: list[_WorkerPricingResult], name: str) -> int | float:
    return sum(getattr(result.diagnostics, name) for result in results)

def _merge_worker_results(
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
    pricing_worker_backend: str,
    source_neighbor_count: int,
    source_neighbor_block_sizes: tuple[int, ...],
    parallel_workers: int,
    *,
    force_time_limit: bool,
    call_id: int,
    dual_id: int,
    expected_epoch: PricingEpoch,
    submission_time_seconds: float,
    pool_startup_time_seconds: float,
    pool_startup_count: int,
    pool_reused_calls: int,
    batch_target: int,
    prefix_task_depth: int,
    all_leaf_tasks_closed: bool | None = None,
    leaf_task_count: int | None = None,
    wall_elapsed_seconds: float | None = None,
    dynamic_diagnostics: dict[str, object] | None = None,
) -> PricingResult:
    for result in results:
        if result.call_id != call_id or result.dual_id != dual_id or result.epoch != expected_epoch:
            raise RuntimeError("pricing worker returned a stale result from a different epoch")

    active_sr = tuple(sorted(duals.nu))
    active_sr_version = len(active_sr)
    known_signature_costs = _known_signature_costs(
        existing_routes,
        existing_column_paths,
        graph,
        residual_customers,
        RouteSignatureCache(),
        active_sr,
        active_sr_version,
    )
    signature_cache = RouteSignatureCache()
    candidates: list[tuple[float, tuple[str, ...]]] = []
    seen_paths: set[tuple[str, ...]] = set()
    for result in results:
        for path in result.route_paths:
            if path in seen_paths:
                continue
            seen_paths.add(path)
            route = route_from_path(next_route_id, path, graph, objective)
            if not restrictions.route_allowed(route):
                raise RuntimeError("pricing worker candidate failed branch validation")
            reduced_cost = route_farkas_reduced_cost(route, duals) if farkas else route_reduced_cost(route, duals)
            if reduced_cost >= -pricing_tolerance:
                continue
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
                continue
            candidates.append((reduced_cost, path))
            coefficient_signature = route_coefficient_signature(signature)
            known_signature_costs[coefficient_signature] = min(
                known_signature_costs.get(coefficient_signature, float("inf")),
                route.cost,
            )

    candidates.sort(key=lambda item: (item[0], item[1]))
    selected = candidates[:batch_target]
    routes = tuple(route_from_path(next_route_id + index, path, graph, objective) for index, (_, path) in enumerate(selected))
    reduced_costs = tuple(cost for cost, _ in selected)

    best_path: tuple[str, ...] | None = None
    best_cost: float | None = None
    for result in results:
        if result.best_path is None:
            continue
        route = route_from_path(next_route_id, result.best_path, graph, objective)
        direct_cost = route_farkas_reduced_cost(route, duals) if farkas else route_reduced_cost(route, duals)
        if best_cost is None or (direct_cost, result.best_path) < (best_cost, best_path or tuple()):
            best_cost = direct_cost
            best_path = result.best_path
    best_route = None if best_path is None else route_from_path(next_route_id, best_path, graph, objective)

    all_tasks_exhausted = (
        all_leaf_tasks_closed
        if all_leaf_tasks_closed is not None
        else bool(results) and all(result.diagnostics.exact_completion for result in results)
    )
    exact_completion = pricing_mode == "closure" and not routes and all_tasks_exhausted and not force_time_limit
    if routes:
        termination_reason = "closure_negative_batch_found" if pricing_mode == "closure" else "productive_batch_found"
        certification_mode = "not_certified_closure_returned_columns" if pricing_mode == "closure" else "not_certified_productive"
    elif force_time_limit:
        termination_reason = "time_limit_no_columns"
        certification_mode = "not_certified_time_limit"
    elif exact_completion:
        termination_reason = "exhausted_no_negative"
        certification_mode = "certified_exhaustive_forward_tasks"
    else:
        termination_reason = "productive_no_columns"
        certification_mode = "not_certified_productive"

    elapsed = (
        wall_elapsed_seconds
        if wall_elapsed_seconds is not None
        else max((result.diagnostics.elapsed_seconds for result in results), default=0.0)
    )
    worker_ids = sorted({result.worker_id for result in results})
    per_worker_elapsed = tuple(
        (worker_id, sum(result.diagnostics.elapsed_seconds for result in results if result.worker_id == worker_id))
        for worker_id in worker_ids
    )
    per_worker_cpu = tuple(
        (worker_id, sum(result.diagnostics.process_cpu_time_seconds for result in results if result.worker_id == worker_id))
        for worker_id in worker_ids
    )
    process_cpu = sum(value for _, value in per_worker_cpu)
    diagnostics = PricingDiagnostics(
        labels_generated=int(_sum_worker_diagnostic(results, "labels_generated")),
        labels_dominated=int(_sum_worker_diagnostic(results, "labels_dominated")),
        labels_pruned=int(_sum_worker_diagnostic(results, "labels_pruned")),
        max_queue_size=max((result.diagnostics.max_queue_size for result in results), default=0),
        complete_routes_generated=int(_sum_worker_diagnostic(results, "complete_routes_generated")),
        returned_routes=len(routes),
        best_reduced_cost=best_cost,
        exact_completion=exact_completion,
        termination_reason=termination_reason,
        certification_mode=certification_mode,
        elapsed_seconds=elapsed,
        pricing_engine="source_neighbor_parallel_forward",
        forward_labels_generated=int(_sum_worker_diagnostic(results, "forward_labels_generated")),
        parallel_labeling_used=parallel_workers > 1,
        parallel_workers=parallel_workers,
        pricing_mode=pricing_mode,
        pricing_status=_pricing_status_from_reason(termination_reason, bool(routes)),
        productive_calls=1 if pricing_mode == "productive" else 0,
        certification_calls=1 if pricing_mode == "closure" else 0,
        negative_routes_verified=len(candidates),
        negative_routes_inserted=len(routes),
        pricing_worker_backend=pricing_worker_backend,
        process_cpu_time_seconds=process_cpu,
        cpu_core_equivalent=process_cpu / elapsed if elapsed > 0.0 else 0.0,
        source_neighbor_count=source_neighbor_count,
        source_neighbor_block_sizes=source_neighbor_block_sizes,
        source_neighbor_task_count=leaf_task_count if leaf_task_count is not None else len(results),
        source_neighbor_task_sizes=tuple(1 for _ in range(leaf_task_count if leaf_task_count is not None else len(results))),
        certification_worker_calls=(leaf_task_count if leaf_task_count is not None else len(results)) if pricing_mode == "closure" else 0,
        productive_worker_calls=(leaf_task_count if leaf_task_count is not None else len(results)) if pricing_mode == "productive" else 0,
        pricing_pool_startup_time_seconds=pool_startup_time_seconds,
        pricing_pool_startup_count=pool_startup_count,
        pricing_pool_reused_calls=pool_reused_calls,
        pricing_task_submission_time_seconds=submission_time_seconds,
        pricing_worker_payload_count=len(results),
        pricing_worker_response_count=len(results),
        pricing_candidate_paths_before_merge=sum(len(result.route_paths) for result in results),
        pricing_candidate_paths_after_merge=len(candidates),
        pricing_decoded_routes_in_main=len(seen_paths),
        pricing_verified_routes_in_main=len(candidates),
        pricing_batch_target=batch_target,
        pricing_returned_batch_size=len(routes),
        prefix_task_depth=prefix_task_depth,
        certification_core_closed_count=sum(1 for result in results if result.diagnostics.exact_completion),
        certification_core_unresolved_count=sum(1 for result in results if not result.diagnostics.exact_completion),
        root_closed_by_all_cores=exact_completion,
        per_worker_elapsed_seconds=per_worker_elapsed,
        per_worker_cpu_time_seconds=per_worker_cpu,
        per_worker_labels_generated=tuple(
            (worker_id, sum(result.diagnostics.labels_generated for result in results if result.worker_id == worker_id))
            for worker_id in worker_ids
        ),
        per_worker_labels_dominated=tuple(
            (worker_id, sum(result.diagnostics.labels_dominated for result in results if result.worker_id == worker_id))
            for worker_id in worker_ids
        ),
        per_worker_labels_pruned=tuple(
            (worker_id, sum(result.diagnostics.labels_pruned for result in results if result.worker_id == worker_id))
            for worker_id in worker_ids
        ),
        per_worker_completed_labels=tuple(
            (worker_id, sum(result.diagnostics.complete_routes_generated for result in results if result.worker_id == worker_id))
            for worker_id in worker_ids
        ),
        per_worker_verified_negative_routes=tuple(
            (worker_id, sum(len(result.route_paths) for result in results if result.worker_id == worker_id))
            for worker_id in worker_ids
        ),
        extensions_attempted=int(_sum_worker_diagnostic(results, "extensions_attempted")),
        extensions_rejected_by_deadline=int(_sum_worker_diagnostic(results, "extensions_rejected_by_deadline")),
        deadline_reachability_removed=int(_sum_worker_diagnostic(results, "deadline_reachability_removed")),
        forward_dominance_tests=int(_sum_worker_diagnostic(results, "forward_dominance_tests")),
        forward_same_node_dominance_tests=int(_sum_worker_diagnostic(results, "forward_same_node_dominance_tests")),
        forward_physical_location_dominance_tests=int(_sum_worker_diagnostic(results, "forward_physical_location_dominance_tests")),
        forward_return_time_credit_checks=int(_sum_worker_diagnostic(results, "forward_return_time_credit_checks")),
        forward_return_time_credit_checks_skipped=int(_sum_worker_diagnostic(results, "forward_return_time_credit_checks_skipped")),
        forward_branch_language_failures=int(_sum_worker_diagnostic(results, "forward_branch_language_failures")),
        forward_branch_interface_failures=int(_sum_worker_diagnostic(results, "forward_branch_interface_failures")),
        labels_dominated_same_node=int(_sum_worker_diagnostic(results, "labels_dominated_same_node")),
        labels_dominated_physical=int(_sum_worker_diagnostic(results, "labels_dominated_physical")),
        **(dynamic_diagnostics or {}),
    )
    additive_fields = (
        "labels_purged",
        "stale_labels_skipped",
        "standard_bound_pruned",
        "farkas_bound_pruned",
        "together_branch_reachability_pruned",
        "forward_labeling_time_seconds",
        "reward_set_size_before_deadline",
        "reward_set_size_after_deadline",
        "deadline_reward_bound_calls",
        "deadline_dominance_prefilter_skips",
        "routes_rejected_by_deadline_in_master",
        "forward_physical_location_dominance_rejections",
        "forward_mask_scalar_prefilter_failures",
        "dom_gate_pairs_seen",
        "dom_gate_mask_failures",
        "dom_gate_scalar_failures",
        "dom_gate_branch_failures",
        "dom_gate_deadline_failures",
        "dominance_bucket_pairs_considered",
        "dominance_bucket_pairs_rejected",
        "dominance_bucket_candidate_pairs",
        "dominance_bucket_scans_avoided",
        "dominance_bucket_queries",
        "dominance_bucket_skipped_by_mask",
        "dominance_bucket_skipped_by_scalar",
        "dominance_bucket_skipped_by_branch",
        "dominance_bucket_skipped_by_deadline",
        "dominance_bucket_skipped_by_return_credit",
        "dom_frontier_queries",
        "dom_frontier_keys_scanned",
        "dom_frontier_keys_skipped_by_mask",
        "dom_frontier_keys_skipped_by_branch",
        "dom_frontier_keys_skipped_by_deadline",
        "dom_frontier_keys_skipped_by_return_credit",
        "frontier_cells_created",
        "frontier_cells_split",
        "mask_trie_subset_queries",
        "mask_trie_superset_queries",
        "mask_trie_returned_items",
        "mask_subset_queries",
        "mask_superset_queries",
        "mask_query_cache_hits",
        "mask_query_cache_misses",
        "cell_splits",
        "cell_pair_products_before_split",
        "cell_pairs_considered",
        "cell_pairs_rejected_by_mask",
        "cell_pairs_rejected_by_envelope",
        "label_pairs_materialized",
        "full_same_node_tests",
        "full_physical_location_tests",
        "labels_deleted_same_node",
        "labels_deleted_physical_location",
        "certification_tasks_exhausted_by_label_search",
        "physdom_cell_pairs_considered",
        "physdom_cell_pairs_rejected_by_mask",
        "physdom_cell_pairs_rejected_by_envelope",
        "physdom_label_pairs_materialized",
        "physdom_full_tests",
        "physdom_deletions",
        "physdom_time",
        "return_credit_incompatible_pairs",
        "dom_pairs_avoided_before_materialization",
        "dom_candidate_pairs_materialized",
        "dom_full_tests_same_node",
        "dom_full_tests_physical_location",
        "dom_labels_deleted_same_node",
        "dom_labels_deleted_physical_location",
        "dom_prefilter_pairs",
        "dom_prefilter_mask_fail",
        "dom_prefilter_branch_fail",
        "dom_prefilter_payload_fail",
        "dom_prefilter_block_fail",
        "dom_prefilter_return_credit_fail",
        "dom_full_tests",
        "dom_full_rejections",
        "physical_location_full_tests",
        "physical_location_rejections",
    )
    aggregate_values = {
        name: _sum_worker_diagnostic(results, name)
        for name in additive_fields
    }
    physdom_deletions = int(aggregate_values["physdom_deletions"])
    aggregate_values["physdom_time_per_deletion"] = (
        float(aggregate_values["physdom_time"]) / physdom_deletions
        if physdom_deletions
        else 0.0
    )
    diagnostics = replace(diagnostics, **aggregate_values)
    return PricingResult(routes, reduced_costs, best_route, best_cost, diagnostics)

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
    pricing_mode: str,
    parallel_workers: int,
    pricing_worker_backend: str,
    prefix_task_depth: int = 1,
    epoch_context: PricingEpochContext | None = None,
) -> PricingResult:
    if pricing_worker_backend not in {"thread", "process"}:
        raise ValueError("pricing_worker_backend must be 'thread' or 'process'")
    if pricing_worker_backend == "process":
        raise RuntimeError("process pricing must use SourceNeighborPricingPool")

    task_plan = _build_deterministic_task_plan(
        graph,
        objective,
        residual_customers,
        restrictions,
        prefix_task_depth=prefix_task_depth,
    )
    call_id = 1
    epoch = _pricing_epoch(
        objective,
        residual_customers,
        restrictions,
        duals,
        existing_routes,
        existing_column_paths,
        epoch_context,
    )
    known_signature_costs = _known_signature_costs(
        existing_routes,
        existing_column_paths,
        graph,
        residual_customers,
        RouteSignatureCache(),
    )
    tasks = [
        _WorkerPricingTask(
            call_id=call_id,
            dual_id=call_id,
            epoch=epoch,
            worker_id=index,
            task_id=index,
            generation=0,
            source_prefixes=(prefix,),
            task_root_prefix=prefix,
            initial_open_labels=None,
            residual_customers=residual_customers,
            restrictions=restrictions,
            duals=duals,
            next_route_id=next_route_id,
            farkas=farkas,
            pricing_tolerance=pricing_tolerance,
            use_standard_acceleration=use_standard_acceleration,
            stop_at_first_negative=stop_at_first_negative,
            batch_size=1 if stop_at_first_negative else batch_size,
            deadline=deadline,
            known_signature_costs=known_signature_costs,
            pricing_mode=pricing_mode,
        )
        for index, prefix in enumerate(task_plan.prefixes)
    ]
    submit_start = time.time()
    with ThreadPoolExecutor(max_workers=parallel_workers) as executor:
        futures = [executor.submit(_execute_forward_pricing_task, task, graph, objective) for task in tasks]
        submission_time = time.time() - submit_start
        results = [future.result() for future in futures]
    has_time_limit = any(result.diagnostics.pricing_status == PRICING_STATUS_TIME_LIMIT_NO_COLUMNS for result in results)
    merged = _merge_worker_results(
        results,
        graph,
        objective,
        residual_customers,
        restrictions,
        duals,
        next_route_id,
        farkas,
        pricing_tolerance,
        existing_routes,
        existing_column_paths,
        pricing_mode,
        "thread",
        len(task_plan.source_neighbors),
        tuple(1 for _ in task_plan.prefixes),
        parallel_workers,
        force_time_limit=has_time_limit,
        call_id=call_id,
        dual_id=call_id,
        expected_epoch=epoch,
        submission_time_seconds=submission_time,
        pool_startup_time_seconds=0.0,
        pool_startup_count=0,
        pool_reused_calls=0,
        batch_target=batch_size,
        prefix_task_depth=prefix_task_depth,
    )
    if has_time_limit and not merged.routes:
        raise PricingTimeLimitReached(merged.diagnostics)
    return merged

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
    source_neighbor_block: tuple[str, ...] | None = None,
    worker_id: int | None = None,
    known_signature_costs_snapshot: dict[RouteCoefficientSignature, float] | None = None,
    source_prefixes: tuple[tuple[str, ...], ...] = tuple(),
    max_frontier_cell_size: int = 512,
    max_frontier_pair_product: int = 2000,
    max_frontier_split_depth: int = 6,
    initial_open_labels: tuple[_Label, ...] | None = None,
    extension_budget: int | None = None,
    prior_best_path: tuple[str, ...] | None = None,
    prior_best_reduced_cost: float = float("inf"),
) -> PricingResult | _ForwardSearchCheckpoint | _ForwardCandidateCheckpoint:
    if batch_size <= 0:
        raise ValueError("pricing batch size must be positive")
    _validate_inequality_dual_signs(duals, pricing_tolerance)
    pricing_start = time.time()
    instance = graph.instance
    source = instance.depot_source
    sink = instance.depot_sink
    active_sr = tuple(sorted(duals.nu))
    active_sr_version = len(active_sr)
    shortest = _shortest_truck_times(graph)
    bounds = _build_pricing_bounds(graph, duals, residual_customers, shortest)
    deadline_counters = _DeadlinePricingCounters(
        shortest=shortest,
        max_frontier_cell_size=max_frontier_cell_size,
        max_frontier_pair_product=max_frontier_pair_product,
        max_frontier_split_depth=max_frontier_split_depth,
    )
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
    best_path = prior_best_path
    best_cost = prior_best_reduced_cost
    returned_routes: list[Route] = []
    returned_costs: list[float] = []
    returned_paths: set[tuple[str, ...]] = set()
    labels_generated = 0 if initial_open_labels is not None else 1
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
    dominance_extension_context = _DominanceExtensionContext(
        objective=objective,
        residual_customers=residual_customers,
        restrictions=restrictions,
    )
    effective_batch_size = 1 if stop_at_first_negative else batch_size
    source_neighbor_set = None if source_neighbor_block is None else frozenset(source_neighbor_block)

    def deadline_diag_kwargs() -> dict[str, object]:
        return {
            "extensions_attempted": deadline_counters.extensions_attempted,
            "extensions_rejected_by_deadline": deadline_counters.extensions_rejected_by_deadline,
            "together_branch_reachability_pruned": deadline_counters.together_branch_reachability_pruned,
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
            "forward_branch_interface_failures": deadline_counters.forward_branch_interface_failures,
            "forward_mask_scalar_prefilter_failures": deadline_counters.forward_mask_scalar_prefilter_failures,
            "dom_gate_pairs_seen": deadline_counters.dom_gate_pairs_seen,
            "dom_gate_mask_failures": deadline_counters.dom_gate_mask_failures,
            "dom_gate_scalar_failures": deadline_counters.dom_gate_scalar_failures,
            "dom_gate_branch_failures": deadline_counters.dom_gate_branch_failures,
            "dom_gate_deadline_failures": deadline_counters.dom_gate_deadline_failures,
            "dominance_bucket_pairs_considered": deadline_counters.dominance_bucket_pairs_considered,
            "dominance_bucket_pairs_rejected": deadline_counters.dominance_bucket_pairs_rejected,
            "dominance_bucket_candidate_pairs": deadline_counters.dominance_bucket_candidate_pairs,
            "dominance_bucket_scans_avoided": deadline_counters.dominance_bucket_scans_avoided,
            "dominance_bucket_queries": deadline_counters.dominance_bucket_queries,
            "dominance_bucket_skipped_by_mask": deadline_counters.dominance_bucket_skipped_by_mask,
            "dominance_bucket_skipped_by_scalar": deadline_counters.dominance_bucket_skipped_by_scalar,
            "dominance_bucket_skipped_by_branch": deadline_counters.dominance_bucket_skipped_by_branch,
            "dominance_bucket_skipped_by_deadline": deadline_counters.dominance_bucket_skipped_by_deadline,
            "dominance_bucket_skipped_by_return_credit": deadline_counters.dominance_bucket_skipped_by_return_credit,
            "dom_frontier_queries": deadline_counters.dom_frontier_queries,
            "dom_frontier_keys_scanned": deadline_counters.dom_frontier_keys_scanned,
            "dom_frontier_keys_skipped_by_mask": deadline_counters.dom_frontier_keys_skipped_by_mask,
            "dom_frontier_keys_skipped_by_branch": deadline_counters.dom_frontier_keys_skipped_by_branch,
            "dom_frontier_keys_skipped_by_deadline": deadline_counters.dom_frontier_keys_skipped_by_deadline,
            "dom_frontier_keys_skipped_by_return_credit": deadline_counters.dom_frontier_keys_skipped_by_return_credit,
            "frontier_cells_created": deadline_counters.frontier_cells_created,
            "frontier_cells_split": deadline_counters.frontier_cells_split,
            "mask_trie_subset_queries": deadline_counters.mask_trie_subset_queries,
            "mask_trie_superset_queries": deadline_counters.mask_trie_superset_queries,
            "mask_trie_returned_items": deadline_counters.mask_trie_returned_items,
            "mask_subset_queries": deadline_counters.mask_subset_queries,
            "mask_superset_queries": deadline_counters.mask_superset_queries,
            "mask_query_cache_hits": deadline_counters.mask_query_cache_hits,
            "mask_query_cache_misses": deadline_counters.mask_query_cache_misses,
            "cell_splits": deadline_counters.cell_splits,
            "cell_pair_products_before_split": deadline_counters.cell_pair_products_before_split,
            "cell_pairs_considered": deadline_counters.cell_pairs_considered,
            "cell_pairs_rejected_by_mask": deadline_counters.cell_pairs_rejected_by_mask,
            "cell_pairs_rejected_by_envelope": deadline_counters.cell_pairs_rejected_by_envelope,
            "label_pairs_materialized": deadline_counters.label_pairs_materialized,
            "full_same_node_tests": deadline_counters.full_same_node_tests,
            "full_physical_location_tests": deadline_counters.full_physical_location_tests,
            "labels_deleted_same_node": deadline_counters.labels_deleted_same_node,
            "labels_deleted_physical_location": deadline_counters.labels_deleted_physical_location,
            "certification_tasks_exhausted_by_label_search": deadline_counters.certification_tasks_exhausted_by_label_search,
            "physdom_cell_pairs_considered": deadline_counters.physdom_cell_pairs_considered,
            "physdom_cell_pairs_rejected_by_mask": deadline_counters.physdom_cell_pairs_rejected_by_mask,
            "physdom_cell_pairs_rejected_by_envelope": deadline_counters.physdom_cell_pairs_rejected_by_envelope,
            "physdom_label_pairs_materialized": deadline_counters.physdom_label_pairs_materialized,
            "physdom_full_tests": deadline_counters.physdom_full_tests,
            "physdom_deletions": deadline_counters.physdom_deletions,
            "physdom_time": deadline_counters.physdom_time,
            "physdom_time_per_deletion": (
                deadline_counters.physdom_time / deadline_counters.physdom_deletions
                if deadline_counters.physdom_deletions
                else 0.0
            ),
            "return_credit_incompatible_pairs": deadline_counters.return_credit_incompatible_pairs,
            "dom_pairs_avoided_before_materialization": deadline_counters.dom_pairs_avoided_before_materialization,
            "dom_candidate_pairs_materialized": deadline_counters.dom_candidate_pairs_materialized,
            "dom_full_tests_same_node": deadline_counters.dom_full_tests_same_node,
            "dom_full_tests_physical_location": deadline_counters.dom_full_tests_physical_location,
            "dom_labels_deleted_same_node": deadline_counters.dom_labels_deleted_same_node,
            "dom_labels_deleted_physical_location": deadline_counters.dom_labels_deleted_physical_location,
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

    def accept_complete_label(new_label: _Label) -> None:
        nonlocal best_route, best_path, best_cost, complete_routes_generated
        if not new_label.represented or not _complete_allowed(new_label, graph, restrictions):
            return None
        try:
            route = route_from_path(next_route_id, new_label.path, graph, objective)
        except ServiceEnvelopeViolation:
            deadline_counters.routes_rejected_by_deadline_in_master += 1
            return None
        if not restrictions.route_allowed(route):
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

    def live_open_state() -> tuple[tuple[_Label, ...], float] | None:
        kept_labels = kept_farkas_labels if farkas else kept_standard_labels
        live_by_location = {
            location: frozenset(labels)
            for location, labels in kept_labels.items()
        }
        live_open_labels: list[_Label] = []
        live_keys: list[float] = []
        seen: set[_Label] = set()
        for open_key, _, open_label in queue:
            if open_label in seen:
                continue
            if open_label not in live_by_location.get(_physical_location(open_label), frozenset()):
                continue
            seen.add(open_label)
            live_open_labels.append(open_label)
            live_keys.append(open_key)
        if not live_open_labels:
            return None
        return tuple(live_open_labels), min(live_keys)

    def insert_open_label(new_label: _Label) -> bool:
        nonlocal labels_dominated, labels_purged
        if farkas:
            accepted, rejected_count, purged_count = _insert_nondominated_farkas_label(
                kept_farkas_labels,
                new_label,
                graph,
                dominance_extension_context,
            )
        else:
            accepted, rejected_count, purged_count = _insert_nondominated_standard_label(
                kept_standard_labels,
                new_label,
                graph,
                duals,
                dominance_extension_context,
                deadline_counters,
                shortest if pricing_mode == "closure" else None,
                bounds if pricing_mode == "closure" else None,
            )
        labels_dominated += rejected_count + purged_count
        labels_purged += purged_count
        return accepted

    def prune_unreachable_together_partner(label: _Label) -> bool:
        nonlocal labels_pruned
        if not _together_branch_partner_unreachable(
            label,
            graph,
            objective,
            shortest,
            residual_customers,
            restrictions,
        ):
            return False
        labels_pruned += 1
        deadline_counters.together_branch_reachability_pruned += 1
        return True

    source_prefix_tuple = tuple(source_prefixes or tuple())
    if initial_open_labels is not None:
        for initial_label in initial_open_labels:
            if initial_label.path[-1] == sink:
                raise RuntimeError("checkpoint cannot contain a completed sink label")
            if prune_unreachable_together_partner(initial_label):
                continue
            if not insert_open_label(initial_label):
                continue
            initial_key = _queue_key(
                initial_label,
                graph,
                objective,
                shortest,
                bounds,
                farkas,
                use_standard_acceleration,
                deadline_counters,
            )
            if (farkas or use_standard_acceleration) and initial_key >= -pricing_tolerance:
                labels_pruned += 1
                if farkas:
                    farkas_bound_pruned += 1
                else:
                    standard_bound_pruned += 1
                continue
            heappush(queue, (initial_key, next(counter), initial_label))
        max_queue_size = len(queue)
    elif source_prefix_tuple:
        for prefix in source_prefix_tuple:
            if not prefix:
                raise RuntimeError("empty source prefix task")
            prefix_label = source_label
            prefix_pruned = False
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
                )
                labels_generated += 1
                if next_node != sink and prune_unreachable_together_partner(prefix_label):
                    prefix_pruned = True
                    break
            if prefix_pruned:
                continue
            if prefix_label.path[-1] == sink:
                accept_complete_label(prefix_label)
                continue
            if not insert_open_label(prefix_label):
                continue
            prefix_key = _queue_key(prefix_label, graph, objective, shortest, bounds, farkas, use_standard_acceleration, deadline_counters)
            if (farkas or use_standard_acceleration) and prefix_key >= -pricing_tolerance:
                labels_pruned += 1
                if farkas:
                    farkas_bound_pruned += 1
                else:
                    standard_bound_pruned += 1
                continue
            heappush(queue, (prefix_key, next(counter), prefix_label))
        max_queue_size = max(max_queue_size, len(queue))
    else:
        source_key = _queue_key(source_label, graph, objective, shortest, bounds, farkas, use_standard_acceleration, deadline_counters)
        heappush(queue, (source_key, next(counter), source_label))
        max_queue_size = 1

    while queue:
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
                    pricing_status=PRICING_STATUS_TIME_LIMIT_NO_COLUMNS,
                    **deadline_diag_kwargs(),
                )
            )
        key, _, label = heappop(queue)
        if (farkas or use_standard_acceleration) and key >= -pricing_tolerance:
            pruned_open_count = 1 + len(queue)
            labels_pruned += pruned_open_count
            if farkas:
                farkas_bound_pruned += pruned_open_count
            else:
                standard_bound_pruned += pruned_open_count
            queue.clear()
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
            new_label = _extend(label, next_node, graph, objective, duals, active_sr, farkas, restrictions)
            labels_generated += 1
            if next_node == sink:
                accept_complete_label(new_label)
                continue
            if prune_unreachable_together_partner(new_label):
                continue
            if farkas:
                accepted, rejected_count, purged_count = _insert_nondominated_farkas_label(
                    kept_farkas_labels,
                    new_label,
                    graph,
                    dominance_extension_context,
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
                    duals,
                    dominance_extension_context,
                    deadline_counters,
                    shortest if pricing_mode == "closure" else None,
                    bounds if pricing_mode == "closure" else None,
                )
                labels_dominated += rejected_count + purged_count
                labels_purged += purged_count
                if not accepted:
                    continue
            new_key = _queue_key(new_label, graph, objective, shortest, bounds, farkas, use_standard_acceleration, deadline_counters)
            if (farkas or use_standard_acceleration) and new_key >= -pricing_tolerance:
                labels_pruned += 1
                if farkas:
                    farkas_bound_pruned += 1
                else:
                    standard_bound_pruned += 1
                continue
            heappush(queue, (new_key, next(counter), new_label))
            max_queue_size = max(max_queue_size, len(queue))

        if len(returned_routes) >= effective_batch_size and queue and extension_budget is None:
            return _pricing_result(
                returned_routes,
                returned_costs,
                _materialize_best_route(best_route, best_path, next_route_id, graph, objective),
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
                termination_reason=(
                    "closure_negative_batch_found"
                    if pricing_mode == "closure"
                    else "productive_batch_found"
                ),
                certification_mode=(
                    "not_certified_closure_returned_columns"
                    if pricing_mode == "closure"
                    else "not_certified_productive"
                ),
                elapsed_seconds=time.time() - pricing_start,
                pricing_mode=pricing_mode,
                **deadline_diag_kwargs(),
            )

        if len(returned_routes) >= effective_batch_size and queue:
            live_state = live_open_state()
            if live_state is not None:
                live_open_labels, completion_lower_bound = live_state
                checkpoint_result = _pricing_result(
                    returned_routes,
                    returned_costs,
                    _materialize_best_route(best_route, best_path, next_route_id, graph, objective),
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
                    termination_reason="candidate_checkpoint",
                    certification_mode="not_certified_candidate_checkpoint",
                    elapsed_seconds=time.time() - pricing_start,
                    pricing_mode=pricing_mode,
                    **deadline_diag_kwargs(),
                )
                return _ForwardCandidateCheckpoint(
                    open_labels=live_open_labels,
                    completion_lower_bound=completion_lower_bound,
                    best_path=best_path,
                    best_reduced_cost=best_cost,
                    result=checkpoint_result,
                )

        if extension_budget is not None and deadline_counters.extensions_attempted >= extension_budget and queue:
            live_state = live_open_state()
            if live_state is None:
                continue
            live_open_labels, completion_lower_bound = live_state
            checkpoint_result = _pricing_result(
                [],
                [],
                None,
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
                termination_reason="checkpoint",
                certification_mode="not_certified_checkpoint",
                elapsed_seconds=time.time() - pricing_start,
                pricing_mode=pricing_mode,
                **deadline_diag_kwargs(),
            )
            return _ForwardSearchCheckpoint(
                open_labels=tuple(live_open_labels),
                completion_lower_bound=completion_lower_bound,
                best_path=best_path,
                best_reduced_cost=best_cost,
                result=checkpoint_result,
            )

    if pricing_mode == "closure" and not queue:
        deadline_counters.certification_tasks_exhausted_by_label_search += 1

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
        **deadline_diag_kwargs(),
    )

def _dual_solution_key(duals: PricingDuals) -> tuple[object, ...]:
    return (
        tuple(sorted(duals.mu.items())),
        duals.kappa,
        tuple(sorted(duals.nu.items())),
    )

def _branch_restriction_signature(restrictions: BranchRestrictions) -> tuple[object, ...]:
    return (
        tuple(sorted(restrictions.together_pairs)),
        tuple(sorted(restrictions.separate_pairs)),
        tuple(sorted(restrictions.pad_forbidden)),
        tuple(sorted(restrictions.pad_required)),
        tuple(sorted(restrictions.conditioned_arc_forbidden)),
        tuple(sorted(restrictions.conditioned_arc_required)),
    )

def _residual_customer_mask(objective: ObjectiveData, residual_customers: frozenset[str]) -> int:
    customer_index = {customer: index for index, customer in enumerate(sorted(objective.bounds.arrival_lb))}
    mask = 0
    for customer in residual_customers:
        mask |= 1 << customer_index[customer]
    return mask

def _objective_scale_signature(objective: ObjectiveData) -> tuple[object, ...]:
    return tuple(sorted(objective.coeffs.__dict__.items()))

def _service_window_signature(objective: ObjectiveData) -> tuple[object, ...]:
    return (
        tuple(sorted(objective.bounds.arrival_lb.items())),
        tuple(sorted(objective.bounds.service_ub.items())),
    )

def _pricing_epoch(
    objective: ObjectiveData,
    residual_customers: frozenset[str],
    restrictions: BranchRestrictions,
    duals: PricingDuals,
    existing_routes: dict[tuple[str, ...], Route] | None,
    existing_column_paths: set[tuple[str, ...]] | None,
    context: PricingEpochContext | None = None,
) -> PricingEpoch:
    active_paths = tuple(sorted(existing_column_paths or tuple()))
    route_keys = tuple(sorted(existing_routes or {}))
    active_sr_ids = tuple(sorted(duals.nu))
    epoch_context = context or PricingEpochContext(
        active_sr_version=len(active_sr_ids),
        fixed_route_signature=tuple(),
        active_column_version=active_paths,
        rmp_structure_version=route_keys,
    )
    return PricingEpoch(
        dual_signature=_dual_solution_key(duals),
        active_sr_ids=active_sr_ids,
        sr_version=epoch_context.active_sr_version,
        residual_customer_mask=_residual_customer_mask(objective, residual_customers),
        branch_signature=_branch_restriction_signature(restrictions),
        fixed_route_signature=epoch_context.fixed_route_signature,
        active_column_version=epoch_context.active_column_version,
        rmp_structure_version=epoch_context.rmp_structure_version,
        objective_scale_version=_objective_scale_signature(objective),
        service_window_version=_service_window_signature(objective),
    )

def _pricing_status_from_reason(termination_reason: str, has_columns: bool) -> str:
    if termination_reason in {"productive_batch_found", "closure_negative_batch_found"}:
        return PRICING_STATUS_NEGATIVE_BATCH
    if termination_reason in {"exhausted_no_negative", "exact_pricing_complete"}:
        return PRICING_STATUS_EXHAUSTED_NO_NEGATIVE
    if termination_reason == "time_limit_with_columns":
        return PRICING_STATUS_TIME_LIMIT_WITH_COLUMNS
    if termination_reason == "time_limit_unresolved":
        return (
            PRICING_STATUS_TIME_LIMIT_WITH_COLUMNS
            if has_columns
            else PRICING_STATUS_TIME_LIMIT_NO_COLUMNS
        )
    if termination_reason == "time_limit_no_columns":
        return PRICING_STATUS_TIME_LIMIT_NO_COLUMNS
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
    pricing_engine: str = "source_neighbor_parallel_forward",
    pricing_mode: str = "productive",
    **diagnostic_values: object,
) -> PricingResult:
    reason = termination_reason or ("exhausted_no_negative" if exact_completion else "pricing_interrupted")
    certification = certification_mode or (
        "certified_exhaustive_forward_tasks" if exact_completion else "not_certified"
    )
    diagnostics = PricingDiagnostics(
        labels_generated=labels_generated,
        labels_dominated=labels_dominated,
        labels_pruned=labels_pruned,
        max_queue_size=max_queue_size,
        complete_routes_generated=complete_routes_generated,
        returned_routes=len(routes),
        best_reduced_cost=best_cost,
        exact_completion=exact_completion,
        termination_reason=reason,
        certification_mode=certification,
        labels_purged=labels_purged,
        stale_labels_skipped=stale_labels_skipped,
        standard_bound_pruned=standard_bound_pruned,
        farkas_bound_pruned=farkas_bound_pruned,
        elapsed_seconds=elapsed_seconds,
        pricing_engine=pricing_engine,
        forward_labels_generated=labels_generated,
        forward_labeling_time_seconds=elapsed_seconds,
        pricing_mode=pricing_mode,
        pricing_status=_pricing_status_from_reason(reason, bool(routes)),
        productive_calls=1 if pricing_mode == "productive" else 0,
        certification_calls=1 if pricing_mode == "closure" else 0,
        negative_routes_verified=len(routes),
        negative_routes_inserted=len(routes),
        pricing_returned_batch_size=len(routes),
        **diagnostic_values,
    )
    return PricingResult(tuple(routes), tuple(reduced_costs), best_route, best_cost, diagnostics)

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
    if not graph.arc_compatible_with_active_pad(arc, label.active_pad):
        return False
    if next_node in instance.customers + instance.hubs and next_node in label.truck_visited:
        return False
    if is_customer_representation(next_node, instance):
        customer = served_customer(next_node)
        if customer not in residual_customers or customer in label.represented:
            return False
        if label.truck_load + instance.demand[customer] > instance.truck_payload + PAYLOAD_TOLERANCE:
            return False
        drone_pad = None
        if is_duplicate(next_node):
            drone_pad = node if arc in graph.hub_duplicate_arcs else label.active_pad
        required_pads = [hub for hub, c in restrictions.pad_required if c == customer]
        if required_pads and (not is_duplicate(next_node) or drone_pad not in required_pads):
            return False
        if is_duplicate(next_node) and (drone_pad, customer) in restrictions.pad_forbidden:
            return False
        for p, q in restrictions.separate_pairs:
            if customer == p and q in label.represented:
                return False
            if customer == q and p in label.represented:
                return False
    next_customer = served_customer(next_node) if is_customer_representation(next_node, instance) else None
    extended_represented = label.represented | ({next_customer} if next_customer is not None else set())
    for customer, i, j in restrictions.conditioned_arc_forbidden:
        conditioned_arc = (i, j)
        if arc == conditioned_arc and customer in extended_represented:
            return False
        if next_customer == customer and conditioned_arc in label.used_arcs:
            return False
    extended_used_arcs = label.used_arcs | {arc}
    extended_truck_visited = label.truck_visited | (
        {next_node} if next_node in instance.customers + instance.hubs else set()
    )
    next_active_pad = (
        node
        if arc in graph.hub_duplicate_arcs
        else next_node
        if next_node in instance.hubs
        else label.active_pad
    )
    for customer, i, j in restrictions.conditioned_arc_required:
        required_arc = (i, j)
        if customer not in extended_represented or required_arc in extended_used_arcs:
            continue
        if not _conditioned_arc_can_still_occur(
            i,
            j,
            next_node,
            next_active_pad,
            frozenset(extended_represented),
            frozenset(extended_truck_visited),
            graph,
        ):
            return False
    if is_duplicate(next_node):
        customer = duplicate_customer(next_node)
        if label.block_count + 1 > instance.drones_per_truck:
            return False
    return True


def _conditioned_arc_can_still_occur(
    tail: str,
    head: str,
    endpoint: str,
    active_pad: str | None,
    represented: frozenset[str],
    truck_visited: frozenset[str],
    graph: TransformedGraph,
) -> bool:
    required_arc = (tail, head)
    if endpoint == tail:
        return required_arc in graph.arcs and graph.arc_compatible_with_active_pad(required_arc, active_pad)
    if tail in graph.instance.nodes:
        return tail not in truck_visited
    if is_duplicate(tail):
        return duplicate_customer(tail) not in represented
    raise RuntimeError(f"conditioned branch uses unknown transformed-arc tail {tail}")

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
        hub = node
        customer = duplicate_customer(next_node)
        return label.physical_time + instance.drone_time[(hub, customer)]
    if arc in graph.duplicate_duplicate_arcs:
        hub = label.active_pad
        customer = duplicate_customer(next_node)
        return label.active_pad_arrival + instance.drone_time[(hub, customer)]
    if arc in graph.duplicate_regular_arcs and next_node in instance.customers:
        hub = label.active_pad
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
        hub = node if arc in graph.hub_duplicate_arcs else active_pad
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
        hub = active_pad
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
    branch_state = (
        _branch_state_from_resources(represented_frozen, used_arcs, restrictions)
        if restrictions is not None
        else None
    )
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
) -> bool:
    return _completion_resources_allowed(label.represented, label.used_arcs, restrictions)


def _completion_resources_allowed(
    represented: frozenset[str],
    used_arcs: frozenset[tuple[str, str]],
    restrictions: BranchRestrictions,
) -> bool:
    for p, q in restrictions.together_pairs:
        if (p in represented) != (q in represented):
            return False
    for customer, i, j in restrictions.conditioned_arc_forbidden:
        if customer in represented and (i, j) in used_arcs:
            return False
    for customer, i, j in restrictions.conditioned_arc_required:
        if customer in represented and (i, j) not in used_arcs:
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

def _label_branch_state(
    label: _Label,
    restrictions: BranchRestrictions,
) -> _BranchState:
    return label.branch_state or _branch_state(label, restrictions)

def _frontier_time_resource(label: _Label) -> float:
    return label.active_pad_arrival if label.active_pad is not None else label.physical_time

def _frontier_bin(value: float, width: float = 1.0) -> int:
    if not isfinite(value):
        raise RuntimeError("frontier resource bin received a nonfinite value")
    return int(value // width)

def _forward_frontier_key(
    label: _Label,
    graph: TransformedGraph,
    objective: ObjectiveData,
    restrictions: BranchRestrictions,
    deadline_counters: _DeadlinePricingCounters,
) -> _ForwardFrontierKey:
    branch_key = _label_branch_state(label, restrictions)
    represented_mask = label.represented_mask or _customer_mask(label.represented, graph)
    truck_node_mask = label.truck_node_mask or _truck_node_mask(label.truck_visited, graph)
    resource_time = _frontier_time_resource(label)
    return _ForwardFrontierKey(
        dom_key=_ForwardDomKey(
            endpoint=label.path[-1],
            physical_location=_physical_location(label),
            active_pad=label.active_pad,
            block_position=_block_position(label, graph),
            branch_state_key=branch_key,
        ),
        customer_mask=represented_mask,
        truck_node_mask=truck_node_mask,
        deadline_reachable_mask=_cached_deadline_reachable_mask(label, graph, objective, deadline_counters),
        branch_language_key=branch_key,
        active_block_resource_class=label.block_count,
        payload_bin=_frontier_bin(label.truck_load),
        drone_count_bin=label.block_count,
        time_bin=_frontier_bin(resource_time),
        wait_bin=_frontier_bin(label.active_wait),
    )

def _rebuild_forward_frontier_for_location(
    kept_labels: dict[str, list[_Label]],
    location: str,
    graph: TransformedGraph,
    objective: ObjectiveData,
    restrictions: BranchRestrictions,
    deadline_counters: _DeadlinePricingCounters | None,
    shortest: dict[tuple[str, str], float] | None = None,
    bounds: _PricingBounds | None = None,
) -> None:
    if deadline_counters is None:
        return
    frontiers = _ForwardFrontierLocationIndex()
    customer_bit_count = len(graph.instance.customers)
    truck_bit_count = len((graph.instance.depot_source, *graph.instance.hubs, *graph.instance.customers, graph.instance.depot_sink))
    for stored in kept_labels.get(location, []):
        key = _forward_frontier_key(stored, graph, objective, restrictions, deadline_counters)
        frontiers.buckets.setdefault(key, []).append(stored)
        frontiers.cells.setdefault(key, _ForwardFrontierCell()).add(stored)
        branch_masks = frontiers.by_branch_customer_mask.setdefault(key.branch_language_key, {})
        keys_for_mask = branch_masks.setdefault(key.customer_mask, [])
        if key not in keys_for_mask:
            keys_for_mask.append(key)
            frontiers.customer_tries_by_branch.setdefault(
                key.branch_language_key,
                MaskContainmentTrie(customer_bit_count),
            ).insert(key.customer_mask, key)
            frontiers.truck_tries_by_branch.setdefault(
                key.branch_language_key,
                MaskContainmentTrie(truck_bit_count),
            ).insert(key.truck_node_mask, key)
            frontiers.deadline_tries_by_branch.setdefault(
                key.branch_language_key,
                MaskContainmentTrie(customer_bit_count),
            ).insert(key.deadline_reachable_mask, key)
        frontiers.labels_count += 1
    deadline_counters.frontier_index[location] = frontiers
    _refresh_frontier_cell_summary(deadline_counters)

def _refresh_frontier_cell_summary(deadline_counters: _DeadlinePricingCounters) -> None:
    cells = [
        cell
        for frontier in deadline_counters.frontier_index.values()
        for cell in frontier.cells.values()
    ]
    deadline_counters.frontier_cells_created = max(deadline_counters.frontier_cells_created, len(cells))
    deadline_counters.frontier_cells_split = max(
        deadline_counters.frontier_cells_split,
        sum(1 for cell in cells if len(cell.labels) > deadline_counters.max_frontier_cell_size),
    )

def _frontier_return_credit_gate_reason(
    a_key: _ForwardFrontierKey,
    b_key: _ForwardFrontierKey,
    graph: TransformedGraph,
) -> str | None:
    a_dom = a_key.dom_key
    b_dom = b_key.dom_key
    if a_dom.endpoint == b_dom.endpoint and a_dom.endpoint in graph.instance.nodes:
        return None
    if a_dom.endpoint == b_dom.endpoint and is_duplicate(a_dom.endpoint):
        if a_dom.active_pad == b_dom.active_pad:
            return None
        return "return_credit"
    if (
        a_dom.physical_location not in graph.instance.hubs
        or a_dom.physical_location != b_dom.physical_location
        or a_dom.active_pad != b_dom.active_pad
    ):
        return "return_credit"
    if a_dom.block_position > b_dom.block_position:
        return "return_credit"
    return None

def _frontier_key_gate_reason(
    possible_dominator_key: _ForwardFrontierKey,
    target_key: _ForwardFrontierKey,
    graph: TransformedGraph,
) -> str | None:
    return_credit_reason = _frontier_return_credit_gate_reason(possible_dominator_key, target_key, graph)
    if return_credit_reason is not None:
        return return_credit_reason
    if possible_dominator_key.customer_mask & ~target_key.customer_mask:
        return "mask"
    if possible_dominator_key.truck_node_mask & ~target_key.truck_node_mask:
        return "mask"
    if possible_dominator_key.customer_mask == 0 and possible_dominator_key.customer_mask != target_key.customer_mask:
        return "mask"
    if possible_dominator_key.branch_language_key != target_key.branch_language_key:
        return "branch"
    if target_key.deadline_reachable_mask & ~possible_dominator_key.deadline_reachable_mask:
        return "deadline"
    return None

def _frontier_cell_gate_reason(
    possible_dominator_key: _ForwardFrontierKey,
    possible_dominator_cell: _ForwardFrontierCell,
    target_key: _ForwardFrontierKey,
    target_cell: _ForwardFrontierCell,
    graph: TransformedGraph,
) -> str | None:
    key_reason = _frontier_key_gate_reason(possible_dominator_key, target_key, graph)
    if key_reason is not None:
        return key_reason
    if possible_dominator_cell.min_payload > target_cell.max_payload + PAYLOAD_TOLERANCE:
        return "envelope"
    if possible_dominator_cell.min_active_drone_count > target_cell.max_active_drone_count:
        return "envelope"
    if possible_dominator_cell.min_time > target_cell.max_time + PAPER_DOMINANCE_TOLERANCE:
        return "envelope"
    if possible_dominator_cell.min_wait > target_cell.max_wait + PAPER_DOMINANCE_TOLERANCE:
        return "envelope"
    return None

def _frontier_cell_attribute(label: _Label, dimension: str) -> float:
    if dimension == "payload":
        return label.truck_load
    if dimension == "time":
        return _frontier_time_resource(label)
    if dimension == "wait":
        return label.active_wait
    if dimension == "mask_cardinality":
        return float((label.represented_mask or 0).bit_count())
    raise RuntimeError(f"unknown frontier split dimension: {dimension}")

def _frontier_cell_spread(cell: _ForwardFrontierCell, dimension: str) -> float:
    if dimension == "payload":
        return cell.max_payload - cell.min_payload
    if dimension == "time":
        return cell.max_time - cell.min_time
    if dimension == "wait":
        return cell.max_wait - cell.min_wait
    if dimension == "mask_cardinality":
        counts = [(label.represented_mask or 0).bit_count() for label in cell.labels]
        return float(max(counts, default=0) - min(counts, default=0))
    raise RuntimeError(f"unknown frontier split dimension: {dimension}")

def _split_frontier_cell_once(cell: _ForwardFrontierCell) -> tuple[_ForwardFrontierCell, ...]:
    if len(cell.labels) <= 1:
        return (cell,)
    dimensions = ("payload", "time", "wait", "mask_cardinality")
    dimension = max(dimensions, key=lambda item: _frontier_cell_spread(cell, item))
    if _frontier_cell_spread(cell, dimension) <= 0.0:
        return (cell,)
    pairs = sorted(cell.labels, key=lambda label: (_frontier_cell_attribute(label, dimension), label.path))
    midpoint = len(pairs) // 2
    if midpoint <= 0 or midpoint >= len(pairs):
        return (cell,)
    children = []
    for chunk in (pairs[:midpoint], pairs[midpoint:]):
        child = _ForwardFrontierCell()
        for label in chunk:
            child.add(label)
        children.append(child)
    return tuple(children)

def _refined_frontier_cells(
    cell: _ForwardFrontierCell,
    deadline_counters: _DeadlinePricingCounters,
) -> tuple[_ForwardFrontierCell, ...]:
    max_size = deadline_counters.max_frontier_cell_size
    max_pair_product = deadline_counters.max_frontier_pair_product
    max_depth = deadline_counters.max_frontier_split_depth
    work = [(cell, 0)]
    refined: list[_ForwardFrontierCell] = []
    while work:
        current, depth = work.pop()
        deadline_counters.cell_pair_products_before_split += len(current.labels)
        should_split = (
            depth < max_depth
            and len(current.labels) > 1
            and (len(current.labels) > max_size or len(current.labels) > max_pair_product)
        )
        if not should_split:
            refined.append(current)
            continue
        children = _split_frontier_cell_once(current)
        if len(children) == 1:
            refined.append(current)
            continue
        deadline_counters.cell_splits += 1
        deadline_counters.frontier_cells_split += 1
        work.extend((child, depth + 1) for child in children)
    return tuple(refined)

def _single_label_frontier_cell(label: _Label) -> _ForwardFrontierCell:
    cell = _ForwardFrontierCell()
    cell.add(label)
    return cell

def _frontier_candidate_key_stats(
    index: _ForwardFrontierLocationIndex,
    label_key: _ForwardFrontierKey,
    *,
    label_may_dominate_stored: bool,
    deadline_counters: _DeadlinePricingCounters | None = None,
) -> tuple[list[_ForwardFrontierKey], int, int, int, int]:
    if deadline_counters is not None:
        if label_may_dominate_stored:
            deadline_counters.mask_superset_queries += 1
        else:
            deadline_counters.mask_subset_queries += 1
    branch_masks = index.by_branch_customer_mask.get(label_key.branch_language_key, {})
    branch_key_count = sum(len(keys) for keys in branch_masks.values())
    branch_label_count = sum(
        len(index.buckets[key])
        for keys in branch_masks.values()
        for key in keys
    )
    customer_trie = index.customer_tries_by_branch.get(label_key.branch_language_key)
    truck_trie = index.truck_tries_by_branch.get(label_key.branch_language_key)
    deadline_trie = index.deadline_tries_by_branch.get(label_key.branch_language_key)
    if customer_trie is None or truck_trie is None or deadline_trie is None:
        return [], index.labels_count - branch_label_count, branch_key_count, branch_label_count, 0
    if label_may_dominate_stored:
        customer_candidates = customer_trie.query_supersets(label_key.customer_mask)
        truck_candidates = truck_trie.query_supersets(label_key.truck_node_mask)
        deadline_candidates = deadline_trie.query_subsets(label_key.deadline_reachable_mask)
        if deadline_counters is not None:
            deadline_counters.mask_trie_superset_queries += 2
            deadline_counters.mask_trie_subset_queries += 1
    else:
        customer_candidates = customer_trie.query_subsets(label_key.customer_mask)
        truck_candidates = truck_trie.query_subsets(label_key.truck_node_mask)
        deadline_candidates = deadline_trie.query_supersets(label_key.deadline_reachable_mask)
        if deadline_counters is not None:
            deadline_counters.mask_trie_subset_queries += 2
            deadline_counters.mask_trie_superset_queries += 1
    if deadline_counters is not None:
        deadline_counters.mask_trie_returned_items += (
            len(customer_candidates) + len(truck_candidates) + len(deadline_candidates)
        )
    candidate_key_set = set(customer_candidates) & set(truck_candidates) & set(deadline_candidates)
    candidate_keys = sorted(
        candidate_key_set,
        key=lambda key: (
            key.dom_key.endpoint,
            key.dom_key.physical_location,
            key.customer_mask,
            key.truck_node_mask,
            key.deadline_reachable_mask,
            key.payload_bin,
            key.time_bin,
            key.wait_bin,
        ),
    )
    candidate_key_count = len(candidate_keys)
    candidate_label_count = sum(len(index.buckets[key]) for key in candidate_keys)
    branch_skipped_labels = index.labels_count - branch_label_count
    mask_skipped_keys = branch_key_count - candidate_key_count
    mask_skipped_labels = branch_label_count - candidate_label_count
    return candidate_keys, branch_skipped_labels, mask_skipped_keys, mask_skipped_labels, candidate_key_count

def _dominance_frontier_candidates(
    kept_labels: dict[str, list[_Label]],
    location: str,
    label: _Label,
    graph: TransformedGraph,
    objective: ObjectiveData,
    restrictions: BranchRestrictions,
    deadline_counters: _DeadlinePricingCounters | None,
    *,
    label_may_dominate_stored: bool,
    shortest: dict[tuple[str, str], float] | None = None,
    bounds: _PricingBounds | None = None,
) -> list[_Label]:
    comparable = kept_labels.get(location, [])
    if deadline_counters is None:
        return list(comparable)
    frontiers = deadline_counters.frontier_index.get(location)
    indexed_count = frontiers.labels_count if frontiers is not None else -1
    if frontiers is None or indexed_count != len(comparable):
        _rebuild_forward_frontier_for_location(
            kept_labels,
            location,
            graph,
            objective,
            restrictions,
            deadline_counters,
            shortest,
            bounds,
        )
        frontiers = deadline_counters.frontier_index.get(location, _ForwardFrontierLocationIndex())
    label_key = _forward_frontier_key(label, graph, objective, restrictions, deadline_counters)
    label_cell = _single_label_frontier_cell(label)
    candidate_keys, branch_skipped_labels, mask_skipped_keys, mask_skipped_labels, candidate_key_count = (
        _frontier_candidate_key_stats(
            frontiers,
            label_key,
            label_may_dominate_stored=label_may_dominate_stored,
            deadline_counters=deadline_counters,
        )
    )
    candidates: list[_Label] = []
    reason_counts = {
        "mask": 0,
        "branch": 0,
        "deadline": 0,
        "return_credit": 0,
    }
    avoided = branch_skipped_labels + mask_skipped_labels
    deadline_counters.dom_frontier_queries += 1
    deadline_counters.dom_frontier_keys_scanned += candidate_key_count
    deadline_counters.dom_frontier_keys_skipped_by_branch += frontiers.key_count - sum(
        len(keys) for keys in frontiers.by_branch_customer_mask.get(label_key.branch_language_key, {}).values()
    )
    deadline_counters.dom_frontier_keys_skipped_by_mask += mask_skipped_keys
    for stored_key in candidate_keys:
        stored_cell = frontiers.cells[stored_key]
        for refined_cell in _refined_frontier_cells(stored_cell, deadline_counters):
            stored_labels = refined_cell.labels
            if label_may_dominate_stored:
                reason = _frontier_cell_gate_reason(label_key, label_cell, stored_key, refined_cell, graph)
            else:
                reason = _frontier_cell_gate_reason(stored_key, refined_cell, label_key, label_cell, graph)
            deadline_counters.cell_pairs_considered += 1
            same_regular = (
                stored_key.dom_key.endpoint == label_key.dom_key.endpoint
                and stored_key.dom_key.endpoint in graph.instance.nodes
            )
            if not same_regular:
                deadline_counters.physdom_cell_pairs_considered += 1
            if reason is None:
                candidates.extend(stored_labels)
                if not same_regular:
                    deadline_counters.physdom_label_pairs_materialized += len(stored_labels)
            else:
                if reason == "envelope":
                    deadline_counters.cell_pairs_rejected_by_envelope += 1
                    if not same_regular:
                        deadline_counters.physdom_cell_pairs_rejected_by_envelope += 1
                else:
                    reason_counts[reason] += 1
                    if reason == "mask":
                        deadline_counters.cell_pairs_rejected_by_mask += 1
                    if reason == "return_credit":
                        deadline_counters.return_credit_incompatible_pairs += len(stored_labels)
                    if not same_regular:
                        deadline_counters.physdom_cell_pairs_rejected_by_mask += 1
                avoided += len(stored_labels)
    deadline_counters.dom_frontier_keys_skipped_by_mask += reason_counts["mask"]
    deadline_counters.dom_frontier_keys_skipped_by_branch += reason_counts["branch"]
    deadline_counters.dom_frontier_keys_skipped_by_deadline += reason_counts["deadline"]
    deadline_counters.dom_frontier_keys_skipped_by_return_credit += reason_counts["return_credit"]
    deadline_counters.dom_pairs_avoided_before_materialization += avoided
    deadline_counters.dom_candidate_pairs_materialized += len(candidates)
    deadline_counters.label_pairs_materialized += len(candidates)
    return candidates

def _dominance_pair_gate_reason(
    a: _Label,
    b: _Label,
    graph: TransformedGraph,
    objective: ObjectiveData,
    restrictions: BranchRestrictions,
    deadline_counters: _DeadlinePricingCounters | None,
) -> str | None:
    comparable, _ = _block_comparable_return_credit(a, b, graph)
    if not comparable:
        return "return_credit"
    if not a.represented.issubset(b.represented):
        return "mask"
    if not a.truck_visited.issubset(b.truck_visited):
        return "mask"
    if not a.represented and a.represented != b.represented:
        return "mask"
    if _branch_state(a, restrictions) != _branch_state(b, restrictions):
        return "branch"
    if deadline_counters is not None:
        a_mask = _cached_deadline_reachable_mask(a, graph, objective, deadline_counters)
        b_mask = _cached_deadline_reachable_mask(b, graph, objective, deadline_counters)
        if b_mask & ~a_mask:
            return "deadline"
    if a.truck_load > b.truck_load or a.block_count > b.block_count:
        return "scalar"
    if _same_original_node(a, b, graph):
        if a.physical_time > b.physical_time:
            return "scalar"
    elif a.active_pad_arrival > b.active_pad_arrival or a.active_wait > b.active_wait:
        return "scalar"
    return None

def _dominance_gate_candidates(
    possible_dominators: list[_Label],
    label: _Label,
    graph: TransformedGraph,
    objective: ObjectiveData,
    restrictions: BranchRestrictions,
    deadline_counters: _DeadlinePricingCounters | None,
) -> list[_Label]:
    if deadline_counters is not None:
        deadline_counters.dominance_bucket_queries += 1
        deadline_counters.dominance_bucket_pairs_considered += len(possible_dominators)
    candidates: list[_Label] = []
    rejected = 0
    reason_counts = {
        "mask": 0,
        "scalar": 0,
        "branch": 0,
        "deadline": 0,
        "return_credit": 0,
    }
    for incumbent in possible_dominators:
        reason = _dominance_pair_gate_reason(
            incumbent,
            label,
            graph,
            objective,
            restrictions,
            deadline_counters,
        )
        if reason is None:
            candidates.append(incumbent)
        else:
            rejected += 1
            reason_counts[reason] += 1
    if deadline_counters is not None:
        deadline_counters.dominance_bucket_candidate_pairs += len(candidates)
        deadline_counters.dominance_bucket_pairs_rejected += rejected
        deadline_counters.dominance_bucket_scans_avoided += rejected
        deadline_counters.dominance_bucket_skipped_by_mask += reason_counts["mask"]
        deadline_counters.dominance_bucket_skipped_by_scalar += reason_counts["scalar"]
        deadline_counters.dominance_bucket_skipped_by_branch += reason_counts["branch"]
        deadline_counters.dominance_bucket_skipped_by_deadline += reason_counts["deadline"]
        deadline_counters.dominance_bucket_skipped_by_return_credit += reason_counts["return_credit"]
    return candidates

def _insert_nondominated_standard_label(
    kept_labels: dict[str, list[_Label]],
    label: _Label,
    graph: TransformedGraph,
    duals: PricingDuals,
    extension_context: _DominanceExtensionContext,
    deadline_counters: _DeadlinePricingCounters | None = None,
    shortest: dict[tuple[str, str], float] | None = None,
    bounds: _PricingBounds | None = None,
) -> tuple[bool, int, int]:
    objective = extension_context.objective
    restrictions = extension_context.restrictions
    location = _physical_location(label)
    comparable = kept_labels.setdefault(location, [])
    frontier_dominators = _dominance_frontier_candidates(
        kept_labels,
        location,
        label,
        graph,
        objective,
        restrictions,
        deadline_counters,
        label_may_dominate_stored=False,
        shortest=shortest,
        bounds=bounds,
    )
    possible_dominators = _dominance_gate_candidates(
        frontier_dominators,
        label,
        graph,
        objective,
        restrictions,
        deadline_counters,
    )
    if any(
        _paper_dominates(incumbent, label, graph, duals, extension_context, deadline_counters)
        for incumbent in possible_dominators
    ):
        return False, 1, 0
    frontier_dominated = _dominance_frontier_candidates(
        kept_labels,
        location,
        label,
        graph,
        objective,
        restrictions,
        deadline_counters,
        label_may_dominate_stored=True,
        shortest=shortest,
        bounds=bounds,
    )
    candidate_ids = {id(candidate) for candidate in frontier_dominated}
    survivors = []
    dominated_count = 0
    for incumbent in comparable:
        if id(incumbent) in candidate_ids and (
            _dominance_pair_gate_reason(
                label,
                incumbent,
                graph,
                objective,
                restrictions,
                deadline_counters,
            )
            is None
            and _paper_dominates(label, incumbent, graph, duals, extension_context, deadline_counters)
        ):
            dominated_count += 1
        else:
            survivors.append(incumbent)
    survivors.append(label)
    kept_labels[location] = survivors
    _rebuild_forward_frontier_for_location(
        kept_labels,
        location,
        graph,
        objective,
        restrictions,
        deadline_counters,
        shortest,
        bounds,
    )
    return True, 0, dominated_count

def _insert_nondominated_farkas_label(
    kept_labels: dict[str, list[_Label]],
    label: _Label,
    graph: TransformedGraph,
    extension_context: _DominanceExtensionContext,
) -> tuple[bool, int, int]:
    location = _physical_location(label)
    comparable = kept_labels.setdefault(location, [])
    if any(_farkas_dominates(incumbent, label, graph, extension_context) for incumbent in comparable):
        return False, 1, 0
    survivors = []
    dominated_count = 0
    for incumbent in comparable:
        if _farkas_dominates(label, incumbent, graph, extension_context):
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
    duals: PricingDuals,
    extension_context: _DominanceExtensionContext,
    deadline_counters: _DeadlinePricingCounters | None = None,
) -> bool:
    objective = extension_context.objective
    restrictions = extension_context.restrictions
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
    if _branch_state(a, restrictions) != _branch_state(b, restrictions):
        if deadline_counters is not None:
            deadline_counters.forward_branch_language_failures += 1
            deadline_counters.dom_gate_branch_failures += 1
            deadline_counters.dom_prefilter_branch_fail += 1
            deadline_counters.forward_return_time_credit_checks_skipped += 1
        return False
    if not _branch_interface_compatible(a, b, graph, extension_context):
        if deadline_counters is not None:
            deadline_counters.forward_branch_interface_failures += 1
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
    physdom_start = time.perf_counter() if deadline_counters is not None and not same_original_node else None
    adjusted_cost = a.reduced_cost - _sr_extra_penalty_bound(a, b, duals)
    if adjusted_cost > b.reduced_cost + objective.coeffs.return_time * return_credit:
        if deadline_counters is not None:
            deadline_counters.forward_mask_scalar_prefilter_failures += 1
            deadline_counters.dom_gate_scalar_failures += 1
            deadline_counters.dom_prefilter_return_credit_fail += 1
            if physdom_start is not None:
                deadline_counters.physdom_time += time.perf_counter() - physdom_start
        return False
    if deadline_counters is not None:
        deadline_counters.dom_full_tests += 1
        if same_original_node:
            deadline_counters.forward_same_node_dominance_tests += 1
            deadline_counters.dom_full_tests_same_node += 1
            deadline_counters.full_same_node_tests += 1
        else:
            deadline_counters.forward_physical_location_dominance_tests += 1
            deadline_counters.dom_full_tests_physical_location += 1
            deadline_counters.full_physical_location_tests += 1
            deadline_counters.physical_location_full_tests += 1
            deadline_counters.physdom_full_tests += 1
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
        deadline_counters.dom_labels_deleted_physical_location += 1
        deadline_counters.labels_deleted_physical_location += 1
        deadline_counters.physdom_deletions += 1
    if dominated and deadline_counters is not None and same_original_node:
        deadline_counters.labels_dominated_same_node += 1
        deadline_counters.dom_labels_deleted_same_node += 1
        deadline_counters.labels_deleted_same_node += 1
    if dominated and deadline_counters is not None:
        deadline_counters.dom_full_rejections += 1
    if physdom_start is not None:
        deadline_counters.physdom_time += time.perf_counter() - physdom_start
    return dominated

def _farkas_dominates(
    a: _Label,
    b: _Label,
    graph: TransformedGraph,
    extension_context: _DominanceExtensionContext,
) -> bool:
    restrictions = extension_context.restrictions
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
    if _branch_state(a, restrictions) != _branch_state(b, restrictions):
        return False
    if not _branch_interface_compatible(a, b, graph, extension_context):
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
    if is_duplicate(node):
        if label.active_pad is None:
            raise ValueError("duplicate label has no active pad")
        return label.active_pad
    return node

def _block_position(label: _Label, graph: TransformedGraph) -> int:
    node = label.path[-1]
    if node in graph.instance.hubs:
        return 0
    if is_duplicate(node):
        if label.active_pad is None:
            raise ValueError("duplicate label has no active pad")
        return graph.order[(label.active_pad, duplicate_customer(node))] + 1
    return 0

def _branch_state(
    label: _Label,
    restrictions: BranchRestrictions,
) -> _BranchState:
    return _branch_state_from_resources(label.represented, label.used_arcs, restrictions)

def _branch_state_from_resources(
    represented: frozenset[str],
    used_arcs: frozenset[tuple[str, str]],
    restrictions: BranchRestrictions,
) -> _BranchState:
    together = tuple(
        (p in represented, q in represented)
        for p, q in sorted(restrictions.together_pairs)
    )
    conditioned_arcs = tuple(
        (customer, (i, j), required, customer in represented, (i, j) in used_arcs)
        for required, branches in (
            (False, restrictions.conditioned_arc_forbidden),
            (True, restrictions.conditioned_arc_required),
        )
        for customer, i, j in sorted(branches)
    )
    return _BranchState(together=together, conditioned_arcs=conditioned_arcs)

def _branch_interface_compatible(
    dominating: _Label,
    dominated: _Label,
    graph: TransformedGraph,
    extension_context: _DominanceExtensionContext,
) -> bool:
    dominating_endpoint = dominating.path[-1]
    dominated_endpoint = dominated.path[-1]
    if dominating_endpoint == dominated_endpoint:
        return True

    restrictions = extension_context.restrictions
    if _branch_state(dominating, restrictions) != _branch_state(dominated, restrictions):
        return False

    dominated_successors = _feasible_first_successors(dominated, graph, extension_context)
    dominating_successors = frozenset(_feasible_first_successors(dominating, graph, extension_context))
    for successor in dominated_successors:
        if successor not in dominating_successors:
            return False
        dominating_represented, dominating_used_arcs = _first_extension_resources(
            dominating,
            successor,
            graph,
        )
        dominated_represented, dominated_used_arcs = _first_extension_resources(
            dominated,
            successor,
            graph,
        )

        dominating_next_state = _branch_state_from_resources(
            dominating_represented,
            dominating_used_arcs,
            restrictions,
        )
        dominated_next_state = _branch_state_from_resources(
            dominated_represented,
            dominated_used_arcs,
            restrictions,
        )
        if dominating_next_state != dominated_next_state:
            return False
    return True


def _feasible_first_successors(
    label: _Label,
    graph: TransformedGraph,
    extension_context: _DominanceExtensionContext,
) -> tuple[str, ...]:
    cached = extension_context.feasible_first_successors.get(label)
    if cached is not None:
        return cached

    feasible: list[str] = []
    for successor in graph.out_arcs.get(label.path[-1], tuple()):
        if not _extension_allowed(
            label,
            successor,
            graph,
            extension_context.residual_customers,
            extension_context.restrictions,
        ):
            continue
        if _extension_rejected_by_service_deadline(
            label,
            successor,
            graph,
            extension_context.objective,
        ):
            continue
        if successor == graph.instance.depot_sink:
            represented, used_arcs = _first_extension_resources(label, successor, graph)
            if not represented or not _completion_resources_allowed(
                represented,
                used_arcs,
                extension_context.restrictions,
            ):
                continue
        feasible.append(successor)

    result = tuple(feasible)
    extension_context.feasible_first_successors[label] = result
    return result


def _first_extension_resources(
    label: _Label,
    successor: str,
    graph: TransformedGraph,
) -> tuple[frozenset[str], frozenset[tuple[str, str]]]:
    represented = label.represented
    if is_customer_representation(successor, graph.instance):
        represented = represented | {served_customer(successor)}
    return represented, label.used_arcs | {(label.path[-1], successor)}

def _sr_extra_penalty_bound(a: _Label, b: _Label, duals: PricingDuals) -> float:
    a_counts = dict(a.sr_counts)
    b_counts = dict(b.sr_counts)
    penalty = 0.0
    for triplet, dual in duals.nu.items():
        if dual < 0.0 and a_counts.get(triplet, 0) == 1 and b_counts.get(triplet, 0) == 2:
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
        return _farkas_completion_lower_bound(label, graph, objective, shortest, bounds, deadline_counters)
    if not use_standard_acceleration:
        return label.reduced_cost
    return _closure_reduced_cost_lower_bound(label, graph, objective, shortest, bounds, deadline_counters)


def _farkas_completion_lower_bound(
    label: _Label,
    graph: TransformedGraph,
    objective: ObjectiveData,
    shortest: dict[tuple[str, str], float],
    bounds: _PricingBounds,
    deadline_counters: _DeadlinePricingCounters | None = None,
) -> float:
    reward = _dual_reward_bound(label, graph, bounds, objective, shortest, deadline_counters)
    return label.reduced_cost - reward

def _closure_reduced_cost_lower_bound(
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

def _completion_return_lb(
    label: _Label,
    graph: TransformedGraph,
    shortest: dict[tuple[str, str], float],
) -> float:
    instance = graph.instance
    node = label.path[-1]
    if is_duplicate(node):
        hub = label.active_pad
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
    current = label.active_pad if is_duplicate(label.path[-1]) else label.path[-1]
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
    return _payload_reward_bound(items, residual_payload)

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

def _together_branch_partner_unreachable(
    label: _Label,
    graph: TransformedGraph,
    objective: ObjectiveData,
    shortest: dict[tuple[str, str], float],
    residual_customers: frozenset[str],
    restrictions: BranchRestrictions,
) -> bool:
    if not restrictions.together_pairs:
        return False
    residual_payload = graph.instance.truck_payload - label.truck_load + PAYLOAD_TOLERANCE
    location = _physical_location(label)
    for p, q in restrictions.together_pairs:
        p_represented = p in label.represented
        q_represented = q in label.represented
        if p_represented == q_represented:
            continue
        missing = q if p_represented else p
        if missing not in residual_customers:
            return True
        if graph.instance.demand[missing] > residual_payload:
            return True
        if not _customer_reachable_from_location(missing, location, graph, shortest):
            return True
        if not _customer_deadline_reachable_from_label(label, missing, graph, objective, shortest):
            return True
    return False


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
    location = label.active_pad if is_duplicate(node) else node
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
