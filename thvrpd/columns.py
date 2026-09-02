from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass, field
import time

from .branching import BranchRestrictions
from .routes import Route
from .transform import TransformedGraph

PAPER_DOMINANCE_TOLERANCE = 1e-9
COST_SIGNATURE_TOLERANCE = 1e-9


@dataclass(frozen=True)
class CoreRouteSignature:
    served_mask: int
    truck_served_mask: int
    drone_served_mask: int
    pad_assignment_key: tuple[tuple[str, int], ...]
    transformed_arc_mask: int
    route_cost_key: int | None


@dataclass(frozen=True)
class ActiveRouteSignature:
    core: CoreRouteSignature
    active_sr_coeff_key: tuple[int, ...]
    active_sr_version: int


@dataclass(frozen=True)
class RouteCoefficientSignature:
    served_mask: int
    truck_served_mask: int
    drone_served_mask: int
    pad_assignment_key: tuple[tuple[str, int], ...]
    transformed_arc_mask: int
    active_sr_coeff_key: tuple[int, ...]
    active_sr_version: int


RouteSignature = ActiveRouteSignature


@dataclass(frozen=True)
class BranchObservableSignature:
    served_mask: int
    truck_served_mask: int
    drone_served_mask: int
    pad_assignment_key: tuple[tuple[str, int], ...]
    transformed_arc_mask: int


@dataclass(frozen=True)
class BranchRouteIndex:
    all_paths: frozenset[tuple[str, ...]]
    signatures: dict[tuple[str, ...], BranchObservableSignature]
    by_customer: dict[str, frozenset[tuple[str, ...]]]
    by_pad: dict[tuple[str, str], frozenset[tuple[str, ...]]]
    by_arc: dict[tuple[str, str], frozenset[tuple[str, ...]]]


@dataclass
class ColumnCacheStats:
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
    signature_build_time: float = 0.0
    sr_coeff_build_time: float = 0.0
    duplicate_equivalent_rejected: int = 0
    cost_dominated_rejected: int = 0
    cost_dominated_removed: int = 0
    column_index_hits: int = 0
    column_index_misses: int = 0
    column_index_replacements: int = 0
    column_index_refreshes: int = 0
    column_index_refresh_time: float = 0.0


@dataclass
class NodeColumnIndex:
    residual_key: tuple[str, ...] = tuple()
    active_sr_key: tuple[tuple[str, str, str], ...] = tuple()
    active_sr_version: int = -1
    indexed_paths: frozenset[tuple[str, ...]] = frozenset()
    by_coeff: dict[RouteCoefficientSignature, tuple[str, ...]] = field(default_factory=dict)

    def matches(
        self,
        residual_customers: frozenset[str],
        active_sr: set[tuple[str, str, str]] | frozenset[tuple[str, str, str]] | tuple[tuple[str, str, str], ...],
        active_sr_version: int,
        paths: set[tuple[str, ...]] | frozenset[tuple[str, ...]] | None = None,
    ) -> bool:
        state_matches = (
            self.residual_key == tuple(sorted(residual_customers))
            and self.active_sr_key == tuple(sorted(active_sr))
            and self.active_sr_version == active_sr_version
        )
        if paths is None:
            return state_matches
        return state_matches and self.indexed_paths == frozenset(paths)

    def reset(
        self,
        residual_customers: frozenset[str],
        active_sr: set[tuple[str, str, str]] | frozenset[tuple[str, str, str]] | tuple[tuple[str, str, str], ...],
        active_sr_version: int,
        paths: set[tuple[str, ...]] | frozenset[tuple[str, ...]] = frozenset(),
    ) -> None:
        self.residual_key = tuple(sorted(residual_customers))
        self.active_sr_key = tuple(sorted(active_sr))
        self.active_sr_version = active_sr_version
        self.indexed_paths = frozenset(paths)
        self.by_coeff.clear()


class RouteSignatureCache:
    def __init__(self) -> None:
        self.route_signatures: dict[
            tuple[
                tuple[str, ...],
                tuple[str, ...],
                tuple[tuple[str, str, str], ...],
                int,
                int | None,
            ],
            RouteSignature,
        ] = {}
        self.resource_signatures: dict[
            tuple[
                tuple[str, ...],
                tuple[str, ...],
                tuple[tuple[str, str], ...],
                tuple[tuple[str, str], ...],
                tuple[str, ...],
                tuple[tuple[str, str, str], ...],
                int,
                int | None,
            ],
            RouteSignature,
        ] = {}
        self.core_signatures: dict[
            tuple[
                tuple[str, ...],
                tuple[str, ...],
                tuple[tuple[str, str], ...],
                tuple[tuple[str, str], ...],
                tuple[str, ...],
                int | None,
            ],
            CoreRouteSignature,
        ] = {}
        self.customer_indexes: dict[tuple[str, ...], dict[str, int]] = {}
        self.branch_arc_indexes: dict[tuple[str, ...], dict[tuple[str, str], int]] = {}
        self.triplet_masks: dict[tuple[tuple[str, ...], tuple[str, str, str]], int] = {}
        self.sr_coeff_cache: dict[tuple[int, int], int] = {}
        self.active_sr_key_cache: dict[tuple[int, int, tuple[tuple[str, str, str], ...]], tuple[int, ...]] = {}
        self.active_sr_versions: dict[int, set[tuple[tuple[str, str, str], ...]]] = {}
        self.stats = ColumnCacheStats()


def branch_observable_signature(
    route: Route,
    graph: TransformedGraph,
    residual_customers: frozenset[str],
    cache: RouteSignatureCache | None = None,
) -> BranchObservableSignature:
    signature = route_signature(
        route,
        graph,
        residual_customers,
        cache,
        active_sr=tuple(),
        active_sr_version=0,
    ).core
    return BranchObservableSignature(
        served_mask=signature.served_mask,
        truck_served_mask=signature.truck_served_mask,
        drone_served_mask=signature.drone_served_mask,
        pad_assignment_key=signature.pad_assignment_key,
        transformed_arc_mask=signature.transformed_arc_mask,
    )


def build_branch_route_index(
    paths: set[tuple[str, ...]] | frozenset[tuple[str, ...]],
    routes: dict[tuple[str, ...], Route],
    graph: TransformedGraph,
    residual_customers: frozenset[str],
    cache: RouteSignatureCache | None = None,
) -> BranchRouteIndex:
    signatures: dict[tuple[str, ...], BranchObservableSignature] = {}
    by_customer: dict[str, set[tuple[str, ...]]] = defaultdict(set)
    by_pad: dict[tuple[str, str], set[tuple[str, ...]]] = defaultdict(set)
    by_arc: dict[tuple[str, str], set[tuple[str, ...]]] = defaultdict(set)
    for path in sorted(paths):
        route = routes[path]
        signatures[path] = branch_observable_signature(route, graph, residual_customers, cache)
        for customer in route.served.intersection(residual_customers):
            by_customer[customer].add(path)
        for hub, customer in route.pad_served:
            if customer in residual_customers:
                by_pad[(hub, customer)].add(path)
        for arc in route.used_arcs:
            by_arc[arc].add(path)
    return BranchRouteIndex(
        all_paths=frozenset(paths),
        signatures=signatures,
        by_customer=_freeze_path_index(by_customer),
        by_pad=_freeze_path_index(by_pad),
        by_arc=_freeze_path_index(by_arc),
    )


def extend_branch_route_index(
    index: BranchRouteIndex,
    new_paths: set[tuple[str, ...]] | frozenset[tuple[str, ...]],
    routes: dict[tuple[str, ...], Route],
    graph: TransformedGraph,
    residual_customers: frozenset[str],
    cache: RouteSignatureCache | None = None,
) -> BranchRouteIndex:
    delta_paths = frozenset(path for path in new_paths if path not in index.all_paths)
    if not delta_paths:
        return index
    delta = build_branch_route_index(delta_paths, routes, graph, residual_customers, cache)
    return BranchRouteIndex(
        all_paths=index.all_paths | delta.all_paths,
        signatures={**index.signatures, **delta.signatures},
        by_customer=_merge_path_index(index.by_customer, delta.by_customer),
        by_pad=_merge_path_index(index.by_pad, delta.by_pad),
        by_arc=_merge_path_index(index.by_arc, delta.by_arc),
    )


def query_branch_route_index(
    index: BranchRouteIndex,
    restrictions: BranchRestrictions,
) -> tuple[set[tuple[str, ...]], dict[str, int]]:
    candidates = set(index.all_paths)
    rejection_counts = {
        "together": 0,
        "separate": 0,
        "launch_pad": 0,
        "conditioned_arc": 0,
    }

    def reject(reason: str, paths: set[tuple[str, ...]] | frozenset[tuple[str, ...]]) -> None:
        removed = candidates.intersection(paths)
        if removed:
            candidates.difference_update(removed)
            rejection_counts[reason] += len(removed)

    for p, q in restrictions.together_pairs:
        p_paths = index.by_customer.get(p, frozenset())
        q_paths = index.by_customer.get(q, frozenset())
        reject("together", set(p_paths).symmetric_difference(q_paths))
    for p, q in restrictions.separate_pairs:
        reject("separate", set(index.by_customer.get(p, frozenset())).intersection(index.by_customer.get(q, frozenset())))
    for hub, customer in restrictions.pad_forbidden:
        reject("launch_pad", index.by_pad.get((hub, customer), frozenset()))
    for hub, customer in restrictions.pad_required:
        served = index.by_customer.get(customer, frozenset())
        pad = index.by_pad.get((hub, customer), frozenset())
        reject("launch_pad", set(served).difference(pad))
    for customer, i, j in restrictions.conditioned_arc_forbidden:
        reject(
            "conditioned_arc",
            set(index.by_customer.get(customer, frozenset())).intersection(index.by_arc.get((i, j), frozenset())),
        )
    for customer, i, j in restrictions.conditioned_arc_required:
        served = index.by_customer.get(customer, frozenset())
        reject("conditioned_arc", set(served).difference(index.by_arc.get((i, j), frozenset())))
    return candidates, rejection_counts


def _freeze_path_index(index: dict) -> dict:
    return {key: frozenset(value) for key, value in index.items()}


def _merge_path_index(left: dict, right: dict) -> dict:
    keys = set(left) | set(right)
    return {
        key: frozenset(left.get(key, frozenset()) | right.get(key, frozenset()))
        for key in keys
    }


def route_signature(
    route: Route,
    graph: TransformedGraph,
    residual_customers: frozenset[str],
    cache: RouteSignatureCache | None = None,
    active_sr: set[tuple[str, str, str]] | frozenset[tuple[str, str, str]] | tuple[tuple[str, str, str], ...] = tuple(),
    active_sr_version: int = 0,
) -> RouteSignature:
    residual_key = tuple(sorted(residual_customers))
    active_key = tuple(sorted(active_sr))
    route_cost_key = _cost_key(route.cost)
    key = (
        route.path,
        residual_key,
        active_key,
        active_sr_version,
        route_cost_key,
    )
    if cache is not None:
        cached = cache.route_signatures.get(key)
        if cached is not None:
            cache.stats.signature_cache_hits += 1
            cache.stats.active_signature_cache_hits += 1
            return cached
        cache.stats.signature_cache_misses += 1
        cache.stats.active_signature_cache_misses += 1
    signature = route_signature_from_resources(
        route.served,
        route.truck_served,
        route.pad_served,
        route.used_arcs,
        graph,
        residual_customers,
        cache,
        active_sr=active_key,
        active_sr_version=active_sr_version,
        route_cost=route.cost,
    )
    if cache is not None:
        cache.route_signatures[key] = signature
    return signature


def route_signature_from_resources(
    served: frozenset[str],
    truck_served: frozenset[str],
    pad_served: frozenset[tuple[str, str]],
    used_arcs: frozenset[tuple[str, str]],
    graph: TransformedGraph,
    residual_customers: frozenset[str],
    cache: RouteSignatureCache | None = None,
    active_sr: set[tuple[str, str, str]] | frozenset[tuple[str, str, str]] | tuple[tuple[str, str, str], ...] = tuple(),
    active_sr_version: int = 0,
    route_cost: float | None = None,
) -> RouteSignature:
    residual_key = tuple(sorted(residual_customers))
    served_key = tuple(sorted(served.intersection(residual_customers)))
    truck_key = tuple(sorted(truck_served.intersection(residual_customers)))
    pad_key = tuple(sorted((hub, customer) for hub, customer in pad_served if customer in residual_customers))
    branch_arcs = tuple(sorted(arc for arc in used_arcs if graph.arc_customer_set(arc).intersection(residual_customers)))
    active_key = tuple(sorted(active_sr))
    route_cost_key = _cost_key(route_cost)
    key = (served_key, truck_key, pad_key, branch_arcs, residual_key, active_key, active_sr_version, route_cost_key)
    if cache is not None:
        cached = cache.resource_signatures.get(key)
        if cached is not None:
            cache.stats.signature_cache_hits += 1
            cache.stats.active_signature_cache_hits += 1
            return cached
        cache.stats.signature_cache_misses += 1
        cache.stats.active_signature_cache_misses += 1
    start = time.time()
    core = _core_route_signature(
        served_key,
        truck_key,
        pad_key,
        branch_arcs,
        residual_key,
        graph,
        cache,
        route_cost_key,
    )
    active_sr_coeff_key = active_sr_signature_key(
        core.served_mask,
        residual_key,
        active_key,
        active_sr_version,
        cache,
    )
    signature = ActiveRouteSignature(
        core=core,
        active_sr_coeff_key=active_sr_coeff_key,
        active_sr_version=active_sr_version,
    )
    if cache is not None:
        cache.stats.signature_build_time += time.time() - start
        cache.resource_signatures[key] = signature
    return signature


def route_coefficient_signature(signature: RouteSignature) -> RouteCoefficientSignature:
    return RouteCoefficientSignature(
        served_mask=signature.core.served_mask,
        truck_served_mask=signature.core.truck_served_mask,
        drone_served_mask=signature.core.drone_served_mask,
        pad_assignment_key=signature.core.pad_assignment_key,
        transformed_arc_mask=signature.core.transformed_arc_mask,
        active_sr_coeff_key=signature.active_sr_coeff_key,
        active_sr_version=signature.active_sr_version,
    )


def sr_coeff_from_mask(served_mask: int, triplet_mask: int) -> int:
    return ((served_mask & triplet_mask).bit_count()) // 2


def active_sr_signature_key(
    served_mask: int,
    residual_key: tuple[str, ...],
    active_sr: tuple[tuple[str, str, str], ...],
    active_sr_version: int,
    cache: RouteSignatureCache | None = None,
) -> tuple[int, ...]:
    if cache is not None:
        cache.active_sr_versions.setdefault(active_sr_version, set()).add(active_sr)
        key = (served_mask, active_sr_version, active_sr)
        cached = cache.active_sr_key_cache.get(key)
        if cached is not None:
            cache.stats.sr_coeff_cache_hits += 1
            cache.stats.active_sr_key_cache_hits += 1
            return cached
        cache.stats.sr_coeff_cache_misses += 1
        cache.stats.active_sr_key_cache_misses += 1
    start = time.time()
    coeffs = tuple(
        sr_coeff_from_mask(
            served_mask,
            triplet_mask(residual_key, triplet, cache),
        )
        for triplet in active_sr
    )
    if cache is not None:
        cache.stats.sr_coeff_build_time += time.time() - start
        cache.stats.active_sr_coeffs_computed += len(active_sr)
        cache.active_sr_key_cache[(served_mask, active_sr_version, active_sr)] = coeffs
    return coeffs


def triplet_mask(
    residual_key: tuple[str, ...],
    triplet: tuple[str, str, str],
    cache: RouteSignatureCache | None = None,
) -> int:
    triplet_key = tuple(sorted(triplet))
    key = (residual_key, triplet_key)
    if cache is not None:
        cached = cache.triplet_masks.get(key)
        if cached is not None:
            return cached
    index = _customer_index(residual_key, cache)
    mask = 0
    for customer in triplet_key:
        bit = index.get(customer)
        if bit is not None:
            mask |= 1 << bit
    if cache is not None:
        cache.triplet_masks[key] = mask
        cache.stats.triplet_masks_built += 1
    return mask


def customer_mask(
    customers: frozenset[str] | set[str] | tuple[str, ...],
    residual_key: tuple[str, ...],
    cache: RouteSignatureCache | None = None,
) -> int:
    index = _customer_index(residual_key, cache)
    mask = 0
    for customer in customers:
        bit = index.get(customer)
        if bit is not None:
            mask |= 1 << bit
    return mask


def _core_route_signature(
    served_key: tuple[str, ...],
    truck_key: tuple[str, ...],
    pad_key: tuple[tuple[str, str], ...],
    branch_arcs: tuple[tuple[str, str], ...],
    residual_key: tuple[str, ...],
    graph: TransformedGraph,
    cache: RouteSignatureCache | None,
    route_cost_key: int | None,
) -> CoreRouteSignature:
    key = (served_key, truck_key, pad_key, branch_arcs, residual_key, route_cost_key)
    if cache is not None:
        cached = cache.core_signatures.get(key)
        if cached is not None:
            cache.stats.core_signature_cache_hits += 1
            return cached
        cache.stats.core_signature_cache_misses += 1
    index = _customer_index(residual_key, cache)
    served_mask = customer_mask(served_key, residual_key, cache)
    truck_served_mask = customer_mask(truck_key, residual_key, cache)
    drone_customers = tuple(customer for _, customer in pad_key)
    drone_served_mask = customer_mask(drone_customers, residual_key, cache)
    pad_assignment_key = tuple((hub, index[customer]) for hub, customer in pad_key)
    arc_index = _branch_arc_index(residual_key, graph, cache)
    transformed_arc_mask = 0
    for arc in branch_arcs:
        transformed_arc_mask |= 1 << arc_index[arc]
    core = CoreRouteSignature(
        served_mask=served_mask,
        truck_served_mask=truck_served_mask,
        drone_served_mask=drone_served_mask,
        pad_assignment_key=pad_assignment_key,
        transformed_arc_mask=transformed_arc_mask,
        route_cost_key=route_cost_key,
    )
    if cache is not None:
        cache.core_signatures[key] = core
    return core


def _customer_index(residual_key: tuple[str, ...], cache: RouteSignatureCache | None = None) -> dict[str, int]:
    if cache is None:
        return {customer: idx for idx, customer in enumerate(residual_key)}
    cached = cache.customer_indexes.get(residual_key)
    if cached is None:
        cached = {customer: idx for idx, customer in enumerate(residual_key)}
        cache.customer_indexes[residual_key] = cached
    return cached


def _branch_arc_index(
    residual_key: tuple[str, ...],
    graph: TransformedGraph,
    cache: RouteSignatureCache | None = None,
) -> dict[tuple[str, str], int]:
    if cache is not None:
        cached = cache.branch_arc_indexes.get(residual_key)
        if cached is not None:
            return cached
    residual = frozenset(residual_key)
    arcs = tuple(sorted(arc for arc in graph.arcs if graph.arc_customer_set(arc).intersection(residual)))
    index = {arc: idx for idx, arc in enumerate(arcs)}
    if cache is not None:
        cache.branch_arc_indexes[residual_key] = index
    return index


def _cost_key(cost: float | None) -> int | None:
    if cost is None:
        return None
    return round(cost / COST_SIGNATURE_TOLERANCE)


def _prefer_path(candidate_path: tuple[str, ...], incumbent_path: tuple[str, ...], routes: dict[tuple[str, ...], Route]) -> bool:
    candidate = routes[candidate_path]
    incumbent = routes[incumbent_path]
    if candidate.cost < incumbent.cost - PAPER_DOMINANCE_TOLERANCE:
        return True
    if abs(candidate.cost - incumbent.cost) <= PAPER_DOMINANCE_TOLERANCE and candidate_path < incumbent_path:
        return True
    return False


def merge_duplicate_column_paths(
    paths: set[tuple[str, ...]],
    routes: dict[tuple[str, ...], Route],
    graph: TransformedGraph,
    residual_customers: frozenset[str],
    cache: RouteSignatureCache | None = None,
    active_sr: set[tuple[str, str, str]] | frozenset[tuple[str, str, str]] | tuple[tuple[str, str, str], ...] = tuple(),
    active_sr_version: int = 0,
) -> set[tuple[str, ...]]:
    seen: dict[RouteCoefficientSignature, tuple[str, ...]] = {}
    for path in sorted(paths):
        signature = route_signature(routes[path], graph, residual_customers, cache, active_sr, active_sr_version)
        coeff_signature = route_coefficient_signature(signature)
        incumbent = seen.get(coeff_signature)
        if incumbent is None or _prefer_path(path, incumbent, routes):
            seen[coeff_signature] = path
    return set(seen.values())


def insert_node_column(
    route: Route,
    routes: dict[tuple[str, ...], Route],
    paths: set[tuple[str, ...]],
    graph: TransformedGraph,
    residual_customers: frozenset[str],
    comparison_paths: set[tuple[str, ...]] | None = None,
    cache: RouteSignatureCache | None = None,
    active_sr: set[tuple[str, str, str]] | frozenset[tuple[str, str, str]] | tuple[tuple[str, str, str], ...] = tuple(),
    active_sr_version: int = 0,
    column_index: NodeColumnIndex | None = None,
) -> tuple[tuple[str, ...], bool]:
    new_signature = route_signature(route, graph, residual_customers, cache, active_sr, active_sr_version)
    new_coeff_signature = route_coefficient_signature(new_signature)
    if column_index is not None:
        if not column_index.matches(residual_customers, active_sr, active_sr_version):
            raise ValueError("node column index does not match the active residual node state")
        incumbent_path = column_index.by_coeff.get(new_coeff_signature)
        if incumbent_path is None:
            if cache is not None:
                cache.stats.column_index_misses += 1
            routes[route.path] = route
            paths.add(route.path)
            column_index.indexed_paths = frozenset(paths)
            column_index.by_coeff[new_coeff_signature] = route.path
            return route.path, True
        if cache is not None:
            cache.stats.column_index_hits += 1
        incumbent = routes[incumbent_path]
        if incumbent.cost <= route.cost + PAPER_DOMINANCE_TOLERANCE:
            if cache is not None:
                incumbent_signature = route_signature(
                    incumbent,
                    graph,
                    residual_customers,
                    cache,
                    active_sr,
                    active_sr_version,
                )
                if incumbent_signature == new_signature or abs(incumbent.cost - route.cost) <= COST_SIGNATURE_TOLERANCE:
                    cache.stats.duplicate_equivalent_rejected += 1
                else:
                    cache.stats.cost_dominated_rejected += 1
            return incumbent_path, False
        paths.discard(incumbent_path)
        routes[route.path] = route
        paths.add(route.path)
        column_index.indexed_paths = frozenset(paths)
        column_index.by_coeff[new_coeff_signature] = route.path
        if cache is not None:
            cache.stats.cost_dominated_removed += 1
            cache.stats.column_index_replacements += 1
        return route.path, True
    dominated_paths = []
    for path in sorted(paths if comparison_paths is None else comparison_paths):
        signature = route_signature(routes[path], graph, residual_customers, cache, active_sr, active_sr_version)
        if route_coefficient_signature(signature) == new_coeff_signature:
            if routes[path].cost <= route.cost + PAPER_DOMINANCE_TOLERANCE:
                if cache is not None:
                    if signature == new_signature or abs(routes[path].cost - route.cost) <= COST_SIGNATURE_TOLERANCE:
                        cache.stats.duplicate_equivalent_rejected += 1
                    else:
                        cache.stats.cost_dominated_rejected += 1
                return path, False
            dominated_paths.append(path)
    for path in dominated_paths:
        paths.discard(path)
    if cache is not None:
        cache.stats.cost_dominated_removed += len(dominated_paths)
    routes[route.path] = route
    paths.add(route.path)
    return route.path, True


def refresh_node_column_index(
    column_index: NodeColumnIndex,
    routes: dict[tuple[str, ...], Route],
    paths: set[tuple[str, ...]],
    graph: TransformedGraph,
    residual_customers: frozenset[str],
    cache: RouteSignatureCache | None = None,
    active_sr: set[tuple[str, str, str]] | frozenset[tuple[str, str, str]] | tuple[tuple[str, str, str], ...] = tuple(),
    active_sr_version: int = 0,
) -> None:
    start = time.time()
    column_index.reset(residual_customers, active_sr, active_sr_version, paths)
    for path in sorted(paths):
        route = routes[path]
        signature = route_signature(route, graph, residual_customers, cache, active_sr, active_sr_version)
        coeff_signature = route_coefficient_signature(signature)
        incumbent_path = column_index.by_coeff.get(coeff_signature)
        if incumbent_path is None or _prefer_path(path, incumbent_path, routes):
            column_index.by_coeff[coeff_signature] = path
    if cache is not None:
        cache.stats.column_index_refreshes += 1
        cache.stats.column_index_refresh_time += time.time() - start
