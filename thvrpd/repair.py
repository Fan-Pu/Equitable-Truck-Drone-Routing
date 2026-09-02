from __future__ import annotations

from collections import Counter
from dataclasses import asdict, dataclass, replace
from hashlib import sha256
import json
from math import ceil, isfinite
import time

import gurobipy as gp
from gurobipy import GRB
import networkx as nx

from .arc_classes import (
    ARC_CLASSES,
    allowable_internal_truck_arcs,
    route_internal_truck_arcs,
    truck_arc_class as _arc_class,
    truck_arc_class_counts,
)
from .config import ObjectiveWeights
from .branching import BranchRestrictions
from .feasibility import (
    FeasibilityGateDiagnostics,
    FeasibilityPrecheckResult,
    exact_feasibility_check,
    exact_feasibility_precheck,
)
from .instance import Arc, InstanceData, _drone_arc_savings, instance_physical_fingerprint
from .objective import ObjectiveCoefficients, ObjectiveData, build_objective_data
from .pricing import PricingDuals, price_route
from .routes import Route, route_from_path
from .solverlog import configure_gurobi_logging
from .transform import build_transformed_graph, duplicate_node


@dataclass(frozen=True)
class RepairValidation:
    valid: bool
    issues: tuple[str, ...]
    routes: tuple[dict[str, object], ...]


@dataclass(frozen=True)
class ArcSwapRepairDiagnostics:
    full_universe_precheck: FeasibilityPrecheckResult
    original_discovery_status: str
    original_discovery_time_seconds: float
    original_discovery_nodes: float
    original_discovery_iterations: float
    original_discovery_fallback_used: bool
    original_discovery_diagnostics: FeasibilityGateDiagnostics | None
    repair_invoked: bool
    projection_type: str
    strengthened_arc_projection: StrengthenedArcProjectionDiagnostics | None
    original_arc_counts: dict[str, int]
    repaired_arc_counts: dict[str, int]
    added_arcs: tuple[Arc, ...]
    removed_arcs: tuple[Arc, ...]
    hamming_edit_count: int
    route_search_variable_count: int
    route_search_constraint_count: int
    route_search_nonzero_count: int
    route_search_build_time_seconds: float
    route_search_solve_time_seconds: float
    route_search_node_count: float
    route_search_iteration_count: float
    repair_variable_count: int
    repair_constraint_count: int
    repair_nonzero_count: int
    repair_build_time_seconds: float
    repair_solve_time_seconds: float
    repair_node_count: float
    repair_iteration_count: float
    rejected_pattern_count: int
    certificate_routes: tuple[dict[str, object], ...]
    certificate_validation: RepairValidation
    original_graph_fingerprint: str
    repaired_graph_fingerprint: str
    original_deadline_fingerprint: str
    repaired_deadline_fingerprint: str
    original_benchmark_fingerprint: str
    repaired_benchmark_fingerprint: str


@dataclass(frozen=True)
class ArcSwapRepairResult:
    feasible: bool
    status: str
    instance: InstanceData | None
    diagnostics: ArcSwapRepairDiagnostics


@dataclass(frozen=True)
class _RepairModel:
    model: gp.Model
    activation: gp.tupledict
    x: gp.tupledict
    y: gp.tupledict
    used: gp.tupledict
    allowed_internal_arcs: tuple[Arc, ...]
    all_truck_arcs: tuple[Arc, ...]
    truck_time: dict[Arc, float]
    eligible_route_arc_count: int
    pruned_route_arc_count: int
    pair_incompatibility_count: int
    payload_cover_count: int


@dataclass(frozen=True)
class _ActivationModel:
    model: gp.Model
    activation: gp.tupledict
    allowed_internal_arcs: tuple[Arc, ...]
    truck_time: dict[Arc, float]


@dataclass(frozen=True)
class StrengthenedArcProjectionDiagnostics:
    build_time_seconds: float
    solve_time_seconds: float
    node_count: float
    iteration_count: float
    variable_count: int
    constraint_count: int
    nonzero_count: int
    eligible_route_arc_count: int
    pruned_route_arc_count: int
    pair_incompatibility_count: int
    payload_cover_count: int


@dataclass(frozen=True)
class StrengthenedArcProjectionResult:
    feasible: bool
    status: str
    routes: tuple[Route, ...]
    diagnostics: StrengthenedArcProjectionDiagnostics


def discover_or_repair_instance(instance: InstanceData) -> ArcSwapRepairResult:
    if instance.config.generation_repair_mode == "none":
        gate = exact_feasibility_check(instance)
        if gate.feasible:
            return _unchanged_result(
                instance,
                gate.diagnostics.precheck,
                gate.status,
                gate.diagnostics,
                sum(stage.solve_time_seconds for stage in gate.diagnostics.stages),
            )
        return _unrepairable_model_result(
            instance,
            gate.diagnostics.precheck,
            gate.status,
            sum(stage.solve_time_seconds for stage in gate.diagnostics.stages),
            gate.diagnostics,
            None,
            0.0,
            0.0,
            0.0,
            0.0,
            0,
        )

    full_instance = _full_allowable_instance(instance)
    full_objective = _feasibility_objective(full_instance)
    full_precheck = exact_feasibility_precheck(full_instance, full_objective)
    if not full_precheck.passed:
        return _unrepairable_precheck_result(instance, full_precheck)

    discovery_start = time.time()
    discovery = exact_feasibility_check(
        instance,
        time_limit=instance.config.generation_feasibility_discovery_limit,
        allow_unresolved=True,
    )
    discovery_time = time.time() - discovery_start
    if discovery.feasible:
        return _unchanged_result(
            instance,
            full_precheck,
            discovery.status,
            discovery.diagnostics,
            discovery_time,
        )

    return _repair_instance(
        instance,
        full_instance,
        full_objective,
        full_precheck,
        discovery.status,
        discovery_time,
        discovery.diagnostics,
    )


def solve_strengthened_arc_projection_reference(
    instance: InstanceData,
) -> StrengthenedArcProjectionResult:
    full_instance = _full_allowable_instance(instance)
    objective = _feasibility_objective(full_instance)
    return _solve_strengthened_arc_projection(instance, full_instance, objective)


def _solve_strengthened_arc_projection(
    instance: InstanceData,
    full_instance: InstanceData,
    objective: ObjectiveData,
) -> StrengthenedArcProjectionResult:
    build_start = time.time()
    artifacts = _build_arc_swap_repair_model(
        instance,
        full_instance,
        objective,
        route_search=True,
    )
    build_time = time.time() - build_start
    solve_start = time.time()
    artifacts.model.optimize()
    solve_time = time.time() - solve_start
    if artifacts.model.SolCount > 0:
        routes = _decode_repair_routes(full_instance, objective, artifacts)
        status = "feasible"
        feasible = True
    elif artifacts.model.Status == GRB.INFEASIBLE:
        routes = tuple()
        status = "infeasible"
        feasible = False
    else:
        raise RuntimeError(f"unexpected strengthened arc projection status {artifacts.model.Status}")
    diagnostics = StrengthenedArcProjectionDiagnostics(
        build_time_seconds=build_time,
        solve_time_seconds=solve_time,
        node_count=float(artifacts.model.NodeCount),
        iteration_count=float(artifacts.model.IterCount),
        variable_count=int(artifacts.model.NumVars),
        constraint_count=int(artifacts.model.NumConstrs),
        nonzero_count=int(artifacts.model.NumNZs),
        eligible_route_arc_count=artifacts.eligible_route_arc_count,
        pruned_route_arc_count=artifacts.pruned_route_arc_count,
        pair_incompatibility_count=artifacts.pair_incompatibility_count,
        payload_cover_count=artifacts.payload_cover_count,
    )
    return StrengthenedArcProjectionResult(feasible, status, routes, diagnostics)


def _repair_instance(
    original: InstanceData,
    full_instance: InstanceData,
    repair_objective: ObjectiveData,
    full_precheck: FeasibilityPrecheckResult,
    discovery_status: str,
    discovery_time: float,
    discovery_diagnostics: FeasibilityGateDiagnostics,
) -> ArcSwapRepairResult:
    projection = _solve_strengthened_arc_projection(
        original,
        full_instance,
        repair_objective,
    )
    if not projection.feasible:
        return _unrepairable_strengthened_projection_result(
            original,
            full_precheck,
            discovery_status,
            discovery_time,
            discovery_diagnostics,
            projection.diagnostics,
        )

    required_internal = frozenset(
        arc
        for route in projection.routes
        for arc in route_internal_truck_arcs(full_instance, route)
    )
    activation_build_start = time.time()
    activation = _build_activation_completion_model(
        original,
        full_instance,
        required_internal,
    )
    activation_build_time = time.time() - activation_build_start
    activation_solve_time = 0.0
    activation_nodes = 0.0
    activation_iterations = 0.0
    rejected_patterns = 0

    while True:
        solve_start = time.time()
        activation.model.optimize()
        activation_solve_time += time.time() - solve_start
        activation_nodes += float(activation.model.NodeCount)
        activation_iterations += float(activation.model.IterCount)
        if activation.model.SolCount == 0:
            if activation.model.Status == GRB.INFEASIBLE:
                raise RuntimeError("activation completion is infeasible for the strengthened arc certificate")
            raise RuntimeError(f"unexpected activation-completion status {activation.model.Status}")

        selected_internal = frozenset(
            arc
            for arc in activation.allowed_internal_arcs
            if activation.activation[arc].X > 0.5
        )
        repaired = _instance_from_activation(original, selected_internal, activation.truck_time)
        validation = _validate_projected_routes(original, repaired, projection.routes)
        if validation.valid:
            return _strengthened_repaired_result(
                original,
                repaired,
                full_precheck,
                discovery_status,
                discovery_time,
                discovery_diagnostics,
                activation,
                activation_build_time,
                activation_solve_time,
                activation_nodes,
                activation_iterations,
                rejected_patterns,
                validation,
                projection.diagnostics,
            )

        _add_activation_no_good(activation, selected_internal, rejected_patterns)
        rejected_patterns += 1
        activation.model.reset()


def _build_activation_completion_model(
    original: InstanceData,
    full_instance: InstanceData,
    required_internal: frozenset[Arc],
) -> _ActivationModel:
    model = gp.Model("THVRPD_activation_completion")
    configure_gurobi_logging(model, None)
    model.Params.Threads = 0
    model.Params.SolutionLimit = 1
    allowed = allowable_internal_truck_arcs(original)
    if not required_internal.issubset(allowed):
        raise ValueError("route-column certificate uses an arc outside the activation universe")
    original_internal = frozenset(
        arc for arc in original.truck_arcs if _arc_class(original, arc) is not None
    )
    original_counts = truck_arc_class_counts(original, original.truck_arcs)
    activation = model.addVars(allowed, vtype=GRB.BINARY, name="activate")
    for arc in allowed:
        activation[arc].Start = 1.0 if arc in original_internal else 0.0
    for arc in required_internal:
        model.addConstr(activation[arc] == 1.0)
    for classification in ARC_CLASSES:
        model.addConstr(
            gp.quicksum(
                activation[arc]
                for arc in allowed
                if _arc_class(original, arc) == classification
            )
            == original_counts[classification]
        )
    model.setObjective(
        gp.quicksum(
            1.0 - activation[arc] if arc in original_internal else activation[arc]
            for arc in allowed
        ),
        GRB.MINIMIZE,
    )
    model.update()
    return _ActivationModel(
        model=model,
        activation=activation,
        allowed_internal_arcs=allowed,
        truck_time=dict(full_instance.truck_time),
    )


def _route_eligible_arcs(
    instance: InstanceData,
    objective: ObjectiveData,
) -> tuple[Arc, ...]:
    if any(instance.truck_time[arc] <= 0.0 for arc in instance.truck_arcs):
        raise ValueError("strictly positive truck times are required to remove MTZ ordering")
    graph = instance.truck_graph()
    source_lengths = nx.single_source_dijkstra_path_length(
        graph,
        instance.depot_source,
        weight="weight",
    )
    sink_lengths = nx.single_source_dijkstra_path_length(
        graph.reverse(copy=False),
        instance.depot_sink,
        weight="weight",
    )
    eligible = []
    for i, j in sorted(instance.truck_arcs):
        earliest_i = source_lengths.get(i, float("inf"))
        return_j = sink_lengths.get(j, float("inf"))
        travel = instance.truck_time[(i, j)]
        if earliest_i + travel + return_j > objective.bounds.route_time_ub + 1e-9:
            continue
        if j in instance.customers and earliest_i + travel > objective.bounds.service_ub[j] + 1e-9:
            continue
        eligible.append((i, j))
    return tuple(eligible)


def _arrival_bounds(
    instance: InstanceData,
    objective: ObjectiveData,
) -> tuple[dict[str, float], dict[str, float]]:
    graph = instance.truck_graph()
    lower = dict(
        nx.single_source_dijkstra_path_length(
            graph,
            instance.depot_source,
            weight="weight",
        )
    )
    to_sink = dict(
        nx.single_source_dijkstra_path_length(
            graph.reverse(copy=False),
            instance.depot_sink,
            weight="weight",
        )
    )
    lower = {node: lower.get(node, 0.0) for node in instance.nodes}
    upper = {
        node: (
            objective.bounds.route_time_ub - to_sink[node]
            if node in to_sink
            else objective.bounds.route_time_ub
        )
        for node in instance.nodes
    }
    lower[instance.depot_source] = 0.0
    upper[instance.depot_source] = 0.0
    for customer in instance.customers:
        upper[customer] = min(upper[customer], objective.bounds.service_ub[customer])
    for node in instance.nodes:
        if lower[node] > upper[node] + 1e-9:
            raise ValueError(f"inconsistent arrival bounds at {node}: {lower[node]} > {upper[node]}")
    return lower, upper


def _pairwise_route_incompatibilities(
    instance: InstanceData,
    objective: ObjectiveData,
) -> tuple[tuple[str, str], ...]:
    graph = build_transformed_graph(instance)
    zero_objective = replace(
        objective,
        coeffs=ObjectiveCoefficients(0.0, 0.0, 0.0, 0.0),
    )
    incompatible = []
    customers = tuple(sorted(instance.customers))
    for left_index, left in enumerate(customers):
        for right in customers[left_index + 1 :]:
            residual = frozenset((left, right))
            result = price_route(
                graph,
                zero_objective,
                residual,
                BranchRestrictions().with_together(left, right),
                PricingDuals(mu={left: 1.0, right: 1.0}, kappa=0.0),
                0,
                pricing_tolerance=0.0,
                use_standard_acceleration=True,
                batch_size=1,
                parallel_workers=1,
                pricing_worker_backend="thread",
                pricing_mode="closure",
            )
            if result.routes:
                continue
            if not result.diagnostics.exact_completion:
                raise RuntimeError("two-customer pricing ended without a route or exact closure")
            incompatible.append((left, right))
    return tuple(incompatible)


def _minimal_payload_covers(instance: InstanceData) -> tuple[tuple[str, ...], ...]:
    covers: set[tuple[str, ...]] = set()
    customers = tuple(sorted(instance.customers))
    for anchor in customers:
        ordered = (anchor,) + tuple(
            sorted(
                (customer for customer in customers if customer != anchor),
                key=lambda customer: (-instance.demand[customer], customer),
            )
        )
        selected = []
        total = 0.0
        for customer in ordered:
            selected.append(customer)
            total += instance.demand[customer]
            if total > instance.truck_payload + 1e-9:
                break
        if total <= instance.truck_payload + 1e-9:
            continue
        for customer in tuple(selected):
            if total - instance.demand[customer] > instance.truck_payload + 1e-9:
                selected.remove(customer)
                total -= instance.demand[customer]
        covers.add(tuple(sorted(selected)))
    return tuple(sorted(covers))


def _add_truck_signature_lex_constraints(
    model: gp.Model,
    instance: InstanceData,
    used,
    direct: dict[tuple[str, int], object],
    assigned: dict[tuple[str, int], object],
) -> None:
    customers = tuple(sorted(instance.customers))
    signature_length = 1 + 2 * len(customers)
    for truck in range(instance.num_trucks - 1):
        left = [used[truck]]
        left.extend(direct[customer, truck] for customer in customers)
        left.extend(assigned[customer, truck] for customer in customers)
        right = [used[truck + 1]]
        right.extend(direct[customer, truck + 1] for customer in customers)
        right.extend(assigned[customer, truck + 1] for customer in customers)
        differences = model.addVars(signature_length, vtype=GRB.BINARY, name=f"lex_diff[{truck}]")
        for position in range(signature_length):
            model.addConstr(differences[position] >= left[position] - right[position])
            model.addConstr(differences[position] >= right[position] - left[position])
            model.addConstr(differences[position] <= left[position] + right[position])
            model.addConstr(differences[position] <= 2.0 - left[position] - right[position])
            model.addConstr(
                right[position] - left[position]
                <= gp.quicksum(differences[prefix] for prefix in range(position))
            )


def _build_arc_swap_repair_model(
    original: InstanceData,
    full_instance: InstanceData,
    objective: ObjectiveData,
    *,
    route_search: bool = False,
    log_file: str | None = None,
    add_pair_incompatibilities: bool = True,
) -> _RepairModel:
    model = gp.Model("THVRPD_repair_route_search" if route_search else "THVRPD_arc_swap_repair")
    configure_gurobi_logging(model, log_file)
    model.Params.Threads = 0
    model.Params.SolutionLimit = 1
    trucks = range(original.num_trucks)
    physical = original.customers + original.hubs
    allowed_internal = allowable_internal_truck_arcs(original)
    universe_arcs = tuple(sorted(full_instance.truck_arcs))
    all_arcs = (
        _route_eligible_arcs(full_instance, objective)
        if route_search
        else universe_arcs
    )
    route_arc_set = set(all_arcs)
    original_internal = frozenset(arc for arc in original.truck_arcs if _arc_class(original, arc) is not None)
    original_counts = truck_arc_class_counts(original, original.truck_arcs)

    activation = model.addVars(allowed_internal, vtype=GRB.BINARY, name="activate")
    x = model.addVars(all_arcs, trucks, vtype=GRB.BINARY, name="x")
    y = model.addVars(original.drone_arcs, trucks, vtype=GRB.BINARY, name="y")
    used = model.addVars(trucks, vtype=GRB.BINARY, name="used")
    pad_active = model.addVars(original.hubs, trucks, vtype=GRB.BINARY, name="pad")

    route_time_ub = objective.bounds.route_time_ub
    max_drone_trip = max(original.drone_trip_time.values(), default=0.0)
    hub_wait_ub = {
        hub: max(
            (original.drone_trip_time[(h, customer)] for h, customer in original.drone_arcs if h == hub),
            default=0.0,
        )
        for hub in original.hubs
    }
    wait = model.addVars(
        original.hubs,
        trucks,
        lb=0.0,
        ub={(hub, truck): hub_wait_ub[hub] for hub in original.hubs for truck in trucks},
        name="wait",
    )
    arrival_lb, arrival_ub = _arrival_bounds(full_instance, objective)
    arrive = model.addVars(
        original.nodes,
        trucks,
        lb={(node, truck): arrival_lb[node] for node in original.nodes for truck in trucks},
        ub={(node, truck): arrival_ub[node] for node in original.nodes for truck in trucks},
        name="arrive",
    )
    service = model.addVars(original.customers, lb=0.0, name="service")
    payload = model.addVars(original.nodes, trucks, lb=0.0, ub=original.truck_payload, name="payload")

    outgoing = {node: [(i, j) for i, j in all_arcs if i == node] for node in original.nodes}
    incoming = {node: [(i, j) for i, j in all_arcs if j == node] for node in original.nodes}
    drone_from = {
        hub: [(h, customer) for h, customer in original.drone_arcs if h == hub]
        for hub in original.hubs
    }
    drone_to = {
        customer: [(hub, c) for hub, c in original.drone_arcs if c == customer]
        for customer in original.customers
    }
    direct = {
        (customer, truck): gp.quicksum(x[i, j, truck] for i, j in incoming[customer])
        for customer in original.customers
        for truck in trucks
    }
    assigned = {
        (customer, truck): (
            direct[customer, truck]
            + gp.quicksum(y[hub, c, truck] for hub, c in drone_to[customer])
        )
        for customer in original.customers
        for truck in trucks
    }
    incompatible_pairs = (
        _pairwise_route_incompatibilities(full_instance, objective)
        if route_search and add_pair_incompatibilities
        else tuple()
    )
    payload_covers = _minimal_payload_covers(original) if route_search else tuple()

    for arc in allowed_internal:
        if not route_search:
            activation[arc].Start = 1.0 if arc in original_internal else 0.0
        for truck in trucks:
            if arc in route_arc_set:
                model.addConstr(x[arc[0], arc[1], truck] <= activation[arc])
        if route_search:
            if arc in route_arc_set:
                model.addConstr(
                    activation[arc]
                    <= gp.quicksum(x[arc[0], arc[1], truck] for truck in trucks)
                )
            else:
                model.addConstr(activation[arc] == 0.0)
    for classification in ARC_CLASSES:
        activation_count = gp.quicksum(
            activation[arc]
            for arc in allowed_internal
            if _arc_class(original, arc) == classification
        )
        if route_search:
            model.addConstr(activation_count <= original_counts[classification])
        else:
            model.addConstr(activation_count == original_counts[classification])

    max_direct_demand = max(original.demand[customer] for customer in original.customers)
    max_pad_block_demand = max(
        (
            min(
                sum(original.demand[customer] for h, customer in original.drone_arcs if h == hub),
                original.drones_per_truck * original.drone_payload,
                original.truck_payload,
            )
            for hub in original.hubs
        ),
        default=0.0,
    )
    big_m_wait = max_drone_trip

    for truck in trucks:
        model.addConstr(
            gp.quicksum(x[i, j, truck] for i, j in outgoing[original.depot_source]) == used[truck]
        )
        model.addConstr(
            gp.quicksum(x[i, j, truck] for i, j in incoming[original.depot_sink]) == used[truck]
        )
        for node in physical:
            out_expr = gp.quicksum(x[i, j, truck] for i, j in outgoing[node])
            in_expr = gp.quicksum(x[i, j, truck] for i, j in incoming[node])
            model.addConstr(out_expr == in_expr)
            model.addConstr(in_expr <= 1.0)
        for hub in original.hubs:
            hub_visit = gp.quicksum(x[i, j, truck] for i, j in incoming[hub])
            launched = gp.quicksum(y[h, customer, truck] for h, customer in drone_from[hub])
            model.addConstr(pad_active[hub, truck] == hub_visit)
            model.addConstr(launched <= original.drones_per_truck * pad_active[hub, truck])
            for h, customer in drone_from[hub]:
                model.addConstr(
                    wait[hub, truck] >= original.drone_trip_time[(hub, customer)] * y[h, customer, truck]
                )
            model.addConstr(wait[hub, truck] <= big_m_wait * launched)
        model.addConstr(arrive[original.depot_source, truck] == 0.0)
        model.addConstr(payload[original.depot_source, truck] == 0.0)

    _add_truck_signature_lex_constraints(model, original, used, direct, assigned)

    for customer in original.customers:
        model.addConstr(gp.quicksum(assigned[customer, truck] for truck in trucks) == 1.0)

    minimum_trucks = ceil(
        sum(original.demand[customer] for customer in original.customers)
        / original.truck_payload
    )
    model.addConstr(gp.quicksum(used[truck] for truck in trucks) >= minimum_trucks)

    for truck in trucks:
        model.addConstr(
            gp.quicksum(
                original.demand[customer] * assigned[customer, truck]
                for customer in original.customers
            )
            <= original.truck_payload * used[truck]
        )
        model.addConstr(
            gp.quicksum(assigned[customer, truck] for customer in original.customers) >= used[truck]
        )
        for left, right in incompatible_pairs:
            model.addConstr(assigned[left, truck] + assigned[right, truck] <= 1.0)
        for cover in payload_covers:
            model.addConstr(
                gp.quicksum(assigned[customer, truck] for customer in cover)
                <= len(cover) - 1
            )
        model.addConstr(
            gp.quicksum(x[i, j, truck] for i, j in all_arcs)
            == used[truck]
            + gp.quicksum(
                x[i, j, truck]
                for node in physical
                for i, j in incoming[node]
            )
        )
        for i, j in all_arcs:
            wait_expr = wait[i, truck] if i in original.hubs else 0.0
            travel_time = full_instance.truck_time[(i, j)]
            wait_upper = hub_wait_ub[i] if i in original.hubs else 0.0
            lower_m = max(0.0, arrival_ub[i] + wait_upper + travel_time - arrival_lb[j])
            upper_m = max(0.0, arrival_ub[j] - arrival_lb[i] - travel_time)
            model.addConstr(
                arrive[j, truck]
                >= arrive[i, truck] + wait_expr + travel_time
                - lower_m * (1 - x[i, j, truck])
            )
            model.addConstr(
                arrive[j, truck]
                <= arrive[i, truck] + wait_expr + travel_time
                + upper_m * (1 - x[i, j, truck])
            )
            if j in original.customers:
                load_m = original.truck_payload + original.demand[j]
                model.addConstr(
                    payload[j, truck]
                    >= payload[i, truck] + original.demand[j] - load_m * (1 - x[i, j, truck])
                )
            elif j in original.hubs:
                launched_load = gp.quicksum(
                    original.demand[customer] * y[h, customer, truck]
                    for h, customer in drone_from[j]
                )
                model.addConstr(
                    payload[j, truck]
                    >= payload[i, truck] + launched_load
                    - (original.truck_payload + max_pad_block_demand) * (1 - x[i, j, truck])
                )
        for customer in original.customers:
            service_m = max(0.0, arrival_ub[customer] - objective.bounds.arrival_lb[customer])
            model.addConstr(
                service[customer]
                >= arrive[customer, truck] - service_m * (1 - direct[customer, truck])
            )
        for hub, customer in original.drone_arcs:
            drone_m = max(
                0.0,
                arrival_ub[hub]
                + original.drone_time[(hub, customer)]
                - objective.bounds.arrival_lb[customer],
            )
            model.addConstr(
                service[customer]
                >= arrive[hub, truck] + original.drone_time[(hub, customer)]
                - drone_m * (1 - y[hub, customer, truck])
            )

    for customer in original.customers:
        model.addConstr(service[customer] >= objective.bounds.arrival_lb[customer])
        model.addConstr(service[customer] <= objective.bounds.service_ub[customer])

    if route_search:
        model.setObjective(0.0, GRB.MINIMIZE)
    else:
        hamming = gp.quicksum(
            1.0 - activation[arc] if arc in original_internal else activation[arc]
            for arc in allowed_internal
        )
        model.setObjective(hamming, GRB.MINIMIZE)
    model.update()
    return _RepairModel(
        model=model,
        activation=activation,
        x=x,
        y=y,
        used=used,
        allowed_internal_arcs=allowed_internal,
        all_truck_arcs=all_arcs,
        truck_time=dict(full_instance.truck_time),
        eligible_route_arc_count=len(all_arcs),
        pruned_route_arc_count=len(universe_arcs) - len(all_arcs),
        pair_incompatibility_count=len(incompatible_pairs),
        payload_cover_count=len(payload_covers),
    )


def _full_allowable_instance(instance: InstanceData) -> InstanceData:
    internal = frozenset(allowable_internal_truck_arcs(instance))
    fixed = frozenset(
        arc
        for arc in instance.truck_arcs
        if arc[0] == instance.depot_source or arc[1] == instance.depot_sink
    )
    original_internal = frozenset(arc for arc in instance.truck_arcs if arc not in fixed)
    if not original_internal.issubset(internal):
        raise ValueError("original graph contains an internal truck arc outside the repair universe")
    arcs = fixed | internal
    truck_time = {
        arc: instance.truck_time[arc] if arc in fixed else _truck_travel_time(instance, arc)
        for arc in arcs
    }
    savings = _drone_arc_savings(
        instance.depot_source,
        instance.customers,
        instance.hubs,
        set(arcs),
        truck_time,
        set(instance.drone_arcs),
        instance.drone_time,
    )
    return replace(
        instance,
        truck_arcs=arcs,
        truck_time=truck_time,
        drone_arc_saving={arc: savings[arc] for arc in sorted(instance.drone_arcs)},
    )


def _instance_from_activation(
    original: InstanceData,
    selected_internal: frozenset[Arc],
    universe_truck_time: dict[Arc, float],
) -> InstanceData:
    fixed = frozenset(
        arc
        for arc in original.truck_arcs
        if arc[0] == original.depot_source or arc[1] == original.depot_sink
    )
    arcs = fixed | selected_internal
    truck_time = {arc: universe_truck_time[arc] for arc in arcs}
    savings = _drone_arc_savings(
        original.depot_source,
        original.customers,
        original.hubs,
        set(arcs),
        truck_time,
        set(original.drone_arcs),
        original.drone_time,
    )
    return replace(
        original,
        truck_arcs=arcs,
        truck_time=truck_time,
        drone_arc_saving={arc: savings[arc] for arc in sorted(original.drone_arcs)},
    )


def _validate_repair_solution(
    original: InstanceData,
    repaired: InstanceData,
    artifacts: _RepairModel,
) -> RepairValidation:
    issues: list[str] = []
    original_fixed = {
        arc
        for arc in original.truck_arcs
        if arc[0] == original.depot_source or arc[1] == original.depot_sink
    }
    repaired_fixed = {
        arc
        for arc in repaired.truck_arcs
        if arc[0] == repaired.depot_source or arc[1] == repaired.depot_sink
    }
    if repaired_fixed != original_fixed:
        issues.append("depot_arc_set_changed")
    if truck_arc_class_counts(original, repaired.truck_arcs) != truck_arc_class_counts(original, original.truck_arcs):
        issues.append("arc_class_counts_changed")
    mandatory = set(original.mandatory_drone_customers)
    if any(left in mandatory or right in mandatory for left, right in repaired.truck_arcs):
        issues.append("truck_arc_incident_to_mandatory_drone_customer")
    for field_name in (
        "config",
        "depot_source",
        "depot_sink",
        "customers",
        "hubs",
        "nodes",
        "drone_arcs",
        "drone_time",
        "drone_trip_time",
        "demand",
        "locations",
        "mandatory_drone_customers",
    ):
        if getattr(repaired, field_name) != getattr(original, field_name):
            issues.append(f"immutable_field_changed:{field_name}")

    try:
        objective = _feasibility_objective(repaired)
        routes = _decode_repair_routes(repaired, objective, artifacts)
    except ValueError as exc:
        return RepairValidation(valid=False, issues=tuple(issues + [str(exc)]), routes=tuple())

    coverage = Counter(customer for route in routes for customer in route.served)
    expected = Counter({customer: 1 for customer in repaired.customers})
    if coverage != expected:
        issues.append("route_certificate_customer_coverage")
    if len(routes) > repaired.num_trucks:
        issues.append("route_certificate_truck_count")
    records = tuple(_route_certificate_record(repaired, route) for route in routes)
    return RepairValidation(valid=not issues, issues=tuple(issues), routes=records)


def _validate_projected_routes(
    original: InstanceData,
    repaired: InstanceData,
    projected_routes: tuple[Route, ...],
) -> RepairValidation:
    issues = _repair_invariant_issues(original, repaired)
    try:
        objective = _feasibility_objective(repaired)
        graph = build_transformed_graph(repaired)
        routes = tuple(
            route_from_path(index + 1, route.path, graph, objective)
            for index, route in enumerate(sorted(projected_routes, key=lambda route: route.path))
        )
    except ValueError as exc:
        return RepairValidation(False, tuple(issues + [str(exc)]), tuple())
    coverage = Counter(customer for route in routes for customer in route.served)
    if coverage != Counter({customer: 1 for customer in repaired.customers}):
        issues.append("route_certificate_customer_coverage")
    if len(routes) > repaired.num_trucks:
        issues.append("route_certificate_truck_count")
    records = tuple(_route_certificate_record(repaired, route) for route in routes)
    return RepairValidation(not issues, tuple(issues), records)


def _repair_invariant_issues(original: InstanceData, repaired: InstanceData) -> list[str]:
    issues: list[str] = []
    original_fixed = {
        arc
        for arc in original.truck_arcs
        if arc[0] == original.depot_source or arc[1] == original.depot_sink
    }
    repaired_fixed = {
        arc
        for arc in repaired.truck_arcs
        if arc[0] == repaired.depot_source or arc[1] == repaired.depot_sink
    }
    if repaired_fixed != original_fixed:
        issues.append("depot_arc_set_changed")
    if truck_arc_class_counts(original, repaired.truck_arcs) != truck_arc_class_counts(original, original.truck_arcs):
        issues.append("arc_class_counts_changed")
    mandatory = set(original.mandatory_drone_customers)
    if any(left in mandatory or right in mandatory for left, right in repaired.truck_arcs):
        issues.append("truck_arc_incident_to_mandatory_drone_customer")
    for field_name in (
        "config",
        "depot_source",
        "depot_sink",
        "customers",
        "hubs",
        "nodes",
        "drone_arcs",
        "drone_time",
        "drone_trip_time",
        "demand",
        "locations",
        "mandatory_drone_customers",
    ):
        if getattr(repaired, field_name) != getattr(original, field_name):
            issues.append(f"immutable_field_changed:{field_name}")
    return issues


def _decode_repair_routes(
    instance: InstanceData,
    objective: ObjectiveData,
    artifacts: _RepairModel,
) -> tuple[Route, ...]:
    graph = build_transformed_graph(instance)
    outgoing = {
        node: tuple(arc for arc in artifacts.all_truck_arcs if arc[0] == node)
        for node in instance.nodes
    }
    routes: list[Route] = []
    for truck in range(instance.num_trucks):
        if artifacts.used[truck].X <= 0.5:
            continue
        physical_path = [instance.depot_source]
        current = instance.depot_source
        while current != instance.depot_sink:
            selected = [
                right
                for left, right in outgoing[current]
                if artifacts.x[left, right, truck].X > 0.5
            ]
            if len(selected) != 1:
                raise ValueError(f"truck {truck} has {len(selected)} selected successors at {current}")
            current = selected[0]
            if current in physical_path:
                raise ValueError(f"truck {truck} route contains a physical cycle at {current}")
            physical_path.append(current)

        transformed: list[str] = [physical_path[0]]
        for node in physical_path[1:-1]:
            transformed.append(node)
            if node in instance.hubs:
                launched = [
                    customer
                    for customer in instance.customers
                    if (node, customer) in instance.drone_arcs
                    and artifacts.y[node, customer, truck].X > 0.5
                ]
                transformed.extend(duplicate_node(customer) for customer in launched)
        transformed.append(instance.depot_sink)
        routes.append(route_from_path(truck + 1, tuple(transformed), graph, objective))
    return tuple(routes)


def _route_certificate_record(instance: InstanceData, route: Route) -> dict[str, object]:
    return {
        "route_id": route.id,
        "path": list(route.path),
        "truck_path": list(route.truck_path),
        "drone_blocks": {hub: list(customers) for hub, customers in sorted(route.drone_blocks.items())},
        "served": sorted(route.served),
        "truck_served": sorted(route.truck_served),
        "drone_served": sorted(route.drone_served),
        "service_times": {customer: route.service_times[customer] for customer in sorted(route.service_times)},
        "return_time": route.return_time,
        "payload": sum(instance.demand[customer] for customer in route.served),
    }


def _complete_activation_start(
    original: InstanceData,
    route_search: _RepairModel,
) -> frozenset[Arc]:
    route_arcs = {
        arc
        for arc in route_search.allowed_internal_arcs
        if route_search.activation[arc].X > 0.5
    }
    original_internal = {
        arc for arc in original.truck_arcs if _arc_class(original, arc) is not None
    }
    target_counts = truck_arc_class_counts(original, original.truck_arcs)
    selected = set(route_arcs)
    for classification in ARC_CLASSES:
        current = sum(_arc_class(original, arc) == classification for arc in selected)
        needed = target_counts[classification] - current
        if needed < 0:
            raise RuntimeError(f"repair route uses too many {classification} arcs")
        candidates = [
            arc
            for arc in sorted(original_internal)
            if _arc_class(original, arc) == classification and arc not in selected
        ]
        candidates.extend(
            arc
            for arc in route_search.allowed_internal_arcs
            if _arc_class(original, arc) == classification
            and arc not in original_internal
            and arc not in selected
        )
        if len(candidates) < needed:
            raise RuntimeError(f"not enough allowable {classification} arcs to preserve the realized count")
        selected.update(candidates[:needed])
    return frozenset(selected)


def _seed_repair_model(
    route_search: _RepairModel,
    repair: _RepairModel,
    selected_internal: frozenset[Arc],
) -> None:
    for arc in repair.allowed_internal_arcs:
        repair.activation[arc].Start = 1.0 if arc in selected_internal else 0.0
    for key in repair.x.keys():
        repair.x[key].Start = route_search.x[key].X
    for key in repair.y.keys():
        repair.y[key].Start = route_search.y[key].X
    for key in repair.used.keys():
        repair.used[key].Start = route_search.used[key].X


def _add_activation_no_good(
    artifacts: _RepairModel | _ActivationModel,
    selected_internal: frozenset[Arc],
    index: int,
) -> None:
    artifacts.model.addConstr(
        gp.quicksum(1.0 - artifacts.activation[arc] for arc in selected_internal)
        + gp.quicksum(
            artifacts.activation[arc]
            for arc in artifacts.allowed_internal_arcs
            if arc not in selected_internal
        )
        >= 1.0,
        name=f"repair_pattern_no_good[{index}]",
    )
    artifacts.model.update()


def _strengthened_repaired_result(
    original: InstanceData,
    repaired: InstanceData,
    full_precheck: FeasibilityPrecheckResult,
    discovery_status: str,
    discovery_time: float,
    discovery_diagnostics: FeasibilityGateDiagnostics,
    activation: _ActivationModel,
    activation_build_time: float,
    activation_solve_time: float,
    activation_nodes: float,
    activation_iterations: float,
    rejected_patterns: int,
    validation: RepairValidation,
    projection: StrengthenedArcProjectionDiagnostics,
) -> ArcSwapRepairResult:
    original_internal = {arc for arc in original.truck_arcs if _arc_class(original, arc) is not None}
    repaired_internal = {arc for arc in repaired.truck_arcs if _arc_class(repaired, arc) is not None}
    original_objective = _feasibility_objective(original)
    repaired_objective = _feasibility_objective(repaired)
    diagnostics = ArcSwapRepairDiagnostics(
        full_universe_precheck=full_precheck,
        original_discovery_status=discovery_status,
        original_discovery_time_seconds=discovery_time,
        original_discovery_nodes=_last_stage_value(discovery_diagnostics, "node_count"),
        original_discovery_iterations=_last_stage_value(discovery_diagnostics, "iteration_count"),
        original_discovery_fallback_used=discovery_diagnostics.fallback_used,
        original_discovery_diagnostics=discovery_diagnostics,
        repair_invoked=True,
        projection_type="strengthened_arc",
        strengthened_arc_projection=projection,
        original_arc_counts=truck_arc_class_counts(original, original.truck_arcs),
        repaired_arc_counts=truck_arc_class_counts(repaired, repaired.truck_arcs),
        added_arcs=tuple(sorted(repaired_internal - original_internal)),
        removed_arcs=tuple(sorted(original_internal - repaired_internal)),
        hamming_edit_count=len(repaired_internal.symmetric_difference(original_internal)),
        route_search_variable_count=projection.variable_count,
        route_search_constraint_count=projection.constraint_count,
        route_search_nonzero_count=projection.nonzero_count,
        route_search_build_time_seconds=projection.build_time_seconds,
        route_search_solve_time_seconds=projection.solve_time_seconds,
        route_search_node_count=projection.node_count,
        route_search_iteration_count=projection.iteration_count,
        repair_variable_count=int(activation.model.NumVars),
        repair_constraint_count=int(activation.model.NumConstrs),
        repair_nonzero_count=int(activation.model.NumNZs),
        repair_build_time_seconds=activation_build_time,
        repair_solve_time_seconds=projection.solve_time_seconds + activation_solve_time,
        repair_node_count=activation_nodes,
        repair_iteration_count=activation_iterations,
        rejected_pattern_count=rejected_patterns,
        certificate_routes=validation.routes,
        certificate_validation=validation,
        original_graph_fingerprint=instance_physical_fingerprint(original),
        repaired_graph_fingerprint=instance_physical_fingerprint(repaired),
        original_deadline_fingerprint=_deadline_fingerprint(original_objective),
        repaired_deadline_fingerprint=_deadline_fingerprint(repaired_objective),
        original_benchmark_fingerprint=_benchmark_fingerprint(original_objective),
        repaired_benchmark_fingerprint=_benchmark_fingerprint(repaired_objective),
    )
    return ArcSwapRepairResult(True, "repaired", repaired, diagnostics)


def _unrepairable_strengthened_projection_result(
    original: InstanceData,
    full_precheck: FeasibilityPrecheckResult,
    discovery_status: str,
    discovery_time: float,
    discovery_diagnostics: FeasibilityGateDiagnostics,
    projection: StrengthenedArcProjectionDiagnostics,
) -> ArcSwapRepairResult:
    objective = _feasibility_objective(original)
    validation = RepairValidation(False, ("strengthened_arc_projection_infeasible",), tuple())
    diagnostics = ArcSwapRepairDiagnostics(
        full_universe_precheck=full_precheck,
        original_discovery_status=discovery_status,
        original_discovery_time_seconds=discovery_time,
        original_discovery_nodes=_last_stage_value(discovery_diagnostics, "node_count"),
        original_discovery_iterations=_last_stage_value(discovery_diagnostics, "iteration_count"),
        original_discovery_fallback_used=discovery_diagnostics.fallback_used,
        original_discovery_diagnostics=discovery_diagnostics,
        repair_invoked=True,
        projection_type="strengthened_arc",
        strengthened_arc_projection=projection,
        original_arc_counts=truck_arc_class_counts(original, original.truck_arcs),
        repaired_arc_counts={},
        added_arcs=tuple(),
        removed_arcs=tuple(),
        hamming_edit_count=0,
        route_search_variable_count=projection.variable_count,
        route_search_constraint_count=projection.constraint_count,
        route_search_nonzero_count=projection.nonzero_count,
        route_search_build_time_seconds=projection.build_time_seconds,
        route_search_solve_time_seconds=projection.solve_time_seconds,
        route_search_node_count=projection.node_count,
        route_search_iteration_count=projection.iteration_count,
        repair_variable_count=0,
        repair_constraint_count=0,
        repair_nonzero_count=0,
        repair_build_time_seconds=0.0,
        repair_solve_time_seconds=projection.solve_time_seconds,
        repair_node_count=0.0,
        repair_iteration_count=0.0,
        rejected_pattern_count=0,
        certificate_routes=tuple(),
        certificate_validation=validation,
        original_graph_fingerprint=instance_physical_fingerprint(original),
        repaired_graph_fingerprint="",
        original_deadline_fingerprint=_deadline_fingerprint(objective),
        repaired_deadline_fingerprint="",
        original_benchmark_fingerprint=_benchmark_fingerprint(objective),
        repaired_benchmark_fingerprint="",
    )
    return ArcSwapRepairResult(False, "unrepairable", None, diagnostics)


def _repaired_result(
    original: InstanceData,
    repaired: InstanceData,
    full_precheck: FeasibilityPrecheckResult,
    discovery_status: str,
    discovery_time: float,
    discovery_diagnostics: FeasibilityGateDiagnostics,
    artifacts: _RepairModel,
    build_time: float,
    solve_time: float,
    node_count: float,
    iteration_count: float,
    rejected_patterns: int,
    validation: RepairValidation,
    route_search: _RepairModel,
    route_search_build_time: float,
    route_search_solve_time: float,
    route_search_nodes: float,
    route_search_iterations: float,
) -> ArcSwapRepairResult:
    original_internal = {arc for arc in original.truck_arcs if _arc_class(original, arc) is not None}
    repaired_internal = {arc for arc in repaired.truck_arcs if _arc_class(repaired, arc) is not None}
    original_objective = _feasibility_objective(original)
    repaired_objective = _feasibility_objective(repaired)
    diagnostics = ArcSwapRepairDiagnostics(
        full_universe_precheck=full_precheck,
        original_discovery_status=discovery_status,
        original_discovery_time_seconds=discovery_time,
        original_discovery_nodes=_last_stage_value(discovery_diagnostics, "node_count"),
        original_discovery_iterations=_last_stage_value(discovery_diagnostics, "iteration_count"),
        original_discovery_fallback_used=discovery_diagnostics.fallback_used,
        original_discovery_diagnostics=discovery_diagnostics,
        repair_invoked=True,
        projection_type="arc_milp_reference",
        strengthened_arc_projection=None,
        original_arc_counts=truck_arc_class_counts(original, original.truck_arcs),
        repaired_arc_counts=truck_arc_class_counts(repaired, repaired.truck_arcs),
        added_arcs=tuple(sorted(repaired_internal - original_internal)),
        removed_arcs=tuple(sorted(original_internal - repaired_internal)),
        hamming_edit_count=len(repaired_internal.symmetric_difference(original_internal)),
        route_search_variable_count=int(route_search.model.NumVars),
        route_search_constraint_count=int(route_search.model.NumConstrs),
        route_search_nonzero_count=int(route_search.model.NumNZs),
        route_search_build_time_seconds=route_search_build_time,
        route_search_solve_time_seconds=route_search_solve_time,
        route_search_node_count=route_search_nodes,
        route_search_iteration_count=route_search_iterations,
        repair_variable_count=int(artifacts.model.NumVars),
        repair_constraint_count=int(artifacts.model.NumConstrs),
        repair_nonzero_count=int(artifacts.model.NumNZs),
        repair_build_time_seconds=build_time,
        repair_solve_time_seconds=solve_time,
        repair_node_count=node_count,
        repair_iteration_count=iteration_count,
        rejected_pattern_count=rejected_patterns,
        certificate_routes=validation.routes,
        certificate_validation=validation,
        original_graph_fingerprint=instance_physical_fingerprint(original),
        repaired_graph_fingerprint=instance_physical_fingerprint(repaired),
        original_deadline_fingerprint=_deadline_fingerprint(original_objective),
        repaired_deadline_fingerprint=_deadline_fingerprint(repaired_objective),
        original_benchmark_fingerprint=_benchmark_fingerprint(original_objective),
        repaired_benchmark_fingerprint=_benchmark_fingerprint(repaired_objective),
    )
    return ArcSwapRepairResult(True, "repaired", repaired, diagnostics)


def _unchanged_result(
    instance: InstanceData,
    full_precheck: FeasibilityPrecheckResult,
    discovery_status: str,
    discovery_diagnostics: FeasibilityGateDiagnostics,
    discovery_time: float,
) -> ArcSwapRepairResult:
    objective = _feasibility_objective(instance)
    fingerprint = instance_physical_fingerprint(instance)
    validation = RepairValidation(True, tuple(), tuple())
    diagnostics = ArcSwapRepairDiagnostics(
        full_universe_precheck=full_precheck,
        original_discovery_status=discovery_status,
        original_discovery_time_seconds=discovery_time,
        original_discovery_nodes=_last_stage_value(discovery_diagnostics, "node_count"),
        original_discovery_iterations=_last_stage_value(discovery_diagnostics, "iteration_count"),
        original_discovery_fallback_used=discovery_diagnostics.fallback_used,
        original_discovery_diagnostics=discovery_diagnostics,
        repair_invoked=False,
        projection_type="unchanged",
        strengthened_arc_projection=None,
        original_arc_counts=truck_arc_class_counts(instance, instance.truck_arcs),
        repaired_arc_counts=truck_arc_class_counts(instance, instance.truck_arcs),
        added_arcs=tuple(),
        removed_arcs=tuple(),
        hamming_edit_count=0,
        route_search_variable_count=0,
        route_search_constraint_count=0,
        route_search_nonzero_count=0,
        route_search_build_time_seconds=0.0,
        route_search_solve_time_seconds=0.0,
        route_search_node_count=0.0,
        route_search_iteration_count=0.0,
        repair_variable_count=0,
        repair_constraint_count=0,
        repair_nonzero_count=0,
        repair_build_time_seconds=0.0,
        repair_solve_time_seconds=0.0,
        repair_node_count=0.0,
        repair_iteration_count=0.0,
        rejected_pattern_count=0,
        certificate_routes=tuple(),
        certificate_validation=validation,
        original_graph_fingerprint=fingerprint,
        repaired_graph_fingerprint=fingerprint,
        original_deadline_fingerprint=_deadline_fingerprint(objective),
        repaired_deadline_fingerprint=_deadline_fingerprint(objective),
        original_benchmark_fingerprint=_benchmark_fingerprint(objective),
        repaired_benchmark_fingerprint=_benchmark_fingerprint(objective),
    )
    return ArcSwapRepairResult(True, "feasible_unchanged", instance, diagnostics)


def _unrepairable_precheck_result(
    instance: InstanceData,
    precheck: FeasibilityPrecheckResult,
) -> ArcSwapRepairResult:
    objective = _feasibility_objective(_full_allowable_instance(instance))
    fingerprint = instance_physical_fingerprint(instance)
    validation = RepairValidation(False, (precheck.witness or "full_universe_precheck_failed",), tuple())
    diagnostics = ArcSwapRepairDiagnostics(
        full_universe_precheck=precheck,
        original_discovery_status="not_run",
        original_discovery_time_seconds=0.0,
        original_discovery_nodes=0.0,
        original_discovery_iterations=0.0,
        original_discovery_fallback_used=False,
        original_discovery_diagnostics=None,
        repair_invoked=False,
        projection_type="not_run",
        strengthened_arc_projection=None,
        original_arc_counts=truck_arc_class_counts(instance, instance.truck_arcs),
        repaired_arc_counts={},
        added_arcs=tuple(),
        removed_arcs=tuple(),
        hamming_edit_count=0,
        route_search_variable_count=0,
        route_search_constraint_count=0,
        route_search_nonzero_count=0,
        route_search_build_time_seconds=0.0,
        route_search_solve_time_seconds=0.0,
        route_search_node_count=0.0,
        route_search_iteration_count=0.0,
        repair_variable_count=0,
        repair_constraint_count=0,
        repair_nonzero_count=0,
        repair_build_time_seconds=0.0,
        repair_solve_time_seconds=0.0,
        repair_node_count=0.0,
        repair_iteration_count=0.0,
        rejected_pattern_count=0,
        certificate_routes=tuple(),
        certificate_validation=validation,
        original_graph_fingerprint=fingerprint,
        repaired_graph_fingerprint="",
        original_deadline_fingerprint=_deadline_fingerprint(objective),
        repaired_deadline_fingerprint="",
        original_benchmark_fingerprint=_benchmark_fingerprint(objective),
        repaired_benchmark_fingerprint="",
    )
    return ArcSwapRepairResult(False, "infeasible_precheck", None, diagnostics)


def _unrepairable_model_result(
    original: InstanceData,
    full_precheck: FeasibilityPrecheckResult,
    discovery_status: str,
    discovery_time: float,
    discovery_diagnostics: FeasibilityGateDiagnostics,
    artifacts: _RepairModel | None,
    build_time: float,
    solve_time: float,
    node_count: float,
    iteration_count: float,
    rejected_patterns: int,
    *,
    route_search: _RepairModel | None = None,
    route_search_build_time: float = 0.0,
    route_search_solve_time: float = 0.0,
    route_search_nodes: float = 0.0,
    route_search_iterations: float = 0.0,
) -> ArcSwapRepairResult:
    objective = _feasibility_objective(original)
    fingerprint = instance_physical_fingerprint(original)
    validation = RepairValidation(False, ("repair_model_infeasible",), tuple())
    diagnostics = ArcSwapRepairDiagnostics(
        full_universe_precheck=full_precheck,
        original_discovery_status=discovery_status,
        original_discovery_time_seconds=discovery_time,
        original_discovery_nodes=_last_stage_value(discovery_diagnostics, "node_count"),
        original_discovery_iterations=_last_stage_value(discovery_diagnostics, "iteration_count"),
        original_discovery_fallback_used=discovery_diagnostics.fallback_used,
        original_discovery_diagnostics=discovery_diagnostics,
        repair_invoked=artifacts is not None,
        projection_type="arc_milp_reference" if artifacts is not None else "not_run",
        strengthened_arc_projection=None,
        original_arc_counts=truck_arc_class_counts(original, original.truck_arcs),
        repaired_arc_counts={},
        added_arcs=tuple(),
        removed_arcs=tuple(),
        hamming_edit_count=0,
        route_search_variable_count=0 if route_search is None else int(route_search.model.NumVars),
        route_search_constraint_count=0 if route_search is None else int(route_search.model.NumConstrs),
        route_search_nonzero_count=0 if route_search is None else int(route_search.model.NumNZs),
        route_search_build_time_seconds=route_search_build_time,
        route_search_solve_time_seconds=route_search_solve_time,
        route_search_node_count=route_search_nodes,
        route_search_iteration_count=route_search_iterations,
        repair_variable_count=0 if artifacts is None else int(artifacts.model.NumVars),
        repair_constraint_count=0 if artifacts is None else int(artifacts.model.NumConstrs),
        repair_nonzero_count=0 if artifacts is None else int(artifacts.model.NumNZs),
        repair_build_time_seconds=build_time,
        repair_solve_time_seconds=solve_time,
        repair_node_count=node_count,
        repair_iteration_count=iteration_count,
        rejected_pattern_count=rejected_patterns,
        certificate_routes=tuple(),
        certificate_validation=validation,
        original_graph_fingerprint=fingerprint,
        repaired_graph_fingerprint="",
        original_deadline_fingerprint=_deadline_fingerprint(objective),
        repaired_deadline_fingerprint="",
        original_benchmark_fingerprint=_benchmark_fingerprint(objective),
        repaired_benchmark_fingerprint="",
    )
    return ArcSwapRepairResult(False, "unrepairable", None, diagnostics)


def _truck_travel_time(instance: InstanceData, arc: Arc) -> float:
    left, right = arc
    left_xy = instance.locations[left]
    right_xy = instance.locations[right]
    return (
        abs(left_xy[0] - right_xy[0]) + abs(left_xy[1] - right_xy[1])
    ) / instance.config.truck_speed * 60.0


def _feasibility_objective(instance: InstanceData) -> ObjectiveData:
    return build_objective_data(
        instance,
        ObjectiveWeights(delay=1.0 / 3.0, return_time=1.0 / 3.0, cost=1.0 / 3.0),
    )


def _last_stage_value(diagnostics: FeasibilityGateDiagnostics, name: str) -> float:
    if not diagnostics.stages:
        return 0.0
    return float(getattr(diagnostics.stages[-1], name))


def _deadline_fingerprint(objective: ObjectiveData) -> str:
    return _json_fingerprint(
        {
            "arrival_lb": objective.bounds.arrival_lb,
            "service_ub": objective.bounds.service_ub,
            "service_deadline": objective.bounds.service_deadline,
            "service_deadline_offset": objective.bounds.service_deadline_offset,
        }
    )


def _benchmark_fingerprint(objective: ObjectiveData) -> str:
    return _json_fingerprint(
        {
            "bounds": asdict(objective.bounds),
            "coeffs": asdict(objective.coeffs),
        }
    )


def _json_fingerprint(value: object) -> str:
    canonical = json.dumps(_finite_json(value), sort_keys=True, separators=(",", ":"), allow_nan=False)
    return sha256(canonical.encode("utf-8")).hexdigest()


def _finite_json(value: object) -> object:
    if isinstance(value, dict):
        return {str(key): _finite_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_finite_json(item) for item in value]
    if isinstance(value, float) and not isfinite(value):
        return "Infinity" if value > 0.0 else "-Infinity"
    return value
