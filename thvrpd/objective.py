from __future__ import annotations

from dataclasses import dataclass, replace
from math import ceil, inf, isfinite
import random
import time

import networkx as nx

from .config import ObjectiveWeights
from .instance import InstanceData


RANDOM_ABSOLUTE_WITNESS_TIME_LIMIT_SECONDS = 30.0


@dataclass(frozen=True)
class ObjectiveBounds:
    arrival_lb: dict[str, float]
    service_ub: dict[str, float]
    data_service_ub: dict[str, float]
    service_deadline: dict[str, float]
    service_deadline_req_ub: dict[str, float]
    service_deadline_offset: dict[str, float]
    service_deadline_candidate: dict[str, float]
    service_deadline_witness_floor: dict[str, float]
    service_deadline_witness_service_time: dict[str, float]
    service_deadline_witness_lift: dict[str, float]
    service_deadline_mode: str
    service_deadline_fraction: float
    service_deadline_offset_min: float
    service_deadline_offset_max: float
    service_deadline_witness_slack: float
    service_deadline_random_seed: int | None
    service_deadline_setup_time: float
    service_deadline_witness_time: float
    service_deadline_witness_status: str
    service_deadline_witness_method: str
    service_deadline_witness_constructive_time: float
    service_deadline_witness_compact_time: float
    service_deadline_witness_failed_customers: tuple[str, ...]
    service_deadline_active_count: int
    min_service_deadline_slack: float
    num_witness_lifts: int
    max_witness_lift: float
    mean_witness_lift: float
    delay_lb: float
    delay_ub: float
    return_lb: float
    return_ub: float
    cost_lb: float
    cost_ub: float
    route_time_ub: float
    min_trucks: int
    max_used_trucks: int
    min_drone_flights: int


@dataclass(frozen=True)
class ObjectiveCoefficients:
    delay: float
    return_time: float
    cost: float
    shift: float


@dataclass(frozen=True)
class ObjectiveData:
    weights: ObjectiveWeights
    bounds: ObjectiveBounds
    coeffs: ObjectiveCoefficients

    def full_value_from_route_sum(self, route_sum: float) -> float:
        return route_sum + self.coeffs.shift


def build_objective_data(instance: InstanceData, weights: ObjectiveWeights) -> ObjectiveData:
    graph = instance.truck_graph()
    shortest = dict(nx.all_pairs_dijkstra_path_length(graph, weight="weight"))

    def sp(i: str, j: str) -> float:
        return float(shortest.get(i, {}).get(j, float("inf")))

    arrival_lb: dict[str, float] = {}
    for customer in instance.customers:
        truck_lb = sp(instance.depot_source, customer)
        if instance.demand[customer] > instance.truck_payload or not isfinite(sp(customer, instance.depot_sink)):
            truck_lb = float("inf")
        drone_candidates = [
            sp(instance.depot_source, hub) + instance.drone_time[(hub, customer)]
            for hub in instance.hubs
            if (hub, customer) in instance.drone_arcs
            and isfinite(sp(instance.depot_source, hub))
            and isfinite(sp(hub, instance.depot_sink))
        ]
        drone_lb = min(drone_candidates) if drone_candidates else float("inf")
        arrival_lb[customer] = min(truck_lb, drone_lb)
        if not isfinite(arrival_lb[customer]):
            raise ValueError(f"customer {customer} has no feasible complete-route service representation")

    route_arc_limit = len(instance.customers) + len(instance.hubs) + 1
    largest_truck_times = sorted(instance.truck_time.values(), reverse=True)[:route_arc_limit]
    largest_waits = sorted(
        [
            max(
                [instance.drone_trip_time[(hub, customer)] for customer in instance.customers if (hub, customer) in instance.drone_arcs],
                default=0.0,
            )
            for hub in instance.hubs
        ],
        reverse=True,
    )
    route_time_ub = sum(largest_truck_times) + sum(largest_waits)
    if not isfinite(route_time_ub):
        raise ValueError("route-duration upper bound is not finite")

    data_service_ub: dict[str, float] = {}
    for customer in instance.customers:
        truck_ub = route_time_ub - sp(customer, instance.depot_sink) if isfinite(sp(customer, instance.depot_sink)) else float("-inf")
        drone_ubs = [
            route_time_ub - instance.drone_time[(customer, hub)] - sp(hub, instance.depot_sink)
            for hub in instance.hubs
            if (hub, customer) in instance.drone_arcs and isfinite(sp(hub, instance.depot_sink))
        ]
        drone_ub = max(drone_ubs) if drone_ubs else float("-inf")
        value = max(arrival_lb[customer], truck_ub, drone_ub)
        if not isfinite(value):
            raise ValueError(f"customer {customer} has no finite service upper bound")
        data_service_ub[customer] = value

    deadline_setup_start = time.time()
    service_deadline: dict[str, float] = {}
    service_deadline_req_ub: dict[str, float] = {}
    service_deadline_offset: dict[str, float] = {}
    service_deadline_candidate: dict[str, float] = {}
    service_deadline_witness_floor: dict[str, float] = {}
    service_deadline_witness_service_time: dict[str, float] = {}
    service_deadline_witness_lift: dict[str, float] = {}
    effective_service_ub: dict[str, float] = {}
    active_deadlines = 0
    min_deadline_slack = inf
    witness_time = 0.0
    witness_status = "not_required"
    witness_constructive_time = 0.0
    witness_compact_time = 0.0
    witness_failed_customers: tuple[str, ...] = tuple()
    random_seed = instance.config.service_deadline_random_seed
    if instance.config.service_deadline_mode == "random_absolute":
        if random_seed is None:
            random_seed = instance.config.seed
        witness_start = time.time()
        (
            service_deadline_witness_service_time,
            witness_status,
            witness_constructive_time,
            witness_compact_time,
            witness_failed_customers,
        ) = _deadline_free_witness_service_times(instance, weights)
        witness_time = time.time() - witness_start
    manual_bounds = instance.config.service_deadline_manual_bounds or {}
    if instance.config.service_deadline_mode == "manual":
        missing = sorted(set(instance.customers).difference(manual_bounds))
        if missing:
            raise ValueError(f"manual service deadlines missing customers: {missing}")
    for customer in instance.customers:
        data_ub = data_service_ub[customer]
        if instance.config.service_deadline_mode == "manual":
            anchor = arrival_lb[customer]
            deadline = float(manual_bounds[customer])
            if deadline < anchor - 1e-9:
                raise ValueError(f"service deadline for {customer} is below the relaxed earliest service benchmark")
            offset = inf
            candidate = deadline
            witness_floor = inf
            lift = 0.0
        elif instance.config.service_deadline_mode == "random_absolute":
            if customer not in service_deadline_witness_service_time:
                raise ValueError(f"deadline-free witness does not serve {customer}")
            offset = _random_absolute_offset(instance.config.seed, random_seed, customer, instance.config.service_deadline_offset_min, instance.config.service_deadline_offset_max)
            anchor = arrival_lb[customer]
            candidate = anchor + offset
            witness_floor = service_deadline_witness_service_time[customer] + instance.config.service_deadline_witness_slack
            deadline = max(candidate, witness_floor)
            lift = max(deadline - candidate, 0.0)
            if deadline < service_deadline_witness_service_time[customer] - 1e-9:
                raise ValueError(f"service deadline for {customer} excludes the deadline-free witness")
        else:
            deadline = inf
            offset = inf
            candidate = inf
            witness_floor = inf
            lift = 0.0
        service_deadline[customer] = deadline
        service_deadline_req_ub[customer] = deadline
        service_deadline_offset[customer] = offset
        service_deadline_candidate[customer] = candidate
        service_deadline_witness_floor[customer] = witness_floor
        service_deadline_witness_lift[customer] = lift
        if customer not in service_deadline_witness_service_time:
            service_deadline_witness_service_time[customer] = inf
        effective = min(data_ub, deadline)
        if effective < arrival_lb[customer] - 1e-9:
            raise ValueError(f"effective service upper bound for {customer} is below the relaxed earliest service benchmark")
        if instance.config.service_deadline_mode == "random_absolute" and effective < service_deadline_witness_service_time[customer] - 1e-9:
            raise ValueError(f"effective service upper bound for {customer} excludes the deadline-free witness")
        effective_service_ub[customer] = effective
        if isfinite(deadline) and deadline < data_ub - 1e-9:
            active_deadlines += 1
            min_deadline_slack = min(min_deadline_slack, deadline - arrival_lb[customer])
    if active_deadlines == 0:
        min_deadline_slack = inf
    service_deadline_setup_time = time.time() - deadline_setup_start
    finite_lifts = [value for value in service_deadline_witness_lift.values() if value > 1e-9]
    num_witness_lifts = len(finite_lifts)
    max_witness_lift = max(finite_lifts, default=0.0)
    mean_witness_lift = sum(finite_lifts) / len(finite_lifts) if finite_lifts else 0.0

    total_demand = sum(instance.demand[customer] for customer in instance.customers)
    min_trucks = max(1, ceil(total_demand / instance.truck_payload))
    if min_trucks > instance.num_trucks:
        raise ValueError("total demand requires more trucks than available")
    max_used_trucks = min(instance.num_trucks, len(instance.customers))

    route_return_lb = min(
        sp(instance.depot_source, node) + sp(node, instance.depot_sink)
        for node in instance.customers + instance.hubs
    )
    if not isfinite(route_return_lb):
        raise ValueError("no finite complete physical truck path exists for return-time normalization")
    mandatory_drone = [
        customer
        for customer in instance.customers
        if not isfinite(sp(instance.depot_source, customer)) or not isfinite(sp(customer, instance.depot_sink))
    ]

    delay_ub = sum(max(effective_service_ub[c] - arrival_lb[c], 0.0) ** 2 for c in instance.customers)
    return_ub = max_used_trucks * route_time_ub
    cost_lb = min_trucks * instance.truck_cost + len(mandatory_drone) * instance.drone_cost
    cost_ub = max_used_trucks * instance.truck_cost + len(instance.customers) * instance.drone_cost
    if not all(isfinite(value) for value in (delay_ub, return_ub, cost_lb, cost_ub)):
        raise ValueError("normalization bounds must be finite")
    if return_ub < min_trucks * route_return_lb or cost_ub < cost_lb:
        raise ValueError("normalization upper bounds must dominate lower bounds")

    bounds = ObjectiveBounds(
        arrival_lb=arrival_lb,
        service_ub=effective_service_ub,
        data_service_ub=data_service_ub,
        service_deadline=service_deadline,
        service_deadline_req_ub=service_deadline_req_ub,
        service_deadline_offset=service_deadline_offset,
        service_deadline_candidate=service_deadline_candidate,
        service_deadline_witness_floor=service_deadline_witness_floor,
        service_deadline_witness_service_time=service_deadline_witness_service_time,
        service_deadline_witness_lift=service_deadline_witness_lift,
        service_deadline_mode=instance.config.service_deadline_mode,
        service_deadline_fraction=instance.config.service_deadline_fraction,
        service_deadline_offset_min=instance.config.service_deadline_offset_min,
        service_deadline_offset_max=instance.config.service_deadline_offset_max,
        service_deadline_witness_slack=instance.config.service_deadline_witness_slack,
        service_deadline_random_seed=random_seed,
        service_deadline_setup_time=service_deadline_setup_time,
        service_deadline_witness_time=witness_time,
        service_deadline_witness_status=witness_status,
        service_deadline_witness_method=instance.config.service_deadline_witness_method,
        service_deadline_witness_constructive_time=witness_constructive_time,
        service_deadline_witness_compact_time=witness_compact_time,
        service_deadline_witness_failed_customers=witness_failed_customers,
        service_deadline_active_count=active_deadlines,
        min_service_deadline_slack=min_deadline_slack,
        num_witness_lifts=num_witness_lifts,
        max_witness_lift=max_witness_lift,
        mean_witness_lift=mean_witness_lift,
        delay_lb=0.0,
        delay_ub=delay_ub,
        return_lb=min_trucks * route_return_lb,
        return_ub=return_ub,
        cost_lb=cost_lb,
        cost_ub=cost_ub,
        route_time_ub=route_time_ub,
        min_trucks=min_trucks,
        max_used_trucks=max_used_trucks,
        min_drone_flights=len(mandatory_drone),
    )
    coeffs = ObjectiveCoefficients(
        delay=weights.delay / (bounds.delay_ub - bounds.delay_lb) if bounds.delay_ub > bounds.delay_lb else 0.0,
        return_time=weights.return_time / (bounds.return_ub - bounds.return_lb) if bounds.return_ub > bounds.return_lb else 0.0,
        cost=weights.cost / (bounds.cost_ub - bounds.cost_lb) if bounds.cost_ub > bounds.cost_lb else 0.0,
        shift=0.0,
    )
    coeffs = ObjectiveCoefficients(
        delay=coeffs.delay,
        return_time=coeffs.return_time,
        cost=coeffs.cost,
        shift=-(coeffs.delay * bounds.delay_lb + coeffs.return_time * bounds.return_lb + coeffs.cost * bounds.cost_lb),
    )
    return ObjectiveData(weights=weights, bounds=bounds, coeffs=coeffs)


def _random_absolute_offset(instance_seed: int, random_seed: int, customer: str, offset_min: float, offset_max: float) -> float:
    customer_index = int(customer[1:]) if customer.startswith("C") and customer[1:].isdigit() else sum(ord(char) for char in customer)
    rng = random.Random((random_seed + 10_003 * instance_seed + 1_000_003 * customer_index) & 0xFFFFFFFF)
    return rng.uniform(offset_min, offset_max)


def _deadline_free_witness_service_times(
    instance: InstanceData,
    weights: ObjectiveWeights,
) -> tuple[dict[str, float], str, float, float, tuple[str, ...]]:
    method = instance.config.service_deadline_witness_method
    constructive_time = 0.0
    compact_time = 0.0
    failed_customers: tuple[str, ...] = tuple()
    if method in {"constructive", "constructive_then_compact"}:
        start = time.time()
        service_times, failed_customers = _constructive_deadline_free_witness_service_times(instance, weights)
        constructive_time = time.time() - start
        if not failed_customers:
            return service_times, "success", constructive_time, compact_time, tuple()
        if method == "constructive":
            raise ValueError(f"constructive deadline-free witness failed customers: {list(failed_customers)}")
    if method in {"compact", "constructive_then_compact"}:
        start = time.time()
        service_times = _compact_deadline_free_witness_service_times(instance, weights)
        compact_time = time.time() - start
        return service_times, "success", constructive_time, compact_time, failed_customers
    raise ValueError(f"unknown service deadline witness method {method}")


def _compact_deadline_free_witness_service_times(instance: InstanceData, weights: ObjectiveWeights) -> dict[str, float]:
    from .compact import solve_compact_solution
    from .routes import route_from_path
    from .transform import build_transformed_graph

    no_deadline_config = replace(instance.config, service_deadline_mode="none")
    no_deadline_instance = replace(instance, config=no_deadline_config)
    solution = solve_compact_solution(
        no_deadline_instance,
        weights,
        time_limit=instance.config.service_deadline_witness_time_limit,
        threads=1,
        require_optimal=False,
    )
    if not solution.route_paths:
        raise ValueError("deadline-free witness compact model did not produce a feasible route set")
    objective = build_objective_data(no_deadline_instance, weights)
    graph = build_transformed_graph(no_deadline_instance)
    service_times: dict[str, float] = {}
    for route_id, path in enumerate(solution.route_paths):
        route = route_from_path(route_id, path, graph, objective)
        for customer, service_time in route.service_times.items():
            if customer in service_times:
                raise ValueError(f"deadline-free witness serves {customer} more than once")
            service_times[customer] = service_time
    missing = set(instance.customers).difference(service_times)
    if missing:
        raise ValueError(f"deadline-free witness does not cover customers: {sorted(missing)}")
    if len(solution.route_paths) > instance.num_trucks:
        raise ValueError("deadline-free witness uses more routes than available trucks")
    return service_times


def _constructive_deadline_free_witness_service_times(
    instance: InstanceData,
    weights: ObjectiveWeights,
) -> tuple[dict[str, float], tuple[str, ...]]:
    from .routes import ServiceEnvelopeViolation, route_from_path
    from .transform import build_transformed_graph

    no_deadline_config = replace(instance.config, service_deadline_mode="none")
    no_deadline_instance = replace(instance, config=no_deadline_config)
    objective = build_objective_data(no_deadline_instance, weights)
    graph = build_transformed_graph(no_deadline_instance)
    deadline = time.time() + instance.config.service_deadline_witness_time_limit
    residual = set(instance.customers)
    selected_paths: list[tuple[str, ...]] = []
    for _ in range(instance.num_trucks):
        if not residual or time.time() >= deadline:
            break
        prefix = [instance.depot_source]
        while residual and time.time() < deadline:
            extension = _constructive_witness_extension(prefix, residual, graph, objective)
            if extension is None:
                break
            prefix = extension
        if len(prefix) == 1:
            continue
        path = tuple(prefix + [instance.depot_sink])
        try:
            route = route_from_path(len(selected_paths), path, graph, objective)
        except (ServiceEnvelopeViolation, ValueError):
            continue
        if not route.served.issubset(residual):
            continue
        selected_paths.append(path)
        residual.difference_update(route.served)
    if residual:
        return {}, tuple(sorted(residual))
    service_times: dict[str, float] = {}
    for route_id, path in enumerate(selected_paths):
        route = route_from_path(route_id, path, graph, objective)
        for customer, service_time in route.service_times.items():
            if customer in service_times:
                raise ValueError(f"constructive deadline-free witness serves {customer} more than once")
            service_times[customer] = service_time
    missing = set(instance.customers).difference(service_times)
    if missing:
        raise ValueError(f"constructive deadline-free witness does not cover customers: {sorted(missing)}")
    return service_times, tuple()


def _constructive_witness_extension(prefix: list[str], residual: set[str], graph, objective) -> list[str] | None:
    instance = graph.instance
    best: tuple[tuple[float, float, str, tuple[str, ...]], list[str]] | None = None
    from .routes import ServiceEnvelopeViolation, route_from_path

    for customer in sorted(residual):
        for connector in _constructive_witness_connectors(prefix, customer, graph):
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
            score = (route.return_time, -len(route.served), customer, path)
            if best is None or score < best[0]:
                best = (score, candidate_prefix)
    return None if best is None else best[1]


def _constructive_witness_connectors(prefix: list[str], customer: str, graph) -> tuple[tuple[str, ...], ...]:
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
