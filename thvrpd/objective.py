from __future__ import annotations

from dataclasses import dataclass
from math import ceil, inf, isfinite
import random
import time

import networkx as nx

from .config import ObjectiveWeights
from .instance import InstanceData


@dataclass(frozen=True)
class ObjectiveBounds:
    arrival_lb: dict[str, float]
    service_ub: dict[str, float]
    data_service_ub: dict[str, float]
    service_deadline: dict[str, float]
    service_deadline_req_ub: dict[str, float]
    service_deadline_offset: dict[str, float]
    service_deadline_candidate: dict[str, float]
    service_deadline_mode: str
    service_deadline_offset_min: float
    service_deadline_offset_max: float
    service_deadline_random_seed: int | None
    service_deadline_setup_time: float
    service_deadline_active_count: int
    min_service_deadline_slack: float
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
    effective_service_ub: dict[str, float] = {}
    active_deadlines = 0
    min_deadline_slack = inf
    random_seed = instance.config.service_deadline_random_seed
    if instance.config.service_deadline_mode == "random_absolute":
        if random_seed is None:
            random_seed = instance.config.seed
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
        elif instance.config.service_deadline_mode == "random_absolute":
            offset = _random_absolute_offset(instance.config.seed, random_seed, customer, instance.config.service_deadline_offset_min, instance.config.service_deadline_offset_max)
            anchor = arrival_lb[customer]
            candidate = anchor + offset
            deadline = candidate
        else:
            deadline = inf
            offset = inf
            candidate = inf
        service_deadline[customer] = deadline
        service_deadline_req_ub[customer] = deadline
        service_deadline_offset[customer] = offset
        service_deadline_candidate[customer] = candidate
        effective = min(data_ub, deadline)
        if effective < arrival_lb[customer] - 1e-9:
            raise ValueError(f"effective service upper bound for {customer} is below the relaxed earliest service benchmark")
        effective_service_ub[customer] = effective
        if isfinite(deadline) and deadline < data_ub - 1e-9:
            active_deadlines += 1
            min_deadline_slack = min(min_deadline_slack, deadline - arrival_lb[customer])
    if active_deadlines == 0:
        min_deadline_slack = inf
    service_deadline_setup_time = time.time() - deadline_setup_start

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
        service_deadline_mode=instance.config.service_deadline_mode,
        service_deadline_offset_min=instance.config.service_deadline_offset_min,
        service_deadline_offset_max=instance.config.service_deadline_offset_max,
        service_deadline_random_seed=random_seed,
        service_deadline_setup_time=service_deadline_setup_time,
        service_deadline_active_count=active_deadlines,
        min_service_deadline_slack=min_deadline_slack,
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
