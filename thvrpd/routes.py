from __future__ import annotations

from dataclasses import dataclass

from .instance import Arc, InstanceData
from .objective import ObjectiveData
from .transform import TransformedGraph, duplicate_customer, is_duplicate

PAYLOAD_TOLERANCE = 1e-9
SERVICE_DEADLINE_TOLERANCE = 1e-9


class ServiceEnvelopeViolation(ValueError):
    pass


@dataclass(frozen=True)
class Route:
    id: int
    path: tuple[str, ...]
    truck_path: tuple[str, ...]
    drone_blocks: dict[str, tuple[str, ...]]
    served: frozenset[str]
    truck_served: frozenset[str]
    pad_served: frozenset[tuple[str, str]]
    service_times: dict[str, float]
    return_time: float
    drone_sorties: int
    delay_square_sum: float
    operating_cost: float
    cost: float
    used_arcs: frozenset[Arc]

    @property
    def drone_served(self) -> frozenset[str]:
        return frozenset(customer for _, customer in self.pad_served)

    def sr_coeff(self, triplet: tuple[str, str, str]) -> int:
        return len(self.served.intersection(triplet)) // 2

    def to_record(self, objective: ObjectiveData) -> dict:
        return {
            "id": self.id,
            "path": list(self.path),
            "truck_path": list(self.truck_path),
            "drone_blocks": {hub: list(customers) for hub, customers in self.drone_blocks.items()},
            "served": sorted(self.served),
            "truck_served": sorted(self.truck_served),
            "service_times": self.service_times,
            "return_time": self.return_time,
            "drone_sorties": self.drone_sorties,
            "delay_square_sum": self.delay_square_sum,
            "operating_cost": self.operating_cost,
            "shifted_delay_contribution": objective.coeffs.delay * self.delay_square_sum,
            "shifted_return_contribution": objective.coeffs.return_time * self.return_time,
            "shifted_cost_contribution": objective.coeffs.cost * self.operating_cost,
            "shifted_cost": self.cost,
        }


def route_from_path(route_id: int, path: tuple[str, ...], graph: TransformedGraph, objective: ObjectiveData) -> Route:
    instance = graph.instance
    if path[0] != instance.depot_source or path[-1] != instance.depot_sink:
        raise ValueError("route path must start at Source and end at Sink")

    physical_seen = {instance.depot_source}
    represented_seen: set[str] = set()
    truck_path = [instance.depot_source]
    drone_blocks: dict[str, list[str]] = {}
    service_times: dict[str, float] = {}
    truck_served: set[str] = set()
    pad_served: set[tuple[str, str]] = set()
    active_pad: str | None = None
    active_pad_arrival = 0.0
    active_wait = 0.0
    physical_time = 0.0

    for i, j in zip(path, path[1:]):
        arc = (i, j)
        if arc not in graph.arcs:
            raise ValueError(f"not a transformed arc: {arc}")
        if not graph.arc_compatible_with_active_pad(arc, active_pad):
            raise ValueError(f"transformed arc is incompatible with active pad {active_pad}: {arc}")
        if j in instance.customers or j in instance.hubs:
            if j in physical_seen:
                raise ValueError(f"route violates physical-node elementarity at {j}")
            physical_seen.add(j)
        if is_duplicate(j):
            customer = duplicate_customer(j)
            if customer in represented_seen:
                raise ValueError(f"route represents customer more than once: {customer}")
            represented_seen.add(customer)
        elif j in instance.customers:
            if j in represented_seen:
                raise ValueError(f"route represents customer more than once: {j}")
            represented_seen.add(j)
        if arc in graph.truck_arcs:
            physical_time += instance.truck_time[arc]
            truck_path.append(j)
            active_pad = j if j in instance.hubs else active_pad
            if j in instance.customers:
                truck_served.add(j)
                service_times[j] = physical_time
            continue
        if arc in graph.hub_duplicate_arcs:
            active_pad = i
            active_pad_arrival = physical_time
            active_wait = 0.0
            customer = duplicate_customer(j)
            drone_blocks.setdefault(active_pad, []).append(customer)
            if len(drone_blocks[active_pad]) > instance.drones_per_truck:
                raise ValueError(f"route violates drone fleet size at {active_pad}")
            pad_served.add((active_pad, customer))
            service_times[customer] = active_pad_arrival + instance.drone_time[(active_pad, customer)]
            active_wait = max(active_wait, instance.drone_trip_time[(active_pad, customer)])
            continue
        if arc in graph.duplicate_duplicate_arcs:
            if active_pad is None:
                raise ValueError("duplicate block has no active pad")
            customer = duplicate_customer(j)
            drone_blocks.setdefault(active_pad, []).append(customer)
            if len(drone_blocks[active_pad]) > instance.drones_per_truck:
                raise ValueError(f"route violates drone fleet size at {active_pad}")
            pad_served.add((active_pad, customer))
            service_times[customer] = active_pad_arrival + instance.drone_time[(active_pad, customer)]
            active_wait = max(active_wait, instance.drone_trip_time[(active_pad, customer)])
            continue
        if arc in graph.duplicate_regular_arcs:
            if active_pad is None:
                raise ValueError("duplicate continuation has no active pad")
            physical_time = active_pad_arrival + active_wait + instance.truck_time[(active_pad, j)]
            truck_path.append(j)
            active_pad = j if j in instance.hubs else active_pad
            active_wait = 0.0
            if j in instance.customers:
                truck_served.add(j)
                service_times[j] = physical_time
            continue
        raise ValueError(f"unclassified transformed arc: {arc}")

    served = frozenset(truck_served | {customer for _, customer in pad_served})
    if not served:
        raise ValueError("route columns must be nonempty")
    payload = sum(instance.demand[customer] for customer in served)
    if payload > instance.truck_payload + PAYLOAD_TOLERANCE:
        raise ValueError(f"route violates truck payload: {payload} > {instance.truck_payload}")
    if len(served) != len(service_times):
        raise ValueError("route does not have one service time per served customer")
    for customer in served:
        upper = objective.bounds.service_ub[customer]
        if service_times[customer] > upper + SERVICE_DEADLINE_TOLERANCE:
            raise ServiceEnvelopeViolation(
                f"route serves {customer} at {service_times[customer]} after effective service upper bound {upper}"
            )
    delay_square_sum = sum((service_times[c] - objective.bounds.arrival_lb[c]) ** 2 for c in served)
    operating_cost = instance.truck_cost + instance.drone_cost * len(pad_served)
    route_cost = (
        objective.coeffs.delay * delay_square_sum
        + objective.coeffs.return_time * physical_time
        + objective.coeffs.cost * operating_cost
    )
    return Route(
        id=route_id,
        path=path,
        truck_path=tuple(truck_path),
        drone_blocks={hub: tuple(customers) for hub, customers in drone_blocks.items()},
        served=served,
        truck_served=frozenset(truck_served),
        pad_served=frozenset(pad_served),
        service_times=service_times,
        return_time=physical_time,
        drone_sorties=len(pad_served),
        delay_square_sum=delay_square_sum,
        operating_cost=operating_cost,
        cost=route_cost,
        used_arcs=frozenset(zip(path, path[1:])),
    )


def validate_route_cover(
    paths: tuple[tuple[str, ...], ...],
    graph: TransformedGraph,
    objective: ObjectiveData,
) -> tuple[Route, ...]:
    instance = graph.instance
    routes = tuple(route_from_path(index, path, graph, objective) for index, path in enumerate(paths))
    if len(routes) > instance.num_trucks:
        raise ValueError(f"route cover uses {len(routes)} trucks but fleet size is {instance.num_trucks}")
    coverage = {
        customer: sum(customer in route.served for route in routes)
        for customer in instance.customers
    }
    if any(count != 1 for count in coverage.values()):
        raise ValueError(f"route cover does not serve every customer exactly once: {coverage}")
    return routes


def is_customer_representation(node: str, instance: InstanceData) -> bool:
    return node in instance.customers or is_duplicate(node)
