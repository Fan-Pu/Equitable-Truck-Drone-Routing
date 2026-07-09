from __future__ import annotations

from dataclasses import dataclass, field
from math import ceil, hypot, isclose, isfinite
import random

import networkx as nx
import numpy as np

from .config import InstanceConfig


Node = str
Arc = tuple[Node, Node]
CAPACITY_TOLERANCE = 1e-9


@dataclass(frozen=True)
class InstanceData:
    config: InstanceConfig
    depot_source: Node
    depot_sink: Node
    customers: tuple[Node, ...]
    hubs: tuple[Node, ...]
    nodes: tuple[Node, ...]
    truck_arcs: frozenset[Arc]
    drone_arcs: frozenset[Arc]
    truck_time: dict[Arc, float]
    drone_time: dict[Arc, float]
    drone_trip_time: dict[Arc, float]
    demand: dict[Node, float]
    locations: dict[Node, tuple[float, float]]
    mandatory_drone_customers: tuple[Node, ...] = ()
    drone_arc_saving: dict[Arc, float] = field(default_factory=dict)
    witness_routes: tuple[tuple[Node, ...], ...] = ()
    witness_route_drone_blocks: tuple[tuple[tuple[Node, tuple[Node, ...]], ...], ...] = ()

    def __post_init__(self) -> None:
        physical = set(self.customers) | set(self.hubs)
        if self.depot_source == self.depot_sink:
            raise ValueError("source and sink depot copies must be distinct nodes")
        if set(self.customers).intersection(self.hubs):
            raise ValueError("customer and synchronization-pad sets must be disjoint")
        if self.depot_source in physical or self.depot_sink in physical:
            raise ValueError("depot copies must be disjoint from customers and pads")
        if set(self.nodes) != {self.depot_source, self.depot_sink} | physical:
            raise ValueError("node set must equal depot copies, customers, and pads")
        if self.truck_payload <= 0.0 or self.drone_payload <= 0.0 or self.drone_endurance <= 0.0:
            raise ValueError("truck payload, drone payload, and drone endurance must be positive")
        if self.truck_cost < 0.0 or self.drone_cost < 0.0:
            raise ValueError("fixed truck and drone costs must be nonnegative")
        for node in self.nodes:
            if node not in self.demand:
                raise ValueError(f"missing demand for node {node}")
            if not isfinite(self.demand[node]) or self.demand[node] < 0.0:
                raise ValueError(f"node demand must be finite and nonnegative: {node}")
        for customer in self.customers:
            if self.demand[customer] <= 0.0:
                raise ValueError(f"customer parcel weight must be strictly positive: {customer}")
            if self.demand[customer] > self.truck_payload + CAPACITY_TOLERANCE:
                raise ValueError(f"customer parcel weight exceeds truck payload: {customer}")
        total_customer_demand = sum(self.demand[customer] for customer in self.customers)
        total_truck_payload = self.num_trucks * self.truck_payload
        if total_customer_demand > total_truck_payload + CAPACITY_TOLERANCE:
            raise ValueError(
                f"aggregate customer demand exceeds total truck payload: {total_customer_demand} > {total_truck_payload}"
            )
        for i, j in self.truck_arcs:
            if i not in self.nodes or j not in self.nodes:
                raise ValueError(f"truck arc uses unknown node: {(i, j)}")
            if j == self.depot_source or i == self.depot_sink:
                raise ValueError(f"truck arcs may not enter Source or leave Sink: {(i, j)}")
            if (i, j) not in self.truck_time:
                raise ValueError(f"missing truck travel time for {(i, j)}")
            if not isfinite(self.truck_time[(i, j)]) or self.truck_time[(i, j)] < 0.0:
                raise ValueError(f"truck travel time must be finite and nonnegative: {(i, j)}")
        for h, customer in self.drone_arcs:
            if h not in self.hubs or customer not in self.customers:
                raise ValueError(f"drone sortie arc must be pad-to-customer: {(h, customer)}")
            if (h, customer) not in self.drone_time or (customer, h) not in self.drone_time:
                raise ValueError(f"missing one-way drone times for {(h, customer)}")
            if (h, customer) not in self.drone_trip_time:
                raise ValueError(f"missing round-trip drone time for {(h, customer)}")
            outbound = self.drone_time[(h, customer)]
            inbound = self.drone_time[(customer, h)]
            trip = self.drone_trip_time[(h, customer)]
            if not isfinite(outbound) or not isfinite(inbound) or min(outbound, inbound) < 0.0:
                raise ValueError(f"one-way drone times must be finite and nonnegative: {(h, customer)}")
            if not isfinite(trip) or trip < 0.0:
                raise ValueError(f"round-trip drone time must be finite and nonnegative: {(h, customer)}")
            if not isclose(trip, outbound + inbound, rel_tol=1e-9, abs_tol=1e-9):
                raise ValueError(f"round-trip drone time must equal outbound plus return time: {(h, customer)}")
            if self.demand[customer] > self.drone_payload or self.drone_trip_time[(h, customer)] > self.drone_endurance:
                raise ValueError(f"infeasible drone sortie retained in drone arc set: {(h, customer)}")
        mandatory = set(self.mandatory_drone_customers)
        if not mandatory.issubset(self.customers):
            raise ValueError("mandatory drone customers must be original customers")
        if len(mandatory) != len(self.mandatory_drone_customers):
            raise ValueError("mandatory drone customers must be unique")
        self._validate_customer_service_reachability()
        if mandatory:
            graph = self.truck_graph()
            for customer in self.mandatory_drone_customers:
                truck_reachable = nx.has_path(graph, self.depot_source, customer) and nx.has_path(graph, customer, self.depot_sink)
                if truck_reachable:
                    raise ValueError(f"mandatory drone customer retains truck-service representation: {customer}")
                if not any((hub, customer) in self.drone_arcs for hub in self.hubs):
                    raise ValueError(f"mandatory drone customer has no retained drone arc: {customer}")

    @property
    def truck_payload(self) -> float:
        return self.config.truck_payload

    @property
    def drone_payload(self) -> float:
        return self.config.drone_payload

    @property
    def drone_endurance(self) -> float:
        return self.config.drone_endurance

    @property
    def truck_cost(self) -> float:
        return self.config.truck_cost

    @property
    def drone_cost(self) -> float:
        return self.config.drone_cost

    @property
    def drones_per_truck(self) -> int:
        return self.config.drones_per_truck

    @property
    def num_trucks(self) -> int:
        return self.config.num_trucks

    def truck_graph(self) -> nx.DiGraph:
        graph = nx.DiGraph()
        graph.add_nodes_from(self.nodes)
        for i, j in self.truck_arcs:
            graph.add_edge(i, j, weight=self.truck_time[(i, j)])
        return graph

    def _validate_customer_service_reachability(self) -> None:
        graph = self.truck_graph()
        for customer in self.customers:
            truck_reachable = nx.has_path(graph, self.depot_source, customer) and nx.has_path(graph, customer, self.depot_sink)
            if truck_reachable:
                continue
            graph_without_customer = graph.copy()
            graph_without_customer.remove_node(customer)
            drone_reachable = any(
                (hub, customer) in self.drone_arcs
                and nx.has_path(graph_without_customer, self.depot_source, hub)
                and nx.has_path(graph_without_customer, hub, self.depot_sink)
                for hub in self.hubs
            )
            if not drone_reachable:
                raise ValueError(f"customer has no feasible service representation: {customer}")


def generate_instance(config: InstanceConfig) -> InstanceData:
    rng = random.Random(config.seed)
    np_rng = np.random.default_rng(config.seed)
    depot_source = "Source"
    depot_sink = "Sink"
    customers = tuple(f"C{i + 1}" for i in range(config.num_customers))
    hubs = tuple(f"H{i + 1}" for i in range(config.num_hubs))

    customer_locations = _sample_customers(config, np_rng)
    locations: dict[str, tuple[float, float]] = {
        customer: tuple(map(float, customer_locations[i]))
        for i, customer in enumerate(customers)
    }
    locations[depot_source] = (0.0, 0.0)
    locations[depot_sink] = locations[depot_source]
    if hubs:
        hub_locations = _choose_hubs_by_kmeans(config.seed, customer_locations, len(hubs))
        for i, hub in enumerate(hubs):
            locations[hub] = tuple(map(float, hub_locations[i]))

    nodes = (depot_source,) + customers + hubs + (depot_sink,)
    truck_arcs: set[Arc] = set()
    truck_time: dict[Arc, float] = {}
    drone_arcs: set[Arc] = set()
    drone_time: dict[Arc, float] = {}
    drone_trip_time: dict[Arc, float] = {}

    for node in customers + hubs:
        _add_truck_arc(config, locations, depot_source, node, truck_arcs, truck_time)
        _add_truck_arc(config, locations, node, depot_sink, truck_arcs, truck_time)

    physical = customers + hubs
    for i in physical:
        for j in physical:
            if i == j:
                continue
            if i in hubs or j in hubs:
                if (i in customers or j in customers) and rng.random() <= config.hub_arc_probability:
                    _add_truck_arc(config, locations, i, j, truck_arcs, truck_time)
            elif rng.random() <= config.truck_arc_probability:
                _add_truck_arc(config, locations, i, j, truck_arcs, truck_time)

    demand = _sample_demands(config, customers, rng, np_rng)
    demand[depot_source] = 0.0
    demand[depot_sink] = 0.0
    for hub in hubs:
        demand[hub] = 0.0

    for hub in hubs:
        for customer in customers:
            one_way = _drone_time(config, locations[hub], locations[customer])
            trip = 2.0 * one_way
            if demand[customer] <= config.drone_payload and trip <= config.drone_endurance:
                drone_arcs.add((hub, customer))
                drone_time[(hub, customer)] = one_way
                drone_time[(customer, hub)] = one_way
                drone_trip_time[(hub, customer)] = trip

    (
        truck_arcs,
        truck_time,
        drone_arcs,
        drone_time,
        drone_trip_time,
        mandatory_drone_customers,
        drone_arc_saving,
        witness_routes,
        witness_route_drone_blocks,
    ) = _apply_drone_required_graph_policy(
        config=config,
        depot_source=depot_source,
        depot_sink=depot_sink,
        customers=customers,
        hubs=hubs,
        locations=locations,
        demand=demand,
        truck_arcs=truck_arcs,
        truck_time=truck_time,
        drone_arcs=drone_arcs,
        drone_time=drone_time,
        drone_trip_time=drone_trip_time,
    )

    return InstanceData(
        config=config,
        depot_source=depot_source,
        depot_sink=depot_sink,
        customers=customers,
        hubs=hubs,
        nodes=nodes,
        truck_arcs=frozenset(truck_arcs),
        drone_arcs=frozenset(drone_arcs),
        truck_time=truck_time,
        drone_time=drone_time,
        drone_trip_time=drone_trip_time,
        demand=demand,
        locations=locations,
        mandatory_drone_customers=mandatory_drone_customers,
        drone_arc_saving=drone_arc_saving,
        witness_routes=witness_routes,
        witness_route_drone_blocks=witness_route_drone_blocks,
    )


def tiny_instance() -> InstanceData:
    config = InstanceConfig(
        seed=1,
        num_trucks=2,
        num_customers=3,
        distribution="PS",
        drones_per_truck=2,
        num_hubs=1,
        truck_arc_probability=1.0,
        hub_arc_probability=1.0,
        truck_payload=20.0,
        drone_payload=5.0,
        drone_endurance=100.0,
    )
    depot_source = "Source"
    depot_sink = "Sink"
    customers = ("C1", "C2", "C3")
    hubs = ("H1",)
    nodes = (depot_source,) + customers + hubs + (depot_sink,)
    locations = {
        depot_source: (0.0, 0.0),
        depot_sink: (0.0, 0.0),
        "C1": (1.0, 0.0),
        "C2": (2.0, 0.0),
        "C3": (1.0, 2.0),
        "H1": (1.0, 1.0),
    }
    truck_arcs: set[Arc] = set()
    truck_time: dict[Arc, float] = {}
    for i in nodes:
        for j in nodes:
            if i == j or j == depot_source or i == depot_sink:
                continue
            if i == depot_source or j == depot_sink or i in customers + hubs:
                _add_truck_arc(config, locations, i, j, truck_arcs, truck_time)
    demand = {depot_source: 0.0, depot_sink: 0.0, "H1": 0.0, "C1": 2.0, "C2": 2.0, "C3": 2.0}
    drone_arcs: set[Arc] = set()
    drone_time: dict[Arc, float] = {}
    drone_trip_time: dict[Arc, float] = {}
    for customer in customers:
        one_way = _drone_time(config, locations["H1"], locations[customer])
        drone_arcs.add(("H1", customer))
        drone_time[("H1", customer)] = one_way
        drone_time[(customer, "H1")] = one_way
        drone_trip_time[("H1", customer)] = 2.0 * one_way
    return InstanceData(
        config=config,
        depot_source=depot_source,
        depot_sink=depot_sink,
        customers=customers,
        hubs=hubs,
        nodes=nodes,
        truck_arcs=frozenset(truck_arcs),
        drone_arcs=frozenset(drone_arcs),
        truck_time=truck_time,
        drone_time=drone_time,
        drone_trip_time=drone_trip_time,
        demand=demand,
        locations=locations,
    )


def _sample_customers(config: InstanceConfig, np_rng: np.random.Generator) -> np.ndarray:
    if config.distribution == "PS":
        return np_rng.uniform(0.0, config.area_side, size=(config.num_customers, 2))
    if config.distribution == "PC":
        k = max(1, config.num_customers // 5)
        centers = np_rng.uniform(0.0, config.area_side, size=(k, 2))
        points = []
        for i in range(config.num_customers):
            point = np_rng.normal(loc=centers[i % k], scale=config.area_side * 0.05, size=2)
            points.append(np.clip(point, 0.0, config.area_side))
        return np.vstack(points)
    n_cluster = config.num_customers // 2
    sparse_config = InstanceConfig(**{**config.__dict__, "num_customers": config.num_customers - n_cluster, "distribution": "PS"})
    cluster_config = InstanceConfig(**{**config.__dict__, "num_customers": n_cluster, "distribution": "PC"})
    return np.vstack((_sample_customers(sparse_config, np_rng), _sample_customers(cluster_config, np_rng)))


def _apply_drone_required_graph_policy(
    *,
    config: InstanceConfig,
    depot_source: Node,
    depot_sink: Node,
    customers: tuple[Node, ...],
    hubs: tuple[Node, ...],
    locations: dict[Node, tuple[float, float]],
    demand: dict[Node, float],
    truck_arcs: set[Arc],
    truck_time: dict[Arc, float],
    drone_arcs: set[Arc],
    drone_time: dict[Arc, float],
    drone_trip_time: dict[Arc, float],
) -> tuple[
    set[Arc],
    dict[Arc, float],
    set[Arc],
    dict[Arc, float],
    dict[Arc, float],
    tuple[Node, ...],
    dict[Arc, float],
    tuple[tuple[Node, ...], ...],
    tuple[tuple[tuple[Node, tuple[Node, ...]], ...], ...],
]:
    if config.mandatory_drone_customer_fraction <= 0.0:
        saving = _drone_arc_savings(depot_source, customers, hubs, truck_arcs, truck_time, drone_arcs, drone_time)
        return (
            truck_arcs,
            truck_time,
            drone_arcs,
            drone_time,
            drone_trip_time,
            (),
            {arc: saving[arc] for arc in sorted(drone_arcs)},
            (),
            (),
        )

    savings = _drone_arc_savings(depot_source, customers, hubs, truck_arcs, truck_time, drone_arcs, drone_time)
    mandatory_count = int(ceil(config.mandatory_drone_customer_fraction * len(customers)))
    hub_slots = {hub: config.max_drone_access_customers_per_hub for hub in hubs}
    pair_candidates = sorted(
        (
            (savings[(hub, customer)], -drone_trip_time[(hub, customer)], hub, customer)
            for hub, customer in drone_arcs
        ),
        key=lambda item: (-item[0], item[1], item[2], item[3]),
    )
    protected_hub_for_customer: dict[Node, Node] = {}
    for _, _, hub, customer in pair_candidates:
        if len(protected_hub_for_customer) >= mandatory_count:
            break
        if customer in protected_hub_for_customer or hub_slots[hub] <= 0:
            continue
        protected_hub_for_customer[customer] = hub
        hub_slots[hub] -= 1
    if len(protected_hub_for_customer) < mandatory_count:
        raise ValueError(
            f"not enough hub-capacitated drone-feasible customers for mandatory drone policy: "
            f"{len(protected_hub_for_customer)} < {mandatory_count}"
        )
    mandatory_drone_customers = tuple(sorted(protected_hub_for_customer))
    mandatory_set = set(mandatory_drone_customers)
    protected_arcs = {(protected_hub_for_customer[customer], customer) for customer in mandatory_drone_customers}

    truck_arcs = {arc for arc in truck_arcs if arc[0] not in mandatory_set and arc[1] not in mandatory_set}
    truck_time = {arc: value for arc, value in truck_time.items() if arc in truck_arcs}

    retained_drone_arcs = _retained_drone_arcs(config, customers, hubs, drone_arcs, drone_trip_time, savings, protected_arcs)
    drone_time = {
        arc: value
        for arc, value in drone_time.items()
        if arc in retained_drone_arcs or (arc[1], arc[0]) in retained_drone_arcs
    }
    drone_trip_time = {arc: value for arc, value in drone_trip_time.items() if arc in retained_drone_arcs}

    witness_routes, witness_route_drone_blocks = _build_drone_required_witness(
        config=config,
        depot_source=depot_source,
        depot_sink=depot_sink,
        customers=customers,
        hubs=hubs,
        locations=locations,
        demand=demand,
        truck_arcs=truck_arcs,
        truck_time=truck_time,
        mandatory_drone_customers=mandatory_drone_customers,
        protected_hub_for_customer=protected_hub_for_customer,
    )
    return (
        truck_arcs,
        truck_time,
        retained_drone_arcs,
        drone_time,
        drone_trip_time,
        mandatory_drone_customers,
        {arc: savings[arc] for arc in sorted(retained_drone_arcs)},
        witness_routes,
        witness_route_drone_blocks,
    )


def _drone_arc_savings(
    depot_source: Node,
    customers: tuple[Node, ...],
    hubs: tuple[Node, ...],
    truck_arcs: set[Arc],
    truck_time: dict[Arc, float],
    drone_arcs: set[Arc],
    drone_time: dict[Arc, float],
) -> dict[Arc, float]:
    graph = nx.DiGraph()
    graph.add_weighted_edges_from((i, j, truck_time[(i, j)]) for i, j in truck_arcs)
    lengths = dict(nx.all_pairs_dijkstra_path_length(graph, weight="weight"))
    savings: dict[Arc, float] = {}
    for hub in hubs:
        for customer in customers:
            if (hub, customer) not in drone_arcs:
                continue
            truck_service = lengths.get(depot_source, {}).get(customer, float("inf"))
            drone_service = lengths.get(depot_source, {}).get(hub, float("inf")) + drone_time[(hub, customer)]
            savings[(hub, customer)] = truck_service - drone_service
    return savings


def _retained_drone_arcs(
    config: InstanceConfig,
    customers: tuple[Node, ...],
    hubs: tuple[Node, ...],
    drone_arcs: set[Arc],
    drone_trip_time: dict[Arc, float],
    savings: dict[Arc, float],
    protected_arcs: set[Arc],
) -> set[Arc]:
    if not config.retain_optional_drone_arcs:
        return set(protected_arcs)
    retained = set(protected_arcs)
    eligible = {
        arc
        for arc in drone_arcs
        if arc in protected_arcs or savings[arc] >= config.min_drone_service_time_saving
    }
    for hub in hubs:
        protected_for_hub = sorted(arc for arc in protected_arcs if arc[0] == hub)
        retained.update(protected_for_hub)
        room = config.max_drone_access_customers_per_hub - len(protected_for_hub)
        if room <= 0:
            continue
        optional = sorted(
            (arc for arc in eligible if arc[0] == hub and arc not in protected_arcs),
            key=lambda arc: (-savings[arc], drone_trip_time[arc], arc[0], arc[1]),
        )
        retained.update(optional[:room])
    for customer in customers:
        protected_for_customer = sorted(arc for arc in protected_arcs if arc[1] == customer)
        retained.update(protected_for_customer)
        room = config.max_drone_launch_hubs_per_customer - len(protected_for_customer)
        if room < 0:
            continue
        retained_for_customer = sorted(
            (arc for arc in retained if arc[1] == customer and arc not in protected_arcs),
            key=lambda arc: (-savings[arc], drone_trip_time[arc], arc[0], arc[1]),
        )
        allowed_optional = set(retained_for_customer[:room])
        retained.difference_update(
            arc
            for arc in list(retained)
            if arc[1] == customer and arc not in protected_arcs and arc not in allowed_optional
        )
    if not protected_arcs.issubset(retained):
        raise ValueError("protected mandatory drone arcs were removed by drone-arc caps")
    return retained


def _build_drone_required_witness(
    *,
    config: InstanceConfig,
    depot_source: Node,
    depot_sink: Node,
    customers: tuple[Node, ...],
    hubs: tuple[Node, ...],
    locations: dict[Node, tuple[float, float]],
    demand: dict[Node, float],
    truck_arcs: set[Arc],
    truck_time: dict[Arc, float],
    mandatory_drone_customers: tuple[Node, ...],
    protected_hub_for_customer: dict[Node, Node],
) -> tuple[tuple[Node, ...], tuple[tuple[tuple[Node, tuple[Node, ...]], ...], ...]]:
    groups: list[dict[str, object]] = []
    mandatory_by_hub = {
        hub: tuple(customer for customer in mandatory_drone_customers if protected_hub_for_customer[customer] == hub)
        for hub in hubs
    }
    for hub in hubs:
        customers_for_hub = mandatory_by_hub[hub]
        for start in range(0, len(customers_for_hub), config.drones_per_truck):
            block = tuple(customers_for_hub[start:start + config.drones_per_truck])
            groups.append(
                {
                    "physical": [hub],
                    "drone_blocks": [(hub, block)],
                    "payload": sum(demand[customer] for customer in block),
                }
            )
    nonmandatory = [customer for customer in customers if customer not in mandatory_drone_customers]
    for customer in sorted(nonmandatory, key=lambda node: (-demand[node], node)):
        feasible_groups = [
            (float(group["payload"]), index)
            for index, group in enumerate(groups)
            if float(group["payload"]) + demand[customer] <= config.truck_payload + CAPACITY_TOLERANCE
        ]
        if feasible_groups:
            _, index = min(feasible_groups)
        else:
            if len(groups) >= config.num_trucks:
                raise ValueError("drone-required witness exceeds truck fleet or payload capacity")
            index = len(groups)
            groups.append({"physical": [], "drone_blocks": [], "payload": 0.0})
        group = groups[index]
        physical = group["physical"]
        if not isinstance(physical, list):
            raise TypeError("witness physical path must be a list")
        physical.append(customer)
        group["payload"] = float(group["payload"]) + demand[customer]
    if len(groups) > config.num_trucks:
        raise ValueError("drone-required witness route count exceeds truck fleet size")
    witness_routes: list[tuple[Node, ...]] = []
    witness_route_drone_blocks: list[tuple[tuple[Node, tuple[Node, ...]], ...]] = []
    for group in groups:
        physical = tuple(group["physical"])
        if not physical:
            continue
        if float(group["payload"]) > config.truck_payload + CAPACITY_TOLERANCE:
            raise ValueError("drone-required witness route exceeds truck payload")
        route_nodes = (depot_source,) + physical + (depot_sink,)
        for i, j in zip(route_nodes, route_nodes[1:]):
            _add_truck_arc(config, locations, i, j, truck_arcs, truck_time)
        witness_routes.append(physical)
        blocks = group["drone_blocks"]
        if not isinstance(blocks, list):
            raise TypeError("witness drone blocks must be a list")
        witness_route_drone_blocks.append(tuple((hub, tuple(block)) for hub, block in blocks))
    covered = set()
    for route in witness_routes:
        covered.update(customer for customer in route if customer in customers)
    for blocks in witness_route_drone_blocks:
        for _, block in blocks:
            covered.update(block)
    if covered != set(customers):
        raise ValueError("drone-required witness does not cover every customer")
    return tuple(witness_routes), tuple(witness_route_drone_blocks)


def instance_generation_metadata(instance: InstanceData) -> dict[str, object]:
    savings = list(instance.drone_arc_saving.values())
    witness_drone_sorties = sum(len(block) for route_blocks in instance.witness_route_drone_blocks for _, block in route_blocks)
    return {
        "truck_arcs": len(instance.truck_arcs),
        "drone_arcs": len(instance.drone_arcs),
        "mandatory_drone_customers": list(instance.mandatory_drone_customers),
        "mandatory_drone_customer_count": len(instance.mandatory_drone_customers),
        "retained_drone_arc_saving_min": min(savings) if savings else None,
        "retained_drone_arc_saving_mean": sum(savings) / len(savings) if savings else None,
        "retained_drone_arc_saving_max": max(savings) if savings else None,
        "witness_route_count": len(instance.witness_routes),
        "witness_drone_sorties": witness_drone_sorties,
        "witness_routes": [list(route) for route in instance.witness_routes],
        "witness_route_drone_blocks": [
            [{"hub": hub, "customers": list(block)} for hub, block in route_blocks]
            for route_blocks in instance.witness_route_drone_blocks
        ],
        "retained_drone_arc_savings": [
            {"hub": hub, "customer": customer, "saving": saving}
            for (hub, customer), saving in sorted(instance.drone_arc_saving.items())
        ],
    }


def _choose_hubs_by_kmeans(seed: int, customer_locations: np.ndarray, count: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    initial = rng.choice(len(customer_locations), size=count, replace=False)
    centers = customer_locations[initial].astype(float).copy()
    for _ in range(100):
        distances = np.linalg.norm(customer_locations[:, None, :] - centers[None, :, :], axis=2)
        labels = np.argmin(distances, axis=1)
        new_centers = centers.copy()
        for idx in range(count):
            members = customer_locations[labels == idx]
            if len(members) == 0:
                farthest = int(np.argmax(np.min(distances, axis=1)))
                new_centers[idx] = customer_locations[farthest]
            else:
                new_centers[idx] = members.mean(axis=0)
        if np.allclose(new_centers, centers):
            break
        centers = new_centers
    return centers


def _sample_demands(
    config: InstanceConfig,
    customers: tuple[Node, ...],
    rng: random.Random,
    np_rng: np.random.Generator,
) -> dict[Node, float]:
    low_count = int(config.low_demand_customer_ratio * len(customers))
    low_customers = set(rng.sample(list(customers), low_count))
    low_values = np.clip(
        np_rng.normal(config.low_demand_weight_mean, config.low_demand_weight_std, size=low_count),
        config.low_demand_weight_min,
        None,
    )
    high_count = len(customers) - low_count
    high_values = np.clip(
        np_rng.normal(config.high_demand_weight_mean, config.high_demand_weight_std, size=high_count),
        config.high_demand_weight_min,
        None,
    )
    demand: dict[Node, float] = {}
    low_index = 0
    high_index = 0
    for customer in customers:
        if customer in low_customers:
            demand[customer] = round(float(low_values[low_index]), 2)
            low_index += 1
        else:
            demand[customer] = round(float(high_values[high_index]), 2)
            high_index += 1
    return demand


def _add_truck_arc(
    config: InstanceConfig,
    locations: dict[Node, tuple[float, float]],
    i: Node,
    j: Node,
    arcs: set[Arc],
    times: dict[Arc, float],
) -> None:
    arcs.add((i, j))
    times[(i, j)] = _truck_time(config, locations[i], locations[j])


def _truck_time(config: InstanceConfig, i: tuple[float, float], j: tuple[float, float]) -> float:
    return (abs(i[0] - j[0]) + abs(i[1] - j[1])) / config.truck_speed * 60.0


def _drone_time(config: InstanceConfig, i: tuple[float, float], j: tuple[float, float]) -> float:
    return hypot(i[0] - j[0], i[1] - j[1]) / config.drone_speed * 60.0
