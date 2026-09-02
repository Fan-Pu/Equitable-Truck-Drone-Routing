from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass

from .instance import Arc, InstanceData, Node

DUP_PREFIX = "DUP"


def duplicate_node(customer: str) -> str:
    return f"{DUP_PREFIX}:{customer}"


def is_duplicate(node: str) -> bool:
    return node.startswith(f"{DUP_PREFIX}:")


def duplicate_customer(node: str) -> str:
    return node.split(":", 1)[1]


def served_customer(node: str) -> str:
    return duplicate_customer(node) if is_duplicate(node) else node


@dataclass(frozen=True)
class TransformedGraph:
    instance: InstanceData
    nodes: tuple[Node, ...]
    duplicate_nodes: tuple[Node, ...]
    arcs: frozenset[Arc]
    truck_arcs: frozenset[Arc]
    hub_duplicate_arcs: frozenset[Arc]
    duplicate_duplicate_arcs: frozenset[Arc]
    duplicate_regular_arcs: frozenset[Arc]
    out_arcs: dict[Node, tuple[Node, ...]]
    in_arcs: dict[Node, tuple[Node, ...]]
    drone_access_by_hub: dict[Node, tuple[Node, ...]]
    truck_successors_by_hub: dict[Node, tuple[Node, ...]]
    order: dict[tuple[Node, Node], int]

    def arc_customer_set(self, arc: Arc) -> frozenset[str]:
        values = set()
        for node in arc:
            if node in self.instance.customers:
                values.add(node)
            elif is_duplicate(node):
                values.add(duplicate_customer(node))
        return frozenset(values)

    def arc_compatible_with_active_pad(self, arc: Arc, active_pad: Node | None) -> bool:
        if arc not in self.arcs:
            return False
        if arc in self.truck_arcs:
            return True
        i, j = arc
        if arc in self.hub_duplicate_arcs:
            return (i, duplicate_customer(j)) in self.instance.drone_arcs
        if active_pad is None:
            return False
        if arc in self.duplicate_duplicate_arcs:
            current = duplicate_customer(i)
            following = duplicate_customer(j)
            return (
                (active_pad, current) in self.instance.drone_arcs
                and (active_pad, following) in self.instance.drone_arcs
                and self.order[(active_pad, current)] < self.order[(active_pad, following)]
            )
        if arc in self.duplicate_regular_arcs:
            current = duplicate_customer(i)
            return (
                (active_pad, current) in self.instance.drone_arcs
                and (active_pad, j) in self.instance.truck_arcs
                and j != current
            )
        raise ValueError(f"unclassified transformed arc: {arc}")

    def realized_travel_time(self, arc: Arc, active_pad: Node | None = None) -> float:
        if not self.arc_compatible_with_active_pad(arc, active_pad):
            raise ValueError(f"transformed arc is incompatible with active pad {active_pad}: {arc}")
        i, j = arc
        if arc in self.truck_arcs:
            return self.instance.truck_time[arc]
        if arc in self.hub_duplicate_arcs:
            return self.instance.drone_trip_time[(i, duplicate_customer(j))]
        if arc in self.duplicate_duplicate_arcs:
            return self.instance.drone_trip_time[(active_pad, duplicate_customer(j))]
        if arc in self.duplicate_regular_arcs:
            return self.instance.truck_time[(active_pad, j)]
        raise ValueError(f"unclassified transformed arc: {arc}")


def build_transformed_graph(instance: InstanceData) -> TransformedGraph:
    customer_rank = {customer: index for index, customer in enumerate(instance.customers)}
    drone_access_by_hub = {
        hub: tuple(customer for customer in instance.customers if (hub, customer) in instance.drone_arcs)
        for hub in instance.hubs
    }
    truck_successors_by_hub = {
        hub: tuple(node for node in instance.nodes if (hub, node) in instance.truck_arcs)
        for hub in instance.hubs
    }
    order = {
        (hub, customer): index
        for hub, access in drone_access_by_hub.items()
        for index, customer in enumerate(access)
    }
    accessible_customers = {
        customer
        for access in drone_access_by_hub.values()
        for customer in access
    }
    duplicate_nodes = tuple(
        duplicate_node(customer)
        for customer in instance.customers
        if customer in accessible_customers
    )

    truck_arcs = set(instance.truck_arcs)
    hub_duplicate_arcs: set[Arc] = set()
    duplicate_duplicate_arcs: set[Arc] = set()
    duplicate_regular_arcs: set[Arc] = set()

    for hub in instance.hubs:
        access = drone_access_by_hub[hub]
        for customer in access:
            duplicate = duplicate_node(customer)
            hub_duplicate_arcs.add((hub, duplicate))
            for following in access:
                if customer_rank[customer] < customer_rank[following]:
                    duplicate_duplicate_arcs.add((duplicate, duplicate_node(following)))
            for regular in truck_successors_by_hub[hub]:
                if regular != customer:
                    duplicate_regular_arcs.add((duplicate, regular))

    arcs = truck_arcs | hub_duplicate_arcs | duplicate_duplicate_arcs | duplicate_regular_arcs
    nodes = instance.nodes + duplicate_nodes
    out_map: dict[str, list[str]] = defaultdict(list)
    in_map: dict[str, list[str]] = defaultdict(list)
    for i, j in sorted(arcs):
        out_map[i].append(j)
        in_map[j].append(i)
    return TransformedGraph(
        instance=instance,
        nodes=nodes,
        duplicate_nodes=duplicate_nodes,
        arcs=frozenset(arcs),
        truck_arcs=frozenset(truck_arcs),
        hub_duplicate_arcs=frozenset(hub_duplicate_arcs),
        duplicate_duplicate_arcs=frozenset(duplicate_duplicate_arcs),
        duplicate_regular_arcs=frozenset(duplicate_regular_arcs),
        out_arcs={node: tuple(out_map[node]) for node in nodes},
        in_arcs={node: tuple(in_map[node]) for node in nodes},
        drone_access_by_hub=drone_access_by_hub,
        truck_successors_by_hub=truck_successors_by_hub,
        order=order,
    )
