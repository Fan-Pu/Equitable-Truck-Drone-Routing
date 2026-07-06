from __future__ import annotations

from dataclasses import dataclass
from collections import defaultdict

from .instance import Arc, InstanceData, Node

DUP_PREFIX = "DUP"


def duplicate_node(hub: str, customer: str) -> str:
    return f"{DUP_PREFIX}:{hub}:{customer}"


def is_duplicate(node: str) -> bool:
    return node.startswith(f"{DUP_PREFIX}:")


def duplicate_hub(node: str) -> str:
    return node.split(":")[1]


def duplicate_customer(node: str) -> str:
    return node.split(":")[2]


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
    travel_time: dict[Arc, float]
    order: dict[tuple[Node, Node], int]

    def arc_customer_set(self, arc: Arc) -> frozenset[str]:
        values = set()
        for node in arc:
            if node in self.instance.customers:
                values.add(node)
            elif is_duplicate(node):
                values.add(duplicate_customer(node))
        return frozenset(values)


def build_transformed_graph(instance: InstanceData) -> TransformedGraph:
    order: dict[tuple[str, str], int] = {}
    duplicate_nodes = []
    for hub in instance.hubs:
        access = sorted(customer for customer in instance.customers if (hub, customer) in instance.drone_arcs)
        for idx, customer in enumerate(access):
            order[(hub, customer)] = idx
            duplicate_nodes.append(duplicate_node(hub, customer))

    truck_arcs = set(instance.truck_arcs)
    hub_duplicate_arcs: set[Arc] = set()
    duplicate_duplicate_arcs: set[Arc] = set()
    duplicate_regular_arcs: set[Arc] = set()
    travel_time = dict(instance.truck_time)

    for hub in instance.hubs:
        access = sorted(customer for customer in instance.customers if (hub, customer) in instance.drone_arcs)
        for customer in access:
            dup = duplicate_node(hub, customer)
            arc = (hub, dup)
            hub_duplicate_arcs.add(arc)
            travel_time[arc] = instance.drone_time[(hub, customer)]
            for next_customer in access:
                if order[(hub, customer)] < order[(hub, next_customer)]:
                    next_dup = duplicate_node(hub, next_customer)
                    dd_arc = (dup, next_dup)
                    duplicate_duplicate_arcs.add(dd_arc)
                    travel_time[dd_arc] = instance.drone_time[(hub, next_customer)]
            for _, regular in sorted(a for a in instance.truck_arcs if a[0] == hub):
                dr_arc = (dup, regular)
                duplicate_regular_arcs.add(dr_arc)
                travel_time[dr_arc] = instance.truck_time[(hub, regular)]

    arcs = truck_arcs | hub_duplicate_arcs | duplicate_duplicate_arcs | duplicate_regular_arcs
    nodes = instance.nodes + tuple(duplicate_nodes)
    out_map: dict[str, list[str]] = defaultdict(list)
    in_map: dict[str, list[str]] = defaultdict(list)
    for i, j in sorted(arcs):
        out_map[i].append(j)
        in_map[j].append(i)
    return TransformedGraph(
        instance=instance,
        nodes=nodes,
        duplicate_nodes=tuple(duplicate_nodes),
        arcs=frozenset(arcs),
        truck_arcs=frozenset(truck_arcs),
        hub_duplicate_arcs=frozenset(hub_duplicate_arcs),
        duplicate_duplicate_arcs=frozenset(duplicate_duplicate_arcs),
        duplicate_regular_arcs=frozenset(duplicate_regular_arcs),
        out_arcs={node: tuple(out_map[node]) for node in nodes},
        in_arcs={node: tuple(in_map[node]) for node in nodes},
        travel_time=travel_time,
        order=order,
    )
