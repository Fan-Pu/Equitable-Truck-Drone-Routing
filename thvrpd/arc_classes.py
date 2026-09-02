from __future__ import annotations

from typing import TYPE_CHECKING

from .instance import Arc, InstanceData

if TYPE_CHECKING:
    from .routes import Route


ARC_CLASSES = ("customer_customer", "customer_pad", "pad_customer")


def truck_arc_class(instance: InstanceData, arc: Arc) -> str | None:
    left, right = arc
    if left in instance.customers and right in instance.customers:
        return "customer_customer"
    if left in instance.customers and right in instance.hubs:
        return "customer_pad"
    if left in instance.hubs and right in instance.customers:
        return "pad_customer"
    return None


def allowable_internal_truck_arcs(instance: InstanceData) -> tuple[Arc, ...]:
    mandatory = set(instance.mandatory_drone_customers)
    return tuple(
        sorted(
            (left, right)
            for left in instance.customers + instance.hubs
            for right in instance.customers + instance.hubs
            if left != right
            and left not in mandatory
            and right not in mandatory
            and not (left in instance.hubs and right in instance.hubs)
        )
    )


def truck_arc_class_counts(
    instance: InstanceData,
    arcs: set[Arc] | frozenset[Arc] | tuple[Arc, ...],
) -> dict[str, int]:
    counts = {name: 0 for name in ARC_CLASSES}
    for arc in arcs:
        classification = truck_arc_class(instance, arc)
        if classification is not None:
            counts[classification] += 1
    return counts


def route_internal_truck_arcs(instance: InstanceData, route: Route) -> tuple[Arc, ...]:
    arcs = tuple(zip(route.truck_path, route.truck_path[1:]))
    return tuple(arc for arc in arcs if truck_arc_class(instance, arc) is not None)

