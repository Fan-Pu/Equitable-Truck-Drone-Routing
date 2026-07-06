from __future__ import annotations

from dataclasses import dataclass

from .instance import Arc
from .routes import Route


@dataclass(frozen=True)
class BranchRestrictions:
    together_pairs: frozenset[tuple[str, str]] = frozenset()
    separate_pairs: frozenset[tuple[str, str]] = frozenset()
    truck_service: frozenset[str] = frozenset()
    drone_service: frozenset[str] = frozenset()
    pad_forbidden: frozenset[tuple[str, str]] = frozenset()
    pad_required: frozenset[tuple[str, str]] = frozenset()
    trans_arc_forbidden: frozenset[Arc] = frozenset()
    trans_arc_required: frozenset[Arc] = frozenset()
    route_forbidden: frozenset[tuple[str, ...]] = frozenset()

    def with_together(self, p: str, q: str) -> "BranchRestrictions":
        return _replace(self, together_pairs=self.together_pairs | {tuple(sorted((p, q)))})

    def with_separate(self, p: str, q: str) -> "BranchRestrictions":
        return _replace(self, separate_pairs=self.separate_pairs | {tuple(sorted((p, q)))})

    def with_truck_service(self, customer: str) -> "BranchRestrictions":
        return _replace(self, truck_service=self.truck_service | {customer})

    def with_drone_service(self, customer: str) -> "BranchRestrictions":
        return _replace(self, drone_service=self.drone_service | {customer})

    def with_pad_forbidden(self, hub: str, customer: str) -> "BranchRestrictions":
        return _replace(self, pad_forbidden=self.pad_forbidden | {(hub, customer)})

    def with_pad_required(self, hub: str, customer: str) -> "BranchRestrictions":
        return _replace(self, pad_required=self.pad_required | {(hub, customer)})

    def with_trans_arc_forbidden(self, arc: Arc) -> "BranchRestrictions":
        return _replace(self, trans_arc_forbidden=self.trans_arc_forbidden | {arc})

    def with_trans_arc_required(self, arc: Arc) -> "BranchRestrictions":
        return _replace(self, trans_arc_required=self.trans_arc_required | {arc})

    def with_route_forbidden(self, path: tuple[str, ...]) -> "BranchRestrictions":
        return _replace(self, route_forbidden=self.route_forbidden | {path})

    def route_allowed(self, route: Route, arc_customer_sets: dict[Arc, frozenset[str]]) -> bool:
        if route.path in self.route_forbidden:
            return False
        for p, q in self.together_pairs:
            if (p in route.served) != (q in route.served):
                return False
        for p, q in self.separate_pairs:
            if p in route.served and q in route.served:
                return False
        for customer in self.truck_service:
            if customer in route.served and customer not in route.truck_served:
                return False
        for customer in self.drone_service:
            if customer in route.served and customer in route.truck_served:
                return False
        for hub, customer in self.pad_forbidden:
            if (hub, customer) in route.pad_served:
                return False
        for hub, customer in self.pad_required:
            if customer in route.served and (hub, customer) not in route.pad_served:
                return False
        if route.used_arcs.intersection(self.trans_arc_forbidden):
            return False
        for arc in self.trans_arc_required:
            if route.served.intersection(arc_customer_sets[arc]) and arc not in route.used_arcs:
                return False
        return True


def _replace(restrictions: BranchRestrictions, **changes: object) -> BranchRestrictions:
    values = restrictions.__dict__.copy()
    values.update(changes)
    return BranchRestrictions(**values)
