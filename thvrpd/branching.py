from __future__ import annotations

from dataclasses import dataclass

from .instance import Arc
from .routes import Route


@dataclass(frozen=True)
class BranchRestrictions:
    together_pairs: frozenset[tuple[str, str]] = frozenset()
    separate_pairs: frozenset[tuple[str, str]] = frozenset()
    pad_forbidden: frozenset[tuple[str, str]] = frozenset()
    pad_required: frozenset[tuple[str, str]] = frozenset()
    conditioned_arc_forbidden: frozenset[tuple[str, str, str]] = frozenset()
    conditioned_arc_required: frozenset[tuple[str, str, str]] = frozenset()

    def with_together(self, p: str, q: str) -> "BranchRestrictions":
        return _replace(self, together_pairs=self.together_pairs | {tuple(sorted((p, q)))})

    def with_separate(self, p: str, q: str) -> "BranchRestrictions":
        return _replace(self, separate_pairs=self.separate_pairs | {tuple(sorted((p, q)))})

    def with_pad_forbidden(self, hub: str, customer: str) -> "BranchRestrictions":
        return _replace(self, pad_forbidden=self.pad_forbidden | {(hub, customer)})

    def with_pad_required(self, hub: str, customer: str) -> "BranchRestrictions":
        return _replace(self, pad_required=self.pad_required | {(hub, customer)})

    def with_conditioned_arc_forbidden(self, customer: str, arc: Arc) -> "BranchRestrictions":
        return _replace(
            self,
            conditioned_arc_forbidden=self.conditioned_arc_forbidden | {(customer, arc[0], arc[1])},
        )

    def with_conditioned_arc_required(self, customer: str, arc: Arc) -> "BranchRestrictions":
        return _replace(
            self,
            conditioned_arc_required=self.conditioned_arc_required | {(customer, arc[0], arc[1])},
        )

    def route_allowed(self, route: Route) -> bool:
        for p, q in self.together_pairs:
            if (p in route.served) != (q in route.served):
                return False
        for p, q in self.separate_pairs:
            if p in route.served and q in route.served:
                return False
        for hub, customer in self.pad_forbidden:
            if (hub, customer) in route.pad_served:
                return False
        for hub, customer in self.pad_required:
            if customer in route.served and (hub, customer) not in route.pad_served:
                return False
        for customer, i, j in self.conditioned_arc_forbidden:
            if customer in route.served and (i, j) in route.used_arcs:
                return False
        for customer, i, j in self.conditioned_arc_required:
            if customer in route.served and (i, j) not in route.used_arcs:
                return False
        return True


def _replace(restrictions: BranchRestrictions, **changes: object) -> BranchRestrictions:
    values = restrictions.__dict__.copy()
    values.update(changes)
    return BranchRestrictions(**values)
