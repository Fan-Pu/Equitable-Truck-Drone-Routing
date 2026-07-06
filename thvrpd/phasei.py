from __future__ import annotations

from dataclasses import dataclass

import gurobipy as gp
from gurobipy import GRB

from .config import SolverConfig
from .rmp import NodeState
from .routes import Route
from .solverlog import configure_gurobi_logging
from .transform import TransformedGraph


@dataclass(frozen=True)
class PhaseIResult:
    objective: float
    uncovered: frozenset[str]


class PhaseISeeder:
    def __init__(
        self,
        graph: TransformedGraph,
        node: NodeState,
        routes: dict[tuple[str, ...], Route],
        solver_config: SolverConfig,
    ) -> None:
        self.graph = graph
        self.node = node
        self.routes = routes
        self.solver_config = solver_config
        self.model = gp.Model(f"PhaseI_{node.id}")
        log_file = None
        if solver_config.gurobi_log_dir is not None:
            log_file = (
                f"{solver_config.gurobi_log_dir}/"
                f"phase_i_node_{node.id}_cols_{len(node.column_paths)}.log"
            )
        configure_gurobi_logging(self.model, log_file)
        self.model.Params.Threads = solver_config.threads
        self.model.ModelSense = GRB.MINIMIZE
        self.z: dict[tuple[str, ...], gp.Var] = {}
        self.slack: dict[str, gp.Var] = {}
        self._build()

    def _build(self) -> None:
        arc_customer_sets = {arc: self.graph.arc_customer_set(arc) for arc in self.graph.arcs}
        for path in sorted(self.node.column_paths):
            route = self.routes[path]
            if (
                route.served
                and route.served.issubset(self.node.residual_customers)
                and self.node.restrictions.route_allowed(route, arc_customer_sets)
            ):
                self.z[path] = self.model.addVar(lb=0.0, obj=0.0, name=f"z_{route.id}")
        self.slack = {
            customer: self.model.addVar(lb=0.0, obj=1.0, name=f"phase_i_slack_{customer}")
            for customer in sorted(self.node.residual_customers)
        }
        self.model.update()
        for customer in sorted(self.node.residual_customers):
            expr = gp.quicksum(var for path, var in self.z.items() if customer in self.routes[path].served)
            self.model.addConstr(expr + self.slack[customer] == 1.0, name=f"phase_i_cover_{customer}")
        self.model.addConstr(gp.quicksum(self.z.values()) <= self.node.fleet_limit, name="phase_i_fleet")
        self.model.update()

    def solve(self) -> PhaseIResult:
        self.model.optimize()
        if self.model.Status != GRB.OPTIMAL:
            raise RuntimeError(f"unexpected Phase-I status {self.model.Status}")
        uncovered = frozenset(
            customer
            for customer, var in self.slack.items()
            if var.X > self.solver_config.integrality_tolerance
        )
        return PhaseIResult(self.model.ObjVal, uncovered)
