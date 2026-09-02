from __future__ import annotations

from dataclasses import dataclass, field
import time

import gurobipy as gp
from gurobipy import GRB

from .config import SolverConfig
from .objective import ObjectiveData
from .pricing import (
    PricingDiagnostics,
    PricingTimeLimitReached,
)
from .rmp import NodeState
from .routes import Route
from .solverlog import configure_gurobi_logging
from .transform import TransformedGraph


@dataclass(frozen=True)
class RoutePoolHeuristicDiagnostics:
    node_pool_routes: int = 0
    max_node_pool_routes: int = 0
    hard_pool_solves: int = 0
    hard_pool_time: float = 0.0
    hard_pool_feasible_solves: int = 0
    full_pool_calls: int = 0
    full_pool_time: float = 0.0
    full_pool_feasible: int = 0
    full_pool_incumbent_updates: int = 0


@dataclass(frozen=True)
class RoutePoolHeuristicResult:
    value: float | None
    selected_routes: tuple[Route, ...]
    diagnostics: RoutePoolHeuristicDiagnostics = field(default_factory=RoutePoolHeuristicDiagnostics)


def run_route_pool_heuristic(
    graph: TransformedGraph,
    objective: ObjectiveData,
    node: NodeState,
    routes: dict[tuple[str, ...], Route],
    global_pool_paths: set[tuple[str, ...]],
    solver_config: SolverConfig,
    incumbent_value: float,
    deadline: float | None = None,
) -> RoutePoolHeuristicResult:
    pool_paths = _node_admissible_pool(graph, node, routes, global_pool_paths)
    hard_pool_solves = 0
    hard_pool_time = 0.0
    hard_pool_feasible_solves = 0
    full_pool_incumbent_updates = 0

    def diagnostics() -> RoutePoolHeuristicDiagnostics:
        return RoutePoolHeuristicDiagnostics(
            node_pool_routes=len(pool_paths),
            max_node_pool_routes=len(pool_paths),
            hard_pool_solves=hard_pool_solves,
            hard_pool_time=hard_pool_time,
            hard_pool_feasible_solves=hard_pool_feasible_solves,
            full_pool_calls=hard_pool_solves,
            full_pool_time=hard_pool_time,
            full_pool_feasible=hard_pool_feasible_solves,
            full_pool_incumbent_updates=full_pool_incumbent_updates,
        )

    hard_start = time.time()
    value, selected = _solve_hard_pool_ip(node, routes, pool_paths, solver_config, deadline)
    hard_pool_time += time.time() - hard_start
    hard_pool_solves += 1
    if value is not None:
        hard_pool_feasible_solves += 1
        if value < incumbent_value:
            full_pool_incumbent_updates += 1
        return RoutePoolHeuristicResult(
            value,
            tuple(routes[path] for path in selected),
            diagnostics=diagnostics(),
        )
    return RoutePoolHeuristicResult(None, tuple(), diagnostics=diagnostics())


def _node_admissible_pool(
    graph: TransformedGraph,
    node: NodeState,
    routes: dict[tuple[str, ...], Route],
    global_pool_paths: set[tuple[str, ...]],
) -> set[tuple[str, ...]]:
    return {
        path
        for path in node.column_paths | global_pool_paths
        if routes[path].served
        and routes[path].served.issubset(node.residual_customers)
        and node.restrictions.route_allowed(routes[path])
    }


def _solve_hard_pool_ip(
    node: NodeState,
    routes: dict[tuple[str, ...], Route],
    pool_paths: set[tuple[str, ...]],
    solver_config: SolverConfig,
    deadline: float | None,
) -> tuple[float | None, tuple[tuple[str, ...], ...]]:
    _raise_if_deadline(deadline)
    model = gp.Model(f"FRPH_{node.id}")
    log_file = f"{solver_config.gurobi_log_dir}/heuristic_full_node_{node.id}_cols_{len(pool_paths)}.log" if solver_config.gurobi_log_dir else None
    configure_gurobi_logging(model, log_file)
    model.Params.TimeLimit = _remaining_time_limit(solver_config.route_pool_time_limit, deadline)
    model.Params.Threads = solver_config.threads
    z = {
        path: model.addVar(vtype=GRB.BINARY, obj=routes[path].cost, name=f"h_{routes[path].id}")
        for path in sorted(pool_paths)
    }
    model.update()
    for customer in sorted(node.residual_customers):
        model.addConstr(gp.quicksum(var for path, var in z.items() if customer in routes[path].served) == 1.0)
    model.addConstr(gp.quicksum(z.values()) <= node.fleet_limit)
    model.optimize()
    if model.SolCount == 0:
        return None, tuple()
    selected = tuple(path for path, var in z.items() if var.X > 0.5)
    return node.fixed_cost + sum(routes[path].cost for path in selected), selected


def _remaining_time_limit(local_limit: float, deadline: float | None) -> float:
    if deadline is None:
        return local_limit
    remaining = deadline - time.time()
    if remaining <= 0.0:
        raise PricingTimeLimitReached(_empty_timeout_diagnostics())
    return min(local_limit, remaining)


def _raise_if_deadline(deadline: float | None) -> None:
    if deadline is not None and time.time() >= deadline:
        raise PricingTimeLimitReached(_empty_timeout_diagnostics())


def _empty_timeout_diagnostics() -> PricingDiagnostics:
    return PricingDiagnostics(
        labels_generated=0,
        labels_dominated=0,
        labels_pruned=0,
        max_queue_size=0,
        complete_routes_generated=0,
        returned_routes=0,
        best_reduced_cost=None,
        exact_completion=False,
        termination_reason="time_limit_unresolved",
        certification_mode="not_certified_time_limit",
    )
