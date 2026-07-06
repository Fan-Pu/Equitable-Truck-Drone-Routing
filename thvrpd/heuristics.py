from __future__ import annotations

from dataclasses import asdict, dataclass, field
import time

import gurobipy as gp
from gurobipy import GRB

from .branching import BranchRestrictions
from .config import SolverConfig
from .objective import ObjectiveData
from .pricing import PricingDiagnostics, PricingDuals, PricingTimeLimitReached, SourceNeighborPricingPool, price_route
from .rmp import NodeState
from .routes import Route
from .solverlog import configure_gurobi_logging
from .transform import TransformedGraph


@dataclass(frozen=True)
class RoutePoolHeuristicDiagnostics:
    node_pool_routes: int = 0
    support_routes: int = 0
    max_node_pool_routes: int = 0
    max_support_routes: int = 0
    node_pool_to_support_ratio: float = 0.0
    hard_pool_solves: int = 0
    hard_pool_time: float = 0.0
    hard_pool_feasible_solves: int = 0
    support_pool_calls: int = 0
    support_pool_time: float = 0.0
    support_pool_feasible: int = 0
    support_pool_incumbent_updates: int = 0
    full_pool_calls: int = 0
    full_pool_time: float = 0.0
    full_pool_feasible: int = 0
    full_pool_incumbent_updates: int = 0
    soft_pool_solves: int = 0
    soft_pool_time: float = 0.0
    soft_pool_feasible_solves: int = 0
    repair_customers: int = 0
    repair_columns_generated: int = 0
    repair_budget_hit: int = 0
    heuristic_budget_hit: int = 0
    repair_time_seconds: float = 0.0


@dataclass(frozen=True)
class RoutePoolHeuristicResult:
    value: float | None
    selected_routes: tuple[Route, ...]
    generated_paths: frozenset[tuple[str, ...]]
    pricing_diagnostics: tuple[dict, ...] = tuple()
    diagnostics: RoutePoolHeuristicDiagnostics = field(default_factory=RoutePoolHeuristicDiagnostics)


def run_route_pool_heuristic(
    graph: TransformedGraph,
    objective: ObjectiveData,
    node: NodeState,
    routes: dict[tuple[str, ...], Route],
    global_pool_paths: set[tuple[str, ...]],
    z_values: dict[tuple[str, ...], float],
    solver_config: SolverConfig,
    next_route_id: int,
    incumbent_value: float,
    deadline: float | None = None,
    side_pool_paths: set[tuple[str, ...]] | None = None,
    allow_repair: bool = True,
    repair_time_budget: float | None = None,
    pricing_process_pool: SourceNeighborPricingPool | None = None,
) -> RoutePoolHeuristicResult:
    pool_paths = _node_admissible_pool(graph, node, routes, global_pool_paths, side_pool_paths or set())
    support_paths = _support_route_set(node, routes, pool_paths, z_values, solver_config)
    hard_pool_solves = 0
    hard_pool_time = 0.0
    hard_pool_feasible_solves = 0
    soft_pool_solves = 0
    soft_pool_time = 0.0
    soft_pool_feasible_solves = 0
    repair_customers_count = 0
    repair_columns_generated = 0
    repair_budget_hit = 0
    heuristic_budget_hit = 0
    repair_time_seconds = 0.0
    support_pool_calls = 0
    support_pool_time = 0.0
    support_pool_feasible = 0
    support_pool_incumbent_updates = 0
    full_pool_calls = 0
    full_pool_time = 0.0
    full_pool_feasible = 0
    full_pool_incumbent_updates = 0

    def diagnostics() -> RoutePoolHeuristicDiagnostics:
        pool_ratio = len(pool_paths) / max(len(support_paths), 1)
        return RoutePoolHeuristicDiagnostics(
            node_pool_routes=len(pool_paths),
            support_routes=len(support_paths),
            max_node_pool_routes=len(pool_paths),
            max_support_routes=len(support_paths),
            node_pool_to_support_ratio=pool_ratio,
            hard_pool_solves=hard_pool_solves,
            hard_pool_time=hard_pool_time,
            hard_pool_feasible_solves=hard_pool_feasible_solves,
            support_pool_calls=support_pool_calls,
            support_pool_time=support_pool_time,
            support_pool_feasible=support_pool_feasible,
            support_pool_incumbent_updates=support_pool_incumbent_updates,
            full_pool_calls=full_pool_calls,
            full_pool_time=full_pool_time,
            full_pool_feasible=full_pool_feasible,
            full_pool_incumbent_updates=full_pool_incumbent_updates,
            soft_pool_solves=soft_pool_solves,
            soft_pool_time=soft_pool_time,
            soft_pool_feasible_solves=soft_pool_feasible_solves,
            repair_customers=repair_customers_count,
            repair_columns_generated=repair_columns_generated,
            repair_budget_hit=repair_budget_hit,
            heuristic_budget_hit=heuristic_budget_hit,
            repair_time_seconds=repair_time_seconds,
        )

    hard_start = time.time()
    value, selected = _solve_hard_pool_ip(node, routes, support_paths, z_values, solver_config, deadline)
    elapsed = time.time() - hard_start
    hard_pool_time += elapsed
    support_pool_time += elapsed
    hard_pool_solves += 1
    support_pool_calls += 1
    if value is not None:
        hard_pool_feasible_solves += 1
        support_pool_feasible += 1
    hard_solution = (
        RoutePoolHeuristicResult(value, tuple(routes[path] for path in selected), frozenset(), diagnostics=diagnostics())
        if value is not None
        else RoutePoolHeuristicResult(None, tuple(), frozenset(), diagnostics=diagnostics())
    )
    if value is not None and value < incumbent_value:
        support_pool_incumbent_updates += 1
        return RoutePoolHeuristicResult(value, tuple(routes[path] for path in selected), frozenset(), diagnostics=diagnostics())

    if pool_paths != support_paths:
        full_hard_start = time.time()
        full_value, full_selected = _solve_hard_pool_ip(node, routes, pool_paths, z_values, solver_config, deadline)
        elapsed = time.time() - full_hard_start
        hard_pool_time += elapsed
        full_pool_time += elapsed
        hard_pool_solves += 1
        full_pool_calls += 1
        if full_value is not None:
            hard_pool_feasible_solves += 1
            full_pool_feasible += 1
            if hard_solution.value is None or full_value < hard_solution.value:
                hard_solution = RoutePoolHeuristicResult(
                    full_value,
                    tuple(routes[path] for path in full_selected),
                    frozenset(),
                    diagnostics=diagnostics(),
                )
            if full_value < incumbent_value:
                full_pool_incumbent_updates += 1
                return RoutePoolHeuristicResult(
                    full_value,
                    tuple(routes[path] for path in full_selected),
                    frozenset(),
                    diagnostics=diagnostics(),
                )

    soft_start = time.time()
    soft = _solve_soft_pool_ip(node, routes, support_paths, z_values, solver_config, deadline)
    soft_pool_time += time.time() - soft_start
    soft_pool_solves += 1
    if soft is None:
        return RoutePoolHeuristicResult(
            hard_solution.value,
            hard_solution.selected_routes,
            hard_solution.generated_paths,
            hard_solution.pricing_diagnostics,
            diagnostics(),
        )
    soft_pool_feasible_solves += 1
    soft_selected, repair_customers = soft
    repair_customers_count = len(repair_customers)
    if not repair_customers:
        return RoutePoolHeuristicResult(
            hard_solution.value,
            hard_solution.selected_routes,
            hard_solution.generated_paths,
            hard_solution.pricing_diagnostics,
            diagnostics(),
        )
    if not allow_repair or (repair_time_budget is not None and repair_time_budget <= 0.0):
        repair_budget_hit = 1
        return RoutePoolHeuristicResult(
            hard_solution.value,
            hard_solution.selected_routes,
            hard_solution.generated_paths,
            hard_solution.pricing_diagnostics,
            diagnostics(),
        )

    covered = frozenset().union(*(routes[path].served for path in soft_selected)) if soft_selected else frozenset()
    residual_for_repair = frozenset(node.residual_customers - covered)
    repair_duals = PricingDuals(
        mu={customer: solver_config.repair_reward if customer in repair_customers else 0.0 for customer in residual_for_repair},
        kappa=0.0,
        nu={},
    )
    generated_paths: set[tuple[str, ...]] = set()
    pricing_diagnostics: list[dict] = []
    repair_restrictions: BranchRestrictions = node.restrictions
    repair_start = time.time()
    repair_deadline = deadline
    if repair_time_budget is not None:
        repair_deadline = min(deadline, repair_start + repair_time_budget) if deadline is not None else repair_start + repair_time_budget
    while True:
        if repair_deadline is not None and time.time() >= repair_deadline:
            repair_budget_hit = 1
            break
        _raise_if_deadline(repair_deadline)
        try:
            priced = price_route(
                graph,
                objective,
                residual_for_repair,
                repair_restrictions,
                repair_duals,
                next_route_id + len(generated_paths),
                farkas=False,
                pricing_tolerance=solver_config.pricing_tolerance,
                use_standard_acceleration=solver_config.enable_pricing_pruning,
                stop_at_first_negative=solver_config.repair_batch_size == 1,
                batch_size=solver_config.repair_batch_size,
                deadline=repair_deadline,
                enable_bidirectional=solver_config.enable_bidirectional_pricing,
                parallel_workers=solver_config.pricing_parallel_workers,
                pricing_worker_backend=solver_config.pricing_worker_backend,
                existing_routes=routes,
                existing_column_paths=set(node.column_paths) | generated_paths,
                small_join_pair_threshold=solver_config.small_join_pair_threshold,
                small_join_cumulative_threshold=solver_config.small_join_cumulative_threshold,
                max_join_bypass_calls=solver_config.max_join_bypass_calls,
                small_dom_bucket_threshold=solver_config.small_dom_bucket_threshold,
                small_dom_cumulative_threshold=solver_config.small_dom_cumulative_threshold,
                max_dom_bypass_calls=solver_config.max_dom_bypass_calls,
                join_payload_bin_width=solver_config.join_payload_bin_width,
                join_eval_budget=solver_config.join_eval_budget,
                pricing_certification_slice_seconds=solver_config.pricing_certification_slice_seconds,
                enable_join_lower_envelope=solver_config.enable_join_lower_envelope,
                join_generator_split_threshold=solver_config.join_generator_split_threshold,
                join_generator_pair_batch_size=solver_config.join_generator_pair_batch_size,
                enable_bucket_join_envelope=solver_config.enable_bucket_join_envelope,
                enable_join_profile_cache=solver_config.enable_join_profile_cache,
                side_pool_batch_size=0,
                pricing_process_pool=pricing_process_pool,
                productive_candidate_multiplier=solver_config.productive_candidate_multiplier,
                source_neighbor_task_size=solver_config.source_neighbor_task_size,
                pricing_diversity_batch_fraction=solver_config.pricing_diversity_batch_fraction,
            )
        except PricingTimeLimitReached as exc:
            if deadline is not None and time.time() >= deadline:
                raise
            repair_budget_hit = 1
            pricing_diagnostics.append({"mode": "repair_budget_interrupted", **asdict(exc.diagnostics)})
            break
        pricing_diagnostics.append({"mode": "repair", **asdict(priced.diagnostics)})
        if not priced.routes:
            break
        for route, reduced_cost in zip(priced.routes, priced.reduced_costs):
            if reduced_cost >= -solver_config.pricing_tolerance:
                raise RuntimeError("repair pricing returned a nonnegative column in the entering batch")
            routes.setdefault(route.path, route)
            global_pool_paths.add(route.path)
            generated_paths.add(route.path)
            repair_restrictions = repair_restrictions.with_route_forbidden(route.path)
    repair_time_seconds = time.time() - repair_start
    repair_columns_generated = len(generated_paths)

    if generated_paths:
        pool_paths = _node_admissible_pool(graph, node, routes, global_pool_paths, side_pool_paths or set())
        support_paths = _support_route_set(node, routes, pool_paths, z_values, solver_config)
        hard_start = time.time()
        value, selected = _solve_hard_pool_ip(node, routes, support_paths, z_values, solver_config, deadline)
        elapsed = time.time() - hard_start
        hard_pool_time += elapsed
        support_pool_time += elapsed
        hard_pool_solves += 1
        support_pool_calls += 1
        if value is not None:
            hard_pool_feasible_solves += 1
            support_pool_feasible += 1
            if value < incumbent_value:
                support_pool_incumbent_updates += 1
            return RoutePoolHeuristicResult(
                value,
                tuple(routes[path] for path in selected),
                frozenset(generated_paths),
                tuple(pricing_diagnostics),
                diagnostics(),
            )
    if hard_solution.value is not None:
        return RoutePoolHeuristicResult(
            hard_solution.value,
            hard_solution.selected_routes,
            frozenset(generated_paths),
            tuple(pricing_diagnostics),
            diagnostics(),
        )
    return RoutePoolHeuristicResult(None, tuple(), frozenset(generated_paths), tuple(pricing_diagnostics), diagnostics())


def _node_admissible_pool(
    graph: TransformedGraph,
    node: NodeState,
    routes: dict[tuple[str, ...], Route],
    global_pool_paths: set[tuple[str, ...]],
    side_pool_paths: set[tuple[str, ...]],
) -> set[tuple[str, ...]]:
    arc_customer_sets = {arc: graph.arc_customer_set(arc) for arc in graph.arcs}
    admissible = set()
    for path in node.column_paths | global_pool_paths | side_pool_paths:
        route = routes[path]
        if (
            route.served
            and route.served.issubset(node.residual_customers)
            and node.restrictions.route_allowed(route, arc_customer_sets)
        ):
            admissible.add(path)
    return admissible


def _support_route_set(
    node: NodeState,
    routes: dict[tuple[str, ...], Route],
    pool_paths: set[tuple[str, ...]],
    z_values: dict[tuple[str, ...], float],
    solver_config: SolverConfig,
) -> set[tuple[str, ...]]:
    support = {path for path in pool_paths if z_values.get(path, 0.0) >= solver_config.support_threshold}
    coverage_frequency: dict[str, int] = {customer: 0 for customer in node.residual_customers}
    for path in pool_paths:
        for customer in routes[path].served:
            if customer in coverage_frequency:
                coverage_frequency[customer] += 1
    for customer in node.residual_customers:
        best = sorted(
            (
                _heuristic_primal_score(routes[path], node, z_values, coverage_frequency, solver_config),
                routes[path].cost,
                path,
            )
            for path in pool_paths
            if customer in routes[path].served
        )
        support.update(path for _, _, path in best[: solver_config.support_best_per_customer])
    return support


def _heuristic_primal_score(
    route: Route,
    node: NodeState,
    z_values: dict[tuple[str, ...], float],
    coverage_frequency: dict[str, int],
    solver_config: SolverConfig,
) -> float:
    coverage_gain = len(route.served.intersection(node.residual_customers))
    lp_support = z_values.get(route.path, 0.0)
    redundancy = sum(coverage_frequency.get(customer, 0) for customer in route.served) / max(len(route.served), 1)
    return (
        route.cost
        - solver_config.seed_reward * coverage_gain
        - solver_config.dive_reward * lp_support
        + solver_config.dive_reward * redundancy
    )


def _solve_hard_pool_ip(
    node: NodeState,
    routes: dict[tuple[str, ...], Route],
    support_paths: set[tuple[str, ...]],
    z_values: dict[tuple[str, ...], float],
    solver_config: SolverConfig,
    deadline: float | None,
) -> tuple[float | None, tuple[tuple[str, ...], ...]]:
    _raise_if_deadline(deadline)
    model = gp.Model(f"RPH_{node.id}")
    log_file = None
    if solver_config.gurobi_log_dir is not None:
        log_file = f"{solver_config.gurobi_log_dir}/heuristic_hard_node_{node.id}_cols_{len(support_paths)}.log"
    configure_gurobi_logging(model, log_file)
    model.Params.TimeLimit = _remaining_time_limit(solver_config.route_pool_time_limit, deadline)
    model.Params.Threads = solver_config.threads
    z = {
        path: model.addVar(
            vtype=GRB.BINARY,
            obj=routes[path].cost - solver_config.dive_reward * z_values.get(path, 0.0),
            name=f"h_{routes[path].id}",
        )
        for path in sorted(support_paths)
    }
    model.update()
    for customer in sorted(node.residual_customers):
        model.addConstr(gp.quicksum(var for path, var in z.items() if customer in routes[path].served) == 1.0)
    model.addConstr(gp.quicksum(z.values()) <= node.fleet_limit)
    model.optimize()
    if model.SolCount == 0:
        return None, tuple()
    selected = tuple(path for path, var in z.items() if var.X > 0.5)
    true_value = node.fixed_cost + sum(routes[path].cost for path in selected)
    return true_value, selected


def _solve_soft_pool_ip(
    node: NodeState,
    routes: dict[tuple[str, ...], Route],
    support_paths: set[tuple[str, ...]],
    z_values: dict[tuple[str, ...], float],
    solver_config: SolverConfig,
    deadline: float | None,
) -> tuple[tuple[tuple[str, ...], ...], frozenset[str]] | None:
    _raise_if_deadline(deadline)
    model = gp.Model(f"SRPH_{node.id}")
    log_file = None
    if solver_config.gurobi_log_dir is not None:
        log_file = f"{solver_config.gurobi_log_dir}/heuristic_soft_node_{node.id}_cols_{len(support_paths)}.log"
    configure_gurobi_logging(model, log_file)
    model.Params.TimeLimit = _remaining_time_limit(solver_config.route_pool_time_limit, deadline)
    model.Params.Threads = solver_config.threads
    penalty = (
        sum(abs(routes[path].cost) for path in support_paths)
        + solver_config.dive_reward * sum(abs(z_values.get(path, 0.0)) for path in support_paths)
        + 1.0
    )
    z = {
        path: model.addVar(
            vtype=GRB.BINARY,
            obj=routes[path].cost - solver_config.dive_reward * z_values.get(path, 0.0),
            name=f"sh_{routes[path].id}",
        )
        for path in sorted(support_paths)
    }
    s = {
        customer: model.addVar(vtype=GRB.BINARY, obj=penalty, name=f"uncovered_{customer}")
        for customer in sorted(node.residual_customers)
    }
    model.update()
    for customer in sorted(node.residual_customers):
        model.addConstr(gp.quicksum(var for path, var in z.items() if customer in routes[path].served) + s[customer] == 1.0)
    model.addConstr(gp.quicksum(z.values()) <= node.fleet_limit)
    model.optimize()
    if model.SolCount == 0:
        return None
    selected = tuple(path for path, var in z.items() if var.X > 0.5)
    repair = frozenset(customer for customer, var in s.items() if var.X > 0.5)
    return selected, repair


def _remaining_time_limit(local_limit: float, deadline: float | None) -> float:
    if deadline is None:
        return local_limit
    import time

    remaining = deadline - time.time()
    if remaining <= 0.0:
        raise PricingTimeLimitReached(_empty_timeout_diagnostics())
    return min(local_limit, remaining)


def _raise_if_deadline(deadline: float | None) -> None:
    if deadline is None:
        return
    import time

    if time.time() >= deadline:
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
