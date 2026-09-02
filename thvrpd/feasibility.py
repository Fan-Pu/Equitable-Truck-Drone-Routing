from __future__ import annotations

from dataclasses import dataclass
from math import ceil, isfinite
import time

import gurobipy as gp
from gurobipy import GRB
import networkx as nx

from .config import ObjectiveWeights
from .instance import InstanceData
from .objective import ObjectiveData, build_objective_data
from .solverlog import configure_gurobi_logging


@dataclass(frozen=True)
class FeasibilityPrecheckResult:
    passed: bool
    witness: str | None
    forced_drone_customers: tuple[str, ...]
    direct_option_count: int
    drone_option_count: int
    elapsed_seconds: float


@dataclass(frozen=True)
class FeasibilitySolveStage:
    dual_reductions: int
    status: str
    status_code: int
    solve_time_seconds: float
    node_count: float
    iteration_count: float
    solution_count: int


@dataclass(frozen=True)
class FeasibilityGateDiagnostics:
    precheck: FeasibilityPrecheckResult
    model_build_time_seconds: float
    variable_count: int
    constraint_count: int
    nonzero_count: int
    legacy_variable_count: int
    fallback_used: bool
    stages: tuple[FeasibilitySolveStage, ...]


@dataclass(frozen=True)
class FeasibilityGateResult:
    feasible: bool
    status: str
    diagnostics: FeasibilityGateDiagnostics


def exact_feasibility_precheck(
    instance: InstanceData,
    objective: ObjectiveData | None = None,
) -> FeasibilityPrecheckResult:
    start = time.time()
    if objective is None:
        objective = _feasibility_objective(instance)
    graph = instance.truck_graph()
    lengths = dict(nx.all_pairs_dijkstra_path_length(graph, weight="weight"))

    def sp(left: str, right: str) -> float:
        return lengths.get(left, {}).get(right, float("inf"))

    total_demand = sum(instance.demand[customer] for customer in instance.customers)
    if ceil(total_demand / instance.truck_payload) > instance.num_trucks:
        return _precheck_failure("fleet_payload_capacity", start)

    direct_options: dict[str, bool] = {}
    drone_options: dict[str, tuple[str, ...]] = {}
    for customer in instance.customers:
        direct_options[customer] = (
            isfinite(sp(instance.depot_source, customer))
            and isfinite(sp(customer, instance.depot_sink))
            and sp(instance.depot_source, customer) <= objective.bounds.service_ub[customer] + 1e-9
        )
        feasible_hubs = tuple(
            hub
            for hub in instance.hubs
            if (
                (hub, customer) in instance.drone_arcs
                and isfinite(sp(instance.depot_source, hub))
                and isfinite(sp(hub, instance.depot_sink))
                and sp(instance.depot_source, hub) + instance.drone_time[(hub, customer)]
                <= objective.bounds.service_ub[customer] + 1e-9
            )
        )
        drone_options[customer] = feasible_hubs
        if not direct_options[customer] and not feasible_hubs:
            return _precheck_failure(
                f"customer_deadline_reachability:{customer}",
                start,
                direct_options,
                drone_options,
            )

    forced_drone = tuple(customer for customer in instance.customers if not direct_options[customer])
    if forced_drone:
        flow = nx.DiGraph()
        source = "__source__"
        sink = "__sink__"
        for customer in forced_drone:
            customer_node = f"customer:{customer}"
            flow.add_edge(source, customer_node, capacity=1)
            for hub in drone_options[customer]:
                flow.add_edge(customer_node, f"hub:{hub}", capacity=1)
        for hub in instance.hubs:
            flow.add_edge(
                f"hub:{hub}",
                sink,
                capacity=instance.num_trucks * instance.drones_per_truck,
            )
        value, _ = nx.maximum_flow(flow, source, sink, capacity="capacity")
        if int(value) != len(forced_drone):
            return _precheck_failure(
                f"forced_drone_pad_capacity:{int(value)}<{len(forced_drone)}",
                start,
                direct_options,
                drone_options,
                forced_drone,
            )

    return FeasibilityPrecheckResult(
        passed=True,
        witness=None,
        forced_drone_customers=forced_drone,
        direct_option_count=sum(direct_options.values()),
        drone_option_count=sum(len(options) for options in drone_options.values()),
        elapsed_seconds=time.time() - start,
    )


def exact_feasibility_check(
    instance: InstanceData,
    *,
    objective: ObjectiveData | None = None,
    log_file: str | None = None,
    time_limit: float | None = None,
    allow_unresolved: bool = False,
) -> FeasibilityGateResult:
    if objective is None:
        objective = _feasibility_objective(instance)
    precheck = exact_feasibility_precheck(instance, objective)
    legacy_variables = legacy_feasibility_variable_count(instance)
    if not precheck.passed:
        return FeasibilityGateResult(
            feasible=False,
            status="infeasible_precheck",
            diagnostics=FeasibilityGateDiagnostics(
                precheck=precheck,
                model_build_time_seconds=0.0,
                variable_count=0,
                constraint_count=0,
                nonzero_count=0,
                legacy_variable_count=legacy_variables,
                fallback_used=False,
                stages=tuple(),
            ),
        )

    build_start = time.time()
    model = _build_aggregated_feasibility_model(instance, objective, log_file, time_limit=time_limit)
    build_time = time.time() - build_start
    stages = [_optimize_feasibility_stage(model, dual_reductions=1)]
    fallback_used = False
    feasible = model.SolCount > 0
    if not feasible and model.Status == GRB.INF_OR_UNBD:
        fallback_used = True
        if time_limit is not None:
            remaining = time_limit - sum(stage.solve_time_seconds for stage in stages)
            if remaining <= 0.0:
                return _unresolved_gate_result(precheck, model, build_time, legacy_variables, stages, fallback_used)
        model.reset()
        model.Params.DualReductions = 0
        if time_limit is not None:
            model.Params.TimeLimit = remaining
        stages.append(_optimize_feasibility_stage(model, dual_reductions=0))
        feasible = model.SolCount > 0

    if feasible:
        status = "feasible"
    elif model.Status == GRB.INFEASIBLE:
        status = "infeasible"
    elif model.Status == GRB.TIME_LIMIT and allow_unresolved:
        return _unresolved_gate_result(precheck, model, build_time, legacy_variables, stages, fallback_used)
    else:
        raise RuntimeError(f"unexpected aggregate feasibility status {_status_name(model.Status)}")

    diagnostics = FeasibilityGateDiagnostics(
        precheck=precheck,
        model_build_time_seconds=build_time,
        variable_count=int(model.NumVars),
        constraint_count=int(model.NumConstrs),
        nonzero_count=int(model.NumNZs),
        legacy_variable_count=legacy_variables,
        fallback_used=fallback_used,
        stages=tuple(stages),
    )
    return FeasibilityGateResult(feasible=feasible, status=status, diagnostics=diagnostics)


def legacy_feasibility_variable_count(instance: InstanceData) -> int:
    trucks = instance.num_trucks
    drones = instance.drones_per_truck
    customers = len(instance.customers)
    hubs = len(instance.hubs)
    nodes = len(instance.nodes)
    physical = customers + hubs
    return (
        len(instance.truck_arcs) * trucks
        + len(instance.drone_arcs) * trucks * drones
        + trucks
        + hubs * trucks
        + hubs * trucks
        + nodes * trucks
        + customers * trucks * drones
        + customers
        + nodes * trucks
        + physical * trucks
    )


def _build_aggregated_feasibility_model(
    instance: InstanceData,
    objective: ObjectiveData,
    log_file: str | None,
    *,
    time_limit: float | None = None,
) -> gp.Model:
    model = gp.Model("THVRPD_aggregate_feasibility")
    configure_gurobi_logging(model, log_file)
    model.Params.Threads = 0
    model.Params.SolutionLimit = 1
    if time_limit is not None:
        model.Params.TimeLimit = time_limit
    trucks = range(instance.num_trucks)
    physical = instance.customers + instance.hubs
    route_time_ub = objective.bounds.route_time_ub
    max_truck_time = max(instance.truck_time.values())
    max_drone_trip = max(instance.drone_trip_time.values(), default=0.0)
    max_drone_oneway = max((instance.drone_time[arc] for arc in instance.drone_arcs), default=0.0)
    hub_wait_ub = {
        hub: max(
            (instance.drone_trip_time[(h, customer)] for h, customer in instance.drone_arcs if h == hub),
            default=0.0,
        )
        for hub in instance.hubs
    }
    max_direct_demand = max(instance.demand[customer] for customer in instance.customers)
    max_pad_block_demand = max(
        (
            min(
                sum(instance.demand[customer] for h, customer in instance.drone_arcs if h == hub),
                instance.drones_per_truck * instance.drone_payload,
                instance.truck_payload,
            )
            for hub in instance.hubs
        ),
        default=0.0,
    )
    big_m_load = instance.truck_payload + max(max_direct_demand, max_pad_block_demand)
    big_m_order = len(physical) + 1.0
    big_m_wait = max_drone_trip
    big_m_time = route_time_ub + max(max_drone_trip + max_truck_time, max_drone_oneway)

    x = model.addVars(instance.truck_arcs, trucks, vtype=GRB.BINARY, name="x")
    y = model.addVars(instance.drone_arcs, trucks, vtype=GRB.BINARY, name="y")
    used = model.addVars(trucks, vtype=GRB.BINARY, name="used")
    pad_active = model.addVars(instance.hubs, trucks, vtype=GRB.BINARY, name="pad")
    wait = model.addVars(
        instance.hubs,
        trucks,
        lb=0.0,
        ub={(hub, truck): hub_wait_ub[hub] for hub in instance.hubs for truck in trucks},
        name="wait",
    )
    arrive = model.addVars(instance.nodes, trucks, lb=0.0, ub=route_time_ub, name="arrive")
    service = model.addVars(instance.customers, lb=0.0, name="service")
    payload = model.addVars(instance.nodes, trucks, lb=0.0, ub=instance.truck_payload, name="payload")
    order = model.addVars(physical, trucks, lb=0.0, ub=len(physical), name="order")

    outgoing = {node: [(i, j) for i, j in instance.truck_arcs if i == node] for node in instance.nodes}
    incoming = {node: [(i, j) for i, j in instance.truck_arcs if j == node] for node in instance.nodes}
    drone_from = {
        hub: [(h, customer) for h, customer in instance.drone_arcs if h == hub]
        for hub in instance.hubs
    }
    drone_to = {
        customer: [(hub, c) for hub, c in instance.drone_arcs if c == customer]
        for customer in instance.customers
    }

    for truck in trucks:
        model.addConstr(
            gp.quicksum(x[i, j, truck] for i, j in outgoing[instance.depot_source]) == used[truck]
        )
        model.addConstr(
            gp.quicksum(x[i, j, truck] for i, j in incoming[instance.depot_sink]) == used[truck]
        )
        for node in physical:
            out_expr = gp.quicksum(x[i, j, truck] for i, j in outgoing[node])
            in_expr = gp.quicksum(x[i, j, truck] for i, j in incoming[node])
            model.addConstr(out_expr == in_expr)
            model.addConstr(in_expr <= 1.0)
        for hub in instance.hubs:
            hub_visit = gp.quicksum(x[i, j, truck] for i, j in incoming[hub])
            launched = gp.quicksum(y[h, customer, truck] for h, customer in drone_from[hub])
            model.addConstr(pad_active[hub, truck] == hub_visit)
            model.addConstr(launched <= instance.drones_per_truck * pad_active[hub, truck])
            for h, customer in drone_from[hub]:
                model.addConstr(
                    wait[hub, truck] >= instance.drone_trip_time[(hub, customer)] * y[h, customer, truck]
                )
            model.addConstr(wait[hub, truck] <= big_m_wait * launched)
        model.addConstr(arrive[instance.depot_source, truck] == 0.0)
        model.addConstr(payload[instance.depot_source, truck] == 0.0)

    for truck in range(instance.num_trucks - 1):
        model.addConstr(used[truck] >= used[truck + 1])
    first_order = {node: index + 1 for index, node in enumerate(physical)}
    first_rank = {
        truck: gp.quicksum(
            first_order[j] * x[i, j, truck]
            for i, j in outgoing[instance.depot_source]
            if j in first_order
        )
        for truck in trucks
    }
    for truck in range(instance.num_trucks - 1):
        model.addConstr(
            first_rank[truck]
            <= first_rank[truck + 1] + (len(physical) + 1) * (1 - used[truck + 1])
        )

    for customer in instance.customers:
        truck_service = gp.quicksum(
            x[i, j, truck]
            for i, j in incoming[customer]
            for truck in trucks
        )
        drone_service = gp.quicksum(
            y[hub, c, truck]
            for hub, c in drone_to[customer]
            for truck in trucks
        )
        model.addConstr(truck_service + drone_service == 1.0)

    for truck in trucks:
        for i, j in instance.truck_arcs:
            wait_expr = wait[i, truck] if i in instance.hubs else 0.0
            model.addConstr(
                arrive[j, truck]
                >= arrive[i, truck] + wait_expr + instance.truck_time[(i, j)]
                - big_m_time * (1 - x[i, j, truck])
            )
            model.addConstr(
                arrive[j, truck]
                <= arrive[i, truck] + wait_expr + instance.truck_time[(i, j)]
                + big_m_time * (1 - x[i, j, truck])
            )
            if i in physical and j in physical:
                model.addConstr(
                    order[j, truck] >= order[i, truck] + 1.0 - big_m_order * (1 - x[i, j, truck])
                )
            elif i == instance.depot_source and j in physical:
                model.addConstr(order[j, truck] >= 1.0 - big_m_order * (1 - x[i, j, truck]))
            if j in instance.customers:
                model.addConstr(
                    payload[j, truck]
                    >= payload[i, truck] + instance.demand[j] - big_m_load * (1 - x[i, j, truck])
                )
            elif j in instance.hubs:
                launched_load = gp.quicksum(
                    instance.demand[customer] * y[h, customer, truck]
                    for h, customer in drone_from[j]
                )
                model.addConstr(
                    payload[j, truck]
                    >= payload[i, truck] + launched_load - big_m_load * (1 - x[i, j, truck])
                )
        for customer in instance.customers:
            visit = gp.quicksum(x[i, j, truck] for i, j in incoming[customer])
            model.addConstr(service[customer] >= arrive[customer, truck] - big_m_time * (1 - visit))
        for hub, customer in instance.drone_arcs:
            model.addConstr(
                service[customer]
                >= arrive[hub, truck] + instance.drone_time[(hub, customer)]
                - big_m_time * (1 - y[hub, customer, truck])
            )

    for customer in instance.customers:
        model.addConstr(service[customer] >= objective.bounds.arrival_lb[customer])
        model.addConstr(service[customer] <= objective.bounds.service_ub[customer])

    model.setObjective(0.0, GRB.MINIMIZE)
    model.update()
    return model


def _optimize_feasibility_stage(model: gp.Model, dual_reductions: int) -> FeasibilitySolveStage:
    start = time.time()
    model.optimize()
    return FeasibilitySolveStage(
        dual_reductions=dual_reductions,
        status=_status_name(model.Status),
        status_code=int(model.Status),
        solve_time_seconds=time.time() - start,
        node_count=float(model.NodeCount),
        iteration_count=float(model.IterCount),
        solution_count=int(model.SolCount),
    )


def _precheck_failure(
    witness: str,
    start: float,
    direct_options: dict[str, bool] | None = None,
    drone_options: dict[str, tuple[str, ...]] | None = None,
    forced_drone: tuple[str, ...] = tuple(),
) -> FeasibilityPrecheckResult:
    return FeasibilityPrecheckResult(
        passed=False,
        witness=witness,
        forced_drone_customers=forced_drone,
        direct_option_count=sum((direct_options or {}).values()),
        drone_option_count=sum(len(options) for options in (drone_options or {}).values()),
        elapsed_seconds=time.time() - start,
    )


def _unresolved_gate_result(
    precheck: FeasibilityPrecheckResult,
    model: gp.Model,
    build_time: float,
    legacy_variables: int,
    stages: list[FeasibilitySolveStage],
    fallback_used: bool,
) -> FeasibilityGateResult:
    return FeasibilityGateResult(
        feasible=False,
        status="unresolved",
        diagnostics=FeasibilityGateDiagnostics(
            precheck=precheck,
            model_build_time_seconds=build_time,
            variable_count=int(model.NumVars),
            constraint_count=int(model.NumConstrs),
            nonzero_count=int(model.NumNZs),
            legacy_variable_count=legacy_variables,
            fallback_used=fallback_used,
            stages=tuple(stages),
        ),
    )


def _feasibility_objective(instance: InstanceData) -> ObjectiveData:
    weights = ObjectiveWeights(delay=1.0 / 3.0, return_time=1.0 / 3.0, cost=1.0 / 3.0)
    return build_objective_data(instance, weights)


def _status_name(status: int) -> str:
    if status == GRB.OPTIMAL:
        return "optimal"
    if status == GRB.SOLUTION_LIMIT:
        return "solution_limit"
    if status == GRB.INFEASIBLE:
        return "infeasible"
    if status == GRB.INF_OR_UNBD:
        return "inf_or_unbd"
    if status == GRB.UNBOUNDED:
        return "unbounded"
    if status == GRB.TIME_LIMIT:
        return "time_limit"
    return f"status_{status}"
