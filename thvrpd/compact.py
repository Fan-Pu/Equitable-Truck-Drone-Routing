from __future__ import annotations

from dataclasses import dataclass
import time
from typing import TYPE_CHECKING

import gurobipy as gp
from gurobipy import GRB

from .config import ObjectiveWeights
from .instance import InstanceData
from .objective import ObjectiveData, build_objective_data
from .solverlog import configure_gurobi_logging
from .transform import build_transformed_graph, duplicate_node, is_duplicate

if TYPE_CHECKING:
    from .feasibility import FeasibilityGateResult


@dataclass(frozen=True)
class CompactTiming:
    model_build_time: float
    solve_time: float
    route_decode_time: float


@dataclass(frozen=True)
class CompactSolution:
    objective_full: float | None
    route_paths: tuple[tuple[str, ...], ...]
    timing: CompactTiming
    status: str = "unknown"
    objective_bound_full: float | None = None
    mip_gap: float | None = None
    status_code: int | None = None
    node_count: float | None = None
    iteration_count: float | None = None
    first_incumbent_time: float | None = None


def _common_time_big_m(
    route_time_ub: float,
    max_truck_time: float,
    max_drone_trip: float,
    max_drone_oneway: float,
) -> float:
    return route_time_ub + max(max_drone_trip + max_truck_time, max_drone_oneway)


def solve_compact_miqp(
    instance: InstanceData,
    weights: ObjectiveWeights,
    time_limit: float = 1800.0,
    log_file: str | None = None,
) -> float:
    solution = solve_compact_solution(instance, weights, time_limit=time_limit, require_optimal=True, log_file=log_file)
    if solution.objective_full is None:
        raise RuntimeError("compact model did not return an objective")
    return solution.objective_full


def solve_compact_solution(
    instance: InstanceData,
    weights: ObjectiveWeights,
    time_limit: float | None = 1800.0,
    threads: int = 0,
    require_optimal: bool = True,
    log_file: str | None = None,
    objective: ObjectiveData | None = None,
    feasibility_only: bool = False,
    solution_limit: int | None = None,
    dual_reductions: int | None = None,
    decode_routes: bool = True,
) -> CompactSolution:
    build_start = time.time()
    if objective is None:
        objective = build_objective_data(instance, weights)
    model = gp.Model("THVRPD_compact")
    configure_gurobi_logging(model, log_file)
    if time_limit is not None:
        model.Params.TimeLimit = time_limit
    model.Params.Threads = threads
    if solution_limit is not None:
        model.Params.SolutionLimit = solution_limit
    if dual_reductions is not None:
        model.Params.DualReductions = dual_reductions
    trucks = range(instance.num_trucks)
    drones = range(instance.drones_per_truck)
    physical = instance.customers + instance.hubs
    route_time_ub = objective.bounds.route_time_ub
    max_truck_time = max(instance.truck_time.values())
    max_drone_trip = max(instance.drone_trip_time.values(), default=0.0)
    hub_wait_ub = {
        h: max(
            (instance.drone_trip_time[(hh, c)] for hh, c in instance.drone_arcs if hh == h),
            default=0.0,
        )
        for h in instance.hubs
    }
    max_drone_oneway = max((instance.drone_time[arc] for arc in instance.drone_arcs), default=0.0)
    max_direct_demand = max(instance.demand[c] for c in instance.customers)
    max_pad_block_demand = max(
        (
            min(
                sum(instance.demand[c] for hh, c in instance.drone_arcs if hh == h),
                instance.drones_per_truck * instance.drone_payload,
                instance.truck_payload,
            )
            for h in instance.hubs
        ),
        default=0.0,
    )
    big_m_load = instance.truck_payload + max(max_direct_demand, max_pad_block_demand)
    big_m_order = len(physical) + 1.0
    big_m_wait = max_drone_trip
    big_m_time = _common_time_big_m(
        route_time_ub,
        max_truck_time,
        max_drone_trip,
        max_drone_oneway,
    )

    x = model.addVars(instance.truck_arcs, trucks, vtype=GRB.BINARY, name="x")
    y = model.addVars(instance.drone_arcs, trucks, drones, vtype=GRB.BINARY, name="y")
    used = model.addVars(trucks, vtype=GRB.BINARY, name="used")
    pad_active = model.addVars(instance.hubs, trucks, vtype=GRB.BINARY, name="pad")
    wait = model.addVars(
        instance.hubs,
        trucks,
        lb=0.0,
        ub={(h, k): hub_wait_ub[h] for h in instance.hubs for k in trucks},
        name="wait",
    )
    arrive = model.addVars(instance.nodes, trucks, lb=0.0, ub=route_time_ub, name="arrive")
    drone_arrive = model.addVars(instance.customers, trucks, drones, lb=0.0, ub=route_time_ub + max_drone_oneway, name="drone_arrive")
    service = model.addVars(instance.customers, lb=0.0, name="service")
    payload = model.addVars(instance.nodes, trucks, lb=0.0, ub=instance.truck_payload, name="payload")
    order = model.addVars(physical, trucks, lb=0.0, ub=len(physical), name="order")

    for k in trucks:
        model.addConstr(gp.quicksum(x[instance.depot_source, j, k] for _, j in instance.truck_arcs if _ == instance.depot_source) == used[k])
        model.addConstr(gp.quicksum(x[i, instance.depot_sink, k] for i, _ in instance.truck_arcs if _ == instance.depot_sink) == used[k])
        model.addConstr(gp.quicksum(x[instance.depot_source, j, k] for _, j in instance.truck_arcs if _ == instance.depot_source) <= 1.0)
        for node in physical:
            out_expr = gp.quicksum(x[node, j, k] for i, j in instance.truck_arcs if i == node)
            in_expr = gp.quicksum(x[i, node, k] for i, j in instance.truck_arcs if j == node)
            model.addConstr(out_expr == in_expr)
            model.addConstr(in_expr <= 1.0)
            model.addConstr(out_expr <= 1.0)
        for h in instance.hubs:
            hub_visit = gp.quicksum(x[i, h, k] for i, j in instance.truck_arcs if j == h)
            model.addConstr(pad_active[h, k] == hub_visit)
            for d in drones:
                model.addConstr(gp.quicksum(y[h, c, k, d] for hh, c in instance.drone_arcs if hh == h) <= pad_active[h, k])
            model.addConstr(
                gp.quicksum(y[h, c, k, d] for hh, c in instance.drone_arcs if hh == h for d in drones)
                <= instance.drones_per_truck * pad_active[h, k]
            )
            for h2, c in instance.drone_arcs:
                if h2 == h:
                    for d in drones:
                        model.addConstr(wait[h, k] >= instance.drone_trip_time[(h, c)] * y[h, c, k, d])
                        model.addConstr(instance.drone_trip_time[(h, c)] * y[h, c, k, d] <= instance.drone_endurance)
                        model.addConstr(instance.demand[c] * y[h, c, k, d] <= instance.drone_payload)
            model.addConstr(
                wait[h, k]
                <= big_m_wait * gp.quicksum(y[h, c, k, d] for hh, c in instance.drone_arcs if hh == h for d in drones)
            )
        model.addConstr(arrive[instance.depot_source, k] == 0.0)
        model.addConstr(payload[instance.depot_source, k] == 0.0)

    for c in instance.customers:
        truck_service = gp.quicksum(x[i, c, k] for i, j in instance.truck_arcs if j == c for k in trucks)
        drone_service = gp.quicksum(y[h, c, k, d] for h, j in instance.drone_arcs if j == c for k in trucks for d in drones)
        model.addConstr(truck_service + drone_service == 1.0)

    for k in trucks:
        for i, j in instance.truck_arcs:
            wait_expr = wait[i, k] if i in instance.hubs else 0.0
            model.addConstr(arrive[j, k] >= arrive[i, k] + wait_expr + instance.truck_time[(i, j)] - big_m_time * (1 - x[i, j, k]))
            model.addConstr(arrive[j, k] <= arrive[i, k] + wait_expr + instance.truck_time[(i, j)] + big_m_time * (1 - x[i, j, k]))
            if i in physical and j in physical:
                model.addConstr(order[j, k] >= order[i, k] + 1.0 - big_m_order * (1 - x[i, j, k]))
            elif i == instance.depot_source and j in physical:
                model.addConstr(order[j, k] >= 1.0 - big_m_order * (1 - x[i, j, k]))
            if j in instance.customers:
                model.addConstr(payload[j, k] >= payload[i, k] + instance.demand[j] - big_m_load * (1 - x[i, j, k]))
            elif j in instance.hubs:
                launched_load = gp.quicksum(instance.demand[c] * y[j, c, k, d] for hh, c in instance.drone_arcs if hh == j for d in drones)
                model.addConstr(payload[j, k] >= payload[i, k] + launched_load - big_m_load * (1 - x[i, j, k]))
        for c in instance.customers:
            visit_c = gp.quicksum(x[i, c, k] for i, j in instance.truck_arcs if j == c)
            model.addConstr(service[c] >= arrive[c, k] - big_m_time * (1 - visit_c))
        for h, c in instance.drone_arcs:
            for d in drones:
                model.addConstr(drone_arrive[c, k, d] >= arrive[h, k] + instance.drone_time[(h, c)] - big_m_time * (1 - y[h, c, k, d]))
                model.addConstr(drone_arrive[c, k, d] <= arrive[h, k] + instance.drone_time[(h, c)] + big_m_time * (1 - y[h, c, k, d]))
                model.addConstr(service[c] >= drone_arrive[c, k, d] - big_m_time * (1 - y[h, c, k, d]))

    for c in instance.customers:
        model.addConstr(service[c] >= objective.bounds.arrival_lb[c])
        model.addConstr(service[c] <= objective.bounds.service_ub[c])

    delay_term = gp.quicksum((service[c] - objective.bounds.arrival_lb[c]) * (service[c] - objective.bounds.arrival_lb[c]) for c in instance.customers)
    return_term = gp.quicksum(arrive[instance.depot_sink, k] for k in trucks)
    cost_term = instance.truck_cost * gp.quicksum(used[k] for k in trucks) + instance.drone_cost * gp.quicksum(y[h, c, k, d] for h, c in instance.drone_arcs for k in trucks for d in drones)
    if feasibility_only:
        model.setObjective(0.0, GRB.MINIMIZE)
    else:
        model.setObjective(
            objective.coeffs.delay * delay_term
            + objective.coeffs.return_time * return_term
            + objective.coeffs.cost * cost_term
            + objective.coeffs.shift,
            GRB.MINIMIZE,
        )
    build_time = time.time() - build_start
    solve_start = time.time()
    first_incumbent_time: float | None = None

    def incumbent_callback(callback_model, where) -> None:
        nonlocal first_incumbent_time
        if where == GRB.Callback.MIPSOL and first_incumbent_time is None:
            first_incumbent_time = float(callback_model.cbGet(GRB.Callback.RUNTIME))

    model.optimize(incumbent_callback)
    solve_time = time.time() - solve_start
    objective_bound = model.ObjBound
    mip_gap = model.MIPGap if model.SolCount > 0 else None
    status_code = int(model.Status)
    node_count = model.NodeCount
    iteration_count = model.IterCount
    if require_optimal and model.Status != GRB.OPTIMAL:
        raise RuntimeError(f"compact model status {model.Status}")
    if model.SolCount == 0:
        return CompactSolution(
            None,
            tuple(),
            CompactTiming(build_time, solve_time, 0.0),
            _compact_status_name(model.Status),
            objective_bound,
            mip_gap,
            status_code,
            node_count,
            iteration_count,
            first_incumbent_time,
        )
    decode_start = time.time()
    route_paths = _extract_route_paths(instance, x, y, used, trucks, drones) if decode_routes else tuple()
    decode_time = time.time() - decode_start
    status = "success" if model.SolCount > 0 else _compact_status_name(model.Status)
    return CompactSolution(
        model.ObjVal,
        route_paths,
        CompactTiming(build_time, solve_time, decode_time),
        status,
        objective_bound,
        mip_gap,
        status_code,
        node_count,
        iteration_count,
        first_incumbent_time,
    )


def compact_feasibility_status(instance: InstanceData) -> tuple[bool, str]:
    result = compact_feasibility_check(instance)
    return result.feasible, "success" if result.feasible else "infeasible"


def compact_feasibility_check(instance: InstanceData) -> "FeasibilityGateResult":
    from .feasibility import exact_feasibility_check

    return exact_feasibility_check(instance)


def _compact_status_name(status: int) -> str:
    if status == GRB.OPTIMAL:
        return "success"
    if status == GRB.TIME_LIMIT:
        return "timeout"
    if status == GRB.INFEASIBLE:
        return "infeasible"
    if status == GRB.SOLUTION_LIMIT:
        return "solution_limit"
    return f"status_{status}"


def _extract_route_paths(
    instance: InstanceData,
    x: gp.tupledict,
    y: gp.tupledict,
    used: gp.tupledict,
    trucks: range,
    drones: range,
) -> tuple[tuple[str, ...], ...]:
    graph = build_transformed_graph(instance)
    route_paths = []
    for k in trucks:
        if used[k].X <= 0.5:
            continue
        outgoing = {
            i: j
            for i, j in instance.truck_arcs
            if x[i, j, k].X > 0.5
        }
        physical_path = [instance.depot_source]
        node = instance.depot_source
        while node != instance.depot_sink:
            node = outgoing[node]
            physical_path.append(node)
        drone_blocks: dict[str, list[str]] = {hub: [] for hub in instance.hubs}
        for hub, customer in instance.drone_arcs:
            if any(y[hub, customer, k, d].X > 0.5 for d in drones):
                drone_blocks[hub].append(customer)
        transformed_path = [physical_path[0]]
        for physical_prev, next_node in zip(physical_path, physical_path[1:]):
            if physical_prev in instance.hubs:
                for customer in sorted(drone_blocks[physical_prev], key=lambda c: graph.order[(physical_prev, c)]):
                    transformed_path.append(duplicate_node(customer))
            transformed_path.append(next_node)
        if any(node in instance.customers or is_duplicate(node) for node in transformed_path):
            route_paths.append(tuple(transformed_path))
    return tuple(route_paths)
