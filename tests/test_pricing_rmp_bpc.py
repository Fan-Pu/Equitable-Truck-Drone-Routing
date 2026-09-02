from __future__ import annotations

import time
from dataclasses import replace
import json
from pathlib import Path

import networkx as nx
import pytest

import thvrpd.bpc as bpc_module
import thvrpd.heuristics as heuristic_module
import thvrpd.pricing as pricing_module
from thvrpd.bpc import (
    BPCBoundInconsistency,
    BPCStats,
    _IncumbentState,
    _activate_sr_cuts,
    _bound_fathoms,
    _branch,
    _extract_root_routes,
    _relative_gap,
    _rmp_integrality_diagnostics,
    _solve_node,
    _validate_root_bound_order,
    solve_branch_price_cut,
)
from thvrpd.branching import BranchRestrictions
from thvrpd.columns import (
    RouteSignatureCache,
    build_branch_route_index,
    customer_mask,
    extend_branch_route_index,
    query_branch_route_index,
    sr_coeff_from_mask,
    triplet_mask,
)
from thvrpd.compact import CompactSolution, CompactTiming, solve_compact_solution
from thvrpd.config import ObjectiveWeights, SolverConfig
from thvrpd.heuristics import run_route_pool_heuristic
from thvrpd.instance import InstanceData, tiny_instance
from thvrpd.objective import build_objective_data
from thvrpd.pricing import (
    PricingDuals,
    PricingEpochContext,
    PricingSchedulerConfig,
    SourceNeighborPricingPool,
    _DeadlinePricingCounters,
    _DominanceExtensionContext,
    _Label,
    _balanced_source_neighbor_plan,
    _branch_interface_compatible,
    _build_pricing_bounds,
    _closure_gap_exceeds_split_threshold,
    _dual_reward_bound,
    _extension_allowed,
    _extension_rejected_by_service_deadline,
    _feasible_first_successors,
    _farkas_completion_lower_bound,
    _farkas_dominates,
    _paper_dominates,
    _price_route_forward_only,
    _pricing_epoch,
    _queue_key,
    _shortest_truck_times,
    _source_neighbor_prefix_tasks,
    _split_open_label_frontier,
    _sr_extra_penalty_bound,
    _task_closure_gap,
    _together_branch_partner_unreachable,
    price_route,
    route_farkas_reduced_cost,
    route_reduced_cost,
)
from thvrpd.rmp import NodeState, RestrictedMaster
from thvrpd.routes import Route, route_from_path, validate_route_cover
from thvrpd.transform import build_transformed_graph, duplicate_node
from thvrpd.warm_start_regeneration import screen_warm_start_candidate
from thvrpd.pc8_performance_regeneration import (
    bpc_trial_qualifies,
    compact_trial_qualifies,
    cross_solver_bounds_consistent,
)


def _setup():
    instance = tiny_instance()
    weights = ObjectiveWeights(0.4, 0.3, 0.3)
    objective = build_objective_data(instance, weights)
    graph = build_transformed_graph(instance)
    return instance, weights, objective, graph


def _two_hub_branch_index_setup():
    base = tiny_instance()
    h2 = "H2"
    config = replace(base.config, num_hubs=2)
    nodes = (base.depot_source,) + base.customers + ("H1", h2) + (base.depot_sink,)
    truck_arcs = set(base.truck_arcs)
    added_truck_arcs = {
        (base.depot_source, h2),
        ("H1", h2),
        (h2, "H1"),
        (h2, base.depot_sink),
    }
    truck_arcs.update(added_truck_arcs)
    truck_time = dict(base.truck_time)
    truck_time.update({arc: 1.0 for arc in added_truck_arcs})
    drone_arcs = set(base.drone_arcs)
    drone_arcs.add((h2, "C1"))
    drone_time = dict(base.drone_time)
    drone_time[(h2, "C1")] = 1.0
    drone_time[("C1", h2)] = 1.0
    drone_trip_time = dict(base.drone_trip_time)
    drone_trip_time[(h2, "C1")] = 2.0
    demand = dict(base.demand)
    demand[h2] = 0.0
    locations = dict(base.locations)
    locations[h2] = (2.0, 2.0)
    instance = replace(
        base,
        config=config,
        hubs=("H1", h2),
        nodes=nodes,
        truck_arcs=frozenset(truck_arcs),
        drone_arcs=frozenset(drone_arcs),
        truck_time=truck_time,
        drone_time=drone_time,
        drone_trip_time=drone_trip_time,
        demand=demand,
        locations=locations,
    )
    weights = ObjectiveWeights(0.4, 0.3, 0.3)
    objective = build_objective_data(instance, weights)
    graph = build_transformed_graph(instance)
    return instance, objective, graph


def _label(
    endpoint: str,
    *,
    active_pad: str = "H1",
    represented: frozenset[str] = frozenset(),
    truck_visited: frozenset[str] = frozenset({"Source", "H1"}),
    arrival: float = 0.0,
    wait: float = 0.0,
    reduced_cost: float = 0.0,
    sr_counts: tuple[tuple[tuple[str, str, str], int], ...] = tuple(),
) -> _Label:
    return _Label(
        path=("Source", active_pad, endpoint),
        represented=represented,
        truck_visited=truck_visited,
        truck_load=0.0,
        active_pad=active_pad,
        active_pad_arrival=arrival,
        active_wait=wait,
        block_count=0,
        physical_time=arrival,
        service_times=tuple(),
        sr_counts=sr_counts,
        reduced_cost=reduced_cost,
        used_arcs=frozenset({("Source", active_pad), (active_pad, endpoint)}),
        represented_mask=0,
        truck_node_mask=0,
    )


def _dominance_context(
    instance: InstanceData,
    objective,
    restrictions: BranchRestrictions,
    residual_customers: frozenset[str] | None = None,
) -> _DominanceExtensionContext:
    return _DominanceExtensionContext(
        objective=objective,
        residual_customers=residual_customers or frozenset(instance.customers),
        restrictions=restrictions,
    )


def _root_node(instance, paths: set[tuple[str, ...]], active_sr=None) -> NodeState:
    return NodeState(
        id=0,
        depth=0,
        restrictions=BranchRestrictions(),
        fixed_routes=tuple(),
        residual_customers=frozenset(instance.customers),
        fleet_limit=instance.num_trucks,
        fixed_cost=0.0,
        column_paths=set(paths),
        active_sr=set(active_sr or set()),
    )


def test_forward_extension_rejects_customer_after_service_envelope() -> None:
    base = tiny_instance()
    objective = build_objective_data(base, ObjectiveWeights(0.4, 0.3, 0.3))
    bounds = dict(objective.bounds.service_ub)
    bounds["C1"] = objective.bounds.arrival_lb["C1"]
    instance = replace(base, config=replace(base.config, service_deadline_mode="manual", service_deadline_manual_bounds=bounds))
    graph = build_transformed_graph(instance)
    objective = build_objective_data(instance, ObjectiveWeights(0.4, 0.3, 0.3))
    source = _Label(
        path=("Source", "C2"), represented=frozenset({"C2"}), truck_visited=frozenset({"Source", "C2"}),
        truck_load=instance.demand["C2"], active_pad=None, active_pad_arrival=0.0, active_wait=0.0,
        block_count=0, physical_time=instance.truck_time[("Source", "C2")], service_times=tuple(),
        sr_counts=tuple(), reduced_cost=0.0, used_arcs=frozenset({("Source", "C2")}),
    )
    assert _extension_rejected_by_service_deadline(source, "C1", graph, objective)


def test_branch_interface_ignores_branch_infeasible_first_continuation() -> None:
    instance, _, objective, graph = _setup()
    left = duplicate_node("C1")
    right = duplicate_node("C2")
    restrictions = BranchRestrictions().with_conditioned_arc_forbidden("C3", (right, "Sink"))
    represented = frozenset(instance.customers)
    a = _label(left, represented=represented, arrival=1.0, reduced_cost=-1.0)
    b = _label(right, represented=represented, arrival=2.0, reduced_cost=0.0)
    context = _dominance_context(instance, objective, restrictions)

    assert "Sink" not in _feasible_first_successors(b, graph, context)
    assert _branch_interface_compatible(a, b, graph, context)
    assert _paper_dominates(a, b, graph, PricingDuals(mu={}, kappa=0.0), context)
    assert _farkas_dominates(a, b, graph, context)


def test_branch_interface_rejects_distinct_endpoint_required_arc_mismatch() -> None:
    instance, _, objective, graph = _setup()
    left = duplicate_node("C1")
    right = duplicate_node("C2")
    restrictions = BranchRestrictions().with_conditioned_arc_required("C3", (right, "Sink"))
    represented = frozenset(instance.customers)
    a = _label(left, represented=represented, arrival=1.0, reduced_cost=-1.0)
    b = _label(right, represented=represented, arrival=2.0, reduced_cost=0.0)
    context = _dominance_context(instance, objective, restrictions)

    assert "Sink" in _feasible_first_successors(b, graph, context)
    assert "Sink" not in _feasible_first_successors(a, graph, context)
    assert not _branch_interface_compatible(a, b, graph, context)
    assert not _paper_dominates(a, b, graph, PricingDuals(mu={}, kappa=0.0), context)
    assert not _farkas_dominates(a, b, graph, context)


def test_same_endpoint_branch_interface_is_automatic() -> None:
    instance, _, objective, graph = _setup()
    endpoint = duplicate_node("C1")
    restrictions = BranchRestrictions().with_conditioned_arc_forbidden("C3", (endpoint, "Sink"))
    label = _label(endpoint)
    context = _dominance_context(instance, objective, restrictions)
    assert _branch_interface_compatible(label, label, graph, context)


def test_feasible_first_successors_apply_all_production_extension_filters() -> None:
    instance, _, objective, graph = _setup()
    endpoint = duplicate_node("C1")
    successor = duplicate_node("C2")
    base_label = _label(endpoint, represented=frozenset({"C1"}))

    elementary_label = replace(base_label, represented=frozenset({"C1", "C2"}))
    elementary_context = _dominance_context(instance, objective, BranchRestrictions())
    assert successor not in _feasible_first_successors(elementary_label, graph, elementary_context)

    residual_context = _dominance_context(
        instance,
        objective,
        BranchRestrictions(),
        frozenset({"C1", "C3"}),
    )
    assert successor not in _feasible_first_successors(base_label, graph, residual_context)

    payload_label = replace(base_label, truck_load=instance.truck_payload - 1.0)
    payload_context = _dominance_context(instance, objective, BranchRestrictions())
    assert successor not in _feasible_first_successors(payload_label, graph, payload_context)

    drone_count_label = replace(base_label, block_count=instance.drones_per_truck)
    drone_count_context = _dominance_context(instance, objective, BranchRestrictions())
    assert successor not in _feasible_first_successors(drone_count_label, graph, drone_count_context)

    pad_context = _dominance_context(
        instance,
        objective,
        BranchRestrictions().with_pad_forbidden("H1", "C2"),
    )
    assert successor not in _feasible_first_successors(base_label, graph, pad_context)

    pair_context = _dominance_context(
        instance,
        objective,
        BranchRestrictions().with_separate("C1", "C2"),
    )
    assert successor not in _feasible_first_successors(base_label, graph, pair_context)

    conditioned_context = _dominance_context(
        instance,
        objective,
        BranchRestrictions().with_conditioned_arc_forbidden("C1", (endpoint, successor)),
    )
    assert successor not in _feasible_first_successors(base_label, graph, conditioned_context)

    bounds = dict(objective.bounds.service_ub)
    bounds["C2"] = objective.bounds.arrival_lb["C2"]
    deadline_instance = replace(
        instance,
        config=replace(
            instance.config,
            service_deadline_mode="manual",
            service_deadline_manual_bounds=bounds,
        ),
    )
    deadline_graph = build_transformed_graph(deadline_instance)
    deadline_objective = build_objective_data(deadline_instance, ObjectiveWeights(0.4, 0.3, 0.3))
    deadline_context = _dominance_context(deadline_instance, deadline_objective, BranchRestrictions())
    late_label = replace(base_label, active_pad_arrival=5.0, physical_time=5.0)
    assert successor not in _feasible_first_successors(late_label, deadline_graph, deadline_context)


def test_feasible_first_successor_cache_is_local_to_pricing_context() -> None:
    instance, _, objective, graph = _setup()
    label = _label(duplicate_node("C1"), represented=frozenset({"C1"}))
    successor = duplicate_node("C2")
    full_context = _dominance_context(instance, objective, BranchRestrictions())
    reduced_context = _dominance_context(
        instance,
        objective,
        BranchRestrictions(),
        frozenset({"C1", "C3"}),
    )

    assert successor in _feasible_first_successors(label, graph, full_context)
    assert successor not in _feasible_first_successors(label, graph, reduced_context)
    assert full_context.feasible_first_successors is not reduced_context.feasible_first_successors


def test_together_branch_partner_reachability_predicate() -> None:
    instance, _, objective, graph = _setup()
    restrictions = BranchRestrictions().with_together("C1", "C2")
    shortest = _shortest_truck_times(graph)
    represented_one = replace(
        _label(
            "C1",
            represented=frozenset({"C1"}),
            truck_visited=frozenset({"Source", "H1", "C1"}),
        ),
        truck_load=instance.demand["C1"],
    )

    assert not _together_branch_partner_unreachable(
        represented_one,
        graph,
        objective,
        shortest,
        frozenset(instance.customers),
        restrictions,
    )
    assert _together_branch_partner_unreachable(
        represented_one,
        graph,
        objective,
        shortest,
        frozenset({"C1", "C3"}),
        restrictions,
    )

    payload_exhausted = replace(represented_one, truck_load=instance.truck_payload - 1.0)
    assert _together_branch_partner_unreachable(
        payload_exhausted,
        graph,
        objective,
        shortest,
        frozenset(instance.customers),
        restrictions,
    )

    unreachable_shortest = dict(shortest)
    unreachable_shortest[("C1", "C2")] = float("inf")
    unreachable_shortest[("C2", "Sink")] = float("inf")
    for hub in instance.hubs:
        unreachable_shortest[("C1", hub)] = float("inf")
    assert _together_branch_partner_unreachable(
        represented_one,
        graph,
        objective,
        unreachable_shortest,
        frozenset(instance.customers),
        restrictions,
    )

    neither = replace(represented_one, represented=frozenset())
    both = replace(represented_one, represented=frozenset({"C1", "C2"}))
    assert not _together_branch_partner_unreachable(
        neither,
        graph,
        objective,
        unreachable_shortest,
        frozenset(instance.customers),
        restrictions,
    )
    assert not _together_branch_partner_unreachable(
        both,
        graph,
        objective,
        unreachable_shortest,
        frozenset(instance.customers),
        restrictions,
    )


def test_together_branch_partner_deadline_pruning_applies_to_prefix_and_resumed_labels() -> None:
    base = tiny_instance()
    baseline = build_objective_data(base, ObjectiveWeights(0.4, 0.3, 0.3))
    bounds = dict(baseline.bounds.service_ub)
    bounds["C2"] = baseline.bounds.arrival_lb["C2"]
    instance = replace(
        base,
        config=replace(
            base.config,
            service_deadline_mode="manual",
            service_deadline_manual_bounds=bounds,
        ),
    )
    objective = build_objective_data(instance, ObjectiveWeights(0.4, 0.3, 0.3))
    graph = build_transformed_graph(instance)
    restrictions = BranchRestrictions().with_together("C2", "C3")
    duals = PricingDuals(mu={customer: 10.0 for customer in instance.customers}, kappa=0.0)

    prefix_standard = _price_route_forward_only(
        graph,
        objective,
        frozenset(instance.customers),
        restrictions,
        duals,
        next_route_id=0,
        use_standard_acceleration=False,
        batch_size=64,
        pricing_mode="closure",
        source_prefixes=(("C3",),),
    )
    prefix_farkas = _price_route_forward_only(
        graph,
        objective,
        frozenset(instance.customers),
        restrictions,
        duals,
        next_route_id=0,
        farkas=True,
        batch_size=64,
        pricing_mode="closure",
        source_prefixes=(("C3",),),
    )
    assert prefix_standard.diagnostics.together_branch_reachability_pruned == 1
    assert prefix_farkas.diagnostics.together_branch_reachability_pruned == 1
    assert prefix_standard.diagnostics.standard_bound_pruned == 0
    assert prefix_farkas.diagnostics.farkas_bound_pruned == 0

    resumed_label = _Label(
        path=("Source", "C3"),
        represented=frozenset({"C3"}),
        truck_visited=frozenset({"Source", "C3"}),
        truck_load=instance.demand["C3"],
        active_pad=None,
        active_pad_arrival=0.0,
        active_wait=0.0,
        block_count=0,
        physical_time=instance.truck_time[("Source", "C3")],
        service_times=(("C3", instance.truck_time[("Source", "C3")]),),
        sr_counts=tuple(),
        reduced_cost=0.0,
        used_arcs=frozenset({("Source", "C3")}),
    )
    resumed = _price_route_forward_only(
        graph,
        objective,
        frozenset(instance.customers),
        restrictions,
        duals,
        next_route_id=0,
        use_standard_acceleration=False,
        batch_size=64,
        pricing_mode="closure",
        initial_open_labels=(resumed_label,),
    )
    assert resumed.diagnostics.together_branch_reachability_pruned == 1
    assert resumed.diagnostics.labels_pruned == 1


@pytest.mark.parametrize("farkas", [False, True])
def test_together_reachability_pruning_preserves_complete_pricing_routes(monkeypatch, farkas: bool) -> None:
    base = tiny_instance()
    baseline = build_objective_data(base, ObjectiveWeights(0.4, 0.3, 0.3))
    bounds = dict(baseline.bounds.service_ub)
    bounds["C2"] = baseline.bounds.arrival_lb["C2"]
    instance = replace(
        base,
        config=replace(
            base.config,
            service_deadline_mode="manual",
            service_deadline_manual_bounds=bounds,
        ),
    )
    objective = build_objective_data(instance, ObjectiveWeights(0.4, 0.3, 0.3))
    graph = build_transformed_graph(instance)
    restrictions = BranchRestrictions().with_together("C2", "C3")
    duals = PricingDuals(mu={customer: 10.0 for customer in instance.customers}, kappa=0.0)
    kwargs = dict(
        graph=graph,
        objective=objective,
        residual_customers=frozenset(instance.customers),
        restrictions=restrictions,
        duals=duals,
        next_route_id=0,
        farkas=farkas,
        pricing_tolerance=1e-7,
        use_standard_acceleration=not farkas,
        batch_size=64,
        parallel_workers=1,
        pricing_mode="closure",
    )
    enabled = price_route(**kwargs)
    monkeypatch.setattr(pricing_module, "_together_branch_partner_unreachable", lambda *args, **kwargs: False)
    disabled = price_route(**kwargs)

    assert {route.path for route in enabled.routes} == {route.path for route in disabled.routes}
    assert enabled.diagnostics.exact_completion == disabled.diagnostics.exact_completion
    assert enabled.diagnostics.together_branch_reachability_pruned > 0
    assert disabled.diagnostics.together_branch_reachability_pruned == 0
    assert enabled.diagnostics.labels_generated <= disabled.diagnostics.labels_generated


def test_branch_hierarchy_uses_pair_then_pad_then_conditioned_arc() -> None:
    instance, _, objective, graph = _setup()
    config = SolverConfig(enable_root_compact_warm_start=False)

    pair_paths = {
        ("Source", "C1", "C2", "Sink"),
        ("Source", "C1", "C3", "Sink"),
    }
    pair_routes = {
        path: route_from_path(index, path, graph, objective)
        for index, path in enumerate(sorted(pair_paths))
    }
    pair_node = _root_node(instance, pair_paths)
    pair_decision = _branch(pair_node, {path: 0.5 for path in pair_paths}, pair_routes, graph, 1, 2, config)
    assert pair_decision.branch_type == "customer_pair"

    truck_path = ("Source", "C1", "Sink")
    drone_path = ("Source", "H1", duplicate_node("C1"), "Sink")
    pad_paths = {truck_path, drone_path}
    pad_routes = {
        path: route_from_path(index, path, graph, objective)
        for index, path in enumerate(sorted(pad_paths))
    }
    pad_node = _root_node(instance, pad_paths)
    pad_decision = _branch(pad_node, {path: 0.5 for path in pad_paths}, pad_routes, graph, 3, 4, config)
    assert pad_decision.branch_type == "launch_pad"

    forward_path = ("Source", "C1", "C2", "Sink")
    reverse_path = ("Source", "C2", "C1", "Sink")
    arc_paths = {forward_path, reverse_path}
    arc_routes = {
        path: route_from_path(index, path, graph, objective)
        for index, path in enumerate(sorted(arc_paths))
    }
    arc_node = _root_node(instance, arc_paths)
    arc_decision = _branch(arc_node, {path: 0.5 for path in arc_paths}, arc_routes, graph, 5, 6, config)
    assert arc_decision.branch_type == "conditioned_arc"
    assert arc_decision.left is not None and arc_decision.left.restrictions.conditioned_arc_forbidden
    assert arc_decision.right is not None and arc_decision.right.restrictions.conditioned_arc_required


def test_conditioned_arc_down_is_enforced_in_both_event_orders() -> None:
    instance, _, objective, graph = _setup()
    arc_before_customer = ("Source", "C1")
    before = _Label(
        path=("Source", "C1"),
        represented=frozenset({"C1"}),
        truck_visited=frozenset({"Source", "C1"}),
        truck_load=instance.demand["C1"],
        active_pad=None,
        active_pad_arrival=0.0,
        active_wait=0.0,
        block_count=0,
        physical_time=instance.truck_time[arc_before_customer],
        service_times=(("C1", instance.truck_time[arc_before_customer]),),
        sr_counts=tuple(),
        reduced_cost=0.0,
        used_arcs=frozenset({arc_before_customer}),
    )
    before_restriction = BranchRestrictions().with_conditioned_arc_forbidden("C2", arc_before_customer)
    assert not _extension_allowed(before, "C2", graph, frozenset(instance.customers), before_restriction)

    arc_after_customer = ("C2", "C1")
    after = _Label(
        path=("Source", "C2"),
        represented=frozenset({"C2"}),
        truck_visited=frozenset({"Source", "C2"}),
        truck_load=instance.demand["C2"],
        active_pad=None,
        active_pad_arrival=0.0,
        active_wait=0.0,
        block_count=0,
        physical_time=instance.truck_time[("Source", "C2")],
        service_times=(("C2", instance.truck_time[("Source", "C2")]),),
        sr_counts=tuple(),
        reduced_cost=0.0,
        used_arcs=frozenset({("Source", "C2")}),
    )
    after_restriction = BranchRestrictions().with_conditioned_arc_forbidden("C2", arc_after_customer)
    assert not _extension_allowed(after, "C1", graph, frozenset(instance.customers), after_restriction)

    source = _Label(
        path=("Source",),
        represented=frozenset(),
        truck_visited=frozenset({"Source"}),
        truck_load=0.0,
        active_pad=None,
        active_pad_arrival=0.0,
        active_wait=0.0,
        block_count=0,
        physical_time=0.0,
        service_times=tuple(),
        sr_counts=tuple(),
        reduced_cost=0.0,
        used_arcs=frozenset(),
    )
    required_before = BranchRestrictions().with_conditioned_arc_required("C2", arc_before_customer)
    assert not _extension_allowed(source, "C2", graph, frozenset(instance.customers), required_before)
    required_after = BranchRestrictions().with_conditioned_arc_required("C2", arc_after_customer)
    assert _extension_allowed(source, "C2", graph, frozenset(instance.customers), required_after)
    assert not _extension_allowed(after, "C3", graph, frozenset(instance.customers), required_after)
    assert _extension_allowed(after, "C1", graph, frozenset(instance.customers), required_after)

    before_route = route_from_path(0, ("Source", "C1", "C2", "Sink"), graph, objective)
    after_route = route_from_path(1, ("Source", "C2", "C1", "Sink"), graph, objective)
    assert not before_restriction.route_allowed(before_route)
    assert not after_restriction.route_allowed(after_route)
    assert required_before.route_allowed(before_route)
    assert required_after.route_allowed(after_route)


def test_branch_index_matches_exact_route_filtering_for_all_three_families() -> None:
    instance, _, objective, graph = _setup()
    paths = {
        ("Source", "C1", "Sink"),
        ("Source", "H1", duplicate_node("C1"), "Sink"),
        ("Source", "C1", "C2", "Sink"),
        ("Source", "C2", "C1", "Sink"),
    }
    routes = {
        path: route_from_path(index, path, graph, objective)
        for index, path in enumerate(sorted(paths))
    }
    index = build_branch_route_index(paths, routes, graph, frozenset(instance.customers))
    restrictions_to_check = (
        BranchRestrictions().with_together("C1", "C2"),
        BranchRestrictions().with_separate("C1", "C2"),
        BranchRestrictions().with_pad_forbidden("H1", "C1"),
        BranchRestrictions().with_pad_required("H1", "C1"),
        BranchRestrictions().with_conditioned_arc_forbidden("C1", ("Source", "C1")),
        BranchRestrictions().with_conditioned_arc_required("C1", ("Source", "C1")),
    )
    for restrictions in restrictions_to_check:
        indexed, _ = query_branch_route_index(index, restrictions)
        brute_force = {path for path, route in routes.items() if restrictions.route_allowed(route)}
        assert indexed == brute_force

    for customer in instance.customers:
        for arc in graph.arcs:
            for restrictions in (
                BranchRestrictions().with_conditioned_arc_forbidden(customer, arc),
                BranchRestrictions().with_conditioned_arc_required(customer, arc),
            ):
                indexed, _ = query_branch_route_index(index, restrictions)
                brute_force = {path for path, route in routes.items() if restrictions.route_allowed(route)}
                assert indexed == brute_force


def test_branch_index_retains_required_noncustomer_arc_routes() -> None:
    instance, objective, graph = _two_hub_branch_index_setup()
    r1 = ("Source", "H1", duplicate_node("C1"), "Sink")
    r2 = ("Source", "H2", duplicate_node("C1"), "Sink")
    r3 = ("Source", "H1", duplicate_node("C2"), "Sink")
    paths = {r1, r2, r3}
    routes = {
        path: route_from_path(route_id, path, graph, objective)
        for route_id, path in enumerate(sorted(paths))
    }
    index = build_branch_route_index(paths, routes, graph, frozenset(instance.customers))

    required = BranchRestrictions().with_conditioned_arc_required("C1", ("Source", "H1"))
    forbidden = BranchRestrictions().with_conditioned_arc_forbidden("C1", ("Source", "H1"))

    assert required.route_allowed(routes[r1])
    assert query_branch_route_index(index, required)[0] == {r1, r3}
    assert query_branch_route_index(index, forbidden)[0] == {r2, r3}


def test_incremental_branch_index_matches_full_rebuild_for_every_transformed_arc() -> None:
    instance, objective, graph = _two_hub_branch_index_setup()
    initial_paths = {
        ("Source", "H1", duplicate_node("C1"), "Sink"),
        ("Source", "H2", duplicate_node("C1"), "Sink"),
    }
    added_paths = {
        ("Source", "H1", duplicate_node("C2"), "Sink"),
        ("Source", "H1", "H2", duplicate_node("C1"), "Sink"),
        ("Source", "H1", duplicate_node("C1"), duplicate_node("C2"), "Sink"),
        ("Source", "C2", "H1", "Sink"),
    }
    all_paths = initial_paths | added_paths
    routes = {
        path: route_from_path(route_id, path, graph, objective)
        for route_id, path in enumerate(sorted(all_paths))
    }
    residual = frozenset(instance.customers)
    initial = build_branch_route_index(initial_paths, routes, graph, residual)
    incremental = extend_branch_route_index(initial, added_paths, routes, graph, residual)
    rebuilt = build_branch_route_index(all_paths, routes, graph, residual)

    assert incremental == rebuilt
    for customer in instance.customers:
        for arc in graph.arcs:
            for restrictions in (
                BranchRestrictions().with_conditioned_arc_forbidden(customer, arc),
                BranchRestrictions().with_conditioned_arc_required(customer, arc),
            ):
                indexed, _ = query_branch_route_index(incremental, restrictions)
                brute_force = {path for path, route in routes.items() if restrictions.route_allowed(route)}
                assert indexed == brute_force


def test_shared_duplicate_labels_from_different_active_pads_are_not_comparable() -> None:
    instance, _, objective, graph = _setup()
    endpoint = duplicate_node("C1")
    h1 = _label(endpoint, active_pad="H1", reduced_cost=-1.0)
    h2 = _label(endpoint, active_pad="H2", reduced_cost=0.0)
    restrictions = BranchRestrictions()
    context = _dominance_context(instance, objective, restrictions)
    assert not _paper_dominates(
        h1,
        h2,
        graph,
        PricingDuals(mu={}, kappa=0.0),
        context,
    )
    assert not _farkas_dominates(h1, h2, graph, context)


def test_sr_adjustment_is_exactly_active_negative_one_to_two() -> None:
    triplet = ("C1", "C2", "C3")
    duals = PricingDuals(mu={}, kappa=0.0, nu={triplet: -2.5})
    endpoint = "C1"
    one = _label(endpoint, sr_counts=((triplet, 1),))
    two = _label(endpoint, sr_counts=((triplet, 2),))
    zero = _label(endpoint, sr_counts=((triplet, 0),))
    three = _label(endpoint, sr_counts=((triplet, 3),))
    assert _sr_extra_penalty_bound(one, two, duals) == pytest.approx(-2.5)
    assert _sr_extra_penalty_bound(zero, two, duals) == 0.0
    assert _sr_extra_penalty_bound(one, three, duals) == 0.0
    assert _sr_extra_penalty_bound(one, two, replace(duals, nu={triplet: 1.0})) == 0.0


def test_deterministic_prefix_tasks_are_prefix_free_disjoint_and_exhaustive() -> None:
    instance, _, objective, graph = _setup()
    restrictions = BranchRestrictions()
    depth_one = _source_neighbor_prefix_tasks(graph, objective, frozenset(instance.customers), restrictions, 1)
    assert depth_one == tuple((node,) for node in sorted(node for node in graph.out_arcs["Source"] if node != "Sink"))
    depth_two = _source_neighbor_prefix_tasks(graph, objective, frozenset(instance.customers), restrictions, 2)
    assert len(depth_two) == len(set(depth_two))
    assert all(not (len(a) < len(b) and b[: len(a)] == a) for a in depth_two for b in depth_two)
    assert {prefix[0] for prefix in depth_two} == {prefix[0] for prefix in depth_one}


def test_balanced_source_assignment_is_deterministic_disjoint_and_exhaustive() -> None:
    instance, _, objective, graph = _setup()
    residual = frozenset(instance.customers)
    restrictions = BranchRestrictions()
    duals = PricingDuals(mu={customer: 0.0 for customer in instance.customers}, kappa=0.0)
    scheduler = PricingSchedulerConfig()
    first = _balanced_source_neighbor_plan(graph, objective, residual, restrictions, duals, 3, scheduler)
    second = _balanced_source_neighbor_plan(graph, objective, residual, restrictions, duals, 3, scheduler)
    assert first == second
    assigned = [neighbor for block in first.blocks for neighbor in block]
    assert len(assigned) == len(set(assigned))
    assert set(assigned) == set(first.source_neighbors)
    assert len(first.blocks) == len(first.block_scores) == 3


def test_dynamic_split_conserves_complete_label_states_and_retains_shallow_labels() -> None:
    shallow = replace(_label("H1"), path=("Source", "H1"))
    first = _label("C1", represented=frozenset({"C1"}))
    second = _label("C2", represented=frozenset({"C2"}))
    third = replace(first, path=("Source", "H1", "C3"), represented=frozenset({"C3"}))
    labels = (shallow, first, second, third)
    retained, children = _split_open_label_frontier(
        labels,
        task_root_prefix=tuple(),
        refinement_depth=2,
        child_limit=2,
    )
    post_split = list(retained)
    for _, child_labels in children:
        post_split.extend(child_labels)
    assert shallow in retained
    assert len(post_split) == len(labels)
    assert set(post_split) == set(labels)
    assert sum(len(child_labels) for _, child_labels in children) > 0


def test_audit_progress_keeps_append_only_tree_history(tmp_path: Path) -> None:
    instance, _, _, _ = _setup()
    node = _root_node(instance, set())
    config = SolverConfig(gurobi_log_dir=str(tmp_path / "gurobi_logs"), logging_mode="audit")
    stats = BPCStats()
    bpc_module._initialize_progress_history(config)
    bpc_module._write_progress(config, stats, node, "node_started", {"queue_bound": 0.0})
    bpc_module._write_progress(config, stats, node, "branch_created", {"left_child": 1, "right_child": 2})
    history = [json.loads(line) for line in (tmp_path / "bpc_progress.jsonl").read_text(encoding="utf-8").splitlines()]
    assert [record["event"] for record in history] == ["node_started", "branch_created"]
    assert [record["event_sequence"] for record in history] == [1, 2]
    latest = json.loads((tmp_path / "bpc_progress.json").read_text(encoding="utf-8"))
    assert latest["event"] == "branch_created"


def test_parallel_forward_certification_matches_serial() -> None:
    instance, _, objective, graph = _setup()
    duals = PricingDuals(mu={customer: 0.0 for customer in instance.customers}, kappa=0.0)
    common = dict(
        graph=graph, objective=objective, residual_customers=frozenset(instance.customers),
        restrictions=BranchRestrictions(), duals=duals, next_route_id=0,
        pricing_tolerance=0.05, batch_size=128, pricing_mode="closure",
    )
    serial = price_route(**common, parallel_workers=1)
    parallel = price_route(**common, parallel_workers=3, pricing_worker_backend="thread", prefix_task_depth=2)
    assert serial.routes == parallel.routes == tuple()
    assert serial.diagnostics.exact_completion
    assert parallel.diagnostics.exact_completion
    assert parallel.diagnostics.certification_worker_calls == parallel.diagnostics.source_neighbor_task_count
    assert not parallel.diagnostics.balanced_process_dynamic


def test_shared_duplicate_pricing_matches_brute_force_routes() -> None:
    instance, _, objective, graph = _setup()
    residual = frozenset(instance.customers)
    restrictions = BranchRestrictions()
    duals = PricingDuals(mu={customer: 10.0 for customer in residual}, kappa=0.0)
    network = nx.DiGraph()
    network.add_nodes_from(graph.nodes)
    network.add_edges_from(graph.arcs)
    brute_costs = []
    for index, path in enumerate(
        nx.all_simple_paths(network, instance.depot_source, instance.depot_sink, cutoff=len(graph.nodes) - 1)
    ):
        try:
            route = route_from_path(index, tuple(path), graph, objective)
        except ValueError:
            continue
        if restrictions.route_allowed(route):
            brute_costs.append(route_reduced_cost(route, duals))
    result = price_route(
        graph,
        objective,
        residual,
        restrictions,
        duals,
        0,
        pricing_tolerance=0.0,
        use_standard_acceleration=True,
        batch_size=1024,
        parallel_workers=1,
        pricing_mode="closure",
    )
    assert result.best_reduced_cost == pytest.approx(min(brute_costs))


def test_productive_pricing_never_certifies_closure() -> None:
    instance, _, objective, graph = _setup()
    duals = PricingDuals(mu={customer: 0.0 for customer in instance.customers}, kappa=0.0)
    result = price_route(
        graph, objective, frozenset(instance.customers), BranchRestrictions(), duals, 0,
        pricing_tolerance=0.05, batch_size=16, parallel_workers=3,
        pricing_worker_backend="thread", pricing_mode="productive",
    )
    assert not result.diagnostics.exact_completion
    assert result.diagnostics.certification_calls == 0


def test_process_backend_uses_same_deterministic_certification_partition() -> None:
    instance, _, objective, graph = _setup()
    duals = PricingDuals(mu={customer: 0.0 for customer in instance.customers}, kappa=0.0)
    result = price_route(
        graph,
        objective,
        frozenset(instance.customers),
        BranchRestrictions(),
        duals,
        0,
        pricing_tolerance=0.05,
        batch_size=32,
        parallel_workers=2,
        pricing_worker_backend="process",
        pricing_mode="closure",
    )
    assert result.diagnostics.exact_completion
    assert result.diagnostics.pricing_worker_backend == "process"
    assert result.diagnostics.balanced_process_dynamic
    assert result.diagnostics.certification_worker_calls == result.diagnostics.source_neighbor_task_count
    assert result.diagnostics.dynamic_splits_performed == 0


def test_dynamic_split_gap_uses_paper_bound_and_single_pricing_tolerance() -> None:
    tolerance = 0.01
    assert _task_closure_gap(-0.005) == pytest.approx(0.005)
    assert _task_closure_gap(-0.010) == pytest.approx(0.010)
    assert _task_closure_gap(-0.015) == pytest.approx(0.015)
    assert _task_closure_gap(0.020) == 0.0

    assert not _closure_gap_exceeds_split_threshold(0.005, tolerance, 1.0)
    assert not _closure_gap_exceeds_split_threshold(0.010, tolerance, 1.0)
    assert _closure_gap_exceeds_split_threshold(0.015, tolerance, 1.0)


def test_process_backend_can_split_and_close_dynamic_leaf_tasks() -> None:
    instance, _, objective, graph = _setup()
    triplet = tuple(instance.customers)
    duals = PricingDuals(
        mu={customer: 1.0 for customer in instance.customers},
        kappa=-1.0,
        nu={triplet: -10.0},
    )
    scheduler = PricingSchedulerConfig(
        split_open_labels_min=1,
        split_gap_factor=0.0,
        split_elapsed_min=0.0,
        split_work_min=1,
        refinement_depth=1,
        checkpoint_extension_period=1,
    )
    result = price_route(
        graph,
        objective,
        frozenset(instance.customers),
        BranchRestrictions(),
        duals,
        0,
        pricing_tolerance=0.05,
        use_standard_acceleration=True,
        batch_size=32,
        parallel_workers=8,
        pricing_worker_backend="process",
        pricing_mode="closure",
        scheduler_config=scheduler,
    )
    assert result.diagnostics.exact_completion
    assert result.diagnostics.leaf_tasks_created >= 8
    assert result.diagnostics.leaf_tasks_closed == result.diagnostics.leaf_tasks_created
    assert result.diagnostics.dynamic_splits_performed > 0
    assert result.diagnostics.dynamic_labels_transferred > 0
    assert result.diagnostics.pending_transfer_peak > 0


def test_candidate_checkpoint_split_race_does_not_fail_worker_processes() -> None:
    instance, _, objective, graph = _setup()
    scheduler = PricingSchedulerConfig(
        split_open_labels_min=1,
        split_gap_factor=0.0,
        split_elapsed_min=0.0,
        split_work_min=1,
        refinement_depth=1,
        checkpoint_extension_period=1,
    )
    duals = PricingDuals(mu={customer: 10.0 for customer in instance.customers}, kappa=0.0)
    pool = SourceNeighborPricingPool(graph, objective, 8, scheduler)
    try:
        for call in range(3):
            result = pool.price(
                residual_customers=frozenset(instance.customers),
                restrictions=BranchRestrictions(),
                duals=duals,
                next_route_id=call * 100,
                farkas=False,
                pricing_tolerance=0.01,
                use_standard_acceleration=True,
                stop_at_first_negative=False,
                batch_size=8,
                deadline=None,
                existing_routes=None,
                existing_column_paths=None,
                pricing_mode="closure",
            )
            assert result.routes
            assert result.diagnostics.candidate_checkpoints > 0
    finally:
        pool.shutdown()


def test_farkas_checkpoint_bound_drives_exact_dynamic_splitting() -> None:
    instance, _, objective, graph = _setup()
    triplet = tuple(instance.customers)
    duals = PricingDuals(
        mu={customer: 1.0 for customer in instance.customers},
        kappa=-1.0,
        nu={triplet: -10.0},
    )
    scheduler = PricingSchedulerConfig(
        split_open_labels_min=1,
        split_gap_factor=0.0,
        split_elapsed_min=0.0,
        split_work_min=1,
        refinement_depth=1,
        checkpoint_extension_period=1,
    )
    result = price_route(
        graph,
        objective,
        frozenset(instance.customers),
        BranchRestrictions(),
        duals,
        0,
        farkas=True,
        pricing_tolerance=0.01,
        batch_size=32,
        parallel_workers=8,
        pricing_worker_backend="process",
        pricing_mode="closure",
        scheduler_config=scheduler,
    )
    assert not result.routes
    assert result.diagnostics.exact_completion
    assert result.diagnostics.dynamic_splits_performed > 0
    assert result.diagnostics.farkas_bound_pruned > 0


def test_process_pool_is_reused_across_pricing_epochs() -> None:
    instance, _, objective, graph = _setup()
    residual = frozenset(instance.customers)
    duals = PricingDuals(mu={customer: 0.0 for customer in instance.customers}, kappa=0.0)
    pool = SourceNeighborPricingPool(graph, objective, 2, PricingSchedulerConfig())
    try:
        for _ in range(2):
            result = pool.price(
                residual_customers=residual,
                restrictions=BranchRestrictions(),
                duals=duals,
                next_route_id=0,
                farkas=False,
                pricing_tolerance=0.05,
                use_standard_acceleration=True,
                stop_at_first_negative=False,
                batch_size=32,
                deadline=None,
                existing_routes=None,
                existing_column_paths=None,
                pricing_mode="closure",
            )
            assert result.diagnostics.exact_completion
        assert pool.startup_count == 1
        assert pool.reused_calls == 2
    finally:
        pool.shutdown()


@pytest.mark.parametrize("farkas", (False, True))
def test_process_workers_collect_a_global_verified_batch(farkas: bool) -> None:
    instance, _, objective, graph = _setup()
    residual = frozenset(instance.customers)
    duals = PricingDuals(mu={customer: 100.0 for customer in residual}, kappa=0.0)
    result = price_route(
        graph,
        objective,
        residual,
        BranchRestrictions(),
        duals,
        0,
        farkas=farkas,
        pricing_tolerance=0.01,
        use_standard_acceleration=True,
        batch_size=2,
        parallel_workers=2,
        pricing_worker_backend="process",
        pricing_mode="closure",
    )
    assert len(result.routes) == 2
    assert result.diagnostics.pricing_candidate_paths_before_merge >= 2
    assert result.diagnostics.pricing_returned_batch_size == 2
    assert result.diagnostics.pricing_epoch_invalidations == 1
    assert result.diagnostics.candidate_checkpoints > 0
    assert result.diagnostics.candidate_worker_resumptions > 0
    assert result.diagnostics.global_verified_candidates == 2
    assert result.diagnostics.global_batch_limit_cancellations == 1
    direct_costs = tuple(
        route_farkas_reduced_cost(route, duals) if farkas else route_reduced_cost(route, duals)
        for route in result.routes
    )
    assert result.reduced_costs == pytest.approx(direct_costs)
    assert all(BranchRestrictions().route_allowed(route) for route in result.routes)
    assert not result.diagnostics.exact_completion


def test_process_collection_returns_smaller_batch_after_all_tasks_exhaust() -> None:
    instance, _, objective, graph = _setup()
    residual = frozenset({"C1"})
    result = price_route(
        graph,
        objective,
        residual,
        BranchRestrictions(),
        PricingDuals(mu={"C1": 100.0}, kappa=0.0),
        0,
        pricing_tolerance=0.01,
        batch_size=64,
        parallel_workers=2,
        pricing_worker_backend="process",
        pricing_mode="closure",
    )
    assert 0 < len(result.routes) < 64
    assert result.diagnostics.leaf_tasks_closed == result.diagnostics.leaf_tasks_created
    assert result.diagnostics.global_batch_limit_cancellations == 0
    assert not result.diagnostics.exact_completion


def test_pricing_epoch_changes_with_every_explicit_node_version() -> None:
    instance, _, objective, _ = _setup()
    residual = frozenset(instance.customers)
    restrictions = BranchRestrictions()
    duals = PricingDuals(mu={customer: 0.0 for customer in instance.customers}, kappa=0.0)
    base_context = PricingEpochContext(
        active_sr_version=1,
        fixed_route_signature=(("Source", "C1", "Sink"),),
        active_column_version=(("Source", "C1", "Sink"),),
        rmp_structure_version=("base",),
    )
    base = _pricing_epoch(objective, residual, restrictions, duals, None, None, base_context)
    variants = (
        replace(base_context, active_sr_version=2),
        replace(base_context, fixed_route_signature=(("Source", "C2", "Sink"),)),
        replace(base_context, active_column_version=(("Source", "C2", "Sink"),)),
        replace(base_context, rmp_structure_version=("changed",)),
    )
    assert all(_pricing_epoch(objective, residual, restrictions, duals, None, None, context) != base for context in variants)
    assert _pricing_epoch(
        objective,
        residual,
        restrictions,
        replace(duals, kappa=-1.0),
        None,
        None,
        base_context,
    ) != base
    assert _pricing_epoch(
        objective,
        frozenset(set(residual) - {next(iter(residual))}),
        restrictions,
        duals,
        None,
        None,
        base_context,
    ) != base
    assert _pricing_epoch(
        objective,
        residual,
        restrictions.with_conditioned_arc_forbidden("C1", ("Source", "C1")),
        duals,
        None,
        None,
        base_context,
    ) != base
    changed_bounds = replace(objective.bounds, service_ub={**objective.bounds.service_ub, "C1": objective.bounds.service_ub["C1"] + 1.0})
    assert _pricing_epoch(
        replace(objective, bounds=changed_bounds),
        residual,
        restrictions,
        duals,
        None,
        None,
        base_context,
    ) != base
    assert _pricing_epoch(
        replace(objective, coeffs=replace(objective.coeffs, cost=objective.coeffs.cost + 1.0)),
        residual,
        restrictions,
        duals,
        None,
        None,
        base_context,
    ) != base


def test_farkas_fractional_knapsack_bound_and_threshold_pruning() -> None:
    instance, _, objective, graph = _setup()
    residual = frozenset(instance.customers)
    duals = PricingDuals(mu={customer: 2.0 for customer in residual}, kappa=0.0)
    shortest = _shortest_truck_times(graph)
    bounds = _build_pricing_bounds(graph, duals, residual, shortest)
    label = _label(
        "C1",
        active_pad=None,
        represented=frozenset({"C1"}),
        truck_visited=frozenset({"Source", "C1"}),
        reduced_cost=-2.0,
    )
    reward = _dual_reward_bound(label, graph, bounds, objective, shortest)
    assert _farkas_completion_lower_bound(label, graph, objective, shortest, bounds) == pytest.approx(
        label.reduced_cost - reward
    )
    later_label = replace(label, represented=frozenset({"C1", "C2"}))
    assert _queue_key(label, graph, objective, shortest, bounds, True, False) < _queue_key(
        later_label, graph, objective, shortest, bounds, True, False
    )
    tight_bounds = replace(
        objective.bounds,
        service_ub={**objective.bounds.service_ub, "C2": -1.0},
    )
    tight_reward = _dual_reward_bound(label, graph, bounds, replace(objective, bounds=tight_bounds), shortest)
    assert tight_reward < reward

    zero_duals = PricingDuals(mu={customer: 0.0 for customer in residual}, kappa=0.0)
    result = price_route(
        graph, objective, residual, BranchRestrictions(), zero_duals, 0,
        farkas=True, pricing_tolerance=0.05, use_standard_acceleration=False,
        batch_size=32, parallel_workers=2, pricing_worker_backend="thread", pricing_mode="closure",
    )
    assert result.diagnostics.exact_completion
    assert result.diagnostics.standard_bound_pruned == 0
    assert result.diagnostics.farkas_bound_pruned > 0


def test_farkas_bound_pricing_matches_exhaustive_route_enumeration() -> None:
    instance, _, objective, graph = _setup()
    residual = frozenset(instance.customers)
    duals = PricingDuals(mu={customer: 1.5 for customer in residual}, kappa=-0.5)
    network = nx.DiGraph()
    network.add_nodes_from(graph.nodes)
    network.add_edges_from(graph.arcs)
    brute_costs: list[float] = []
    for index, path in enumerate(
        nx.all_simple_paths(network, instance.depot_source, instance.depot_sink, cutoff=len(graph.nodes) - 1)
    ):
        try:
            route = route_from_path(index, tuple(path), graph, objective)
        except ValueError:
            continue
        brute_costs.append(route_farkas_reduced_cost(route, duals))
    result = price_route(
        graph,
        objective,
        residual,
        BranchRestrictions(),
        duals,
        0,
        farkas=True,
        pricing_tolerance=0.0,
        batch_size=1024,
        parallel_workers=1,
        pricing_worker_backend="thread",
        pricing_mode="closure",
    )
    assert result.diagnostics.exact_completion
    assert result.best_reduced_cost == pytest.approx(min(brute_costs))


@pytest.mark.parametrize(
    "restrictions",
    (
        BranchRestrictions().with_together("C1", "C2"),
        BranchRestrictions().with_separate("C1", "C2"),
        BranchRestrictions().with_pad_forbidden("H1", "C1"),
        BranchRestrictions().with_pad_required("H1", "C1"),
        BranchRestrictions().with_conditioned_arc_forbidden("C1", ("Source", "C1")),
        BranchRestrictions().with_conditioned_arc_required("C1", ("Source", "C1")),
    ),
)
@pytest.mark.parametrize("farkas", (False, True))
def test_standard_and_farkas_pricing_respect_every_branch_family(
    restrictions: BranchRestrictions,
    farkas: bool,
) -> None:
    instance, _, objective, graph = _setup()
    duals = PricingDuals(mu={customer: 100.0 for customer in instance.customers}, kappa=0.0)
    result = price_route(
        graph,
        objective,
        frozenset(instance.customers),
        restrictions,
        duals,
        0,
        farkas=farkas,
        pricing_tolerance=0.01,
        use_standard_acceleration=not farkas,
        batch_size=64,
        parallel_workers=1,
        pricing_worker_backend="thread",
        pricing_mode="closure",
    )
    assert result.diagnostics.exact_completion
    assert all(restrictions.route_allowed(route) for route in result.routes)


def test_pricing_tolerance_controls_all_negative_column_acceptance() -> None:
    instance, _, objective, graph = _setup()
    residual = frozenset({"C1"})
    base_duals = PricingDuals(mu={"C1": 0.0}, kappa=0.0)
    base = price_route(
        graph, objective, residual, BranchRestrictions(), base_duals, 0,
        pricing_tolerance=0.0, use_standard_acceleration=False, batch_size=128, pricing_mode="closure",
    )
    assert base.best_reduced_cost is not None
    not_negative = price_route(
        graph, objective, residual, BranchRestrictions(),
        PricingDuals(mu={"C1": base.best_reduced_cost + 0.049}, kappa=0.0), 0,
        pricing_tolerance=0.05, use_standard_acceleration=False, batch_size=128, pricing_mode="closure",
    )
    negative = price_route(
        graph, objective, residual, BranchRestrictions(),
        PricingDuals(mu={"C1": base.best_reduced_cost + 0.051}, kappa=0.0), 0,
        pricing_tolerance=0.05, use_standard_acceleration=False, batch_size=128, pricing_mode="closure",
    )
    assert not not_negative.routes
    assert negative.routes
    assert all(cost < -0.05 for cost in negative.reduced_costs)


def test_fractional_knapsack_reward_bound_has_no_cardinality_alternative() -> None:
    instance, _, objective, graph = _setup()
    duals = PricingDuals(mu={customer: 10.0 for customer in instance.customers}, kappa=0.0)
    shortest = _shortest_truck_times(graph)
    bounds = _build_pricing_bounds(graph, duals, frozenset(instance.customers), shortest)
    label = _Label(
        path=("Source",), represented=frozenset(), truck_visited=frozenset({"Source"}), truck_load=0.0,
        active_pad=None, active_pad_arrival=0.0, active_wait=0.0, block_count=0, physical_time=0.0,
        service_times=tuple(), sr_counts=tuple(), reduced_cost=0.0, used_arcs=frozenset(),
    )
    value = _dual_reward_bound(label, graph, bounds, objective, shortest, _DeadlinePricingCounters(shortest=shortest))
    assert value > 0.0
    assert not hasattr(pricing_module, "_resource_restricted_reward_bound")


def test_active_sr_set_is_monotone_within_node() -> None:
    instance, _, _, _ = _setup()
    node = _root_node(instance, set())
    stats = BPCStats()
    first = ("C1", "C2", "C3")
    assert _activate_sr_cuts(node, {first: 1.4}, stats) == 1
    version = node.active_sr_version
    assert _activate_sr_cuts(node, {first: 1.2}, stats) == 0
    assert node.active_sr == {first}
    assert node.active_sr_version == version
    assert not hasattr(node, "removed_sr")


def test_row_local_sr_coefficient_cache_matches_popcount_truth() -> None:
    instance, _, objective, graph = _setup()
    path = ("Source", "C1", "C2", "C3", "Sink")
    route = route_from_path(0, path, graph, objective)
    routes = {path: route}
    triplet = ("C1", "C2", "C3")
    node = _root_node(instance, {path}, {triplet})
    cache = RouteSignatureCache()
    rmp = RestrictedMaster(graph, node, routes, SolverConfig(enable_root_compact_warm_start=False), cache)
    value = rmp._sr_coeff(path, triplet)
    residual = tuple(sorted(node.residual_customers))
    truth = sr_coeff_from_mask(customer_mask(route.served, residual, cache), triplet_mask(residual, triplet, cache))
    assert value == truth == 1
    assert rmp._sr_coeff(path, triplet) == truth
    assert cache.stats.sr_coeff_cache_hits >= 1


def test_incremental_rmp_matches_full_rebuild_objective() -> None:
    instance, _, objective, graph = _setup()
    full_path = ("Source", "C1", "C2", "C3", "Sink")
    alternative_path = ("Source", "C1", "C3", "C2", "Sink")
    routes = {
        full_path: route_from_path(0, full_path, graph, objective),
        alternative_path: route_from_path(1, alternative_path, graph, objective),
    }
    incremental_node = _root_node(instance, {full_path})
    incremental_config = SolverConfig(enable_root_compact_warm_start=False, enable_incremental_rmp=True)
    first = RestrictedMaster(graph, incremental_node, routes, incremental_config, RouteSignatureCache()).solve()
    assert first.objective is not None
    incremental_node.column_paths.add(alternative_path)
    incremental = RestrictedMaster(graph, incremental_node, routes, incremental_config, RouteSignatureCache()).solve()
    rebuilt_node = _root_node(instance, {full_path, alternative_path})
    rebuilt = RestrictedMaster(
        graph,
        rebuilt_node,
        routes,
        replace(incremental_config, enable_incremental_rmp=False),
        RouteSignatureCache(),
    ).solve()
    assert rebuilt.objective == pytest.approx(incremental.objective)


def test_full_pool_hard_model_uses_true_route_cost() -> None:
    instance, _, objective, graph = _setup()
    path = ("Source", "C1", "C2", "C3", "Sink")
    route = route_from_path(0, path, graph, objective)
    routes = {path: route}
    node = _root_node(instance, {path})
    result = run_route_pool_heuristic(
        graph, objective, node, routes, {path},
        SolverConfig(enable_root_compact_warm_start=False, pricing_parallel_workers=1),
        incumbent_value=float("inf"),
    )
    assert result.value == pytest.approx(route.cost)
    assert result.selected_routes == (route,)
    assert result.diagnostics.full_pool_feasible == 1
    assert not hasattr(result.diagnostics, "soft_pool_solves")


def test_infeasible_hard_pool_returns_no_incumbent_without_repair(monkeypatch) -> None:
    instance, _, objective, graph = _setup()
    node = _root_node(instance, set())
    calls: list[str] = []
    monkeypatch.setattr(
        heuristic_module,
        "_solve_hard_pool_ip",
        lambda *args, **kwargs: (calls.append("hard") or (None, tuple())),
    )
    result = run_route_pool_heuristic(
        graph, objective, node, {}, set(), SolverConfig(enable_root_compact_warm_start=False),
        incumbent_value=float("inf"),
    )
    assert calls == ["hard"]
    assert result.value is None
    assert not hasattr(heuristic_module, "_solve_soft_pool_ip")


def test_unresolved_hard_pool_returns_no_incumbent(monkeypatch) -> None:
    instance, _, objective, graph = _setup()
    node = _root_node(instance, set())
    calls: list[str] = []
    monkeypatch.setattr(
        heuristic_module,
        "_solve_hard_pool_ip",
        lambda *args, **kwargs: (calls.append("hard") or (None, tuple())),
    )

    result = run_route_pool_heuristic(
        graph, objective, node, {}, set(), SolverConfig(enable_root_compact_warm_start=False),
        incumbent_value=float("inf"),
    )
    assert calls == ["hard"]
    assert result.value is None


def test_empty_initial_rmp_proceeds_directly_to_farkas_pricing() -> None:
    instance, _, objective, graph = _setup()
    node = _root_node(instance, set())
    routes: dict[tuple[str, ...], Route] = {}
    stats = BPCStats(pricing_tolerance=0.01)
    result = _solve_node(
        graph,
        objective,
        SolverConfig(
            time_limit=30.0,
            pricing_parallel_workers=1,
            pricing_worker_backend="thread",
            enable_root_compact_warm_start=False,
        ),
        node,
        routes,
        set(),
        RouteSignatureCache(),
        _IncumbentState(),
        stats,
        time.time() + 30.0,
        time.time(),
        None,
        {},
    )
    assert result is not None
    assert stats.farkas_pricing_calls > 0
    assert stats.columns_added_farkas > 0


def test_compact_initialization_decodes_and_verifies_returned_paths(monkeypatch) -> None:
    instance, weights, objective, graph = _setup()
    path = ("Source", "C1", "C2", "C3", "Sink")
    monkeypatch.setattr(
        bpc_module,
        "solve_compact_solution",
        lambda *args, **kwargs: CompactSolution(0.0, (path,), CompactTiming(1.0, 60.0, 2.0), "time_limit"),
    )
    routes = {}
    stats = BPCStats()
    extracted = _extract_root_routes(
        instance, weights, SolverConfig(root_compact_solve_time_limit=60.0), graph, objective,
        routes, stats, time.time() + 1.0,
    )
    assert extracted == {path}
    assert routes[path] == route_from_path(0, path, graph, objective)
    assert stats.root_model_build_time == pytest.approx(1.0)
    assert stats.root_model_solve_time == pytest.approx(60.0)
    assert stats.root_route_decode_time == pytest.approx(2.0)
    assert stats.root_compact_incumbent_validated
    assert stats.root_compact_route_paths == (path,)


def test_validate_route_cover_rejects_missing_or_duplicate_customer_coverage() -> None:
    _, _, objective, graph = _setup()
    complete = ("Source", "C1", "C2", "C3", "Sink")
    routes = validate_route_cover((complete,), graph, objective)
    assert len(routes) == 1
    assert routes[0].served == frozenset(("C1", "C2", "C3"))

    with pytest.raises(ValueError, match="exactly once"):
        validate_route_cover((("Source", "C1", "Sink"),), graph, objective)
    with pytest.raises(ValueError, match="exactly once"):
        validate_route_cover((complete, complete), graph, objective)


def test_warm_start_screening_accepts_valid_cover_and_resumes_from_record(monkeypatch, tmp_path: Path) -> None:
    instance, weights, _, _ = _setup()
    path = ("Source", "C1", "C2", "C3", "Sink")
    calls = 0

    def solve(*args, **kwargs):
        nonlocal calls
        calls += 1
        return CompactSolution(
            0.25,
            (path,),
            CompactTiming(0.1, 1.0, 0.1),
            "success",
            objective_bound_full=0.2,
            mip_gap=0.2,
            status_code=2,
            node_count=3.0,
            iteration_count=4.0,
            first_incumbent_time=0.5,
        )

    monkeypatch.setattr("thvrpd.compact.solve_compact_solution", solve)
    candidate_dir = tmp_path / "candidate_000"
    first = screen_warm_start_candidate(instance, 0, 1, candidate_dir, weights, 60.0, 45.0, 2, 0)
    second = screen_warm_start_candidate(instance, 0, 1, candidate_dir, weights, 60.0, 45.0, 2, 0)

    assert calls == 2
    assert first == second
    assert first.accepted
    assert first.status == "warm_start_certified"
    assert first.diagnostics["validation"]["valid"] is True
    assert first.diagnostics["validation"]["route_count"] == 1


def test_warm_start_screening_rejects_timeout_without_incumbent(monkeypatch, tmp_path: Path) -> None:
    instance, weights, _, _ = _setup()
    monkeypatch.setattr(
        "thvrpd.compact.solve_compact_solution",
        lambda *args, **kwargs: CompactSolution(
            None,
            tuple(),
            CompactTiming(0.1, 60.0, 0.0),
            "timeout",
            status_code=9,
            node_count=100.0,
            iteration_count=200.0,
        ),
    )
    result = screen_warm_start_candidate(
        instance,
        0,
        1,
        tmp_path / "candidate_000",
        weights,
        60.0,
        45.0,
        2,
        0,
    )

    assert not result.accepted
    assert result.status == "warm_start_rejected_no_incumbent"
    assert result.diagnostics["validation"]["valid"] is None


def test_warm_start_screening_rejects_incumbent_after_safety_deadline(monkeypatch, tmp_path: Path) -> None:
    instance, weights, _, _ = _setup()
    path = ("Source", "C1", "C2", "C3", "Sink")
    monkeypatch.setattr(
        "thvrpd.compact.solve_compact_solution",
        lambda *args, **kwargs: CompactSolution(
            0.25,
            (path,),
            CompactTiming(0.1, 60.0, 0.1),
            "success",
            first_incumbent_time=50.0,
        ),
    )
    result = screen_warm_start_candidate(
        instance,
        0,
        1,
        tmp_path / "candidate_000",
        weights,
        60.0,
        45.0,
        2,
        0,
    )

    assert not result.accepted
    assert result.status == "warm_start_rejected_late_incumbent"
    assert result.diagnostics["screening_repetitions_completed"] == 1
    assert result.diagnostics["validation"]["valid"] is True


def test_pc8_performance_selection_predicates_require_low_bpc_gap_and_compact_timeout() -> None:
    valid = {"valid": True}
    base = {"gap_full": 0.05, "bpc_stats": {"root_compact_incumbent_validated": True}}
    assert bpc_trial_qualifies(base, valid, 0.05)
    assert not bpc_trial_qualifies({**base, "gap_full": 0.0500001}, valid, 0.05)
    assert not bpc_trial_qualifies(base, {"valid": False}, 0.05)
    assert not bpc_trial_qualifies(
        {**base, "bpc_stats": {"root_compact_incumbent_validated": False}},
        valid,
        0.05,
    )
    assert compact_trial_qualifies({"status_code": 9})
    assert not compact_trial_qualifies({"status_code": 2})
    assert cross_solver_bounds_consistent(
        {"lower_bound_full": 0.10},
        {"objective_full": 0.11},
    )
    assert not cross_solver_bounds_consistent(
        {"lower_bound_full": 0.11},
        {"objective_full": 0.10},
    )


def test_compact_is_the_only_root_initializer_and_sets_complete_incumbent(monkeypatch) -> None:
    instance, weights, _, _ = _setup()
    path = ("Source", "C1", "C2", "C3", "Sink")
    monkeypatch.setattr(
        bpc_module,
        "solve_compact_solution",
        lambda *args, **kwargs: CompactSolution(0.0, (path,), CompactTiming(0.0, 0.0, 0.0), "time_limit"),
    )

    result = solve_branch_price_cut(
        instance,
        weights,
        SolverConfig(time_limit=30.0, pricing_parallel_workers=1, pricing_worker_backend="thread"),
    )

    assert result.stats.root_compact_attempted
    assert result.stats.root_compact_accepted_columns == 1
    assert result.stats.incumbent_source == "compact_root"
    assert not any(name.startswith("root_constructive") for name in BPCStats.__dataclass_fields__)


@pytest.mark.parametrize(
    "paths, expected_columns",
    (
        (tuple(), 0),
        ((("Source", "C1", "Sink"),), 1),
    ),
)
def test_empty_or_partial_compact_initialization_proceeds_to_farkas(
    monkeypatch,
    paths: tuple[tuple[str, ...], ...],
    expected_columns: int,
) -> None:
    instance, weights, _, _ = _setup()
    monkeypatch.setattr(
        bpc_module,
        "solve_compact_solution",
        lambda *args, **kwargs: CompactSolution(None, paths, CompactTiming(0.0, 0.0, 0.0), "time_limit"),
    )

    result = solve_branch_price_cut(
        instance,
        weights,
        SolverConfig(time_limit=30.0, pricing_parallel_workers=1, pricing_worker_backend="thread"),
    )

    assert result.stats.root_compact_attempted
    assert result.stats.root_compact_accepted_columns == expected_columns
    assert not result.stats.root_compact_incumbent_validated
    assert result.stats.farkas_pricing_calls > 0
    assert result.stats.columns_added_farkas > 0
    assert result.stats.incumbent_source != "compact_root"


def test_compact_objective_uses_same_normalization_shift() -> None:
    instance, weights, objective, graph = _setup()
    compact = solve_compact_solution(instance, weights, time_limit=30.0, threads=1, require_optimal=True, objective=objective)
    routes = tuple(route_from_path(index, path, graph, objective) for index, path in enumerate(compact.route_paths))
    shifted = sum(route.cost for route in routes)
    assert compact.objective_full == pytest.approx(objective.full_value_from_route_sum(shifted), abs=1e-6)


def test_bpc_and_compact_report_same_full_objective_on_tiny_instance() -> None:
    instance, weights, objective, _ = _setup()
    config = SolverConfig(
        time_limit=60.0, pricing_tolerance=1e-9, pricing_parallel_workers=1, enable_root_compact_warm_start=False,
    )
    bpc = solve_branch_price_cut(instance, weights, config)
    compact = solve_compact_solution(instance, weights, time_limit=60.0, threads=1, require_optimal=True, objective=objective)
    assert bpc.objective_full == pytest.approx(compact.objective_full, abs=1e-6)
    assert bpc.objective_full == pytest.approx(bpc.objective_shifted + bpc.objective.coeffs.shift)
    assert bpc.lower_bound_full == pytest.approx(bpc.lower_bound_shifted + bpc.objective.coeffs.shift)
    assert bpc.stats.pricing_tolerance == pytest.approx(1e-9)
    assert bpc.stats.pricing_engine == "source_neighbor_parallel_forward"
    assert bpc.stats.root_rmp_is_integer is not None
    assert bpc.stats.root_fractional_variable_count is not None
    assert bpc.stats.root_nonzero_variable_count is not None
    assert bpc.stats.root_max_integrality_violation is not None
    assert bpc.stats.root_fathom_reason is not None


def test_queue_bound_fathoming_and_gap_invariant() -> None:
    assert _bound_fathoms(0.48, 0.47, 1e-6)
    assert _bound_fathoms(0.4699995, 0.47, 1e-6)
    assert not _bound_fathoms(0.46, 0.47, 1e-6)
    assert not _bound_fathoms(0.48, float("inf"), 1e-6)
    assert _relative_gap(0.5, 0.4) == pytest.approx(0.2)
    with pytest.raises(ValueError, match="lower bound exceeds upper bound"):
        _relative_gap(0.4, 0.5)


def test_root_bound_inconsistency_is_reported(monkeypatch) -> None:
    _validate_root_bound_order(0.4, 0.4, 1e-6)
    _validate_root_bound_order(0.4000005, 0.4, 1e-6)
    with pytest.raises(BPCBoundInconsistency, match="root lower bound exceeds") as exc_info:
        _validate_root_bound_order(0.41, 0.4, 1e-6)
    assert exc_info.value.lower_bound == pytest.approx(0.41)
    assert exc_info.value.upper_bound == pytest.approx(0.4)
    assert exc_info.value.tolerance == pytest.approx(1e-6)

    events = []
    monkeypatch.setattr(
        bpc_module,
        "_write_progress",
        lambda _config, _stats, _node, event, extra=None: events.append((event, extra)),
    )
    root = NodeState(
        id=1,
        depth=0,
        restrictions=BranchRestrictions(),
        fixed_routes=tuple(),
        residual_customers=frozenset(),
        fleet_limit=1,
        fixed_cost=0.0,
        column_paths=set(),
        active_sr=set(),
    )
    with pytest.raises(BPCBoundInconsistency):
        bpc_module._report_root_bound_inconsistency(root, 0.41, 0.4, SolverConfig(), BPCStats())
    assert events == [
        (
            "invalid_root_bound",
            {"lower_bound": 0.41, "upper_bound": 0.4, "tolerance": 1e-6},
        )
    ]


def test_rmp_integrality_diagnostics_distinguish_fractional_root() -> None:
    integral = _rmp_integrality_diagnostics(
        {("r1",): 1.0, ("r2",): 1e-8, ("r3",): 0.0},
        1e-6,
    )
    assert integral[:3] == (True, 0, 1)
    assert integral[3] == pytest.approx(1e-8)

    fractional = _rmp_integrality_diagnostics(
        {("r12",): 0.5, ("r13",): 0.5, ("r23",): 0.5, ("r123",): 0.0},
        1e-6,
    )
    assert fractional[:3] == (False, 3, 3)
    assert fractional[3] == pytest.approx(0.5)


def test_bpc_uses_balanced_process_scheduler_without_changing_warm_start_contract() -> None:
    instance, weights, _, _ = _setup()
    config = SolverConfig(
        time_limit=60.0,
        pricing_parallel_workers=2,
        pricing_worker_backend="process",
        enable_root_compact_warm_start=False,
    )
    result = solve_branch_price_cut(instance, weights, config)
    assert result.stats.pricing_balanced_process_dynamic_calls > 0
    assert result.stats.pricing_parallel_workers_max == 2
    assert not result.stats.root_compact_attempted
    assert result.stats.pricing_stale_worker_results_discarded == 0


def test_retired_pricing_symbols_are_absent() -> None:
    for name in (
        "_BackwardLabel", "_price_route_bidirectional", "_join_forward_backward",
        "_build_balanced_dynamic_task_plan", "_select_diverse_pricing_candidates",
        "_resource_restricted_reward_bound",
    ):
        assert not hasattr(pricing_module, name)


def test_retired_seeding_repair_basis_and_inactive_interfaces_are_absent() -> None:
    config_fields = SolverConfig.__dataclass_fields__
    for name in (
        "repair_reward",
        "seed_reward",
        "phase_i_max_rounds",
        "seed_batch_size",
        "repair_batch_size",
        "enable_rmp_basis_reuse",
        "enable_inactive_column_storage",
        "column_inactive_age_min",
        "column_active_value_tol",
        "column_deactivation_min_active_columns",
        "column_deactivation_batch_size",
    ):
        assert name not in config_fields
    node = _root_node(tiny_instance(), set())
    for name in (
        "basis_variables",
        "basis_cover",
        "basis_fleet",
        "basis_sr",
        "inactive_column_paths",
        "column_age",
        "pending_column_deactivation_bound",
    ):
        assert not hasattr(node, name)
    assert not Path("thvrpd/phasei.py").exists()
    assert not hasattr(bpc_module, "_run_phase_i_seeding")
    assert not hasattr(bpc_module, "_add_initial_routes")
    assert not hasattr(bpc_module, "_construct_root_incumbent_routes")
    assert not hasattr(bpc_module, "_best_constructive_extension")
    assert not hasattr(bpc_module, "_constructive_connectors")
    assert not hasattr(heuristic_module, "_solve_soft_pool_ip")
