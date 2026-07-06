from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys
from dataclasses import replace
import time

import pytest
from gurobipy import GRB

import thvrpd.bpc as bpc_module
from thvrpd.bpc import (
    BPCTimeLimitNoIncumbent,
    BPCStats,
    _AdaptiveProductiveSliceController,
    _ClosureAwarePricingState,
    _activate_sr_cuts,
    _add_side_pool_routes,
    _add_initial_routes,
    _branch,
    _blend_pricing_duals,
    _insert_node_column,
    _inherit_child_routes,
    _merge_duplicate_column_paths_for_node,
    _remove_inactive_sr_cuts_after_closure,
    _prune_side_pool,
    _record_branch_decision,
    _record_productive_slice_timeout,
    _construct_root_incumbent_routes,
    _deactivate_inactive_node_columns,
    _diversify_constructive_drone_routes,
    _extract_root_routes,
    _child_certification_signature,
    _record_child_certification_epoch_discard,
    _ensure_child_closure_batch_state,
    _observe_child_certification_yield,
    _rehydrate_negative_inactive_columns,
    _sync_closure_aware_pricing_stats,
    _write_progress,
    solve_branch_price_cut,
)
from thvrpd.columns import (
    NodeColumnIndex,
    RouteSignatureCache,
    build_branch_route_index,
    customer_mask,
    extend_branch_route_index,
    insert_node_column,
    merge_duplicate_column_paths,
    query_branch_route_index,
    refresh_node_column_index,
    route_coefficient_signature,
    route_signature,
    sr_coeff_from_mask,
    triplet_mask,
)
from thvrpd.compact import CompactSolution, CompactTiming, solve_compact_miqp, solve_compact_solution
from thvrpd.config import InstanceConfig, ObjectiveWeights, SolverConfig
from thvrpd.heuristics import run_route_pool_heuristic
from thvrpd.instance import InstanceData, generate_instance, tiny_instance
from thvrpd.objective import build_objective_data
from thvrpd.phasei import PhaseISeeder
from thvrpd.pricing import (
    PricingDiagnostics,
    PricingDuals,
    PricingTimeLimitReached,
    _BackwardInterface,
    _DeadlinePricingCounters,
    _JoinEvalCache,
    _Label,
    _backward_dominates,
    _branch_state,
    _build_backward_label,
    _build_pricing_bounds,
    _bucketed_join_generators,
    _bucketed_join_pairs,
    _compatible_dominance_keys,
    _compatible_join_lookup_keys,
    _chunk_source_neighbors,
    _DominanceCounter,
    _dominance_candidate_labels,
    _dominance_bucket_key,
    _dominance_buckets_compatible,
    _dual_reward_bound,
    _dual_solution_key,
    _evaluate_time_expr,
    _expand_backward_label,
    _farkas_dominates,
    _insert_nondominated_backward_label,
    _insert_nondominated_farkas_label,
    _insert_nondominated_standard_label,
    _iter_join_generator_pairs,
    _join_forward_backward,
    _join_group_lower_bound,
    _join_lower_bound,
    _joined_reduced_cost,
    _joined_reduced_cost_cached,
    _join_prefilter,
    _paper_dominates,
    _partition_source_neighbors,
    _select_diverse_pricing_candidates,
    _merge_compact_worker_results,
    _shortest_truck_times,
    _source_neighbor_prefix_tasks,
    _sr_join_correction,
    _try_build_backward_label,
    _WorkerPricingResult,
    _admissible_source_neighbors,
    _extension_rejected_by_service_deadline,
    _extend,
    price_route,
    run_source_neighbor_parallel_forward_pricing,
    route_farkas_reduced_cost,
    route_reduced_cost,
    SourceNeighborPricingPool,
)
from thvrpd.rmp import (
    NodeState,
    RMPResult,
    RestrictedMaster,
    SRCutMetadata,
    _farkas_column_activity,
    _farkas_rhs,
    _validate_farkas_certificate,
)
from thvrpd.routes import route_from_path
from thvrpd.solve import _build_solve_record, _build_solve_timeout_record
from thvrpd.transform import build_transformed_graph, duplicate_node
from thvrpd.branching import BranchRestrictions


def _setup():
    instance = tiny_instance()
    weights = ObjectiveWeights(0.4, 0.3, 0.3)
    objective = build_objective_data(instance, weights)
    graph = build_transformed_graph(instance)
    return instance, weights, objective, graph


def _arc_customer_sets(graph):
    return {arc: graph.arc_customer_set(arc) for arc in graph.arcs}


def _with_arrival_deadlines(base: InstanceData) -> InstanceData:
    baseline = build_objective_data(base, ObjectiveWeights(0.4, 0.3, 0.3))
    manual_bounds = {customer: baseline.bounds.arrival_lb[customer] for customer in base.customers}
    return replace(base, config=replace(base.config, service_deadline_mode="manual", service_deadline_manual_bounds=manual_bounds))


def _source_label(instance, graph) -> _Label:
    return _Label(
        path=(instance.depot_source,),
        represented=frozenset(),
        truck_visited=frozenset({instance.depot_source}),
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
        represented_mask=0,
        truck_node_mask=0,
    )


def test_forward_extension_rejects_customer_after_service_envelope() -> None:
    base = tiny_instance()
    instance = _with_arrival_deadlines(base)
    objective = build_objective_data(instance, ObjectiveWeights(0.4, 0.3, 0.3))
    graph = build_transformed_graph(instance)
    duals = PricingDuals(mu={customer: 0.0 for customer in instance.customers}, kappa=0.0)
    label = _extend(_source_label(instance, graph), "C2", graph, objective, duals, tuple(), False)
    assert _extension_rejected_by_service_deadline(label, "C3", graph, objective)


def test_deadline_aware_reward_bound_removes_unreachable_customer() -> None:
    base = tiny_instance()
    instance = _with_arrival_deadlines(base)
    objective = build_objective_data(instance, ObjectiveWeights(0.4, 0.3, 0.3))
    graph = build_transformed_graph(instance)
    duals = PricingDuals(mu={customer: 0.0 for customer in instance.customers}, kappa=0.0)
    duals = PricingDuals(mu={**duals.mu, "C3": 1.0}, kappa=0.0)
    label = _extend(_source_label(instance, graph), "C2", graph, objective, duals, tuple(), False)
    shortest = _shortest_truck_times(graph)
    bounds = _build_pricing_bounds(graph, duals, frozenset({"C3"}), shortest)
    counters = _DeadlinePricingCounters(shortest=shortest)
    assert _dual_reward_bound(label, graph, bounds, objective, shortest, counters) == 0.0
    assert counters.deadline_reward_bound_calls == 1
    assert counters.reward_set_size_before_deadline == 1
    assert counters.reward_set_size_after_deadline == 0
    assert counters.deadline_reachability_removed == 1


def test_deadline_pricing_diagnostics_and_forward_only_engine() -> None:
    base = tiny_instance()
    instance = _with_arrival_deadlines(base)
    objective = build_objective_data(instance, ObjectiveWeights(0.4, 0.3, 0.3))
    graph = build_transformed_graph(instance)
    duals = PricingDuals(mu={customer: 1.0 for customer in instance.customers}, kappa=0.0)
    result = price_route(
        graph,
        objective,
        frozenset(instance.customers),
        BranchRestrictions(),
        duals,
        next_route_id=0,
        pricing_tolerance=1e-7,
        batch_size=2,
        pricing_worker_backend="thread",
        parallel_workers=2,
    )
    assert result.diagnostics.pricing_engine == "source_neighbor_parallel_forward"
    assert result.diagnostics.backward_labels_generated == 0
    assert result.diagnostics.join_pairs_tested == 0
    assert result.diagnostics.extensions_attempted > 0
    assert result.diagnostics.extensions_rejected_by_deadline >= 0
    assert result.diagnostics.deadline_reward_bound_calls >= 0


def test_root_constructive_heuristic_produces_verified_service_window_incumbent() -> None:
    instance, _, objective, graph = _setup()
    routes = {}
    _add_initial_routes(graph, objective, routes)
    stats = BPCStats()
    generated, selected = _construct_root_incumbent_routes(
        graph,
        objective,
        SolverConfig(root_constructive_time_limit=1.0),
        routes,
        stats,
        time.time() + 5.0,
    )
    assert generated
    assert selected
    selected_routes = tuple(routes[path] for path in selected)
    assert frozenset().union(*(route.served for route in selected_routes)) == frozenset(instance.customers)
    assert all(
        service_time <= objective.bounds.service_ub[customer] + 1e-9
        for route in selected_routes
        for customer, service_time in route.service_times.items()
    )
    assert stats.root_constructive_status == "success"


def test_drone_diversification_warm_start_generates_only_verified_variants() -> None:
    _, _, objective, graph = _setup()
    base_path = ("Source", "H1", "C1", "Sink")
    base_route = route_from_path(0, base_path, graph, objective)
    routes = {base_path: base_route}
    stats = BPCStats(root_constructive_value=base_route.cost)
    generated = _diversify_constructive_drone_routes(
        graph,
        objective,
        SolverConfig(enable_drone_diversification_warm_start=True),
        routes,
        stats,
        bpc_module._IncumbentState(value=base_route.cost, routes=(base_route,)),
        (base_path,),
        time.time(),
        time.time() + 5.0,
    )
    expected = ("Source", "H1", duplicate_node("H1", "C1"), "Sink")
    assert generated == {expected}
    assert expected in routes
    assert routes[expected].served == base_route.served
    assert routes[expected].drone_sorties == 1
    assert all(
        service_time <= objective.bounds.service_ub[customer] + 1e-9
        for customer, service_time in routes[expected].service_times.items()
    )
    assert stats.drone_diversification_attempted is True
    assert stats.drone_diversification_routes_generated == 1
    assert stats.drone_diversification_columns_accepted == 1
    assert stats.drone_insertion_attempts >= 1
    assert stats.drone_insertion_verified == 1
    assert stats.drone_primal_pool_size == 1


def test_compact_warm_start_skips_after_constructive_incumbent_by_policy() -> None:
    instance, weights, objective, graph = _setup()
    stats = BPCStats()
    routes = {}
    extracted = _extract_root_routes(
        instance,
        weights,
        SolverConfig(root_compact_after_constructive="skip"),
        graph,
        objective,
        routes,
        stats,
        time.time() + 60.0,
        constructive_incumbent_found=True,
    )
    assert extracted == set()
    assert stats.root_compact_status == "skipped"
    assert stats.root_compact_skipped_reason == "constructive_incumbent"
    assert stats.root_compact_attempted is False


def test_conditional_compact_policy_triggers_small_budget_for_low_diversity() -> None:
    instance, weights, objective, graph = _setup()
    stats = BPCStats(
        root_constructive_incumbent_found=True,
        root_constructive_diversity_score=0.0,
        root_constructive_drone_sorties=0,
        root_constructive_truck_count=instance.num_trucks,
        root_constructive_value=1.0,
    )
    extracted = _extract_root_routes(
        instance,
        weights,
        SolverConfig(
            root_compact_after_constructive="conditional_small_budget",
            root_compact_time_limit_after_constructive=0.0,
        ),
        graph,
        objective,
        {},
        stats,
        time.time() + 60.0,
        constructive_incumbent_found=True,
    )
    assert extracted == set()
    assert stats.root_compact_conditional_triggered is True
    assert "low_diversity" in stats.root_compact_conditional_reason
    assert stats.root_compact_status == "skipped"
    assert stats.root_compact_skipped_reason == "zero_budget"


def test_conditional_wall_budget_counts_compact_build_solve_decode_and_insertion(monkeypatch) -> None:
    instance, weights, objective, graph = _setup()
    stats = BPCStats(
        root_constructive_incumbent_found=True,
        root_constructive_diversity_score=0.0,
        root_constructive_drone_sorties=0,
        root_constructive_truck_count=instance.num_trucks,
    )
    calls = []

    def fake_compact_solution(*args, **kwargs):
        calls.append(kwargs)
        return CompactSolution(
            objective_full=None,
            route_paths=tuple(),
            timing=CompactTiming(model_build_time=1.25, solve_time=0.0, route_decode_time=0.0),
            status="budget_exhausted_build",
        )

    monkeypatch.setattr(bpc_module, "solve_compact_solution", fake_compact_solution)
    extracted = _extract_root_routes(
        instance,
        weights,
            SolverConfig(
                root_compact_after_constructive="conditional_wall_budget",
                root_compact_wall_time_limit=2.0,
                root_compact_solve_time_limit=0.5,
                compact_after_no_drone_incumbent="small_budget",
            ),
        graph,
        objective,
        {},
        stats,
        time.time() + 60.0,
        constructive_incumbent_found=True,
    )
    assert extracted == set()
    assert calls
    assert calls[0]["time_limit"] == pytest.approx(0.5)
    assert calls[0]["wall_deadline"] is not None
    assert stats.root_compact_budget_seconds == pytest.approx(2.0)
    assert stats.root_compact_wall_budget_seconds == pytest.approx(2.0)
    assert stats.root_compact_solve_budget_seconds == pytest.approx(0.5)
    assert stats.root_compact_status == "budget_exhausted_build"
    assert stats.root_compact_wall_budget_hit is True
    assert stats.root_model_build_time == pytest.approx(1.25)


def test_rmp_insertion_rejects_late_route_object() -> None:
    instance, _, objective, graph = _setup()
    route = route_from_path(0, ("Source", "C1", "Sink"), graph, objective)
    late_route = replace(route, service_times={"C1": objective.bounds.service_ub["C1"] + 10.0})
    node = NodeState(
        id=0,
        depth=0,
        restrictions=BranchRestrictions(),
        fixed_routes=tuple(),
        residual_customers=frozenset(instance.customers),
        fleet_limit=instance.num_trucks,
        fixed_cost=0.0,
        column_paths=set(),
        active_sr=set(),
    )
    stats = BPCStats()
    with pytest.raises(RuntimeError, match="service upper bound"):
        _insert_node_column(late_route, {route.path: route}, node, graph, stats, objective=objective)
    assert stats.late_rmp_routes_rejected == 1


def _two_hub_instance(h1_sink_time: float = 1.0) -> tuple[InstanceData, object, object]:
    config = InstanceConfig(
        seed=11,
        num_trucks=1,
        num_customers=1,
        distribution="PS",
        drones_per_truck=2,
        num_hubs=2,
        truck_payload=10.0,
        drone_payload=10.0,
        drone_endurance=100.0,
        truck_cost=0.0,
        drone_cost=0.0,
    )
    source = "Source"
    sink = "Sink"
    customers = ("C1",)
    hubs = ("H1", "H2")
    nodes = (source,) + customers + hubs + (sink,)
    truck_arcs = {
        (source, "C1"),
        (source, "H1"),
        (source, "H2"),
        ("C1", sink),
        ("C1", "H1"),
        ("C1", "H2"),
        ("H1", sink),
        ("H1", "C1"),
        ("H1", "H2"),
        ("H2", sink),
        ("H2", "C1"),
        ("H2", "H1"),
    }
    truck_time = {arc: 3.0 for arc in truck_arcs}
    truck_time[(source, "H1")] = 1.0
    truck_time[("H1", sink)] = h1_sink_time
    truck_time[("H1", "H2")] = 1.0
    truck_time[("H2", sink)] = 1.0
    truck_time[(source, "C1")] = 2.0
    truck_time[("C1", sink)] = 2.0
    drone_arcs = {("H1", "C1"), ("H2", "C1")}
    drone_time = {("H1", "C1"): 1.0, ("C1", "H1"): 1.0, ("H2", "C1"): 1.0, ("C1", "H2"): 1.0}
    drone_trip_time = {("H1", "C1"): 2.0, ("H2", "C1"): 2.0}
    demand = {source: 0.0, sink: 0.0, "C1": 1.0, "H1": 0.0, "H2": 0.0}
    locations = {source: (0.0, 0.0), sink: (0.0, 0.0), "C1": (1.0, 0.0), "H1": (0.0, 1.0), "H2": (1.0, 1.0)}
    instance = InstanceData(
        config=config,
        depot_source=source,
        depot_sink=sink,
        customers=customers,
        hubs=hubs,
        nodes=nodes,
        truck_arcs=frozenset(truck_arcs),
        drone_arcs=frozenset(drone_arcs),
        truck_time=truck_time,
        drone_time=drone_time,
        drone_trip_time=drone_trip_time,
        demand=demand,
        locations=locations,
    )
    objective = build_objective_data(instance, ObjectiveWeights(0.0, 1.0, 0.0))
    graph = build_transformed_graph(instance)
    return instance, objective, graph


def _pricing_source_label(instance, objective, duals, farkas: bool = False) -> _Label:
    source_cost = -duals.kappa if farkas else objective.coeffs.cost * instance.truck_cost - duals.kappa
    return _Label(
        path=(instance.depot_source,),
        represented=frozenset(),
        truck_visited=frozenset({instance.depot_source}),
        truck_load=0.0,
        active_pad=None,
        active_pad_arrival=0.0,
        active_wait=0.0,
        block_count=0,
        physical_time=0.0,
        service_times=tuple(),
        sr_counts=tuple(),
        reduced_cost=source_cost,
        used_arcs=frozenset(),
    )


def test_backward_label_resources_exclude_meet_node() -> None:
    instance, _, _, graph = _setup()
    suffix = _build_backward_label(("C1", "C2", instance.depot_sink), graph, frozenset(instance.customers))
    assert suffix.path == ("C1", "C2", instance.depot_sink)
    assert suffix.represented == frozenset({"C2"})
    assert suffix.truck_visited == frozenset({"C2"})
    assert suffix.truck_load == pytest.approx(instance.demand["C2"])


def test_backward_expansion_preserves_arc_validity_and_elementarity() -> None:
    instance, _, _, graph = _setup()
    sink_suffix = _build_backward_label((instance.depot_sink,), graph, frozenset(instance.customers))
    expansion = _expand_backward_label(
        sink_suffix,
        graph,
        frozenset(instance.customers),
        tuple(),
        BranchRestrictions(),
        _arc_customer_sets(graph),
    )
    assert expansion.labels
    assert all((i, j) in graph.arcs for label in expansion.labels for i, j in zip(label.path, label.path[1:]))
    duplicate = duplicate_node("H1", "C1")
    assert _try_build_backward_label(("H1", duplicate, "C1", instance.depot_sink), graph, frozenset(instance.customers)) is None


def test_backward_profile_matches_full_route_decoding_after_join() -> None:
    instance, _, objective, graph = _setup()
    restrictions = BranchRestrictions().with_together("C1", "C2").with_trans_arc_required(("H1", duplicate_node("H1", "C1")))
    active_sr = (tuple(instance.customers),)
    suffix = _build_backward_label(
        ("H1", duplicate_node("H1", "C1"), instance.depot_sink),
        graph,
        frozenset(instance.customers),
        active_sr,
        restrictions,
        _arc_customer_sets(graph),
    )
    duals = PricingDuals(mu={c: 0.0 for c in instance.customers}, kappa=0.0, nu={active_sr[0]: -0.1})
    prefix = _extend(_pricing_source_label(instance, objective, duals), "H1", graph, objective, duals, active_sr, False)
    route = route_from_path(0, prefix.path + suffix.path[1:], graph, objective)
    profile = suffix.profile
    assert profile is not None
    assert suffix.represented == frozenset({"C1"})
    assert suffix.truck_served == frozenset()
    assert suffix.pad_served == frozenset({("H1", "C1")})
    assert suffix.sr_counts == ((active_sr[0], 1),)
    assert suffix.branch_state is not None
    assert suffix.branch_state.together == ((True, False),)
    assert suffix.branch_state.required_arcs == ((("H1", duplicate_node("H1", "C1")), True, True),)
    assert _evaluate_time_expr(dict(profile.service_times)["C1"], prefix) == pytest.approx(route.service_times["C1"])
    assert _evaluate_time_expr(profile.return_time, prefix) == pytest.approx(route.return_time)


def test_backward_insertion_rejects_duplicate_suffix_and_prunes_nonidentical_dominated_suffix() -> None:
    instance, objective, graph = _two_hub_instance(h1_sink_time=1.0)
    residual = frozenset(instance.customers)
    restrictions = BranchRestrictions()
    arc_customer_sets = _arc_customer_sets(graph)
    duals = PricingDuals(mu={"C1": 0.0}, kappa=0.0, nu={})
    direct = _build_backward_label(("H1", instance.depot_sink), graph, residual, tuple(), restrictions, arc_customer_sets)
    detour = _build_backward_label(("H1", "H2", instance.depot_sink), graph, residual, tuple(), restrictions, arc_customer_sets)

    by_node: dict[str, list] = {}
    paths: set[tuple[str, ...]] = set()
    assert _insert_nondominated_backward_label(by_node, paths, detour, graph, objective, restrictions, arc_customer_sets, duals, False) == (True, 0, 0, 0)
    inserted, rejected, purged, tests = _insert_nondominated_backward_label(
        by_node,
        paths,
        direct,
        graph,
        objective,
        restrictions,
        arc_customer_sets,
        duals,
        False,
    )
    assert (inserted, rejected, purged) == (True, 0, 1)
    assert tests > 0
    assert by_node["H1"] == [direct]
    assert detour.path not in paths
    assert _insert_nondominated_backward_label(by_node, paths, direct, graph, objective, restrictions, arc_customer_sets, duals, False) == (False, 1, 0, 0)


def test_backward_dominance_rejects_uncertified_resource_and_branch_cases() -> None:
    instance, objective, graph = _two_hub_instance(h1_sink_time=1.0)
    residual = frozenset(instance.customers)
    restrictions = BranchRestrictions()
    arc_customer_sets = _arc_customer_sets(graph)
    duals_positive = PricingDuals(mu={"C1": 5.0}, kappa=0.0, nu={})
    empty_suffix = _build_backward_label(("H1", instance.depot_sink), graph, residual, tuple(), restrictions, arc_customer_sets)
    customer_suffix = _build_backward_label(("H1", "C1", instance.depot_sink), graph, residual, tuple(), restrictions, arc_customer_sets)
    assert not _backward_dominates(
        empty_suffix,
        customer_suffix,
        graph,
        objective,
        restrictions,
        arc_customer_sets,
        duals_positive,
        False,
    )

    duals = PricingDuals(mu={"C1": -1.0}, kappa=0.0, nu={})
    drone_suffix = _build_backward_label(
        ("H1", duplicate_node("H1", "C1"), instance.depot_sink),
        graph,
        residual,
        tuple(),
        restrictions,
        arc_customer_sets,
    )
    assert not _backward_dominates(
        customer_suffix,
        drone_suffix,
        graph,
        objective,
        restrictions,
        arc_customer_sets,
        duals,
        False,
    )
    branch_restrictions = BranchRestrictions().with_route_forbidden(("Source", "H1", instance.depot_sink))
    branch_direct = _build_backward_label(("H1", instance.depot_sink), graph, residual, tuple(), branch_restrictions, arc_customer_sets)
    branch_detour = _build_backward_label(("H1", "H2", instance.depot_sink), graph, residual, tuple(), branch_restrictions, arc_customer_sets)
    assert not _backward_dominates(
        branch_direct,
        branch_detour,
        graph,
        objective,
        branch_restrictions,
        arc_customer_sets,
        duals,
        False,
    )


def test_backward_farkas_dominance_uses_farkas_valid_suffix_comparison() -> None:
    instance, objective, graph = _two_hub_instance(h1_sink_time=10.0)
    residual = frozenset(instance.customers)
    restrictions = BranchRestrictions()
    arc_customer_sets = _arc_customer_sets(graph)
    duals = PricingDuals(mu={"C1": 0.0}, kappa=0.0, nu={})
    slow_direct = _build_backward_label(("H1", instance.depot_sink), graph, residual, tuple(), restrictions, arc_customer_sets)
    fast_detour = _build_backward_label(("H1", "H2", instance.depot_sink), graph, residual, tuple(), restrictions, arc_customer_sets)
    assert not _backward_dominates(
        slow_direct,
        fast_detour,
        graph,
        objective,
        restrictions,
        arc_customer_sets,
        duals,
        False,
    )
    assert _backward_dominates(
        slow_direct,
        fast_detour,
        graph,
        objective,
        restrictions,
        arc_customer_sets,
        duals,
        True,
    )


def test_branch_state_records_together_and_required_arc_automata() -> None:
    instance, _, objective, graph = _setup()
    duals = PricingDuals(mu={c: 0.0 for c in instance.customers}, kappa=0.0, nu={})
    source = _pricing_source_label(instance, objective, duals)
    prefix = _extend(source, "H1", graph, objective, duals, tuple(), False)
    required_arc = ("H1", duplicate_node("H1", "C1"))
    drone_label = _extend(prefix, required_arc[1], graph, objective, duals, tuple(), False)
    truck_label = _extend(source, "C1", graph, objective, duals, tuple(), False)
    restrictions = BranchRestrictions().with_together("C1", "C2").with_trans_arc_required(required_arc)
    arc_customer_sets = {arc: graph.arc_customer_set(arc) for arc in graph.arcs}

    drone_state = _branch_state(drone_label, restrictions, arc_customer_sets)
    truck_state = _branch_state(truck_label, restrictions, arc_customer_sets)

    assert drone_state.together == ((True, False),)
    assert drone_state.required_arcs == ((required_arc, True, True),)
    assert truck_state.required_arcs == ((required_arc, True, False),)


def test_same_node_join_decodes_and_matches_direct_reduced_costs() -> None:
    instance, _, objective, graph = _setup()
    duals = PricingDuals(mu={c: 10.0 for c in instance.customers}, kappa=-1.0, nu={tuple(instance.customers): -0.25})
    source = _pricing_source_label(instance, objective, duals)
    prefix = _extend(source, "H1", graph, objective, duals, tuple(sorted(duals.nu)), False)
    suffix_path = ("H1", duplicate_node("H1", "C1"), instance.depot_sink)
    suffix = _build_backward_label(suffix_path, graph, frozenset(instance.customers))
    arc_customer_sets = {arc: graph.arc_customer_set(arc) for arc in graph.arcs}
    candidate = _join_forward_backward(
        prefix,
        suffix,
        graph,
        objective,
        frozenset(instance.customers),
        BranchRestrictions(),
        arc_customer_sets,
        duals,
        False,
    )
    assert candidate is not None
    full_path = prefix.path + suffix.path[1:]
    route = route_from_path(0, full_path, graph, objective)
    assert candidate.path == full_path
    assert candidate.reduced_cost == pytest.approx(route_reduced_cost(route, duals))

    farkas_source = _pricing_source_label(instance, objective, duals, True)
    farkas_prefix = _extend(farkas_source, "H1", graph, objective, duals, tuple(sorted(duals.nu)), True)
    farkas_candidate = _join_forward_backward(
        farkas_prefix,
        suffix,
        graph,
        objective,
        frozenset(instance.customers),
        BranchRestrictions(),
        arc_customer_sets,
        duals,
        True,
    )
    assert farkas_candidate is not None
    assert farkas_candidate.reduced_cost == pytest.approx(route_farkas_reduced_cost(route, duals))


def test_backward_cost_function_and_sr_join_correction_match_direct_route() -> None:
    instance, _, objective, graph = _setup()
    active_sr = (tuple(instance.customers),)
    duals = PricingDuals(mu={c: 4.0 for c in instance.customers}, kappa=-0.3, nu={active_sr[0]: -0.7})
    source = replace(_pricing_source_label(instance, objective, duals), sr_counts=tuple((triplet, 0) for triplet in active_sr))
    prefix = _extend(source, "C2", graph, objective, duals, active_sr, False)
    suffix = _build_backward_label(
        ("C2", "C1", instance.depot_sink),
        graph,
        frozenset(instance.customers),
        active_sr,
        BranchRestrictions(),
        _arc_customer_sets(graph),
    )
    candidate = _join_forward_backward(
        prefix,
        suffix,
        graph,
        objective,
        frozenset(instance.customers),
        BranchRestrictions(),
        _arc_customer_sets(graph),
        duals,
        False,
    )
    assert candidate is not None
    route = route_from_path(0, candidate.path, graph, objective)
    assert candidate.reduced_cost == pytest.approx(route_reduced_cost(route, duals))
    assert _sr_join_correction(dict(prefix.sr_counts), dict(suffix.sr_counts), duals) == pytest.approx(0.7)

    farkas_source = replace(_pricing_source_label(instance, objective, duals, True), sr_counts=tuple((triplet, 0) for triplet in active_sr))
    farkas_prefix = _extend(farkas_source, "C2", graph, objective, duals, active_sr, True)
    farkas_candidate = _join_forward_backward(
        farkas_prefix,
        suffix,
        graph,
        objective,
        frozenset(instance.customers),
        BranchRestrictions(),
        _arc_customer_sets(graph),
        duals,
        True,
    )
    assert farkas_candidate is not None
    assert farkas_candidate.reduced_cost == pytest.approx(route_farkas_reduced_cost(route, duals))


def test_backward_lower_envelope_is_certified_for_standard_interfaces() -> None:
    instance, _, objective, graph = _setup()
    active_sr = (tuple(instance.customers),)
    duals = PricingDuals(mu={"C1": 5.0, "C2": -2.0, "C3": 0.0}, kappa=0.0, nu={active_sr[0]: -0.4})
    suffix = _build_backward_label(
        ("C2", "C1", instance.depot_sink),
        graph,
        frozenset(instance.customers),
        active_sr,
        BranchRestrictions(),
        _arc_customer_sets(graph),
    )
    assert suffix.cost_function is not None
    assert suffix.lower_envelope is not None
    for interface in (
        _BackwardInterface(physical_time=0.0, pad_arrival=0.0, active_wait=0.0),
        _BackwardInterface(physical_time=7.5, pad_arrival=3.0, active_wait=1.25),
        _BackwardInterface(physical_time=20.0, pad_arrival=12.0, active_wait=6.0),
    ):
        lower = suffix.lower_envelope.evaluate(duals, farkas=False)
        exact = suffix.cost_function.evaluate(interface, graph, objective, duals, farkas=False)
        assert lower <= exact + 1e-9
    with pytest.raises(ValueError, match="Farkas"):
        suffix.lower_envelope.evaluate(duals, farkas=True)


def test_join_lower_bound_valid_and_farkas_bypasses_standard_envelope() -> None:
    instance, _, objective, graph = _setup()
    active_sr = (tuple(instance.customers),)
    duals = PricingDuals(mu={c: 4.0 for c in instance.customers}, kappa=-0.2, nu={active_sr[0]: -0.5})
    source = replace(_pricing_source_label(instance, objective, duals), sr_counts=tuple((triplet, 0) for triplet in active_sr))
    prefix = _extend(source, "C2", graph, objective, duals, active_sr, False)
    suffix = _build_backward_label(
        ("C2", "C1", instance.depot_sink),
        graph,
        frozenset(instance.customers),
        active_sr,
        BranchRestrictions(),
        _arc_customer_sets(graph),
    )
    lower_bound = _join_lower_bound(prefix, suffix, graph, duals, farkas=False)
    exact = _joined_reduced_cost(prefix, suffix, graph, objective, duals, farkas=False)
    assert lower_bound <= exact + 1e-9
    assert _join_lower_bound(prefix, suffix, graph, duals, farkas=True) == float("-inf")
    assert _join_lower_bound(prefix, suffix, graph, duals, farkas=False, enable_join_lower_envelope=False) == float("-inf")


def test_join_interface_cache_invalidation_by_dual_and_active_sr_version() -> None:
    instance, _, objective, graph = _setup()
    active_sr = (tuple(instance.customers),)
    duals = PricingDuals(mu={c: 4.0 for c in instance.customers}, kappa=-0.2, nu={active_sr[0]: -0.5})
    source = replace(_pricing_source_label(instance, objective, duals), sr_counts=tuple((triplet, 0) for triplet in active_sr))
    prefix = _extend(source, "C2", graph, objective, duals, active_sr, False)
    suffix = _build_backward_label(
        ("C2", "C1", instance.depot_sink),
        graph,
        frozenset(instance.customers),
        active_sr,
        BranchRestrictions(),
        _arc_customer_sets(graph),
    )
    cache = _JoinEvalCache()
    key = _dual_solution_key(duals)
    first = _joined_reduced_cost_cached(prefix, suffix, graph, objective, duals, False, cache, 1, key)
    second = _joined_reduced_cost_cached(prefix, suffix, graph, objective, duals, False, cache, 1, key)
    assert first == pytest.approx(_joined_reduced_cost(prefix, suffix, graph, objective, duals, False))
    assert second == pytest.approx(first)
    assert cache.misses == 1
    assert cache.hits == 1
    _joined_reduced_cost_cached(prefix, suffix, graph, objective, duals, False, cache, 2, key)
    assert cache.misses == 2
    duals_changed = PricingDuals(mu={c: 3.0 for c in instance.customers}, kappa=-0.2, nu={active_sr[0]: -0.5})
    _joined_reduced_cost_cached(prefix, suffix, graph, objective, duals_changed, False, cache, 1, _dual_solution_key(duals_changed))
    assert cache.misses == 3


def test_same_node_join_rejects_branch_incompatible_suffix() -> None:
    instance, _, objective, graph = _setup()
    duals = PricingDuals(mu={c: 10.0 for c in instance.customers}, kappa=0.0, nu={})
    source = _pricing_source_label(instance, objective, duals)
    prefix = _extend(source, "H1", graph, objective, duals, tuple(), False)
    suffix = _build_backward_label(("H1", duplicate_node("H1", "C1"), instance.depot_sink), graph, frozenset(instance.customers))
    arc_customer_sets = {arc: graph.arc_customer_set(arc) for arc in graph.arcs}
    candidate = _join_forward_backward(
        prefix,
        suffix,
        graph,
        objective,
        frozenset(instance.customers),
        BranchRestrictions().with_truck_service("C1"),
        arc_customer_sets,
        duals,
        False,
    )
    assert candidate is None


def test_source_neighbor_partition_disjoint_exhaustive() -> None:
    neighbors = tuple(f"N{i}" for i in range(1, 9))
    blocks = _partition_source_neighbors(neighbors, 4)
    assert blocks == (
        ("N1", "N2"),
        ("N3", "N4"),
        ("N5", "N6"),
        ("N7", "N8"),
    )
    flattened = [node for block in blocks for node in block]
    assert flattened == list(neighbors)
    assert sum(len(block) for block in blocks) == len(set(flattened))


def test_parallel_forward_certification_matches_serial_forward() -> None:
    instance, _, objective, graph = _setup()
    duals = PricingDuals(mu={c: 0.0 for c in instance.customers}, kappa=0.0, nu={})
    serial = price_route(
        graph,
        objective,
        frozenset(instance.customers),
        BranchRestrictions(),
        duals,
        next_route_id=700,
        batch_size=1000,
        use_standard_acceleration=False,
        parallel_workers=1,
    )
    threaded = price_route(
        graph,
        objective,
        frozenset(instance.customers),
        BranchRestrictions(),
        duals,
        next_route_id=800,
        batch_size=1000,
        use_standard_acceleration=False,
        pricing_mode="closure",
        parallel_workers=2,
        pricing_worker_backend="thread",
    )
    assert threaded.diagnostics.pricing_engine == "source_neighbor_parallel_forward"
    assert threaded.diagnostics.exact_completion is True
    assert threaded.diagnostics.certification_mode == "source_neighbor_partitions_closed"
    assert threaded.diagnostics.parallel_calls == 1
    assert threaded.diagnostics.source_neighbor_count == len(
        [node for node in graph.out_arcs[instance.depot_source] if node != instance.depot_sink]
    )
    assert threaded.best_reduced_cost == pytest.approx(serial.best_reduced_cost)
    assert threaded.routes == tuple()
    assert threaded.diagnostics.backward_labels_generated == 0
    assert threaded.diagnostics.join_pairs_tested == 0


def test_parallel_forward_farkas_thread_and_process_return_valid_columns() -> None:
    instance, _, objective, graph = _setup()
    duals = PricingDuals(mu={c: 2.0 for c in instance.customers}, kappa=-0.5, nu={tuple(instance.customers): -0.1})
    threaded = price_route(
        graph,
        objective,
        frozenset(instance.customers),
        BranchRestrictions(),
        duals,
        next_route_id=900,
        farkas=True,
        batch_size=1,
        parallel_workers=2,
        pricing_worker_backend="thread",
    )
    processed = price_route(
        graph,
        objective,
        frozenset(instance.customers),
        BranchRestrictions(),
        duals,
        next_route_id=1000,
        farkas=True,
        batch_size=1,
        parallel_workers=2,
        pricing_worker_backend="process",
    )
    assert threaded.routes
    assert processed.routes
    for result in (threaded, processed):
        assert result.diagnostics.pricing_engine == "source_neighbor_parallel_forward"
        assert result.diagnostics.parallel_calls == 1
        assert result.diagnostics.pricing_status == "NEGATIVE_BATCH"
        assert result.diagnostics.backward_labels_generated == 0
        assert result.diagnostics.join_exact_rc_evals == 0
        for route, reduced_cost in zip(result.routes, result.reduced_costs):
            assert reduced_cost < 0.0
            assert reduced_cost == pytest.approx(route_farkas_reduced_cost(route, duals))


def test_source_neighbor_task_chunks_are_disjoint_and_exhaustive() -> None:
    neighbors = ("A", "B", "C", "D", "E")

    chunks = _chunk_source_neighbors(neighbors, 2)

    assert chunks == (("A", "B"), ("C", "D"), ("E",))
    assert tuple(item for chunk in chunks for item in chunk) == neighbors
    assert len(set(item for chunk in chunks for item in chunk)) == len(neighbors)


def test_diversity_selection_is_deterministic_and_uses_verified_routes_only() -> None:
    instance, _, objective, graph = _setup()
    routes = [
        route_from_path(1, ("Source", "C1", "Sink"), graph, objective),
        route_from_path(2, ("Source", "C2", "Sink"), graph, objective),
        route_from_path(3, ("Source", "C1", "C2", "Sink"), graph, objective),
    ]
    candidates = [(routes[0], -3.0, 0), (routes[1], -2.0, 1), (routes[2], -1.0, 0)]

    selected, quota = _select_diverse_pricing_candidates(candidates, frozenset(instance.customers), 2, 0.5)

    assert quota == 1
    assert [route.path for route, _, _ in selected] == [routes[2].path, routes[0].path]
    selected_again, _ = _select_diverse_pricing_candidates(candidates, frozenset(instance.customers), 2, 0.5)
    assert selected_again == selected


def test_persistent_process_pool_reuses_workers_and_returns_batch() -> None:
    instance, _, objective, graph = _setup()
    duals = PricingDuals(mu={c: 100.0 for c in instance.customers}, kappa=0.0, nu={})
    pool = SourceNeighborPricingPool(graph, objective, 2, source_neighbor_task_size=1)
    try:
        first = price_route(
            graph,
            objective,
            frozenset(instance.customers),
            BranchRestrictions(),
            duals,
            next_route_id=1100,
            farkas=False,
            batch_size=4,
            parallel_workers=2,
            pricing_worker_backend="process",
            pricing_process_pool=pool,
            productive_candidate_multiplier=1.0,
            source_neighbor_task_size=1,
            pricing_diversity_batch_fraction=0.5,
        )
        second = price_route(
            graph,
            objective,
            frozenset(instance.customers),
            BranchRestrictions(),
            duals,
            next_route_id=1200,
            farkas=False,
            batch_size=4,
            parallel_workers=2,
            pricing_worker_backend="process",
            pricing_process_pool=pool,
            productive_candidate_multiplier=1.0,
            source_neighbor_task_size=1,
            pricing_diversity_batch_fraction=0.5,
        )
    finally:
        pool.shutdown()
    assert first.routes
    assert len(first.routes) > 1
    assert first.diagnostics.pricing_engine == "source_neighbor_parallel_forward"
    assert first.diagnostics.pricing_worker_backend == "process"
    assert first.diagnostics.pricing_pool_startup_count == 1
    assert second.diagnostics.pricing_pool_startup_count == 0
    assert first.diagnostics.pricing_pool_reused_calls == 1
    assert second.diagnostics.pricing_pool_reused_calls == 1
    assert first.diagnostics.pricing_first_hit_enabled is False
    assert first.diagnostics.first_hit_exits == 0
    assert first.diagnostics.pricing_batch_target == 4
    assert first.diagnostics.pricing_returned_batch_size == len(first.routes)
    assert first.diagnostics.pricing_worker_payload_count == first.diagnostics.source_neighbor_task_count
    assert first.diagnostics.pricing_worker_response_count == first.diagnostics.source_neighbor_task_count
    assert first.diagnostics.source_neighbor_task_count == first.diagnostics.source_neighbor_count
    assert first.diagnostics.local_worker_candidate_quota >= 1
    assert first.diagnostics.diversity_quota == 2
    assert first.diagnostics.diversity_selected_routes == len(first.routes)
    assert first.diagnostics.pricing_candidate_paths_before_merge >= first.diagnostics.pricing_candidate_paths_after_merge
    assert first.diagnostics.pricing_decoded_routes_in_main >= len(first.routes)
    assert first.diagnostics.pricing_verified_routes_in_main >= len(first.routes)
    assert first.diagnostics.backward_labels_generated == 0
    assert first.diagnostics.join_pairs_tested == 0
    for route, reduced_cost in zip(first.routes, first.reduced_costs):
        assert reduced_cost < 0.0
        assert reduced_cost == pytest.approx(route_reduced_cost(route, duals))


def test_persistent_process_merge_rejects_stale_call_id() -> None:
    instance, _, objective, graph = _setup()
    diagnostics = PricingDiagnostics(
        labels_generated=1,
        labels_dominated=0,
        labels_pruned=0,
        max_queue_size=1,
        complete_routes_generated=0,
        returned_routes=0,
        best_reduced_cost=None,
        exact_completion=True,
    )
    stale = _WorkerPricingResult(
        call_id=1,
        dual_id=1,
        worker_id=0,
        route_paths=tuple(),
        reduced_costs=tuple(),
        best_path=None,
        best_reduced_cost=None,
        diagnostics=diagnostics,
    )
    with pytest.raises(RuntimeError, match="stale"):
        _merge_compact_worker_results(
            [stale],
            graph,
            objective,
            frozenset(instance.customers),
            BranchRestrictions(),
            PricingDuals(mu={c: 0.0 for c in instance.customers}, kappa=0.0, nu={}),
            next_route_id=1300,
            farkas=False,
            pricing_tolerance=1e-7,
            existing_routes=None,
            existing_column_paths=None,
            pricing_mode="productive",
            pricing_yield_ratio=0.0,
            pricing_worker_backend="process",
            source_neighbor_count=1,
            source_neighbor_block_sizes=(1,),
            parallel_workers=1,
            selected=None,
            force_time_limit=False,
            call_id=2,
            dual_id=1,
            submission_time_seconds=0.0,
            pool_startup_time_seconds=0.0,
            pool_startup_count=0,
            pool_reused_calls=1,
            first_hit_enabled=False,
            batch_target=4,
        )


def test_productive_slice_timeout_is_unresolved_not_global_time_limit() -> None:
    instance, _, _, _ = _setup()
    node = NodeState(
        id=1,
        depth=0,
        restrictions=BranchRestrictions(),
        fixed_routes=tuple(),
        residual_customers=frozenset(instance.customers),
        fleet_limit=instance.num_trucks,
        fixed_cost=0.0,
        column_paths=set(),
        active_sr=set(),
    )
    stats = BPCStats()
    diagnostics = PricingDiagnostics(
        labels_generated=1,
        labels_dominated=0,
        labels_pruned=0,
        max_queue_size=1,
        complete_routes_generated=0,
        returned_routes=0,
        best_reduced_cost=None,
        exact_completion=False,
        termination_reason="time_limit_unresolved",
        certification_mode="not_certified_time_limit",
        pricing_mode="productive",
        pricing_status="TIME_LIMIT_NO_COLUMNS",
    )

    _record_productive_slice_timeout(
        stats,
        PricingTimeLimitReached(diagnostics),
        SolverConfig(),
        node,
        {},
        set(),
        RouteSignatureCache(),
    )

    assert stats.status == "unknown"
    assert stats.pricing_productive_time_limit_no_columns == 1
    assert stats.pricing_productive_mode_calls == 1
    assert stats.pricing_closure_mode_calls == 0
    assert stats.pricing_diagnostics[-1]["certification_mode"] == "not_certified_time_limit"


def test_solver_config_yield_alignment_defaults_and_validation() -> None:
    config = SolverConfig(first_incumbent_route_pool_time_limit=0.0)

    assert config.enable_bidirectional_pricing is False
    assert config.enable_join_lower_envelope is False
    assert config.enable_bucket_join_envelope is False
    assert config.enable_join_profile_cache is False
    assert config.productive_slice_min_seconds == 5.0
    assert config.productive_slice_max_seconds == 30.0
    assert config.closure_attempt_batch_period == 4
    assert config.closure_attempt_time_period == 120.0
    assert config.closure_batch_size == 32
    assert config.prefix_task_depth == 1
    assert config.prefix_task_depth_root == 1
    assert config.prefix_task_depth_child == 1
    assert config.prefix_task_min_branching_for_depth2 == 4
    assert config.post_incumbent_primal_budget_factor == 0.25
    assert config.enable_constructive_root_incumbent is True
    assert config.root_compact_after_constructive == "conditional_wall_budget"
    assert config.root_compact_time_limit_after_constructive == 1.0
    assert config.root_compact_time_limit_without_constructive == 5.0
    assert config.constructive_diversity_threshold == 0.35
    assert config.enable_sr_aging is True
    assert config.sr_inactive_age_threshold == 1
    assert config.sr_removal_batch_size == 32
    assert config.sr_max_removals_per_node == 32
    assert config.sr_removal_min_active_count == 64
    assert config.sr_removal_rmp_growth_threshold == 0.20
    assert config.enable_node_column_aging is True
    assert config.child_certification_slice_seconds == 30.0
    assert config.child_productive_before_certification is False
    assert config.enable_rmp_basis_reuse is True
    assert config.root_compact_wall_time_limit == 1.0
    assert config.root_compact_solve_time_limit == 1.0
    assert config.enable_drone_diversification_warm_start is True
    assert config.enable_incremental_rmp is True
    assert config.enable_active_coefficient_cache is True
    assert config.enable_global_branch_route_index is True
    assert config.enable_resumable_child_certification is True
    assert config.enable_child_closure_batch_adaptation is False
    assert config.child_closure_batch_min == 16
    assert config.child_closure_batch_initial == 32
    assert config.child_closure_batch_max == 128
    assert config.child_certification_yield_window == 5
    assert config.child_certification_yield_low == pytest.approx(0.20)
    assert config.child_certification_yield_high == pytest.approx(0.60)
    assert config.child_cert_useful_yield_window == 5
    assert config.child_cert_no_route_yield_window == 5
    assert config.child_cert_dual_stability_window == 3
    assert config.child_closure_batch_growth_factor == pytest.approx(1.0)
    assert config.child_closure_batch_shrink_factor == pytest.approx(1.0)
    assert config.child_useful_yield_low == pytest.approx(0.20)
    assert config.child_useful_yield_high == pytest.approx(0.60)
    assert config.child_no_route_yield_high == pytest.approx(1.0)
    assert config.use_row_local_sr_coeff_cache is True
    assert config.use_dominance_prefilter_keys is True
    assert config.use_promised_drone_construction is False
    assert config.no_drone_incumbent_trigger is False
    assert config.compact_after_no_drone_incumbent == "small_budget"
    assert config.logging_mode == "audit"
    assert config.progress_snapshot_period == 1
    assert config.pricing_jsonl_enabled is True

    with pytest.raises(ValueError, match="productive pricing slice must be positive"):
        SolverConfig(productive_pricing_slice_seconds=0.0)
    with pytest.raises(ValueError, match="first-incumbent route-pool time limit must be nonnegative"):
        SolverConfig(first_incumbent_route_pool_time_limit=-1.0)
    with pytest.raises(ValueError, match="productive slice bounds"):
        SolverConfig(productive_slice_min_seconds=40.0, productive_slice_max_seconds=30.0)
    with pytest.raises(ValueError, match="productive yield thresholds"):
        SolverConfig(productive_yield_low_threshold=2.0, productive_yield_high_threshold=1.0)
    with pytest.raises(ValueError, match="prefix_task_depth"):
        SolverConfig(prefix_task_depth=0)
    with pytest.raises(ValueError, match="post-incumbent primal budget factor"):
        SolverConfig(post_incumbent_primal_budget_factor=1.5)
    with pytest.raises(ValueError, match="root_compact_after_constructive"):
        SolverConfig(root_compact_after_constructive="bad")
    with pytest.raises(ValueError, match="logging_mode"):
        SolverConfig(logging_mode="verbose")
    with pytest.raises(ValueError, match="root compact time limits"):
        SolverConfig(root_compact_wall_time_limit=-1.0)
    with pytest.raises(ValueError, match="root compact time limits"):
        SolverConfig(root_compact_solve_time_limit=-1.0)
    with pytest.raises(ValueError, match="child closure batches"):
        SolverConfig(child_closure_batch_min=32, child_closure_batch_initial=16)
    with pytest.raises(ValueError, match="child certification yield thresholds"):
        SolverConfig(child_certification_yield_low=0.9, child_certification_yield_high=0.1)
    with pytest.raises(ValueError, match="child useful yield thresholds"):
        SolverConfig(child_useful_yield_low=0.9, child_useful_yield_high=0.1)
    with pytest.raises(ValueError, match="child no-route yield threshold"):
        SolverConfig(child_no_route_yield_high=1.1)
    with pytest.raises(ValueError, match="compact_after_no_drone_incumbent"):
        SolverConfig(compact_after_no_drone_incumbent="bad")


def test_closure_aware_state_forces_certification_by_count_and_time() -> None:
    config = SolverConfig(closure_attempt_batch_period=2, closure_attempt_time_period=10.0)
    state = _ClosureAwarePricingState()
    assert not state.should_certify(config)
    state.observe_productive_batch(3.0)
    assert not state.should_certify(config)
    state.observe_productive_batch(1.0)
    assert state.should_certify(config)
    state.reset_after_certification_attempt()
    assert not state.should_certify(config)
    state.observe_productive_batch(11.0)
    assert state.should_certify(config)
    state.reset_after_certification_attempt()
    state.force_next_certification = True
    assert state.should_certify(config)


def test_adaptive_productive_slice_controller_updates_bounds_and_stats() -> None:
    controller = _AdaptiveProductiveSliceController(
        current_seconds=20.0,
        min_seconds=5.0,
        max_seconds=30.0,
        low_threshold=0.5,
        high_threshold=2.0,
        window_size=1,
    )
    assert controller.observe(10.0, 1, 1) == "decrease"
    assert controller.current_seconds == pytest.approx(10.0)
    assert controller.observe(1.0, 5, 5) == "increase"
    assert controller.current_seconds == pytest.approx(12.5)

    stats = BPCStats()
    state = _ClosureAwarePricingState()
    state.observe_productive_batch(7.0)
    _sync_closure_aware_pricing_stats(stats, state, controller, SolverConfig())
    assert stats.productive_batches_since_last_cert == 1
    assert stats.productive_time_since_last_cert == pytest.approx(7.0)
    assert stats.adaptive_slice_decreases == 1
    assert stats.adaptive_slice_increases == 1
    assert stats.productive_yield_window_rate == pytest.approx(5.0)


def test_dual_stabilization_blend_uses_true_dual_verification_in_master() -> None:
    instance, _, objective, graph = _setup()
    current = PricingDuals(mu={c: -100.0 for c in instance.customers}, kappa=0.0, nu={})
    previous = PricingDuals(mu={c: 100.0 for c in instance.customers}, kappa=0.0, nu={})
    blended = _blend_pricing_duals(current, previous, 0.3)
    assert blended is not None
    assert blended.mu[instance.customers[0]] == pytest.approx(40.0)

    pool = SourceNeighborPricingPool(graph, objective, 2, source_neighbor_task_size=1)
    try:
        result = price_route(
            graph,
            objective,
            frozenset(instance.customers),
            BranchRestrictions(),
            current,
            next_route_id=820,
            batch_size=4,
            pricing_worker_backend="process",
            parallel_workers=2,
            pricing_process_pool=pool,
            search_duals=previous,
        )
    finally:
        pool.shutdown()
    assert result.diagnostics.stabilized_dual_enabled is True
    assert result.diagnostics.stabilized_candidates_returned >= result.diagnostics.true_dual_rejected_candidates
    assert not result.routes
    assert result.diagnostics.true_dual_rejected_candidates > 0


def test_prefix_task_depth_two_matches_source_neighbor_closure_on_tiny_instance() -> None:
    instance, _, objective, graph = _setup()
    residual = frozenset(instance.customers)
    restrictions = BranchRestrictions()
    prefixes = _source_neighbor_prefix_tasks(graph, objective, residual, restrictions, 2)
    assert prefixes
    assert all(prefix[0] in _admissible_source_neighbors(graph) for prefix in prefixes)

    duals = PricingDuals(mu={c: -10.0 for c in instance.customers}, kappa=0.0, nu={})
    source_neighbor_result = price_route(
        graph,
        objective,
        residual,
        restrictions,
        duals,
        next_route_id=830,
        batch_size=32,
        pricing_mode="closure",
        pricing_worker_backend="process",
        parallel_workers=2,
        source_neighbor_task_size=1,
        prefix_task_depth=1,
    )
    prefix_result = price_route(
        graph,
        objective,
        residual,
        restrictions,
        duals,
        next_route_id=840,
        batch_size=32,
        pricing_mode="closure",
        pricing_worker_backend="process",
        parallel_workers=2,
        source_neighbor_task_size=1,
        prefix_task_depth=2,
    )
    assert not source_neighbor_result.routes
    assert not prefix_result.routes
    assert source_neighbor_result.diagnostics.exact_completion is True
    assert prefix_result.diagnostics.exact_completion is True
    assert prefix_result.diagnostics.prefix_task_depth == 2
    assert prefix_result.diagnostics.certification_mode == source_neighbor_result.diagnostics.certification_mode


def test_production_cli_help_hides_legacy_join_and_bidirectional_controls() -> None:
    repo_root = Path(__file__).resolve().parents[1]
    for module_name in ("thvrpd.solve", "thvrpd.experiments"):
        completed = subprocess.run(
            [sys.executable, "-m", module_name, "--help"],
            cwd=repo_root,
            capture_output=True,
            text=True,
            check=True,
        )

        assert "--productive-pricing-slice-seconds" in completed.stdout
        assert "--pricing-worker-backend" in completed.stdout
        assert "--join" not in completed.stdout
        assert "--disable-bidirectional-pricing" not in completed.stdout


def test_standard_and_farkas_pricing_match_direct_route_cost() -> None:
    instance, _, objective, graph = _setup()
    duals = PricingDuals(mu={c: 10.0 for c in instance.customers}, kappa=-1.0, nu={tuple(instance.customers): -0.5})
    result = price_route(
        graph,
        objective,
        frozenset(instance.customers),
        BranchRestrictions(),
        duals,
        next_route_id=100,
        farkas=False,
    )
    assert result.route is not None
    assert abs(result.reduced_cost - route_reduced_cost(result.route, duals)) < 1e-8
    farkas = price_route(
        graph,
        objective,
        frozenset(instance.customers),
        BranchRestrictions(),
        duals,
        next_route_id=101,
        farkas=True,
    )
    assert farkas.route is not None
    assert abs(farkas.reduced_cost - route_farkas_reduced_cost(farkas.route, duals)) < 1e-8


def test_pricing_rejects_positive_inequality_dual_signs() -> None:
    instance, _, objective, graph = _setup()
    with pytest.raises(ValueError, match="nonpositive"):
        price_route(
            graph,
            objective,
            frozenset(instance.customers),
            BranchRestrictions(),
            PricingDuals(mu={c: 0.0 for c in instance.customers}, kappa=0.0, nu={tuple(instance.customers): 0.1}),
            next_route_id=150,
        )


def test_productive_batch_pricing_returns_diagnostics() -> None:
    instance, _, objective, graph = _setup()
    duals = PricingDuals(mu={c: 100.0 for c in instance.customers}, kappa=0.0, nu={})
    result = price_route(
        graph,
        objective,
        frozenset(instance.customers),
        BranchRestrictions(),
        duals,
        next_route_id=500,
        farkas=False,
        batch_size=2,
    )
    assert len(result.routes) == 1
    assert all(cost < 0.0 for cost in result.reduced_costs)
    assert result.diagnostics.returned_routes == 1
    assert result.diagnostics.labels_generated >= result.diagnostics.returned_routes
    assert result.diagnostics.max_queue_size >= 1
    assert result.diagnostics.exact_completion is False
    assert result.diagnostics.termination_reason == "productive_batch_found"
    assert result.diagnostics.certification_mode == "not_certified_productive"
    assert result.diagnostics.pricing_status == "NEGATIVE_BATCH"
    assert result.diagnostics.productive_calls == 1
    assert result.diagnostics.certification_calls == 0
    assert result.diagnostics.negative_routes_inserted == result.diagnostics.returned_routes
    assert result.diagnostics.negative_routes_verified >= result.diagnostics.returned_routes
    assert result.diagnostics.elapsed_seconds >= 0.0
    assert result.diagnostics.pricing_engine == "source_neighbor_parallel_forward"
    assert result.diagnostics.pricing_worker_backend == "thread"
    assert result.diagnostics.parallel_calls == 1
    assert result.diagnostics.first_hit_exits == 1
    assert result.diagnostics.source_neighbor_count > 0
    assert result.diagnostics.process_cpu_time_seconds >= 0.0
    assert result.diagnostics.cpu_core_equivalent >= 0.0
    assert result.diagnostics.dominance_work_estimate >= 0
    assert result.diagnostics.dominance_activation_mode in {"none", "direct_dominated", "indexed_dominated", "mixed"}
    assert result.diagnostics.side_pool_candidates_seen >= result.diagnostics.side_pool_routes_retained
    assert result.diagnostics.backward_labels_generated == 0
    assert result.diagnostics.backward_cost_function_eval_time_seconds == 0.0
    assert result.diagnostics.join_pairs_tested == 0
    assert result.diagnostics.join_exact_rc_evals == 0


def test_closure_mode_negative_batch_does_not_certify_closure() -> None:
    instance, _, objective, graph = _setup()
    duals = PricingDuals(mu={c: 100.0 for c in instance.customers}, kappa=0.0, nu={})
    result = price_route(
        graph,
        objective,
        frozenset(instance.customers),
        BranchRestrictions(),
        duals,
        next_route_id=550,
        farkas=False,
        batch_size=1,
        pricing_mode="closure",
    )
    assert result.routes
    assert result.diagnostics.exact_completion is False
    assert result.diagnostics.termination_reason == "closure_negative_batch_found"
    assert result.diagnostics.certification_mode == "not_certified_closure_returned_columns"
    assert result.diagnostics.pricing_status == "NEGATIVE_BATCH"
    assert result.diagnostics.certification_calls == 1


def test_productive_pricing_batch_cap_preserves_direct_cost_verification() -> None:
    instance, _, objective, graph = _setup()
    duals = PricingDuals(mu={c: 50.0 for c in instance.customers}, kappa=0.0, nu={})
    one = price_route(
        graph,
        objective,
        frozenset(instance.customers),
        BranchRestrictions(),
        duals,
        next_route_id=600,
        batch_size=1,
    )
    large = price_route(
        graph,
        objective,
        frozenset(instance.customers),
        BranchRestrictions(),
        duals,
        next_route_id=700,
        batch_size=64,
    )
    assert one.routes
    assert large.routes
    assert len(large.routes) >= len(one.routes)
    assert len(large.routes) <= 64
    for route, reduced_cost in zip(large.routes, large.reduced_costs):
        assert reduced_cost == pytest.approx(route_reduced_cost(route, duals))


def test_interrupted_pricing_is_not_a_certificate() -> None:
    instance, _, objective, graph = _setup()
    duals = PricingDuals(mu={c: 0.0 for c in instance.customers}, kappa=0.0, nu={})
    with pytest.raises(PricingTimeLimitReached) as raised:
        price_route(
            graph,
            objective,
            frozenset(instance.customers),
            BranchRestrictions(),
            duals,
            next_route_id=501,
            farkas=False,
            deadline=time.time() - 1.0,
        )
    assert raised.value.diagnostics.exact_completion is False
    assert raised.value.diagnostics.returned_routes == 0
    assert raised.value.diagnostics.termination_reason == "time_limit_unresolved"
    assert raised.value.diagnostics.certification_mode == "not_certified_time_limit"
    assert raised.value.diagnostics.pricing_status == "TIME_LIMIT_NO_COLUMNS"


def test_dual_reward_bound_uses_fractional_knapsack_item() -> None:
    instance, _, _, graph = _setup()
    duals = PricingDuals(mu={"C1": 200.0, "C2": 100.0, "C3": 0.0}, kappa=0.0, nu={})
    shortest = _shortest_truck_times(graph)
    bounds = _build_pricing_bounds(graph, duals, frozenset({"C1", "C2"}), shortest)
    label = _Label(
        path=(instance.depot_source,),
        represented=frozenset(),
        truck_visited=frozenset({instance.depot_source}),
        truck_load=17.5,
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
    reward = _dual_reward_bound(label, graph, bounds)
    expected = duals.mu["C1"] + duals.mu["C2"] * 0.5 / instance.demand["C2"]
    assert reward == pytest.approx(expected)
    assert duals.mu["C1"] < reward < duals.mu["C1"] + duals.mu["C2"]


def test_dual_reward_bound_excludes_customers_over_total_residual_payload() -> None:
    instance, _, _, graph = _setup()
    duals = PricingDuals(mu={"C1": 0.0, "C2": 100.0, "C3": 0.0}, kappa=0.0, nu={})
    shortest = _shortest_truck_times(graph)
    bounds = _build_pricing_bounds(graph, duals, frozenset({"C2"}), shortest)
    label = _Label(
        path=(instance.depot_source,),
        represented=frozenset(),
        truck_visited=frozenset({instance.depot_source}),
        truck_load=19.0,
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
    assert _dual_reward_bound(label, graph, bounds) == 0.0


def test_dual_reward_bound_uses_cardinality_cap() -> None:
    instance, _, _, graph = _setup()
    duals = PricingDuals(mu={"C1": 100.0, "C2": 90.0, "C3": 80.0}, kappa=0.0, nu={})
    shortest = _shortest_truck_times(graph)
    bounds = _build_pricing_bounds(graph, duals, frozenset(instance.customers), shortest)
    label = _Label(
        path=(instance.depot_source, "H1"),
        represented=frozenset(),
        truck_visited=frozenset({instance.depot_source, "H1", "C2", "C3"}),
        truck_load=0.0,
        active_pad=None,
        active_pad_arrival=0.0,
        active_wait=0.0,
        block_count=0,
        physical_time=0.0,
        service_times=tuple(),
        sr_counts=tuple(),
        reduced_cost=0.0,
        used_arcs=frozenset({(instance.depot_source, "H1")}),
    )
    assert _dual_reward_bound(label, graph, bounds) == pytest.approx(duals.mu["C1"])


def test_same_node_dominance_uses_return_time_credit() -> None:
    instance, _, objective, graph = _setup()
    duals = PricingDuals(mu={c: 0.0 for c in instance.customers}, kappa=0.0, nu={})
    credit = objective.coeffs.return_time * 5.0
    assert credit > 0.0
    earlier = _Label(
        path=("Source", "C1"),
        represented=frozenset({"C1"}),
        truck_visited=frozenset({"Source", "C1"}),
        truck_load=instance.demand["C1"],
        active_pad=None,
        active_pad_arrival=0.0,
        active_wait=0.0,
        block_count=0,
        physical_time=5.0,
        service_times=(("C1", 5.0),),
        sr_counts=tuple(),
        reduced_cost=0.5 * credit,
        used_arcs=frozenset({("Source", "C1")}),
    )
    later = _Label(
        path=("Source", "C1"),
        represented=frozenset({"C1"}),
        truck_visited=frozenset({"Source", "C1"}),
        truck_load=instance.demand["C1"],
        active_pad=None,
        active_pad_arrival=0.0,
        active_wait=0.0,
        block_count=0,
        physical_time=10.0,
        service_times=(("C1", 10.0),),
        sr_counts=tuple(),
        reduced_cost=0.0,
        used_arcs=frozenset({("Source", "C1")}),
    )
    arc_customer_sets = {arc: graph.arc_customer_set(arc) for arc in graph.arcs}
    counters = _DeadlinePricingCounters(shortest=_shortest_truck_times(graph))
    assert _paper_dominates(earlier, later, graph, objective, duals, BranchRestrictions(), arc_customer_sets, counters)
    assert counters.dom_gate_pairs_seen == 1
    assert counters.forward_same_node_dominance_tests == 1
    assert counters.forward_return_time_credit_checks == 1
    assert counters.forward_return_time_credit_checks_skipped == 0
    assert counters.dom_prefilter_pairs == 1
    assert counters.dom_full_tests == 1
    assert counters.dom_full_rejections == 1
    assert counters.labels_dominated_same_node == 1
    assert counters.labels_dominated_physical == 0


def test_dominance_gate_skip_does_not_delete_label() -> None:
    instance, _, objective, graph = _setup()
    duals = PricingDuals(mu={c: 0.0 for c in instance.customers}, kappa=0.0, nu={})
    c1 = _Label(
        path=("Source", "C1"),
        represented=frozenset({"C1"}),
        truck_visited=frozenset({"Source", "C1"}),
        truck_load=instance.demand["C1"],
        active_pad=None,
        active_pad_arrival=0.0,
        active_wait=0.0,
        block_count=0,
        physical_time=5.0,
        service_times=(("C1", 5.0),),
        sr_counts=tuple(),
        reduced_cost=0.0,
        used_arcs=frozenset({("Source", "C1")}),
    )
    c2 = _Label(
        path=("Source", "C2"),
        represented=frozenset({"C2"}),
        truck_visited=frozenset({"Source", "C2"}),
        truck_load=instance.demand["C2"],
        active_pad=None,
        active_pad_arrival=0.0,
        active_wait=0.0,
        block_count=0,
        physical_time=5.0,
        service_times=(("C2", 5.0),),
        sr_counts=tuple(),
        reduced_cost=0.0,
        used_arcs=frozenset({("Source", "C2")}),
    )
    counters = _DeadlinePricingCounters(shortest=_shortest_truck_times(graph))

    assert not _paper_dominates(c1, c2, graph, objective, duals, BranchRestrictions(), _arc_customer_sets(graph), counters)
    assert counters.dom_gate_pairs_seen == 1
    assert counters.dom_gate_scalar_failures == 1
    assert counters.forward_return_time_credit_checks_skipped == 1
    assert counters.dom_full_tests == 0
    assert counters.dom_full_rejections == 0
    assert counters.labels_dominated_same_node == 0
    assert counters.labels_dominated_physical == 0


def test_physical_location_dominance_reports_return_credit_rejection() -> None:
    instance, _, objective, graph = _setup()
    duals = PricingDuals(mu={c: 0.0 for c in instance.customers}, kappa=0.0, nu={})
    h1_c1 = duplicate_node("H1", "C1")
    h1_c2 = duplicate_node("H1", "C2")
    earlier = _Label(
        path=("Source", "H1", h1_c1),
        represented=frozenset({"C1"}),
        truck_visited=frozenset({"Source", "H1"}),
        truck_load=instance.demand["C1"],
        active_pad="H1",
        active_pad_arrival=5.0,
        active_wait=1.0,
        block_count=1,
        physical_time=5.0,
        service_times=(("C1", 6.0),),
        sr_counts=tuple(),
        reduced_cost=0.0,
        used_arcs=frozenset({("Source", "H1"), ("H1", h1_c1)}),
    )
    later = _Label(
        path=("Source", "H1", h1_c1, h1_c2),
        represented=frozenset({"C1", "C2"}),
        truck_visited=frozenset({"Source", "H1"}),
        truck_load=instance.demand["C1"] + instance.demand["C2"],
        active_pad="H1",
        active_pad_arrival=6.0,
        active_wait=2.0,
        block_count=2,
        physical_time=5.0,
        service_times=(("C1", 6.0), ("C2", 7.0)),
        sr_counts=tuple(),
        reduced_cost=0.0,
        used_arcs=frozenset({("Source", "H1"), ("H1", h1_c1), (h1_c1, h1_c2)}),
    )
    arc_customer_sets = {arc: graph.arc_customer_set(arc) for arc in graph.arcs}
    counters = _DeadlinePricingCounters(shortest=_shortest_truck_times(graph))
    assert _paper_dominates(earlier, later, graph, objective, duals, BranchRestrictions(), arc_customer_sets, counters)
    assert counters.dom_gate_pairs_seen == 1
    assert counters.forward_physical_location_dominance_tests == 1
    assert counters.forward_physical_location_dominance_rejections == 1
    assert counters.forward_return_time_credit_checks == 1
    assert counters.forward_return_time_credit_checks_skipped == 0
    assert counters.physical_location_full_tests == 1
    assert counters.physical_location_rejections == 1
    assert counters.labels_dominated_same_node == 0
    assert counters.labels_dominated_physical == 1


def test_same_regular_node_dominance_ignores_stale_active_pad_state() -> None:
    instance, _, objective, graph = _setup()
    duals = PricingDuals(mu={c: 0.0 for c in instance.customers}, kappa=0.0, nu={})
    credit = objective.coeffs.return_time * 5.0
    earlier = _Label(
        path=("Source", "C1"),
        represented=frozenset({"C1"}),
        truck_visited=frozenset({"Source", "C1"}),
        truck_load=instance.demand["C1"],
        active_pad="H1",
        active_pad_arrival=100.0,
        active_wait=5.0,
        block_count=0,
        physical_time=5.0,
        service_times=(("C1", 5.0),),
        sr_counts=tuple(),
        reduced_cost=0.5 * credit,
        used_arcs=frozenset({("Source", "C1")}),
    )
    later = _Label(
        path=("Source", "C1"),
        represented=frozenset({"C1"}),
        truck_visited=frozenset({"Source", "C1"}),
        truck_load=instance.demand["C1"],
        active_pad=None,
        active_pad_arrival=0.0,
        active_wait=0.0,
        block_count=0,
        physical_time=10.0,
        service_times=(("C1", 10.0),),
        sr_counts=tuple(),
        reduced_cost=0.0,
        used_arcs=frozenset({("Source", "C1")}),
    )
    arc_customer_sets = {arc: graph.arc_customer_set(arc) for arc in graph.arcs}
    assert _paper_dominates(earlier, later, graph, objective, duals, BranchRestrictions(), arc_customer_sets)


def test_same_regular_node_farkas_dominance_ignores_stale_active_pad_state() -> None:
    instance, _, _, graph = _setup()
    earlier = _Label(
        path=("Source", "C1"),
        represented=frozenset({"C1"}),
        truck_visited=frozenset({"Source", "C1"}),
        truck_load=instance.demand["C1"],
        active_pad="H1",
        active_pad_arrival=100.0,
        active_wait=5.0,
        block_count=0,
        physical_time=5.0,
        service_times=(("C1", 5.0),),
        sr_counts=tuple(),
        reduced_cost=0.0,
        used_arcs=frozenset({("Source", "C1")}),
    )
    later = _Label(
        path=("Source", "C1"),
        represented=frozenset({"C1"}),
        truck_visited=frozenset({"Source", "C1"}),
        truck_load=instance.demand["C1"],
        active_pad=None,
        active_pad_arrival=0.0,
        active_wait=0.0,
        block_count=0,
        physical_time=10.0,
        service_times=(("C1", 10.0),),
        sr_counts=tuple(),
        reduced_cost=0.0,
        used_arcs=frozenset({("Source", "C1")}),
    )
    arc_customer_sets = {arc: graph.arc_customer_set(arc) for arc in graph.arcs}
    assert _farkas_dominates(earlier, later, graph, BranchRestrictions(), arc_customer_sets)


def test_standard_nondominated_label_set_purges_dominated_incumbent() -> None:
    instance, _, objective, graph = _setup()
    duals = PricingDuals(mu={c: 0.0 for c in instance.customers}, kappa=0.0, nu={})
    dominated = _Label(
        path=("Source", "C1"),
        represented=frozenset({"C1"}),
        truck_visited=frozenset({"Source", "C1"}),
        truck_load=instance.demand["C1"],
        active_pad=None,
        active_pad_arrival=0.0,
        active_wait=0.0,
        block_count=0,
        physical_time=10.0,
        service_times=(("C1", 10.0),),
        sr_counts=tuple(),
        reduced_cost=1.0,
        used_arcs=frozenset({("Source", "C1")}),
    )
    stronger = _Label(
        path=("Source", "C1"),
        represented=frozenset({"C1"}),
        truck_visited=frozenset({"Source", "C1"}),
        truck_load=instance.demand["C1"],
        active_pad=None,
        active_pad_arrival=0.0,
        active_wait=0.0,
        block_count=0,
        physical_time=5.0,
        service_times=(("C1", 5.0),),
        sr_counts=tuple(),
        reduced_cost=0.0,
        used_arcs=frozenset({("Source", "C1")}),
    )
    arc_customer_sets = {arc: graph.arc_customer_set(arc) for arc in graph.arcs}
    kept: dict[str, list[_Label]] = {}
    assert _insert_nondominated_standard_label(
        kept,
        dominated,
        graph,
        objective,
        duals,
        BranchRestrictions(),
        arc_customer_sets,
    ) == (True, 0, 0)
    assert _insert_nondominated_standard_label(
        kept,
        stronger,
        graph,
        objective,
        duals,
        BranchRestrictions(),
        arc_customer_sets,
    ) == (True, 0, 1)
    assert kept["C1"] == [stronger]


def test_farkas_nondominated_label_set_purges_dominated_incumbent() -> None:
    instance, _, _, graph = _setup()
    dominated = _Label(
        path=("Source", "C1"),
        represented=frozenset({"C1"}),
        truck_visited=frozenset({"Source", "C1"}),
        truck_load=instance.demand["C1"],
        active_pad=None,
        active_pad_arrival=0.0,
        active_wait=0.0,
        block_count=0,
        physical_time=10.0,
        service_times=(("C1", 10.0),),
        sr_counts=tuple(),
        reduced_cost=1.0,
        used_arcs=frozenset({("Source", "C1")}),
    )
    stronger = _Label(
        path=("Source", "C1"),
        represented=frozenset({"C1"}),
        truck_visited=frozenset({"Source", "C1"}),
        truck_load=instance.demand["C1"],
        active_pad=None,
        active_pad_arrival=0.0,
        active_wait=0.0,
        block_count=0,
        physical_time=5.0,
        service_times=(("C1", 5.0),),
        sr_counts=tuple(),
        reduced_cost=0.0,
        used_arcs=frozenset({("Source", "C1")}),
    )
    arc_customer_sets = {arc: graph.arc_customer_set(arc) for arc in graph.arcs}
    kept: dict[str, list[_Label]] = {}
    assert _insert_nondominated_farkas_label(
        kept,
        dominated,
        graph,
        BranchRestrictions(),
        arc_customer_sets,
    ) == (True, 0, 0)
    assert _insert_nondominated_farkas_label(
        kept,
        stronger,
        graph,
        BranchRestrictions(),
        arc_customer_sets,
    ) == (True, 0, 1)
    assert kept["C1"] == [stronger]


def test_farkas_pricing_invalidates_infeasible_rmp_certificate() -> None:
    instance, _, objective, graph = _setup()
    node = NodeState(
        id=1,
        depth=0,
        restrictions=BranchRestrictions(),
        fixed_routes=tuple(),
        residual_customers=frozenset(instance.customers),
        fleet_limit=instance.num_trucks,
        fixed_cost=0.0,
        column_paths=set(),
        active_sr=set(),
    )
    result = RestrictedMaster(graph, node, {}, SolverConfig()).solve()
    assert result.status == GRB.INFEASIBLE
    assert result.farkas_rhs is not None and result.farkas_rhs > 0.0
    assert result.max_farkas_column_activity is None
    priced = price_route(
        graph,
        objective,
        node.residual_customers,
        node.restrictions,
        result.duals,
        next_route_id=200,
        farkas=True,
    )
    assert priced.route is not None
    assert priced.reduced_cost is not None and priced.reduced_cost < 0.0
    certificate_activity = (
        sum(result.duals.mu[customer] for customer in priced.route.served)
        + result.duals.kappa
        + sum(dual * priced.route.sr_coeff(triplet) for triplet, dual in result.duals.nu.items())
    )
    assert certificate_activity > 0.0
    assert abs(priced.reduced_cost - route_farkas_reduced_cost(priced.route, result.duals)) < 1e-8


def test_rmp_farkas_certificate_satisfies_paper_conditions_for_current_columns() -> None:
    instance, _, objective, graph = _setup()
    route = route_from_path(1, ("Source", "C1", "Sink"), graph, objective)
    routes = {route.path: route}
    node = NodeState(
        id=2,
        depth=0,
        restrictions=BranchRestrictions(),
        fixed_routes=tuple(),
        residual_customers=frozenset(instance.customers),
        fleet_limit=instance.num_trucks,
        fixed_cost=0.0,
        column_paths={route.path},
        active_sr=set(),
    )
    result = RestrictedMaster(graph, node, routes, SolverConfig()).solve()
    assert result.status == GRB.INFEASIBLE
    assert result.farkas_rhs is not None and result.farkas_rhs > 0.0
    assert result.max_farkas_column_activity is not None
    assert result.max_farkas_column_activity <= 1e-8
    assert _farkas_rhs(result.duals, node.residual_customers, node.fleet_limit, node.active_sr) == pytest.approx(
        result.farkas_rhs
    )
    assert _farkas_column_activity(route, result.duals) == pytest.approx(result.max_farkas_column_activity)


def test_farkas_certificate_validation_rejects_invalid_ray() -> None:
    instance, _, objective, graph = _setup()
    route = route_from_path(1, ("Source", "C1", "Sink"), graph, objective)
    duals = PricingDuals(mu={customer: 1.0 for customer in instance.customers}, kappa=0.0, nu={})
    with pytest.raises(RuntimeError, match="current-column validity"):
        _validate_farkas_certificate(
            duals,
            {route.path: route},
            {route.path},
            frozenset(instance.customers),
            instance.num_trucks,
            set(),
            1e-9,
        )
    zero_duals = PricingDuals(mu={customer: 0.0 for customer in instance.customers}, kappa=0.0, nu={})
    with pytest.raises(RuntimeError, match="residual RHS"):
        _validate_farkas_certificate(
            zero_duals,
            {},
            set(),
            frozenset(instance.customers),
            instance.num_trucks,
            set(),
            1e-9,
        )


def test_farkas_certificate_records_best_nonviolating_reduced_cost() -> None:
    instance, _, objective, graph = _setup()
    restrictions = BranchRestrictions()
    for first in range(len(instance.customers)):
        for second in range(first + 1, len(instance.customers)):
            restrictions = restrictions.with_separate(instance.customers[first], instance.customers[second])
    duals = PricingDuals(mu={customer: 0.4 for customer in instance.customers}, kappa=-1.0, nu={})
    priced = price_route(
        graph,
        objective,
        frozenset(instance.customers),
        restrictions,
        duals,
        next_route_id=250,
        farkas=True,
    )
    assert priced.routes == tuple()
    assert priced.best_route is not None
    assert priced.diagnostics.exact_completion is True
    assert priced.diagnostics.complete_routes_generated > 0
    assert priced.diagnostics.best_reduced_cost is not None
    assert priced.diagnostics.best_reduced_cost >= 0.0
    assert priced.diagnostics.standard_bound_pruned == 0
    assert priced.diagnostics.farkas_bound_pruned == 0


def test_standard_pricing_pruning_certifies_absence_of_negative_column() -> None:
    instance, _, objective, graph = _setup()
    duals = PricingDuals(mu={c: 0.0 for c in instance.customers}, kappa=0.0, nu={})
    exhaustive = price_route(
        graph,
        objective,
        frozenset(instance.customers),
        BranchRestrictions(),
        duals,
        next_route_id=100,
        farkas=False,
        use_standard_acceleration=False,
    )
    pruned = price_route(
        graph,
        objective,
        frozenset(instance.customers),
        BranchRestrictions(),
        duals,
        next_route_id=100,
        farkas=False,
        pricing_tolerance=1e-9,
        use_standard_acceleration=True,
    )
    assert exhaustive.route is not None
    assert exhaustive.reduced_cost is not None and exhaustive.reduced_cost >= 0.0
    assert pruned.routes == tuple()
    if pruned.best_reduced_cost is not None:
        assert pruned.best_reduced_cost >= -1e-9
    assert pruned.diagnostics.exact_completion is True
    assert pruned.diagnostics.pricing_status == "EXHAUSTED_NO_NEGATIVE"
    assert pruned.diagnostics.standard_bound_pruned == pruned.diagnostics.labels_pruned
    assert pruned.diagnostics.farkas_bound_pruned == 0


def test_pricing_enforces_conditional_branch_automata() -> None:
    instance, _, objective, graph = _setup()
    high_c1_duals = PricingDuals(mu={"C1": 100.0, "C2": 0.0, "C3": 0.0}, kappa=0.0, nu={})
    infeasible_together = price_route(
        graph,
        objective,
        frozenset({"C1"}),
        BranchRestrictions().with_together("C1", "C2"),
        high_c1_duals,
        next_route_id=300,
        farkas=False,
        use_standard_acceleration=False,
    )
    assert infeasible_together.route is None

    required_arc = ("Source", "C1")
    arc_up = price_route(
        graph,
        objective,
        frozenset({"C1"}),
        BranchRestrictions().with_trans_arc_required(required_arc),
        high_c1_duals,
        next_route_id=301,
        farkas=False,
        use_standard_acceleration=False,
    )
    assert arc_up.route is not None
    assert required_arc in arc_up.route.used_arcs


def test_rmp_residual_customer_model_and_sr_separation() -> None:
    instance, _, objective, graph = _setup()
    routes = {}
    for customer in instance.customers:
        path = ("Source", customer, "Sink")
        routes[path] = route_from_path(len(routes), path, graph, objective)
    combo_path = ("Source", "C1", "C2", "Sink")
    routes[combo_path] = route_from_path(len(routes), combo_path, graph, objective)
    combo_path_2 = ("Source", "C2", "C3", "Sink")
    routes[combo_path_2] = route_from_path(len(routes), combo_path_2, graph, objective)
    node = NodeState(
        id=1,
        depth=0,
        restrictions=BranchRestrictions(),
        fixed_routes=tuple(),
        residual_customers=frozenset(instance.customers),
        fleet_limit=2,
        fixed_cost=0.0,
        column_paths=set(routes),
        active_sr=set(),
    )
    rmp = RestrictedMaster(graph, node, routes, SolverConfig())
    result = rmp.solve()
    assert result.status == GRB.OPTIMAL
    cuts = rmp.violated_sr_cuts({combo_path: 0.6, combo_path_2: 0.6}, 1e-9)
    assert tuple(instance.customers) in cuts
    node_with_sr = NodeState(
        id=2,
        depth=0,
        restrictions=BranchRestrictions(),
        fixed_routes=tuple(),
        residual_customers=frozenset(instance.customers),
        fleet_limit=2,
        fixed_cost=0.0,
        column_paths=set(routes),
        active_sr={tuple(instance.customers)},
    )
    sr_result = RestrictedMaster(graph, node_with_sr, routes, SolverConfig()).solve()
    assert sr_result.status == GRB.OPTIMAL
    assert sr_result.duals.kappa <= 1e-9
    assert all(value <= 1e-9 for value in sr_result.duals.nu.values())


def test_incremental_rmp_matches_full_rebuild_after_column_append() -> None:
    instance, _, objective, graph = _setup()
    initial_paths = [("Source", "C1", "C2", "Sink"), ("Source", "C3", "Sink")]
    routes = {path: route_from_path(i, path, graph, objective) for i, path in enumerate(initial_paths)}
    combo_path = ("Source", "C2", "C3", "Sink")
    config = SolverConfig(enable_incremental_rmp=True, enable_active_coefficient_cache=True)
    node = NodeState(
        id=7,
        depth=1,
        restrictions=BranchRestrictions(),
        fixed_routes=tuple(),
        residual_customers=frozenset(instance.customers),
        fleet_limit=2,
        fixed_cost=0.0,
        column_paths=set(initial_paths),
        active_sr=set(),
    )
    cache = RouteSignatureCache()
    initial = RestrictedMaster(graph, node, routes, config, cache)
    initial_result = initial.solve()
    assert initial_result.status == GRB.OPTIMAL

    routes[combo_path] = route_from_path(len(routes), combo_path, graph, objective)
    node.column_paths.add(combo_path)
    incremental = RestrictedMaster(graph, node, routes, config, cache)
    incremental_result = incremental.solve()
    assert incremental.used_incremental_update is True
    assert incremental.used_full_rebuild is False

    full_node = NodeState(
        id=8,
        depth=1,
        restrictions=BranchRestrictions(),
        fixed_routes=tuple(),
        residual_customers=frozenset(instance.customers),
        fleet_limit=2,
        fixed_cost=0.0,
        column_paths=set(node.column_paths),
        active_sr=set(),
    )
    full_result = RestrictedMaster(
        graph,
        full_node,
        routes,
        replace(config, enable_incremental_rmp=False),
        RouteSignatureCache(),
    ).solve()
    assert full_result.status == GRB.OPTIMAL
    assert incremental_result.objective == pytest.approx(full_result.objective)
    assert incremental_result.z_values == pytest.approx(full_result.z_values)


def test_incremental_rmp_compatibility_key_rebuilds_on_branch_state_change() -> None:
    instance, _, objective, graph = _setup()
    paths = [("Source", "C1", "Sink"), ("Source", "C2", "C3", "Sink")]
    routes = {path: route_from_path(i, path, graph, objective) for i, path in enumerate(paths)}
    config = SolverConfig(enable_incremental_rmp=True)
    node = NodeState(
        id=77,
        depth=1,
        restrictions=BranchRestrictions(),
        fixed_routes=tuple(),
        residual_customers=frozenset(instance.customers),
        fleet_limit=2,
        fixed_cost=0.0,
        column_paths=set(paths),
        active_sr=set(),
    )
    initial = RestrictedMaster(graph, node, routes, config, RouteSignatureCache())
    assert initial.solve().status == GRB.OPTIMAL
    node.restrictions = node.restrictions.with_truck_service("C1")
    rebuilt = RestrictedMaster(graph, node, routes, config, RouteSignatureCache())
    assert rebuilt.used_full_rebuild is True
    assert rebuilt.used_incremental_update is False
    assert "branch_state" in rebuilt.compatibility_failure_reasons


def test_active_sr_coefficient_cache_matches_direct_recomputation_and_reports_density() -> None:
    instance, _, objective, graph = _setup()
    paths = [("Source", "C1", "C2", "Sink"), ("Source", "C2", "C3", "Sink")]
    routes = {path: route_from_path(i, path, graph, objective) for i, path in enumerate(paths)}
    triplet = tuple(instance.customers)
    node = NodeState(
        id=78,
        depth=1,
        restrictions=BranchRestrictions(),
        fixed_routes=tuple(),
        residual_customers=frozenset(instance.customers),
        fleet_limit=2,
        fixed_cost=0.0,
        column_paths=set(paths),
        active_sr={triplet},
        active_sr_version=1,
    )
    rmp = RestrictedMaster(graph, node, routes, SolverConfig(enable_active_coefficient_cache=True), RouteSignatureCache())
    direct_nonzero = sum(1 for path in paths if routes[path].sr_coeff(triplet))
    assert rmp.active_sr_nonzero_count == direct_nonzero
    assert rmp.active_sr_coefficient_count == len(paths)
    assert all(rmp._sr_coeff(path, triplet) == routes[path].sr_coeff(triplet) for path in paths)
    assert rmp.active_sr_full_rebuilds == 1
    assert rmp.active_sr_rows_added == 1
    path3 = ("Source", "C1", "C3", "Sink")
    routes[path3] = route_from_path(2, path3, graph, objective)
    node.column_paths.add(path3)
    incremental = RestrictedMaster(graph, node, routes, SolverConfig(enable_incremental_rmp=True), RouteSignatureCache())
    assert incremental.used_incremental_update is True
    assert incremental.active_sr_row_local_updates >= 1


def test_inactive_postroot_sr_cut_removal_requires_reprice_and_can_reactivate() -> None:
    instance, _, objective, graph = _setup()
    path = ("Source", "C1", "Sink")
    route = route_from_path(0, path, graph, objective)
    routes = {path: route}
    triplet = tuple(instance.customers)
    node = NodeState(
        id=2,
        depth=1,
        restrictions=BranchRestrictions(),
        fixed_routes=tuple(),
        residual_customers=frozenset(instance.customers),
        fleet_limit=instance.num_trucks,
        fixed_cost=0.0,
        column_paths={path},
        active_sr={triplet},
        sr_cut_meta={triplet: SRCutMetadata()},
    )
    rmp = RestrictedMaster(graph, node, routes, SolverConfig())
    result = RMPResult(
        status=GRB.OPTIMAL,
        objective=route.cost,
        z_values={path: 1.0},
        duals=PricingDuals(mu={customer: 0.0 for customer in instance.customers}, kappa=0.0, nu={triplet: 0.0}),
    )
    stats = BPCStats()
    assert _remove_inactive_sr_cuts_after_closure(
        rmp,
        result,
        node,
        SolverConfig(sr_removal_min_active_count=1, sr_removal_rmp_growth_threshold=0.0),
        stats,
    )
    assert triplet not in node.active_sr
    assert triplet in node.removed_sr
    assert node.pending_sr_removal_bound == pytest.approx(route.cost)
    assert node.sr_removals_performed == 1
    assert stats.sr_cuts_removed == 1
    assert stats.sr_removal_nodes == 1
    assert stats.sr_cut_repricing_after_removal == 1
    assert stats.sr_removal_candidate_marks == 1
    assert stats.sr_cut_coefficient_nonzeros_observed >= 0
    assert stats.sr_cut_coefficient_density_max >= 0.0
    assert stats.sr_cut_metadata_update_time >= 0.0
    assert node.sr_cut_meta[triplet].removal_candidate_count == 1
    assert node.sr_cut_meta[triplet].nonzero_count >= 0
    assert node.sr_cut_meta[triplet].coefficient_density >= 0.0

    new_count = _activate_sr_cuts(node, {triplet: 1.2}, stats)
    assert new_count == 0
    assert triplet in node.active_sr
    assert triplet not in node.removed_sr
    assert node.sr_cut_meta[triplet].reactivation_count == 1
    assert stats.sr_cuts_reactivated == 1


def test_postroot_sr_removal_respects_age_threshold_and_node_cap() -> None:
    instance, _, objective, graph = _setup()
    path = ("Source", "C1", "Sink")
    route = route_from_path(0, path, graph, objective)
    routes = {path: route}
    triplet = tuple(instance.customers)
    node = NodeState(
        id=2,
        depth=1,
        restrictions=BranchRestrictions(),
        fixed_routes=tuple(),
        residual_customers=frozenset(instance.customers),
        fleet_limit=instance.num_trucks,
        fixed_cost=0.0,
        column_paths={path},
        active_sr={triplet},
        sr_cut_meta={triplet: SRCutMetadata()},
    )
    rmp = RestrictedMaster(graph, node, routes, SolverConfig())
    result = RMPResult(
        status=GRB.OPTIMAL,
        objective=route.cost,
        z_values={path: 1.0},
        duals=PricingDuals(mu={customer: 0.0 for customer in instance.customers}, kappa=0.0, nu={triplet: 0.0}),
    )
    stats = BPCStats()
    assert not _remove_inactive_sr_cuts_after_closure(
        rmp,
        result,
        node,
        SolverConfig(sr_inactive_age_threshold=2, sr_removal_min_active_count=1, sr_removal_rmp_growth_threshold=0.0),
        stats,
    )
    assert triplet in node.active_sr
    assert _remove_inactive_sr_cuts_after_closure(
        rmp,
        result,
        node,
        SolverConfig(
            sr_inactive_age_threshold=2,
            sr_max_removals_per_node=1,
            sr_removal_min_active_count=1,
            sr_removal_rmp_growth_threshold=0.0,
        ),
        stats,
    )
    assert node.sr_removals_performed == 1
    node.active_sr.add(triplet)
    assert not _remove_inactive_sr_cuts_after_closure(
        rmp,
        result,
        node,
        SolverConfig(
            sr_inactive_age_threshold=1,
            sr_max_removals_per_node=1,
            sr_removal_min_active_count=1,
            sr_removal_rmp_growth_threshold=0.0,
        ),
        stats,
    )


def test_inactive_column_deactivation_requires_repricing_and_rehydrates_negative_column() -> None:
    instance, _, objective, graph = _setup()
    path = ("Source", "C1", "Sink")
    route = route_from_path(0, path, graph, objective)
    routes = {path: route}
    node = NodeState(
        id=3,
        depth=1,
        restrictions=BranchRestrictions(),
        fixed_routes=tuple(),
        residual_customers=frozenset(instance.customers),
        fleet_limit=instance.num_trucks,
        fixed_cost=0.0,
        column_paths={path},
        active_sr=set(),
        column_age={path: 3},
    )
    result = RMPResult(
        status=GRB.OPTIMAL,
        objective=route.cost,
        z_values={path: 0.0},
        duals=PricingDuals(mu={customer: 0.0 for customer in instance.customers}, kappa=0.0, nu={}),
    )
    stats = BPCStats()
    config = SolverConfig(column_deactivation_min_active_columns=1, column_inactive_age_min=3)
    assert _deactivate_inactive_node_columns(node, result, config, stats)
    assert path not in node.column_paths
    assert path in node.inactive_column_paths
    assert node.pending_column_deactivation_bound == pytest.approx(route.cost)
    assert stats.inactive_columns_deactivated == 1

    duals = PricingDuals(
        mu={customer: route.cost + 1.0 if customer == "C1" else 0.0 for customer in instance.customers},
        kappa=0.0,
        nu={},
    )
    assert _rehydrate_negative_inactive_columns(graph, objective, node, routes, duals, config, stats, RouteSignatureCache())
    assert path in node.column_paths
    assert path not in node.inactive_column_paths
    assert stats.inactive_columns_rehydrated == 1
    assert stats.inactive_column_reduced_cost_checks == 1


def test_light_logging_skips_minor_progress_but_preserves_major_snapshot(tmp_path: Path) -> None:
    node = NodeState(
        id=1,
        depth=0,
        restrictions=BranchRestrictions(),
        fixed_routes=tuple(),
        residual_customers=frozenset({"C1"}),
        fleet_limit=1,
        fixed_cost=0.0,
        column_paths=set(),
        active_sr=set(),
    )
    stats = BPCStats()
    config = SolverConfig(logging_mode="light", gurobi_log_dir=str(tmp_path / "gurobi_logs"))
    _write_progress(config, stats, node, "rmp_solved", {"rmp_status": GRB.OPTIMAL})
    assert stats.progress_events_seen == 1
    assert stats.progress_events_skipped == 1
    assert not (tmp_path / "bpc_progress.json").exists()
    _write_progress(config, stats, node, "node_closed", {"bound": 0.0})
    assert stats.progress_events_written == 1
    assert (tmp_path / "bpc_progress.json").exists()


def test_child_node_inheritance_filters_by_child_admissibility_and_refreshes_signatures() -> None:
    instance, _, objective, graph = _setup()
    truck_path = ("Source", "C1", "Sink")
    drone_path = ("Source", "H1", duplicate_node("H1", "C1"), "Sink")
    c2_path = ("Source", "C2", "Sink")
    routes = {
        truck_path: route_from_path(0, truck_path, graph, objective),
        drone_path: route_from_path(1, drone_path, graph, objective),
        c2_path: route_from_path(2, c2_path, graph, objective),
    }
    parent = NodeState(
        id=1,
        depth=0,
        restrictions=BranchRestrictions(),
        fixed_routes=tuple(),
        residual_customers=frozenset({"C1", "C2"}),
        fleet_limit=instance.num_trucks,
        fixed_cost=0.0,
        column_paths={truck_path, drone_path},
        active_sr=set(),
    )
    child = parent.copy_for_child(2, parent.restrictions.with_truck_service("C1"))
    stats = BPCStats()
    cache = RouteSignatureCache()
    _inherit_child_routes(graph, objective, parent, child, routes, {c2_path}, stats, cache)
    assert child.column_paths == {truck_path, c2_path}
    assert stats.child_branch_index_candidates_before == 3
    assert stats.child_branch_index_candidates_after == 2
    assert stats.child_branch_index_reject_service_mode == 1
    assert stats.child_inherited_route_candidates == 2
    assert stats.child_inherited_route_accepted == 2
    assert stats.child_inherited_route_rejected == 0
    assert stats.child_reject_branch == 0
    assert stats.child_refresh_count == 2
    assert stats.child_hydration_time >= 0.0
    assert child.column_index.matches(child.residual_customers, child.active_sr, child.active_sr_version, child.column_paths)


def test_child_certification_signature_reuses_only_unchanged_node_and_dual_state() -> None:
    instance, _, _, _ = _setup()
    node = NodeState(
        id=9,
        depth=1,
        restrictions=BranchRestrictions().with_truck_service("C1"),
        fixed_routes=tuple(),
        residual_customers=frozenset(instance.customers),
        fleet_limit=instance.num_trucks,
        fixed_cost=0.0,
        column_paths={("Source", "C1", "Sink")},
        active_sr={tuple(instance.customers)},
    )
    duals = PricingDuals(
        mu={customer: float(i + 1) for i, customer in enumerate(instance.customers)},
        kappa=-0.25,
        nu={tuple(instance.customers): -0.5},
    )
    signature = _child_certification_signature(node, duals)
    assert _child_certification_signature(node, duals) == signature
    assert _child_certification_signature(node, replace(duals, kappa=-0.30)) != signature

    node.active_sr_version += 1
    assert _child_certification_signature(node, duals) != signature


def test_child_certification_signature_changes_when_visible_route_set_changes() -> None:
    instance, _, _, _ = _setup()
    node = NodeState(
        id=10,
        depth=1,
        restrictions=BranchRestrictions(),
        fixed_routes=tuple(),
        residual_customers=frozenset(instance.customers),
        fleet_limit=instance.num_trucks,
        fixed_cost=0.0,
        column_paths={("Source", "C1", "Sink")},
        active_sr=set(),
    )
    duals = PricingDuals(mu={customer: 0.0 for customer in instance.customers}, kappa=0.0)
    signature = _child_certification_signature(node, duals)
    node.column_paths.add(("Source", "C2", "Sink"))
    assert _child_certification_signature(node, duals) != signature


def test_child_certification_epoch_discard_records_structured_cause() -> None:
    instance, _, objective, graph = _setup()
    fixed_c2 = route_from_path(20, ("Source", "C2", "Sink"), graph, objective)
    fixed_c3 = route_from_path(21, ("Source", "C3", "Sink"), graph, objective)
    node = NodeState(
        id=13,
        depth=1,
        restrictions=BranchRestrictions().with_truck_service("C1"),
        fixed_routes=(fixed_c2,),
        residual_customers=frozenset({"C1", "C3"}),
        fleet_limit=instance.num_trucks - 1,
        fixed_cost=0.1,
        column_paths={("Source", "C1", "Sink")},
        active_sr={tuple(instance.customers)},
        active_sr_version=1,
    )
    duals = PricingDuals(
        mu={customer: float(index + 1) for index, customer in enumerate(instance.customers)},
        kappa=-0.25,
        nu={tuple(instance.customers): -0.5},
    )
    base = _child_certification_signature(node, duals)

    stats = BPCStats()
    _record_child_certification_epoch_discard(stats, base, _child_certification_signature(node, replace(duals, kappa=-0.3)))
    assert stats.child_certification_state_discarded_by_dual == 1

    stats = BPCStats()
    node.active_sr_version += 1
    _record_child_certification_epoch_discard(stats, base, _child_certification_signature(node, duals))
    assert stats.child_certification_state_discarded_by_sr == 1
    node.active_sr_version -= 1

    stats = BPCStats()
    node.residual_customers = frozenset({"C1"})
    _record_child_certification_epoch_discard(stats, base, _child_certification_signature(node, duals))
    assert stats.child_certification_state_discarded_by_residual == 1
    node.residual_customers = frozenset({"C1", "C3"})

    stats = BPCStats()
    node.restrictions = node.restrictions.with_drone_service("C3")
    _record_child_certification_epoch_discard(stats, base, _child_certification_signature(node, duals))
    assert stats.child_certification_state_discarded_by_branch == 1
    node.restrictions = BranchRestrictions().with_truck_service("C1")

    stats = BPCStats()
    node.fixed_routes = node.fixed_routes + (fixed_c3,)
    _record_child_certification_epoch_discard(stats, base, _child_certification_signature(node, duals))
    assert stats.child_certification_state_discarded_by_fixed_routes == 1
    node.fixed_routes = (fixed_c2,)

    stats = BPCStats()
    node.column_paths.add(("Source", "C3", "Sink"))
    _record_child_certification_epoch_discard(stats, base, _child_certification_signature(node, duals))
    assert stats.child_certification_state_discarded_by_active_columns == 1


def test_child_certification_yield_tracking_does_not_adapt_batch_limit() -> None:
    instance, _, _, _ = _setup()
    node = NodeState(
        id=11,
        depth=1,
        restrictions=BranchRestrictions(),
        fixed_routes=tuple(),
        residual_customers=frozenset(instance.customers),
        fleet_limit=instance.num_trucks,
        fixed_cost=0.0,
        column_paths=set(),
        active_sr=set(),
    )
    config = SolverConfig(
        child_closure_batch_min=16,
        child_closure_batch_initial=32,
        child_closure_batch_max=128,
    )
    stats = BPCStats()
    _ensure_child_closure_batch_state(node, config, stats)
    assert node.child_closure_batch_limit == 32
    _observe_child_certification_yield(node, 32, config, stats, no_route=False)
    _observe_child_certification_yield(node, 32, config, stats, no_route=False)
    assert node.child_closure_batch_limit == 32
    assert stats.child_closure_batch_increases == 0
    _observe_child_certification_yield(node, 0, config, stats, no_route=True)
    _observe_child_certification_yield(node, 0, config, stats, no_route=True)
    assert node.child_closure_batch_limit == 32
    assert stats.child_closure_batch_decreases == 0
    assert stats.child_certification_yield_observations == 4
    assert node.residual_customers == frozenset(instance.customers)
    assert node.fixed_routes == tuple()


def test_branch_route_index_keeps_every_bruteforce_branch_admissible_route() -> None:
    _, _, objective, graph = _setup()
    paths = {
        ("Source", "C1", "Sink"),
        ("Source", "H1", duplicate_node("H1", "C1"), "Sink"),
        ("Source", "C2", "Sink"),
        ("Source", "C1", "C2", "Sink"),
    }
    routes = {path: route_from_path(i, path, graph, objective) for i, path in enumerate(sorted(paths))}
    restrictions = BranchRestrictions().with_truck_service("C1").with_separate("C1", "C2")
    arc_customer_sets = _arc_customer_sets(graph)
    index = build_branch_route_index(paths, routes, graph, frozenset({"C1", "C2"}), RouteSignatureCache())
    indexed, counts = query_branch_route_index(index, restrictions, arc_customer_sets)
    brute_force = {
        path
        for path, route in routes.items()
        if restrictions.route_allowed(route, arc_customer_sets)
    }
    assert brute_force.issubset(indexed)
    assert indexed == brute_force
    assert counts["service_mode"] == 1
    assert counts["separate"] == 1


def test_extended_branch_route_index_matches_full_rebuild_query() -> None:
    _, _, objective, graph = _setup()
    initial_paths = {
        ("Source", "C1", "Sink"),
        ("Source", "H1", duplicate_node("H1", "C1"), "Sink"),
    }
    new_paths = {
        ("Source", "C2", "Sink"),
        ("Source", "C1", "C2", "Sink"),
    }
    all_paths = initial_paths | new_paths
    routes = {path: route_from_path(i, path, graph, objective) for i, path in enumerate(sorted(all_paths))}
    residual = frozenset({"C1", "C2"})
    cache = RouteSignatureCache()
    full = build_branch_route_index(all_paths, routes, graph, residual, cache)
    incremental = extend_branch_route_index(
        build_branch_route_index(initial_paths, routes, graph, residual, RouteSignatureCache()),
        new_paths,
        routes,
        graph,
        residual,
        RouteSignatureCache(),
    )
    restrictions = BranchRestrictions().with_drone_service("C1").with_separate("C1", "C2")
    arc_customer_sets = _arc_customer_sets(graph)
    full_candidates, full_counts = query_branch_route_index(full, restrictions, arc_customer_sets)
    incremental_candidates, incremental_counts = query_branch_route_index(incremental, restrictions, arc_customer_sets)
    assert incremental.all_paths == full.all_paths
    assert incremental_candidates == full_candidates
    assert incremental_counts == full_counts


def test_branch_decision_reports_customer_pair_rule() -> None:
    instance, _, objective, graph = _setup()
    paths = [
        ("Source", "C1", "C2", "Sink"),
        ("Source", "C1", "Sink"),
        ("Source", "C2", "Sink"),
    ]
    routes = {path: route_from_path(i, path, graph, objective) for i, path in enumerate(paths)}
    node = NodeState(
        id=1,
        depth=0,
        restrictions=BranchRestrictions(),
        fixed_routes=tuple(),
        residual_customers=frozenset({"C1", "C2"}),
        fleet_limit=instance.num_trucks,
        fixed_cost=0.0,
        column_paths=set(routes),
        active_sr=set(),
    )
    decision = _branch(
        node,
        {paths[0]: 0.5, paths[1]: 0.5, paths[2]: 0.5},
        routes,
        graph,
        2,
        3,
        SolverConfig(),
    )
    assert decision.branch_type == "customer_pair"
    assert ("C1", "C2") in decision.left.restrictions.together_pairs
    assert ("C1", "C2") in decision.right.restrictions.separate_pairs


def test_branch_decision_reports_service_mode_rule() -> None:
    instance, _, objective, graph = _setup()
    drone_path = ("Source", "H1", duplicate_node("H1", "C1"), "Sink")
    truck_path = ("Source", "C1", "Sink")
    routes = {
        truck_path: route_from_path(0, truck_path, graph, objective),
        drone_path: route_from_path(1, drone_path, graph, objective),
    }
    node = NodeState(
        id=1,
        depth=0,
        restrictions=BranchRestrictions(),
        fixed_routes=tuple(),
        residual_customers=frozenset({"C1"}),
        fleet_limit=instance.num_trucks,
        fixed_cost=0.0,
        column_paths=set(routes),
        active_sr=set(),
    )
    decision = _branch(node, {truck_path: 0.5, drone_path: 0.5}, routes, graph, 2, 3, SolverConfig())
    assert decision.branch_type == "service_mode"
    assert "C1" in decision.left.restrictions.truck_service
    assert "C1" in decision.right.restrictions.drone_service


def test_branch_decision_reports_launch_pad_rule() -> None:
    instance = generate_instance(
        InstanceConfig(
            seed=1,
            num_trucks=2,
            num_customers=3,
            num_hubs=2,
            drones_per_truck=4,
            distribution="PS",
            hub_arc_probability=1.0,
        )
    )
    objective = build_objective_data(instance, ObjectiveWeights(0.4, 0.3, 0.3))
    graph = build_transformed_graph(instance)
    customer = "C2"
    h1_path = ("Source", "H1", duplicate_node("H1", customer), "Sink")
    h2_path = ("Source", "H2", duplicate_node("H2", customer), "Sink")
    routes = {
        h1_path: route_from_path(0, h1_path, graph, objective),
        h2_path: route_from_path(1, h2_path, graph, objective),
    }
    node = NodeState(
        id=1,
        depth=0,
        restrictions=BranchRestrictions(),
        fixed_routes=tuple(),
        residual_customers=frozenset({customer}),
        fleet_limit=instance.num_trucks,
        fixed_cost=0.0,
        column_paths=set(routes),
        active_sr=set(),
    )
    decision = _branch(node, {h1_path: 0.5, h2_path: 0.5}, routes, graph, 2, 3, SolverConfig())
    assert decision.branch_type == "launch_pad"
    assert ("H1", customer) in decision.left.restrictions.pad_forbidden
    assert ("H1", customer) in decision.right.restrictions.pad_required


def test_branch_decision_reports_transformed_arc_rule() -> None:
    instance, _, objective, graph = _setup()
    c1_c2 = ("Source", "C1", "C2", "Sink")
    c2_c1 = ("Source", "C2", "C1", "Sink")
    routes = {
        c1_c2: route_from_path(0, c1_c2, graph, objective),
        c2_c1: route_from_path(1, c2_c1, graph, objective),
    }
    node = NodeState(
        id=1,
        depth=0,
        restrictions=BranchRestrictions(),
        fixed_routes=tuple(),
        residual_customers=frozenset({"C1", "C2"}),
        fleet_limit=instance.num_trucks,
        fixed_cost=0.0,
        column_paths=set(routes),
        active_sr=set(),
    )
    decision = _branch(node, {c1_c2: 0.5, c2_c1: 0.5}, routes, graph, 2, 3, SolverConfig())
    assert decision.branch_type == "transformed_arc"
    assert decision.left.restrictions.trans_arc_forbidden
    assert decision.right.restrictions.trans_arc_required


def test_branch_decision_reports_route_variable_fallback() -> None:
    instance, _, objective, graph = _setup()
    path = ("Source", "C1", "Sink")
    alias = ("alias",)
    route = route_from_path(0, path, graph, objective)
    routes = {path: route, alias: route}
    node = NodeState(
        id=1,
        depth=0,
        restrictions=BranchRestrictions(),
        fixed_routes=tuple(),
        residual_customers=frozenset({"C1"}),
        fleet_limit=instance.num_trucks,
        fixed_cost=0.0,
        column_paths=set(routes),
        active_sr=set(),
    )
    decision = _branch(node, {path: 0.5, alias: 0.5}, routes, graph, 2, 3, SolverConfig())
    assert decision.branch_type == "route_variable"
    assert path in decision.left.restrictions.route_forbidden
    assert decision.right.fixed_routes == (route,)
    assert decision.right.residual_customers == frozenset()
    assert decision.right.fleet_limit == instance.num_trucks - 1


def test_route_variable_up_branch_drops_sr_cuts_involving_fixed_route_customers() -> None:
    instance, _, objective, graph = _setup()
    path = ("Source", "C1", "Sink")
    alias = ("alias",)
    route = route_from_path(0, path, graph, objective)
    routes = {path: route, alias: route}
    triplet = tuple(instance.customers)
    node = NodeState(
        id=1,
        depth=0,
        restrictions=BranchRestrictions(),
        fixed_routes=tuple(),
        residual_customers=frozenset(instance.customers),
        fleet_limit=instance.num_trucks,
        fixed_cost=0.0,
        column_paths=set(routes),
        active_sr={triplet},
        sr_cut_meta={triplet: SRCutMetadata(age=2, last_activity=1.0)},
    )
    decision = _branch(node, {path: 0.5, alias: 0.5}, routes, graph, 2, 3, SolverConfig())
    assert decision.branch_type == "route_variable"
    assert decision.right.fixed_routes == (route,)
    assert decision.right.residual_customers == frozenset({"C2", "C3"})
    assert decision.right.active_sr == set()
    assert decision.right.sr_cut_meta == {}
    assert decision.right.column_paths == set()


def test_branch_decision_counters_are_recorded_by_type() -> None:
    stats = BPCStats()
    for branch_type in (
        "customer_pair",
        "service_mode",
        "launch_pad",
        "transformed_arc",
        "route_variable",
    ):
        _record_branch_decision(stats, branch_type)
    assert stats.customer_pair_branches == 1
    assert stats.service_mode_branches == 1
    assert stats.launch_pad_branches == 1
    assert stats.transformed_arc_branches == 1
    assert stats.route_variable_branches == 1


def test_phase_i_seeding_identifies_residual_coverage_slack() -> None:
    instance, weights, objective, graph = _setup()
    routes = {}
    for customer in instance.customers:
        path = ("Source", customer, "Sink")
        routes[path] = route_from_path(len(routes), path, graph, objective)
    node = NodeState(
        id=1,
        depth=0,
        restrictions=BranchRestrictions(),
        fixed_routes=tuple(),
        residual_customers=frozenset(instance.customers),
        fleet_limit=2,
        fixed_cost=0.0,
        column_paths=set(routes),
        active_sr=set(),
    )
    phase_i = PhaseISeeder(graph, node, routes, SolverConfig()).solve()
    assert phase_i.objective > 0.0
    assert phase_i.uncovered

    bpc = solve_branch_price_cut(
        instance,
        weights,
        SolverConfig(root_extraction_time_limit=0.0, root_constructive_time_limit=0.0),
    )
    assert bpc.stats.phase_i_solves > 0
    assert bpc.stats.phase_i_columns_added > 0


def test_bpc_batch_size_one_and_large_batch_same_tiny_objective() -> None:
    instance, weights, _, _ = _setup()
    one = solve_branch_price_cut(
        instance,
        weights,
        SolverConfig(root_extraction_time_limit=0.0, pricing_batch_size=1, route_pool_time_limit=0.1),
    )
    large = solve_branch_price_cut(
        instance,
        weights,
        SolverConfig(root_extraction_time_limit=0.0, pricing_batch_size=64, route_pool_time_limit=0.1),
    )

    assert one.objective_full == pytest.approx(large.objective_full)
    assert one.gap_full == pytest.approx(0.0)
    assert large.gap_full == pytest.approx(0.0)


def test_bpc_process_backend_uses_persistent_pool() -> None:
    instance, weights, _, _ = _setup()
    result = solve_branch_price_cut(
        instance,
        weights,
        SolverConfig(
            root_extraction_time_limit=0.0,
            pricing_batch_size=8,
            pricing_parallel_workers=2,
            pricing_worker_backend="process",
            source_neighbor_task_size=1,
            productive_candidate_multiplier=1.0,
            pricing_diversity_batch_fraction=0.5,
            route_pool_time_limit=0.1,
            time_limit=60.0,
        ),
    )
    assert result.gap_full == pytest.approx(0.0)
    assert result.stats.pricing_pool_startup_count == 1
    assert result.stats.pricing_pool_reused_calls == result.stats.pricing_worker_backend_process_calls
    assert result.stats.pricing_pool_shutdown_time >= 0.0
    assert result.stats.pricing_first_hit_enabled_calls == 0
    assert result.stats.pricing_returned_batch_size_max > 1
    assert result.stats.pricing_decoded_routes_in_main >= result.stats.pricing_negative_routes_inserted
    assert result.stats.pricing_verified_routes_in_main >= result.stats.pricing_negative_routes_inserted
    assert result.stats.pricing_source_neighbor_task_count_max >= result.stats.pricing_parallel_workers_max
    assert result.stats.pricing_local_worker_candidate_quota_max >= 1
    assert result.stats.pricing_diversity_quota_max > 0
    assert result.stats.pricing_diversity_selected_routes >= result.stats.pricing_negative_routes_inserted
    assert result.stats.pricing_backward_labels_generated == 0
    assert result.stats.pricing_join_pairs_tested == 0


def test_duplicate_signature_merging_keeps_one_observable_column() -> None:
    instance, _, objective, graph = _setup()
    path = ("Source", "C1", "Sink")
    route = route_from_path(0, path, graph, objective)
    routes = {path: route, ("alias",): route}
    merged = merge_duplicate_column_paths(set(routes), routes, graph, frozenset(instance.customers))
    assert len(merged) == 1


def test_cost_dominated_duplicate_merge_keeps_lower_cost_observable_column() -> None:
    instance, _, objective, graph = _setup()
    path = ("Source", "C1", "Sink")
    base_route = route_from_path(0, path, graph, objective)
    high_path = ("high",)
    low_path = ("low",)
    high_route = replace(base_route, id=1, path=high_path, cost=base_route.cost + 1.0)
    low_route = replace(base_route, id=2, path=low_path, cost=base_route.cost - 1.0)
    routes = {high_path: high_route, low_path: low_route}

    merged = merge_duplicate_column_paths(set(routes), routes, graph, frozenset(instance.customers))

    assert merged == {low_path}


def test_route_signature_cache_keeps_different_launch_pads_distinct() -> None:
    instance, objective, graph = _two_hub_instance()
    h1_path = ("Source", "H1", duplicate_node("H1", "C1"), "Sink")
    h2_path = ("Source", "H2", duplicate_node("H2", "C1"), "Sink")
    h1_route = route_from_path(0, h1_path, graph, objective)
    h2_route = route_from_path(1, h2_path, graph, objective)
    cache = RouteSignatureCache()

    h1_signature = route_signature(h1_route, graph, frozenset(instance.customers), cache)
    h2_signature = route_signature(h2_route, graph, frozenset(instance.customers), cache)
    repeated_h1 = route_signature(h1_route, graph, frozenset(instance.customers), cache)

    assert h1_signature != h2_signature
    assert repeated_h1 == h1_signature
    assert cache.stats.signature_cache_hits >= 1
    merged = merge_duplicate_column_paths(
        {h1_path, h2_path},
        {h1_path: h1_route, h2_path: h2_route},
        graph,
        frozenset(instance.customers),
        cache,
    )
    assert merged == {h1_path, h2_path}


def test_sr_coeff_from_served_mask_matches_route_coefficient() -> None:
    instance, _, objective, graph = _setup()
    route = route_from_path(0, ("Source", "C1", "C2", "Sink"), graph, objective)
    residual_key = tuple(sorted(instance.customers))
    triplet = tuple(sorted(instance.customers))
    cache = RouteSignatureCache()

    served_mask = customer_mask(route.served, residual_key, cache)
    mask_coeff = sr_coeff_from_mask(served_mask, triplet_mask(residual_key, triplet, cache))

    assert mask_coeff == route.sr_coeff(triplet)
    assert cache.stats.triplet_masks_built == 1


def test_active_signature_refines_only_active_sr_coefficients() -> None:
    instance, _, objective, graph = _setup()
    route = route_from_path(0, ("Source", "C1", "C2", "Sink"), graph, objective)
    residual = frozenset(instance.customers)
    triplet = tuple(sorted(instance.customers))
    cache = RouteSignatureCache()

    inactive = route_signature(route, graph, residual, cache, active_sr=tuple(), active_sr_version=0)
    active = route_signature(route, graph, residual, cache, active_sr=(triplet,), active_sr_version=1)

    assert inactive.core.served_mask == active.core.served_mask
    assert inactive.core.pad_assignment_key == active.core.pad_assignment_key
    assert inactive.active_sr_coeff_key == tuple()
    assert active.active_sr_coeff_key == (route.sr_coeff(triplet),)
    assert cache.stats.active_sr_coeffs_computed == 1


def test_bucketed_join_generator_matches_exhaustive_prefilter() -> None:
    instance, objective, graph = _two_hub_instance()
    residual = frozenset(instance.customers)
    restrictions = BranchRestrictions()
    arc_customer_sets = _arc_customer_sets(graph)
    duals = PricingDuals(mu={"C1": 0.0}, kappa=0.0, nu={})
    source = _pricing_source_label(instance, objective, duals)
    forward_labels = [
        _extend(source, "H1", graph, objective, duals, tuple(), False),
        _extend(source, "H2", graph, objective, duals, tuple(), False),
    ]
    backward_labels = [
        _build_backward_label(("H1", instance.depot_sink), graph, residual, tuple(), restrictions, arc_customer_sets),
        _build_backward_label(("H2", instance.depot_sink), graph, residual, tuple(), restrictions, arc_customer_sets),
        _build_backward_label(("H1", "H2", instance.depot_sink), graph, residual, tuple(), restrictions, arc_customer_sets),
    ]

    exhaustive = {
        (forward.path, backward.path)
        for forward in forward_labels
        for backward in backward_labels
        if _join_prefilter(forward, backward, graph)
    }
    bucketed = _bucketed_join_pairs(forward_labels, backward_labels, graph, small_join_pair_threshold=1)
    generated = {(forward.path, backward.path) for forward, backward in bucketed["pairs"]}

    assert generated == exhaustive
    assert bucketed["compatible_key_lookups"] > 0
    assert bucketed["key_cache_misses"] > 0


def test_lazy_join_generators_match_bucketed_pairs_and_have_valid_bounds() -> None:
    instance, objective, graph = _two_hub_instance()
    residual = frozenset(instance.customers)
    restrictions = BranchRestrictions()
    arc_customer_sets = _arc_customer_sets(graph)
    duals = PricingDuals(mu={"C1": 3.0}, kappa=0.0, nu={})
    source = _pricing_source_label(instance, objective, duals)
    forward_labels = [
        _extend(source, "H1", graph, objective, duals, tuple(), False),
        _extend(source, "H2", graph, objective, duals, tuple(), False),
    ]
    backward_labels = [
        _build_backward_label(("H1", instance.depot_sink), graph, residual, tuple(), restrictions, arc_customer_sets),
        _build_backward_label(("H2", instance.depot_sink), graph, residual, tuple(), restrictions, arc_customer_sets),
        _build_backward_label(("H1", "H2", instance.depot_sink), graph, residual, tuple(), restrictions, arc_customer_sets),
    ]

    pair_result = _bucketed_join_pairs(forward_labels, backward_labels, graph, small_join_pair_threshold=1)
    generator_result = _bucketed_join_generators(
        forward_labels,
        backward_labels,
        graph,
        duals,
        farkas=False,
        small_join_pair_threshold=1,
        pricing_tolerance=1e-9,
    )
    generated_pairs = set()
    for generator in generator_result["generators"]:
        for forward, backward in _iter_join_generator_pairs(generator, graph):
            if _join_prefilter(forward, backward, graph):
                generated_pairs.add((forward.path, backward.path))
                assert generator.lower_bound <= _joined_reduced_cost(forward, backward, graph, objective, duals, False) + 1e-9

    assert generated_pairs == {(forward.path, backward.path) for forward, backward in pair_result["pairs"]}
    assert generator_result["indexed_activation_count"] == 1
    assert generator_result["bucket_lower_envelope_rejects"] >= 0


def test_join_group_lower_bound_is_disabled_for_farkas() -> None:
    instance, objective, graph = _two_hub_instance()
    residual = frozenset(instance.customers)
    restrictions = BranchRestrictions()
    arc_customer_sets = _arc_customer_sets(graph)
    duals = PricingDuals(mu={"C1": 3.0}, kappa=0.0, nu={})
    source = _pricing_source_label(instance, objective, duals)
    forward = (_extend(source, "H1", graph, objective, duals, tuple(), False),)
    backward = (
        _build_backward_label(("H1", instance.depot_sink), graph, residual, tuple(), restrictions, arc_customer_sets),
    )

    assert _join_group_lower_bound(forward, backward, duals, farkas=False) <= _joined_reduced_cost(
        forward[0],
        backward[0],
        graph,
        objective,
        duals,
        False,
    )
    assert _join_group_lower_bound(forward, backward, duals, farkas=True) == float("-inf")


def test_small_join_bypass_matches_indexed_join_pairs() -> None:
    instance, objective, graph = _two_hub_instance()
    residual = frozenset(instance.customers)
    restrictions = BranchRestrictions()
    arc_customer_sets = _arc_customer_sets(graph)
    duals = PricingDuals(mu={"C1": 0.0}, kappa=0.0, nu={})
    source = _pricing_source_label(instance, objective, duals)
    forward_labels = [
        _extend(source, "H1", graph, objective, duals, tuple(), False),
        _extend(source, "H2", graph, objective, duals, tuple(), False),
    ]
    backward_labels = [
        _build_backward_label(("H1", instance.depot_sink), graph, residual, tuple(), restrictions, arc_customer_sets),
        _build_backward_label(("H2", instance.depot_sink), graph, residual, tuple(), restrictions, arc_customer_sets),
        _build_backward_label(("H1", "H2", instance.depot_sink), graph, residual, tuple(), restrictions, arc_customer_sets),
    ]

    direct = _bucketed_join_pairs(forward_labels, backward_labels, graph)
    indexed = _bucketed_join_pairs(forward_labels, backward_labels, graph, small_join_pair_threshold=1)

    direct_pairs = {(forward.path, backward.path) for forward, backward in direct["pairs"]}
    indexed_pairs = {(forward.path, backward.path) for forward, backward in indexed["pairs"]}

    assert direct_pairs == indexed_pairs
    assert direct["small_bypass_calls"] == 1
    assert indexed["small_bypass_calls"] == 0


def test_workload_adaptive_join_activates_indexed_mode_on_cumulative_work() -> None:
    instance, objective, graph = _two_hub_instance()
    residual = frozenset(instance.customers)
    restrictions = BranchRestrictions()
    arc_customer_sets = _arc_customer_sets(graph)
    duals = PricingDuals(mu={"C1": 0.0}, kappa=0.0, nu={})
    source = _pricing_source_label(instance, objective, duals)
    forward_labels = [
        _extend(source, "H1", graph, objective, duals, tuple(), False),
        _extend(source, "H2", graph, objective, duals, tuple(), False),
    ]
    backward_labels = [
        _build_backward_label(("H1", instance.depot_sink), graph, residual, tuple(), restrictions, arc_customer_sets),
        _build_backward_label(("H2", instance.depot_sink), graph, residual, tuple(), restrictions, arc_customer_sets),
        _build_backward_label(("H1", "H2", instance.depot_sink), graph, residual, tuple(), restrictions, arc_customer_sets),
    ]

    direct = _bucketed_join_pairs(
        forward_labels,
        backward_labels,
        graph,
        small_join_pair_threshold=5_000,
        small_join_cumulative_threshold=250_000,
        max_join_bypass_calls=1_000,
    )
    indexed = _bucketed_join_pairs(
        forward_labels,
        backward_labels,
        graph,
        small_join_pair_threshold=5_000,
        small_join_cumulative_threshold=5,
        max_join_bypass_calls=1_000,
        cumulative_pair_count=6,
    )
    exhausted = _bucketed_join_pairs(
        forward_labels,
        backward_labels,
        graph,
        small_join_pair_threshold=5_000,
        small_join_cumulative_threshold=250_000,
        max_join_bypass_calls=1_000,
        previous_join_bypass_calls=1_000,
    )

    direct_pairs = {(forward.path, backward.path) for forward, backward in direct["pairs"]}
    indexed_pairs = {(forward.path, backward.path) for forward, backward in indexed["pairs"]}
    exhausted_pairs = {(forward.path, backward.path) for forward, backward in exhausted["pairs"]}

    assert direct_pairs == indexed_pairs == exhausted_pairs
    assert direct["local_bypass_calls"] == 1
    assert direct["cumulative_bypass_calls"] == 1
    assert indexed["indexed_activation_count"] == 1
    assert indexed["join_work_estimate"] == 6
    assert exhausted["indexed_activation_count"] == 1


def test_cached_join_key_graph_reuses_compatible_keys() -> None:
    instance, objective, graph = _two_hub_instance()
    residual = frozenset(instance.customers)
    restrictions = BranchRestrictions()
    arc_customer_sets = _arc_customer_sets(graph)
    duals = PricingDuals(mu={"C1": 0.0}, kappa=0.0, nu={})
    source = _pricing_source_label(instance, objective, duals)
    forward_labels = [_extend(source, "H1", graph, objective, duals, tuple(), False)]
    backward_labels = [
        _build_backward_label(("H1", instance.depot_sink), graph, residual, tuple(), restrictions, arc_customer_sets),
        _build_backward_label(("H1", "H2", instance.depot_sink), graph, residual, tuple(), restrictions, arc_customer_sets),
    ]
    cache = {}

    first = _bucketed_join_pairs(
        forward_labels,
        backward_labels,
        graph,
        compatible_key_cache=cache,
        small_join_pair_threshold=1,
    )
    second = _bucketed_join_pairs(
        forward_labels,
        backward_labels,
        graph,
        compatible_key_cache=cache,
        small_join_pair_threshold=1,
    )

    assert {(f.path, b.path) for f, b in first["pairs"]} == {(f.path, b.path) for f, b in second["pairs"]}
    assert first["key_cache_misses"] > 0
    assert second["key_cache_hits"] > 0


def test_direct_dominance_keys_cover_compatible_bucket_pairs() -> None:
    instance, objective, graph = _two_hub_instance()
    residual = frozenset(instance.customers)
    restrictions = BranchRestrictions()
    arc_customer_sets = _arc_customer_sets(graph)
    labels = [
        _build_backward_label(("H1", instance.depot_sink), graph, residual, tuple(), restrictions, arc_customer_sets),
        _build_backward_label((duplicate_node("H1", "C1"), instance.depot_sink), graph, residual, tuple(), restrictions, arc_customer_sets),
        _build_backward_label(("H1", "H2", instance.depot_sink), graph, residual, tuple(), restrictions, arc_customer_sets),
    ]

    for label in labels:
        label_key = _dominance_bucket_key(label)
        incumbent_keys = set(_compatible_dominance_keys(label_key, graph, True))
        purged_keys = set(_compatible_dominance_keys(label_key, graph, False))
        for incumbent in labels:
            incumbent_key = _dominance_bucket_key(incumbent)
            if _dominance_buckets_compatible(incumbent_key, label_key, True):
                assert incumbent_key in incumbent_keys
            if _dominance_buckets_compatible(incumbent_key, label_key, False):
                assert incumbent_key in purged_keys


def test_cached_dominance_key_lookup_matches_uncached_candidates() -> None:
    instance, objective, graph = _two_hub_instance()
    residual = frozenset(instance.customers)
    restrictions = BranchRestrictions()
    arc_customer_sets = _arc_customer_sets(graph)
    labels = [
        _build_backward_label(("H1", instance.depot_sink), graph, residual, tuple(), restrictions, arc_customer_sets),
        _build_backward_label((duplicate_node("H1", "C1"), instance.depot_sink), graph, residual, tuple(), restrictions, arc_customer_sets),
        _build_backward_label(("H1", "H2", instance.depot_sink), graph, residual, tuple(), restrictions, arc_customer_sets),
    ]
    dominance_index = {}
    for label in labels:
        dominance_index.setdefault(_dominance_bucket_key(label), []).append(label)
    target = labels[-1]
    uncached_counter = _DominanceCounter()
    cached_counter = _DominanceCounter()
    cache = {}

    uncached = _dominance_candidate_labels(
        target,
        dominance_index,
        graph,
        uncached_counter,
        incumbent_may_dominate_label=True,
        dominance_key_cache=None,
        small_dom_bucket_threshold=0,
    )
    cached_first = _dominance_candidate_labels(
        target,
        dominance_index,
        graph,
        cached_counter,
        incumbent_may_dominate_label=True,
        dominance_key_cache=cache,
        small_dom_bucket_threshold=0,
    )
    cached_second = _dominance_candidate_labels(
        target,
        dominance_index,
        graph,
        cached_counter,
        incumbent_may_dominate_label=True,
        dominance_key_cache=cache,
        small_dom_bucket_threshold=0,
    )

    assert {label.path for label in uncached} == {label.path for label in cached_first}
    assert {label.path for label in cached_first} == {label.path for label in cached_second}
    assert cached_counter.key_cache_misses == 1
    assert cached_counter.key_cache_hits == 1


def test_workload_adaptive_dominance_activates_indexed_mode_on_cumulative_work() -> None:
    instance, objective, graph = _two_hub_instance()
    residual = frozenset(instance.customers)
    restrictions = BranchRestrictions()
    arc_customer_sets = _arc_customer_sets(graph)
    labels = [
        _build_backward_label(("H1", instance.depot_sink), graph, residual, tuple(), restrictions, arc_customer_sets),
        _build_backward_label((duplicate_node("H1", "C1"), instance.depot_sink), graph, residual, tuple(), restrictions, arc_customer_sets),
        _build_backward_label(("H1", "H2", instance.depot_sink), graph, residual, tuple(), restrictions, arc_customer_sets),
    ]
    dominance_index = {}
    for label in labels:
        dominance_index.setdefault(_dominance_bucket_key(label), []).append(label)
    target = labels[-1]
    direct_counter = _DominanceCounter()
    indexed_counter = _DominanceCounter()
    exhausted_counter = _DominanceCounter(small_bypass_calls=2_000)

    direct = _dominance_candidate_labels(
        target,
        dominance_index,
        graph,
        direct_counter,
        incumbent_may_dominate_label=True,
        small_dom_bucket_threshold=100,
        small_dom_cumulative_threshold=500_000,
        max_dom_bypass_calls=2_000,
    )
    indexed = _dominance_candidate_labels(
        target,
        dominance_index,
        graph,
        indexed_counter,
        incumbent_may_dominate_label=True,
        small_dom_bucket_threshold=100,
        small_dom_cumulative_threshold=1,
        max_dom_bypass_calls=2_000,
    )
    exhausted = _dominance_candidate_labels(
        target,
        dominance_index,
        graph,
        exhausted_counter,
        incumbent_may_dominate_label=True,
        small_dom_bucket_threshold=100,
        small_dom_cumulative_threshold=500_000,
        max_dom_bypass_calls=2_000,
    )

    assert {label.path for label in direct} == {label.path for label in indexed}
    assert {label.path for label in direct} == {label.path for label in exhausted}
    assert direct_counter.small_bypass_calls == 1
    assert indexed_counter.indexed_activation_count == 1
    assert indexed_counter.work_estimate > 1
    assert exhausted_counter.indexed_activation_count == 1


def test_cost_dominated_insert_replaces_higher_cost_observable_column() -> None:
    instance, _, objective, graph = _setup()
    path = ("Source", "C1", "Sink")
    base_route = route_from_path(0, path, graph, objective)
    high_path = ("high",)
    low_path = ("low",)
    high_route = replace(base_route, id=1, path=high_path, cost=base_route.cost + 1.0)
    low_route = replace(base_route, id=2, path=low_path, cost=base_route.cost - 1.0)
    routes = {high_path: high_route}
    paths = {high_path}

    inserted_path, added = insert_node_column(
        low_route,
        routes,
        paths,
        graph,
        frozenset(instance.customers),
    )

    assert inserted_path == low_path
    assert added is True
    assert paths == {low_path}


def test_node_column_index_replaces_same_coeff_higher_cost_route() -> None:
    instance, _, objective, graph = _setup()
    path = ("Source", "C1", "Sink")
    base_route = route_from_path(0, path, graph, objective)
    high_path = ("high",)
    low_path = ("low",)
    high_route = replace(base_route, id=1, path=high_path, cost=base_route.cost + 1.0)
    low_route = replace(base_route, id=2, path=low_path, cost=base_route.cost - 1.0)
    routes = {high_path: high_route}
    paths = {high_path}
    residual = frozenset(instance.customers)
    cache = RouteSignatureCache()
    index = NodeColumnIndex()
    refresh_node_column_index(index, routes, paths, graph, residual, cache)

    inserted_path, added = insert_node_column(
        low_route,
        routes,
        paths,
        graph,
        residual,
        cache=cache,
        column_index=index,
    )
    signature = route_signature(low_route, graph, residual, cache)
    coeff_signature = route_coefficient_signature(signature)

    assert inserted_path == low_path
    assert added is True
    assert paths == {low_path}
    assert index.by_coeff[coeff_signature] == low_path
    assert cache.stats.column_index_hits == 1
    assert cache.stats.column_index_replacements == 1


def test_node_column_index_refresh_tracks_active_sr_version() -> None:
    instance, _, objective, graph = _setup()
    route = route_from_path(0, ("Source", "C1", "C2", "Sink"), graph, objective)
    routes = {route.path: route}
    paths = {route.path}
    residual = frozenset(instance.customers)
    triplet = tuple(sorted(instance.customers))
    cache = RouteSignatureCache()
    index = NodeColumnIndex()

    refresh_node_column_index(index, routes, paths, graph, residual, cache, active_sr=tuple(), active_sr_version=0)
    inactive_signature = next(iter(index.by_coeff))
    refresh_node_column_index(index, routes, paths, graph, residual, cache, active_sr=(triplet,), active_sr_version=1)
    active_signature = next(iter(index.by_coeff))

    assert inactive_signature.active_sr_coeff_key == tuple()
    assert active_signature.active_sr_coeff_key == (route.sr_coeff(triplet),)
    assert index.active_sr_version == 1


def test_side_pool_retains_verified_routes_without_rmp_insertion() -> None:
    instance, _, objective, graph = _setup()
    route = route_from_path(0, ("Source", "C1", "Sink"), graph, objective)
    routes = {}
    side_pool_paths = set()
    node = NodeState(
        id=0,
        depth=0,
        restrictions=BranchRestrictions(),
        fixed_routes=tuple(),
        residual_customers=frozenset(instance.customers),
        fleet_limit=instance.num_trucks,
        fixed_cost=0.0,
        column_paths=set(),
        active_sr=set(),
    )
    stats = BPCStats()

    _add_side_pool_routes(
        (route,),
        routes,
        side_pool_paths,
        node,
        graph,
        stats,
        SolverConfig(),
        {},
    )

    assert route.path in side_pool_paths
    assert route.path not in node.column_paths
    assert stats.side_pool_routes_added == 1
    assert stats.side_pool_routes == 1


def test_side_pool_pruning_preserves_per_customer_diversity_cap() -> None:
    instance, _, objective, graph = _setup()
    base_route = route_from_path(0, ("Source", "C1", "Sink"), graph, objective)
    routes = {
        ("side", str(index)): replace(base_route, id=index, path=("side", str(index)), cost=base_route.cost + index)
        for index in range(5)
    }
    side_pool_paths = set(routes)
    node = NodeState(
        id=0,
        depth=0,
        restrictions=BranchRestrictions(),
        fixed_routes=tuple(),
        residual_customers=frozenset(instance.customers),
        fleet_limit=instance.num_trucks,
        fixed_cost=0.0,
        column_paths=set(),
        active_sr=set(),
    )
    stats = BPCStats()

    _prune_side_pool(
        routes,
        side_pool_paths,
        node,
        SolverConfig(side_pool_max_size=20_000, side_pool_per_customer_keep=2),
        {},
        stats,
    )

    assert len(side_pool_paths) == 2
    assert stats.side_pool_routes_pruned == 3
    assert stats.side_pool_per_customer_keep == 2
    assert all(routes[path].served == frozenset({"C1"}) for path in side_pool_paths)


def test_exact_duplicate_and_cost_dominated_column_counters() -> None:
    instance, _, objective, graph = _setup()
    path = ("Source", "C1", "Sink")
    base_route = route_from_path(0, path, graph, objective)
    duplicate_route = replace(base_route, id=1, path=("duplicate",))
    worse_route = replace(base_route, id=2, path=("worse",), cost=base_route.cost + 1.0)
    routes = {path: base_route}
    paths = {path}
    cache = RouteSignatureCache()

    _, duplicate_added = insert_node_column(duplicate_route, routes, paths, graph, frozenset(instance.customers), cache=cache)
    _, worse_added = insert_node_column(worse_route, routes, paths, graph, frozenset(instance.customers), cache=cache)

    assert duplicate_added is False
    assert worse_added is False
    assert cache.stats.duplicate_equivalent_rejected == 1
    assert cache.stats.cost_dominated_rejected == 1


def test_branch_aware_duplicate_merge_keeps_allowed_equivalent_column() -> None:
    instance, _, objective, graph = _setup()
    forbidden_path = ("Source", "C1", "Sink")
    allowed_path = ("alias",)
    route = route_from_path(0, forbidden_path, graph, objective)
    routes = {
        forbidden_path: route,
        allowed_path: replace(route, id=1, path=allowed_path),
    }
    node = NodeState(
        id=1,
        depth=0,
        restrictions=BranchRestrictions().with_route_forbidden(forbidden_path),
        fixed_routes=tuple(),
        residual_customers=frozenset(instance.customers),
        fleet_limit=instance.num_trucks,
        fixed_cost=0.0,
        column_paths={forbidden_path, allowed_path},
        active_sr=set(),
    )
    merged = _merge_duplicate_column_paths_for_node(node, routes, graph)
    assert forbidden_path in merged
    assert allowed_path in merged


def test_branch_aware_insert_ignores_forbidden_duplicate_signature() -> None:
    instance, _, objective, graph = _setup()
    forbidden_path = ("Source", "C1", "Sink")
    allowed_path = ("alias",)
    route = route_from_path(0, forbidden_path, graph, objective)
    routes = {forbidden_path: route}
    node = NodeState(
        id=1,
        depth=0,
        restrictions=BranchRestrictions().with_route_forbidden(forbidden_path),
        fixed_routes=tuple(),
        residual_customers=frozenset(instance.customers),
        fleet_limit=instance.num_trucks,
        fixed_cost=0.0,
        column_paths={forbidden_path},
        active_sr=set(),
    )
    inserted_path, added = _insert_node_column(replace(route, id=1, path=allowed_path), routes, node, graph)
    assert inserted_path == allowed_path
    assert added is True
    assert allowed_path in node.column_paths


def test_compact_root_extraction_returns_canonical_route_columns() -> None:
    instance, weights, objective, graph = _setup()
    solution = solve_compact_solution(instance, weights, time_limit=1800.0, require_optimal=True)
    assert solution.route_paths
    assert solution.objective_bound_full is not None
    assert solution.mip_gap is not None
    assert solution.status_code == GRB.OPTIMAL
    assert solution.node_count is not None
    assert solution.timing.model_build_time > 0.0
    assert solution.timing.solve_time >= 0.0
    assert solution.timing.route_decode_time >= 0.0
    routes = tuple(route_from_path(i, path, graph, objective) for i, path in enumerate(solution.route_paths))
    covered = frozenset().union(*(route.served for route in routes))
    assert covered == frozenset(instance.customers)
    for route in routes:
        physical_internal = route.truck_path[1:-1]
        assert len(physical_internal) == len(set(physical_internal))


def test_route_pool_diving_returns_hard_feasible_incumbent() -> None:
    instance, _, objective, graph = _setup()
    routes = {}
    for customer in instance.customers:
        path = ("Source", customer, "Sink")
        routes[path] = route_from_path(len(routes), path, graph, objective)
    combo_path = ("Source", "C1", "C2", "Sink")
    routes[combo_path] = route_from_path(len(routes), combo_path, graph, objective)
    node = NodeState(
        id=1,
        depth=0,
        restrictions=BranchRestrictions(),
        fixed_routes=tuple(),
        residual_customers=frozenset(instance.customers),
        fleet_limit=instance.num_trucks,
        fixed_cost=0.0,
        column_paths=set(routes),
        active_sr=set(),
    )
    result = run_route_pool_heuristic(
        graph,
        objective,
        node,
        routes,
        set(routes),
        {path: 1.0 / len(routes) for path in routes},
        SolverConfig(),
        len(routes),
        float("inf"),
    )
    assert result.value is not None
    covered = frozenset().union(*(route.served for route in result.selected_routes))
    assert covered == frozenset(instance.customers)
    assert result.diagnostics.hard_pool_solves == 1
    assert result.diagnostics.hard_pool_feasible_solves == 1
    assert result.diagnostics.support_pool_calls == 1
    assert result.diagnostics.support_pool_feasible == 1
    assert result.diagnostics.support_pool_incumbent_updates == 1
    assert result.diagnostics.full_pool_calls == 0
    assert result.diagnostics.soft_pool_solves == 0
    assert result.diagnostics.support_routes > 0


def test_full_node_admissible_hard_pool_solve_is_tracked_as_primal_only() -> None:
    instance, _, objective, graph = _setup()
    routes = {}
    for customer in instance.customers:
        path = ("Source", customer, "Sink")
        routes[path] = route_from_path(len(routes), path, graph, objective)
    combo_path = ("Source", "C1", "C2", "C3", "Sink")
    routes[combo_path] = route_from_path(len(routes), combo_path, graph, objective)
    node = NodeState(
        id=12,
        depth=0,
        restrictions=BranchRestrictions(),
        fixed_routes=tuple(),
        residual_customers=frozenset(instance.customers),
        fleet_limit=instance.num_trucks,
        fixed_cost=0.0,
        column_paths=set(routes),
        active_sr=set(),
    )

    result = run_route_pool_heuristic(
        graph,
        objective,
        node,
        routes,
        set(routes),
        {},
        SolverConfig(support_best_per_customer=1),
        len(routes),
        0.0,
    )

    assert result.value is not None
    assert all(route.served.issubset(node.residual_customers) for route in result.selected_routes)
    assert result.value == pytest.approx(sum(route.cost for route in result.selected_routes))
    assert result.diagnostics.support_pool_calls == 1
    assert result.diagnostics.full_pool_calls == 1
    assert result.diagnostics.support_pool_feasible == 1
    assert result.diagnostics.full_pool_feasible == 1
    assert result.diagnostics.support_pool_incumbent_updates == 0
    assert result.diagnostics.full_pool_incumbent_updates == 0
    assert result.diagnostics.hard_pool_solves == 2
    assert result.diagnostics.max_node_pool_routes == len(routes)
    assert result.diagnostics.node_pool_to_support_ratio > 1.0


def test_repair_pricing_adds_multiple_generated_routes_before_hard_resolve() -> None:
    instance, _, objective, graph = _setup()
    routes = {}
    node = NodeState(
        id=1,
        depth=0,
        restrictions=BranchRestrictions(),
        fixed_routes=tuple(),
        residual_customers=frozenset(instance.customers),
        fleet_limit=instance.num_trucks,
        fixed_cost=0.0,
        column_paths=set(),
        active_sr=set(),
    )
    result = run_route_pool_heuristic(
        graph,
        objective,
        node,
        routes,
        set(),
        {},
        SolverConfig(repair_reward=10.0),
        0,
        float("inf"),
    )
    assert len(result.generated_paths) > 1
    assert result.value is not None
    assert result.pricing_diagnostics
    assert {diagnostic["mode"] for diagnostic in result.pricing_diagnostics} == {"repair"}
    assert sum(diagnostic["elapsed_seconds"] for diagnostic in result.pricing_diagnostics) >= 0.0
    assert result.diagnostics.hard_pool_solves == 2
    assert result.diagnostics.support_pool_calls == 2
    assert result.diagnostics.full_pool_calls == 0
    assert result.diagnostics.soft_pool_solves == 1
    assert result.diagnostics.soft_pool_feasible_solves == 1
    assert result.diagnostics.repair_customers > 0
    assert result.diagnostics.repair_columns_generated == len(result.generated_paths)


def test_repair_budget_skip_preserves_heuristic_lower_bound_neutrality() -> None:
    instance, _, objective, graph = _setup()
    routes = {}
    node = NodeState(
        id=1,
        depth=0,
        restrictions=BranchRestrictions(),
        fixed_routes=tuple(),
        residual_customers=frozenset(instance.customers),
        fleet_limit=instance.num_trucks,
        fixed_cost=0.0,
        column_paths=set(),
        active_sr=set(),
    )

    result = run_route_pool_heuristic(
        graph,
        objective,
        node,
        routes,
        set(),
        {},
        SolverConfig(repair_reward=10.0),
        0,
        float("inf"),
        allow_repair=False,
        repair_time_budget=0.0,
    )

    assert result.value is None
    assert not result.generated_paths
    assert not result.pricing_diagnostics
    assert result.diagnostics.soft_pool_feasible_solves == 1
    assert result.diagnostics.repair_customers > 0
    assert result.diagnostics.repair_budget_hit == 1
    assert result.diagnostics.repair_columns_generated == 0


def test_bpc_matches_compact_miqp_on_tiny_instance(tmp_path) -> None:
    instance = tiny_instance()
    weights = ObjectiveWeights(0.4, 0.3, 0.3)
    solver_config = SolverConfig(time_limit=1800.0)
    bpc = solve_branch_price_cut(instance, weights, solver_config)
    compact = solve_compact_miqp(instance, weights, time_limit=1800.0)
    assert abs(bpc.objective_full - compact) < 1e-5
    assert bpc.gap == 0.0
    assert bpc.gap_full == 0.0
    assert bpc.gap_shifted == 0.0
    assert bpc.lower_bound_shifted == bpc.upper_bound_shifted
    assert bpc.lower_bound_full == bpc.upper_bound_full
    record = bpc.to_record()
    assert record["gap"] == record["gap_full"]
    assert record["lower_bound_full"] == bpc.lower_bound_full
    assert record["upper_bound_full"] == bpc.upper_bound_full
    assert record["gap_shifted"] == bpc.gap_shifted
    assert set(record["customer_service_times"]) == set(instance.customers)
    assert len(record["return_times"]) == len(bpc.routes)
    assert "normalization_bounds" in record
    components = record["objective_components"]
    weighted = components["weighted_normalized"]
    assert abs(sum(weighted.values()) - bpc.objective_full) < 1e-8
    assert components["raw"]["delay_square_sum"] >= 0.0
    assert components["raw"]["return_time_sum"] == sum(record["return_times"])
    assert components["raw"]["operating_cost"] >= instance.truck_cost * len(bpc.routes)
    assert "delay_square_sum" in record["routes"][0]
    assert "operating_cost" in record["routes"][0]
    assert "shifted_cost" in record["routes"][0]
    assert "full_single_route_value" not in record["routes"][0]
    assert record["bpc_stats"]["root_model_build_time"] >= 0.0
    assert record["bpc_stats"]["root_model_solve_time"] >= 0.0
    assert record["bpc_stats"]["root_route_decode_time"] >= 0.0
    assert "heuristic_hard_pool_solves" in record["bpc_stats"]
    assert "heuristic_soft_pool_solves" in record["bpc_stats"]
    assert "customer_pair_branches" in record["bpc_stats"]
    assert "service_mode_branches" in record["bpc_stats"]
    assert "launch_pad_branches" in record["bpc_stats"]
    assert "transformed_arc_branches" in record["bpc_stats"]
    assert "route_variable_branches" in record["bpc_stats"]
    assert "pricing_standard_bound_pruned" in record["bpc_stats"]
    assert "pricing_farkas_bound_pruned" in record["bpc_stats"]
    solve_record = _build_solve_record(instance.config, solver_config, instance, bpc, tmp_path)
    assert solve_record["total_drones"] == instance.num_trucks * instance.drones_per_truck
    assert solve_record["service_metrics"]["service_feasible"] is True
    assert solve_record["service_metrics"]["payload_feasible"] is True
    assert solve_record["service_metrics"]["total_customer_demand"] == sum(
        instance.demand[customer] for customer in instance.customers
    )
    assert solve_record["service_metrics"]["available_truck_payload_total"] == instance.num_trucks * instance.truck_payload
    assert solve_record["service_metrics"]["available_drones_total"] == solve_record["total_drones"]
    assert "gurobi_log_files" in solve_record


def test_solve_timeout_record_preserves_no_incumbent_diagnostics(tmp_path) -> None:
    instance, _, objective, _ = _setup()
    stats = BPCStats(
        status="time_limit",
        open_nodes_at_termination=1,
        pricing_labels_generated=2,
        standard_pricing_calls=0,
    )
    exc = BPCTimeLimitNoIncumbent(
        runtime=1.25,
        nodes_processed=1,
        lower_bound_shifted=0.5,
        stats=stats,
        objective=objective,
    )
    (tmp_path / "bpc_progress.json").write_text(
        json.dumps({"event": "time_limit_pricing_unresolved", "stats": {"open_nodes_at_termination": 1}}),
        encoding="utf-8",
    )
    (tmp_path / "pricing_diagnostics.jsonl").write_text(
        json.dumps(
            {
                "mode": "standard_interrupted",
                "labels_generated": 7,
                "labels_dominated": 3,
                "backward_dominance_tests": 5,
                "backward_labels_dominated": 1,
                "labels_purged": 2,
                "stale_labels_skipped": 1,
                "labels_pruned": 2,
                "standard_bound_pruned": 2,
                "farkas_bound_pruned": 0,
                "complete_routes_generated": 1,
                "max_queue_size": 4,
                "elapsed_seconds": 0.75,
                "exact_completion": False,
                "termination_reason": "time_limit_unresolved",
                "certification_mode": "not_certified_time_limit",
            }
        )
        + "\n",
        encoding="utf-8",
    )
    solver_config = SolverConfig(time_limit=0.01, gurobi_log_dir=str(tmp_path / "gurobi_logs"))
    record = _build_solve_timeout_record(
        instance.config,
        solver_config,
        instance,
        exc,
        tmp_path,
        runtime_seconds=1.3,
    )
    assert record["status"] == "time_limited_internal"
    assert record["solver_status"] == "time_limited_internal"
    assert record["total_drones"] == instance.num_trucks * instance.drones_per_truck
    assert record["runtime"] == 1.25
    assert record["runtime_seconds"] == 1.3
    assert record["nodes_processed"] == 1
    assert record["lower_bound_shifted"] == 0.5
    assert record["lower_bound_full"] == objective.full_value_from_route_sum(0.5)
    assert record["bpc_progress"]["event"] == "time_limit_pricing_unresolved"
    assert record["pricing_diagnostics_summary"]["count"] == 1
    assert record["pricing_diagnostics_summary"]["last_pricing_diagnostic"]["exact_completion"] is False
    assert record["pricing_diagnostics_summary"]["last_pricing_diagnostic"]["termination_reason"] == "time_limit_unresolved"
    assert record["bpc_stats"]["pricing_labels_generated"] == 7
    assert record["bpc_stats"]["pricing_backward_dominance_tests"] == 5
    assert record["bpc_stats"]["pricing_backward_labels_dominated"] == 1
    assert record["bpc_stats"]["pricing_labels_purged"] == 2
    assert record["bpc_stats"]["pricing_stale_labels_skipped"] == 1
    assert record["bpc_stats"]["pricing_diagnostics_count"] == 1
    assert record["bpc_stats"]["standard_pricing_calls"] == 1
    assert record["error"] == "BPC time limit reached before a feasible incumbent was found"
