from __future__ import annotations

import json
import tomllib
from dataclasses import replace
from math import ceil, isinf
from pathlib import Path

import pytest

from thvrpd.config import (
    GENERIC_CASE_DEFAULTS,
    MEDIUM_CASE_DEFAULTS,
    InstanceConfig,
    ObjectiveWeights,
    case_defaults,
)
from thvrpd.experiments import DEFAULT_SEEDS, SCALES, _merge_timeout_stats, _read_pricing_diagnostics, _summary_row
from thvrpd.instance import generate_instance, tiny_instance
from thvrpd.objective import build_objective_data
from thvrpd.routes import ServiceEnvelopeViolation, route_from_path
from thvrpd.service_windows import load_manual_service_deadline_bounds
from thvrpd.transform import build_transformed_graph, duplicate_node


def test_pyproject_declares_required_dependencies() -> None:
    pyproject = tomllib.loads(Path("pyproject.toml").read_text(encoding="utf-8"))
    dependencies = set(pyproject["project"]["dependencies"])
    assert {
        "gurobipy",
        "numpy",
        "networkx",
        "matplotlib",
        "scikit-learn",
        "sortedcontainers",
        "colorama",
        "pytest",
    }.issubset(dependencies)


def test_tiny_instance_objective_bounds_are_finite() -> None:
    instance = tiny_instance()
    objective = build_objective_data(instance, ObjectiveWeights(0.4, 0.3, 0.3))
    assert set(objective.bounds.arrival_lb) == set(instance.customers)
    assert objective.bounds.delay_ub > objective.bounds.delay_lb
    assert objective.bounds.return_ub > objective.bounds.return_lb
    assert objective.bounds.cost_ub > objective.bounds.cost_lb
    assert objective.coeffs.shift < 0


def test_nonbinding_service_deadlines_leave_service_bounds_unchanged() -> None:
    instance = tiny_instance()
    objective = build_objective_data(instance, ObjectiveWeights(0.4, 0.3, 0.3))
    assert objective.bounds.service_deadline_mode == "none"
    assert objective.bounds.service_deadline_active_count == 0
    assert all(isinf(value) for value in objective.bounds.service_deadline.values())
    assert objective.bounds.service_ub == objective.bounds.data_service_ub


def test_manual_service_deadline_file_bounds_build_effective_bounds() -> None:
    base = tiny_instance()
    baseline = build_objective_data(base, ObjectiveWeights(0.4, 0.3, 0.3))
    manual_bounds = {
        customer: baseline.bounds.arrival_lb[customer] + 0.60 * (baseline.bounds.data_service_ub[customer] - baseline.bounds.arrival_lb[customer])
        for customer in base.customers
    }
    instance = replace(base, config=replace(base.config, service_deadline_mode="manual", service_deadline_manual_bounds=manual_bounds))
    objective = build_objective_data(instance, ObjectiveWeights(0.4, 0.3, 0.3))
    assert objective.bounds.service_deadline_mode == "manual"
    assert objective.bounds.service_deadline_active_count == len(instance.customers)
    for customer in instance.customers:
        assert objective.bounds.service_deadline[customer] == pytest.approx(manual_bounds[customer])
        assert objective.bounds.service_deadline_req_ub[customer] == pytest.approx(manual_bounds[customer])
        assert objective.bounds.service_ub[customer] == pytest.approx(manual_bounds[customer])
        assert objective.bounds.service_ub[customer] >= objective.bounds.arrival_lb[customer]


def test_manual_service_deadline_mode_requires_explicit_bounds() -> None:
    base = tiny_instance()
    with pytest.raises(ValueError, match="manual service deadline mode requires"):
        replace(base.config, service_deadline_mode="manual")


def test_manual_service_deadline_file_loader_json_and_csv(tmp_path: Path) -> None:
    json_path = tmp_path / "deadlines.json"
    json_path.write_text(json.dumps({"deadlines": {"C1": 10.0, "C2": 20.5}}), encoding="utf-8")
    assert load_manual_service_deadline_bounds(json_path) == {"C1": 10.0, "C2": 20.5}
    csv_path = tmp_path / "deadlines.csv"
    csv_path.write_text("customer,deadline\nC1,11\nC2,22.25\n", encoding="utf-8")
    assert load_manual_service_deadline_bounds(csv_path) == {"C1": 11.0, "C2": 22.25}


def test_manual_service_deadline_loader_accepts_bpc_result_record(tmp_path: Path) -> None:
    result_path = tmp_path / "result.json"
    result_path.write_text(
        json.dumps({"normalization_bounds": {"service_deadline": {"C1": 12.0, "C2": 23.5}}}),
        encoding="utf-8",
    )
    assert load_manual_service_deadline_bounds(result_path) == {"C1": 12.0, "C2": 23.5}


def test_random_absolute_service_deadlines_are_seeded_and_customer_specific() -> None:
    base = tiny_instance()
    config = replace(
        base.config,
        service_deadline_mode="random_absolute",
        service_deadline_offset_min=1.0,
        service_deadline_offset_max=5.0,
        service_deadline_witness_slack=5.0,
        service_deadline_random_seed=17,
    )
    objective = build_objective_data(replace(base, config=config), ObjectiveWeights(0.4, 0.3, 0.3))
    repeated = build_objective_data(replace(base, config=config), ObjectiveWeights(0.4, 0.3, 0.3))
    assert objective.bounds.service_deadline_mode == "random_absolute"
    assert objective.bounds.service_deadline_random_seed == 17
    assert objective.bounds.service_deadline_offset == repeated.bounds.service_deadline_offset
    assert len(set(objective.bounds.service_deadline_offset.values())) == len(base.customers)
    for customer in base.customers:
        offset = objective.bounds.service_deadline_offset[customer]
        candidate = objective.bounds.service_deadline_candidate[customer]
        witness = objective.bounds.service_deadline_witness_service_time[customer]
        deadline = objective.bounds.service_deadline[customer]
        assert 1.0 <= offset <= 5.0
        assert candidate == pytest.approx(objective.bounds.arrival_lb[customer] + offset)
        assert deadline >= witness + config.service_deadline_witness_slack - 1e-9
        assert objective.bounds.service_ub[customer] == pytest.approx(min(objective.bounds.data_service_ub[customer], deadline))


def test_random_absolute_witness_floor_lifts_too_tight_offsets() -> None:
    base = tiny_instance()
    config = replace(
        base.config,
        service_deadline_mode="random_absolute",
        service_deadline_offset_min=0.0,
        service_deadline_offset_max=0.0,
        service_deadline_witness_slack=5.0,
        service_deadline_random_seed=23,
    )
    objective = build_objective_data(replace(base, config=config), ObjectiveWeights(0.4, 0.3, 0.3))
    assert objective.bounds.service_deadline_witness_status == "success"
    assert objective.bounds.service_deadline_witness_time >= 0.0
    for customer in base.customers:
        witness = objective.bounds.service_deadline_witness_service_time[customer]
        deadline = objective.bounds.service_deadline[customer]
        assert deadline == pytest.approx(witness + config.service_deadline_witness_slack)
        assert objective.bounds.service_deadline_witness_lift[customer] == pytest.approx(
            deadline - objective.bounds.service_deadline_candidate[customer]
        )
        assert objective.bounds.service_ub[customer] >= witness


def test_random_absolute_witness_methods_report_constructive_and_compact_timing() -> None:
    base = tiny_instance()
    weights = ObjectiveWeights(0.4, 0.3, 0.3)
    constructive_config = replace(
        base.config,
        service_deadline_mode="random_absolute",
        service_deadline_offset_min=45.0,
        service_deadline_offset_max=120.0,
        service_deadline_witness_method="constructive",
        service_deadline_witness_time_limit=5.0,
    )
    constructive = build_objective_data(replace(base, config=constructive_config), weights)
    assert constructive.bounds.service_deadline_witness_status == "success"
    assert constructive.bounds.service_deadline_witness_method == "constructive"
    assert constructive.bounds.service_deadline_witness_constructive_time >= 0.0
    assert constructive.bounds.service_deadline_witness_compact_time == pytest.approx(0.0)
    assert constructive.bounds.service_deadline_witness_failed_customers == tuple()

    compact_config = replace(constructive_config, service_deadline_witness_method="compact")
    compact = build_objective_data(replace(base, config=compact_config), weights)
    assert compact.bounds.service_deadline_witness_status == "success"
    assert compact.bounds.service_deadline_witness_method == "compact"
    assert compact.bounds.service_deadline_witness_constructive_time == pytest.approx(0.0)
    assert compact.bounds.service_deadline_witness_compact_time >= 0.0


def test_route_decoder_rejects_service_envelope_violation() -> None:
    base = tiny_instance()
    baseline = build_objective_data(base, ObjectiveWeights(0.4, 0.3, 0.3))
    manual_bounds = {customer: baseline.bounds.arrival_lb[customer] for customer in base.customers}
    instance = replace(base, config=replace(base.config, service_deadline_mode="manual", service_deadline_manual_bounds=manual_bounds))
    objective = build_objective_data(instance, ObjectiveWeights(0.4, 0.3, 0.3))
    graph = build_transformed_graph(instance)
    with pytest.raises(ServiceEnvelopeViolation, match="effective service upper bound"):
        route_from_path(0, ("Source", "C2", "C3", "Sink"), graph, objective)


def test_objective_weights_allow_zero_component_for_ablations() -> None:
    instance = tiny_instance()
    objective = build_objective_data(instance, ObjectiveWeights(0.0, 0.5, 0.5))
    assert objective.weights.delay == 0.0
    assert objective.coeffs.delay == 0.0
    assert objective.coeffs.return_time > 0.0
    assert objective.coeffs.cost > 0.0


def test_objective_weights_reject_negative_component() -> None:
    with pytest.raises(ValueError, match="nonnegative"):
        ObjectiveWeights(-0.1, 0.6, 0.5)


def test_instance_config_rejects_zero_drones_per_truck() -> None:
    with pytest.raises(ValueError, match="drones per truck"):
        InstanceConfig(seed=1, num_trucks=1, num_customers=1, distribution="PS", drones_per_truck=0)


def test_instance_config_rejects_zero_hubs() -> None:
    with pytest.raises(ValueError, match="hub count"):
        InstanceConfig(seed=1, num_trucks=1, num_customers=1, distribution="PS", drones_per_truck=1, num_hubs=0)


def test_instance_config_rejects_invalid_arc_probabilities() -> None:
    with pytest.raises(ValueError, match="arc probabilities"):
        InstanceConfig(
            seed=1,
            num_trucks=1,
            num_customers=1,
            distribution="PS",
            drones_per_truck=1,
            truck_arc_probability=-0.1,
        )
    with pytest.raises(ValueError, match="arc probabilities"):
        InstanceConfig(
            seed=1,
            num_trucks=1,
            num_customers=1,
            distribution="PS",
            drones_per_truck=1,
            hub_arc_probability=1.1,
        )


def test_instance_config_default_arc_probabilities_are_sparse_large_defaults() -> None:
    config = InstanceConfig(seed=1, num_trucks=5, num_customers=25, distribution="PS", drones_per_truck=4)
    assert config.truck_arc_probability == pytest.approx(0.05)
    assert config.hub_arc_probability == pytest.approx(0.18)
    assert config.mandatory_drone_customer_fraction == pytest.approx(0.16)
    assert config.max_drone_access_customers_per_hub == 2
    assert config.max_drone_launch_hubs_per_customer == 1
    assert config.min_drone_service_time_saving == pytest.approx(0.0)
    assert config.retain_optional_drone_arcs is False
    assert config.drone_payload == pytest.approx(6.0)
    assert config.drone_speed == pytest.approx(100.0)
    assert config.drone_endurance == pytest.approx(75.0)
    assert config.drone_cost == pytest.approx(1.0)


def test_drone_required_generator_removes_truck_service_and_builds_witness() -> None:
    config = InstanceConfig(seed=1, num_trucks=5, num_customers=25, distribution="PS", drones_per_truck=4)
    instance = generate_instance(config)
    mandatory = set(instance.mandatory_drone_customers)
    assert len(mandatory) == ceil(config.mandatory_drone_customer_fraction * config.num_customers)
    assert all(customer in instance.customers for customer in mandatory)
    assert all(customer not in arc for customer in mandatory for arc in instance.truck_arcs)
    assert all(any((hub, customer) in instance.drone_arcs for hub in instance.hubs) for customer in mandatory)
    assert {customer for _, customer in instance.drone_arcs} == mandatory
    assert all(
        sum(1 for hub in instance.hubs if (hub, customer) in instance.drone_arcs) == 1
        for customer in mandatory
    )
    assert all(
        sum(1 for _, customer in instance.drone_arcs if _ == hub) <= config.max_drone_access_customers_per_hub
        for hub in instance.hubs
    )
    witness_covered = set()
    for route in instance.witness_routes:
        witness_covered.update(node for node in route if node in instance.customers)
    for route_blocks in instance.witness_route_drone_blocks:
        for _, block in route_blocks:
            assert len(block) <= config.drones_per_truck
            witness_covered.update(block)
    assert witness_covered == set(instance.customers)
    assert len(instance.witness_routes) <= config.num_trucks
    assert sum(len(block) for route_blocks in instance.witness_route_drone_blocks for _, block in route_blocks) >= len(mandatory)
    assert all(instance.demand[customer] <= config.drone_payload for _, customer in instance.drone_arcs)
    assert all(instance.drone_trip_time[arc] <= config.drone_endurance for arc in instance.drone_arcs)


def test_instance_rejects_nonpositive_customer_demand() -> None:
    base = tiny_instance()
    demand = dict(base.demand)
    demand["C1"] = 0.0
    with pytest.raises(ValueError, match="strictly positive"):
        replace(base, demand=demand)


def test_instance_rejects_customer_demand_above_truck_payload() -> None:
    base = tiny_instance()
    with pytest.raises(ValueError, match="exceeds truck payload"):
        replace(base, config=replace(base.config, truck_payload=1.0))


def test_instance_rejects_aggregate_demand_above_fleet_payload() -> None:
    base = tiny_instance()
    with pytest.raises(ValueError, match="aggregate customer demand"):
        replace(base, config=replace(base.config, num_trucks=1, truck_payload=4.0))


def test_instance_rejects_customer_without_service_representation() -> None:
    base = tiny_instance()
    truck_arcs = frozenset(
        arc
        for arc in base.truck_arcs
        if "C1" not in arc
    )
    drone_arcs = frozenset(
        arc
        for arc in base.drone_arcs
        if arc != ("H1", "C1")
    )
    with pytest.raises(ValueError, match="no feasible service representation"):
        replace(base, truck_arcs=truck_arcs, drone_arcs=drone_arcs)


def test_instance_rejects_inconsistent_drone_trip_time() -> None:
    base = tiny_instance()
    drone_trip_time = dict(base.drone_trip_time)
    drone_trip_time[("H1", "C1")] += 1.0
    with pytest.raises(ValueError, match="outbound plus return"):
        replace(base, drone_trip_time=drone_trip_time)


def test_transformed_network_uses_hub_specific_duplicates() -> None:
    instance = tiny_instance()
    graph = build_transformed_graph(instance)
    assert duplicate_node("H1", "C1") in graph.duplicate_nodes
    assert ("H1", duplicate_node("H1", "C1")) in graph.hub_duplicate_arcs
    assert (duplicate_node("H1", "C1"), duplicate_node("H1", "C2")) in graph.duplicate_duplicate_arcs


def test_transformed_network_arc_sets_match_paper_definitions() -> None:
    instance = tiny_instance()
    graph = build_transformed_graph(instance)
    duplicate_nodes = {
        duplicate_node(hub, customer)
        for hub, customer in instance.drone_arcs
    }
    assert set(graph.duplicate_nodes) == duplicate_nodes
    assert graph.hub_duplicate_arcs == frozenset(
        (hub, duplicate_node(hub, customer))
        for hub, customer in instance.drone_arcs
    )
    assert graph.duplicate_duplicate_arcs == frozenset(
        (duplicate_node(hub, first), duplicate_node(hub, second))
        for hub in instance.hubs
        for first in instance.customers
        for second in instance.customers
        if (hub, first) in instance.drone_arcs
        and (hub, second) in instance.drone_arcs
        and graph.order[(hub, first)] < graph.order[(hub, second)]
    )
    assert graph.duplicate_regular_arcs == frozenset(
        (duplicate_node(hub, customer), regular)
        for hub, customer in instance.drone_arcs
        for i, regular in instance.truck_arcs
        if i == hub
    )
    assert graph.arc_customer_set((duplicate_node("H1", "C1"), "C2")) == frozenset({"C1", "C2"})


def test_route_encoding_preserves_drone_block_cost_components() -> None:
    instance = tiny_instance()
    objective = build_objective_data(instance, ObjectiveWeights(0.4, 0.3, 0.3))
    graph = build_transformed_graph(instance)
    path = ("Source", "H1", duplicate_node("H1", "C1"), duplicate_node("H1", "C2"), "Sink")
    route = route_from_path(0, path, graph, objective)
    assert route.truck_path == ("Source", "H1", "Sink")
    assert route.drone_blocks == {"H1": ("C1", "C2")}
    assert route.drone_sorties == 2
    assert route.service_times["C1"] == instance.truck_time[("Source", "H1")] + instance.drone_time[("H1", "C1")]
    assert route.return_time == instance.truck_time[("Source", "H1")] + max(
        instance.drone_trip_time[("H1", "C1")],
        instance.drone_trip_time[("H1", "C2")],
    ) + instance.truck_time[("H1", "Sink")]


def test_route_decoder_accepts_exact_capacity_float_payload() -> None:
    base = tiny_instance()
    demand = dict(base.demand)
    demand.update({"C1": 0.1, "C2": 0.2, "C3": 0.1})
    instance = replace(base, config=replace(base.config, truck_payload=0.3), demand=demand)
    objective = build_objective_data(instance, ObjectiveWeights(0.4, 0.3, 0.3))
    graph = build_transformed_graph(instance)
    route = route_from_path(0, ("Source", "C1", "C2", "Sink"), graph, objective)
    assert route.served == frozenset({"C1", "C2"})


def test_route_decoder_rejects_non_elementary_paths() -> None:
    instance = tiny_instance()
    objective = build_objective_data(instance, ObjectiveWeights(0.4, 0.3, 0.3))
    graph = build_transformed_graph(instance)
    with pytest.raises(ValueError, match="physical-node elementarity"):
        route_from_path(0, ("Source", "C1", "C2", "C1", "Sink"), graph, objective)
    with pytest.raises(ValueError, match="represents customer more than once"):
        route_from_path(0, ("Source", "H1", duplicate_node("H1", "C1"), "C1", "Sink"), graph, objective)


def test_route_decoder_rejects_oversized_drone_blocks() -> None:
    base = tiny_instance()
    instance = replace(base, config=replace(base.config, drones_per_truck=1))
    objective = build_objective_data(instance, ObjectiveWeights(0.4, 0.3, 0.3))
    graph = build_transformed_graph(instance)
    with pytest.raises(ValueError, match="drone fleet size"):
        route_from_path(
            0,
            ("Source", "H1", duplicate_node("H1", "C1"), duplicate_node("H1", "C2"), "Sink"),
            graph,
            objective,
        )


def test_experiment_summary_exposes_memory_metrics() -> None:
    row = _summary_row(
        {
            "case_id": "small_full_PS_seed_1",
            "instance_config": {"num_customers": 5, "num_trucks": 2, "num_hubs": 2, "drones_per_truck": 4},
            "solver_config": {"pricing_tolerance": 0.05},
            "runtime_seconds": 10.0,
            "runtime": 7.5,
            "lower_bound_shifted": 0.25,
            "lower_bound_full": 0.2,
            "upper_bound_shifted": 0.5,
            "upper_bound_full": 0.45,
            "gap": 0.5555555555555556,
            "gap_full": 0.5555555555555556,
            "gap_shifted": 0.5,
            "memory": {"python_current_allocated_mb": 1.25, "python_peak_allocated_mb": 3.5},
            "bpc_stats": {
                "pricing_diagnostics_elapsed_seconds": 2.75,
                "pricing_max_call_elapsed_seconds": 1.5,
                "pricing_labels_purged": 6,
                "pricing_backward_dominance_tests": 5,
                "pricing_backward_labels_dominated": 2,
                "pricing_dominance_prefilter_pairs": 9,
                "pricing_dominance_prefilter_rejected": 4,
                "pricing_backward_full_dominance_tests": 5,
                "pricing_join_prefilter_pairs": 13,
                "pricing_join_prefilter_rejected": 6,
                "pricing_join_full_decodes": 3,
                "pricing_lazy_rejected_before_decode": 7,
                "pricing_fully_decoded_routes": 8,
                "pricing_signature_cache_hits": 10,
                "pricing_signature_cache_misses": 11,
                "pricing_sr_coeff_cache_hits": 12,
                "pricing_sr_coeff_cache_misses": 13,
                "pricing_duplicate_equivalent_rejected": 14,
                "pricing_cost_dominated_rejected": 15,
                "pricing_stale_labels_skipped": 4,
                "pricing_standard_bound_pruned": 8,
                "pricing_farkas_bound_pruned": 13,
                "route_pool_hydration_time": 0.5,
                "route_signature_build_time": 0.6,
                "sr_coeff_build_time": 0.7,
                "duplicate_lookup_time": 0.8,
                "rmp_column_insertion_time": 0.9,
                "progress_serialization_time": 1.1,
                "signature_cache_hits": 16,
                "signature_cache_misses": 17,
                "sr_coeff_cache_hits": 18,
                "sr_coeff_cache_misses": 19,
                "duplicate_equivalent_columns_rejected": 20,
                "cost_dominated_columns_rejected": 21,
                "cost_dominated_columns_removed": 22,
                "farkas_certificate_rhs": 2.5,
                "farkas_certificate_max_column_activity": -0.25,
                "root_extraction_time": 4.0,
                "root_compact_status": "success",
                "root_model_solve_time": 1.25,
                "seed_pricing_calls": 3,
                "seed_pricing_time": 1.75,
                "repair_pricing_calls": 2,
                "repair_pricing_time": 0.5,
                "duplicate_columns_rejected": 10,
                "duplicate_merge_events": 12,
                "heuristic_hard_pool_solves": 4,
                "heuristic_hard_pool_time": 0.125,
                "heuristic_soft_pool_solves": 3,
                "heuristic_soft_pool_time": 0.25,
                "heuristic_repair_customers": 5,
                "heuristic_repair_columns_generated": 7,
                "heuristic_max_node_pool_routes": 11,
                "heuristic_max_support_routes": 9,
                "preclosure_heuristic_calls": 2,
                "preclosure_heuristic_columns_generated": 3,
                "preclosure_heuristic_incumbent_updates": 1,
                "time_to_first_incumbent": 4.5,
                "branching_nodes": 6,
                "child_nodes_created": 12,
                "customer_pair_branches": 1,
                "service_mode_branches": 2,
                "launch_pad_branches": 3,
                "transformed_arc_branches": 4,
                "route_variable_branches": 5,
                "root_closed": True,
                "best_reduced_cost_at_stop": -0.01,
            },
        }
    )
    assert row["bpc_runtime_seconds"] == 7.5
    assert row["lower_bound_shifted"] == 0.25
    assert row["lower_bound_full"] == 0.2
    assert row["upper_bound_shifted"] == 0.5
    assert row["upper_bound_full"] == 0.45
    assert row["gap"] == 0.5555555555555556
    assert row["gap_full"] == 0.5555555555555556
    assert row["gap_shifted"] == 0.5
    assert row["worker_overhead_seconds"] == 2.5
    assert row["python_current_allocated_mb"] == 1.25
    assert row["python_peak_allocated_mb"] == 3.5
    assert row["pricing_diagnostics_elapsed_seconds"] == 2.75
    assert row["pricing_tolerance"] == 0.05
    assert row["pricing_max_call_elapsed_seconds"] == 1.5
    assert row["pricing_labels_purged"] == 6
    assert row["pricing_backward_dominance_tests"] == 5
    assert row["pricing_backward_labels_dominated"] == 2
    assert row["pricing_dominance_prefilter_pairs"] == 9
    assert row["pricing_join_prefilter_rejected"] == 6
    assert row["pricing_fully_decoded_routes"] == 8
    assert row["pricing_signature_cache_hits"] == 10
    assert row["pricing_cost_dominated_rejected"] == 15
    assert row["pricing_stale_labels_skipped"] == 4
    assert row["pricing_standard_bound_pruned"] == 8
    assert row["pricing_farkas_bound_pruned"] == 13
    assert row["farkas_certificate_rhs"] == 2.5
    assert row["farkas_certificate_max_column_activity"] == -0.25
    assert row["root_compact_non_solver_time"] == 2.75
    assert row["root_compact_status"] == "success"
    assert row["route_pool_hydration_time"] == 0.5
    assert row["rmp_column_insertion_time"] == 0.9
    assert row["progress_serialization_time"] == 1.1
    assert row["signature_cache_hits"] == 16
    assert row["cost_dominated_columns_removed"] == 22
    assert row["seed_pricing_calls"] == 3
    assert row["seed_pricing_time"] == 1.75
    assert row["repair_pricing_calls"] == 2
    assert row["repair_pricing_time"] == 0.5
    assert row["duplicate_columns_rejected"] == 10
    assert row["duplicate_merge_events"] == 12
    assert row["heuristic_hard_pool_solves"] == 4
    assert row["heuristic_hard_pool_time"] == 0.125
    assert row["heuristic_soft_pool_solves"] == 3
    assert row["heuristic_soft_pool_time"] == 0.25
    assert row["heuristic_repair_customers"] == 5
    assert row["heuristic_repair_columns_generated"] == 7
    assert row["heuristic_max_node_pool_routes"] == 11
    assert row["heuristic_max_support_routes"] == 9
    assert row["preclosure_heuristic_calls"] == 2
    assert row["preclosure_heuristic_columns_generated"] == 3
    assert row["preclosure_heuristic_incumbent_updates"] == 1
    assert row["time_to_first_incumbent"] == 4.5
    assert row["branching_nodes"] == 6
    assert row["child_nodes_created"] == 12
    assert row["customer_pair_branches"] == 1
    assert row["service_mode_branches"] == 2
    assert row["launch_pad_branches"] == 3
    assert row["transformed_arc_branches"] == 4
    assert row["route_variable_branches"] == 5
    assert row["root_closed"] is True
    assert row["best_reduced_cost_at_stop"] == -0.01


def test_timeout_pricing_diagnostics_split_seed_and_repair_modes(tmp_path) -> None:
    records = [
        {
            "mode": "seed",
            "labels_generated": 5,
            "labels_dominated": 1,
            "backward_dominance_tests": 1,
            "backward_labels_dominated": 1,
            "labels_purged": 0,
            "stale_labels_skipped": 0,
            "labels_pruned": 0,
            "standard_bound_pruned": 0,
            "farkas_bound_pruned": 0,
            "complete_routes_generated": 2,
            "max_queue_size": 3,
            "elapsed_seconds": 0.25,
        },
        {
            "mode": "seed_interrupted",
            "labels_generated": 7,
            "labels_dominated": 2,
            "backward_dominance_tests": 2,
            "backward_labels_dominated": 2,
            "labels_purged": 1,
            "stale_labels_skipped": 1,
            "labels_pruned": 1,
            "standard_bound_pruned": 1,
            "farkas_bound_pruned": 0,
            "complete_routes_generated": 1,
            "max_queue_size": 4,
            "elapsed_seconds": 0.75,
        },
        {
            "mode": "repair",
            "labels_generated": 11,
            "labels_dominated": 3,
            "backward_dominance_tests": 3,
            "backward_labels_dominated": 3,
            "labels_purged": 2,
            "stale_labels_skipped": 0,
            "labels_pruned": 2,
            "standard_bound_pruned": 0,
            "farkas_bound_pruned": 2,
            "complete_routes_generated": 4,
            "max_queue_size": 6,
            "elapsed_seconds": 1.5,
        },
    ]
    (tmp_path / "pricing_diagnostics.jsonl").write_text(
        "\n".join(json.dumps(record) for record in records) + "\n",
        encoding="utf-8",
    )
    summary = _read_pricing_diagnostics(tmp_path)
    assert summary["mode_counts"] == {"repair": 1, "seed": 1, "seed_interrupted": 1}
    assert summary["mode_elapsed_seconds"] == {"repair": 1.5, "seed": 0.25, "seed_interrupted": 0.75}
    assert summary["pricing_standard_bound_pruned"] == 1
    assert summary["pricing_farkas_bound_pruned"] == 2
    assert summary["pricing_labels_purged"] == 3
    assert summary["pricing_backward_dominance_tests"] == 6
    assert summary["pricing_backward_labels_dominated"] == 6
    assert summary["pricing_stale_labels_skipped"] == 1
    merged = _merge_timeout_stats({}, summary)
    assert merged["seed_pricing_calls"] == 2
    assert merged["seed_pricing_time"] == 1.0
    assert merged["repair_pricing_calls"] == 1
    assert merged["repair_pricing_time"] == 1.5
    assert merged["pricing_standard_bound_pruned"] == 1
    assert merged["pricing_farkas_bound_pruned"] == 2
    assert merged["pricing_labels_purged"] == 3
    assert merged["pricing_backward_dominance_tests"] == 6
    assert merged["pricing_backward_labels_dominated"] == 6
    assert merged["pricing_stale_labels_skipped"] == 1


def test_experiment_defaults_match_prompt_scales_and_seed_count() -> None:
    assert DEFAULT_SEEDS == [1, 2, 3]
    common_graph = {"truck_arc_probability": 0.05, "hub_arc_probability": 0.18}
    assert SCALES["small"] == {"num_customers": 5, "num_trucks": 2, "num_hubs": 2, "drones_per_truck": 4, **common_graph}
    assert SCALES["medium"] == {
        "num_customers": 15,
        "num_trucks": 3,
        "num_hubs": 2,
        "drones_per_truck": 4,
        **MEDIUM_CASE_DEFAULTS,
    }
    assert SCALES["large"] == {
        "num_customers": 25,
        "num_trucks": 5,
        "num_hubs": 2,
        "drones_per_truck": 4,
        **common_graph,
    }


def test_dimension_matched_medium_case_defaults_reproduce_selected_profile() -> None:
    medium = case_defaults(num_customers=15, num_trucks=3, num_hubs=2, drones_per_truck=4)
    assert medium == MEDIUM_CASE_DEFAULTS
    assert medium["truck_arc_probability"] == pytest.approx(0.05)
    assert medium["hub_arc_probability"] == pytest.approx(0.18)
    assert medium["max_drone_launch_hubs_per_customer"] == 1
    assert medium["min_drone_service_time_saving"] == pytest.approx(0.0)
    assert medium["retain_optional_drone_arcs"] is False
    assert medium["service_deadline_mode"] == "none"
    assert medium["service_deadline_offset_min"] == pytest.approx(45.0)
    assert medium["service_deadline_offset_max"] == pytest.approx(120.0)
    assert medium["service_deadline_witness_slack"] == pytest.approx(5.0)
    assert medium["pricing_tolerance"] == pytest.approx(0.05)
    assert medium["pricing_parallel_workers"] == 6
    assert medium["pricing_worker_backend"] == "thread"
    assert medium["prefix_task_depth_child"] == 1

    generic = case_defaults(num_customers=25, num_trucks=5, num_hubs=2, drones_per_truck=4)
    assert generic == GENERIC_CASE_DEFAULTS
    assert generic["pricing_parallel_workers"] == 6

    small = case_defaults(num_customers=5, num_trucks=2, num_hubs=2, drones_per_truck=4)
    assert small["pricing_parallel_workers"] == 6
