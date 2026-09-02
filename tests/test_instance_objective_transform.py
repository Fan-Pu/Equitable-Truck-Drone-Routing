from __future__ import annotations

import json
from inspect import signature
from math import ceil, floor, hypot, inf
import tomllib
from dataclasses import dataclass, replace
from pathlib import Path
from types import SimpleNamespace

import pytest
from gurobipy import GRB

import thvrpd.campaign as campaign_module
import thvrpd.feasibility as feasibility_module
import thvrpd.repair as repair_module
import thvrpd.solution_space_audit as audit_module
from thvrpd.campaign import (
    CampaignCase,
    _parser as campaign_parser,
    _prepare_instance_snapshots,
    _solver_command,
    campaign_cases,
)
from thvrpd.columns import route_signature
from thvrpd.compact import _common_time_big_m, compact_feasibility_status, solve_compact_solution
from thvrpd.config import GENERIC_CASE_DEFAULTS, InstanceConfig, ObjectiveWeights, SolverConfig, case_defaults
from thvrpd.experiments import SCALES, _parser as experiments_parser
from thvrpd.dense_multiscale_campaign import (
    LARGE_RESULT_DIRS,
    SCALE_ORDER,
    SCENARIOS as DENSE_MULTISCALE_SCENARIOS,
    _large_artifact_hashes,
    _validate_manifest as validate_dense_multiscale_manifest,
)
from thvrpd.medium_crossover_campaign import PILOT_CONFIGS, _pilot_metrics
from thvrpd.feasibility import (
    FeasibilityGateDiagnostics,
    FeasibilityGateResult,
    FeasibilityPrecheckResult,
    FeasibilitySolveStage,
    exact_feasibility_check,
    exact_feasibility_precheck,
    legacy_feasibility_variable_count,
)
from thvrpd.instance import (
    GeographicOverlapError,
    INSTANCE_SNAPSHOT_SCHEMA_VERSION,
    InstanceAcceptanceResult,
    InstanceData,
    LOCATION_OVERLAP_TOLERANCE,
    _instance_snapshot_digest,
    generate_candidate,
    generate_instance,
    instance_physical_fingerprint,
    read_instance_snapshot,
    tiny_instance,
    validate_instance_case,
    write_instance_snapshot,
)
from thvrpd.objective import build_objective_data
from thvrpd.routes import ServiceEnvelopeViolation, route_from_path
from thvrpd.repair import (
    ArcSwapRepairResult,
    RepairValidation,
    allowable_internal_truck_arcs,
    discover_or_repair_instance,
    solve_strengthened_arc_projection_reference,
    truck_arc_class_counts,
)
from thvrpd.service_windows import load_manual_service_deadline_bounds
from thvrpd.solve import _parser as solve_parser
from thvrpd.transform import build_transformed_graph, duplicate_node


def _shared_duplicate_instance() -> InstanceData:
    config = InstanceConfig(
        seed=7,
        num_trucks=2,
        num_customers=3,
        distribution="PS",
        drones_per_truck=3,
        num_hubs=2,
        truck_arc_probability=1.0,
        hub_arc_probability=1.0,
        truck_payload=20.0,
        drone_payload=5.0,
        drone_endurance=100.0,
        service_deadline_mode="none",
    )
    source = "Source"
    sink = "Sink"
    customers = ("C1", "C2", "C10")
    hubs = ("H1", "H2")
    nodes = (source,) + customers + hubs + (sink,)
    truck_arcs = {
        (source, "H1"),
        (source, "H2"),
        ("H1", "C10"),
        ("H1", sink),
        ("H2", sink),
    }
    for customer in customers:
        truck_arcs.add((source, customer))
        truck_arcs.add((customer, sink))
    truck_time = {arc: 1.0 for arc in truck_arcs}
    drone_arcs = {
        ("H1", "C1"),
        ("H1", "C2"),
        ("H1", "C10"),
        ("H2", "C1"),
        ("H2", "C10"),
    }
    drone_time = {}
    drone_trip_time = {}
    for hub, customer in drone_arcs:
        drone_time[(hub, customer)] = 1.0
        drone_time[(customer, hub)] = 1.0
        drone_trip_time[(hub, customer)] = 2.0
    demand = {node: 0.0 for node in nodes}
    demand.update({customer: 1.0 for customer in customers})
    locations = {node: (float(index), 0.0) for index, node in enumerate(nodes)}
    locations[sink] = locations[source]
    return InstanceData(
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


def _fake_gate_result(feasible: bool, status: str) -> FeasibilityGateResult:
    return FeasibilityGateResult(
        feasible=feasible,
        status=status,
        diagnostics=FeasibilityGateDiagnostics(
            precheck=FeasibilityPrecheckResult(True, None, tuple(), 1, 0, 0.0),
            model_build_time_seconds=0.0,
            variable_count=1,
            constraint_count=1,
            nonzero_count=1,
            legacy_variable_count=2,
            fallback_used=False,
            stages=(FeasibilitySolveStage(1, status, 2, 0.0, 0.0, 0.0, int(feasible)),),
        ),
    )


@dataclass(frozen=True)
class _FakeRepairDiagnostics:
    status: str


def _fake_repair_result(instance: InstanceData, feasible: bool, status: str) -> ArcSwapRepairResult:
    return ArcSwapRepairResult(
        feasible=feasible,
        status=status,
        instance=instance if feasible else None,
        diagnostics=_FakeRepairDiagnostics(status),
    )


def _forced_drone_capacity_instance() -> InstanceData:
    config = InstanceConfig(
        seed=31,
        num_trucks=1,
        num_customers=2,
        num_hubs=1,
        distribution="PS",
        drones_per_truck=1,
        truck_payload=20.0,
        drone_payload=5.0,
        drone_endurance=100.0,
        mandatory_drone_customer_fraction=1.0,
        service_deadline_mode="none",
    )
    source, sink = "Source", "Sink"
    customers = ("C1", "C2")
    hubs = ("H1",)
    nodes = (source, *customers, *hubs, sink)
    truck_arcs = frozenset({(source, "H1"), ("H1", sink)})
    drone_arcs = frozenset({("H1", "C1"), ("H1", "C2")})
    return InstanceData(
        config=config,
        depot_source=source,
        depot_sink=sink,
        customers=customers,
        hubs=hubs,
        nodes=nodes,
        truck_arcs=truck_arcs,
        drone_arcs=drone_arcs,
        truck_time={arc: 1.0 for arc in truck_arcs},
        drone_time={
            ("H1", "C1"): 1.0,
            ("C1", "H1"): 1.0,
            ("H1", "C2"): 1.0,
            ("C2", "H1"): 1.0,
        },
        drone_trip_time={("H1", "C1"): 2.0, ("H1", "C2"): 2.0},
        demand={source: 0.0, sink: 0.0, "H1": 0.0, "C1": 1.0, "C2": 1.0},
        locations={source: (0.0, 0.0), sink: (0.0, 0.0), "C1": (1.0, 0.0), "C2": (2.0, 0.0), "H1": (1.0, 1.0)},
        mandatory_drone_customers=customers,
    )


def _one_truck_disconnected_customers_instance() -> InstanceData:
    config = InstanceConfig(
        seed=37,
        num_trucks=1,
        num_customers=2,
        num_hubs=1,
        distribution="PS",
        drones_per_truck=1,
        truck_payload=20.0,
        mandatory_drone_customer_fraction=0.0,
        service_deadline_mode="none",
    )
    source, sink = "Source", "Sink"
    customers = ("C1", "C2")
    hubs = ("H1",)
    nodes = (source, *customers, *hubs, sink)
    truck_arcs = frozenset(
        {
            (source, "C1"),
            ("C1", sink),
            (source, "C2"),
            ("C2", sink),
            (source, "H1"),
            ("H1", sink),
        }
    )
    return InstanceData(
        config=config,
        depot_source=source,
        depot_sink=sink,
        customers=customers,
        hubs=hubs,
        nodes=nodes,
        truck_arcs=truck_arcs,
        drone_arcs=frozenset(),
        truck_time={arc: 1.0 for arc in truck_arcs},
        drone_time={},
        drone_trip_time={},
        demand={source: 0.0, sink: 0.0, "H1": 0.0, "C1": 1.0, "C2": 1.0},
        locations={source: (0.0, 0.0), sink: (0.0, 0.0), "C1": (1.0, 0.0), "C2": (2.0, 0.0), "H1": (1.0, 1.0)},
    )


def _repairable_mandatory_drone_instance() -> InstanceData:
    config = InstanceConfig(
        seed=41,
        num_trucks=1,
        num_customers=4,
        num_hubs=1,
        distribution="PS",
        drones_per_truck=1,
        truck_payload=20.0,
        drone_payload=5.0,
        drone_endurance=100.0,
        mandatory_drone_customer_fraction=0.25,
        service_deadline_mode="random_absolute",
        service_deadline_offset_min=30.0,
        service_deadline_offset_max=30.0,
    )
    source, sink = "Source", "Sink"
    customers = ("C1", "C2", "C3", "C4")
    hubs = ("H1",)
    nodes = (source, *customers, *hubs, sink)
    locations = {
        source: (0.0, 0.0),
        sink: (0.0, 0.0),
        "C1": (1.0, 0.0),
        "C2": (2.0, 0.0),
        "C3": (3.0, 0.0),
        "C4": (2.0, 2.0),
        "H1": (2.0, 1.0),
    }
    fixed = {
        *((source, node) for node in ("C1", "C2", "C3", "H1")),
        *((node, sink) for node in ("C1", "C2", "C3", "H1")),
    }
    internal = {
        ("C1", "C2"),
        ("C1", "C3"),
        ("C1", "H1"),
        ("H1", "C1"),
    }
    truck_arcs = frozenset(fixed | internal)

    def truck_time(arc: tuple[str, str]) -> float:
        left, right = arc
        return (
            abs(locations[left][0] - locations[right][0])
            + abs(locations[left][1] - locations[right][1])
        ) / config.truck_speed * 60.0

    drone_one_way = hypot(
        locations["H1"][0] - locations["C4"][0],
        locations["H1"][1] - locations["C4"][1],
    ) / config.drone_speed * 60.0
    return InstanceData(
        config=config,
        depot_source=source,
        depot_sink=sink,
        customers=customers,
        hubs=hubs,
        nodes=nodes,
        truck_arcs=truck_arcs,
        drone_arcs=frozenset({("H1", "C4")}),
        truck_time={arc: truck_time(arc) for arc in truck_arcs},
        drone_time={("H1", "C4"): drone_one_way, ("C4", "H1"): drone_one_way},
        drone_trip_time={("H1", "C4"): 2.0 * drone_one_way},
        demand={source: 0.0, sink: 0.0, "H1": 0.0, **{customer: 1.0 for customer in customers}},
        locations=locations,
        mandatory_drone_customers=("C4",),
    )
def test_pyproject_declares_required_dependencies() -> None:
    pyproject = tomllib.loads(Path("pyproject.toml").read_text(encoding="utf-8"))
    dependencies = set(pyproject["project"]["dependencies"])
    assert {"gurobipy", "numpy", "networkx", "psutil", "pytest"}.issubset(dependencies)


def test_full_campaign_plan_and_commands_use_scale_defaults(tmp_path: Path) -> None:
    cases = campaign_cases()
    assert len(cases) == 24
    assert [case.seed for case in cases if case.distribution == "PS"] == [1, 2, 3, 4] * 3
    assert [case.seed for case in cases if case.distribution == "PC"] == [5, 6, 7, 8] * 3
    assert len({(case.case_id, solver) for case in cases for solver in ("bpc", "compact")}) == 48
    large = cases[0]
    instance_file = tmp_path / "instances" / "large_PS_seed1.json"
    instance_hash = "a" * 64
    bpc_command = _solver_command(
        large,
        "bpc",
        tmp_path / "bpc",
        3600.0,
        0,
        Path("D:/gurobi"),
        instance_file,
        instance_hash,
    )
    compact_command = _solver_command(
        large,
        "compact",
        tmp_path / "compact",
        3600.0,
        0,
        Path("D:/gurobi"),
        instance_file,
        instance_hash,
    )
    assert "--pricing-tolerance" not in bpc_command
    assert "--pricing-parallel-workers" not in bpc_command
    assert "--truck-arc-probability" not in bpc_command
    assert "--service-deadline-mode" not in bpc_command
    assert "--time-limit" in bpc_command and "3600.0" in bpc_command
    assert "--time-limit" in compact_command and "3600.0" in compact_command
    assert bpc_command[bpc_command.index("--threads") + 1] == "0"
    assert compact_command[compact_command.index("--threads") + 1] == "0"
    assert bpc_command[bpc_command.index("--instance-file") + 1] == str(instance_file)
    assert compact_command[compact_command.index("--instance-file") + 1] == str(instance_file)
    assert bpc_command[bpc_command.index("--instance-sha256") + 1] == instance_hash
    assert compact_command[compact_command.index("--instance-sha256") + 1] == instance_hash


def test_solver_defaults_match_revised_public_contract() -> None:
    config = SolverConfig()
    assert config.threads == 0
    assert solve_parser().get_default("threads") == 0
    assert experiments_parser().get_default("threads") == 0
    assert campaign_parser().get_default("threads") == 0
    assert signature(solve_compact_solution).parameters["threads"].default == 0
    assert 'parser.add_argument("--threads", type=int, default=0)' in Path("thvrpd/compact_benchmark.py").read_text()
    assert config.pricing_tolerance == pytest.approx(0.0001)
    assert "Params.OptimalityTol" not in Path("thvrpd/rmp.py").read_text()
    assert config.pricing_parallel_workers == 12
    assert config.pricing_worker_backend == "process"
    assert config.pricing_workload_customer_weight == pytest.approx(1.0)
    assert config.pricing_workload_out_degree_weight == pytest.approx(0.25)
    assert config.pricing_workload_drone_pad_weight == pytest.approx(0.5)
    assert config.pricing_workload_deadline_weight == pytest.approx(0.5)
    assert config.pricing_split_open_labels_min == 500
    assert config.pricing_split_gap_factor == pytest.approx(1.0)
    assert solve_parser().get_default("pricing_split_gap_factor") == pytest.approx(1.0)
    assert experiments_parser().get_default("pricing_split_gap_factor") == pytest.approx(1.0)
    assert config.pricing_split_elapsed_min == pytest.approx(5.0)
    assert config.pricing_split_work_min == 2000
    assert config.pricing_dynamic_refinement_depth == 2
    assert config.pricing_checkpoint_extension_period == 1000
    assert config.enable_root_compact_warm_start is True
    assert config.root_compact_solve_time_limit == pytest.approx(60.0)
    with pytest.raises(ValueError, match="threads must be nonnegative"):
        replace(config, threads=-1)
    retired = {
        "enable_constructive_initial_columns",
        "root_constructive_time_limit",
        "enable_bidirectional_pricing",
        "enable_dynamic_kcore_refinement",
        "enable_resumable_child_certification",
        "enable_sr_aging",
        "enable_drone_diversification_warm_start",
        "support_threshold",
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
    }
    assert retired.isdisjoint(config.__dataclass_fields__)


def test_solver_cli_exposes_no_retired_options() -> None:
    options = {option for action in solve_parser()._actions for option in action.option_strings}
    assert "--pricing-parallel-workers" in options
    assert "--pricing-split-open-labels-min" in options
    assert "--pricing-dynamic-refinement-depth" in options
    assert "--root-compact-solve-time-limit" in options
    assert "--root-constructive-time-limit" not in options
    assert "--disable-constructive-initial-columns" not in options
    assert "--disable-bidirectional-pricing" not in options
    assert "--disable-dynamic-kcore-refinement" not in options
    assert "--root-compact-wall-time-limit" not in options
    assert "--enable-drone-diversification-warm-start" not in options
    assert "--support-threshold" not in options
    assert "--max-drone-access-customers-per-hub" not in options
    assert "--max-drone-launch-hubs-per-customer" not in options
    assert "--retain-optional-drone-arcs" not in options
    assert "--service-deadline-witness-slack" not in options
    assert "--seed-batch-size" not in options
    assert "--repair-batch-size" not in options
    assert "--seed-reward" not in options
    assert "--repair-reward" not in options
    assert "--phase-i-max-rounds" not in options
    assert "--disable-rmp-basis-reuse" not in options
    assert "--disable-inactive-column-storage" not in options


def test_scale_defaults_are_consistent() -> None:
    assert SCALES == {
        "small": {"num_customers": 5, "num_trucks": 2, "num_hubs": 2, "drones_per_truck": 4},
        "medium": {"num_customers": 10, "num_trucks": 3, "num_hubs": 2, "drones_per_truck": 4},
        "large": {"num_customers": 30, "num_trucks": 6, "num_hubs": 3, "drones_per_truck": 4},
    }
    defaults_by_scale = [case_defaults(**dimensions) for dimensions in SCALES.values()]
    assert all(defaults == defaults_by_scale[0] for defaults in defaults_by_scale[1:])
    for dimensions in SCALES.values():
        defaults = case_defaults(**dimensions)
        assert defaults["pricing_tolerance"] == pytest.approx(0.0001)
        assert defaults["pricing_parallel_workers"] == 12
        assert defaults["pricing_worker_backend"] == "process"
        assert defaults["truck_arc_probability"] == pytest.approx(0.05)
        assert defaults["hub_arc_probability"] == pytest.approx(0.18)
        assert defaults["service_deadline_mode"] == "random_absolute"
        assert defaults["service_deadline_offset_min"] == pytest.approx(30.0)
        assert defaults["service_deadline_offset_max"] == pytest.approx(90.0)
    config = InstanceConfig(seed=1, num_trucks=2, num_customers=5, distribution="PS", drones_per_truck=4)
    assert config.area_side == pytest.approx(25.0)
    assert config.truck_speed == pytest.approx(40.0)
    assert config.drone_speed == pytest.approx(40.0)
    assert config.truck_payload == pytest.approx(50.0)
    assert config.drone_payload == pytest.approx(2.3)
    assert config.drone_endurance == pytest.approx(30.0)
    assert config.truck_cost == pytest.approx(20.0)
    assert config.drone_cost == pytest.approx(6.0)
    assert config.service_deadline_mode == "random_absolute"
    assert config.service_deadline_offset_min == pytest.approx(30.0)
    assert config.service_deadline_offset_max == pytest.approx(90.0)


def test_tiny_objective_bounds_and_shift_are_consistent() -> None:
    instance = tiny_instance()
    objective = build_objective_data(instance, ObjectiveWeights(0.4, 0.3, 0.3))
    assert objective.bounds.delay_ub >= objective.bounds.delay_lb
    assert objective.bounds.return_ub >= objective.bounds.return_lb
    assert objective.bounds.cost_ub >= objective.bounds.cost_lb
    shifted = 1.234
    assert objective.full_value_from_route_sum(shifted) == pytest.approx(shifted + objective.coeffs.shift)


def test_common_time_big_m_uses_the_tighter_paper_formula() -> None:
    truck_dominant = _common_time_big_m(
        route_time_ub=300.0,
        max_truck_time=25.0,
        max_drone_trip=30.0,
        max_drone_oneway=15.0,
    )
    assert truck_dominant == pytest.approx(355.0)

    drone_dominant = _common_time_big_m(
        route_time_ub=300.0,
        max_truck_time=10.0,
        max_drone_trip=5.0,
        max_drone_oneway=40.0,
    )
    assert drone_dominant == pytest.approx(340.0)

    old_additive_value = 300.0 + 25.0 + 30.0 + 15.0
    assert truck_dominant < old_additive_value
    assert truck_dominant <= old_additive_value


def test_manual_service_deadline_loader_and_effective_bounds(tmp_path: Path) -> None:
    path = tmp_path / "deadlines.json"
    path.write_text(json.dumps({"C1": 20.0, "C2": 30.0, "C3": 40.0}), encoding="utf-8")
    bounds = load_manual_service_deadline_bounds(path)
    base = tiny_instance()
    instance = replace(base, config=replace(base.config, service_deadline_mode="manual", service_deadline_manual_bounds=bounds))
    objective = build_objective_data(instance, ObjectiveWeights(0.4, 0.3, 0.3))
    for customer in base.customers:
        assert objective.bounds.service_ub[customer] == pytest.approx(
            min(bounds[customer], objective.bounds.data_service_ub[customer])
        )


def test_random_absolute_service_deadlines_are_seeded() -> None:
    config = InstanceConfig(
        seed=10,
        num_trucks=2,
        num_customers=5,
        num_hubs=2,
        distribution="PS",
        drones_per_truck=4,
        service_deadline_mode="random_absolute",
        service_deadline_offset_min=30.0,
        service_deadline_offset_max=90.0,
        mandatory_drone_customer_fraction=0.0,
    )
    first = generate_candidate(config)
    second = generate_candidate(config)
    first_bounds = build_objective_data(first, ObjectiveWeights(0.4, 0.3, 0.3)).bounds.service_ub
    second_bounds = build_objective_data(second, ObjectiveWeights(0.4, 0.3, 0.3)).bounds.service_ub
    assert first_bounds == second_bounds
    assert set(first_bounds) == set(first.customers)
    objective = build_objective_data(first, ObjectiveWeights(0.4, 0.3, 0.3))
    for customer in first.customers:
        offset = objective.bounds.service_deadline[customer] - objective.bounds.arrival_lb[customer]
        assert 30.0 <= offset <= 90.0
        assert objective.bounds.service_deadline_candidate[customer] == pytest.approx(
            objective.bounds.service_deadline[customer]
        )


def test_route_decoder_enforces_service_envelope() -> None:
    base = tiny_instance()
    baseline = build_objective_data(base, ObjectiveWeights(0.4, 0.3, 0.3))
    bounds = dict(baseline.bounds.service_ub)
    bounds["C1"] = baseline.bounds.arrival_lb["C1"]
    instance = replace(base, config=replace(base.config, service_deadline_mode="manual", service_deadline_manual_bounds=bounds))
    graph = build_transformed_graph(instance)
    objective = build_objective_data(instance, ObjectiveWeights(0.4, 0.3, 0.3))
    with pytest.raises(ServiceEnvelopeViolation):
        route_from_path(0, ("Source", "C2", "C1", "Sink"), graph, objective)


def test_transformed_network_uses_shared_canonical_duplicates() -> None:
    instance = tiny_instance()
    graph = build_transformed_graph(instance)
    duplicates = {duplicate_node(customer) for customer in instance.customers}
    assert set(graph.duplicate_nodes) == duplicates
    assert (duplicate_node("C2"), duplicate_node("C1")) not in graph.arcs
    assert (duplicate_node("C1"), duplicate_node("C2")) in graph.arcs


def test_shared_duplicate_union_arcs_require_the_active_pad() -> None:
    instance = _shared_duplicate_instance()
    graph = build_transformed_graph(instance)
    c1 = duplicate_node("C1")
    c2 = duplicate_node("C2")
    c10 = duplicate_node("C10")
    assert graph.duplicate_nodes == (c1, c2, c10)
    assert ("H1", c2) in graph.hub_duplicate_arcs
    assert ("H2", c2) not in graph.hub_duplicate_arcs
    assert (c1, c2) in graph.duplicate_duplicate_arcs
    assert graph.arc_compatible_with_active_pad((c1, c2), "H1")
    assert not graph.arc_compatible_with_active_pad((c1, c2), "H2")
    assert (c2, c10) in graph.duplicate_duplicate_arcs
    assert (c10, c2) not in graph.duplicate_duplicate_arcs
    assert (c1, "C10") in graph.duplicate_regular_arcs
    assert graph.arc_compatible_with_active_pad((c1, "C10"), "H1")
    assert not graph.arc_compatible_with_active_pad((c1, "C10"), "H2")
    assert (c10, "C10") not in graph.duplicate_regular_arcs
    assert graph.realized_travel_time(("H1", c1)) == pytest.approx(2.0)
    assert graph.realized_travel_time((c1, c2), "H1") == pytest.approx(2.0)
    assert graph.realized_travel_time((c1, "C10"), "H1") == pytest.approx(1.0)


def test_route_decoder_rejects_union_arc_from_incompatible_pad() -> None:
    instance = _shared_duplicate_instance()
    objective = build_objective_data(instance, ObjectiveWeights(0.4, 0.3, 0.3))
    graph = build_transformed_graph(instance)
    c1 = duplicate_node("C1")
    c2 = duplicate_node("C2")
    c10 = duplicate_node("C10")
    h1_route = route_from_path(0, ("Source", "H1", c1, c2, "Sink"), graph, objective)
    h2_route = route_from_path(1, ("Source", "H2", c1, c10, "Sink"), graph, objective)
    assert h1_route.pad_served == frozenset({("H1", "C1"), ("H1", "C2")})
    assert h2_route.pad_served == frozenset({("H2", "C1"), ("H2", "C10")})
    h1_signature = route_signature(h1_route, graph, frozenset(instance.customers)).core
    h2_signature = route_signature(h2_route, graph, frozenset(instance.customers)).core
    assert h1_signature.pad_assignment_key != h2_signature.pad_assignment_key
    assert h1_route.return_time == pytest.approx(4.0)
    assert h2_route.return_time == pytest.approx(4.0)
    with pytest.raises(ValueError, match="incompatible with active pad"):
        route_from_path(2, ("Source", "H2", c1, c2, "Sink"), graph, objective)


def test_route_encoding_preserves_drone_block_cost_components() -> None:
    instance = tiny_instance()
    objective = build_objective_data(instance, ObjectiveWeights(0.4, 0.3, 0.3))
    graph = build_transformed_graph(instance)
    path = ("Source", "H1", duplicate_node("C1"), duplicate_node("C2"), "Sink")
    route = route_from_path(0, path, graph, objective)
    assert route.drone_served == frozenset({"C1", "C2"})
    assert route.pad_served == frozenset({("H1", "C1"), ("H1", "C2")})
    expected_wait = max(instance.drone_trip_time[("H1", "C1")], instance.drone_trip_time[("H1", "C2")])
    expected_return = (
        instance.truck_time[("Source", "H1")]
        + expected_wait
        + instance.truck_time[("H1", "Sink")]
    )
    assert route.return_time == pytest.approx(expected_return)
    assert route.cost + objective.coeffs.shift == pytest.approx(objective.full_value_from_route_sum(route.cost))


def test_instance_config_rejects_invalid_basic_data() -> None:
    with pytest.raises(ValueError):
        replace(tiny_instance().config, drones_per_truck=0)
    with pytest.raises(ValueError):
        replace(tiny_instance().config, truck_arc_probability=1.1)
    assert "retain_optional_drone_arcs" not in GENERIC_CASE_DEFAULTS
    assert "max_drone_launch_hubs_per_customer" not in InstanceConfig.__dataclass_fields__
    assert "service_deadline_witness_slack" not in InstanceConfig.__dataclass_fields__


def test_instance_locations_allow_only_colocated_depot_copies() -> None:
    instance = tiny_instance()
    assert instance.locations[instance.depot_source] == instance.locations[instance.depot_sink]

    overlap_cases = (
        ("C1", "C2"),
        ("H1", "C1"),
        (instance.depot_source, "C1"),
    )
    for left, right in overlap_cases:
        locations = dict(instance.locations)
        locations[right] = locations[left]
        with pytest.raises(GeographicOverlapError, match="geographic node overlap") as exc_info:
            replace(instance, locations=locations)
        assert left in str(exc_info.value)
        assert right in str(exc_info.value)

    two_hub = _shared_duplicate_instance()
    locations = dict(two_hub.locations)
    locations["H2"] = locations["H1"]
    with pytest.raises(GeographicOverlapError, match="H1.*H2"):
        replace(two_hub, locations=locations)


def test_instance_locations_reject_near_overlap_missing_nonfinite_and_split_depot() -> None:
    instance = tiny_instance()

    near_overlap = dict(instance.locations)
    near_overlap["C2"] = (
        near_overlap["C1"][0] + 0.5 * LOCATION_OVERLAP_TOLERANCE,
        near_overlap["C1"][1],
    )
    with pytest.raises(GeographicOverlapError, match="C1.*C2"):
        replace(instance, locations=near_overlap)

    missing = dict(instance.locations)
    del missing["C1"]
    with pytest.raises(ValueError, match="missing geographic location for node C1"):
        replace(instance, locations=missing)

    nonfinite = dict(instance.locations)
    nonfinite["C1"] = (inf, 0.0)
    with pytest.raises(ValueError, match="node location must be finite: C1"):
        replace(instance, locations=nonfinite)

    split_depot = dict(instance.locations)
    split_depot[instance.depot_sink] = (LOCATION_OVERLAP_TOLERANCE * 2.0, 0.0)
    with pytest.raises(ValueError, match="source and sink depot copies must be geographically colocated"):
        replace(instance, locations=split_depot)


@pytest.mark.parametrize("distribution,seed", [("PS", 12), ("PC", 4)])
def test_generated_candidate_locations_do_not_overlap(distribution: str, seed: int) -> None:
    config = InstanceConfig(
        seed=seed,
        num_trucks=8,
        num_customers=20,
        num_hubs=2,
        distribution=distribution,
        drones_per_truck=4,
        mandatory_drone_customer_fraction=0.0,
    )
    instance = generate_candidate(config)
    assert instance.locations[instance.depot_source] == instance.locations[instance.depot_sink]
    sites = (instance.depot_source, *instance.customers, *instance.hubs)
    for index, left in enumerate(sites):
        for right in sites[index + 1 :]:
            assert hypot(
                instance.locations[left][0] - instance.locations[right][0],
                instance.locations[left][1] - instance.locations[right][1],
            ) > LOCATION_OVERLAP_TOLERANCE


def test_candidate_generation_uses_bounded_demands_and_exact_drone_range() -> None:
    config = InstanceConfig(
        seed=12,
        num_trucks=8,
        num_customers=20,
        num_hubs=2,
        distribution="PS",
        drones_per_truck=4,
        mandatory_drone_customer_fraction=0.0,
    )
    instance = generate_candidate(config)
    low = [customer for customer in instance.customers if instance.demand[customer] <= config.drone_payload]
    high = [customer for customer in instance.customers if instance.demand[customer] >= config.high_demand_weight_min]
    assert len(low) == floor(config.low_demand_customer_ratio * config.num_customers)
    assert len(high) == config.num_customers - len(low)
    assert all(config.low_demand_weight_min <= instance.demand[customer] <= config.drone_payload for customer in low)
    assert all(instance.demand[customer] >= config.high_demand_weight_min for customer in high)
    radius = config.drone_speed * config.drone_endurance / 120.0
    assert radius == pytest.approx(10.0)
    for hub in instance.hubs:
        for customer in instance.customers:
            distance = hypot(
                instance.locations[hub][0] - instance.locations[customer][0],
                instance.locations[hub][1] - instance.locations[customer][1],
            )
            qualifies = distance <= radius + 1e-9 and instance.demand[customer] <= config.drone_payload
            assert ((hub, customer) in instance.drone_arcs) is qualifies


def test_drone_only_customers_keep_all_pads_and_lose_every_truck_arc() -> None:
    config = InstanceConfig(
        seed=4,
        num_trucks=5,
        num_customers=25,
        num_hubs=2,
        distribution="PC",
        drones_per_truck=4,
        mandatory_drone_customer_fraction=0.16,
    )
    instance = generate_candidate(config)
    assert len(instance.mandatory_drone_customers) == ceil(0.16 * len(instance.customers))
    for customer in instance.mandatory_drone_customers:
        assert not any(customer in arc for arc in instance.truck_arcs)
        qualifying_pads = {
            hub
            for hub in instance.hubs
            if hypot(
                instance.locations[hub][0] - instance.locations[customer][0],
                instance.locations[hub][1] - instance.locations[customer][1],
            ) <= 10.0 + 1e-9
        }
        retained_pads = {hub for hub in instance.hubs if (hub, customer) in instance.drone_arcs}
        assert retained_pads == qualifying_pads
        assert retained_pads


def test_instance_regeneration_uses_deterministic_realized_seeds(monkeypatch, tmp_path: Path) -> None:
    generated_seeds: list[int] = []

    def fake_candidate(config: InstanceConfig) -> InstanceData:
        generated_seeds.append(config.seed)
        base = tiny_instance()
        return replace(base, config=replace(base.config, seed=config.seed))

    statuses: list[tuple[bool, str]] = [(False, "unrepairable"), (True, "feasible_unchanged")]
    monkeypatch.setattr("thvrpd.instance.generate_candidate", fake_candidate)
    monkeypatch.setattr(
        "thvrpd.repair.discover_or_repair_instance",
        lambda instance: _fake_repair_result(instance, *statuses.pop(0)),
    )
    config = InstanceConfig(seed=17, num_trucks=2, num_customers=3, num_hubs=1, distribution="PS", drones_per_truck=2)
    instance = generate_instance(config)
    assert generated_seeds == [17, 1_000_020]
    assert instance.requested_seed == 17
    assert instance.config.seed == 1_000_020
    assert instance.generation_attempt == 1
    assert [record["status"] for record in instance.generation_feasibility_diagnostics] == [
        "unrepairable",
        "feasible_unchanged",
    ]
    snapshot = tmp_path / "generated.json"
    digest = write_instance_snapshot(instance, snapshot)
    loaded, _ = read_instance_snapshot(snapshot, expected_sha256=digest)
    assert loaded == instance
    assert not hasattr(instance, "witness_routes")


def test_instance_post_feasibility_acceptance_advances_deterministic_seed(monkeypatch) -> None:
    generated_seeds: list[int] = []
    screened: list[tuple[int, int]] = []

    def fake_candidate(config: InstanceConfig) -> InstanceData:
        generated_seeds.append(config.seed)
        base = tiny_instance()
        return replace(base, config=replace(base.config, seed=config.seed))

    def accept(instance: InstanceData, attempt: int, realized_seed: int) -> InstanceAcceptanceResult:
        screened.append((attempt, realized_seed))
        accepted = attempt == 1
        return InstanceAcceptanceResult(
            accepted=accepted,
            status="certified" if accepted else "rejected",
            diagnostics={"realized_seed": realized_seed},
        )

    monkeypatch.setattr("thvrpd.instance.generate_candidate", fake_candidate)
    monkeypatch.setattr(
        "thvrpd.repair.discover_or_repair_instance",
        lambda instance: _fake_repair_result(instance, True, "feasible_unchanged"),
    )
    config = InstanceConfig(seed=31, num_trucks=2, num_customers=3, num_hubs=1, distribution="PS", drones_per_truck=2)
    instance = generate_instance(config, post_feasibility_acceptance=accept)

    assert generated_seeds == [31, 1_000_034]
    assert screened == [(0, 31), (1, 1_000_034)]
    assert instance.config.seed == 1_000_034
    assert instance.generation_attempt == 1
    first, second = instance.generation_feasibility_diagnostics
    assert first["post_feasibility_acceptance"]["accepted"] is False
    assert first["post_feasibility_acceptance"]["status"] == "rejected"
    assert second["post_feasibility_acceptance"]["accepted"] is True
    assert second["post_feasibility_acceptance"]["status"] == "certified"


def test_instance_regeneration_advances_seed_after_geographic_overlap(monkeypatch) -> None:
    generated_seeds: list[int] = []

    def fake_candidate(config: InstanceConfig) -> InstanceData:
        generated_seeds.append(config.seed)
        if len(generated_seeds) == 1:
            raise GeographicOverlapError("geographic node overlap between C1 and H1")
        base = tiny_instance()
        return replace(base, config=replace(base.config, seed=config.seed))

    monkeypatch.setattr("thvrpd.instance.generate_candidate", fake_candidate)
    monkeypatch.setattr(
        "thvrpd.repair.discover_or_repair_instance",
        lambda instance: _fake_repair_result(instance, True, "feasible_unchanged"),
    )
    config = InstanceConfig(seed=23, num_trucks=2, num_customers=3, num_hubs=1, distribution="PS", drones_per_truck=2)
    instance = generate_instance(config)
    assert generated_seeds == [23, 1_000_026]
    assert instance.requested_seed == 23
    assert instance.config.seed == 1_000_026
    assert instance.generation_attempt == 1
    assert instance.generation_feasibility_diagnostics[0]["status"] == "candidate_rejected"


def test_instance_snapshot_round_trip_is_lossless_and_canonical(tmp_path: Path) -> None:
    instance = replace(
        tiny_instance(),
        requested_seed=17,
        generation_attempt=2,
        generation_feasibility_time=1.25,
        generation_feasibility_diagnostics=({"status": "feasible", "attempt": 2},),
        drone_arc_saving={("H1", "C1"): float("inf"), ("H1", "C2"): 0.5},
    )
    first_path = tmp_path / "first.json"
    second_path = tmp_path / "second.json"
    first_digest = write_instance_snapshot(instance, first_path)
    second_digest = write_instance_snapshot(instance, second_path)
    loaded, verified_digest = read_instance_snapshot(first_path, expected_sha256=first_digest)

    assert first_digest == second_digest == verified_digest
    assert loaded == instance
    assert json.loads(first_path.read_text())["schema_version"] == INSTANCE_SNAPSHOT_SCHEMA_VERSION
    with pytest.raises(FileExistsError, match="instance snapshot already exists"):
        write_instance_snapshot(instance, first_path)


def test_dense_multiscale_campaign_has_expected_16_case_order_and_manifest_contract(tmp_path: Path) -> None:
    cases = tuple(
        CampaignCase(scale, distribution, seed, dict(SCALES[scale]))
        for scale in SCALE_ORDER
        for distribution, seeds in DENSE_MULTISCALE_SCENARIOS
        for seed in seeds
    )
    expected_ids = [
        *(f"small_PS_seed{seed}" for seed in (1, 2, 3, 4)),
        *(f"small_PC_seed{seed}" for seed in (5, 6, 7, 8)),
        *(f"medium_PS_seed{seed}" for seed in (1, 2, 3, 4)),
        *(f"medium_PC_seed{seed}" for seed in (5, 6, 7, 8)),
    ]
    args = SimpleNamespace(
        time_limit=1800.0,
        threads=0,
        truck_arc_probability=0.10,
        hub_arc_probability=0.30,
    )
    manifest = {
        "case_order": expected_ids,
        "time_limit_seconds_per_run": 1800.0,
        "threads": 0,
        "truck_arc_probability": 0.10,
        "hub_arc_probability": 0.30,
    }

    assert len(cases) == 16
    assert [case.case_id for case in cases] == expected_ids
    validate_dense_multiscale_manifest(manifest, args, cases)
    with pytest.raises(ValueError, match="time limit"):
        validate_dense_multiscale_manifest({**manifest, "time_limit_seconds_per_run": 3600.0}, args, cases)


def test_dense_multiscale_large_artifact_guard_hashes_both_retained_directories(tmp_path: Path) -> None:
    for index, directory_name in enumerate(LARGE_RESULT_DIRS):
        directory = tmp_path / directory_name
        directory.mkdir()
        (directory / "artifact.txt").write_text(f"large-{index}", encoding="utf-8")

    hashes = _large_artifact_hashes(tmp_path)

    assert set(hashes) == {
        f"{directory_name}\\artifact.txt" for directory_name in LARGE_RESULT_DIRS
    }
    assert all(len(digest) == 64 for digest in hashes.values())


def test_medium_crossover_selects_scale_by_aggregate_runtime_solve_count_and_gap(tmp_path: Path) -> None:
    stage = tmp_path / "pilot"
    stage.mkdir()
    records = [
        {
            "bpc_runtime": 10.0,
            "compact_runtime": 20.0,
            "bpc_gap": 0.0,
            "compact_gap": 0.0,
            "bpc_status": "optimal",
            "compact_status_code": 2,
        }
        for _ in range(4)
    ]
    (stage / "stage_comparison.json").write_text(json.dumps({"cases": records}), encoding="utf-8")

    qualifying = _pilot_metrics(stage, PILOT_CONFIGS[0])
    assert qualifying["qualifies"]

    records[0]["bpc_gap"] = None
    records[1]["bpc_gap"] = None
    records[2]["bpc_gap"] = None
    (stage / "stage_comparison.json").write_text(json.dumps({"cases": records}), encoding="utf-8")
    unresolved = _pilot_metrics(stage, PILOT_CONFIGS[0])
    assert not unresolved["qualifies"]
    assert unresolved["bpc_median_gap"] is None


def test_instance_snapshot_rejects_schema_hash_case_and_geographic_mismatches(tmp_path: Path) -> None:
    instance = replace(tiny_instance(), requested_seed=1)
    snapshot_path = tmp_path / "instance.json"
    digest = write_instance_snapshot(instance, snapshot_path)

    with pytest.raises(ValueError, match="expected SHA-256"):
        read_instance_snapshot(snapshot_path, expected_sha256="0" * 64)
    with pytest.raises(ValueError, match="case mismatch"):
        validate_instance_case(
            instance,
            requested_seed=2,
            num_trucks=2,
            num_customers=3,
            num_hubs=1,
            distribution="PS",
            drones_per_truck=2,
        )
    with pytest.raises(ValueError, match="configuration mismatch"):
        validate_instance_case(
            instance,
            requested_seed=1,
            num_trucks=2,
            num_customers=3,
            num_hubs=1,
            distribution="PS",
            drones_per_truck=2,
            expected_config=replace(instance.config, truck_speed=instance.config.truck_speed + 1.0),
        )

    document = json.loads(snapshot_path.read_text())
    document["schema_version"] = INSTANCE_SNAPSHOT_SCHEMA_VERSION + 1
    snapshot_path.write_text(json.dumps(document), encoding="utf-8")
    with pytest.raises(ValueError, match="unsupported instance snapshot schema"):
        read_instance_snapshot(snapshot_path)

    snapshot_path.unlink()
    write_instance_snapshot(instance, snapshot_path)
    document = json.loads(snapshot_path.read_text())
    document["instance"]["demand"]["C1"] += 1.0
    snapshot_path.write_text(json.dumps(document), encoding="utf-8")
    with pytest.raises(ValueError, match="SHA-256 mismatch"):
        read_instance_snapshot(snapshot_path)

    snapshot_path.unlink()
    write_instance_snapshot(instance, snapshot_path)
    document = json.loads(snapshot_path.read_text())
    document["instance"]["locations"]["H1"] = document["instance"]["locations"]["C1"]
    payload = {
        "schema_version": document["schema_version"],
        "instance": document["instance"],
    }
    document["sha256"] = _instance_snapshot_digest(payload)
    snapshot_path.write_text(json.dumps(document), encoding="utf-8")
    with pytest.raises(GeographicOverlapError, match="geographic node overlap"):
        read_instance_snapshot(snapshot_path)


def test_campaign_reuses_immutable_frozen_snapshot(monkeypatch, tmp_path: Path) -> None:
    case = CampaignCase(
        "small",
        "PS",
        1,
        {"num_customers": 5, "num_trucks": 2, "num_hubs": 2, "drones_per_truck": 4},
    )
    calls = 0

    def fake_generate(config: InstanceConfig) -> InstanceData:
        nonlocal calls
        calls += 1
        return replace(generate_candidate(config), requested_seed=config.seed)

    monkeypatch.setattr(campaign_module, "generate_instance", fake_generate)
    first = _prepare_instance_snapshots(tmp_path, (case,))
    first_snapshot = first[case.case_id]
    assert calls == 1
    assert first_snapshot.path.exists()

    monkeypatch.setattr(
        campaign_module,
        "generate_instance",
        lambda config: pytest.fail("existing frozen snapshot must be reused"),
    )
    second = _prepare_instance_snapshots(tmp_path, (case,))
    assert second[case.case_id].sha256 == first_snapshot.sha256
    assert second[case.case_id].instance == first_snapshot.instance


def test_compact_feasibility_gate_accepts_a_feasible_candidate() -> None:
    feasible, status = compact_feasibility_status(tiny_instance())
    assert feasible
    assert status == "success"


def test_exact_prechecks_are_sound_and_report_named_witnesses() -> None:
    feasible = exact_feasibility_precheck(tiny_instance())
    assert feasible.passed
    assert feasible.witness is None

    forced_capacity = exact_feasibility_precheck(_forced_drone_capacity_instance())
    assert not forced_capacity.passed
    assert forced_capacity.witness == "forced_drone_pad_capacity:1<2"
    assert forced_capacity.forced_drone_customers == ("C1", "C2")

    instance = tiny_instance()
    objective = build_objective_data(instance, ObjectiveWeights(0.4, 0.3, 0.3))
    service_ub = dict(objective.bounds.service_ub)
    service_ub["C1"] = -1.0
    impossible_deadline = replace(objective, bounds=replace(objective.bounds, service_ub=service_ub))
    deadline = exact_feasibility_precheck(instance, impossible_deadline)
    assert not deadline.passed
    assert deadline.witness == "customer_deadline_reachability:C1"


def test_aggregated_gate_matches_legacy_model_and_reduces_variables() -> None:
    weights = ObjectiveWeights(1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0)
    for instance, expected_feasible in (
        (tiny_instance(), True),
        (_one_truck_disconnected_customers_instance(), False),
    ):
        objective = build_objective_data(instance, weights)
        aggregate = exact_feasibility_check(instance, objective=objective)
        legacy = solve_compact_solution(
            instance,
            weights,
            time_limit=60.0,
            threads=1,
            require_optimal=False,
            objective=objective,
            feasibility_only=True,
            solution_limit=1,
            dual_reductions=0,
            decode_routes=False,
        )
        assert aggregate.feasible is expected_feasible
        assert (legacy.objective_full is not None) is expected_feasible
        assert aggregate.diagnostics.variable_count < legacy_feasibility_variable_count(instance)
        assert aggregate.diagnostics.legacy_variable_count == legacy_feasibility_variable_count(instance)


def test_physical_fingerprint_ignores_generation_diagnostics() -> None:
    instance = tiny_instance()
    decorated = replace(
        instance,
        requested_seed=99,
        generation_attempt=4,
        generation_feasibility_time=12.5,
        generation_feasibility_diagnostics=({"status": "feasible"},),
    )
    assert instance_physical_fingerprint(instance) == instance_physical_fingerprint(decorated)


def test_dual_reductions_fallback_runs_only_for_inf_or_unbd(monkeypatch) -> None:
    class FakeModel:
        def __init__(self) -> None:
            self.Params = type("Params", (), {"DualReductions": 1})()
            self.SolCount = 0
            self.Status = GRB.INF_OR_UNBD
            self.NumVars = 10
            self.NumConstrs = 20
            self.NumNZs = 30

        def reset(self) -> None:
            self.SolCount = 0

    model = FakeModel()
    calls: list[int] = []

    def fake_stage(fake_model, dual_reductions: int) -> FeasibilitySolveStage:
        calls.append(dual_reductions)
        if dual_reductions == 1:
            fake_model.Status = GRB.INF_OR_UNBD
        else:
            assert fake_model.Params.DualReductions == 0
            fake_model.Status = GRB.INFEASIBLE
        return FeasibilitySolveStage(
            dual_reductions,
            "inf_or_unbd" if dual_reductions == 1 else "infeasible",
            int(fake_model.Status),
            0.0,
            0.0,
            0.0,
            0,
        )

    monkeypatch.setattr(feasibility_module, "_build_aggregated_feasibility_model", lambda *args, **kwargs: model)
    monkeypatch.setattr(feasibility_module, "_optimize_feasibility_stage", fake_stage)
    result = exact_feasibility_check(tiny_instance())
    assert not result.feasible
    assert result.status == "infeasible"
    assert result.diagnostics.fallback_used
    assert calls == [1, 0]


def test_unexpected_aggregate_feasibility_status_is_an_error(monkeypatch) -> None:
    class FakeModel:
        SolCount = 0
        Status = GRB.TIME_LIMIT
        NumVars = 10
        NumConstrs = 20
        NumNZs = 30

    model = FakeModel()
    monkeypatch.setattr(feasibility_module, "_build_aggregated_feasibility_model", lambda *args, **kwargs: model)
    monkeypatch.setattr(
        feasibility_module,
        "_optimize_feasibility_stage",
        lambda *args, **kwargs: FeasibilitySolveStage(1, "status_9", GRB.TIME_LIMIT, 0.0, 0.0, 0.0, 0),
    )
    with pytest.raises(RuntimeError, match="unexpected aggregate feasibility status"):
        exact_feasibility_check(tiny_instance())


def test_arc_swap_repair_settings_are_part_of_generic_defaults() -> None:
    defaults = case_defaults(num_customers=30, num_trucks=6, num_hubs=3, drones_per_truck=4)
    assert defaults["generation_feasibility_discovery_limit"] == 60.0
    assert defaults["generation_repair_mode"] == "arc_swap"
    with pytest.raises(ValueError, match="discovery limit"):
        replace(tiny_instance().config, generation_feasibility_discovery_limit=0.0)
    with pytest.raises(ValueError, match="repair mode"):
        replace(tiny_instance().config, generation_repair_mode="unknown")


def test_easy_feasible_candidate_remains_unchanged() -> None:
    instance = tiny_instance()
    result = discover_or_repair_instance(instance)
    assert result.feasible
    assert result.status == "feasible_unchanged"
    assert result.instance == instance
    assert not result.diagnostics.repair_invoked
    assert result.diagnostics.hamming_edit_count == 0
    assert result.diagnostics.original_graph_fingerprint == result.diagnostics.repaired_graph_fingerprint


def test_arc_swap_repairs_infeasible_graph_and_preserves_required_data() -> None:
    original = _repairable_mandatory_drone_instance()
    assert not exact_feasibility_check(original).feasible
    result = discover_or_repair_instance(original)
    assert result.feasible
    assert result.status == "repaired"
    assert result.instance is not None
    repaired = result.instance
    diagnostics = result.diagnostics

    assert diagnostics.repair_invoked
    assert diagnostics.projection_type == "strengthened_arc"
    assert diagnostics.strengthened_arc_projection is not None
    assert diagnostics.strengthened_arc_projection.solve_time_seconds >= 0.0
    assert diagnostics.certificate_validation.valid
    assert diagnostics.certificate_validation.issues == ()
    assert diagnostics.certificate_routes
    assert diagnostics.rejected_pattern_count == 0
    assert diagnostics.hamming_edit_count == len(diagnostics.added_arcs) + len(diagnostics.removed_arcs)
    assert diagnostics.hamming_edit_count > 0
    assert diagnostics.original_arc_counts == diagnostics.repaired_arc_counts
    assert truck_arc_class_counts(original, original.truck_arcs) == truck_arc_class_counts(repaired, repaired.truck_arcs)

    original_fixed = {
        arc for arc in original.truck_arcs if arc[0] == original.depot_source or arc[1] == original.depot_sink
    }
    repaired_fixed = {
        arc for arc in repaired.truck_arcs if arc[0] == repaired.depot_source or arc[1] == repaired.depot_sink
    }
    assert repaired_fixed == original_fixed
    assert not any("C4" in arc for arc in repaired.truck_arcs)
    assert set(diagnostics.added_arcs).issubset(allowable_internal_truck_arcs(original))
    assert set(diagnostics.removed_arcs).issubset(allowable_internal_truck_arcs(original))
    assert repaired.locations == original.locations
    assert repaired.demand == original.demand
    assert repaired.drone_arcs == original.drone_arcs
    assert repaired.drone_time == original.drone_time
    assert repaired.drone_trip_time == original.drone_trip_time
    assert repaired.mandatory_drone_customers == original.mandatory_drone_customers
    assert diagnostics.original_graph_fingerprint != diagnostics.repaired_graph_fingerprint
    assert exact_feasibility_check(repaired).feasible

    covered = sorted(
        customer
        for route in diagnostics.certificate_routes
        for customer in route["served"]
    )
    assert covered == sorted(original.customers)
    assert any("C4" in route["drone_served"] for route in diagnostics.certificate_routes)
    for hub, customer in repaired.drone_arcs:
        assert repaired.demand[customer] <= repaired.drone_payload
        assert repaired.drone_trip_time[(hub, customer)] <= repaired.drone_endurance


def test_strengthened_arc_projection_reference_preserves_small_feasibility() -> None:
    original = _repairable_mandatory_drone_instance()
    result = solve_strengthened_arc_projection_reference(original)
    assert result.feasible
    assert result.status == "feasible"
    assert result.routes
    assert result.diagnostics.variable_count > 0
    assert result.diagnostics.constraint_count > 0
    assert result.diagnostics.eligible_route_arc_count > 0
    assert result.diagnostics.pruned_route_arc_count >= 0


def test_arc_swap_repair_is_deterministic_for_same_candidate() -> None:
    original = _repairable_mandatory_drone_instance()
    first = discover_or_repair_instance(original)
    second = discover_or_repair_instance(original)
    assert first.instance is not None and second.instance is not None
    assert first.instance.truck_arcs == second.instance.truck_arcs
    assert first.diagnostics.added_arcs == second.diagnostics.added_arcs
    assert first.diagnostics.removed_arcs == second.diagnostics.removed_arcs
    assert first.diagnostics.certificate_routes == second.diagnostics.certificate_routes


def test_unresolved_discovery_invokes_repair(monkeypatch) -> None:
    original = _repairable_mandatory_drone_instance()
    unresolved = _fake_gate_result(False, "unresolved")
    monkeypatch.setattr(repair_module, "exact_feasibility_check", lambda *args, **kwargs: unresolved)
    result = discover_or_repair_instance(original)
    assert result.feasible
    assert result.status == "repaired"
    assert result.diagnostics.original_discovery_status == "unresolved"
    assert result.diagnostics.repair_invoked


def test_repair_adds_no_good_after_rejected_certificate(monkeypatch) -> None:
    original = _repairable_mandatory_drone_instance()
    actual_validate = repair_module._validate_projected_routes
    calls = 0

    def reject_first(source, repaired, routes) -> RepairValidation:
        nonlocal calls
        calls += 1
        if calls == 1:
            return RepairValidation(False, ("deliberate_test_rejection",), tuple())
        return actual_validate(source, repaired, routes)

    monkeypatch.setattr(repair_module, "_validate_projected_routes", reject_first)
    result = discover_or_repair_instance(original)
    assert result.feasible
    assert result.diagnostics.rejected_pattern_count == 1
    assert result.diagnostics.certificate_validation.valid


def test_audit_pattern_signature_ignores_route_order() -> None:
    instance = tiny_instance()
    objective = build_objective_data(instance, ObjectiveWeights(0.4, 0.3, 0.3))
    graph = build_transformed_graph(instance)
    first = route_from_path(1, ("Source", "C1", "Sink"), graph, objective)
    second = route_from_path(2, ("Source", "C2", "C3", "Sink"), graph, objective)
    left = audit_module._pattern_record((first, second), objective)
    right = audit_module._pattern_record((second, first), objective)
    assert left["signature"] == right["signature"]


def test_audit_binary_no_good_finds_distinct_small_pattern() -> None:
    base = tiny_instance()
    direct_depot_arc = (base.depot_source, base.depot_sink)
    instance = replace(
        base,
        truck_arcs=frozenset(set(base.truck_arcs) - {direct_depot_arc}),
        truck_time={arc: value for arc, value in base.truck_time.items() if arc != direct_depot_arc},
    )
    objective = build_objective_data(instance, ObjectiveWeights(0.4, 0.3, 0.3))
    artifacts = repair_module._build_arc_swap_repair_model(
        instance,
        instance,
        objective,
        route_search=True,
        add_pair_incompatibilities=False,
    )
    artifacts.model.optimize()
    first = audit_module._pattern_record(
        repair_module._decode_repair_routes(instance, objective, artifacts),
        objective,
    )
    audit_module._add_binary_no_good(artifacts)
    artifacts.model.reset()
    artifacts.model.optimize()
    second = audit_module._pattern_record(
        repair_module._decode_repair_routes(instance, objective, artifacts),
        objective,
    )
    assert first["signature"] != second["signature"]


def test_audit_timeout_never_implies_uniqueness() -> None:
    classification = audit_module.classify_case(
        compact_optimal=True,
        compact_nodes=0.0,
        compact_solve_seconds=0.01,
        pattern_count=1,
        diversity_status="timeout",
        pattern_limit=20,
    )
    assert classification == "uniqueness_unresolved"
