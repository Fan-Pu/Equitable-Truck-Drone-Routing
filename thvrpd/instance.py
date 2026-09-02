from __future__ import annotations

from dataclasses import asdict, dataclass, field, replace
from hashlib import sha256
import json
from math import ceil, hypot, inf, isclose, isfinite
from pathlib import Path
import random
import time
from typing import Callable

import networkx as nx
import numpy as np

from .config import InstanceConfig


Node = str
Arc = tuple[Node, Node]
CAPACITY_TOLERANCE = 1e-9
LOCATION_OVERLAP_TOLERANCE = 1e-9
INSTANCE_SNAPSHOT_SCHEMA_VERSION = 1


class GeographicOverlapError(ValueError):
    pass


@dataclass(frozen=True)
class InstanceAcceptanceResult:
    accepted: bool
    status: str
    diagnostics: dict[str, object]


def write_instance_snapshot(instance: "InstanceData", path: Path) -> str:
    if path.exists():
        raise FileExistsError(f"instance snapshot already exists: {path}")
    payload = _instance_snapshot_payload(instance)
    digest = _instance_snapshot_digest(payload)
    document = {**payload, "sha256": digest}
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(document, sort_keys=True, indent=2, allow_nan=False), encoding="utf-8")
    return digest


def read_instance_snapshot(path: Path, expected_sha256: str | None = None) -> tuple["InstanceData", str]:
    document = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError("instance snapshot must contain a JSON object")
    schema_version = document.get("schema_version")
    if schema_version != INSTANCE_SNAPSHOT_SCHEMA_VERSION:
        raise ValueError(f"unsupported instance snapshot schema version: {schema_version}")
    if not isinstance(document.get("instance"), dict):
        raise ValueError("instance snapshot is missing the instance payload")
    recorded_digest = document.get("sha256")
    if not isinstance(recorded_digest, str):
        raise ValueError("instance snapshot is missing its SHA-256 digest")
    payload = {
        "schema_version": schema_version,
        "instance": document["instance"],
    }
    actual_digest = _instance_snapshot_digest(payload)
    if recorded_digest != actual_digest:
        raise ValueError(f"instance snapshot SHA-256 mismatch: {recorded_digest} != {actual_digest}")
    if expected_sha256 is not None and actual_digest != expected_sha256:
        raise ValueError(f"instance snapshot does not match expected SHA-256: {actual_digest} != {expected_sha256}")
    return _instance_from_snapshot_record(document["instance"]), actual_digest


def validate_instance_case(
    instance: "InstanceData",
    *,
    requested_seed: int,
    num_trucks: int,
    num_customers: int,
    num_hubs: int,
    distribution: str,
    drones_per_truck: int,
    expected_config: InstanceConfig | None = None,
) -> None:
    actual_requested_seed = instance.requested_seed if instance.requested_seed is not None else instance.config.seed
    expected = (
        requested_seed,
        num_trucks,
        num_customers,
        num_hubs,
        distribution,
        drones_per_truck,
    )
    actual = (
        actual_requested_seed,
        instance.num_trucks,
        len(instance.customers),
        len(instance.hubs),
        instance.config.distribution,
        instance.drones_per_truck,
    )
    if actual != expected:
        raise ValueError(f"instance snapshot case mismatch: expected {expected}, found {actual}")
    if expected_config is not None:
        expected_config_record = asdict(expected_config)
        actual_config_record = asdict(instance.config)
        expected_config_record.pop("seed")
        actual_config_record.pop("seed")
        if actual_config_record != expected_config_record:
            raise ValueError(
                f"instance snapshot configuration mismatch: expected {expected_config_record}, "
                f"found {actual_config_record}"
            )


def _instance_snapshot_payload(instance: "InstanceData") -> dict[str, object]:
    return {
        "schema_version": INSTANCE_SNAPSHOT_SCHEMA_VERSION,
        "instance": {
            "config": asdict(instance.config),
            "depot_source": instance.depot_source,
            "depot_sink": instance.depot_sink,
            "customers": list(instance.customers),
            "hubs": list(instance.hubs),
            "nodes": list(instance.nodes),
            "truck_arcs": [list(arc) for arc in sorted(instance.truck_arcs)],
            "drone_arcs": [list(arc) for arc in sorted(instance.drone_arcs)],
            "truck_time": _arc_value_records(instance.truck_time),
            "drone_time": _arc_value_records(instance.drone_time),
            "drone_trip_time": _arc_value_records(instance.drone_trip_time),
            "demand": {node: value for node, value in sorted(instance.demand.items())},
            "locations": {node: list(value) for node, value in sorted(instance.locations.items())},
            "mandatory_drone_customers": list(instance.mandatory_drone_customers),
            "drone_arc_saving": [
                {"arc": list(arc), "value": _encode_snapshot_float(value)}
                for arc, value in sorted(instance.drone_arc_saving.items())
            ],
            "requested_seed": instance.requested_seed,
            "generation_attempt": instance.generation_attempt,
            "generation_feasibility_time": instance.generation_feasibility_time,
            "generation_feasibility_diagnostics": list(instance.generation_feasibility_diagnostics),
        },
    }


def _instance_snapshot_digest(payload: dict[str, object]) -> str:
    canonical = json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return sha256(canonical.encode("utf-8")).hexdigest()


def instance_physical_fingerprint(instance: "InstanceData") -> str:
    record = dict(_instance_snapshot_payload(instance)["instance"])
    for field_name in (
        "requested_seed",
        "generation_attempt",
        "generation_feasibility_time",
        "generation_feasibility_diagnostics",
    ):
        record.pop(field_name, None)
    canonical = json.dumps(record, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return sha256(canonical.encode("utf-8")).hexdigest()


def _arc_value_records(values: dict[Arc, float]) -> list[dict[str, object]]:
    return [
        {"arc": list(arc), "value": _encode_snapshot_float(value)}
        for arc, value in sorted(values.items())
    ]


def _encode_snapshot_float(value: float) -> float | str:
    if value == inf:
        return "Infinity"
    if value == -inf:
        return "-Infinity"
    if not isfinite(value):
        raise ValueError("instance snapshot cannot encode NaN")
    return float(value)


def _decode_snapshot_float(value: object) -> float:
    if value == "Infinity":
        return inf
    if value == "-Infinity":
        return -inf
    if not isinstance(value, (int, float)):
        raise ValueError(f"invalid numeric value in instance snapshot: {value}")
    result = float(value)
    if not isfinite(result):
        raise ValueError(f"nonfinite numeric value in instance snapshot: {value}")
    return result


def _arc_values_from_records(records: object) -> dict[Arc, float]:
    if not isinstance(records, list):
        raise ValueError("instance snapshot arc values must be a list")
    values: dict[Arc, float] = {}
    for record in records:
        if not isinstance(record, dict) or not isinstance(record.get("arc"), list) or len(record["arc"]) != 2:
            raise ValueError("invalid arc-value record in instance snapshot")
        arc = (str(record["arc"][0]), str(record["arc"][1]))
        if arc in values:
            raise ValueError(f"duplicate arc-value record in instance snapshot: {arc}")
        values[arc] = _decode_snapshot_float(record.get("value"))
    return values


def _instance_from_snapshot_record(record: dict[str, object]) -> "InstanceData":
    config_record = record.get("config")
    if not isinstance(config_record, dict):
        raise ValueError("instance snapshot is missing InstanceConfig")
    config = InstanceConfig(**config_record)
    drone_saving_records = record.get("drone_arc_saving")
    if not isinstance(drone_saving_records, list):
        raise ValueError("instance snapshot drone_arc_saving must be a list")
    drone_arc_saving: dict[Arc, float] = {}
    for saving_record in drone_saving_records:
        if (
            not isinstance(saving_record, dict)
            or not isinstance(saving_record.get("arc"), list)
            or len(saving_record["arc"]) != 2
        ):
            raise ValueError("invalid drone-arc-saving record in instance snapshot")
        arc = (str(saving_record["arc"][0]), str(saving_record["arc"][1]))
        if arc in drone_arc_saving:
            raise ValueError(f"duplicate drone-arc-saving record in instance snapshot: {arc}")
        drone_arc_saving[arc] = _decode_snapshot_float(saving_record.get("value"))
    locations_record = record.get("locations")
    demand_record = record.get("demand")
    if not isinstance(locations_record, dict) or not isinstance(demand_record, dict):
        raise ValueError("instance snapshot is missing locations or demand")
    return InstanceData(
        config=config,
        depot_source=str(record["depot_source"]),
        depot_sink=str(record["depot_sink"]),
        customers=tuple(map(str, record["customers"])),
        hubs=tuple(map(str, record["hubs"])),
        nodes=tuple(map(str, record["nodes"])),
        truck_arcs=frozenset((str(arc[0]), str(arc[1])) for arc in record["truck_arcs"]),
        drone_arcs=frozenset((str(arc[0]), str(arc[1])) for arc in record["drone_arcs"]),
        truck_time=_arc_values_from_records(record["truck_time"]),
        drone_time=_arc_values_from_records(record["drone_time"]),
        drone_trip_time=_arc_values_from_records(record["drone_trip_time"]),
        demand={str(node): _decode_snapshot_float(value) for node, value in demand_record.items()},
        locations={
            str(node): (_decode_snapshot_float(value[0]), _decode_snapshot_float(value[1]))
            for node, value in locations_record.items()
        },
        mandatory_drone_customers=tuple(map(str, record.get("mandatory_drone_customers", []))),
        drone_arc_saving=drone_arc_saving,
        requested_seed=None if record.get("requested_seed") is None else int(record["requested_seed"]),
        generation_attempt=int(record.get("generation_attempt", 0)),
        generation_feasibility_time=float(record.get("generation_feasibility_time", 0.0)),
        generation_feasibility_diagnostics=tuple(record.get("generation_feasibility_diagnostics", [])),
    )


@dataclass(frozen=True)
class InstanceData:
    config: InstanceConfig
    depot_source: Node
    depot_sink: Node
    customers: tuple[Node, ...]
    hubs: tuple[Node, ...]
    nodes: tuple[Node, ...]
    truck_arcs: frozenset[Arc]
    drone_arcs: frozenset[Arc]
    truck_time: dict[Arc, float]
    drone_time: dict[Arc, float]
    drone_trip_time: dict[Arc, float]
    demand: dict[Node, float]
    locations: dict[Node, tuple[float, float]]
    mandatory_drone_customers: tuple[Node, ...] = ()
    drone_arc_saving: dict[Arc, float] = field(default_factory=dict)
    requested_seed: int | None = None
    generation_attempt: int = 0
    generation_feasibility_time: float = 0.0
    generation_feasibility_diagnostics: tuple[dict[str, object], ...] = ()

    def __post_init__(self) -> None:
        physical = set(self.customers) | set(self.hubs)
        if self.depot_source == self.depot_sink:
            raise ValueError("source and sink depot copies must be distinct nodes")
        if set(self.customers).intersection(self.hubs):
            raise ValueError("customer and synchronization-pad sets must be disjoint")
        if self.depot_source in physical or self.depot_sink in physical:
            raise ValueError("depot copies must be disjoint from customers and pads")
        if set(self.nodes) != {self.depot_source, self.depot_sink} | physical:
            raise ValueError("node set must equal depot copies, customers, and pads")
        self._validate_geographic_locations()
        if self.truck_payload <= 0.0 or self.drone_payload <= 0.0 or self.drone_endurance <= 0.0:
            raise ValueError("truck payload, drone payload, and drone endurance must be positive")
        if self.truck_cost < 0.0 or self.drone_cost < 0.0:
            raise ValueError("fixed truck and drone costs must be nonnegative")
        for node in self.nodes:
            if node not in self.demand:
                raise ValueError(f"missing demand for node {node}")
            if not isfinite(self.demand[node]) or self.demand[node] < 0.0:
                raise ValueError(f"node demand must be finite and nonnegative: {node}")
        for customer in self.customers:
            if self.demand[customer] <= 0.0:
                raise ValueError(f"customer parcel weight must be strictly positive: {customer}")
            if self.demand[customer] > self.truck_payload + CAPACITY_TOLERANCE:
                raise ValueError(f"customer parcel weight exceeds truck payload: {customer}")
        total_customer_demand = sum(self.demand[customer] for customer in self.customers)
        total_truck_payload = self.num_trucks * self.truck_payload
        if total_customer_demand > total_truck_payload + CAPACITY_TOLERANCE:
            raise ValueError(
                f"aggregate customer demand exceeds total truck payload: {total_customer_demand} > {total_truck_payload}"
            )
        for i, j in self.truck_arcs:
            if i not in self.nodes or j not in self.nodes:
                raise ValueError(f"truck arc uses unknown node: {(i, j)}")
            if j == self.depot_source or i == self.depot_sink:
                raise ValueError(f"truck arcs may not enter Source or leave Sink: {(i, j)}")
            if (i, j) not in self.truck_time:
                raise ValueError(f"missing truck travel time for {(i, j)}")
            if not isfinite(self.truck_time[(i, j)]) or self.truck_time[(i, j)] < 0.0:
                raise ValueError(f"truck travel time must be finite and nonnegative: {(i, j)}")
        for h, customer in self.drone_arcs:
            if h not in self.hubs or customer not in self.customers:
                raise ValueError(f"drone sortie arc must be pad-to-customer: {(h, customer)}")
            if (h, customer) not in self.drone_time or (customer, h) not in self.drone_time:
                raise ValueError(f"missing one-way drone times for {(h, customer)}")
            if (h, customer) not in self.drone_trip_time:
                raise ValueError(f"missing round-trip drone time for {(h, customer)}")
            outbound = self.drone_time[(h, customer)]
            inbound = self.drone_time[(customer, h)]
            trip = self.drone_trip_time[(h, customer)]
            if not isfinite(outbound) or not isfinite(inbound) or min(outbound, inbound) < 0.0:
                raise ValueError(f"one-way drone times must be finite and nonnegative: {(h, customer)}")
            if not isfinite(trip) or trip < 0.0:
                raise ValueError(f"round-trip drone time must be finite and nonnegative: {(h, customer)}")
            if not isclose(trip, outbound + inbound, rel_tol=1e-9, abs_tol=1e-9):
                raise ValueError(f"round-trip drone time must equal outbound plus return time: {(h, customer)}")
            if self.demand[customer] > self.drone_payload or self.drone_trip_time[(h, customer)] > self.drone_endurance:
                raise ValueError(f"infeasible drone sortie retained in drone arc set: {(h, customer)}")
        mandatory = set(self.mandatory_drone_customers)
        if not mandatory.issubset(self.customers):
            raise ValueError("mandatory drone customers must be original customers")
        if len(mandatory) != len(self.mandatory_drone_customers):
            raise ValueError("mandatory drone customers must be unique")
        self._validate_customer_service_reachability()
        if mandatory:
            graph = self.truck_graph()
            for customer in self.mandatory_drone_customers:
                truck_reachable = nx.has_path(graph, self.depot_source, customer) and nx.has_path(graph, customer, self.depot_sink)
                if truck_reachable:
                    raise ValueError(f"mandatory drone customer retains truck-service representation: {customer}")
                if not any((hub, customer) in self.drone_arcs for hub in self.hubs):
                    raise ValueError(f"mandatory drone customer has no retained drone arc: {customer}")

    def _validate_geographic_locations(self) -> None:
        for node in self.nodes:
            if node not in self.locations:
                raise ValueError(f"missing geographic location for node {node}")
            coordinate = self.locations[node]
            if not isinstance(coordinate, (tuple, list)) or len(coordinate) != 2:
                raise ValueError(f"node location must be a two-dimensional coordinate: {node}")
            if not all(isfinite(float(value)) for value in coordinate):
                raise ValueError(f"node location must be finite: {node}")

        source_location = self.locations[self.depot_source]
        sink_location = self.locations[self.depot_sink]
        if hypot(
            source_location[0] - sink_location[0],
            source_location[1] - sink_location[1],
        ) > LOCATION_OVERLAP_TOLERANCE:
            raise ValueError("source and sink depot copies must be geographically colocated")

        physical_sites = (self.depot_source, *self.customers, *self.hubs)
        for index, left in enumerate(physical_sites):
            left_location = self.locations[left]
            for right in physical_sites[index + 1 :]:
                right_location = self.locations[right]
                if hypot(
                    left_location[0] - right_location[0],
                    left_location[1] - right_location[1],
                ) <= LOCATION_OVERLAP_TOLERANCE:
                    raise GeographicOverlapError(
                        f"geographic node overlap between {left} at {left_location} "
                        f"and {right} at {right_location}"
                    )

    @property
    def truck_payload(self) -> float:
        return self.config.truck_payload

    @property
    def drone_payload(self) -> float:
        return self.config.drone_payload

    @property
    def drone_endurance(self) -> float:
        return self.config.drone_endurance

    @property
    def truck_cost(self) -> float:
        return self.config.truck_cost

    @property
    def drone_cost(self) -> float:
        return self.config.drone_cost

    @property
    def drones_per_truck(self) -> int:
        return self.config.drones_per_truck

    @property
    def num_trucks(self) -> int:
        return self.config.num_trucks

    def truck_graph(self) -> nx.DiGraph:
        graph = nx.DiGraph()
        graph.add_nodes_from(self.nodes)
        for i, j in self.truck_arcs:
            graph.add_edge(i, j, weight=self.truck_time[(i, j)])
        return graph

    def _validate_customer_service_reachability(self) -> None:
        graph = self.truck_graph()
        for customer in self.customers:
            truck_reachable = nx.has_path(graph, self.depot_source, customer) and nx.has_path(graph, customer, self.depot_sink)
            if truck_reachable:
                continue
            graph_without_customer = graph.copy()
            graph_without_customer.remove_node(customer)
            drone_reachable = any(
                (hub, customer) in self.drone_arcs
                and nx.has_path(graph_without_customer, self.depot_source, hub)
                and nx.has_path(graph_without_customer, hub, self.depot_sink)
                for hub in self.hubs
            )
            if not drone_reachable:
                raise ValueError(f"customer has no feasible service representation: {customer}")


def generate_instance(
    config: InstanceConfig,
    *,
    post_feasibility_acceptance: Callable[
        [InstanceData, int, int], InstanceAcceptanceResult
    ]
    | None = None,
    start_attempt: int = 0,
    prior_feasibility_time: float = 0.0,
    prior_feasibility_diagnostics: tuple[dict[str, object], ...] = (),
) -> InstanceData:
    requested_seed = config.seed
    attempt = start_attempt
    feasibility_time = prior_feasibility_time
    feasibility_diagnostics = list(prior_feasibility_diagnostics)
    while True:
        realized_seed = requested_seed + attempt * 1_000_003
        candidate_config = replace(config, seed=realized_seed)
        try:
            candidate = generate_candidate(candidate_config)
        except GeographicOverlapError as exc:
            feasibility_diagnostics.append(
                {
                    "attempt": attempt,
                    "realized_seed": realized_seed,
                    "status": "candidate_rejected",
                    "witness": str(exc),
                }
            )
            attempt += 1
            continue
        except ValueError as exc:
            rejection = str(exc)
            if not (
                rejection.startswith("not enough drone-feasible customers")
                or rejection.startswith("customer has no feasible service representation")
                or rejection.startswith("aggregate customer demand exceeds total truck payload")
            ):
                raise
            feasibility_diagnostics.append(
                {
                    "attempt": attempt,
                    "realized_seed": realized_seed,
                    "status": "candidate_rejected",
                    "witness": rejection,
                }
            )
            attempt += 1
            continue
        from .repair import discover_or_repair_instance

        check_start = time.time()
        result = discover_or_repair_instance(candidate)
        record = {
            "attempt": attempt,
            "realized_seed": realized_seed,
            "status": result.status,
            "feasible": result.feasible,
            "diagnostics": json.loads(
                json.dumps(asdict(result.diagnostics), allow_nan=False)
            ),
        }
        acceptance = None
        if result.feasible and post_feasibility_acceptance is not None:
            if result.instance is None:
                raise RuntimeError("feasible generation result is missing its instance")
            acceptance = post_feasibility_acceptance(result.instance, attempt, realized_seed)
            record["post_feasibility_acceptance"] = json.loads(
                json.dumps(asdict(acceptance), allow_nan=False)
            )
        attempt_time = time.time() - check_start
        feasibility_time += attempt_time
        record["elapsed_seconds"] = attempt_time
        feasibility_diagnostics.append(record)
        if result.feasible:
            if result.instance is None:
                raise RuntimeError("feasible generation result is missing its instance")
            if acceptance is not None and not acceptance.accepted:
                attempt += 1
                continue
            return replace(
                result.instance,
                requested_seed=requested_seed,
                generation_attempt=attempt,
                generation_feasibility_time=feasibility_time,
                generation_feasibility_diagnostics=tuple(feasibility_diagnostics),
            )
        if result.status not in {"infeasible_precheck", "unrepairable"}:
            raise RuntimeError(f"unexpected generation feasibility status {result.status}")
        attempt += 1


def generate_candidate(config: InstanceConfig) -> InstanceData:
    rng = random.Random(config.seed)
    np_rng = np.random.default_rng(config.seed)
    depot_source = "Source"
    depot_sink = "Sink"
    customers = tuple(f"C{i + 1}" for i in range(config.num_customers))
    hubs = tuple(f"H{i + 1}" for i in range(config.num_hubs))

    customer_locations = _sample_customers(config, np_rng)
    locations: dict[str, tuple[float, float]] = {
        customer: tuple(map(float, customer_locations[i]))
        for i, customer in enumerate(customers)
    }
    locations[depot_source] = (0.0, 0.0)
    locations[depot_sink] = locations[depot_source]
    if hubs:
        hub_locations = _choose_hubs_by_kmeans(config.seed, customer_locations, len(hubs))
        for i, hub in enumerate(hubs):
            locations[hub] = tuple(map(float, hub_locations[i]))

    nodes = (depot_source,) + customers + hubs + (depot_sink,)
    truck_arcs: set[Arc] = set()
    truck_time: dict[Arc, float] = {}
    drone_arcs: set[Arc] = set()
    drone_time: dict[Arc, float] = {}
    drone_trip_time: dict[Arc, float] = {}

    for node in customers + hubs:
        _add_truck_arc(config, locations, depot_source, node, truck_arcs, truck_time)
        _add_truck_arc(config, locations, node, depot_sink, truck_arcs, truck_time)

    physical = customers + hubs
    for i in physical:
        for j in physical:
            if i == j:
                continue
            if i in hubs or j in hubs:
                if (i in customers or j in customers) and rng.random() <= config.hub_arc_probability:
                    _add_truck_arc(config, locations, i, j, truck_arcs, truck_time)
            elif rng.random() <= config.truck_arc_probability:
                _add_truck_arc(config, locations, i, j, truck_arcs, truck_time)

    demand = _sample_demands(config, customers, rng, np_rng)
    demand[depot_source] = 0.0
    demand[depot_sink] = 0.0
    for hub in hubs:
        demand[hub] = 0.0

    for hub in hubs:
        for customer in customers:
            one_way = _drone_time(config, locations[hub], locations[customer])
            trip = 2.0 * one_way
            if demand[customer] <= config.drone_payload and trip <= config.drone_endurance:
                drone_arcs.add((hub, customer))
                drone_time[(hub, customer)] = one_way
                drone_time[(customer, hub)] = one_way
                drone_trip_time[(hub, customer)] = trip

    (
        truck_arcs,
        truck_time,
        drone_arcs,
        drone_time,
        drone_trip_time,
        mandatory_drone_customers,
        drone_arc_saving,
    ) = _apply_drone_required_graph_policy(
        config=config,
        depot_source=depot_source,
        depot_sink=depot_sink,
        customers=customers,
        hubs=hubs,
        locations=locations,
        demand=demand,
        truck_arcs=truck_arcs,
        truck_time=truck_time,
        drone_arcs=drone_arcs,
        drone_time=drone_time,
        drone_trip_time=drone_trip_time,
    )

    return InstanceData(
        config=config,
        depot_source=depot_source,
        depot_sink=depot_sink,
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
        mandatory_drone_customers=mandatory_drone_customers,
        drone_arc_saving=drone_arc_saving,
    )


def tiny_instance() -> InstanceData:
    config = InstanceConfig(
        seed=1,
        num_trucks=2,
        num_customers=3,
        distribution="PS",
        drones_per_truck=2,
        num_hubs=1,
        truck_arc_probability=1.0,
        hub_arc_probability=1.0,
        truck_payload=20.0,
        drone_payload=5.0,
        drone_endurance=100.0,
    )
    depot_source = "Source"
    depot_sink = "Sink"
    customers = ("C1", "C2", "C3")
    hubs = ("H1",)
    nodes = (depot_source,) + customers + hubs + (depot_sink,)
    locations = {
        depot_source: (0.0, 0.0),
        depot_sink: (0.0, 0.0),
        "C1": (1.0, 0.0),
        "C2": (2.0, 0.0),
        "C3": (1.0, 2.0),
        "H1": (1.0, 1.0),
    }
    truck_arcs: set[Arc] = set()
    truck_time: dict[Arc, float] = {}
    for i in nodes:
        for j in nodes:
            if i == j or j == depot_source or i == depot_sink:
                continue
            if i == depot_source or j == depot_sink or i in customers + hubs:
                _add_truck_arc(config, locations, i, j, truck_arcs, truck_time)
    demand = {depot_source: 0.0, depot_sink: 0.0, "H1": 0.0, "C1": 2.0, "C2": 2.0, "C3": 2.0}
    drone_arcs: set[Arc] = set()
    drone_time: dict[Arc, float] = {}
    drone_trip_time: dict[Arc, float] = {}
    for customer in customers:
        one_way = _drone_time(config, locations["H1"], locations[customer])
        drone_arcs.add(("H1", customer))
        drone_time[("H1", customer)] = one_way
        drone_time[(customer, "H1")] = one_way
        drone_trip_time[("H1", customer)] = 2.0 * one_way
    return InstanceData(
        config=config,
        depot_source=depot_source,
        depot_sink=depot_sink,
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


def _sample_customers(config: InstanceConfig, np_rng: np.random.Generator) -> np.ndarray:
    if config.distribution == "PS":
        return np_rng.uniform(0.0, config.area_side, size=(config.num_customers, 2))
    if config.distribution == "PC":
        k = max(1, config.num_customers // 5)
        centers = np_rng.uniform(0.0, config.area_side, size=(k, 2))
        points = []
        for i in range(config.num_customers):
            while True:
                point = np_rng.normal(loc=centers[i % k], scale=0.7, size=2)
                if np.all((0.0 <= point) & (point <= config.area_side)):
                    points.append(point)
                    break
        return np.vstack(points)
    n_cluster = config.num_customers // 2
    sparse_config = InstanceConfig(**{**config.__dict__, "num_customers": config.num_customers - n_cluster, "distribution": "PS"})
    cluster_config = InstanceConfig(**{**config.__dict__, "num_customers": n_cluster, "distribution": "PC"})
    return np.vstack((_sample_customers(sparse_config, np_rng), _sample_customers(cluster_config, np_rng)))


def _apply_drone_required_graph_policy(
    *,
    config: InstanceConfig,
    depot_source: Node,
    depot_sink: Node,
    customers: tuple[Node, ...],
    hubs: tuple[Node, ...],
    locations: dict[Node, tuple[float, float]],
    demand: dict[Node, float],
    truck_arcs: set[Arc],
    truck_time: dict[Arc, float],
    drone_arcs: set[Arc],
    drone_time: dict[Arc, float],
    drone_trip_time: dict[Arc, float],
) -> tuple[
    set[Arc],
    dict[Arc, float],
    set[Arc],
    dict[Arc, float],
    dict[Arc, float],
    tuple[Node, ...],
    dict[Arc, float],
]:
    if config.mandatory_drone_customer_fraction <= 0.0:
        saving = _drone_arc_savings(depot_source, customers, hubs, truck_arcs, truck_time, drone_arcs, drone_time)
        return (
            truck_arcs,
            truck_time,
            drone_arcs,
            drone_time,
            drone_trip_time,
            (),
            {arc: saving[arc] for arc in sorted(drone_arcs)},
        )

    savings = _drone_arc_savings(depot_source, customers, hubs, truck_arcs, truck_time, drone_arcs, drone_time)
    mandatory_count = int(ceil(config.mandatory_drone_customer_fraction * len(customers)))
    eligible_customers = {
        customer
        for _, customer in drone_arcs
    }
    ranked_customers = sorted(
        eligible_customers,
        key=lambda customer: (
            -max(savings[(hub, customer)] for hub in hubs if (hub, customer) in drone_arcs),
            min(drone_trip_time[(hub, customer)] for hub in hubs if (hub, customer) in drone_arcs),
            customer,
        ),
    )
    if len(ranked_customers) < mandatory_count:
        raise ValueError(
            f"not enough drone-feasible customers for mandatory drone policy: "
            f"{len(ranked_customers)} < {mandatory_count}"
        )
    mandatory_drone_customers = tuple(sorted(ranked_customers[:mandatory_count]))
    mandatory_set = set(mandatory_drone_customers)

    truck_arcs = {arc for arc in truck_arcs if arc[0] not in mandatory_set and arc[1] not in mandatory_set}
    truck_time = {arc: value for arc, value in truck_time.items() if arc in truck_arcs}
    return (
        truck_arcs,
        truck_time,
        drone_arcs,
        drone_time,
        drone_trip_time,
        mandatory_drone_customers,
        {arc: savings[arc] for arc in sorted(drone_arcs)},
    )


def _drone_arc_savings(
    depot_source: Node,
    customers: tuple[Node, ...],
    hubs: tuple[Node, ...],
    truck_arcs: set[Arc],
    truck_time: dict[Arc, float],
    drone_arcs: set[Arc],
    drone_time: dict[Arc, float],
) -> dict[Arc, float]:
    graph = nx.DiGraph()
    graph.add_weighted_edges_from((i, j, truck_time[(i, j)]) for i, j in truck_arcs)
    lengths = dict(nx.all_pairs_dijkstra_path_length(graph, weight="weight"))
    savings: dict[Arc, float] = {}
    for hub in hubs:
        for customer in customers:
            if (hub, customer) not in drone_arcs:
                continue
            truck_service = lengths.get(depot_source, {}).get(customer, float("inf"))
            drone_service = lengths.get(depot_source, {}).get(hub, float("inf")) + drone_time[(hub, customer)]
            savings[(hub, customer)] = truck_service - drone_service
    return savings


def instance_generation_metadata(instance: InstanceData) -> dict[str, object]:
    savings = list(instance.drone_arc_saving.values())
    launch_pad_counts = {
        customer: sum((hub, customer) in instance.drone_arcs for hub in instance.hubs)
        for customer in instance.customers
    }
    return {
        "requested_seed": instance.requested_seed,
        "realized_seed": instance.config.seed,
        "generation_attempt": instance.generation_attempt,
        "generation_feasibility_time": instance.generation_feasibility_time,
        "generation_feasibility_diagnostics": list(instance.generation_feasibility_diagnostics),
        "truck_arcs": len(instance.truck_arcs),
        "drone_arcs": len(instance.drone_arcs),
        "multi_pad_customer_count": sum(count > 1 for count in launch_pad_counts.values()),
        "mandatory_drone_customers": list(instance.mandatory_drone_customers),
        "mandatory_drone_customer_count": len(instance.mandatory_drone_customers),
        "retained_drone_arc_saving_min": min(savings) if savings else None,
        "retained_drone_arc_saving_mean": sum(savings) / len(savings) if savings else None,
        "retained_drone_arc_saving_max": max(savings) if savings else None,
        "retained_drone_arc_savings": [
            {"hub": hub, "customer": customer, "saving": saving}
            for (hub, customer), saving in sorted(instance.drone_arc_saving.items())
        ],
    }


def _choose_hubs_by_kmeans(seed: int, customer_locations: np.ndarray, count: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    initial = rng.choice(len(customer_locations), size=count, replace=False)
    centers = customer_locations[initial].astype(float).copy()
    for _ in range(100):
        distances = np.linalg.norm(customer_locations[:, None, :] - centers[None, :, :], axis=2)
        labels = np.argmin(distances, axis=1)
        new_centers = centers.copy()
        for idx in range(count):
            members = customer_locations[labels == idx]
            if len(members) == 0:
                farthest = int(np.argmax(np.min(distances, axis=1)))
                new_centers[idx] = customer_locations[farthest]
            else:
                new_centers[idx] = members.mean(axis=0)
        if np.allclose(new_centers, centers):
            break
        centers = new_centers
    return centers


def _sample_demands(
    config: InstanceConfig,
    customers: tuple[Node, ...],
    rng: random.Random,
    np_rng: np.random.Generator,
) -> dict[Node, float]:
    low_count = int(config.low_demand_customer_ratio * len(customers))
    low_customers = set(rng.sample(list(customers), low_count))
    low_values = _sample_truncated_normal(
        np_rng,
        config.low_demand_weight_mean,
        config.low_demand_weight_std,
        low_count,
        config.low_demand_weight_min,
        config.drone_payload,
    )
    high_count = len(customers) - low_count
    high_values = _sample_truncated_normal(
        np_rng,
        config.high_demand_weight_mean,
        config.high_demand_weight_std,
        high_count,
        config.high_demand_weight_min,
        float("inf"),
    )
    demand: dict[Node, float] = {}
    low_index = 0
    high_index = 0
    for customer in customers:
        if customer in low_customers:
            demand[customer] = round(float(low_values[low_index]), 2)
            low_index += 1
        else:
            demand[customer] = round(float(high_values[high_index]), 2)
            high_index += 1
    return demand


def _sample_truncated_normal(
    rng: np.random.Generator,
    mean: float,
    standard_deviation: float,
    count: int,
    lower: float,
    upper: float,
) -> np.ndarray:
    values: list[float] = []
    while len(values) < count:
        value = float(rng.normal(mean, standard_deviation))
        if lower <= value <= upper:
            values.append(value)
    return np.asarray(values, dtype=float)


def _add_truck_arc(
    config: InstanceConfig,
    locations: dict[Node, tuple[float, float]],
    i: Node,
    j: Node,
    arcs: set[Arc],
    times: dict[Arc, float],
) -> None:
    arcs.add((i, j))
    times[(i, j)] = _truck_time(config, locations[i], locations[j])


def _truck_time(config: InstanceConfig, i: tuple[float, float], j: tuple[float, float]) -> float:
    return (abs(i[0] - j[0]) + abs(i[1] - j[1])) / config.truck_speed * 60.0


def _drone_time(config: InstanceConfig, i: tuple[float, float], j: tuple[float, float]) -> float:
    return hypot(i[0] - j[0], i[1] - j[1]) / config.drone_speed * 60.0
