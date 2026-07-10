from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import time
from typing import Any

from .compact import solve_compact_solution
from .config import InstanceConfig, ObjectiveWeights, case_defaults
from .instance import generate_instance, instance_generation_metadata
from .metrics import solution_service_metrics
from .objective import build_objective_data
from .routes import Route, route_from_path
from .service_windows import load_manual_service_deadline_bounds
from .transform import build_transformed_graph


class _RouteSetResult:
    def __init__(self, routes: tuple[Route, ...], objective: Any, objective_full: float | None, runtime: float) -> None:
        self.routes = routes
        self.objective = objective
        self.objective_full = objective_full
        self.runtime = runtime

    def to_record(self) -> dict[str, Any]:
        customer_service_times = {
            customer: route.service_times[customer]
            for route in self.routes
            for customer in route.served
        }
        return {
            "objective_full": self.objective_full,
            "runtime": self.runtime,
            "customer_service_times": dict(sorted(customer_service_times.items())),
            "return_times": [route.return_time for route in self.routes],
            "routes": [route.to_record(self.objective) for route in self.routes],
        }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--num-trucks", type=int, required=True)
    parser.add_argument("--num-customers", type=int, required=True)
    parser.add_argument("--num-hubs", type=int, default=2)
    parser.add_argument("--distribution", choices=["PS", "PC", "mixed"], required=True)
    parser.add_argument("--drones-per-truck", type=int, required=True)
    parser.add_argument("--weights", nargs=3, type=float, default=[0.4, 0.3, 0.3])
    parser.add_argument("--time-limit", type=float, default=1200.0)
    parser.add_argument("--threads", type=int, default=1)
    parser.add_argument("--truck-arc-probability", type=float)
    parser.add_argument("--hub-arc-probability", type=float)
    parser.add_argument("--truck-speed", type=float, default=40.0)
    parser.add_argument("--drone-speed", type=float, default=100.0)
    parser.add_argument("--truck-payload", type=float, default=50.0)
    parser.add_argument("--drone-payload", type=float, default=6.0)
    parser.add_argument("--drone-endurance", type=float, default=75.0)
    parser.add_argument("--truck-cost", type=float, default=20.0)
    parser.add_argument("--drone-cost", type=float, default=1.0)
    parser.add_argument("--mandatory-drone-customer-fraction", type=float)
    parser.add_argument("--max-drone-access-customers-per-hub", type=int)
    parser.add_argument("--max-drone-launch-hubs-per-customer", type=int)
    parser.add_argument("--min-drone-service-time-saving", type=float)
    parser.add_argument("--retain-optional-drone-arcs", action=argparse.BooleanOptionalAction, default=None)
    parser.add_argument("--service-deadline-mode", choices=["none", "manual", "random_absolute"])
    parser.add_argument("--service-deadline-file", type=Path)
    parser.add_argument("--service-deadline-fraction", type=float, default=0.60)
    parser.add_argument("--service-deadline-offset-min", type=float)
    parser.add_argument("--service-deadline-offset-max", type=float)
    parser.add_argument("--service-deadline-witness-slack", type=float)
    parser.add_argument("--service-deadline-random-seed", type=int)
    parser.add_argument(
        "--service-deadline-witness-method",
        choices=["constructive", "compact", "constructive_then_compact"],
        default="constructive_then_compact",
    )
    parser.add_argument("--service-deadline-witness-time-limit", type=float, default=30.0)
    args = parser.parse_args()

    defaults = case_defaults(
        num_customers=args.num_customers,
        num_trucks=args.num_trucks,
        num_hubs=args.num_hubs,
        drones_per_truck=args.drones_per_truck,
    )
    for name, value in defaults.items():
        if hasattr(args, name) and getattr(args, name) is None:
            setattr(args, name, value)

    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    log_dir = output_dir / "gurobi_logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    log_file = log_dir / "compact_arc.log"

    weights = ObjectiveWeights(*args.weights)
    manual_deadlines = load_manual_service_deadline_bounds(args.service_deadline_file) if args.service_deadline_file else None
    instance_config = InstanceConfig(
        seed=args.seed,
        num_trucks=args.num_trucks,
        num_customers=args.num_customers,
        num_hubs=args.num_hubs,
        distribution=args.distribution,
        drones_per_truck=args.drones_per_truck,
        truck_arc_probability=args.truck_arc_probability,
        hub_arc_probability=args.hub_arc_probability,
        truck_speed=args.truck_speed,
        drone_speed=args.drone_speed,
        truck_payload=args.truck_payload,
        drone_payload=args.drone_payload,
        drone_endurance=args.drone_endurance,
        truck_cost=args.truck_cost,
        drone_cost=args.drone_cost,
        mandatory_drone_customer_fraction=args.mandatory_drone_customer_fraction,
        max_drone_access_customers_per_hub=args.max_drone_access_customers_per_hub,
        max_drone_launch_hubs_per_customer=args.max_drone_launch_hubs_per_customer,
        min_drone_service_time_saving=args.min_drone_service_time_saving,
        retain_optional_drone_arcs=args.retain_optional_drone_arcs,
        service_deadline_mode=args.service_deadline_mode,
        service_deadline_fraction=args.service_deadline_fraction,
        service_deadline_manual_bounds=manual_deadlines,
        service_deadline_offset_min=args.service_deadline_offset_min,
        service_deadline_offset_max=args.service_deadline_offset_max,
        service_deadline_witness_slack=args.service_deadline_witness_slack,
        service_deadline_random_seed=args.service_deadline_random_seed,
        service_deadline_witness_method=args.service_deadline_witness_method,
        service_deadline_witness_time_limit=args.service_deadline_witness_time_limit,
    )
    instance = generate_instance(instance_config)
    objective = build_objective_data(instance, weights)
    graph = build_transformed_graph(instance)

    start = time.time()
    solution = solve_compact_solution(
        instance,
        weights,
        time_limit=args.time_limit,
        threads=args.threads,
        require_optimal=False,
        log_file=str(log_file),
        objective=objective,
    )
    runtime = time.time() - start
    routes = tuple(route_from_path(i, path, graph, objective) for i, path in enumerate(solution.route_paths))
    result = _RouteSetResult(routes, objective, solution.objective_full, runtime)
    metrics = solution_service_metrics(instance, result) if routes else {}

    record = {
        "status": solution.status,
        "objective_full": solution.objective_full,
        "objective_bound_full": solution.objective_bound_full,
        "mip_gap": solution.mip_gap,
        "status_code": solution.status_code,
        "node_count": solution.node_count,
        "iteration_count": solution.iteration_count,
        "runtime": runtime,
        "time_limit": args.time_limit,
        "threads": args.threads,
        "timing": asdict(solution.timing),
        "instance_config": asdict(instance_config),
        "objective_weights": asdict(weights),
        "normalization_bounds": asdict(objective.bounds),
        "objective_coefficients": asdict(objective.coeffs),
        "instance_metadata": instance_generation_metadata(instance),
        "transformed_nodes": len(graph.nodes),
        "transformed_arcs": len(graph.arcs),
        "route_count": len(routes),
        "route_paths": [list(path) for path in solution.route_paths],
        "routes": [route.to_record(objective) for route in routes],
        "service_metrics": metrics,
        "gurobi_log_file": str(log_file),
    }
    output_path = output_dir / f"compact-arc-seed={args.seed}.json"
    output_path.write_text(json.dumps(record, indent=2), encoding="utf-8")
    print(output_path)


if __name__ == "__main__":
    main()
