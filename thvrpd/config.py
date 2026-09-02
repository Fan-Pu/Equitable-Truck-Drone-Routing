from __future__ import annotations

from dataclasses import dataclass
from math import isclose, isfinite


GENERIC_CASE_DEFAULTS = {
    "truck_arc_probability": 0.05,
    "hub_arc_probability": 0.18,
    "mandatory_drone_customer_fraction": 0.16,
    "service_deadline_mode": "random_absolute",
    "service_deadline_offset_min": 30.0,
    "service_deadline_offset_max": 90.0,
    "generation_feasibility_discovery_limit": 60.0,
    "generation_repair_mode": "arc_swap",
    "pricing_tolerance": 0.0001,
    "pricing_parallel_workers": 12,
    "pricing_worker_backend": "process",
}

def case_defaults(
    *,
    num_customers: int,
    num_trucks: int,
    num_hubs: int,
    drones_per_truck: int,
) -> dict[str, float | int | bool | str]:
    return dict(GENERIC_CASE_DEFAULTS)


@dataclass(frozen=True)
class ObjectiveWeights:
    delay: float
    return_time: float
    cost: float

    def __post_init__(self) -> None:
        total = self.delay + self.return_time + self.cost
        if not isclose(total, 1.0, rel_tol=0.0, abs_tol=1e-9):
            raise ValueError("objective weights must sum to one")
        if self.delay < 0 or self.return_time < 0 or self.cost < 0:
            raise ValueError("objective weights must be nonnegative")


@dataclass(frozen=True)
class InstanceConfig:
    seed: int
    num_trucks: int
    num_customers: int
    distribution: str
    drones_per_truck: int
    num_hubs: int = 2
    area_side: float = 25.0
    truck_arc_probability: float = 0.05
    hub_arc_probability: float = 0.18
    truck_speed: float = 40.0
    drone_speed: float = 40.0
    truck_payload: float = 50.0
    drone_payload: float = 2.3
    drone_endurance: float = 30.0
    truck_cost: float = 20.0
    drone_cost: float = 6.0
    mandatory_drone_customer_fraction: float = 0.16
    low_demand_customer_ratio: float = 0.5
    low_demand_weight_mean: float = 1.0
    low_demand_weight_std: float = 5.0
    low_demand_weight_min: float = 0.5
    high_demand_weight_mean: float = 10.0
    high_demand_weight_std: float = 5.0
    high_demand_weight_min: float = 6.0
    service_deadline_mode: str = "random_absolute"
    service_deadline_manual_bounds: dict[str, float] | None = None
    service_deadline_offset_min: float = 30.0
    service_deadline_offset_max: float = 90.0
    service_deadline_random_seed: int | None = None
    generation_feasibility_discovery_limit: float = 60.0
    generation_repair_mode: str = "arc_swap"

    def __post_init__(self) -> None:
        if self.distribution not in {"PS", "PC", "mixed"}:
            raise ValueError("distribution must be PS, PC, or mixed")
        if self.num_trucks <= 0 or self.num_customers <= 0:
            raise ValueError("truck and customer counts must be positive")
        if self.drones_per_truck <= 0:
            raise ValueError("drones per truck must be positive")
        if self.num_hubs <= 0:
            raise ValueError("hub count must be positive")
        if self.area_side <= 0.0:
            raise ValueError("region side length must be positive")
        if self.truck_speed <= 0.0 or self.drone_speed <= 0.0:
            raise ValueError("truck and drone speeds must be positive")
        if self.truck_payload <= 0.0 or self.drone_payload <= 0.0:
            raise ValueError("truck and drone payloads must be positive")
        if self.drone_endurance <= 0.0:
            raise ValueError("drone endurance must be positive")
        if self.truck_cost < 0.0 or self.drone_cost < 0.0:
            raise ValueError("truck and drone costs must be nonnegative")
        if not 0.0 <= self.truck_arc_probability <= 1.0 or not 0.0 <= self.hub_arc_probability <= 1.0:
            raise ValueError("truck and hub arc probabilities must be in [0, 1]")
        if not 0.0 <= self.mandatory_drone_customer_fraction <= 1.0:
            raise ValueError("mandatory drone customer fraction must be in [0, 1]")
        if not 0.0 <= self.low_demand_customer_ratio <= 1.0:
            raise ValueError("low-demand customer ratio must be in [0, 1]")
        if self.low_demand_weight_std <= 0.0 or self.high_demand_weight_std <= 0.0:
            raise ValueError("demand standard deviations must be positive")
        if self.low_demand_weight_min <= 0.0 or self.low_demand_weight_min > self.drone_payload:
            raise ValueError("low-demand bounds must satisfy 0 < minimum <= drone payload")
        if self.high_demand_weight_min <= 0.0:
            raise ValueError("high-demand minimum must be positive")
        if self.service_deadline_mode not in {"none", "manual", "random_absolute"}:
            raise ValueError("service_deadline_mode must be none, manual, or random_absolute")
        if self.service_deadline_mode == "manual":
            if not self.service_deadline_manual_bounds:
                raise ValueError("manual service deadline mode requires explicit customer bounds")
            for customer, bound in self.service_deadline_manual_bounds.items():
                if not isinstance(customer, str) or not customer:
                    raise ValueError("manual service deadline customer ids must be nonempty strings")
                if not isfinite(float(bound)):
                    raise ValueError(f"manual service deadline for {customer} must be finite")
        if (
            not isfinite(self.service_deadline_offset_min)
            or not isfinite(self.service_deadline_offset_max)
            or self.service_deadline_offset_min < 0.0
            or self.service_deadline_offset_max < self.service_deadline_offset_min
        ):
            raise ValueError("service deadline offset bounds must be finite and satisfy 0 <= min <= max")
        if self.generation_feasibility_discovery_limit <= 0.0:
            raise ValueError("generation feasibility discovery limit must be positive")
        if self.generation_repair_mode not in {"arc_swap", "none"}:
            raise ValueError("generation repair mode must be arc_swap or none")


@dataclass(frozen=True)
class SolverConfig:
    threads: int = 0
    time_limit: float = 1800.0
    pricing_tolerance: float = 0.0001
    cut_tolerance: float = 1e-7
    integrality_tolerance: float = 1e-6
    route_pool_time_limit: float = 2.0
    pricing_batch_size: int = 64
    farkas_batch_size: int = 16
    enable_root_compact_warm_start: bool = True
    root_compact_solve_time_limit: float = 60.0
    enable_incremental_rmp: bool = True
    enable_active_coefficient_cache: bool = True
    use_row_local_sr_coeff_cache: bool = True
    sr_cut_add_batch_size: int = 32
    enable_global_branch_route_index: bool = True
    enable_pricing_pruning: bool = True
    pricing_parallel_workers: int = 12
    pricing_worker_backend: str = "process"
    pricing_workload_customer_weight: float = 1.0
    pricing_workload_out_degree_weight: float = 0.25
    pricing_workload_drone_pad_weight: float = 0.5
    pricing_workload_deadline_weight: float = 0.5
    pricing_split_open_labels_min: int = 500
    pricing_split_gap_factor: float = 1.0
    pricing_split_elapsed_min: float = 5.0
    pricing_split_work_min: int = 2000
    pricing_dynamic_refinement_depth: int = 2
    pricing_checkpoint_extension_period: int = 1000
    logging_mode: str = "audit"
    progress_snapshot_period: int = 1
    gurobi_log_dir: str | None = None

    def __post_init__(self) -> None:
        if self.threads < 0:
            raise ValueError("threads must be nonnegative")
        if self.time_limit <= 0.0:
            raise ValueError("time_limit must be positive")
        if self.pricing_tolerance < 0.0 or self.cut_tolerance < 0.0 or self.integrality_tolerance < 0.0:
            raise ValueError("solver tolerances must be nonnegative")
        if self.route_pool_time_limit <= 0.0:
            raise ValueError("route pool time limit must be positive")
        if min(self.pricing_batch_size, self.farkas_batch_size) <= 0:
            raise ValueError("pricing batch sizes must be positive")
        if self.enable_root_compact_warm_start and self.root_compact_solve_time_limit <= 0.0:
            raise ValueError("root_compact_solve_time_limit must be positive when compact initialization is enabled")
        if self.sr_cut_add_batch_size <= 0:
            raise ValueError("sr_cut_add_batch_size must be positive")
        if self.pricing_parallel_workers <= 0:
            raise ValueError("pricing_parallel_workers must be positive")
        if self.pricing_worker_backend not in {"thread", "process"}:
            raise ValueError("pricing_worker_backend must be thread or process")
        if min(
            self.pricing_workload_customer_weight,
            self.pricing_workload_out_degree_weight,
            self.pricing_workload_drone_pad_weight,
            self.pricing_workload_deadline_weight,
        ) < 0.0:
            raise ValueError("pricing workload weights must be nonnegative")
        if self.pricing_split_open_labels_min <= 0 or self.pricing_split_work_min <= 0:
            raise ValueError("pricing split workload thresholds must be positive")
        if self.pricing_split_gap_factor < 0.0 or self.pricing_split_elapsed_min < 0.0:
            raise ValueError("pricing split gap and elapsed thresholds must be nonnegative")
        if self.pricing_dynamic_refinement_depth <= 0 or self.pricing_checkpoint_extension_period <= 0:
            raise ValueError("pricing refinement depth and checkpoint period must be positive")
        if self.logging_mode not in {"audit", "light"}:
            raise ValueError("logging_mode must be audit or light")
        if self.progress_snapshot_period <= 0:
            raise ValueError("progress_snapshot_period must be positive")
