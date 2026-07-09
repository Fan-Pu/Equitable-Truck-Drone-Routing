from __future__ import annotations

from dataclasses import dataclass, field
from itertools import combinations
import time

import gurobipy as gp
from gurobipy import GRB

from .branching import BranchRestrictions
from .config import SolverConfig
from .pricing import PricingDuals, _validate_inequality_dual_signs
from .routes import Route
from .columns import NodeColumnIndex, sr_coeff_from_mask, triplet_mask, customer_mask, RouteSignatureCache
from .solverlog import configure_gurobi_logging
from .transform import TransformedGraph


@dataclass
class RMPResult:
    status: int
    objective: float | None
    z_values: dict[tuple[str, ...], float]
    duals: PricingDuals
    farkas_rhs: float | None = None
    max_farkas_column_activity: float | None = None


@dataclass
class SRCutMetadata:
    age: int = 0
    inactive_count: int = 0
    last_activity: float = 0.0
    last_violation: float = 0.0
    last_positive_dual_iteration: int = -1
    activity_count: int = 0
    nonzero_count: int = 0
    coefficient_density: float = 0.0
    update_time_contribution: float = 0.0
    removal_candidate_count: int = 0
    removal_count: int = 0
    reactivation_count: int = 0


@dataclass
class RMPModelState:
    model: gp.Model
    z: dict[tuple[str, ...], gp.Var]
    cover_constraints: dict[str, gp.Constr]
    fleet_constraint: gp.Constr
    sr_constraints: dict[tuple[str, str, str], gp.Constr]
    residual_key: tuple[str, ...]
    restriction_key: tuple
    fleet_limit: int
    fixed_cost: float
    active_sr_key: tuple[tuple[str, str, str], ...]
    active_sr_version: int
    column_paths: frozenset[tuple[str, ...]]
    compatibility_key: tuple

    def compatible(self, node: "NodeState") -> bool:
        return self.compatibility_key == _rmp_compatibility_key(node)

    def compatibility_failures(self, node: "NodeState") -> tuple[str, ...]:
        expected = _rmp_compatibility_key(node)
        actual = self.compatibility_key
        names = _RMP_COMPATIBILITY_FIELDS
        return tuple(name for name, left, right in zip(names, actual, expected) if left != right)


@dataclass
class NodeState:
    id: int
    depth: int
    restrictions: BranchRestrictions
    fixed_routes: tuple[Route, ...]
    residual_customers: frozenset[str]
    fleet_limit: int
    fixed_cost: float
    column_paths: set[tuple[str, ...]]
    active_sr: set[tuple[str, str, str]]
    active_sr_version: int = 0
    column_index: NodeColumnIndex = field(default_factory=NodeColumnIndex)
    sr_cut_meta: dict[tuple[str, str, str], SRCutMetadata] = field(default_factory=dict)
    removed_sr: set[tuple[str, str, str]] = field(default_factory=set)
    pending_sr_removal_bound: float | None = None
    sr_removals_performed: int = 0
    inactive_column_paths: set[tuple[str, ...]] = field(default_factory=set)
    column_age: dict[tuple[str, ...], int] = field(default_factory=dict)
    pending_column_deactivation_bound: float | None = None
    previous_rmp_solve_time: float = 0.0
    rmp_solve_time_growth: float = 0.0
    basis_variables: dict[tuple[str, ...], int] = field(default_factory=dict)
    basis_cover: dict[str, int] = field(default_factory=dict)
    basis_fleet: int | None = None
    basis_sr: dict[tuple[str, str, str], int] = field(default_factory=dict)
    rmp_model_state: RMPModelState | None = None
    previous_rmp_build_time: float = 0.0
    rmp_build_time_growth: float = 0.0
    child_certification_signature: tuple | None = None
    child_certification_exhausted_task_count: int = 0
    child_certification_unresolved_task_count: int = 0
    child_closure_batch_limit: int = 0
    child_certification_yield_history: list[float] = field(default_factory=list)
    child_useful_yield_history: list[float] = field(default_factory=list)
    child_no_route_yield_history: list[float] = field(default_factory=list)
    child_dual_signature_history: list[tuple] = field(default_factory=list)

    def copy_for_child(self, node_id: int, restrictions: BranchRestrictions) -> "NodeState":
        return NodeState(
            id=node_id,
            depth=self.depth + 1,
            restrictions=restrictions,
            fixed_routes=self.fixed_routes,
            residual_customers=self.residual_customers,
            fleet_limit=self.fleet_limit,
            fixed_cost=self.fixed_cost,
            column_paths=set(),
            active_sr=set(self.active_sr),
            active_sr_version=self.active_sr_version,
            column_index=NodeColumnIndex(),
            sr_cut_meta={
                triplet: SRCutMetadata(
                    age=meta.age,
                    inactive_count=meta.inactive_count,
                    last_activity=meta.last_activity,
                    last_violation=meta.last_violation,
                    last_positive_dual_iteration=meta.last_positive_dual_iteration,
                    activity_count=meta.activity_count,
                    nonzero_count=meta.nonzero_count,
                    coefficient_density=meta.coefficient_density,
                    update_time_contribution=meta.update_time_contribution,
                    removal_candidate_count=meta.removal_candidate_count,
                    removal_count=meta.removal_count,
                    reactivation_count=meta.reactivation_count,
                )
                for triplet, meta in self.sr_cut_meta.items()
            },
            removed_sr=set(self.removed_sr),
            pending_sr_removal_bound=self.pending_sr_removal_bound,
            sr_removals_performed=self.sr_removals_performed,
            inactive_column_paths=set(),
            column_age={},
            pending_column_deactivation_bound=None,
            previous_rmp_solve_time=self.previous_rmp_solve_time,
            rmp_solve_time_growth=0.0,
            basis_variables=dict(self.basis_variables),
            basis_cover=dict(self.basis_cover),
            basis_fleet=self.basis_fleet,
            basis_sr=dict(self.basis_sr),
            rmp_model_state=None,
            previous_rmp_build_time=self.previous_rmp_build_time,
            rmp_build_time_growth=0.0,
            child_certification_signature=None,
            child_certification_exhausted_task_count=0,
            child_certification_unresolved_task_count=0,
            child_closure_batch_limit=0,
            child_certification_yield_history=[],
            child_useful_yield_history=[],
            child_no_route_yield_history=[],
            child_dual_signature_history=[],
        )


class RestrictedMaster:
    def __init__(
        self,
        graph: TransformedGraph,
        node: NodeState,
        routes: dict[tuple[str, ...], Route],
        solver_config: SolverConfig,
        sr_cache: RouteSignatureCache | None = None,
    ) -> None:
        self.graph = graph
        self.node = node
        self.routes = routes
        self.solver_config = solver_config
        self._sr_cache = sr_cache if solver_config.enable_active_coefficient_cache and sr_cache is not None else RouteSignatureCache()
        self.used_incremental_update = False
        self.used_full_rebuild = True
        self.compatibility_failure_reasons: tuple[str, ...] = tuple()
        self.active_sr_nonzero_count = 0
        self.active_sr_coefficient_count = 0
        self.active_sr_rows_added = 0
        self.active_sr_rows_removed = 0
        self.active_sr_row_local_updates = 0
        self.active_sr_full_rebuilds = 0
        if (
            solver_config.enable_incremental_rmp
            and node.rmp_model_state is not None
            and node.rmp_model_state.compatible(node)
        ):
            self._load_state(node.rmp_model_state)
            self._sync_incremental()
            self.used_incremental_update = True
            self.used_full_rebuild = False
        else:
            if node.rmp_model_state is not None:
                self.compatibility_failure_reasons = node.rmp_model_state.compatibility_failures(node)
            self.model = gp.Model(f"RMP_{node.id}")
            log_file = None
            if solver_config.gurobi_log_dir is not None:
                log_file = (
                    f"{solver_config.gurobi_log_dir}/"
                    f"rmp_node_{node.id}_cols_{len(node.column_paths)}_sr_{len(node.active_sr)}.log"
                )
            configure_gurobi_logging(self.model, log_file)
            self.model.Params.InfUnbdInfo = 1
            self.model.Params.Threads = solver_config.threads
            self.model.Params.OptimalityTol = max(min(solver_config.pricing_tolerance, 1e-2), 1e-9)
            self.model.Params.FeasibilityTol = max(min(solver_config.cut_tolerance, 1e-2), 1e-9)
            self.model.ModelSense = GRB.MINIMIZE
            self.z: dict[tuple[str, ...], gp.Var] = {}
            self.cover_constraints: dict[str, gp.Constr] = {}
            self.fleet_constraint: gp.Constr | None = None
            self.sr_constraints: dict[tuple[str, str, str], gp.Constr] = {}
            self._build()
            self._store_model_state()
        self._refresh_active_sr_density()

    def _build(self) -> None:
        if self.node.active_sr:
            self.active_sr_full_rebuilds += 1
        for path in sorted(self._eligible_column_paths()):
            route = self.routes[path]
            self.z[path] = self.model.addVar(lb=0.0, obj=route.cost, name=f"z_{route.id}")
        self.model.update()
        for customer in sorted(self.node.residual_customers):
            expr = gp.quicksum(var for path, var in self.z.items() if customer in self.routes[path].served)
            self.cover_constraints[customer] = self.model.addConstr(expr == 1.0, name=f"cover_{customer}")
        self.fleet_constraint = self.model.addConstr(gp.quicksum(self.z.values()) <= self.node.fleet_limit, name="fleet")
        for triplet in sorted(self.node.active_sr):
            expr = gp.quicksum(self._sr_coeff(path, triplet) * var for path, var in self.z.items())
            self.sr_constraints[triplet] = self.model.addConstr(expr <= 1.0, name=f"sr_{'_'.join(triplet)}")
            self.active_sr_rows_added += 1
        self.model.update()

    def _eligible_column_paths(self) -> set[tuple[str, ...]]:
        arc_customer_sets = {arc: self.graph.arc_customer_set(arc) for arc in self.graph.arcs}
        return {
            path
            for path in self.node.column_paths
            if (
                self.routes[path].served
                and self.routes[path].served.issubset(self.node.residual_customers)
                and self.node.restrictions.route_allowed(self.routes[path], arc_customer_sets)
            )
        }

    def _load_state(self, state: RMPModelState) -> None:
        self.model = state.model
        self.z = state.z
        self.cover_constraints = state.cover_constraints
        self.fleet_constraint = state.fleet_constraint
        self.sr_constraints = state.sr_constraints

    def _sync_incremental(self) -> None:
        target_paths = self._eligible_column_paths()
        current_paths = set(self.z)
        for path in sorted(current_paths - target_paths):
            self.model.remove(self.z.pop(path))
        self.model.update()
        added_paths = sorted(target_paths - set(self.z))
        for path in added_paths:
            route = self.routes[path]
            self.z[path] = self.model.addVar(lb=0.0, obj=route.cost, name=f"z_{route.id}")
        self.model.update()
        for path in added_paths:
            route = self.routes[path]
            var = self.z[path]
            for customer in route.served.intersection(self.node.residual_customers):
                self.model.chgCoeff(self.cover_constraints[customer], var, 1.0)
            self.model.chgCoeff(self.fleet_constraint, var, 1.0)
            for triplet, constr in self.sr_constraints.items():
                coeff = self._sr_coeff(path, triplet)
                if coeff:
                    self.model.chgCoeff(constr, var, coeff)
                    self.active_sr_row_local_updates += 1
        active_sr = set(self.node.active_sr)
        for triplet in sorted(set(self.sr_constraints) - active_sr):
            self.model.remove(self.sr_constraints.pop(triplet))
            self.active_sr_rows_removed += 1
        self.model.update()
        for triplet in sorted(active_sr - set(self.sr_constraints)):
            coeff_terms = []
            for path, var in self.z.items():
                coeff = self._sr_coeff(path, triplet)
                if coeff:
                    coeff_terms.append(coeff * var)
                    self.active_sr_row_local_updates += 1
            expr = gp.quicksum(coeff_terms)
            self.sr_constraints[triplet] = self.model.addConstr(expr <= 1.0, name=f"sr_{'_'.join(triplet)}")
            self.active_sr_rows_added += 1
        self.model.update()
        self._store_model_state()

    def _store_model_state(self) -> None:
        self.node.rmp_model_state = RMPModelState(
            model=self.model,
            z=self.z,
            cover_constraints=self.cover_constraints,
            fleet_constraint=self.fleet_constraint,
            sr_constraints=self.sr_constraints,
            residual_key=tuple(sorted(self.node.residual_customers)),
            restriction_key=_restriction_key(self.node.restrictions),
            fleet_limit=self.node.fleet_limit,
            fixed_cost=self.node.fixed_cost,
            active_sr_key=tuple(sorted(self.node.active_sr)),
            active_sr_version=self.node.active_sr_version,
            column_paths=frozenset(self.z),
            compatibility_key=_rmp_compatibility_key(self.node),
        )

    def _refresh_active_sr_density(self) -> None:
        self.active_sr_coefficient_count = len(self.z) * len(self.node.active_sr)
        self.active_sr_nonzero_count = 0
        for path in self.z:
            for triplet in self.node.active_sr:
                if self._sr_coeff(path, triplet):
                    self.active_sr_nonzero_count += 1

    def solve(self) -> RMPResult:
        self.model.optimize()
        if self.model.Status == GRB.OPTIMAL:
            z_values = {path: var.X for path, var in self.z.items()}
            duals = PricingDuals(
                mu={customer: constr.Pi for customer, constr in self.cover_constraints.items()},
                kappa=self.fleet_constraint.Pi,
                nu={triplet: constr.Pi for triplet, constr in self.sr_constraints.items()},
            )
            _validate_inequality_dual_signs(duals, self.solver_config.pricing_tolerance)
            return RMPResult(self.model.Status, self.model.ObjVal + self.node.fixed_cost, z_values, duals)
        if self.model.Status == GRB.INFEASIBLE:
            duals = PricingDuals(
                mu={customer: -constr.FarkasDual for customer, constr in self.cover_constraints.items()},
                kappa=-self.fleet_constraint.FarkasDual,
                nu={triplet: -constr.FarkasDual for triplet, constr in self.sr_constraints.items()},
            )
            _validate_inequality_dual_signs(duals, self.solver_config.pricing_tolerance)
            rhs, max_activity = _validate_farkas_certificate(
                duals,
                self.routes,
                self.z.keys(),
                self.node.residual_customers,
                self.node.fleet_limit,
                self.node.active_sr,
                self.solver_config.pricing_tolerance,
            )
            return RMPResult(self.model.Status, None, {}, duals, rhs, max_activity)
        raise RuntimeError(f"unexpected RMP status {self.model.Status}")

    def load_basis_from_node(self) -> bool:
        if not self.solver_config.enable_rmp_basis_reuse:
            return False
        if (
            not self.node.basis_variables
            or not self.node.basis_cover
            or self.node.basis_fleet is None
        ):
            return False
        if not set(self.z).issubset(self.node.basis_variables):
            return False
        if not set(self.cover_constraints).issubset(self.node.basis_cover):
            return False
        if not set(self.sr_constraints).issubset(self.node.basis_sr):
            return False
        for path, var in self.z.items():
            var.VBasis = self.node.basis_variables[path]
        for customer, constr in self.cover_constraints.items():
            constr.CBasis = self.node.basis_cover[customer]
        self.fleet_constraint.CBasis = self.node.basis_fleet
        for triplet, constr in self.sr_constraints.items():
            constr.CBasis = self.node.basis_sr[triplet]
        self.model.update()
        return True

    def store_basis_to_node(self) -> None:
        if not self.solver_config.enable_rmp_basis_reuse or self.model.Status != GRB.OPTIMAL:
            return
        self.node.basis_variables = {path: var.VBasis for path, var in self.z.items()}
        self.node.basis_cover = {customer: constr.CBasis for customer, constr in self.cover_constraints.items()}
        self.node.basis_fleet = self.fleet_constraint.CBasis
        self.node.basis_sr = {triplet: constr.CBasis for triplet, constr in self.sr_constraints.items()}

    def violated_sr_cuts(self, z_values: dict[tuple[str, ...], float], cut_tolerance: float) -> set[tuple[str, str, str]]:
        return set(self.violated_sr_cut_activities(z_values, cut_tolerance))

    def violated_sr_cut_activities(
        self,
        z_values: dict[tuple[str, ...], float],
        cut_tolerance: float,
    ) -> dict[tuple[str, str, str], float]:
        activities: dict[tuple[str, str, str], float] = {}
        for triplet in combinations(sorted(self.node.residual_customers), 3):
            if triplet in self.node.active_sr:
                continue
            activity = sum(self._sr_coeff(path, triplet) * value for path, value in z_values.items())
            if activity > 1.0 + cut_tolerance:
                activities[triplet] = activity
        return activities

    def sr_cut_activity(self, z_values: dict[tuple[str, ...], float], triplet: tuple[str, str, str]) -> float:
        return sum(self._sr_coeff(path, triplet) * value for path, value in z_values.items())

    def _sr_coeff(self, path: tuple[str, ...], triplet: tuple[str, str, str]) -> int:
        residual_key = tuple(sorted(self.node.residual_customers))
        served_mask = customer_mask(self.routes[path].served, residual_key, self._sr_cache)
        triplet_bits = triplet_mask(residual_key, triplet, self._sr_cache)
        if self.solver_config.use_row_local_sr_coeff_cache:
            key = (served_mask, triplet_bits)
            cached = self._sr_cache.sr_coeff_cache.get(key)
            if cached is not None:
                self._sr_cache.stats.sr_coeff_cache_hits += 1
                return cached
            self._sr_cache.stats.sr_coeff_cache_misses += 1
            start = time.time()
            coeff = sr_coeff_from_mask(served_mask, triplet_bits)
            self._sr_cache.stats.sr_coeff_build_time += time.time() - start
            self._sr_cache.stats.active_sr_coeffs_computed += 1
            self._sr_cache.sr_coeff_cache[key] = coeff
            return coeff
        return sr_coeff_from_mask(served_mask, triplet_bits)


def _restriction_key(restrictions: BranchRestrictions) -> tuple:
    return (
        tuple(sorted(restrictions.together_pairs)),
        tuple(sorted(restrictions.separate_pairs)),
        tuple(sorted(restrictions.truck_service)),
        tuple(sorted(restrictions.drone_service)),
        tuple(sorted(restrictions.pad_forbidden)),
        tuple(sorted(restrictions.pad_required)),
        tuple(sorted(restrictions.trans_arc_forbidden)),
        tuple(sorted(restrictions.trans_arc_required)),
        tuple(sorted(restrictions.route_forbidden)),
    )


_RMP_COMPATIBILITY_FIELDS = (
    "residual_customers",
    "fixed_routes",
    "fleet_limit",
    "fixed_cost",
    "branch_state",
    "active_sr",
    "active_sr_version",
    "service_deadline_version",
    "objective_scale_version",
    "active_column_version",
    "rmp_structure_version",
)


def _service_deadline_version(node: NodeState) -> int:
    return 0


def _objective_scale_version(node: NodeState) -> int:
    return 0


def _active_column_version(node: NodeState) -> tuple:
    return (tuple(sorted(node.inactive_column_paths)),)


def _rmp_structure_version(node: NodeState) -> tuple:
    return (
        tuple(sorted(node.residual_customers)),
        tuple(route.path for route in node.fixed_routes),
        node.fleet_limit,
        node.fixed_cost,
        _restriction_key(node.restrictions),
        tuple(sorted(node.active_sr)),
        node.active_sr_version,
        _service_deadline_version(node),
        _objective_scale_version(node),
        _active_column_version(node),
    )


def _rmp_compatibility_key(node: NodeState) -> tuple:
    return (
        tuple(sorted(node.residual_customers)),
        tuple(route.path for route in node.fixed_routes),
        node.fleet_limit,
        node.fixed_cost,
        _restriction_key(node.restrictions),
        tuple(sorted(node.active_sr)),
        node.active_sr_version,
        _service_deadline_version(node),
        _objective_scale_version(node),
        _active_column_version(node),
        _rmp_structure_version(node),
    )


def _farkas_column_activity(route: Route, duals: PricingDuals) -> float:
    return (
        sum(duals.mu[customer] for customer in route.served)
        + duals.kappa
        + sum(dual * route.sr_coeff(triplet) for triplet, dual in duals.nu.items())
    )


def _farkas_rhs(
    duals: PricingDuals,
    residual_customers: frozenset[str],
    fleet_limit: int,
    active_sr: set[tuple[str, str, str]],
) -> float:
    return (
        sum(duals.mu[customer] for customer in residual_customers)
        + fleet_limit * duals.kappa
        + sum(duals.nu[triplet] for triplet in active_sr)
    )


def _validate_farkas_certificate(
    duals: PricingDuals,
    routes: dict[tuple[str, ...], Route],
    column_paths,
    residual_customers: frozenset[str],
    fleet_limit: int,
    active_sr: set[tuple[str, str, str]],
    tolerance: float,
) -> tuple[float, float | None]:
    certificate_tolerance = max(tolerance, 1e-9)
    rhs = _farkas_rhs(duals, residual_customers, fleet_limit, active_sr)
    if rhs <= certificate_tolerance:
        raise RuntimeError("Farkas certificate residual RHS is not strictly positive")
    max_activity: float | None = None
    for path in column_paths:
        activity = _farkas_column_activity(routes[path], duals)
        max_activity = activity if max_activity is None else max(max_activity, activity)
        if activity > certificate_tolerance:
            raise RuntimeError("Farkas certificate violates current-column validity")
    return rhs, max_activity
