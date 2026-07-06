"""Truck-as-hub VRP with drones."""

from .config import InstanceConfig, ObjectiveWeights, SolverConfig
from .instance import InstanceData, generate_instance


def solve_branch_price_cut(*args, **kwargs):
    from .bpc import solve_branch_price_cut as _solve_branch_price_cut

    return _solve_branch_price_cut(*args, **kwargs)

__all__ = [
    "InstanceConfig",
    "ObjectiveWeights",
    "SolverConfig",
    "InstanceData",
    "generate_instance",
    "solve_branch_price_cut",
]
