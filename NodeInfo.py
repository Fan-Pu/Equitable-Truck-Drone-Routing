from __future__ import annotations

import GeneralHelper
from collections import defaultdict


class NodeInfo:
    def __init__(self, parent_id, self_id):
        self.parent_id = parent_id
        self.id = self_id
        self.columns = list()  # value: the keys of routes
        self.column_elementary_paths = {}  # key: route_key; value: the elementary paths for each column
        self.vehicle_fleet_branch_lb = 0
        self.vehicle_fleet_branch_ub = GeneralHelper.net.num_trucks
        # branching constraints
        self.child_ids = []
        # for label setting algorithm
        self.disabled_arcs = set()  # the arcs must not be visited, original network, individually disabled
        self.disabled_arcs_drones_left = set()  # the drone arcs must not be visited, original network, by down-branch
        self.must_visit_arcs = set()  # (i,j) in the original network

        self.disabled_arcs_trans = set()  # same as self.disabled_arcs, defined for transformed network
        self.must_visit_arcs_trans = set()
        self.removed_columns_keys = set()  # columns indices that have been removed from current node

    def as_child(self, parent: NodeInfo):
        self.columns = parent.columns.copy()
        self.column_elementary_paths = parent.column_elementary_paths.copy()
        self.vehicle_fleet_branch_lb = parent.vehicle_fleet_branch_lb
        self.vehicle_fleet_branch_ub = parent.vehicle_fleet_branch_ub
        self.disabled_arcs = parent.disabled_arcs.copy()
        self.disabled_arcs_drones_left = parent.disabled_arcs_drones_left.copy()
        self.must_visit_arcs = parent.must_visit_arcs.copy()
        self.disabled_arcs_trans = parent.disabled_arcs_trans.copy()
        self.must_visit_arcs_trans = parent.must_visit_arcs_trans.copy()
        self.removed_columns_keys = parent.removed_columns_keys.copy()
