from __future__ import annotations
import copy
import GeneralHelper
from collections import defaultdict


class NodeInfo:
    def __init__(self, parent_id, self_id):
        self.parent_id = parent_id
        self.id = self_id
        self.columns = list()  # value: the keys of routes
        self.column_elementary_paths = {}  # key: route_key; value: the elementary paths for each column
        self.column_customer_visits = defaultdict(set)  # key: route_key; value: the customers visited by the route
        self.vehicle_fleet_branch_lb = 0
        self.vehicle_fleet_branch_ub = GeneralHelper.net.num_trucks
        # SR inequalities
        self.SR_infos = {customer_triple: [] for customer_triple in
                         GeneralHelper.transformed_net.PI}  # value: column keys
        self.column_in_SR_triples = defaultdict(list)  # key: column_key; value: involved SR inequality keys
        # branching constraints
        self.child_ids = []
        # for label setting algorithm
        self.disabled_arcs = set()  # the arcs must not be visited, original network, individually disabled
        self.disabled_arcs_drones_left = set()  # the drone arcs must not be visited, original network, by down-branch
        self.must_visit_arcs = set()  # (i,j) in the original network
        # in the transformed network
        self.disabled_arcs_trans = set()  # same as self.disabled_arcs, defined for transformed network
        self.must_visit_arcs_trans = set()
        self.removed_columns_keys = set()  # columns indices that have been removed from current node

    def as_child(self, parent: NodeInfo):
        self.columns = parent.columns.copy()
        self.column_elementary_paths = parent.column_elementary_paths.copy()
        self.column_customer_visits = copy.deepcopy(parent.column_customer_visits)
        self.vehicle_fleet_branch_lb = parent.vehicle_fleet_branch_lb
        self.vehicle_fleet_branch_ub = parent.vehicle_fleet_branch_ub
        self.SR_infos = copy.deepcopy(parent.SR_infos)
        self.column_in_SR_triples = copy.deepcopy(parent.column_in_SR_triples)
        self.disabled_arcs = parent.disabled_arcs.copy()
        self.disabled_arcs_drones_left = parent.disabled_arcs_drones_left.copy()
        self.must_visit_arcs = parent.must_visit_arcs.copy()
        self.disabled_arcs_trans = parent.disabled_arcs_trans.copy()
        self.must_visit_arcs_trans = parent.must_visit_arcs_trans.copy()
        self.removed_columns_keys = parent.removed_columns_keys.copy()

    def init_SR_infos(self):
        for triple in GeneralHelper.transformed_net.PI:
            for route_key, covered_customer in self.column_customer_visits.items():
                if len(covered_customer & set(triple)) >= 2:
                    self.SR_infos[triple].append(route_key)
                    self.column_in_SR_triples[route_key].append(triple)
