from __future__ import annotations

import copy
from collections import defaultdict

import CommonHelper


class NodeInfo:
    def __init__(self, parent_id, self_id, depth):
        self.parent_id = parent_id
        self.id = self_id
        self.depth = depth
        self.columns = list()  # value: the keys of routes
        self.column_customer_visits = defaultdict(set)  # key: route_key; value: the customers visited by the route
        self.vehicle_fleet_branch_lb = 0
        self.vehicle_fleet_branch_ub = CommonHelper.net.num_trucks
        # SR inequalities
        self.SR_infos = {customer_triple: [] for customer_triple in
                         CommonHelper.transformed_net.PI}  # value: column keys
        self.added_SR_keys = set()  # the keys of SR inequalities that has been added to current RMP
        self.column_in_SR_triples = defaultdict(list)  # key: column_key; value: involved SR inequality keys
        # branching constraints
        self.child_ids = []
        # for label setting algorithm
        self.disabled_arcs_trucks = set()  # the arcs must not be visited, original network
        self.disabled_arcs_drones = set()  # the drone arcs must not be visited, original network, by down-branch
        self.must_visit_arcs_trucks = set()  # (i,j) in the original network
        self.must_visit_arcs_drones = set()
        # in the transformed network
        self.disabled_arcs_trans = set()  # same as self.disabled_arcs, defined for transformed network
        self.must_visit_arcs_trans = set()
        self.removed_columns_keys = set()  # columns indices that have been removed from current node

    def as_child(self, parent: NodeInfo):
        self.columns = parent.columns.copy()
        self.column_customer_visits = copy.deepcopy(parent.column_customer_visits)
        self.vehicle_fleet_branch_lb = parent.vehicle_fleet_branch_lb
        self.vehicle_fleet_branch_ub = parent.vehicle_fleet_branch_ub
        self.SR_infos = copy.deepcopy(parent.SR_infos)
        self.added_SR_keys = parent.added_SR_keys.copy()
        self.column_in_SR_triples = copy.deepcopy(parent.column_in_SR_triples)
        self.disabled_arcs_trucks = parent.disabled_arcs_trucks.copy()
        self.disabled_arcs_drones = parent.disabled_arcs_drones.copy()
        self.must_visit_arcs_trucks = parent.must_visit_arcs_trucks.copy()
        self.must_visit_arcs_drones = parent.must_visit_arcs_drones.copy()
        self.disabled_arcs_trans = parent.disabled_arcs_trans.copy()
        self.must_visit_arcs_trans = parent.must_visit_arcs_trans.copy()
        self.removed_columns_keys = parent.removed_columns_keys.copy()

    def init_SR_infos(self):
        for triple in CommonHelper.transformed_net.PI:
            if len(self.added_SR_keys) >= CommonHelper.SR_num:
                break
            for route_key, covered_customer in self.column_customer_visits.items():
                if len(covered_customer & set(triple)) >= 2:
                    self.SR_infos[triple].append(route_key)
                    self.added_SR_keys.add(triple)
                    self.column_in_SR_triples[route_key].append(triple)

    # def serialize(self) -> dict:
    #     """Turn this NodeInfo into a pure‐Python dict."""
    #     return {
    #         'parent_id': self.parent_id,
    #         'id': self.id,
    #         'depth': self.depth,
    #         'columns': list(self.columns),
    #         'column_customer_visits': {
    #             ck: list(customers)
    #             for ck, customers in self.column_customer_visits.items()
    #         },
    #         'vehicle_fleet_branch_lb': self.vehicle_fleet_branch_lb,
    #         'vehicle_fleet_branch_ub': self.vehicle_fleet_branch_ub,
    #         'SR_infos': {
    #             tuple(triple): list(cols)
    #             for triple, cols in self.SR_infos.items()
    #         },
    #         'added_SR_keys': [tuple(k) for k in self.added_SR_keys],
    #         'column_in_SR_triples': {
    #             ck: [tuple(t) for t in triples]
    #             for ck, triples in self.column_in_SR_triples.items()
    #         },
    #         'child_ids': list(self.child_ids),
    #         'disabled_arcs_trucks': [tuple(a) for a in self.disabled_arcs_trucks],
    #         'disabled_arcs_drones': [tuple(a) for a in self.disabled_arcs_drones],
    #         'must_visit_arcs_trucks': [tuple(a) for a in self.must_visit_arcs_trucks],
    #         'must_visit_arcs_drones': [tuple(a) for a in self.must_visit_arcs_drones],
    #         'disabled_arcs_trans': [tuple(a) for a in self.disabled_arcs_trans],
    #         'must_visit_arcs_trans': [tuple(a) for a in self.must_visit_arcs_trans],
    #         'removed_columns_keys': list(self.removed_columns_keys),
    #     }
    #
    # @classmethod
    # def deserialize(cls, data: dict) -> NodeInfo:
    #     """Rebuild a NodeInfo from one made by serialize()."""
    #     ni = cls(
    #         parent_id=data['parent_id'],
    #         self_id=data['id'],
    #         depth=data['depth']
    #     )
    #     ni.columns = data['columns']
    #     ni.column_customer_visits = defaultdict(
    #         set,
    #         {ck: set(v) for ck, v in data['column_customer_visits'].items()}
    #     )
    #     ni.vehicle_fleet_branch_lb = data['vehicle_fleet_branch_lb']
    #     ni.vehicle_fleet_branch_ub = data['vehicle_fleet_branch_ub']
    #     ni.SR_infos = {
    #         tuple(triple): list(cols)
    #         for triple, cols in data['SR_infos'].items()
    #     }
    #     ni.added_SR_keys = {tuple(k) for k in data['added_SR_keys']}
    #     ni.column_in_SR_triples = defaultdict(
    #         list,
    #         {ck: [tuple(t) for t in triples]
    #          for ck, triples in data['column_in_SR_triples'].items()}
    #     )
    #     ni.child_ids = data['child_ids']
    #     ni.disabled_arcs_trucks = {tuple(a) for a in data['disabled_arcs_trucks']}
    #     ni.disabled_arcs_drones = {tuple(a) for a in data['disabled_arcs_drones']}
    #     ni.must_visit_arcs_trucks = {tuple(a) for a in data['must_visit_arcs_trucks']}
    #     ni.must_visit_arcs_drones = {tuple(a) for a in data['must_visit_arcs_drones']}
    #     ni.disabled_arcs_trans = {tuple(a) for a in data['disabled_arcs_trans']}
    #     ni.must_visit_arcs_trans = {tuple(a) for a in data['must_visit_arcs_trans']}
    #     ni.removed_columns_keys = set(data['removed_columns_keys'])
    #     return ni
