import GeneralHelper
from collections import defaultdict


class NodeInfo:
    def __init__(self, parent_id, self_id):
        self.parent_id = parent_id
        self.id = self_id
        self.columns = set()  # value: the keys of routes
        self.vehicle_fleet_branch_lb = 0
        self.vehicle_fleet_branch_ub = GeneralHelper.net.num_trucks
        # branching constraints
        self.arc_flow_lb_cons_dict = defaultdict(set)  # key: (i,j,frac_val), value: route_keys
        self.arc_flow_ub_cons_dict = defaultdict(set)  # key: (i,j,frac_val), value: route_keys
        self.child_ids = []
        # for label setting algorithm
        self.disabled_arcs = set()  # for branching, the arcs (in transformed network) must not be visited
        self.must_visit_arcs = set()  # (i,j)
        self.removed_columns_keys = set()  # columns indices that have been removed from current node

    def as_child(self, parent):
        self.columns = parent.columns.copy()
        self.vehicle_fleet_branch_lb = parent.vehicle_fleet_branch_lb
        self.vehicle_fleet_branch_ub = parent.vehicle_fleet_branch_ub
        self.arc_flow_lb_cons_dict = parent.arc_flow_lb_cons_dict.copy()
        self.arc_flow_ub_cons_dict = parent.arc_flow_ub_cons_dict.copy()
        self.disabled_arcs = parent.disabled_arcs.copy()
        self.must_visit_arcs = parent.must_visit_arcs.copy()
        self.removed_columns_keys = parent.removed_columns_keys.copy()
