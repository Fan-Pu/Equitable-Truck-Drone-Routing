import GeneralHelper


class NodeInfo:
    def __init__(self, parent_id):
        self.parent_id = parent_id
        self.columns = set()  # value: the keys of routes
        self.vehicle_fleet_branch_lb = 0
        self.vehicle_fleet_branch_ub = GeneralHelper.net.num_trucks
        # branching constraints
        self.vehicle_fleet_branch_lb_cons_list = []
        self.vehicle_fleet_branch_ub_cons_list = []
        self.vehicle_fleet_branch_lb_cons = None
        self.vehicle_fleet_branch_ub_cons = None
        self.arc_flow_lb_cons_list = []  # value (i,j,frac_val,cons)
        self.arc_flow_ub_cons_list = []  # value (i,j,frac_val,cons)
        self.child_ids = []
        self.disabled_arcs = set()  # for branching, the arcs (in transformed network) must not be visited
        self.must_visit_arcs = set()  # (i,j)
        self.removed_columns_indices = set()  # columns indices that have been removed from current node

    def as_child(self, parent):
        self.columns = parent.columns.copy()
        self.vehicle_fleet_branch_lb = parent.vehicle_fleet_branch_lb
        self.vehicle_fleet_branch_ub = parent.vehicle_fleet_branch_ub
        self.vehicle_fleet_branch_lb_cons = parent.vehicle_fleet_branch_lb_cons
        self.vehicle_fleet_branch_ub_cons = parent.vehicle_fleet_branch_ub_cons
        self.arc_flow_lb_cons_list = parent.arc_flow_lb_cons_list.copy()
        self.arc_flow_ub_cons_list = parent.arc_flow_ub_cons_list.copy()
        self.disabled_arcs = parent.disabled_arcs.copy()
        self.must_visit_arcs = parent.must_visit_arcs.copy()
        self.removed_columns_indices = parent.removed_columns_indices.copy()
