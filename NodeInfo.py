class NodeInfo:
    def __init__(self, parent_id):
        self.parent_id = parent_id
        self.columns = set()  # value: the keys of routes
        self.branches = []  # value: (lhs_var_names, rhs, cons_name)
