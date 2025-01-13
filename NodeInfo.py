class NodeInfo:
    def __init__(self, model, parent_id):
        self.model = model
        self.parent_id = parent_id
        self.columns = []  # value: the keys of routes
        self.branches = []  # value: (lhs_var_names, rhs, cons_name)
