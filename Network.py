epsilon_weight = 0.01


class Network:
    def __init__(self, num_trucks, num_drones, **kwargs):
        """Class for storing network data"""

        # Assign all provided keyword arguments to instance attributes
        self.__dict__.update(kwargs)

        # Derived attributes
        self.num_customers = len(self.customers)
        self.num_hubs = len(self.hubs)

        # Initialize drone-truck mapping
        self.drone_dict = {k: list(range(k * num_drones, (k + 1) * num_drones)) for k in range(num_trucks)}
        self.kd_dict = {drone_id: k for k, drones in self.drone_dict.items() for drone_id in drones}
        self.total_drone_num = num_trucks * num_drones

        # Additional attributes
        self.num_trucks = num_trucks
        self.num_drones = num_drones
        self.truck_arcs = []
        self.drone_arcs = []
        for node_i, arcs in self.truck_out_arcs.items():
            for node_j in arcs:
                self.truck_arcs.append((node_i, node_j))
        for node_i, arcs in self.drone_out_arcs.items():
            for node_j in arcs:
                self.drone_arcs.append((node_i, node_j + "_prime"))

        for node in self.hubs:
            self.demand_weights[self.all_nodes_indices[node]] = epsilon_weight
