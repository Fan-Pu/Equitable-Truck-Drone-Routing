class Network:
    def __init__(self, depot_source, depot_sink, customers, hubs, all_nodes, all_nodes_indices, customer_indices,
                 hub_indices, truck_net, drone_net, truck_out_arcs, truck_in_arcs, drone_out_arcs, drone_in_arcs,
                 truck_travel_times, drone_travel_times, demand_weights, num_trucks, num_drones, a_lb):
        """
        the class for storing network data
        """

        # Automatically assign all parameters to self
        for name, value in locals().items():
            if name != "self":  # Skip the 'self' parameter
                setattr(self, name, value)
        self.num_customers = len(customers)
        self.num_hubs = len(hubs)
        # set drone_dict and kd_dict
        self.drone_dict = {}
        self.kd_dict = {}  # key: drone_id
        drone_id = 0
        for k in range(num_trucks):
            temp_list = []
            for d in range(num_drones):
                temp_list.append(drone_id)
                self.kd_dict[drone_id] = k
                drone_id += 1
            self.drone_dict[k] = temp_list
        self.total_drone_num = drone_id
        # arrival time lower bound
        self.a_lb = a_lb
