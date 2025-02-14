class Network:
    def __init__(self, num_trucks, num_drones, a_lb, **kwargs):
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
        self.a_lb = a_lb
