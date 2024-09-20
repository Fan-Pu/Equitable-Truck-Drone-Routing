import random

import gurobipy as gp
import matplotlib.pyplot as plt
import networkx as nx
from gurobipy import GRB

drone_min_t = 5
drone_max_t = 20
truck_min_t = 10
truck_max_t = 50


class ToyTest:
    def __init__(self, num_customers, num_hubs, num_trucks, num_drones):
        random.seed(1998)
        self.num_customers = num_customers
        self.num_hubs = num_hubs
        self.num_trucks = num_trucks
        self.num_drones = num_drones
        self.create_random_truck_drone_network(num_customers, num_hubs)

    def create_random_truck_drone_network(self, num_customers, num_hubs):
        """
        Creates a random graph to represent the truck-drone routing problem.

        Args:
            num_customers (int): Number of customer locations.
            num_hubs (int): Number of docking hub locations.

        Returns:
            G (nx.DiGraph): Directed graph representing the network.
        """
        G = nx.DiGraph()

        # Generate node names
        depot_source = "Source"
        depot_sink = "Sink"
        customers = [f"C_{i + 1}" for i in range(num_customers)]
        hubs = [f"Hub_{i + 1}" for i in range(num_hubs)]

        self.depot_source = depot_source
        self.depot_sink = depot_sink
        self.customers = customers
        self.hubs = hubs

        # Add depot, customers, and hubs as nodes
        G.add_node(depot_source)
        G.add_node(depot_sink)
        for customer in customers:
            G.add_node(customer)
        for hub in hubs:
            G.add_node(hub)

        self.all_nodes = [depot_source] + customers + hubs + [depot_sink]
        self.customer_indices = {}
        self.hub_indices = {}
        self.all_nodes_indices = {}
        for i in range(len(self.all_nodes)):
            node_name = self.all_nodes[i]
            self.all_nodes_indices[node_name] = i
            self.customer_indices[node_name] = i
            if node_name in customers:
                self.customer_indices[node_name] = i
            elif node_name in hubs:
                self.hub_indices[node_name] = i

        # Ensure that each customer and hub has a path from depot_source and to depot_sink
        # Add edges from depot_source to at least one customer or hub
        for location in customers + hubs:
            truck_travel_time = random.randint(truck_min_t, truck_max_t)
            drone_travel_time = random.randint(drone_min_t, drone_max_t)
            G.add_edge(depot_source, location, travel_time={'truck': truck_travel_time, 'drone': drone_travel_time})

        # Add edges from at least one customer or hub to depot_sink
        for location in customers + hubs:
            truck_travel_time = random.randint(truck_min_t, truck_max_t)
            drone_travel_time = random.randint(drone_min_t, drone_max_t)
            G.add_edge(location, depot_sink, travel_time={'truck': truck_travel_time, 'drone': drone_travel_time})

        # Randomly generate additional arcs for truck routes (between depot_source, hubs, and customers)
        all_locations = customers + hubs
        for i in range(len(all_locations)):
            for j in range(i + 1, len(all_locations)):
                if random.random() > 0.5:  # Randomly decide if a arc exists
                    truck_travel_time = random.randint(truck_min_t, truck_max_t)
                    drone_travel_time = random.randint(drone_min_t, drone_max_t)
                    G.add_edge(all_locations[i], all_locations[j],
                               travel_time={'truck': truck_travel_time, 'drone': drone_travel_time})
                    G.add_edge(all_locations[j], all_locations[i],
                               travel_time={'truck': truck_travel_time, 'drone': drone_travel_time})

        # Randomly generate arcs for drone routes (between hubs and customers)
        for hub in hubs:
            for customer in customers:
                if random.random() > 0.5:  # Randomly decide if a drone arc exists
                    truck_travel_time = random.randint(truck_min_t, truck_max_t)
                    drone_travel_time = random.randint(drone_min_t, drone_max_t)
                    G.add_edge(hub, customer, travel_time={'truck': truck_travel_time, 'drone': drone_travel_time})
                    G.add_edge(customer, hub, travel_time={'truck': truck_travel_time, 'drone': drone_travel_time})

        self.G = G

    def visualize(self):
        pos = nx.spring_layout(self.G)  # Position the nodes using spring layout

        # Color mapping: Different colors for depot, customers, and hubs
        node_colors = []
        for node in self.G.nodes():
            if node == self.depot_source or node == self.depot_sink:
                node_colors.append('red')  # Red for depot
            elif node in self.customers:
                node_colors.append('yellow')  # Blue for customers
            elif node in self.hubs:
                node_colors.append('green')  # Green for hubs
            else:
                node_colors.append('gray')  # Default color for others (if any)

        # Draw the nodes and edges
        nx.draw(self.G, pos, with_labels=True, node_color=node_colors, node_size=1000, font_size=10, font_weight='bold',
                edge_color='gray')

        # Create labels for travel times on edges
        edge_labels = {}
        for u, v, data in self.G.edges(data=True):
            travel_times = data['travel_time']
            # Format the travel times for display
            edge_labels[(u, v)] = f"T: {travel_times['truck']} | D: {travel_times['drone']}"

        # Draw edge labels (travel times for both truck and drone)
        nx.draw_networkx_edge_labels(self.G, pos, edge_labels=edge_labels, font_size=8)
        plt.show()

    def solve(self):
        model = gp.Model("model")
        self.constraints = []
        # add decision variables
        u_list = {}
        for s in range(self.num_hubs):
            u_list[s] = model.addVar(name=f"u_{s}", vtype=GRB.BINARY)
        x_list = {}
        y_list = {}
        f_list = {}
        z_list = {}
        edges = self.G.edges(data=True)
        for i_name, j_name, data in edges:
            i = self.all_nodes_indices[i_name]
            j = self.all_nodes_indices[j_name]
            for k in range(self.num_trucks):
                x_list[(i, j, k)] = model.addVar(name=f"x_{(i, j, k)}", vtype=GRB.BINARY)
                f_list[(i, j, k)] = model.addVar(name=f"f_{(i, j, k)}", vtype=GRB.BINARY)
                for d in range(self.num_drones):
                    z_list[(i, j, k, d)] = model.addVar(name=f"z_{(i, j, k, d)}", vtype=GRB.BINARY)
            for d in range(self.num_drones):
                y_list[(i, j, d)] = model.addVar(name=f"y_{(i, j, d)}", vtype=GRB.BINARY)
        # continuous variables
        t_list = {}
        w_list = {}
        v_list = {}
        c_list = {}
        for n in range(1, len(self.all_nodes_indices)):
            for d in range(self.num_drones):
                t_list[(n, d)] = model.addVar(name=f"t_{(n, d)}", vtype=GRB.CONTINUOUS, lb=0)
                w_list[(n, d)] = model.addVar(name=f"w_{(n, d)}", vtype=GRB.CONTINUOUS, lb=0)
                v_list[(n, d)] = model.addVar(name=f"v_{(n, d)}", vtype=GRB.CONTINUOUS, lb=0)
                c_list[(n, d)] = model.addVar(name=f"c_{(n, d)}", vtype=GRB.CONTINUOUS, lb=0)

        # add objective function
        obj_expr = 0
        for i_name, j_name, data in edges:
            i = self.all_nodes_indices[i_name]
            j = self.all_nodes_indices[j_name]
            truck_time = data['travel_time']['truck']
            drone_time = data['travel_time']['drone']
            for k in range(self.num_trucks):
                obj_expr += truck_time * (x_list[(i, j, k)] + f_list[(i, j, k)])
            for d in range(self.num_drones):
                obj_expr += drone_time * y_list[(i, j, d)]
        model.setObjective(obj_expr, GRB.MINIMIZE)

        # flow conservation
        lhs = rhs = 0
        source_id = 0
        sink_id = len(self.all_nodes_indices) - 1
        # cons 1
        out_arcs = self.G.out_edges(self.depot_source)
        in_arcs = self.G.in_edges(self.depot_sink)
        for i_name, j_name in out_arcs:
            i = self.all_nodes_indices[i_name]
            j = self.all_nodes_indices[j_name]
            for k in range(self.num_trucks):
                lhs += x_list[(i, j, k)] + f_list[i, j, k]
        for i_name, j_name in in_arcs:
            i = self.all_nodes_indices[i_name]
            j = self.all_nodes_indices[j_name]
            for k in range(self.num_trucks):
                rhs += x_list[(i, j, k)] + f_list[i, j, k]
        self.constraints.append(model.addConstr(lhs == rhs, "conserv1"))
        # cons 2
        lhs = rhs = 0
        for i_name, j_name in out_arcs:
            i = self.all_nodes_indices[i_name]
            j = self.all_nodes_indices[j_name]
            for d in range(self.num_drones):
                sum_z = 0
                for k in range(self.num_trucks):
                    sum_z += z_list[(i, j, k, d)]
                lhs += y_list[(i, j, d)] + sum_z
        for i_name, j_name in in_arcs:
            i = self.all_nodes_indices[i_name]
            j = self.all_nodes_indices[j_name]
            for d in range(self.num_drones):
                sum_z = 0
                for k in range(self.num_trucks):
                    sum_z += z_list[(i, j, k, d)]
                rhs += y_list[(i, j, d)] + sum_z
        self.constraints.append(model.addConstr(lhs == rhs, "conserv2"))
        # cons 3
        id = 0
        for n_name in self.all_nodes[1:-1]:
            n = self.all_nodes_indices[n_name]
            lhs = rhs = 0
            out_arcs = self.G.out_edges(n_name)
            in_arcs = self.G.in_edges(n_name)
            # for lhs
            for _, j_name in out_arcs:
                j = self.all_nodes_indices[j_name]
                for k in range(self.num_trucks):
                    lhs += x_list[(n, j, k)] + f_list[(n, j, k)]
            # for rhs
            for j_name, _ in in_arcs:
                j = self.all_nodes_indices[j_name]
                for k in range(self.num_trucks):
                    rhs += x_list[(j, n, k)] + f_list[(j, n, k)]
            self.constraints.append(model.addConstr(lhs == rhs, f"conserv3_{id}"))
            id += 1
        # cons 4
        id = 0
        for n_name in self.all_nodes[1:-1]:
            n = self.all_nodes_indices[n_name]
            lhs = rhs = 0
            out_arcs = self.G.out_edges(n_name)
            in_arcs = self.G.in_edges(n_name)
            # for lhs
            for _, j_name in out_arcs:
                j = self.all_nodes_indices[j_name]
                for d in range(self.num_drones):
                    sum_z = 0
                    for k in range(self.num_trucks):
                        sum_z += z_list[(n, j, k, d)]
                    lhs += y_list[(n, j, d)] + sum_z
            # for rhs
            for j_name, _ in in_arcs:
                j = self.all_nodes_indices[j_name]
                for d in range(self.num_drones):
                    sum_z = 0
                    for k in range(self.num_trucks):
                        sum_z += z_list[(j, n, k, d)]
                    rhs += y_list[(j, n, d)] + sum_z
            self.constraints.append(model.addConstr(lhs == rhs, f"conserv4_{id}"))
            id += 1

        # customer serve once
        lhs = rhs = 0
        for n_name in self.customers:
            n = self.all_nodes_indices[n_name]
            
            sdsa = 0

        sdas = 0
