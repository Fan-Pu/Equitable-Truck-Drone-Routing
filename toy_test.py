import random

seed = 1
import gurobipy as gp
import matplotlib.pyplot as plt
import networkx as nx
from gurobipy import GRB

drone_min_t = 5
drone_max_t = 10
truck_min_t = 10
truck_max_t = 50
M = 99999
epsilon = 0.01
drone_endurance = 150
drone_max_weight = 500
drone_max_volume = 100
truck_max_weight = 500
truck_max_drone_dock_num = 10

hub_cost_min = 1
hub_cost_max = 2

demand_weight_max = 5
demand_weight_min = 1
demand_volume_max = 3
demand_volume_min = 1


class ToyTest:
    def __init__(self, num_customers, num_hubs, num_trucks, num_drones):
        self.constraints = []
        self.num_customers = num_customers
        self.num_hubs = num_hubs
        self.num_trucks = num_trucks
        self.num_drones = num_drones
        self.seed = 1998  # You can change this seed value for different deterministic outcomes
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
        random.seed(self.seed)  # Set the seed here for reproducibility

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
            if node_name in customers:
                self.customer_indices[node_name] = i
            elif node_name in hubs:
                self.hub_indices[node_name] = i

        # Ensure that each customer and hub has a path from depot_source to current location
        for location in customers + hubs:
            truck_travel_time = random.randint(truck_min_t, truck_max_t)
            drone_travel_time = random.randint(drone_min_t, drone_max_t)
            G.add_edge(depot_source, location, travel_time={'truck': truck_travel_time, 'drone': drone_travel_time})

        # Ensure that each customer and hub has a path to depot_sink
        for location in customers + hubs:
            truck_travel_time = random.randint(truck_min_t, truck_max_t)
            drone_travel_time = random.randint(drone_min_t, drone_max_t)
            G.add_edge(location, depot_sink, travel_time={'truck': truck_travel_time, 'drone': drone_travel_time})

        # Randomly generate additional arcs for truck routes (between depot_source, hubs, and customers)
        all_locations = customers + hubs
        for i in range(len(all_locations)):
            for j in range(len(all_locations)):
                if j == i:
                    continue
                if (i == 2 and j == 4) or (i == 2 and j == 0):
                    truck_travel_time = random.randint(truck_min_t, truck_max_t)
                    drone_travel_time = random.randint(drone_min_t, drone_max_t)
                    G.add_edge(all_locations[i], all_locations[j],
                               travel_time={'truck': truck_travel_time, 'drone': drone_travel_time})
                    G.add_edge(all_locations[j], all_locations[i],
                               travel_time={'truck': truck_travel_time, 'drone': drone_travel_time})
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
                    truck_travel_time = random.randint(truck_min_t, truck_max_t) * 0.1
                    drone_travel_time = random.randint(drone_min_t, drone_max_t) * 0.1
                    G.add_edge(hub, customer, travel_time={'truck': truck_travel_time, 'drone': drone_travel_time})
                    G.add_edge(customer, hub, travel_time={'truck': truck_travel_time, 'drone': drone_travel_time})

        self.G = G

        self.t_lb = self.get_shortest_drone_travel_times()

        self.demand_weights = {}
        self.demand_volume = {}
        for n_name in customers:
            n = self.all_nodes_indices[n_name]
            self.demand_weights[n] = random.uniform(demand_weight_min, demand_weight_max)
            self.demand_volume[n] = random.uniform(demand_volume_min, demand_volume_max)
        for s_name in hubs:
            s = self.all_nodes_indices[s_name]
            self.demand_weights[s] = self.demand_volume[s] = epsilon
        print(self.demand_weights)
        print(self.demand_volume)

    def get_shortest_drone_travel_times(self):
        # Create a dictionary of edge weights based on drone travel time
        drone_travel_times = {
            (u, v): d['travel_time']['drone']
            for u, v, d in self.G.edges(data=True)
        }

        # Set the drone travel time as the weight of the edges in the graph
        nx.set_edge_attributes(self.G, drone_travel_times, 'weight')

        # Calculate the shortest travel time from depot_source to each customer using Dijkstra's algorithm
        shortest_drone_times = {}
        for customer in self.customers:
            try:
                # Use Dijkstra's algorithm to get the shortest path and travel time
                path_length = nx.single_source_dijkstra_path_length(self.G, self.depot_source, weight='weight')[
                    customer]
                shortest_drone_times[customer] = path_length
            except KeyError:
                # If there's no path to the customer, we can store inf or some other indication
                shortest_drone_times[customer] = float('inf')

        return shortest_drone_times

    def visualize(self):
        pos = nx.spring_layout(self.G, seed=self.seed)  # Position the nodes using spring layout with a fixed seed
        fig, ax = plt.subplots()
        plt.sca(ax)
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
        nx.draw(self.G, pos, with_labels=True, node_color=node_colors, node_size=500, font_size=10, font_weight='bold',
                edge_color='gray')

        # Create custom labels for nodes (you can adjust this to display specific attributes)
        node_labels = {}
        for node in self.G.nodes():
            if node in self.customers:
                n = self.all_nodes_indices[node]
                node_labels[node] = f"w:{self.demand_weights[n]:.2f}|v:{self.demand_volume[n]:.2f}"

        label_pos = {node: (x, y + 0.05) for node, (x, y) in pos.items()}  # Adjust 0.05 to control the offset
        # Draw the node labels
        nx.draw_networkx_labels(self.G, label_pos, labels=node_labels, font_size=10)

        # Create labels for travel times on edges
        edge_labels = {}
        for u, v, data in self.G.edges(data=True):
            travel_times = data['travel_time']
            # Format the travel times for display
            edge_labels[(u, v)] = f"T: {travel_times['truck']:.2f} | D: {travel_times['drone']:.2f}"

        # Draw edge labels (travel times for both truck and drone)
        nx.draw_networkx_edge_labels(self.G, pos, edge_labels=edge_labels, font_size=8)

    def solve(self):
        model = gp.Model("model")
        # add decision variables
        x_list = {}
        y_list = {}
        f_list = {}
        z_list = {}
        xi_list = {}
        for s_name in self.hubs:
            xi = random.randint(hub_cost_min, hub_cost_max)
            xi_list[s_name] = xi
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
        u_list = {}
        for s_name in self.hubs:
            s = self.all_nodes_indices[s_name]
            u_list[s] = model.addVar(name=f"u_{s}", vtype=GRB.BINARY)
        # continuous variables
        t_list = {}
        w_list = {}
        v_list = {}
        c_list = {}
        ad_list = {}
        ak_list = {}
        for n in range(len(self.all_nodes_indices)):
            for d in range(self.num_drones):
                t_list[(n, d)] = model.addVar(name=f"t_{(n, d)}", vtype=GRB.CONTINUOUS, lb=0)
                w_list[(n, d)] = model.addVar(name=f"w_{(n, d)}", vtype=GRB.CONTINUOUS, lb=0)
                v_list[(n, d)] = model.addVar(name=f"v_{(n, d)}", vtype=GRB.CONTINUOUS, lb=0)
                ad_list[(n, d)] = model.addVar(name=f"ad_{(n, d)}", vtype=GRB.CONTINUOUS, lb=0)
            for k in range(self.num_trucks):
                c_list[(n, k)] = model.addVar(name=f"c_{(n, k)}", vtype=GRB.CONTINUOUS, lb=0)
                ak_list[(n, k)] = model.addVar(name=f"ak_{(n, k)}", vtype=GRB.CONTINUOUS, lb=0)
        # upper bound of arriving time
        a_max = model.addVar(name="a_max", vtype=GRB.CONTINUOUS, lb=0)

        # add objective function
        obj_expr = 0
        # for i_name, j_name, data in edges:
        #     i = self.all_nodes_indices[i_name]
        #     j = self.all_nodes_indices[j_name]
        #     truck_time = data['travel_time']['truck']
        #     drone_time = data['travel_time']['drone']
        #     for k in range(self.num_trucks):
        #         obj_expr += truck_time * (x_list[(i, j, k)] + f_list[(i, j, k)])
        #     for d in range(self.num_drones):
        #         obj_expr += drone_time * y_list[(i, j, d)]

        for s_name in self.hubs:
            s = self.all_nodes_indices[s_name]
            obj_expr += xi_list[s_name] * u_list[s]

        for n_name in self.customers:
            n = self.all_nodes_indices[n_name]
            arrive_time = 0
            for k in range(self.num_trucks):
                arrive_time += ak_list[(n, k)]
            for d in range(self.num_drones):
                arrive_time += ad_list[(n, d)]
            obj_expr += arrive_time - self.t_lb[n_name]

        # obj_expr += a_max

        model.setObjective(obj_expr, GRB.MINIMIZE)

        # objective linearization ************************************************************
        id = 0
        for k in range(self.num_trucks):
            self.constraints.append(
                model.addConstr(a_max >= ak_list[(self.all_nodes_indices[self.depot_sink], k)], f"obj_{id}"))
            id += 1
        for d in range(self.num_drones):
            self.constraints.append(
                model.addConstr(a_max >= ad_list[(self.all_nodes_indices[self.depot_sink], d)], f"obj_{id}"))
            id += 1

        # flow conservation ************************************************************
        id = 0
        # cons 1
        out_arcs = self.G.out_edges(self.depot_source)
        in_arcs = self.G.in_edges(self.depot_sink)
        for k in range(self.num_trucks):
            lhs = rhs = 0
            for i_name, j_name in out_arcs:
                i = self.all_nodes_indices[i_name]
                j = self.all_nodes_indices[j_name]
                lhs += x_list[(i, j, k)] + f_list[i, j, k]
            for i_name, j_name in in_arcs:
                i = self.all_nodes_indices[i_name]
                j = self.all_nodes_indices[j_name]
                rhs += x_list[(i, j, k)] + f_list[i, j, k]
            self.constraints.append(model.addConstr(lhs == rhs, f"conserv1_{id}"))
            id += 1
        # cons 2
        for d in range(self.num_drones):
            lhs = rhs = 0
            for i_name, j_name in out_arcs:
                i = self.all_nodes_indices[i_name]
                j = self.all_nodes_indices[j_name]
                sum_z = 0
                for k in range(self.num_trucks):
                    sum_z += z_list[(i, j, k, d)]
                lhs += y_list[(i, j, d)] + sum_z
            for i_name, j_name in in_arcs:
                i = self.all_nodes_indices[i_name]
                j = self.all_nodes_indices[j_name]
                sum_z = 0
                for k in range(self.num_trucks):
                    sum_z += z_list[(i, j, k, d)]
                rhs += y_list[(i, j, d)] + sum_z
            self.constraints.append(model.addConstr(lhs == rhs, f"conserv2_{i}"))
            i += 1
        # cons 3
        id = 0
        for n_name in self.all_nodes[1:-1]:
            out_arcs = self.G.out_edges(n_name)
            in_arcs = self.G.in_edges(n_name)
            n = self.all_nodes_indices[n_name]
            for k in range(self.num_trucks):
                lhs = rhs = 0
                # for lhs
                for _, j_name in out_arcs:
                    j = self.all_nodes_indices[j_name]
                    lhs += x_list[(n, j, k)] + f_list[(n, j, k)]
                # for rhs
                for j_name, _ in in_arcs:
                    j = self.all_nodes_indices[j_name]
                    rhs += x_list[(j, n, k)] + f_list[(j, n, k)]
                self.constraints.append(model.addConstr(lhs == rhs, f"conserv3_{id}"))
                id += 1
        # cons 4
        id = 0
        for n_name in self.all_nodes[1:-1]:
            n = self.all_nodes_indices[n_name]
            out_arcs = self.G.out_edges(n_name)
            in_arcs = self.G.in_edges(n_name)
            for d in range(self.num_drones):
                lhs = rhs = 0
                # for lhs
                for _, j_name in out_arcs:
                    j = self.all_nodes_indices[j_name]
                    sum_z = 0
                    for k in range(self.num_trucks):
                        sum_z += z_list[(n, j, k, d)]
                    lhs += y_list[(n, j, d)] + sum_z
                # for rhs
                for j_name, _ in in_arcs:
                    j = self.all_nodes_indices[j_name]
                    sum_z = 0
                    for k in range(self.num_trucks):
                        sum_z += z_list[(j, n, k, d)]
                    rhs += y_list[(j, n, d)] + sum_z
                self.constraints.append(model.addConstr(lhs == rhs, f"conserv4_{id}"))
                id += 1

        # customer serve once ************************************************************
        id = 0
        for n_name in self.customers:
            lhs = 0
            n = self.all_nodes_indices[n_name]
            in_arcs = self.G.in_edges(n_name)
            for i_name, _ in in_arcs:
                i = self.all_nodes_indices[i_name]
                for k in range(self.num_trucks):
                    lhs += x_list[(i, n, k)] + f_list[(i, n, k)]
                for d in range(self.num_drones):
                    lhs += y_list[(i, n, d)]
            rhs = 1
            self.constraints.append(model.addConstr(lhs == rhs, f"servonce_{id}"))
            id += 1

        # truck drone use once ************************************************************
        for k in range(self.num_trucks):
            lhs = rhs = 0
            out_arcs = self.G.out_edges(self.depot_source)
            for i_name, j_name in out_arcs:
                i = self.all_nodes_indices[i_name]
                j = self.all_nodes_indices[j_name]
                lhs += x_list[(i, j, k)] + f_list[i, j, k]
            self.constraints.append(model.addConstr(lhs <= 1, f"truck_once_{i}"))
            i += 1
        for d in range(self.num_drones):
            lhs = rhs = 0
            out_arcs = self.G.out_edges(self.depot_source)
            for i_name, j_name in out_arcs:
                i = self.all_nodes_indices[i_name]
                j = self.all_nodes_indices[j_name]
                lhs += y_list[(i, j, d)]
                for k in range(self.num_trucks):
                    lhs += z_list[(i, j, k, d)]
            self.constraints.append(model.addConstr(lhs <= 1, f"truck_once_{i}"))
            i += 1

        # linking constraints ************************************************************
        id = 0
        for i_name, j_name, _ in edges:
            i = self.all_nodes_indices[i_name]
            j = self.all_nodes_indices[j_name]
            # cons 1,2
            for k in range(self.num_trucks):
                sum_z = 0
                for d in range(self.num_drones):
                    sum_z += z_list[(i, j, k, d)]
                lhs = 1 + M * (f_list[(i, j, k)] - 1)
                rhs = M * f_list[(i, j, k)]
                # cons 1
                self.constraints.append(model.addConstr(lhs <= sum_z, f"linking_{id}"))
                id += 1
                self.constraints.append(model.addConstr(sum_z <= rhs, f"linking_{id}"))
                id += 1
                # cons 2
                lhs = x_list[(i, j, k)] + f_list[(i, j, k)]
                rhs = 1
                self.constraints.append(model.addConstr(lhs <= rhs, f"linking_{id}"))
                id += 1
            # cons 3
            for d in range(self.num_drones):
                sum_z = 0
                for k in range(self.num_trucks):
                    sum_z += z_list[(i, j, k, d)]
                lhs = y_list[(i, j, d)] + sum_z
                rhs = 1
                self.constraints.append(model.addConstr(lhs <= rhs, f"linking_{id}"))
                id += 1
        # cons 4
        for n_name in self.customers:
            n = self.customer_indices[n_name]
            out_arcs = self.G.out_edges(n_name)
            for _, j_name in out_arcs:
                j = self.all_nodes_indices[j_name]
                for k in range(self.num_trucks):
                    for d in range(self.num_drones):
                        sum_y = 0
                        in_arcs = self.G.in_edges(n_name)
                        for i_name, _ in in_arcs:
                            i = self.all_nodes_indices[i_name]
                            sum_y += y_list[(i, n, d)]
                            rhs = M * (1 - z_list[(n, j, k, d)])
                            self.constraints.append(model.addConstr(sum_y <= rhs, f"linking_{id}"))
                            id += 1

        # sync point opening ************************************************************
        id = 0
        for s_name in self.hubs:
            s = self.all_nodes_indices[s_name]
            lhs = rhs = 0
            in_arcs = self.G.in_edges(s_name)
            for i_name, _ in in_arcs:
                i = self.all_nodes_indices[i_name]
                for k in range(self.num_trucks):
                    lhs += x_list[(i, s, k)] + f_list[(i, s, k)]
                    for d in range(self.num_drones):
                        lhs += z_list[(i, s, k, d)]
                for d in range(self.num_drones):
                    lhs += y_list[(i, s, d)]
            rhs = M * u_list[s]
            self.constraints.append(model.addConstr(lhs <= rhs, f"sync_{id}"))
            id += 1

        # drone flight endurance ************************************************************
        id = 0
        # cons 1
        for n_name in self.all_nodes:
            n = self.all_nodes_indices[n_name]
            if n_name == self.depot_source:
                continue
            for d in range(self.num_drones):
                self.constraints.append(model.addConstr(t_list[(n, d)] <= drone_endurance, f"drone_endu1_{id}"))
                id += 1
        # cons 2
        for n_name in self.customers:
            n = self.all_nodes_indices[n_name]
            out_arcs = self.G.out_edges(n_name, data=True)
            for _, j_name, data in out_arcs:
                j = self.all_nodes_indices[j_name]
                travel_time = data["travel_time"]["drone"]
                for d in range(self.num_drones):
                    rhs = t_list[(n, d)] + travel_time + M * (y_list[(n, j, d)] - 1)
                    self.constraints.append(model.addConstr(t_list[(j, d)] >= rhs, f"drone_endu2_{id}"))
                    id += 1
        # cons 3
        for n_name in self.hubs + [self.depot_source]:
            n = self.all_nodes_indices[n_name]
            out_arcs = self.G.out_edges(n_name, data=True)
            for _, j_name, data in out_arcs:
                j = self.all_nodes_indices[j_name]
                travel_time = data["travel_time"]["drone"]
                for d in range(self.num_drones):
                    rhs = travel_time + M * (y_list[(n, j, d)] - 1)
                    self.constraints.append(model.addConstr(t_list[(j, d)] >= rhs, f"drone_endu3_{id}"))
                    id += 1

        # drone payload weight and volume ************************************************************
        id = 0
        # cons 1
        for n_name in self.customers:
            n = self.all_nodes_indices[n_name]
            for d in range(self.num_drones):
                self.constraints.append(model.addConstr(w_list[(n, d)] <= drone_max_weight, f"drone_load1_{id}"))
                id += 1
        # cons 2
        for n_name in self.customers + self.hubs:
            n = self.all_nodes_indices[n_name]
            in_arcs = self.G.in_edges(n_name)
            for i_name, _ in in_arcs:
                i = self.all_nodes_indices[i_name]
                for d in range(self.num_drones):
                    rhs = w_list[(i, d)] + self.demand_weights[n] + M * (y_list[(i, n, d)] - 1)
                    self.constraints.append(model.addConstr(w_list[(n, d)] >= rhs, f"drone_load2_{id}"))
                    id += 1
        # cons 3
        for n_name in self.customers:
            n = self.all_nodes_indices[n_name]
            for d in range(self.num_drones):
                self.constraints.append(model.addConstr(v_list[(n, d)] <= drone_max_volume, f"drone_load3_{id}"))
                id += 1
        # cons 4
        for n_name in self.customers + self.hubs:
            n = self.all_nodes_indices[n_name]
            in_arcs = self.G.in_edges(n_name)
            for i_name, _ in in_arcs:
                i = self.all_nodes_indices[i_name]
                for d in range(self.num_drones):
                    rhs = v_list[(i, d)] + self.demand_volume[n] + M * (y_list[(i, n, d)] - 1)
                    self.constraints.append(model.addConstr(v_list[(n, d)] >= rhs, f"drone_load4_{id}"))
                    id += 1

        # truck capacity ************************************************************
        id = 0
        # cons 1
        for n_name in self.customers:
            n = self.all_nodes_indices[n_name]
            for k in range(self.num_trucks):
                self.constraints.append(model.addConstr(c_list[(n, k)] <= truck_max_weight, f"truck_load1_{id}"))
                id += 1
        # cons 2
        for n_name in self.customers + self.hubs:
            n = self.all_nodes_indices[n_name]
            in_arcs = self.G.in_edges(n_name)
            for i_name, _ in in_arcs:
                i = self.all_nodes_indices[i_name]
                for k in range(self.num_trucks):
                    rhs = c_list[(i, k)] + self.demand_weights[n] + M * (x_list[(i, n, k)] + f_list[(i, n, k)] - 1)
                    self.constraints.append(model.addConstr(c_list[(n, k)] >= rhs, f"truck_load2_{id}"))
                    id += 1
        # cons 3
        for i_name, j_name, _ in edges:
            i = self.all_nodes_indices[i_name]
            j = self.all_nodes_indices[j_name]
            for k in range(self.num_trucks):
                lhs = 0
                for d in range(self.num_drones):
                    lhs += z_list[(i, j, k, d)]
                self.constraints.append(model.addConstr(lhs <= truck_max_drone_dock_num, f"truck_load3_{id}"))
                id += 1

        # drone truck synchronization ************************************************************
        id = 0
        # cons 1 and 2
        for n_name in self.customers + self.hubs + [self.depot_source]:
            if n_name == self.depot_sink:
                continue
            n = self.all_nodes_indices[n_name]
            out_arcs = self.G.out_edges(n_name, data=True)
            for _, j_name, data in out_arcs:
                j = self.all_nodes_indices[j_name]
                for d in range(self.num_drones):
                    # cons 1
                    travel_time = data["travel_time"]["drone"]
                    rhs = ad_list[(n, d)] + travel_time + M * (y_list[(n, j, d)] - 1)
                    self.constraints.append(model.addConstr(ad_list[(j, d)] >= rhs, f"sync1_{id}"))
                    id += 1
                for k in range(self.num_trucks):
                    # cons 2
                    travel_time = data["travel_time"]["truck"]
                    rhs = ak_list[(n, k)] + travel_time + M * (x_list[(n, j, k)] + f_list[(n, j, k)] - 1)
                    self.constraints.append(model.addConstr(ak_list[(j, k)] >= rhs, f"sync2_{id}"))
                    id += 1
        # cons 3 and 4
        for s_name in self.hubs:
            s = self.all_nodes_indices[s_name]
            out_arcs = self.G.out_edges(s_name, data=True)
            for _, j_name, data in out_arcs:
                j = self.all_nodes_indices[j_name]
                for k in range(self.num_trucks):
                    for d in range(self.num_drones):
                        # cons 3
                        rhs = ak_list[(s, k)] - M * (1 - z_list[(s, j, k, d)])
                        self.constraints.append(model.addConstr(ad_list[(s, d)] >= rhs, f"sync3_{id}"))
                        id += 1
                        # cons 4
                        travel_time = data["travel_time"]["truck"]
                        rhs = ad_list[(s, d)] + travel_time - M * (1 - z_list[(s, j, k, d)])
                        self.constraints.append(model.addConstr(ak_list[(j, k)] >= rhs, f"sync4_{id}"))
                        id += 1

        # test
        # self.constraints.append(model.addConstr(z_list[(2, 3, 0, 0)] == 1))
        # self.constraints.append(model.addConstr(y_list[(3, 5, 0)] == 1))
        # self.constraints.append(model.addConstr(x_list[(3, 7, 0)] == 1))
        # self.constraints.append(model.addConstr(z_list[(7, 8, 0, 0)] == 1))

        # self.constraints.append(model.addConstr(y_list[(3, 1, 0)] == 1))
        # self.constraints.append(model.addConstr(y_list[(1, 4, 0)] == 1))

        model.update()
        model.write("model.lp")
        model.optimize()
        if model.status == GRB.OPTIMAL:
            print(f"Optimal solution found")
        elif model.status == GRB.INFEASIBLE:
            model.computeIIS()
            model.write("model.ilp")
            print("No feasible solution found")
        elif model.status == GRB.UNBOUNDED:
            print("The model is unbounded")

        # Retrieve the objective value
        if model.status == GRB.OPTIMAL:
            print(f"Objective value: {model.objVal}")
            # Extract and store the solution values for the decision variables
            self.x_values = {(i, j, k): var.X for (i, j, k), var in x_list.items()}
            self.y_values = {(i, j, d): var.X for (i, j, d), var in y_list.items()}
            self.f_values = {(i, j, d): var.X for (i, j, d), var in f_list.items()}
            self.z_values = {(i, j, k, d): var.X for (i, j, k, d), var in z_list.items()}
            self.u_values = {s: var.X for s, var in u_list.items()}
            self.t_values = {(n, d): var.X for (n, d), var in t_list.items()}
            self.w_values = {(n, d): var.X for (n, d), var in w_list.items()}
            self.v_values = {(n, d): var.X for (n, d), var in v_list.items()}
            self.c_values = {(n, k): var.X for (n, k), var in c_list.items()}
            self.ak_value = {(n, k): var.X for (n, k), var in ak_list.items()}
            self.ad_value = {(n, d): var.X for (n, d), var in ad_list.items()}
            sad = 0

    def visualize_routes(self):
        pos = nx.spring_layout(self.G, seed=self.seed)  # Use the same layout for consistency

        # Draw the network without routes first
        node_colors = []
        for node in self.G.nodes():
            if node == self.depot_source or node == self.depot_sink:
                node_colors.append('red')  # Red for depot
            elif node in self.customers:
                node_colors.append('yellow')  # Yellow for customers
            elif node in self.hubs:
                node_colors.append('green')  # Green for hubs
            else:
                node_colors.append('gray')  # Default color for others (if any)

        # truck route
        fig, ax = plt.subplots()
        plt.sca(ax)
        nx.draw_networkx_nodes(self.G, pos, node_color=node_colors, node_size=1000)
        nx.draw_networkx_labels(self.G, pos, font_size=10, font_weight='bold')

        truck_edges = {k: [] for k in range(self.num_trucks)}
        drone_edges = {d: [] for d in range(self.num_drones)}

        # truck routes
        for (i, j, k), val in self.x_values.items():
            if val > 0.5:
                truck_edges[k].append((self.all_nodes[i], self.all_nodes[j]))
        for (i, j, k), val in self.f_values.items():
            if val > 0.5:
                truck_edges[k].append((self.all_nodes[i], self.all_nodes[j]))
        for (i, j, d), val in self.y_values.items():
            if val > 0.5:
                drone_edges[d].append((self.all_nodes[i], self.all_nodes[j]))
        for (i, j, k, d), val in self.z_values.items():
            if val > 0.5:
                drone_edges[d].append((self.all_nodes[i], self.all_nodes[j]))

        cmap = plt.get_cmap("viridis")
        # Generate n colors from the colormap
        total_num = self.num_drones + self.num_trucks
        colors = [cmap(i / total_num) for i in range(total_num)]  # Ensure we don't exceed colormap range
        for i in range(self.num_trucks):
            nx.draw_networkx_edges(self.G, pos, edgelist=truck_edges[i],
                                   edge_color=[colors[i]] * len(truck_edges[i]), width=2,
                                   label=f"T{i}", arrows=True, arrowsize=20)
        # Add a legend
        plt.legend()

        # drone route
        fig, ax = plt.subplots()
        plt.sca(ax)
        nx.draw_networkx_nodes(self.G, pos, node_color=node_colors, node_size=1000)
        nx.draw_networkx_labels(self.G, pos, font_size=10, font_weight='bold')
        for i in range(self.num_drones):
            nx.draw_networkx_edges(self.G, pos, edgelist=drone_edges[i],
                                   edge_color=[colors[i + self.num_trucks]] * len(drone_edges[i]),
                                   width=2, label=f"D{i}", arrows=True, arrowsize=20, style='dashed')
        # Add a legend
        plt.legend()
