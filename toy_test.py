import random

import gurobipy as gp
import matplotlib.cm as cm
import matplotlib.pyplot as plt
import networkx as nx
from gurobipy import GRB

drone_min_t = 1
drone_max_t = 2

truck_min_t = 10
truck_max_t = 50

M = 10000
epsilon = 0.01

demand_weight_max = 10
demand_weight_min = 1

drone_endurance = 150
drone_max_weight = 5

truck_max_weight = 100
truck_max_drone_dock_num = 10

# for plot only
node_size = 500


class ToyTest:
    def __init__(self, num_customers, num_hubs, num_trucks, num_drones):
        self.constraints = []
        self.num_customers = num_customers
        self.num_hubs = num_hubs
        self.num_trucks = num_trucks
        self.num_drones = num_drones
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
        self.seed = 2024  # You can change this seed value for different deterministic outcomes
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
        random.seed(self.seed)
        G = nx.DiGraph()
        # Generate node names
        depot_source = "Source"
        depot_sink = "Sink"
        customers = [f"C{i + 1}" for i in range(num_customers)]
        hubs = [f"H{i + 1}" for i in range(num_hubs)]

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

        # Ensure that each customer and hub has a path from depot_source
        for location in customers + hubs:
            truck_travel_time = random.randint(truck_min_t, truck_max_t)
            drone_travel_time = random.randint(drone_min_t, drone_max_t)
            G.add_edge(depot_source, location, travel_time={'truck': truck_travel_time, 'drone': drone_travel_time})

        # Ensure that each customer and hub has a path to depot_sink
        for location in customers + hubs:
            truck_travel_time = random.randint(truck_min_t, truck_max_t)
            drone_travel_time = random.randint(drone_min_t, drone_max_t)
            G.add_edge(location, depot_sink, travel_time={'truck': truck_travel_time, 'drone': drone_travel_time})

        # make each hub has access to all other places
        for location in hubs:
            # to customers
            for term in customers:
                truck_travel_time = random.randint(truck_min_t, truck_max_t)
                drone_travel_time = random.randint(drone_min_t, drone_max_t)
                G.add_edge(location, term,
                           travel_time={'truck': truck_travel_time, 'drone': drone_travel_time})
            # to depot sink
            if not G.has_edge(location, depot_sink):
                truck_travel_time = random.randint(truck_min_t, truck_max_t)
                drone_travel_time = random.randint(drone_min_t, drone_max_t)
                G.add_edge(location, depot_sink,
                           travel_time={'truck': truck_travel_time, 'drone': drone_travel_time})

        # Randomly generate additional arcs for truck routes (between depot_source, hubs, and customers)
        all_locations = customers + hubs
        for i in range(len(all_locations)):
            for j in range(len(all_locations)):
                if j == i:
                    continue
                # each customer node connects hubs
                if i in customers and j in hubs:
                    truck_travel_time = random.randint(truck_min_t, truck_max_t)
                    drone_travel_time = random.randint(drone_min_t, drone_max_t)
                    G.add_edge(all_locations[i], all_locations[j],
                               travel_time={'truck': truck_travel_time, 'drone': drone_travel_time})
                    G.add_edge(all_locations[j], all_locations[i],
                               travel_time={'truck': truck_travel_time, 'drone': drone_travel_time})
                elif random.random() > 0.5:  # Randomly decide if a arc exists
                    truck_travel_time = random.randint(truck_min_t, truck_max_t)
                    drone_travel_time = random.randint(drone_min_t, drone_max_t)
                    G.add_edge(all_locations[i], all_locations[j],
                               travel_time={'truck': truck_travel_time, 'drone': drone_travel_time})
                    G.add_edge(all_locations[j], all_locations[i],
                               travel_time={'truck': truck_travel_time, 'drone': drone_travel_time})

        self.G = G

        self.t_lb = self.get_shortest_arrival_times()

        self.demand_weights = {}
        for n_name in customers:
            n = self.all_nodes_indices[n_name]
            self.demand_weights[n] = random.uniform(demand_weight_min, demand_weight_max)
        for s_name in hubs:
            s = self.all_nodes_indices[s_name]
            self.demand_weights[s] = epsilon
        for n_name in [self.depot_source, self.depot_sink]:
            n = self.all_nodes_indices[n_name]
            self.demand_weights[n] = 0
        print(self.demand_weights)

    def get_shortest_arrival_times(self):
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
        # pos = nx.spring_layout(self.G, seed=self.seed)
        pos = nx.circular_layout(self.G)
        fig, ax = plt.subplots()
        plt.sca(ax)

        # Color mapping: Different colors for depot, customers, and hubs
        node_colors = []
        for node in self.G.nodes():
            if node == self.depot_source or node == self.depot_sink:
                node_colors.append('orange')  # Red for depot
            elif node in self.customers:
                node_colors.append('lightblue')  # Yellow for customers
            elif node in self.hubs:
                node_colors.append('lightgreen')  # Green for hubs
            else:
                node_colors.append('gray')  # Default color for others (if any)

        # Draw the nodes
        nx.draw_networkx_nodes(self.G, pos, node_color=node_colors, node_size=node_size)
        nx.draw_networkx_labels(self.G, pos, font_size=10, font_weight='bold')

        # Draw curved edges with varying curvature to avoid overlaps
        edge_curvatures = [0.2, 0.4, -0.2, -0.4]  # Example curvatures
        for i, (u, v) in enumerate(self.G.edges()):
            curvature = edge_curvatures[i % len(edge_curvatures)]  # Cycle through curvatures
            nx.draw_networkx_edges(
                self.G, pos, edgelist=[(u, v)], edge_color='gray', arrowsize=15, width=1,
                connectionstyle=f"arc3,rad={curvature}"
            )

        # Create custom labels for nodes
        node_labels = {}
        for node in self.G.nodes():
            if node in self.customers:
                n = self.all_nodes_indices[node]
                node_labels[node] = f"w:{self.demand_weights[n]:.2f}"

        label_pos = {node: (x, y + 0.05) for node, (x, y) in pos.items()}  # Adjust 0.05 to control the offset
        # Draw the node labels
        nx.draw_networkx_labels(self.G, label_pos, labels=node_labels, font_size=10)

        # Create labels for travel times on edges
        edge_labels = {}
        for u, v, data in self.G.edges(data=True):
            travel_times = data['travel_time']
            # Format the travel times for display
            edge_labels[(u, v)] = f"K: {travel_times['truck']:.2f} | D: {travel_times['drone']:.2f}"

        # Draw edge labels (travel times for both truck and drone)
        # nx.draw_networkx_edge_labels(self.G, pos, edge_labels=edge_labels, font_size=8, label_pos=0.5)

        plt.axis('off')  # Turn off the axis
        plt.tight_layout()

    def solve(self):
        model = gp.Model("model")
        # add decision variables
        x_dict = {}
        y_dict = {}
        edges = self.G.edges(data=True)
        for i_name, j_name, data in edges:
            i = self.all_nodes_indices[i_name]
            j = self.all_nodes_indices[j_name]
            for k in range(self.num_trucks):
                x_dict[(i, j, k)] = model.addVar(name=f"x_{(i, j, k)}", vtype=GRB.BINARY)
            for d in range(self.total_drone_num):
                y_dict[(i, j, d)] = model.addVar(name=f"y_{(i, j, d)}", vtype=GRB.BINARY)
        # auxiliary variables
        ad_dict = {}
        ak_dict = {}
        a_dict = {}
        t_dict = {}
        wk_dict = {}
        for n in range(len(self.all_nodes_indices)):
            a_dict[n] = model.addVar(name=f"t_{n}", vtype=GRB.CONTINUOUS, lb=0)
            for d in range(self.total_drone_num):
                t_dict[(n, d)] = model.addVar(name=f"t_{(n, d)}", vtype=GRB.CONTINUOUS, lb=0)
                wk_dict[(n, d)] = model.addVar(name=f"wk_{(n, d)}", vtype=GRB.CONTINUOUS, lb=0)
                ad_dict[(n, d)] = model.addVar(name=f"ad_{(n, d)}", vtype=GRB.CONTINUOUS, lb=0)
            for k in range(self.num_trucks):
                ak_dict[(n, k)] = model.addVar(name=f"ak_{(n, k)}", vtype=GRB.CONTINUOUS, lb=0)

        # add objective function
        obj_expr = 0
        # first term
        # for k in range(self.num_trucks):
        #     for i_name, j_name, data in edges:
        #         i = self.all_nodes_indices[i_name]
        #         j = self.all_nodes_indices[j_name]
        #         travel_time = data["travel_time"]["truck"]
        #         obj_expr += travel_time * x_dict[(i, j, k)]
        #     for n_name in self.hubs:
        #         n = self.hub_indices[n_name]
        #         obj_expr += t_dict[(n, k)]
        # second term
        for n_name in self.customers:
            n = self.all_nodes_indices[n_name]
            obj_expr += a_dict[n]
        model.setObjective(obj_expr, GRB.MINIMIZE)

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
                lhs += x_dict[(i, j, k)]
            for i_name, j_name in in_arcs:
                i = self.all_nodes_indices[i_name]
                j = self.all_nodes_indices[j_name]
                rhs += x_dict[(i, j, k)]
            self.constraints.append(model.addConstr(lhs == rhs, f"conserv1_{id}"))
            id += 1
        # cons 2
        for n_name in self.all_nodes:
            if n_name == self.depot_source or n_name == self.depot_sink:
                continue
            out_arcs = self.G.out_edges(n_name)
            in_arcs = self.G.in_edges(n_name)
            for k in range(self.num_trucks):
                lhs = rhs = 0
                for i_name, j_name in out_arcs:
                    i = self.all_nodes_indices[i_name]
                    j = self.all_nodes_indices[j_name]
                    lhs += x_dict[(i, j, k)]
                for i_name, j_name in in_arcs:
                    i = self.all_nodes_indices[i_name]
                    j = self.all_nodes_indices[j_name]
                    rhs += x_dict[(i, j, k)]
                self.constraints.append(model.addConstr(lhs == rhs, f"conserv2_{id}"))
                id += 1

        # only launch from depot once ************************************************************
        id = 0
        out_arcs = self.G.out_edges(self.depot_source)
        for k in range(self.num_trucks):
            lhs = 0
            for i_name, j_name in out_arcs:
                i = self.all_nodes_indices[i_name]
                j = self.all_nodes_indices[j_name]
                lhs += x_dict[(i, j, k)]
            self.constraints.append(model.addConstr(lhs <= 1, f"launch_once_{id}"))
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
                    lhs += x_dict[(i, n, k)]
                for d in range(self.total_drone_num):
                    lhs += y_dict[(i, n, d)]
            self.constraints.append(model.addConstr(lhs == 1, f"servonce_{id}"))
            id += 1

        # drone launch ************************************************************
        id = 0
        # cons 1, launch and retrieve only at sync points
        for d in range(self.total_drone_num):
            lhs = 0
            for n_name in self.all_nodes:
                if n_name in self.hubs:
                    continue
                n = self.all_nodes_indices[n_name]
                out_arcs = self.G.out_edges(n_name)
                for _, j_name in out_arcs:
                    j = self.all_nodes_indices[j_name]
                    lhs += y_dict[(n, j, d)]
            self.constraints.append(model.addConstr(lhs <= 0, f"drone_launch1_{id}"))
            id += 1
        # cons 2, launch from sync points
        for n_name in self.hubs:
            n = self.all_nodes_indices[n_name]
            out_arcs = self.G.out_edges(n_name)
            in_arcs = self.G.in_edges(n_name)
            for k in range(self.num_trucks):
                lhs = rhs = 0
                for d in self.drone_dict[k]:
                    for _, j_name in out_arcs:
                        j = self.all_nodes_indices[j_name]
                        lhs += y_dict[(n, j, d)]
                for i_name, _ in in_arcs:
                    i = self.all_nodes_indices[i_name]
                    rhs += x_dict[(i, n, k)]
                rhs *= len(self.drone_dict[k])
                self.constraints.append(model.addConstr(lhs <= rhs, f"drone_launch2_{id}"))
                id += 1

        # drone battery ************************************************************
        id = 0
        for n_name in self.hubs:
            n = self.all_nodes_indices[n_name]
            out_arcs = self.G.out_edges(n_name, data=True)
            for _, j_name, data in out_arcs:
                j = self.all_nodes_indices[j_name]
                travel_time = data["travel_time"]["drone"]
                for d in range(self.total_drone_num):
                    lhs = 2 * travel_time * y_dict[(n, j, d)]
                    self.constraints.append(model.addConstr(lhs <= drone_endurance, f"drone_battery_{id}"))
                    id += 1

        # payload ************************************************************
        id = 0
        # cons 1
        for n_name in self.all_nodes:
            n = self.all_nodes_indices[n_name]
            for k in range(self.num_trucks):
                self.constraints.append(model.addConstr(wk_dict[(n, k)] <= truck_max_weight, f"payload1_{id}"))
                id += 1
        # cons 2
        for n_name in self.customers + self.hubs:
            n = self.all_nodes_indices[n_name]
            in_arcs = self.G.in_edges(n_name, data=False)
            for i_name, _ in in_arcs:
                i = self.all_nodes_indices[i_name]
                for k in range(self.num_trucks):
                    self.constraints.append(model.addConstr(
                        wk_dict[(n, k)] >= wk_dict[(i, k)] + self.demand_weights[n] + M * (x_dict[(i, n, k)] - 1),
                        f"payload2_{id}"))
                    id += 1
        # cons 3
        for n_name in self.hubs:
            n = self.all_nodes_indices[n_name]
            out_arcs = self.G.out_edges(n_name, data=False)
            for _, j_name in out_arcs:
                j = self.all_nodes_indices[j_name]
                for d in range(self.total_drone_num):
                    self.constraints.append(
                        model.addConstr(self.demand_weights[j] * y_dict[(n, j, d)] <= drone_max_weight,
                                        f"payload3_{id}"))
                    id += 1

        # truck waiting times ************************************************************
        id = 0
        # cons 1
        for n_name in self.hubs:
            n = self.all_nodes_indices[n_name]
            out_arcs = self.G.out_edges(n_name, data=True)
            for k in range(self.num_trucks):
                for _, j_name, data in out_arcs:
                    j = self.all_nodes_indices[j_name]
                    travel_time = data["travel_time"]["drone"]
                    rhs = 0
                    for d in self.drone_dict[k]:
                        rhs += 2 * travel_time * y_dict[(n, j, d)]
                    self.constraints.append(
                        model.addConstr(t_dict[(n, k)] >= rhs, f"wait1_{id}"))
                    id += 1
        # cons 2
        for n_name in self.all_nodes:
            if n_name in self.hubs:
                continue
            n = self.all_nodes_indices[n_name]
            for k in range(self.num_trucks):
                self.constraints.append(
                    model.addConstr(t_dict[(n, k)] <= 0, f"wait2_{id}"))
                id += 1

        # realized service times ************************************************************
        id = 0
        # cons 1
        for d in range(self.total_drone_num):
            n = self.all_nodes_indices[self.depot_source]
            self.constraints.append(model.addConstr(ad_dict[(n, d)] <= 0, f"realized1_{id}"))
            id += 1
        # cons 2
        for n_name in self.customers:
            n = self.all_nodes_indices[n_name]
            for k in range(self.num_trucks):
                self.constraints.append(model.addConstr(a_dict[n] >= ak_dict[(n, k)], f"realized2_{id}"))
                id += 1
        # cons 3
        for k in range(self.num_trucks):
            n = self.all_nodes_indices[self.depot_source]
            self.constraints.append(model.addConstr(ak_dict[(n, k)] <= 0, f"realized3_{id}"))
            id += 1
        # cons 4
        for n_name in self.customers:
            n = self.all_nodes_indices[n_name]
            for d in range(self.total_drone_num):
                self.constraints.append(model.addConstr(a_dict[n] >= ad_dict[(n, d)], f"realized4_{id}"))
                id += 1
        # cons 4
        for n_name in self.all_nodes:
            if n_name == self.depot_source:
                continue
            n = self.all_nodes_indices[n_name]
            in_arcs = self.G.in_edges(n_name, data=True)
            for i_name, _, data in in_arcs:
                i = self.all_nodes_indices[i_name]
                travel_time = data["travel_time"]["truck"]
                for k in range(self.num_trucks):
                    rhs = ak_dict[(i, k)] + t_dict[(i, k)] + travel_time * x_dict[(i, n, k)] + M * (
                            x_dict[(i, n, k)] - 1)
                    self.constraints.append(model.addConstr(ak_dict[(n, k)] >= rhs, f"realized5_{id}"))
                    id += 1
        # cons 5
        for n_name in self.hubs:
            n = self.all_nodes_indices[n_name]
            out_arcs = self.G.out_edges(n_name, data=True)
            for _, j_name, data in out_arcs:
                j = self.all_nodes_indices[j_name]
                travel_time = data["travel_time"]["drone"]
                for d in range(self.total_drone_num):
                    rhs = ak_dict[(n, self.kd_dict[d])] + travel_time * y_dict[(n, j, d)] + M * (
                            y_dict[(n, j, d)] - 1)
                    self.constraints.append(model.addConstr(ad_dict[(j, d)] >= rhs, f"realized6_{id}"))
                    id += 1

        # test
        # self.constraints.append(model.addConstr(z_list[(2, 3, 0, 0)] == 1))
        # self.constraints.append(model.addConstr(y_dict[(3, 5, 0)] == 1))
        # self.constraints.append(model.addConstr(x_dict[(3, 7, 0)] == 1))
        # self.constraints.append(model.addConstr(z_list[(7, 8, 0, 0)] == 1))

        # self.constraints.append(model.addConstr(y_dict[(3, 1, 0)] == 1))
        # self.constraints.append(model.addConstr(y_dict[(1, 4, 0)] == 1))

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

        # Retrieve the values
        if model.status == GRB.OPTIMAL:
            print(f"Objective value: {model.objVal}")
            # Extract and store the solution values for the decision variables
            self.x_values = {(i, j, k): var.X for (i, j, k), var in x_dict.items()}
            self.y_values = {(i, j, d): var.X for (i, j, d), var in y_dict.items()}
            self.t_values = {(n, d): var.X for (n, d), var in t_dict.items()}
            self.w_values = {(n, d): var.X for (n, d), var in wk_dict.items()}
            self.ak_value = {(n, k): var.X for (n, k), var in ak_dict.items()}
            self.ad_value = {(n, d): var.X for (n, d), var in ad_dict.items()}
            dasd = 0

    def visualize_routes(self):
        pos = nx.circular_layout(self.G)

        # Color mapping: Different colors for depot, customers, and hubs
        node_colors = []
        for node in self.G.nodes():
            if node == self.depot_source or node == self.depot_sink:
                node_colors.append('orange')  # Orange for depot
            elif node in self.customers:
                node_colors.append('lightblue')  # Light blue for customers
            elif node in self.hubs:
                node_colors.append('lightgreen')  # Light green for hubs
            else:
                node_colors.append('gray')  # Default color for others (if any)

        # Truck route visualization
        fig, ax = plt.subplots()
        plt.sca(ax)
        plt.title("truck routes")
        nx.draw_networkx_nodes(self.G, pos, node_color=node_colors, node_size=node_size)
        nx.draw_networkx_labels(self.G, pos, font_size=10, font_weight='bold')

        truck_edges = {k: [] for k in range(self.num_trucks)}
        drone_edges = {d: [] for d in range(self.total_drone_num)}

        # Extract truck and drone routes
        for (i, j, k), val in self.x_values.items():
            if val > 0.5:
                truck_edges[k].append((self.all_nodes[i], self.all_nodes[j]))
        for (i, j, d), val in self.y_values.items():
            if val > 0.5:
                drone_edges[d].append((self.all_nodes[i], self.all_nodes[j]))

        # Draw truck routes with curved edges
        cmap = cm.get_cmap('tab10', self.num_trucks)
        for i in range(self.num_trucks):
            nx.draw_networkx_edges(
                self.G, pos, edgelist=truck_edges[i],
                edge_color=[cmap(i)],  # Dark gray for truck routes
                width=1, label=f"K{i}",
                arrows=True, arrowsize=15,
                connectionstyle="arc3,rad=0.2"  # Curved edges
            )

        # Create labels for travel times on edges
        edge_labels = {}
        for i_name, j_name, data in self.G.edges(data=True):
            for k in range(self.num_trucks):
                if (i_name, j_name) in truck_edges[k]:
                    travel_times = data['travel_time']
                    # Format the travel times for display
                    edge_labels[(i_name, j_name)] = f"{travel_times['truck']:.2f}"
        # Draw edge labels (travel times)
        nx.draw_networkx_edge_labels(self.G, pos, edge_labels=edge_labels, font_size=8, label_pos=0.5)

        # Add a legend
        plt.legend()

        # Drone route visualization
        fig, ax = plt.subplots()
        plt.sca(ax)
        plt.title("Drone routes")
        nx.draw_networkx_nodes(self.G, pos, node_color=node_colors, node_size=node_size)
        nx.draw_networkx_labels(self.G, pos, font_size=10, font_weight='bold')

        # Draw drone routes with curved dashed edges
        for i in range(self.total_drone_num):
            k = self.kd_dict[i]
            nx.draw_networkx_edges(
                self.G, pos, edgelist=drone_edges[i],
                edge_color=[cmap(k)],  # Light red for drone routes
                width=1, label=f"D{i}",
                arrows=True, arrowsize=15,
                style='dashed',
                connectionstyle="arc3,rad=0.2"  # Curved edges
            )

        # Create labels for travel times on edges
        edge_labels = {}
        for i_name, j_name, data in self.G.edges(data=True):
            for d in range(self.total_drone_num):
                if (i_name, j_name) in drone_edges[d]:
                    travel_times = data['travel_time']
                    # Format the travel times for display
                    edge_labels[(i_name, j_name)] = f"{travel_times['drone']:.2f}"
        # Draw edge labels (travel times)
        nx.draw_networkx_edge_labels(self.G, pos, edge_labels=edge_labels, font_size=8, label_pos=0.5)

        # Add a legend
        plt.legend()
