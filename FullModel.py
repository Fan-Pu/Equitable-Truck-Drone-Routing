import math

import gurobipy as gp
import matplotlib.cm as cm
import matplotlib.pyplot as plt
import networkx as nx
from gurobipy import GRB

import GeneralHelper
from GeneralHelper import *

# for plot only
node_size = 500


class FullModel:
    def __init__(self):
        self.net = GeneralHelper.net
        self.constraints = []
        # set drone_dict and kd_dict
        self.drone_dict = {}
        self.kd_dict = {}  # key: drone_id
        drone_id = 0
        for k in range(self.net.num_trucks):
            temp_list = []
            for d in range(self.net.num_drones):
                temp_list.append(drone_id)
                self.kd_dict[drone_id] = k
                drone_id += 1
            self.drone_dict[k] = temp_list
        self.total_drone_num = drone_id
        self.all_nodes = self.net.all_nodes
        self.all_nodes_indices = self.net.all_nodes_indices
        self.truck_in_arcs = self.net.truck_in_arcs
        self.truck_out_arcs = self.net.truck_out_arcs
        self.drone_in_arcs = self.net.drone_in_arcs
        self.drone_out_arcs = self.net.drone_out_arcs
        self.customers = self.net.customers
        self.depot_source = self.net.depot_source
        self.depot_sink = self.net.depot_sink
        self.hubs = self.net.hubs
        self.drone_travel_times = self.net.drone_travel_times
        self.truck_travel_times = self.net.truck_travel_times
        self.demand_weights = self.net.demand_weights
        self.final_route = []

    def visualize(self):
        # pos = nx.spring_layout(self.truck_net, seed=self.seed)
        pos = nx.circular_layout(self.net.truck_net)
        fig, ax = plt.subplots()
        plt.sca(ax)

        # Color mapping: Different colors for depot, customers, and hubs
        node_colors = []
        for node in self.net.truck_net.nodes():
            if node == self.depot_source or node == self.depot_sink:
                node_colors.append('orange')  # Red for depot
            elif node in self.customers:
                node_colors.append('lightblue')  # Yellow for customers
            elif node in self.hubs:
                node_colors.append('lightgreen')  # Green for hubs
            else:
                node_colors.append('gray')  # Default color for others (if any)

        # Draw the nodes
        nx.draw_networkx_nodes(self.net.truck_net, pos, node_color=node_colors, node_size=node_size)
        nx.draw_networkx_labels(self.net.truck_net, pos, font_size=10, font_weight='bold')

        # Draw curved edges with varying curvature to avoid overlaps
        edge_curvatures = [0.2, 0.4, -0.2, -0.4]  # Example curvatures
        for i, (u, v) in enumerate(self.net.truck_net.edges()):
            curvature = edge_curvatures[i % len(edge_curvatures)]  # Cycle through curvatures
            nx.draw_networkx_edges(
                self.net.truck_net, pos, edgelist=[(u, v)], edge_color='gray', arrowsize=15, width=1,
                connectionstyle=f"arc3,rad={curvature}"
            )

        # Create custom labels for nodes
        node_labels = {}
        for node in self.net.truck_net.nodes():
            if node in self.customers:
                n = self.all_nodes_indices[node]
                node_labels[node] = f"w:{self.demand_weights[n]:.2f}"

        label_pos = {node: (x, y + 0.05) for node, (x, y) in pos.items()}  # Adjust 0.05 to control the offset
        # Draw the node labels
        nx.draw_networkx_labels(self.net.truck_net, label_pos, labels=node_labels, font_size=10)

        plt.axis('off')  # Turn off the axis
        plt.tight_layout()

    def solve(self):
        model = gp.Model("model")
        # add decision variables
        x_dict = {}
        y_dict = {}
        for n in self.all_nodes:
            for node_j in self.truck_out_arcs[n]:
                i = self.all_nodes_indices[n]
                j = self.all_nodes_indices[node_j]
                for k in range(self.net.num_trucks):
                    x_dict[(i, j, k)] = model.addVar(name=f"x_{(i, j, k)}", vtype=GRB.BINARY)
            for node_j in self.drone_out_arcs[n]:
                i = self.all_nodes_indices[n]
                j = self.all_nodes_indices[node_j]
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
                wk_dict[(n, d)] = model.addVar(name=f"wk_{(n, d)}", vtype=GRB.CONTINUOUS, lb=0)
                ad_dict[(n, d)] = model.addVar(name=f"ad_{(n, d)}", vtype=GRB.CONTINUOUS, lb=0)
            for k in range(self.net.num_trucks):
                ak_dict[(n, k)] = model.addVar(name=f"ak_{(n, k)}", vtype=GRB.CONTINUOUS, lb=0)
                t_dict[(n, k)] = model.addVar(name=f"t_{(n, k)}", vtype=GRB.CONTINUOUS, lb=0)

        # add objective function
        obj_expr = 0
        for n_name in self.customers:
            n = self.all_nodes_indices[n_name]
            obj_expr += (a_dict[n] - self.net.a_lb[n_name]) ** 2
        for k in range(self.net.num_trucks):
            n = self.all_nodes_indices[self.depot_sink]
            obj_expr += ak_dict[(n, k)]
        model.setObjective(obj_expr, GRB.MINIMIZE)

        # flow conservation ************************************************************
        id = 0
        # cons 1
        for k in range(self.net.num_trucks):
            lhs = rhs = 0
            n_name = self.depot_source
            for j_name in self.truck_out_arcs[n_name]:
                n, j = self.all_nodes_indices[n_name], self.all_nodes_indices[j_name]
                lhs += x_dict[(n, j, k)]
            n_name = self.depot_sink
            for i_name in self.truck_in_arcs[n_name]:
                n, i = self.all_nodes_indices[n_name], self.all_nodes_indices[i_name]
                rhs += x_dict[(i, n, k)]
            self.constraints.append(model.addConstr(lhs == rhs, f"conserv1_{id}"))
            id += 1
        # cons 2
        for n_name in self.all_nodes:
            if n_name == self.depot_source or n_name == self.depot_sink:
                continue
            n = self.all_nodes_indices[n_name]
            for k in range(self.net.num_trucks):
                lhs = rhs = 0
                for j_name in self.truck_out_arcs[n_name]:
                    j = self.all_nodes_indices[j_name]
                    lhs += x_dict[(n, j, k)]
                for i_name in self.truck_in_arcs[n_name]:
                    i = self.all_nodes_indices[i_name]
                    rhs += x_dict[(i, n, k)]
                self.constraints.append(model.addConstr(lhs == rhs, f"conserv2_{id}"))
                id += 1

        # only launch from depot once ************************************************************
        id = 0
        for k in range(self.net.num_trucks):
            lhs = 0
            for j_name in self.truck_out_arcs[self.depot_source]:
                n, j = self.all_nodes_indices[self.depot_source], self.all_nodes_indices[j_name]
                lhs += x_dict[(n, j, k)]
            self.constraints.append(model.addConstr(lhs <= 1, f"launch_once_{id}"))
            id += 1

        # customer serve once ************************************************************
        id = 0
        for n_name in self.customers:
            lhs = 0
            n = self.all_nodes_indices[n_name]
            for i_name in self.truck_in_arcs[n_name]:
                i = self.all_nodes_indices[i_name]
                for k in range(self.net.num_trucks):
                    lhs += x_dict[(i, n, k)]
            for i_name in self.drone_in_arcs[n_name]:
                i = self.all_nodes_indices[i_name]
                for d in range(self.total_drone_num):
                    lhs += y_dict[(i, n, d)]
            self.constraints.append(model.addConstr(lhs == 1, f"servonce_{id}"))
            id += 1

        # drone launch ************************************************************
        id = 0
        for n_name in self.hubs:
            n = self.all_nodes_indices[n_name]
            for k in range(self.net.num_trucks):
                lhs = rhs = 0
                for d in self.drone_dict[k]:
                    for j_name in self.drone_out_arcs[n_name]:
                        j = self.all_nodes_indices[j_name]
                        lhs += y_dict[(n, j, d)]
                for i_name in self.truck_in_arcs[n_name]:
                    i = self.all_nodes_indices[i_name]
                    rhs += x_dict[(i, n, k)]
                rhs *= len(self.drone_dict[k])
                self.constraints.append(model.addConstr(lhs <= rhs, f"drone_launch_{id}"))
                id += 1

        # drone battery ************************************************************
        id = 0
        for n_name in self.hubs:
            n = self.all_nodes_indices[n_name]
            for j_name in self.drone_out_arcs[n_name]:
                j = self.all_nodes_indices[j_name]
                travel_time = self.drone_travel_times[(n_name, j_name)]
                for d in range(self.total_drone_num):
                    lhs = 2 * travel_time * y_dict[(n, j, d)]
                    self.constraints.append(model.addConstr(lhs <= drone_endurance, f"drone_battery_{id}"))
                    id += 1

        # payload ************************************************************
        id = 0
        # cons 1
        for n_name in self.all_nodes:
            n = self.all_nodes_indices[n_name]
            for k in range(self.net.num_trucks):
                self.constraints.append(model.addConstr(wk_dict[(n, k)] <= truck_max_weight, f"payload1_{id}"))
                id += 1
        # cons 2
        for n_name in self.customers:
            n = self.all_nodes_indices[n_name]
            for i_name in self.truck_in_arcs[n_name]:
                i = self.all_nodes_indices[i_name]
                for k in range(self.net.num_trucks):
                    self.constraints.append(model.addConstr(
                        wk_dict[(n, k)] >= wk_dict[(i, k)] + self.demand_weights[n] + M * (x_dict[(i, n, k)] - 1),
                        f"payload2_{id}"))
                    id += 1
        # cons 3
        for n_name in self.hubs:
            n = self.all_nodes_indices[n_name]
            for i_name in self.truck_in_arcs[n_name]:
                i = self.all_nodes_indices[i_name]
                for k in range(self.net.num_trucks):
                    rhs = 0
                    for j_name in self.drone_out_arcs[n_name]:
                        j = self.all_nodes_indices[j_name]
                        demand_weight = self.demand_weights[j]
                        for d in self.drone_dict[k]:
                            rhs += demand_weight * y_dict[(n, j, d)]
                    self.constraints.append(model.addConstr(
                        wk_dict[(n, k)] >= wk_dict[(i, k)] + self.demand_weights[n] + rhs + M * (x_dict[(i, n, k)] - 1),
                        f"payload3_{id}"))
                    id += 1

        # truck waiting times ************************************************************
        id = 0
        # cons 1
        for n_name in self.hubs:
            n = self.all_nodes_indices[n_name]
            for k in range(self.net.num_trucks):
                for j_name in self.drone_out_arcs[n_name]:
                    j = self.all_nodes_indices[j_name]
                    travel_time = self.drone_travel_times[(n_name, j_name)]
                    rhs = 0
                    for d in self.drone_dict[k]:
                        rhs += travel_time * y_dict[(n, j, d)]
                    self.constraints.append(
                        model.addConstr(t_dict[(n, k)] >= rhs, f"wait1_{id}"))
                    id += 1
        # cons 2
        for n_name in self.all_nodes:
            if n_name in self.hubs:
                continue
            n = self.all_nodes_indices[n_name]
            for k in range(self.net.num_trucks):
                self.constraints.append(
                    model.addConstr(t_dict[(n, k)] <= 0, f"wait2_{id}"))
                id += 1

        # realized service times ************************************************************
        id = 0
        # cons 1
        lhs = 0
        for k in range(self.net.num_trucks):
            n = self.all_nodes_indices[self.depot_source]
            lhs += ak_dict[(n, k)]
        for d in range(self.total_drone_num):
            n = self.all_nodes_indices[self.depot_source]
            lhs += ad_dict[(n, d)]
        self.constraints.append(model.addConstr(lhs <= 0, f"realized1"))
        # cons 2
        for n_name in self.customers:
            n = self.all_nodes_indices[n_name]
            for k in range(self.net.num_trucks):
                self.constraints.append(model.addConstr(a_dict[n] >= ak_dict[(n, k)], f"realized2_{id}"))
                id += 1
        # cons 3
        for n_name in self.customers:
            n = self.all_nodes_indices[n_name]
            for d in range(self.total_drone_num):
                self.constraints.append(model.addConstr(a_dict[n] >= ad_dict[(n, d)], f"realized3_{id}"))
                id += 1
        # cons 4
        for n_name in self.all_nodes:
            if n_name == self.depot_source:
                continue
            n = self.all_nodes_indices[n_name]
            for i_name in self.truck_in_arcs[n_name]:
                i = self.all_nodes_indices[i_name]
                travel_time = self.truck_travel_times[(i_name, n_name)]
                for k in range(self.net.num_trucks):
                    rhs = ak_dict[(i, k)] + t_dict[(i, k)] + travel_time * x_dict[(i, n, k)] + M * (
                            x_dict[(i, n, k)] - 1)
                    self.constraints.append(model.addConstr(ak_dict[(n, k)] >= rhs, f"realized4_{id}"))
                    id += 1
        # cons 5
        for n_name in self.hubs:
            n = self.all_nodes_indices[n_name]
            for j_name in self.drone_out_arcs[n_name]:
                j = self.all_nodes_indices[j_name]
                travel_time = self.drone_travel_times[(n_name, j_name)]
                for d in range(self.total_drone_num):
                    rhs = ak_dict[(n, self.kd_dict[d])] + travel_time * y_dict[(n, j, d)] + M * (
                            y_dict[(n, j, d)] - 1)
                    self.constraints.append(model.addConstr(ad_dict[(j, d)] >= rhs, f"realized5_{id}"))
                    id += 1

        # test
        # for k in range(len(GeneralHelper.route_list)):
        #     route = GeneralHelper.route_list[k]
        #     for node_pre, node_next in zip(route['truck'], route['truck'][1:]):
        #         i = self.net.all_nodes_indices[node_pre]
        #         j = self.net.all_nodes_indices[node_next]
        #         model.addConstr(x_dict[(i, j, k)] == 1, "")
        #     launches = route['launches']
        #     drone_visits = route['drone']
        #     for m in range(len(launches)):
        #         launch_point = launches[m]
        #         drones = drone_visits[m]
        #         for d in range(len(drones)):
        #             i = self.all_nodes_indices[launch_point]
        #             node_j = drones[d]
        #             j = self.all_nodes_indices[node_j]
        #             model.addConstr(y_dict[(i, j, k * num_drones_per_truck + d)] == 1, "")

        model.setParam(GRB.Param.TimeLimit, 1)
        model.update()
        model.write("full_model.lp")
        model.optimize()
        if model.status == GRB.OPTIMAL:
            print(f"Optimal solution found")
        elif model.status == GRB.INFEASIBLE:
            model.computeIIS()
            model.write("model.ilp")
            print("No feasible solution found")
        elif model.status == GRB.UNBOUNDED:
            print("The model is unbounded")
        elif model.status == GRB.INF_OR_UNBD:
            print("No feasible solution found")

        # Retrieve the values
        if model.status == GRB.OPTIMAL:
            print(f"Objective value: {model.objVal}")
            # Extract and store the solution values for the decision variables
            self.x_values = {(i, j, k): var.X for (i, j, k), var in x_dict.items()}
            self.y_values = {(i, j, d): var.X for (i, j, d), var in y_dict.items()}
            self.t_values = {(n, k): var.X for (n, k), var in t_dict.items()}
            self.wk_values = {(n, d): var.X for (n, d), var in wk_dict.items()}
            self.ak_value = {(n, k): var.X for (n, k), var in ak_dict.items()}
            self.ad_value = {(n, d): var.X for (n, d), var in ad_dict.items()}
            self.a_value = {n: var.X for n, var in a_dict.items()}

            # construct the route
            truck_routes = {k: [] for k in range(self.net.num_trucks)}
            for (i, j, k), var in x_dict.items():
                if abs(var.X - 1) <= close_tolerance:
                    truck_routes[k].append((self.all_nodes[i], self.all_nodes[j]))
            drone_routes = []
            for (i, j, d), var in y_dict.items():
                if abs(var.X - 1) <= close_tolerance:
                    drone_routes.append((self.all_nodes[i], self.all_nodes[j], d))
            fdsfsd = 0
        solving_time = model.Runtime
        mip_gap_percent = model.MIPGap * 100

        return model.objVal, solving_time, mip_gap_percent

    def visualize_routes(self):
        pos = nx.circular_layout(self.net.truck_net)

        # Color mapping: Different colors for depot, customers, and hubs
        node_colors = []
        for node in self.net.truck_net.nodes():
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
        nx.draw_networkx_nodes(self.net.truck_net, pos, node_color=node_colors, node_size=node_size)
        nx.draw_networkx_labels(self.net.truck_net, pos, font_size=10, font_weight='bold')

        truck_edges = {k: [] for k in range(self.net.num_trucks)}
        drone_edges = {d: [] for d in range(self.total_drone_num)}

        # Extract truck and drone routes
        for (i, j, k), val in self.x_values.items():
            if val > 0.5:
                truck_edges[k].append((self.all_nodes[i], self.all_nodes[j]))
        for (i, j, d), val in self.y_values.items():
            if val > 0.5:
                drone_edges[d].append((self.all_nodes[i], self.all_nodes[j]))

        # Draw truck routes with curved edges
        cmap = cm.get_cmap('tab10', self.net.num_trucks)
        for i in range(self.net.num_trucks):
            nx.draw_networkx_edges(
                self.net.truck_net, pos, edgelist=truck_edges[i],
                edge_color=[cmap(i)],  # Dark gray for truck routes
                width=1, label=f"K{i}",
                arrows=True, arrowsize=15,
                connectionstyle="arc3,rad=0.2"  # Curved edges
            )

        # Create labels for travel times on edges
        edge_labels = {}
        for i_name, j_name in self.net.truck_net.edges(data=False):
            for k in range(self.net.num_trucks):
                if (i_name, j_name) in truck_edges[k]:
                    travel_time = self.truck_travel_times[(i_name, j_name)]
                    # Format the travel times for display
                    edge_labels[(i_name, j_name)] = f"{travel_time:.2f}"
        # Draw edge labels (travel times)
        nx.draw_networkx_edge_labels(self.net.truck_net, pos, edge_labels=edge_labels, font_size=8, label_pos=0.5)

        # Add a legend
        plt.legend()

        # Drone route visualization
        fig, ax = plt.subplots()
        plt.sca(ax)
        plt.title("Drone routes")
        nx.draw_networkx_nodes(self.net.drone_net, pos, node_color=node_colors, node_size=node_size)
        nx.draw_networkx_labels(self.net.drone_net, pos, font_size=10, font_weight='bold')

        # Draw drone routes with curved dashed edges
        for i in range(self.total_drone_num):
            k = self.kd_dict[i]
            nx.draw_networkx_edges(
                self.net.drone_net, pos, edgelist=drone_edges[i],
                edge_color=[cmap(k)],  # Light red for drone routes
                width=1, label=f"D{i}",
                arrows=True, arrowsize=15,
                style='dashed',
                connectionstyle="arc3,rad=0.2"  # Curved edges
            )

        # Create labels for travel times on edges
        edge_labels = {}
        for i_name, j_name in self.net.drone_net.edges(data=False):
            for d in range(self.total_drone_num):
                if (i_name, j_name) in drone_edges[d]:
                    travel_time = self.drone_travel_times[(i_name, j_name)]
                    # Format the travel times for display
                    edge_labels[(i_name, j_name)] = f"{travel_time:.2f}"
        # Draw edge labels (travel times)
        nx.draw_networkx_edge_labels(self.net.drone_net, pos, edge_labels=edge_labels, font_size=8, label_pos=0.5)

        # Add a legend
        plt.legend()

    def update_arc_infos(self, location, term, travel_time, is_truck):
        if is_truck:
            self.truck_travel_times[(location, term)] = travel_time
            self.truck_out_arcs[location].append(term)
            self.truck_in_arcs[term].append(location)
        else:
            self.drone_travel_times[(location, term)] = travel_time
            self.drone_out_arcs[location].append(term)
            self.drone_in_arcs[term].append(location)
