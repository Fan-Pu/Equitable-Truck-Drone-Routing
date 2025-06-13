import gurobipy as gp
import matplotlib.cm as cm
from gurobipy import GRB
from collections import defaultdict
import GeneralHelper
from GeneralHelper import *


class FullModel:
    def __init__(self):
        # save the values of decision variable
        self.a_value = None
        self.ad_value = None
        self.ak_value = None
        self.wk_values = None
        self.t_values = None
        self.y_values = None
        self.x_values = None
        self.net = GeneralHelper.net
        self.constraints = []
        self.solutions = {k: {} for k in range(self.net.num_trucks)}
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
        self.model = gp.Model("model")

    def solve(self, time_limit):
        # add decision variables
        x_dict = {}
        y_dict = {}
        for n in self.all_nodes:
            for node_j in self.truck_out_arcs[n]:
                i = self.all_nodes_indices[n]
                j = self.all_nodes_indices[node_j]
                for k in range(self.net.num_trucks):
                    x_dict[(i, j, k)] = self.model.addVar(name=f"x_{(i, j, k)}", vtype=GRB.BINARY, lb=0, ub=1)
            for node_j in self.drone_out_arcs[n]:
                i = self.all_nodes_indices[n]
                j = self.all_nodes_indices[node_j]
                for d in range(self.total_drone_num):
                    y_dict[(i, j, d)] = self.model.addVar(name=f"y_{(i, j, d)}", vtype=GRB.BINARY, lb=0, ub=1)
        # auxiliary variables
        ad_dict = {}
        ak_dict = {}
        a_dict = {}
        t_dict = {}
        wk_dict = {}
        for n in range(len(self.all_nodes_indices)):
            a_dict[n] = self.model.addVar(name=f"t_{n}", vtype=GRB.CONTINUOUS, lb=0)
            for d in range(self.total_drone_num):
                wk_dict[(n, d)] = self.model.addVar(name=f"wk_{(n, d)}", vtype=GRB.CONTINUOUS, lb=0)
                ad_dict[(n, d)] = self.model.addVar(name=f"ad_{(n, d)}", vtype=GRB.CONTINUOUS, lb=0)
            for k in range(self.net.num_trucks):
                ak_dict[(n, k)] = self.model.addVar(name=f"ak_{(n, k)}", vtype=GRB.CONTINUOUS, lb=0)
                t_dict[(n, k)] = self.model.addVar(name=f"t_{(n, k)}", vtype=GRB.CONTINUOUS, lb=0)

        # add objective function
        obj_expr = 0
        coef_list = []
        for n_name in self.customers:
            n = self.all_nodes_indices[n_name]
            obj_expr += cw * (a_dict[n] - GeneralHelper.transformed_net.a_lb[n_name]) ** 2
        for k in range(self.net.num_trucks):
            n = self.all_nodes_indices[self.depot_sink]
            obj_expr += cw * ak_dict[(n, k)]
        for k in range(self.net.num_trucks):
            for j_name in self.truck_out_arcs[self.depot_source]:
                n, j = self.all_nodes_indices[self.depot_source], self.all_nodes_indices[j_name]
                obj_expr += truck_cost * x_dict[(n, j, k)]
        for d in range(self.total_drone_num):
            for n_name in self.hubs:
                n = self.all_nodes_indices[n_name]
                for j_name in self.drone_out_arcs[n_name]:
                    j = self.all_nodes_indices[j_name]
                    obj_expr += drone_cost_per_flight * y_dict[(n, j, d)]
        self.model.setObjective(obj_expr, GRB.MINIMIZE)

        # flow conservation ************************************************************
        cons_id = 0
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
            self.constraints.append(
                self.model.addConstr(lhs == rhs, name=f"conserv1_{cons_id}"))
            cons_id += 1

        # cons 2
        for n_name in self.all_nodes:
            if n_name in (self.depot_source, self.depot_sink):
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
                self.constraints.append(
                    self.model.addConstr(lhs == rhs, name=f"conserv2_{cons_id}"))
                cons_id += 1

        # only launch from depot once ************************************************************
        cons_id = 0
        for k in range(self.net.num_trucks):
            lhs = 0
            for j_name in self.truck_out_arcs[self.depot_source]:
                n, j = self.all_nodes_indices[self.depot_source], self.all_nodes_indices[j_name]
                lhs += x_dict[(n, j, k)]
            self.constraints.append(
                self.model.addConstr(lhs <= 1, name=f"launch_once_{cons_id}"))
            cons_id += 1

        # customer serve once ************************************************************
        cons_id = 0
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
            self.constraints.append(
                self.model.addConstr(lhs == 1, name=f"servonce_{cons_id}"))
            cons_id += 1

        # drone launch ************************************************************
        cons_id = 0
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
                self.constraints.append(
                    self.model.addConstr(lhs <= rhs, name=f"drone_launch_{cons_id}"))
                cons_id += 1

        # drone battery ************************************************************
        cons_id = 0
        for n_name in self.hubs:
            n = self.all_nodes_indices[n_name]
            for j_name in self.drone_out_arcs[n_name]:
                j = self.all_nodes_indices[j_name]
                travel_time = self.drone_travel_times[(n_name, j_name)]
                for d in range(self.total_drone_num):
                    lhs = 2 * travel_time * y_dict[(n, j, d)]
                    self.constraints.append(
                        self.model.addConstr(lhs <= drone_endurance,
                                             name=f"drone_battery_{cons_id}"))
                    cons_id += 1

        # payload ************************************************************
        cons_id = 0
        # cons 1
        for n_name in self.all_nodes:
            n = self.all_nodes_indices[n_name]
            for k in range(self.net.num_trucks):
                self.constraints.append(
                    self.model.addConstr(wk_dict[(n, k)] <= truck_max_weight,
                                         name=f"payload1_{cons_id}"))
                cons_id += 1
        # cons 2
        for n_name in self.customers:
            n = self.all_nodes_indices[n_name]
            for i_name in self.truck_in_arcs[n_name]:
                i = self.all_nodes_indices[i_name]
                for k in range(self.net.num_trucks):
                    self.constraints.append(
                        self.model.addConstr(
                            wk_dict[(n, k)]
                            >= wk_dict[(i, k)]
                            + self.demand_weights[n]
                            + M * (x_dict[(i, n, k)] - 1),
                            name=f"payload2_{cons_id}"))
                    cons_id += 1
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
                    self.constraints.append(
                        self.model.addConstr(
                            wk_dict[(n, k)] >= wk_dict[(i, k)] + self.demand_weights[n] + rhs + M * (
                                    x_dict[(i, n, k)] - 1),
                            name=f"payload3_{cons_id}"))
                    cons_id += 1

        # truck waiting times ************************************************************
        cons_id = 0
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
                        self.model.addConstr(t_dict[(n, k)] >= rhs,
                                             name=f"wait1_{cons_id}"))
                    cons_id += 1
        # cons 2
        for n_name in self.all_nodes:
            if n_name in self.hubs:
                continue
            n = self.all_nodes_indices[n_name]
            for k in range(self.net.num_trucks):
                self.constraints.append(
                    self.model.addConstr(t_dict[(n, k)] <= 0,
                                         name=f"wait2_{cons_id}"))
                cons_id += 1

        # realized service times ************************************************************
        lhs = 0  # cons sum
        src = self.all_nodes_indices[self.depot_source]
        for k in range(self.net.num_trucks):
            lhs += ak_dict[(src, k)]
        for d in range(self.total_drone_num):
            lhs += ad_dict[(src, d)]
        self.constraints.append(
            self.model.addConstr(lhs <= 0, name="realized_sum"))
        # cons a_n
        cons_id = 0
        for n_name in self.customers:
            n = self.all_nodes_indices[n_name]
            for k in range(self.net.num_trucks):
                self.constraints.append(
                    self.model.addConstr(a_dict[n] >= ak_dict[(n, k)], name=f"realized_an1_{cons_id}"))
                cons_id += 1
        # cons a_n
        for n_name in self.customers:
            n = self.all_nodes_indices[n_name]
            for d in range(self.total_drone_num):
                self.constraints.append(
                    self.model.addConstr(a_dict[n] >= ad_dict[(n, d)], name=f"realized_an2_{cons_id}"))
                cons_id += 1
        # cons a_n,k
        for n_name in self.all_nodes:
            if n_name == self.depot_source:
                continue
            n = self.all_nodes_indices[n_name]
            for i_name in self.truck_in_arcs[n_name]:
                i = self.all_nodes_indices[i_name]
                travel_time = self.truck_travel_times[(i_name, n_name)]
                for k in range(self.net.num_trucks):
                    rhs = (ak_dict[(i, k)]
                           + t_dict[(i, k)]
                           + travel_time * x_dict[(i, n, k)]
                           + M * (x_dict[(i, n, k)] - 1))
                    self.constraints.append(
                        self.model.addConstr(ak_dict[(n, k)] >= rhs, name=f"realized_ank1_{cons_id}"))
                    cons_id += 1

                    rhs = (ak_dict[(i, k)]
                           + t_dict[(i, k)]
                           + travel_time * x_dict[(i, n, k)]
                           + M * (1 - x_dict[(i, n, k)]))
                    self.constraints.append(
                        self.model.addConstr(ak_dict[(n, k)] <= rhs, name=f"realized_ank2_{cons_id}"))
                    cons_id += 1
        for n_name in self.customers:
            n = self.all_nodes_indices[n_name]
            for i_name in self.truck_in_arcs[n_name]:
                i = self.all_nodes_indices[i_name]
                for k in range(self.net.num_trucks):
                    rhs = (GeneralHelper.transformed_net.a_lb[n_name] + M * (x_dict[(i, n, k)] - 1))
                    self.constraints.append(
                        self.model.addConstr(ak_dict[(n, k)] >= rhs, name=f"realized_ank3_{cons_id}"))
                    cons_id += 1
        # cons a_n,d
        for n_name in self.hubs:
            n = self.all_nodes_indices[n_name]
            for j_name in self.drone_out_arcs[n_name]:
                j = self.all_nodes_indices[j_name]
                travel_time = self.drone_travel_times[(n_name, j_name)]
                for d in range(self.total_drone_num):
                    # cons 1
                    rhs = (ak_dict[(n, self.kd_dict[d])]
                           + travel_time * y_dict[(n, j, d)]
                           + M * (y_dict[(n, j, d)] - 1))
                    self.constraints.append(
                        self.model.addConstr(ad_dict[(j, d)] >= rhs,
                                             name=f"realized_and1_{cons_id}"))
                    cons_id += 1
                    # cons 2
                    rhs = (ak_dict[(n, self.kd_dict[d])]
                           + travel_time * y_dict[(n, j, d)]
                           + M * (1 - y_dict[(n, j, d)]))
                    self.constraints.append(
                        self.model.addConstr(ad_dict[(j, d)] <= rhs,
                                             name=f"realized_and2_{cons_id}"))
                    cons_id += 1
                    # cons 3
                    rhs = (GeneralHelper.transformed_net.a_lb[j_name] + M * (y_dict[(n, j, d)] - 1))
                    self.constraints.append(
                        self.model.addConstr(ad_dict[(j, d)] >= rhs, name=f"realized_and3_{cons_id}"))
                    cons_id += 1

        # truck_hub
        cons_id = 0
        for n_name in self.hubs:
            n = self.all_nodes_indices[n_name]
            for k in range(self.net.num_trucks):
                lhs = 0
                for j_name in self.truck_out_arcs[n_name]:
                    j = self.all_nodes_indices[j_name]
                    lhs += x_dict[(n, j, k)]
                rhs = 0
                for d in self.drone_dict[k]:
                    for j_name in self.drone_out_arcs[n_name]:
                        j = self.all_nodes_indices[j_name]
                        rhs += y_dict[(n, j, d)]
                self.constraints.append(
                    self.model.addConstr(lhs <= rhs, name=f"truck_hub_{cons_id}"))
                cons_id += 1

        self.model.setParam(GRB.Param.TimeLimit, time_limit)
        self.model.update()
        self.model.write("full_model.lp")
        self.model.optimize()

        if self.model.Status == GRB.OPTIMAL:
            print("Optimal solution found")
        elif self.model.Status == GRB.INFEASIBLE:
            self.model.computeIIS()
            self.model.write("self.model.ilp")
            print("No feasible solution found")
        elif self.model.Status == GRB.UNBOUNDED:
            print("The model is unbounded")
        elif self.model.Status == GRB.INF_OR_UNBD:
            self.model.computeIIS()
            self.model.write("self.model.ilp")
            print("No feasible solution found")

        mip_gap_percent = None
        obj_val = -1

        # Retrieve the values
        # if self.model.Status in [GRB.OPTIMAL, GRB.SUBOPTIMAL, GRB.INTERRUPTED, GRB.TIME_LIMIT]:
        if self.model.SolCount > 0:
            print(f"Objective value: {self.model.ObjVal}")
            obj_val = self.model.ObjVal
            mip_gap_percent = self.model.MIPGap * 100
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
            from collections import defaultdict
            drone_routes = defaultdict(list)
            for (i, j, d), var in y_dict.items():
                if abs(var.X - 1) <= close_tolerance:
                    drone_routes[d].append((self.all_nodes[i], self.all_nodes[j]))

            truck_routes = {k: self.sort_arcs_to_path(val) for k, val in truck_routes.items()}
            for k, path in truck_routes.items():
                if not path:
                    continue
                self.solutions[k]['truck'] = []
                for pre_node, next_node in path:
                    self.solutions[k]['truck'].append(pre_node)
                self.solutions[k]['truck'].append(path[-1][-1])
                self.solutions[k]['drone'] = {}
            for d, path in drone_routes.items():
                k = int(d / num_drones_per_truck)
                for hub, node in path:
                    if hub not in self.solutions[k]['drone']:
                        self.solutions[k]['drone'][hub] = [node + "_T"]
                    else:
                        self.solutions[k]['drone'][hub].append(node + "_T")

        solving_time = self.model.Runtime

        return obj_val, solving_time, mip_gap_percent, [sol for sol in self.solutions.values()]

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
        nx.draw_networkx_nodes(self.net.truck_net, pos, node_color=node_colors, node_size=500)
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
        nx.draw_networkx_nodes(self.net.drone_net, pos, node_color=node_colors, node_size=500)
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

    def sort_arcs_to_path(self, arcs: list):
        """
        Given a list of arcs as (origin, destination) pairs,
        return a list of arcs ordered into a single path.
        """

        if not arcs:
            return []

        # find the start node: appears as origin but never as destination
        origins = {u for u, v in arcs}
        destinations = {v for u, v in arcs}
        starts = origins - destinations
        if len(starts) != 1:
            raise ValueError(f"Expected exactly one start node, got {starts}")
        start = next(iter(starts))

        # build a lookup from each node to its next node
        next_node = {u: v for u, v in arcs}

        # walk from start until no further arc is found
        path = []
        cur = start
        while cur in next_node:
            nxt = next_node[cur]
            path.append((cur, nxt))
            cur = nxt

        return path

    def reset(self):
        for key, sol in self.solutions.items():
            sol.clear()
        self.model.reset()
