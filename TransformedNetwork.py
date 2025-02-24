import copy
import heapq
from itertools import combinations


class TransformedNetwork:

    def __init__(self, net):
        """
        the class of the transformed network
        :param net: original network
        """
        self.num_trucks = copy.deepcopy(net.num_trucks)
        self.num_drones = copy.deepcopy(net.num_drones)
        self.customers = copy.deepcopy(net.customers)
        self.hubs = copy.deepcopy(net.hubs)
        self.depot_source = copy.deepcopy(net.depot_source)
        self.depot_sink = copy.deepcopy(net.depot_sink)
        self.all_nodes = copy.deepcopy(net.all_nodes)
        self.all_nodes_indices = copy.deepcopy(net.all_nodes_indices)
        self.origin_truck_arcs = copy.deepcopy(net.truck_arcs)
        self.origin_drone_arcs = copy.deepcopy(net.drone_arcs)
        self.out_arcs = None
        self.in_arcs = None
        self.travel_times = {}
        self.a_lb = {}
        self.a_ub = {}
        self.demand_weights = {}
        self.arcs_1 = set()  # black links
        self.arcs_2 = set()  # blue links
        self.arcs_3 = set()  # green links
        self.arcs_4 = set()  # orange links
        self.arcs_5 = set()  # purple links
        self.max_timespan = -1

        # duplicate the nodes
        new_nodes = []
        for node in net.all_nodes:
            needs_duplicate = False
            if node in net.hubs:
                needs_duplicate = True
            elif node in net.customers:
                for node_i in net.drone_in_arcs[node]:
                    if node_i in net.hubs:
                        needs_duplicate = True
                        break

            if needs_duplicate:
                new_node = node + "_prime"
                new_nodes.append(new_node)
                self.all_nodes.append(new_node)
                self.all_nodes_indices[new_node] = len(self.all_nodes) - 1
        for new_node in new_nodes:
            node = new_node.replace("_prime", "")
            if node in net.hubs:
                self.hubs.append(new_node)
            elif node in net.customers:
                self.customers.append(new_node)

        self.out_arcs = {key: [] for key in self.all_nodes}
        self.in_arcs = {key: [] for key in self.all_nodes}

        self.node_hubs_set_for_drone = {node: [] for node in self.all_nodes if node not in self.hubs}
        self.node_hubs_set_for_truck = {node: [] for node in self.all_nodes if node not in self.hubs}

        # black arcs
        for node, arcs in net.truck_out_arcs.items():
            for node_j in arcs:
                self.out_arcs[node].append(node_j)
                self.in_arcs[node_j].append(node)
                self.travel_times[(node, node_j)] = net.truck_travel_times[(node, node_j)]
                self.arcs_1.add((node, node_j))

        # blue arc
        for node in self.hubs:
            node_prime = node + "_prime"
            if node_prime not in self.hubs:
                continue
            self.out_arcs[node].append(node_prime)
            self.in_arcs[node_prime].append(node)
            self.travel_times[(node, node_prime)] = 0
            self.arcs_2.add((node, node_prime))

        # green arc
        for node in net.hubs:
            node_prime = node + "_prime"
            for node_j in net.drone_out_arcs[node]:
                node_j_prime = node_j + "_prime"
                self.out_arcs[node_prime].append(node_j_prime)
                self.in_arcs[node_j_prime].append(node_prime)
                self.travel_times[(node_prime, node_j_prime)] = net.drone_travel_times[(node, node_j)]
                self.arcs_3.add((node_prime, node_j_prime))

        # orange arc
        omega_D_set = {}
        for node in net.hubs:
            omega_D_set[node] = []
            for node_j in net.drone_out_arcs[node]:
                omega_D_set[node].append(node_j)
            for node_i, node_j in combinations(omega_D_set[node], 2):
                node_i_prime = node_i + "_prime"
                node_j_prime = node_j + "_prime"
                # i to j
                self.out_arcs[node_i_prime].append(node_j_prime)
                self.in_arcs[node_j_prime].append(node_i_prime)
                self.arcs_4.add((node_i_prime, node_j_prime))
                # j to i
                self.out_arcs[node_j_prime].append(node_i_prime)
                self.in_arcs[node_i_prime].append(node_j_prime)
                self.arcs_4.add((node_j_prime, node_i_prime))

        # purple arc
        omega_K_set = {}
        for node in net.hubs:
            omega_K_set[node] = []
            for node_j in net.truck_out_arcs[node]:
                omega_K_set[node].append(node_j)
            for node_i in omega_D_set[node]:
                for node_j in omega_K_set[node]:
                    if node_i == node_j:
                        continue
                    node_i_prime = node_i + "_prime"
                    # i to j
                    self.out_arcs[node_i_prime].append(node_j)
                    self.in_arcs[node_j].append(node_i_prime)
                    self.arcs_5.add((node_i_prime, node_j))

        # update self.node_hubs_set
        for node in self.node_hubs_set_for_drone.keys():
            _node = node.replace("_prime", "")
            for hub, node_list in omega_D_set.items():
                # the node is accessible from the hub
                self.node_hubs_set_for_drone[node].append(hub) if (
                        _node in node_list and hub not in self.node_hubs_set_for_drone[node]) else None
        for node in self.node_hubs_set_for_truck.keys():
            _node = node.replace("_prime", "")
            for hub, node_list in omega_K_set.items():
                # the node is accessible from the hub
                self.node_hubs_set_for_truck[node].append(hub) if (
                        _node in node_list and hub not in self.node_hubs_set_for_truck[node]) else None

        # set travel times for orange and purple arcs
        for node_i_prime, node_j_prime in self.arcs_4:
            key = (node_i_prime, node_j_prime)
            self.travel_times[key] = {}
            for hub in self.node_hubs_set_for_drone[node_j_prime]:
                self.travel_times[key][hub] = net.drone_travel_times[(hub, node_j_prime.replace("_prime", ""))]
        for node_i_prime, node_j in self.arcs_5:
            key = (node_i_prime, node_j)
            self.travel_times[key] = {}
            for hub in self.node_hubs_set_for_truck[node_j]:
                self.travel_times[key][hub] = net.truck_travel_times[(hub, node_j)]

        # get the lower bound of the arrival times at the customer nodes
        for node in self.all_nodes:
            self.a_lb[node] = 0

        # set the delivery demand
        for n, demand in net.demand_weights.items():
            node = net.all_nodes[n]
            self.demand_weights[node] = demand
            node_prime = node + "_prime"
            if node_prime in self.all_nodes:
                self.demand_weights[node_prime] = demand

        self.out_arcs = {node: list(dict.fromkeys(arc_list)) for node, arc_list in self.out_arcs.items()}
        self.in_arcs = {node: list(dict.fromkeys(arc_list)) for node, arc_list in self.in_arcs.items()}
        self.max_timespan = self.astar_longest_path()

    def astar_longest_path(self):
        """
        A* Search for the Longest Elementary Path.
        """
        start_node = self.depot_source
        end_node = self.depot_sink

        # Priority queue (max-heap), elements are (-f(n), cur_node, visited_nodes, g(n))
        Q = [(-self.heuristic(start_node), start_node, [start_node], 0)]
        heapq.heapify(Q)

        # Longest path travel time estimate
        T = {node: float('-inf') for node in self.all_nodes}
        T[start_node] = 0

        while Q:
            _, node, visited, g_n = heapq.heappop(Q)

            if node == end_node:
                return T[node]  # Return longest travel time

            for node_prime in self.out_arcs[node]:
                if node_prime not in visited:
                    g_n_prime = g_n + self.travel_times[node, node_prime]
                    h_n_prime = self.heuristic(node_prime)
                    f_n_prime = g_n_prime + h_n_prime

                    if g_n_prime > T[node_prime]:  # Update T if found a longer path
                        T[node_prime] = g_n_prime

                    # Insert new state into priority queue
                    heapq.heappush(Q, (-f_n_prime, node_prime, visited + [node_prime], g_n_prime))

        return float('-inf')  # No path found

    def heuristic(self, node):
        """
        Heuristic function: Returns the maximum travel time of an outgoing edge.
        """
        if len(self.out_arcs[node]) > 0:
            return max([self.travel_times[(node, node_j)] for node_j in self.out_arcs[node]])
        else:
            return 0
