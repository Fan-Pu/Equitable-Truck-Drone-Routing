import copy
import heapq
from sortedcontainers import SortedSet
from Network import Network
import itertools


class TransformedNetwork:

    def __init__(self, net: Network):
        """
        the class of the transformed network
        :param net: original network
        """
        self.num_trucks = copy.deepcopy(net.num_trucks)
        self.num_drones = copy.deepcopy(net.num_drones)
        self.customers = copy.deepcopy(net.customers)
        self.customers_prime = list()
        self.customers_origin = copy.deepcopy(net.customers)
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
        self.demand_weights = {}
        self.arcs_ori = SortedSet()  # original truck links
        self.arcs_scp = SortedSet()  # links sc'
        self.arcs_cpcp = SortedSet()  # links c'c'
        self.arcs_cpc = SortedSet()  # links c'c
        self.arcs = SortedSet()
        self.PI = list()  # for SR inequality

        # duplicate the nodes
        for node in net.customers:
            needs_duplicate = any(
                predecessor in net.hubs
                for predecessor in net.drone_in_arcs[node]
            )
            if needs_duplicate:
                new_node = node + "_prime"
                self.customers_prime.append(new_node)
                self.all_nodes.append(new_node)
                self.all_nodes_indices[new_node] = len(self.all_nodes) - 1
                self.customers.append(new_node)

        self.out_arcs = {key: [] for key in self.all_nodes}
        self.in_arcs = {key: [] for key in self.all_nodes}

        # given node n, the list contains the hubs that directly connect to n via drone links
        self.node_hubs_set_for_drone = {node: [] for node in self.all_nodes if node not in self.hubs}
        # given node n, the list contains the hubs that directly connect to n via truck links
        self.node_hubs_set_for_truck = {node: [] for node in self.all_nodes if node not in self.hubs}

        # black arcs
        for node, arcs in net.truck_out_arcs.items():
            for node_j in arcs:
                self.out_arcs[node].append(node_j)
                self.in_arcs[node_j].append(node)
                self.travel_times[(node, node_j)] = net.truck_travel_times[(node, node_j)]
                self.arcs_ori.add((node, node_j))

        # green arc sc'
        for node in net.hubs:
            for node_j in net.drone_out_arcs[node]:
                node_j_prime = node_j + "_prime"
                self.out_arcs[node].append(node_j_prime)
                self.in_arcs[node_j_prime].append(node)
                self.travel_times[(node, node_j_prime)] = net.drone_travel_times[(node, node_j)]
                self.arcs_scp.add((node, node_j_prime))

        # orange arc c'c'
        omega_D_set = {}
        for node in net.hubs:
            omega_D_set[node] = []
            for node_j in net.drone_out_arcs[node]:
                omega_D_set[node].append(node_j)
            omega_D_set[node].sort()
            for j in range(len(omega_D_set[node])):
                node_j = omega_D_set[node][j]
                for k in range(j + 1, len(omega_D_set[node])):
                    node_k = omega_D_set[node][k]
                    node_j_prime = node_j + "_prime"
                    node_k_prime = node_k + "_prime"
                    arc = (node_j_prime, node_k_prime)
                    if arc not in self.arcs_cpcp:
                        self.out_arcs[node_j_prime].append(node_k_prime)
                        self.in_arcs[node_k_prime].append(node_j_prime)
                        self.arcs_cpcp.add(arc)

        # purple arc c'c
        omega_K_set = {}
        for node in net.hubs:
            omega_K_set[node] = []
            for node_j in net.truck_out_arcs[node]:
                omega_K_set[node].append(node_j)
            omega_K_set[node].sort()
            for node_i in omega_D_set[node]:
                for node_j in omega_K_set[node]:
                    if node_i == node_j:
                        continue
                    node_i_prime = node_i + "_prime"
                    # i to j
                    self.out_arcs[node_i_prime].append(node_j)
                    self.in_arcs[node_j].append(node_i_prime)
                    self.arcs_cpc.add((node_i_prime, node_j))

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
        for node_i_prime, node_j_prime in self.arcs_cpcp:
            key = (node_i_prime, node_j_prime)
            self.travel_times[key] = {}
            for hub in self.node_hubs_set_for_drone[node_j_prime]:
                self.travel_times[key][hub] = net.drone_travel_times[(hub, node_j_prime.replace("_prime", ""))]
        for node_i_prime, node_j in self.arcs_cpc:
            key = (node_i_prime, node_j)
            self.travel_times[key] = {}
            for hub in self.node_hubs_set_for_truck[node_j]:
                self.travel_times[key][hub] = net.truck_travel_times[(hub, node_j)]

        # set the delivery demand
        for n, demand in net.demand_weights.items():
            node = net.all_nodes[n]
            self.demand_weights[node] = demand
            node_prime = node + "_prime"
            if node_prime in self.all_nodes:
                self.demand_weights[node_prime] = demand

        self.out_arcs = {node: list(dict.fromkeys(arc_list)) for node, arc_list in self.out_arcs.items()}
        self.in_arcs = {node: list(dict.fromkeys(arc_list)) for node, arc_list in self.in_arcs.items()}
        self.arcs = SortedSet(self.arcs_ori | self.arcs_scp | self.arcs_cpcp | self.arcs_cpc)

        # for SR inequality
        self.PI = list(itertools.combinations(net.customers, 3))

        # set the arrival time lower bound
        self.astar_shortest_path()

    def astar_shortest_path(self):
        """
        A* Search for the shortest path.
        """

        start_node = self.depot_source

        # Priority queue (max-heap), elements are (f(n), cur_node, visited_nodes, g(n))
        Q = [(self.heuristic(start_node), start_node, [start_node], 0, 0, 0)]
        heapq.heapify(Q)

        # arrive time estimate
        self.a_lb = {node: float('inf') for node in self.customers_origin}

        while Q and max(self.a_lb.values()) == float('inf'):
            _, node, visited, a_n, w_n, s_n = heapq.heappop(Q)

            for j in self.out_arcs[node]:
                if j in visited or j.replace("_prime", "") in visited or j + "_prime" in visited:
                    continue

                if j in self.hubs:
                    s_j = a_n + self.travel_times[node, j]
                else:
                    s_j = s_n

                if (node, j) in self.arcs_scp:
                    w_j = self.travel_times[node, j]
                    a_j = s_j + w_j
                elif (node, j) in self.arcs_cpcp:
                    w_j = max(self.travel_times[node, j], w_n)
                    a_j = s_j + w_j
                else:
                    w_j = 0
                    a_j = a_n + self.travel_times[node, j]

                f_j = a_j + self.heuristic(j)

                if j in self.customers_origin:
                    self.a_lb[j] = min(a_j, self.a_lb[j])
                elif j in self.customers_prime:
                    customer_j = j.replace("_prime", "")
                    self.a_lb[customer_j] = min(a_j, self.a_lb[customer_j])

                # Insert new state into priority queue
                heapq.heappush(Q, (f_j, j, visited + [j], a_j, w_j, s_j))

    def heuristic(self, node):
        """
        Heuristic function: Returns the minimum travel time of an outgoing edge.
        """
        if len(self.out_arcs[node]) > 0:
            return min([self.travel_times[(node, node_j)] for node_j in self.out_arcs[node]])
        else:
            return 0
