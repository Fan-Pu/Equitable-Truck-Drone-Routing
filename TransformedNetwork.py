import copy
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
        self.out_arcs = {key: [] for key in self.all_nodes}
        self.in_arcs = {key: [] for key in self.all_nodes}
        self.travel_times = {}

        # duplicate the nodes
        new_nodes = []
        for node in net.all_nodes:
            needs_duplicate = False
            if node in net.hubs:
                needs_duplicate = True
            elif node in net.customers:
                for node_j in net.truck_in_arcs[node]:
                    if node_j in net.hubs:
                        needs_duplicate = True
                        break

            if needs_duplicate:
                new_node = node + "_prime"
                new_nodes.append(new_node)
                self.all_nodes.append(new_node)
                self.all_nodes_indices[new_node] = len(self.all_nodes) - 1
                self.out_arcs[new_node] = []
                self.in_arcs[new_node] = []
        for new_node in new_nodes:
            node = new_node.replace("_prime", "")
            if node in net.hubs:
                self.hubs.append(new_node)
            elif node in net.customers:
                self.customers.append(new_node)

        # black arcs
        for node, arcs in net.truck_out_arcs.items():
            for node_j in arcs:
                self.out_arcs[node].append(node_j)
                self.in_arcs[node_j].append(node)
                self.travel_times[(node, node_j)] = net.truck_travel_times[(node, node_j)]

        # blue arc
        for node in self.hubs:
            node_prime = node + "_prime"
            if node_prime not in self.hubs:
                continue
            self.out_arcs[node].append(node_prime)
            self.in_arcs[node_prime].append(node)
            self.travel_times[(node, node_prime)] = 0

        # green arc
        for node in net.hubs:
            node_prime = node + "_prime"
            for node_j in net.drone_out_arcs[node]:
                node_j_prime = node_j + "_prime"
                self.out_arcs[node_prime].append(node_j_prime)
                self.in_arcs[node_j_prime].append(node_prime)
                self.travel_times[(node_prime, node_j_prime)] = net.drone_travel_times[(node, node_j)]

        # orange arc
        omega_D_set = {}
        for node in net.hubs:
            node_prime = node + "_prime"
            omega_D_set[node] = []
            for node_j in net.drone_out_arcs[node]:
                omega_D_set[node].append(node_j)
            for node_i, node_j in combinations(omega_D_set[node], 2):
                node_i_prime = node_i + "_prime"
                node_j_prime = node_j + "_prime"
                # i to j
                self.out_arcs[node_i_prime].append(node_j_prime)
                self.in_arcs[node_j_prime].append(node_i_prime)
                self.travel_times[(node_i_prime, node_j_prime)] = self.travel_times[(node_prime, node_j_prime)]
                # j to i
                self.out_arcs[node_j_prime].append(node_i_prime)
                self.in_arcs[node_i_prime].append(node_j_prime)
                self.travel_times[(node_j_prime, node_i_prime)] = self.travel_times[(node_prime, node_i_prime)]

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
                    self.travel_times[(node_i_prime, node_j)] = net.truck_travel_times[(node, node_j)]
