import GeneralHelper
from GeneralHelper import *


class LabelForward:
    def __init__(self, path, truck_load, drones_used, arrival_time, sync_time, wait_time, cost, depth):
        self.path = path  # Ordered sequence of visited nodes (partial path)
        self.truck_load = truck_load  # Truck loading weight
        self.drones_used = drones_used  # Number of drones used
        self.arrival_time = arrival_time  # Arrival time at last node
        self.sync_time = sync_time  # Arrival time at last synchronization point
        self.wait_time = wait_time  # truck waiting time
        self.cost = cost  # Accumulated cost
        self.net = GeneralHelper.transformed_net
        self.depth = depth

    def dominates(self, other, farkas, duals=None):
        """Check if this label dominates another."""
        strict = False  # check whether contains a strict condition

        # condition 1.1 violated
        if not is_subsequence(self.path, other.path):
            return False
        elif self.path != other.path:  # Order-sensitive check
            strict = True

        # condition 1.2 violated
        if self.truck_load > other.truck_load:
            return False
        elif self.truck_load < other.truck_load:
            strict = True

        # condition 1.3 violated
        if self.drones_used > other.drones_used:
            return False
        elif self.drones_used < other.drones_used:
            strict = True

        # condition 2.1 violated
        if self.cost > other.cost:
            return False
        elif self.cost < other.cost:
            strict = True

        # condition 2.2 violated
        if not farkas:
            if self.arrival_time > other.arrival_time:
                return False
            elif self.arrival_time < other.arrival_time:
                strict = True

        # condition 2.3 violated
        if not farkas:
            if self.sync_time > other.sync_time:
                return False
            elif self.sync_time < other.sync_time:
                strict = True

        if strict:
            GeneralHelper.forward_dominance_num += 1

        # here all conditions are satisfied, we need at least one is strict
        return strict

    def allow_extend(self, node_j, farkas=False):
        """
        check whether we can extend the current label to node_j
        """
        node_i = self.path[-1]
        _node_j = node_j.replace("_prime", "")
        arc = (node_i, node_j)

        # check condition 1
        if node_j in self.net.customers:  # a customer node
            if "_prime" in node_j:  # node_j is a duplication
                dup_node = _node_j
            else:
                dup_node = node_j + "_prime"
            if node_j in self.path or dup_node in self.path:
                return False
        else:  # not a customer node
            if node_j in self.path:
                return False

        # check condition 2
        if arc in self.net.arcs_4:
            hub, _ = get_latest_hub(self.net, self.path)
            if (hub, _node_j) not in self.net.origin_drone_arcs:
                return False

        # check condition 3
        if arc in self.net.arcs_5:
            hub, _ = get_latest_hub(self.net, self.path)
            if (hub, _node_j) not in self.net.origin_truck_arcs:
                return False

        # check condition 4
        if self.arrival_time < self.net.a_lb[node_i]:
            return False

        # check condition 5
        if self.truck_load + self.net.demand_weights[node_j] > truck_max_weight:
            return False

        # check condition 6
        if arc in self.net.arcs_3 or arc in self.net.arcs_4:
            if self.drones_used >= num_drones_per_truck:
                return False

        return True

    def extend(self, node_j, duals, farkas):
        """
        extend the label to j
        """

        node_i = self.path[-1]
        label_j = self.__class__(
            path=self.path[:],
            truck_load=self.truck_load,
            drones_used=self.drones_used,
            arrival_time=self.arrival_time,
            sync_time=self.sync_time,
            wait_time=self.wait_time,
            cost=self.cost,
            depth=self.depth + 1)

        label_j.path.append(node_j)
        # update z
        if (node_i, node_j) in self.net.arcs_3 or (node_i, node_j) in self.net.arcs_4:
            label_j.drones_used += 1
        else:
            label_j.drones_used = 0

        # update a
        hub, _ = get_latest_hub(self.net, self.path)
        label_j.arrival_time = get_arrive_time(self.arrival_time, node_i, node_j, hub, self.sync_time,
                                               self.wait_time, self.net)

        # update sync time
        if node_j in self.net.hubs:
            label_j.sync_time = label_j.arrival_time
        else:
            pass

        # update waiting time
        if (node_i, node_j) in self.net.arcs_3 or (node_i, node_j) in self.net.arcs_4:
            label_j.wait_time = max(self.wait_time, label_j.arrival_time - label_j.sync_time)
        else:
            label_j.wait_time = 0

        # update cost
        if node_j in self.net.customers:
            index = self.net.customers.index(node_j.replace("_prime", ""))
            if not farkas:
                label_j.cost += (label_j.arrival_time - self.net.a_lb[node_j]) ** 2 - duals["mu"][index]
            else:
                label_j.cost -= duals["mu"][index]
        elif node_j == self.net.depot_sink:
            if not farkas:
                label_j.cost += label_j.arrival_time

        if label_j.path == test_path:
            sds = 0

        return label_j

    def __eq__(self, other):
        if isinstance(other, LabelForward):
            return self.path == other.path  # Consider customers equal if they have the same ID
        return False

    def __hash__(self):
        return hash(tuple(self.path))  # Hash based on ID so customers with same ID are treated as the same

    def __lt__(self, other):
        """Defines how to compare two LabelForward objects"""
        return self.depth < other.depth  # Compare based on depth
