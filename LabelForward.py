import GeneralHelper
from NodeInfo import NodeInfo
from GeneralHelper import *


class LabelForward:
    def __init__(self, path, truck_load, drones_used, arrival_time, sync_time, wait_time, cost, depth, drone_flights,
                 truck_path):
        self.path = path  # Ordered sequence of visited nodes (partial path)
        self.truck_load = truck_load  # Truck loading weight
        self.drones_used = drones_used  # Number of drones used
        self.arrival_time = arrival_time  # Arrival time at last node
        self.sync_time = sync_time  # Arrival time at last synchronization point
        self.wait_time = wait_time  # truck waiting time
        self.cost = cost  # Accumulated cost
        # auxiliary components
        self.net = GeneralHelper.transformed_net
        self.depth = depth
        self.alternative_extensions = {node_j for node_j in self.net.out_arcs[self.path[-1]]}
        self.drone_flights = {k: v.copy() for k, v in drone_flights.items()}
        self.truck_path = truck_path
        self.latest_hub = None
        self.hash_path = truck_drone_path_to_hashable(self.truck_path, self.drone_flights) if depth == 0 else None

    def dominates(self, other, farkas: bool, duals=None):
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

    def allow_extend(self, node_j, node_info: NodeInfo, farkas=False):
        """
        check whether we can extend the current label to node_j
        """
        node_i = self.path[-1]
        _node_i = node_i.replace("_prime", "")
        _node_j = node_j.replace("_prime", "")
        arc = (node_i, node_j)

        # check branch arcs in transformed network
        if arc in node_info.disabled_arcs_trans:
            return False

        # check branch arcs in original network
        if arc in self.net.arcs_ori:  # a truck path
            if arc in node_info.disabled_arcs:
                return False
        elif arc in self.net.arcs_scp:  # green arc
            temp_arc = (node_i, _node_j)
            if temp_arc in node_info.disabled_arcs or temp_arc in node_info.disabled_arcs_drones_left:
                return False
        elif arc in self.net.arcs_cpcp:  # orange arc
            if (self.latest_hub, _node_j) in node_info.disabled_arcs_drones_left:
                return False

        # check if all required arcs is visited
        if node_j == self.net.depot_sink:
            for i, _ in node_info.must_visit_arcs:
                if i not in self.path and i + "_prime" not in self.path:
                    return False
            for i, j in node_info.must_visit_arcs_trans:
                if (i, j) not in zip(self.path, self.path[1:]):
                    return False

        # check Customer Visits Constraints
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

        # check Node Accessibility Check
        if arc in self.net.arcs_cpcp:
            if (self.latest_hub, node_j) not in self.net.arcs_scp:
                return False
        if arc in self.net.arcs_cpc:
            if (self.latest_hub, node_j) not in self.net.arcs_ori:
                return False

        # check Arrival Time Constraints
        if self.arrival_time < self.net.a_lb[node_i]:
            return False

        # check Loading Capacity Constraints
        if self.truck_load + self.net.demand_weights[node_j] > truck_max_weight:
            return False

        # check Drone Fleet Constraints
        if arc in self.net.arcs_scp or arc in self.net.arcs_cpcp:
            if self.drones_used >= num_drones_per_truck:
                return False

        return True

    def extend(self, node_j, duals, farkas):
        """
        extend the label to j
        """

        node_i = self.path[-1]
        arc = (node_i, node_j)
        label_j = self.__class__(
            path=self.path[:],
            truck_load=self.truck_load,
            drones_used=self.drones_used,
            arrival_time=self.arrival_time,
            sync_time=self.sync_time,
            wait_time=self.wait_time,
            cost=self.cost,
            depth=self.depth + 1,
            drone_flights=self.drone_flights,
            truck_path=self.truck_path[:])
        label_j.alternative_extensions = {node for node in self.net.out_arcs[node_j]}
        label_j.latest_hub = self.latest_hub

        # update path
        label_j.path.append(node_j)

        # update latest_hub
        if node_j in self.net.hubs:
            label_j.latest_hub = node_j
            label_j.drone_flights[node_j] = set()

        # update truck load
        if node_j in self.net.customers:
            label_j.truck_load = self.truck_load + self.net.demand_weights[node_j]

        # update z
        if node_j in self.net.customers_prime:
            label_j.drones_used += 1
        else:
            label_j.drones_used = 0

        # update a
        label_j.arrival_time = get_arrive_time(self.arrival_time, node_i, node_j, label_j.latest_hub, self.sync_time,
                                               self.wait_time, self.net)

        # update sync time
        if node_j in self.net.hubs:
            label_j.sync_time = label_j.arrival_time

        # update waiting time
        if node_j in self.net.customers_prime:
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

        # update auxiliary infos
        if arc in self.net.arcs_scp or arc in self.net.arcs_cpcp:
            label_j.drone_flights[label_j.latest_hub].add(node_j)
        elif arc in self.net.arcs_ori or arc in self.net.arcs_cpc:
            label_j.truck_path.append(node_j)

        label_j.hash_path = truck_drone_path_to_hashable(label_j.truck_path, label_j.drone_flights)

        return label_j

    def __eq__(self, other):
        if isinstance(other, LabelForward):
            return self.path == other.path
            # return self.truck_path == other.truck_path and self.drone_flights == other.drone_flights
        return False

    def __hash__(self):
        return hash(tuple(self.path))  # Hash based on ID so customers with same ID are treated as the same

    def __lt__(self, other):
        """Defines how to compare two LabelForward objects"""
        return self.depth < other.depth  # Compare based on depth
