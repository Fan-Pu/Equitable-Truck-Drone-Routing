import GeneralHelper
from GeneralHelper import *


class LabelBackward:
    def __init__(self, path, truck_load, drones_used, arrival_times, arrival_ubs, cost, depth, drone_flights,
                 truck_path, pending_flights):
        self.path = path  # Ordered sequence of visited nodes
        self.truck_load = truck_load  # Truck loading weight
        self.drones_used = drones_used  # Number of drones used
        self.arrival_times = arrival_times  # List of arrival times
        self.arrival_ubs = arrival_ubs  # Arrival time upper bounds
        # auxiliary components
        self.net = GeneralHelper.transformed_net
        self.cost = cost  # reduced cost
        self.depth = depth
        self.alternative_extensions = {node_j for node_j in self.net.in_arcs[self.path[0]]}
        self.drone_flights = {k: v.copy() for k, v in drone_flights.items()}
        self.truck_path = truck_path
        self.pending_flights = pending_flights.copy()

    def dominates(self, other, farkas, duals):
        """Check if this label dominates another."""
        strict = False  # check whether contains a strict condition
        # the path is complete
        if self.path[0] == self.net.depot_source:
            if self.cost < other.cost:
                strict = True
                return strict

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

        # cost-based dominance
        if len(self.pending_flights) + len(other.pending_flights) == 0:
            if self.cost > other.cost:
                return False
            elif self.cost < other.cost:
                strict = True
                GeneralHelper.backward_cost_dominance_num += 1
        else:  # this is not conclusive
            return False

        # if not farkas:  # normal pricing
        #     # cost-based dominance
        #     if self.cost < other.cost:
        #         strict = True
        #         GeneralHelper.backward_cost_dominance_num += 1
        #         return strict
        #
        #     # # arrival time-based dominance
        #     # lhs = rhs = 0
        #     # customer_on_path1 = [node for node in self.path if node in self.net.customers]
        #     # customer_on_path2 = [node for node in other.path if node in self.net.customers]
        #     # for i in range(len(customer_on_path1)):
        #     #     node = customer_on_path1[i]
        #     #     if i == len(customer_on_path1) - 1:  # depot sink node
        #     #         lhs += (self.net.a_lb[node] - self.arrival_ubs[i])
        #     #     else:
        #     #         lhs -= (self.arrival_ubs[i] - self.net.a_lb[node]) ** 2
        #     #
        #     # for node in customer_on_path2:
        #     #     if node in customer_on_path1:
        #     #         continue
        #     #     index = self.net.customers.index(node.replace("_prime", ""))
        #     #     rhs += duals["mu"][index]
        #     #
        #     # if lhs < rhs:
        #     #     return False
        #     # elif lhs > rhs:
        #     #     strict = True
        # else:  # Farkas pricing
        #     c_1 = cal_label_cost_farkas(self.net, self.path, duals)
        #     c_2 = cal_label_cost_farkas(self.net, other.path, duals)
        #     if c_1 > c_2:
        #         return False
        #     elif c_1 < c_2:
        #         strict = True

        # if strict:
        #     GeneralHelper.backward_arrival_dominance_num += 1

        # here all conditions are satisfied, we need at least one is strict
        return strict

    def allow_extend(self, node_j):
        """
        Check whether we can extend the current label to node_j.
        """

        node_i = self.path[0]
        _node_j = node_j.replace("_prime", "")
        path_set = set(self.path)  # Convert list to set for O(1) lookup

        # Condition 1: Check customer node duplication
        if node_j in self.net.customers:  # If node_j is a customer node
            dup_node = _node_j if "_prime" in node_j else node_j + "_prime"
            if node_j in path_set or dup_node in path_set:
                return False
        else:  # If node_j is not a customer node
            if node_j in path_set:
                return False

        # Condition 2: Check truck load limit
        demand_weight = self.net.demand_weights.get(node_j, 0)
        if self.truck_load + demand_weight > truck_max_weight:
            return False

        # Condition 3: Check drone usage limit
        if (node_j, node_i) in self.net.arcs_4 or (node_j, node_i) in self.net.arcs_5:  # Set union for efficiency
            if self.drones_used >= num_drones_per_truck:
                return False

        # Condition 4: Check hub duplication validity
        if node_j in self.net.hubs and "_prime" in node_j:
            prefix = [self.path[0]]

            # Find prefix efficiently using an iterator
            for node, node_next in zip(self.path, self.path[1:]):
                prefix.append(node_next)
                if (node, node_next) in self.net.arcs_5:
                    break
            else:
                raise Exception("prefix is not found!")

            # Validate prefix path
            for node in prefix[:-1]:  # Iterate without the last element
                _node = node.replace("_prime", "")
                if (_node_j, _node) not in self.net.origin_drone_arcs:
                    return False

            # Validate last node separately
            last_node = prefix[-1].replace("_prime", "")
            if (_node_j, last_node) not in self.net.origin_truck_arcs:
                return False

        return True

    def extend(self, node_j, duals, farkas):
        """
        extend the label to j
        """
        node_i = self.path[0]

        if self.path == ['H1_prime', 'C3_prime', 'C1_prime', 'Sink'] and node_j == "H1":
            sdsa = 0

        # Use a shallow copy where possible to avoid unnecessary list duplications
        label_j = self.__class__(
            path=self.path[:],
            truck_load=self.truck_load,
            drones_used=self.drones_used,  # This may be updated below
            arrival_times=self.arrival_times[:],
            arrival_ubs=self.arrival_ubs[:],
            cost=self.cost,
            depth=self.depth + 1,
            drone_flights=self.drone_flights,
            truck_path=self.truck_path[:],
            pending_flights=self.pending_flights)
        label_j.alternative_extensions = {node for node in self.net.in_arcs[node_j]}

        label_j.path = [node_j] + label_j.path

        # Update drones_used
        label_j.drones_used = self.drones_used + 1 \
            if (node_j, node_i) in self.net.arcs_4 or (node_j, node_i) in self.net.arcs_5 else 0

        # Update arrival_ubs only when necessary
        if not farkas and node_j != self.net.depot_source:
            label_j.arrival_ubs = self.measure_arrival_ubs()

        arc = (node_j, node_i)

        # update cost
        if not farkas:
            # update delta_a
            delta_a = []
            prefix = None
            if arc in self.net.arcs_1 or arc in self.net.arcs_2:
                delta_a = [self.net.travel_times[arc]] * len(self.path)
            elif arc in self.net.arcs_4 or arc in self.net.arcs_5:
                delta_a = [0] * len(self.path)
            elif arc in self.net.arcs_3:
                prefix, node_m_next = find_prefix(self.path, self.net)
                for i in range(len(self.path)):
                    node_n = self.path[i]
                    if node_n in prefix:
                        delta_a.append(self.net.travel_times[(node_j, node_n)])
                    else:
                        wait_time = max([self.net.travel_times[(node_j, n_prime)] for n_prime in prefix])
                        travel_time = self.net.travel_times[(node_j.replace("_prime", ""), node_m_next)]
                        delta_a.extend([wait_time + travel_time] * (len(self.path) - len(prefix)))
                        break

            # cost update
            if arc in self.net.arcs_1:
                sum_term = sum(
                    delta_a[i] ** 2 + 2 * delta_a[i] * (
                            self.arrival_times[i] - self.net.a_lb[node.replace("_prime", "")])
                    for i, node in enumerate(self.path) if node in self.net.customers
                )
                # condition 1
                if node_j in self.net.customers:
                    index = self.net.customers.index(node_j.replace("_prime", ""))
                    label_j.cost = self.cost + delta_a[-1] + sum_term - duals["mu"][index]
                # condition 2
                else:
                    label_j.cost = self.cost + delta_a[-1] + sum_term
            # condition 3
            elif arc in self.net.arcs_3:
                sum_term = sum(
                    delta_a[i] ** 2 + 2 * delta_a[i] * (
                            self.arrival_times[i] - self.net.a_lb[node.replace("_prime", "")])
                    for i, node in enumerate(self.path) if node in self.net.customers and node not in prefix
                )

                rho = sum(
                    (self.arrival_times[i] + delta_a[i] - self.net.a_lb[node.replace("_prime", "")]) ** 2 - duals["mu"][
                        self.net.customers.index(node.replace("_prime", ""))]
                    for i, node in enumerate(prefix)
                )

                label_j.cost = self.cost + delta_a[-1] + rho + sum_term
            else:
                label_j.cost = self.cost + delta_a[-1]

            # update arrival times
            label_j.arrival_times = [0] + [self.arrival_times[i] + delta_num for i, delta_num in enumerate(delta_a)]
        else:  # farkas
            if node_j in self.net.customers:
                index = self.net.customers.index(node_j.replace("_prime", ""))
                label_j.cost = self.cost - duals["mu"][index]
            else:
                label_j.cost = self.cost

        # update auxiliary infos
        if arc in self.net.arcs_4 or arc in self.net.arcs_5:
            label_j.pending_flights.add(node_j)

        if arc in self.net.arcs_2:
            label_j.drone_flights[node_j] = label_j.pending_flights.copy()
            label_j.pending_flights.clear()

        if arc in self.net.arcs_1 or arc in self.net.arcs_2:
            label_j.truck_path = [node_j] + label_j.truck_path

        # if is_route_subset(test_path, (label_j.truck_path, label_j.drone_flights)):
        #     sdas = 0

        return label_j

    def measure_arrival_ubs(self):
        result = []
        first_node = self.path[0]
        arrive_time = self.net.a_lb[first_node]  # earliest arrival time
        result.append(self.net.max_timespan - arrive_time)
        sync_time = arrive_time
        wait_time = 0
        last_hub = first_node.replace("_prime", "") if first_node in self.net.hubs else None

        for j in range(1, len(self.path)):
            node_i = self.path[j - 1]
            node_j = self.path[j]

            arrive_time = get_arrive_time(arrive_time, node_i, node_j, last_hub, sync_time, wait_time, self.net)

            # update sync_time and last_hub
            if node_j in self.net.hubs:
                sync_time = arrive_time
                last_hub = node_j.replace("_prime", "")

            # update waiting time
            if (node_i, node_j) in self.net.arcs_3 or (node_i, node_j) in self.net.arcs_4:
                wait_time = max(wait_time, arrive_time - sync_time)
            else:
                wait_time = 0

            result.append(self.net.max_timespan - arrive_time)
        return result

    def __eq__(self, other):
        if isinstance(other, LabelBackward):
            return self.path == other.path  # Consider customers equal if they have the same ID
        return False

    def __hash__(self):
        return hash(tuple(self.path))  # Hash based on ID so customers with same ID are treated as the same

    def __lt__(self, other):
        """Defines how to compare two LabelForward objects"""
        return self.depth < other.depth  # Compare based on depth
