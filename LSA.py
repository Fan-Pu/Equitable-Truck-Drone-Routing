import concurrent.futures
import copy

import GeneralHelper
from GeneralHelper import *


class LabelForward:
    def __init__(self, path, truck_load, drones_used, arrival_time, sync_time, cost):
        self.path = path  # Ordered sequence of visited nodes (partial path)
        self.truck_load = truck_load  # Truck loading weight
        self.drones_used = drones_used  # Number of drones used
        self.arrival_time = arrival_time  # Arrival time at last node
        self.sync_time = sync_time  # Arrival time at last synchronization point
        self.cost = cost  # Accumulated cost
        self.net = GeneralHelper.transformed_net

    def dominates(self, other):
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
        if self.arrival_time > other.arrival_time:
            return False
        elif self.arrival_time < other.arrival_time:
            strict = True

        # condition 2.3 violated
        if self.sync_time > other.sync_time:
            return False
        elif self.sync_time < other.sync_time:
            strict = True

        # here all conditions are satisfied, we need at least one is strict
        return strict

    def allow_extend(self, node_j):
        """
        check whether we can extend the current label to node_j
        """
        result = True
        node_i = self.path[-1]

        # check the first condition
        if node_j in self.path:
            result = False

        # check the second condition
        if self.arrival_time < self.net.a_lb[node_i]:
            result = False

        # check the third condition
        if self.truck_load + self.net.demand_weights[node_j] > truck_max_weight:
            result = False

        # check the last condition
        if (node_i, node_j) in self.net.arcs_3 or (node_i, node_j) in self.net.arcs_4:
            if self.drones_used >= num_drones_per_truck:
                result = False

        return result

    def extend(self, node_j, duals):
        """
        extend the label to j
        """
        node_i = self.path[-1]
        label_j = copy.deepcopy(self)
        label_j.path.append(node_j)
        # update z
        if (node_i, node_j) in self.net.arcs_3 or (node_i, node_j) in self.net.arcs_4:
            label_j.drones_used += 1
        else:
            label_j.drones_used = 0
        # update a
        if (node_i, node_j) in self.net.arcs_1 or (node_i, node_j) in self.net.arcs_5:
            label_j.arrival_time += self.net.travel_times[(node_i, node_j)]
        elif (node_i, node_j) in self.net.arcs_2:
            pass
        else:
            label_j.arrival_time = label_j.sync_time + self.net.travel_times[(node_i, node_j)]
        # update sync time
        if node_j in self.net.hubs:
            label_j.sync_time = label_j.arrival_time
        else:
            pass
        # update cost
        if node_j in self.net.customers:
            if "_prime" in node_j:
                temp_str = node_j.replace("_prime", "")
                index = self.net.customers.index(temp_str)
            else:
                index = self.net.customers.index(node_j)
            label_j.cost += (label_j.arrival_time - self.net.a_lb[node_j]) ** 2 - duals["mu"][index]
        elif node_j == self.net.depot_sink:
            label_j.cost += label_j.arrival_time
        else:
            pass
        return label_j


class LabelBackward:
    def __init__(self, path, truck_load, drones_used, arrival_ubs):
        self.path = path  # Ordered sequence of visited nodes
        self.truck_load = truck_load  # Truck loading weight
        self.drones_used = drones_used  # Number of drones used
        self.arrival_ubs = arrival_ubs  # Arrival time upper bounds
        self.net = GeneralHelper.transformed_net

    def dominates(self, other, duals):
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

        customer_on_path1 = [node for node in self.path if node in self.net.customers]
        customer_on_path2 = [node for node in other.path if node in self.net.customers]

        lhs = rhs = 0
        for i in range(len(customer_on_path1)):
            node = customer_on_path1[i]
            if i == len(customer_on_path1) - 1:  # depot sink node
                lhs += (self.net.a_lb[node] - self.arrival_ubs[i])
            else:
                lhs -= (self.arrival_ubs[i] - self.net.a_lb[node]) ** 2

        for node in customer_on_path2:
            if node in customer_on_path1:
                continue
            if "_prime" in node:
                temp_str = node.replace("_prime", "")
                index = self.net.customers.index(temp_str)
            else:
                index = self.net.customers.index(node)
            rhs += duals["mu"][index]

        if lhs < rhs:
            return False
        elif lhs > rhs:
            strict = True

        # here all conditions are satisfied, we need at least one is strict
        return strict

    def allow_extend(self, node_j):
        """
        check whether we can extend the current label to node_j
        """
        result = True
        node_i = self.path[-1]

        # check the first condition
        if node_j in self.path:
            result = False

        # check the second condition
        if self.truck_load + self.net.demand_weights[node_j] > truck_max_weight:
            result = False

        # check the last condition
        if (node_i, node_j) in self.net.arcs_3 or (node_i, node_j) in self.net.arcs_4:
            if self.drones_used >= num_drones_per_truck:
                result = False

        return result

    def extend(self, node_j, duals):
        """
        extend the label to j
        """
        node_i = self.path[0]
        label_j = copy.deepcopy(self)
        label_j.path.insert(0, node_j)
        # update z
        if (node_j, node_i) in self.net.arcs_3 or (node_j, node_i) in self.net.arcs_4:
            label_j.drones_used += 1
        else:
            label_j.drones_used = 0
        # update T
        label_j.arrival_ubs = self.measure_arrival_ubs()

        return label_j

    def measure_arrival_ubs(self):
        result = []
        arrive_time = self.net.a_lb[self.path[0]]  # earliest arrival time
        result.append(self.net.max_timespan - arrive_time)
        sync_time = arrive_time
        for j in range(1, len(self.path)):
            node_i = self.path[j - 1]
            node_j = self.path[j]
            # update arrive time
            if (node_i, node_j) in self.net.arcs_1 or (node_i, node_j) in self.net.arcs_5:
                arrive_time = arrive_time + self.net.travel_times[(node_i, node_j)]
            elif (node_i, node_j) in self.net.arcs_2:
                pass
            else:
                arrive_time = sync_time + self.net.travel_times[(node_i, node_j)]
            # update sync_time
            if node_j in self.net.hubs:
                sync_time = arrive_time
            else:
                pass

            result.append(self.net.max_timespan - arrive_time)
        return result


class BiDirectionalLabelSetting:
    def __init__(self, duals):
        self.net = GeneralHelper.transformed_net
        self.forward_labels = {node: [] for node in self.net.all_nodes}
        self.backward_labels = {node: [] for node in self.net.all_nodes}
        self.solutions = []
        self.duals = duals
        self.forward_label_list = []
        self.backward_label_list = []

    def forward_labeling_one_step(self):
        """Forward search from the depot."""
        new_labels_terminations = []
        while len(self.forward_label_list) > 0:
            label = self.forward_label_list.pop()
            node_i = label.path[-1]
            # check extending
            for node_j in self.net.out_arcs[node_i]:
                is_feasible = label.allow_extend(node_j)
                if is_feasible:
                    label_j = label.extend(node_j, self.duals)
                    # check dominance
                    dominated_by_j = []
                    other_dominates_j = False
                    for other_label in self.forward_labels[node_j]:
                        if label_j.dominates(other_label):
                            dominated_by_j.append(other_label)
                        elif other_label.dominates(label_j):
                            other_dominates_j = True
                            break  # discard label_j
                    # remove the labels dominated by j
                    self.forward_labels[node_j] = list(set(self.forward_labels[node_j]) - set(dominated_by_j))
                    self.forward_label_list = list(set(self.forward_label_list) - set(dominated_by_j))
                    # if j is not dominated, save it
                    if not other_dominates_j:
                        self.forward_labels[node_j].append(label_j)
                        self.forward_label_list.append(label_j)
                        new_labels_terminations.append(node_j)
        return new_labels_terminations

    def backward_labeling_one_step(self):
        """Backward search from the sink."""
        new_labels_terminations = []
        while len(self.backward_label_list) > 0:
            label = self.backward_label_list.pop()
            node_i = label.path[-1]
            # check extending
            for node_j in self.net.in_arcs[node_i]:
                is_feasible = label.allow_extend(node_j)
                if is_feasible:
                    label_j = label.extend(node_j, self.duals)
                    # check dominance
                    dominated_by_j = []
                    other_dominates_j = False
                    for other_label in self.backward_labels[node_j]:
                        if label_j.dominates(other_label, self.duals):
                            dominated_by_j.append(other_label)
                        elif other_label.dominates(label_j, self.duals):
                            other_dominates_j = True
                            break  # discard label_j
                    # remove the labels dominated by j
                    self.backward_labels[node_j] = list(set(self.backward_labels[node_j]) - set(dominated_by_j))
                    self.backward_label_list = list(set(self.backward_label_list) - set(dominated_by_j))
                    # if j is not dominated, save it
                    if not other_dominates_j:
                        self.backward_labels[node_j].append(label_j)
                        self.backward_label_list.append(label_j)
                        new_labels_terminations.append(node_j)
        return new_labels_terminations

    def merge_labels(self):
        """Merge forward and backward labels at common nodes."""
        for node in self.graph:
            for f_label in self.forward_labels[node]:
                for b_label in self.backward_labels[node]:
                    if (set(f_label.path) & set(b_label.path)) == {node} and \
                            f_label.truck_load + b_label.truck_load <= self.truck_capacity and \
                            f_label.drones_used + b_label.drones_used <= self.drone_limit:
                        total_cost = f_label.cost + b_label.cost
                        complete_path = f_label.path + b_label.path[1:]
                        self.solutions.append((total_cost, complete_path))

    def solve(self):
        """Execute forward, backward, and merge steps."""
        self.forward_label_list.append(LabelForward(
            [self.net.depot_source], 0, 0, 0, 0, -self.duals["nu"])
        )
        self.backward_label_list.append(LabelBackward(
            [self.net.depot_sink], 0, 0, [self.net.max_timespan])
        )

        forward_new_labels_terminations = backward_new_labels_terminations = []
        with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
            # Submit the tasks
            future_forward = executor.submit(self.forward_labeling_one_step)
            future_backward = executor.submit(self.backward_labeling_one_step)
            # Wait for both tasks to complete and get results
            forward_new_labels_terminations = future_forward.result()
            backward_new_labels_terminations = future_backward.result()

        self.merge_labels()
        return min(self.solutions, key=lambda x: x[0]) if self.solutions else None
