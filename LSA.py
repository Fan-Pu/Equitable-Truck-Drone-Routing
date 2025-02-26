import concurrent.futures
import time
import numpy as np

import GeneralHelper
from GeneralHelper import *


class LabelForward:
    def __init__(self, path, truck_load, drones_used, arrival_time, sync_time, wait_time, cost):
        self.path = path  # Ordered sequence of visited nodes (partial path)
        self.truck_load = truck_load  # Truck loading weight
        self.drones_used = drones_used  # Number of drones used
        self.arrival_time = arrival_time  # Arrival time at last node
        self.sync_time = sync_time  # Arrival time at last synchronization point
        self.wait_time = wait_time  # truck waiting time
        self.cost = cost  # Accumulated cost
        self.net = GeneralHelper.transformed_net

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
            cost=self.cost
        )

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

        if is_sublist_ordered(self.path, test_path):
            idx_last = test_path.index(self.path[-1])
            if node_j in test_path and test_path.index(node_j) == idx_last + 1:
                sdas = 0

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
        self.cost = None

    def dominates(self, other, farkas, duals):
        """Check if this label dominates another."""
        strict = False  # check whether contains a strict condition
        # the path is complete
        if self.path[0] == self.net.depot_source:
            # compare the cost
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

        customer_on_path1 = [node for node in self.path if node in self.net.customers]
        customer_on_path2 = [node for node in other.path if node in self.net.customers]

        if not farkas:
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
                index = self.net.customers.index(node.replace("_prime", ""))
                rhs += duals["mu"][index]

            if lhs < rhs:
                return False
            elif lhs > rhs:
                strict = True
        else:
            c_1 = c_2 = 0
            for node in customer_on_path1:
                index = self.net.customers.index(node.replace("_prime", ""))
                c_1 -= duals["mu"][index]
            for node in customer_on_path2:
                index = self.net.customers.index(node.replace("_prime", ""))
                c_2 -= duals["mu"][index]
            if c_1 > c_2:
                return False
            elif c_1 < c_2:
                strict = True

        if strict:
            GeneralHelper.backward_dominance_num += 1

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
        if (node_i, node_j) in self.net.arcs_3.union(self.net.arcs_4):  # Set union for efficiency
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

    def extend(self, node_j, farkas):
        """
        extend the label to j
        """
        node_i = self.path[0]

        # Use a shallow copy where possible to avoid unnecessary list duplications
        label_j = self.__class__(
            path=[node_j] + self.path,
            truck_load=self.truck_load,
            drones_used=self.drones_used,  # This may be updated below
            arrival_ubs=self.arrival_ubs[:]
        )
        # Update drones_used efficiently
        label_j.drones_used = self.drones_used + 1 if (node_j, node_i) in self.net.arcs_3 or (
            node_j, node_i) in self.net.arcs_4 else 0

        # Update arrival_ubs only when necessary
        if not farkas:
            label_j.arrival_ubs = self.measure_arrival_ubs()

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


class BiDirectionalLabelSetting:
    def __init__(self, duals):
        self.net = GeneralHelper.transformed_net
        self.forward_labels = {node: [] for node in self.net.all_nodes}
        self.backward_labels = {node: [] for node in self.net.all_nodes}
        self.best_solution = (np.inf, [])
        self.explored_solutions = []
        self.duals = duals
        self.forward_label_list = []
        self.backward_label_list = []
        self.explored_forward_paths = []
        self.explored_backward_paths = []

    def forward_labeling_one_step(self, farkas):
        """Forward search from the depot."""
        new_labels_terminations = []
        new_labels = []
        if len(self.forward_label_list) > 0:
            label = self.forward_label_list.pop()
            node_i = label.path[-1]
            # check extending
            for node_j in self.net.out_arcs[node_i]:
                new_path = label.path + [node_j]
                # avoid re-explore
                if new_path in self.explored_forward_paths:
                    continue

                is_feasible = label.allow_extend(node_j)
                if is_feasible:
                    label_j = label.extend(node_j, self.duals, farkas)
                    is_new_label = True if label_j not in self.forward_labels[node_j] else False
                    # check dominance
                    dominated_by_j, other_dominates_j = (
                        self.dominance_check(label_j, self.forward_labels[node_j], farkas))
                    # remove the labels dominated by j
                    self.forward_labels[node_j] = list(set(self.forward_labels[node_j]) - set(dominated_by_j))
                    self.forward_label_list = list(set(self.forward_label_list) - set(dominated_by_j))
                    # if j is not dominated, save it
                    if not other_dominates_j:
                        self.forward_labels[node_j].append(label_j) if is_new_label else None
                        # only if this label are possible to be extended
                        if node_j != self.net.depot_sink and len(self.net.out_arcs[node_j]) > 0:
                            self.forward_label_list.append(label_j)
                            self.explored_forward_paths.append(label_j.path)
                        # check whether it is completed
                        if node_j == self.net.depot_sink:
                            if label_j.cost < self.best_solution[0]:
                                self.best_solution = (label_j.cost, label_j.path)
                            if label_j.path not in self.explored_solutions:
                                self.explored_solutions.append(label_j.path)
                        # only append the labels that have not been added
                        new_labels.append(label_j) if is_new_label else None
                        new_labels_terminations.append(node_j) if node_j not in new_labels_terminations else None
        return new_labels_terminations, new_labels

    def backward_labeling_one_step(self, farkas):
        """Backward search from the sink."""
        new_labels_terminations = []
        new_labels = []
        if len(self.backward_label_list) > 0:
            label = self.backward_label_list.pop()
            node_i = label.path[0]
            # check extending
            for node_j in self.net.in_arcs[node_i]:
                new_path = [node_j] + label.path
                # avoid re-explore
                if new_path in self.explored_backward_paths:
                    continue

                is_feasible = label.allow_extend(node_j)
                if is_feasible:
                    label_j = label.extend(node_j, farkas)

                    is_new_label = True if label_j not in self.backward_labels[node_j] else False
                    complete = False  # whether label_j is complete
                    if node_j == self.net.depot_source:  # check whether it is completed
                        complete = True
                        label_j.cost = self.calculate_cost(0, 0, 0, -self.duals["nu"],
                                                           label_j.path, self.duals)

                    # check dominance
                    dominated_by_j, other_dominates_j = (
                        self.dominance_check(label_j, self.backward_labels[node_j], farkas))
                    # remove the labels dominated by j
                    self.backward_labels[node_j] = list(set(self.backward_labels[node_j]) - set(dominated_by_j))
                    self.backward_label_list = list(set(self.backward_label_list) - set(dominated_by_j))
                    # if j is not dominated, save it
                    if not other_dominates_j:
                        self.backward_labels[node_j].append(label_j) if label_j not in self.backward_labels[
                            node_j] else None
                        # only if this label are possibly to be extended
                        if node_j != self.net.depot_source and len(self.net.in_arcs[node_j]) > 0:
                            self.backward_label_list.append(label_j)
                            self.explored_backward_paths.append(label_j.path)
                        # check whether it is completed, update incumbent
                        if complete:
                            if label_j.cost < self.best_solution[0]:
                                self.best_solution = (label_j.cost, label_j.path)
                            if label_j.path not in self.explored_solutions:
                                self.explored_solutions.append(label_j.path)

                        new_labels.append(label_j) if is_new_label else None
                        new_labels_terminations.append(node_j) if node_j not in new_labels_terminations else None
        return new_labels_terminations, new_labels

    def merge_labels(self, node, new_forward, new_backwards, farkas):
        """Merge forward and backward labels at common nodes."""
        for f_label in self.forward_labels[node]:
            for b_label in self.backward_labels[node]:
                # we do not check the merge between an old label to another old label
                if f_label not in new_forward and b_label not in new_backwards:
                    continue
                # at least one is the new label
                if self.check_merge_feasibility(f_label, b_label, f_label.path[-1]):
                    complete_path = f_label.path + b_label.path[1:]
                    # avoid multi comparisons
                    if complete_path in self.explored_solutions:
                        continue
                    # this is a new unexplored label
                    last_hub = next(
                        (node.replace("_prime", "") for node in reversed(f_label.path) if node in self.net.hubs), None)
                    if not farkas:
                        total_cost = self.calculate_cost(f_label.arrival_time, f_label.sync_time, f_label.cost,
                                                         b_label.path, self.duals, last_hub)
                    else:
                        total_cost = self.calculate_cost_farkas(f_label.cost, b_label.path, self.duals)
                    GeneralHelper.label_merge_num += 1
                    # update incumbent
                    if total_cost < self.best_solution[0]:
                        self.best_solution = (total_cost, complete_path)
                    if complete_path not in self.explored_solutions:
                        self.explored_solutions.append(complete_path)

    def check_merge_feasibility(self, f_label, b_label, common_node):
        if (set(f_label.path) & set(b_label.path)) != {common_node}:
            return False
        if f_label.truck_load + b_label.truck_load > truck_max_weight:
            return False
        if f_label.drones_used + b_label.drones_used > num_drones_per_truck:
            return False
        hub, idx = get_latest_hub(self.net, f_label.path)
        # condition 2
        if "_prime" in common_node and common_node in self.net.customers:
            # find the prefix
            prefix = f_label.path[idx + 1:-1] + [b_label.path[0]]

            for i, (node, node_next) in enumerate(zip(b_label.path, b_label.path[1:])):
                prefix.append(node_next)
                if (node, node_next) in self.net.arcs_5:
                    break
            else:
                raise Exception("prefix is not found!")
            # check the validity
            for node in prefix:
                _node = node.replace("_prime", "")
                if node == prefix[-1]:
                    if (hub, _node) not in self.net.origin_truck_arcs:
                        return False
                else:
                    if (hub, _node) not in self.net.origin_drone_arcs:
                        return False
        # condition 3
        node_i = common_node
        arrive_time = f_label.arrival_time
        sync_time = f_label.sync_time
        wait_time = f_label.wait_time
        for node_j in b_label.path[1:]:
            arrive_time = get_arrive_time(arrive_time, node_i, node_j, hub, sync_time, wait_time, self.net)
            if node_j in self.net.customers and arrive_time < self.net.a_lb[node_j.replace("_prime", "")]:
                return False
            node_i = node_j
            if node_j in self.net.hubs:
                sync_time = arrive_time
                hub = node_j.replace("_prime", "")
            if (node_i, node_j) in self.net.arcs_3 or (node_i, node_j) in self.net.arcs_4:
                wait_time = max(wait_time, arrive_time - sync_time)
            else:
                wait_time = 0

        return True

    def solve(self, farkas):
        """
        Execute forward, backward, and merge steps.
        """

        self.forward_label_list.append(LabelForward(
            [self.net.depot_source], 0, 0, 0, 0, 0, -self.duals["nu"])
        )

        self.backward_label_list.append(LabelBackward(
            [self.net.depot_sink], 0, 0, [self.net.max_timespan])
        )

        s_time = time.time()
        while True:
            # combined mode
            if LSA_mode == 0:
                # forward and backward labeling are feasible
                if len(self.forward_label_list) > 0 and len(self.backward_label_list) > 0:
                    # two thread to paralleling handle bi-direction labeling
                    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
                        # Submit the tasks
                        future_forward = executor.submit(self.forward_labeling_one_step, farkas)
                        future_backward = executor.submit(self.backward_labeling_one_step, farkas)
                        # Wait for both tasks to complete and get results
                        forward_new_labels_terminations, new_f_labels = future_forward.result()
                        backward_new_labels_terminations, new_b_labels = future_backward.result()
                        # check label merges
                        nodes_updated = list(
                            set(forward_new_labels_terminations) | set(backward_new_labels_terminations))
                        for node in nodes_updated:
                            self.merge_labels(node, new_f_labels, new_b_labels, farkas)
                # at least one direction is finished
                else:
                    self.print_runtime_info(s_time)
                    return self.best_solution
            # forward only
            elif LSA_mode == 1:
                if len(self.forward_label_list) > 0:
                    forward_new_labels_terminations, new_f_labels = self.forward_labeling_one_step(farkas)
                    # check label merges
                    nodes_updated = forward_new_labels_terminations
                    for node in nodes_updated:
                        self.merge_labels(node, new_f_labels, [], farkas)
                else:
                    self.print_runtime_info(s_time)
                    return self.best_solution
            # backward only
            elif LSA_mode == 2:
                if len(self.backward_label_list) > 0:
                    backward_new_labels_terminations, new_b_labels = self.backward_labeling_one_step(farkas)
                    # check label merges
                    nodes_updated = backward_new_labels_terminations
                    for node in nodes_updated:
                        self.merge_labels(node, [], new_b_labels, farkas)
                else:
                    self.print_runtime_info(s_time)
                    return self.best_solution

    def calculate_cost(self, arrival_time_i, sync_time_i, wait_time_i, cost_i, partial_path, duals, last_hub=None):
        """
        node_i is the first point on the partial path, calculate the cost of this partial path
        """
        sync_time = sync_time_i
        arrival_time = arrival_time_i
        wait_time = wait_time_i
        full_cost = cost_i
        for j in range(1, len(partial_path)):
            node_pre = partial_path[j - 1]
            node_j = partial_path[j]
            # update arrival time
            arrival_time = get_arrive_time(arrival_time, node_pre, node_j, last_hub, sync_time, wait_time, self.net)
            # update sync time
            if node_j in self.net.hubs:
                sync_time = arrival_time
                last_hub = node_j.replace("_prime", "")
            else:
                pass
            # update cost
            if node_j in self.net.customers:
                index = self.net.customers.index(node_j.replace("_prime", ""))
                full_cost += (arrival_time - self.net.a_lb[node_j]) ** 2 - duals["mu"][index]
            elif node_j == self.net.depot_sink:
                full_cost += arrival_time
            else:
                pass

        return full_cost

    def calculate_cost_farkas(self, cost_i, partial_path, duals):
        """
        node_i is the first point on the partial path, calculate the cost of this partial path
        """
        full_cost = cost_i
        for j in range(1, len(partial_path)):
            node_j = partial_path[j]
            # update cost
            if node_j in self.net.customers:
                index = self.net.customers.index(node_j.replace("_prime", ""))
                full_cost -= duals["mu"][index]

        return full_cost

    def dominance_check(self, label_j, other_labels, farkas):
        """Check if label_j dominates any other label in a parallelized manner."""
        dominated_by_j = []
        other_dominates_j = False

        for other in other_labels:
            j_dominates = label_j.dominates(other, farkas, self.duals)
            other_dominates = other.dominates(label_j, farkas, self.duals)

            if j_dominates:
                dominated_by_j.append(other)
            if other_dominates:
                other_dominates_j = True
                break  # Stop checking if label_j is already dominated

        return dominated_by_j, other_dominates_j

    def print_runtime_info(self, start_time):
        arrival_times, sync_times = [0], [0]
        last_hub = None
        obj, path = self.best_solution

        # for j in range(1, len(path)):
        #     node_i, node_j = path[j - 1], path[j]
        #     arrive_time = get_arrive_time(arrival_times[-1], node_i, node_j, last_hub, sync_times[-1], self.net)
        #
        #     sync_time = arrive_time if node_j in self.net.hubs else sync_times[-1]
        #     last_hub = node_j.replace("_prime", "") if node_j in self.net.hubs else last_hub
        #
        #     arrival_times.append(arrive_time)
        #     sync_times.append(sync_time)
        # print(f"forward dominance check passed: {GeneralHelper.forward_dominance_num}")
        # print(f"backward dominance check passed: {GeneralHelper.backward_dominance_num}")
        # print(f"label merge found unexplored solutions: {GeneralHelper.label_merge_num}")
        # print(f"best solution: {self.best_solution[0]:.2f}, {self.best_solution[1]}")
        # print(f"runtime: {time.time() - start_time:.4f}s")
