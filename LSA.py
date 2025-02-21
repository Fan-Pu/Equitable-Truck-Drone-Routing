import concurrent.futures
import copy

import numpy as np

import GeneralHelper
from GeneralHelper import *

forward_dominance_num = 0
backward_dominance_num = 0


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

        if strict:
            global forward_dominance_num
            forward_dominance_num += 1

        # here all conditions are satisfied, we need at least one is strict
        return strict

    def allow_extend(self, node_j):
        """
        check whether we can extend the current label to node_j
        """
        result = True
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
                result = False
        else:  # not a customer node
            if node_j in self.path:
                result = False

        # check condition 2
        if arc in self.net.arcs_4:
            hub = get_latest_hub(self.net, self.path)
            if (hub, _node_j) not in self.net.origin_drone_arcs:
                result = False

        # check condition 3
        if arc in self.net.arcs_5:
            hub = get_latest_hub(self.net, self.path)
            if (hub, _node_j) not in self.net.origin_truck_arcs:
                result = False

        # check condition 4
        if self.arrival_time < self.net.a_lb[node_i]:
            result = False

        # check condition 5
        if self.truck_load + self.net.demand_weights[node_j] > truck_max_weight:
            result = False

        # check condition 6
        if arc in self.net.arcs_3 or arc in self.net.arcs_4:
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
        hub = get_latest_hub(self.net, self.path)
        label_j.arrival_time = get_arrive_time(self.arrival_time, node_i, node_j, hub, self.sync_time, self.net)

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
        self.cost = None

    def dominates(self, other, duals):
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

        if strict:
            global backward_dominance_num
            backward_dominance_num += 1

        # here all conditions are satisfied, we need at least one is strict
        return strict

    def allow_extend(self, node_j):
        """
        check whether we can extend the current label to node_j
        """
        result = True
        node_i = self.path[0]
        _node_j = node_j.replace("_prime", "")

        if self.path == test_path and node_j == "Source":
            sdas = 0

        # check condition 1
        if node_j in self.net.customers:  # a customer node
            if "_prime" in node_j:  # node_j is a duplication
                dup_node = _node_j
            else:
                dup_node = node_j + "_prime"
            if node_j in self.path or dup_node in self.path:
                result = False
        else:  # not a customer node
            if node_j in self.path:
                result = False

        # check condition 2
        if self.truck_load + self.net.demand_weights[node_j] > truck_max_weight:
            result = False

        # check condition 3
        if (node_i, node_j) in self.net.arcs_3 or (node_i, node_j) in self.net.arcs_4:
            if self.drones_used >= num_drones_per_truck:
                result = False

        # check condition 4
        if node_j in self.net.hubs and "_prime" in node_j:
            # find the prefix
            prefix = [self.path[0]]

            for i, (node, node_next) in enumerate(zip(self.path, self.path[1:])):
                prefix.append(node_next)
                if (node, node_next) in self.net.arcs_5:
                    break
            else:
                raise Exception("prefix is not found!")

            # check the validity
            for node in prefix:
                _node = node.replace("_prime", "")
                if node == prefix[-1]:
                    if (_node_j, _node) not in self.net.origin_truck_arcs:
                        result = False
                else:
                    if (_node_j, _node) not in self.net.origin_drone_arcs:
                        result = False

        return result

    def extend(self, node_j):
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
        first_node = self.path[0]
        arrive_time = self.net.a_lb[first_node]  # earliest arrival time
        result.append(self.net.max_timespan - arrive_time)
        sync_time = arrive_time
        last_hub = first_node.replace("_prime", "") if first_node in self.net.hubs else None

        for j in range(1, len(self.path)):
            node_i = self.path[j - 1]
            node_j = self.path[j]

            arrive_time = get_arrive_time(arrive_time, node_i, node_j, last_hub, sync_time, self.net)

            # update sync_time and last_hub
            if node_j in self.net.hubs:
                sync_time = arrive_time
                last_hub = node_j.replace("_prime", "")

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

    def forward_labeling_one_step(self):
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
                        self.forward_labels[node_j].append(label_j) if label_j not in self.forward_labels[
                            node_j] else None
                        # only if this label are possible to be extended
                        if label_j.path[-1] != self.net.depot_sink and len(self.net.out_arcs[label_j.path[-1]]) > 0:
                            self.forward_label_list.append(label_j)
                            self.explored_forward_paths.append(label_j.path)
                        # check whether it is completed
                        if label_j.path[-1] == self.net.depot_sink:
                            if label_j.cost < self.best_solution[0]:
                                self.best_solution = (label_j.cost, label_j.path)
                            if label_j.path not in self.explored_solutions:
                                self.explored_solutions.append(label_j.path)

                        new_labels.append(label_j)
                        new_labels_terminations.append(node_j) if node_j not in new_labels_terminations else None
        return new_labels_terminations, new_labels

    def backward_labeling_one_step(self):
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
                    label_j = label.extend(node_j)
                    complete = False  # whether label_j is complete
                    if label_j.path[0] == self.net.depot_source:  # check whether it is completed
                        complete = True
                        label_j.cost = self.calculate_cost(0, 0, -self.duals["nu"],
                                                           label_j.path, self.duals)

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
                        self.backward_labels[node_j].append(label_j) if label_j not in self.backward_labels[
                            node_j] else None
                        # only if this label are possibly to be extended
                        if label_j.path[0] != self.net.depot_source and len(self.net.in_arcs[label_j.path[0]]) > 0:
                            self.backward_label_list.append(label_j)
                            self.explored_backward_paths.append(label_j.path)
                        # check whether it is completed, update incumbent
                        if complete:
                            if label_j.cost < self.best_solution[0]:
                                self.best_solution = (label_j.cost, label_j.path)
                            if label_j.path not in self.explored_solutions:
                                self.explored_solutions.append(label_j.path)

                        new_labels.append(label_j)
                        new_labels_terminations.append(node_j) if node_j not in new_labels_terminations else None
        return new_labels_terminations, new_labels

    def merge_labels(self, node, new_forward, new_backwards):
        """Merge forward and backward labels at common nodes."""
        for f_label in self.forward_labels[node]:
            for b_label in self.backward_labels[node]:
                # we do not check the merge between an old label to another old label
                if f_label not in new_forward and b_label not in new_backwards:
                    continue
                # at least one is the new label
                if (set(f_label.path) & set(b_label.path)) == {node} and \
                        f_label.truck_load + b_label.truck_load <= truck_max_weight and \
                        f_label.drones_used + b_label.drones_used <= num_drones_per_truck:
                    complete_path = f_label.path + b_label.path[1:]
                    # avoid multi comparisons
                    if complete_path in self.explored_solutions:
                        continue
                    total_cost = self.calculate_cost(f_label.arrival_time, f_label.sync_time, f_label.cost,
                                                     b_label.path, self.duals)
                    if total_cost < self.best_solution[0]:
                        self.best_solution = (total_cost, complete_path)
                    if complete_path not in self.explored_solutions:
                        self.explored_solutions.append(complete_path)

    def solve(self):
        """
        Execute forward, backward, and merge steps.
        """

        # self.forward_label_list.append(LabelForward(
        #     [self.net.depot_source], 0, 0, 0, 0, -self.duals["nu"])
        # )

        self.backward_label_list.append(LabelBackward(
            [self.net.depot_sink], 0, 0, [self.net.max_timespan])
        )

        while True:
            # forward and backward labeling are feasible
            if len(self.forward_label_list) > 0 and len(self.backward_label_list) > 0:
                # two thread to paralleling handle bi-direction labeling
                with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
                    # Submit the tasks
                    future_forward = executor.submit(self.forward_labeling_one_step)
                    future_backward = executor.submit(self.backward_labeling_one_step)
                    # Wait for both tasks to complete and get results
                    forward_new_labels_terminations, new_f_labels = future_forward.result()
                    backward_new_labels_terminations, new_b_labels = future_backward.result()
                    # check label merges
                    nodes_updated = list(set(forward_new_labels_terminations) | set(backward_new_labels_terminations))
                    for node in nodes_updated:
                        self.merge_labels(node, new_f_labels, new_b_labels)
            # only forward labeling is feasible
            elif len(self.forward_label_list) > 0:
                forward_new_labels_terminations, new_f_labels = self.forward_labeling_one_step()
                # check label merges
                nodes_updated = forward_new_labels_terminations
                for node in nodes_updated:
                    self.merge_labels(node, new_f_labels, [])
            # only backward labeling is feasible
            elif len(self.backward_label_list) > 0:
                backward_new_labels_terminations, new_b_labels = self.backward_labeling_one_step()
                # check label merges
                nodes_updated = backward_new_labels_terminations
                for node in nodes_updated:
                    self.merge_labels(node, [], new_b_labels)
            # no labels can be extended
            else:
                arrival_times, sync_times = [0], [0]
                last_hub = None
                obj, path = self.best_solution

                for j in range(1, len(path)):
                    node_i, node_j = path[j - 1], path[j]
                    arrive_time = get_arrive_time(arrival_times[-1], node_i, node_j, last_hub, sync_times[-1], self.net)

                    sync_time = arrive_time if node_j in self.net.hubs else sync_times[-1]
                    last_hub = node_j.replace("_prime", "") if node_j in self.net.hubs else last_hub

                    arrival_times.append(arrive_time)
                    sync_times.append(sync_time)
                print(forward_dominance_num)
                print(backward_dominance_num)
                return self.best_solution

    def calculate_cost(self, arrival_time_i, sync_time_i, cost_i, partial_path, duals):
        """
        node_i is the first point on the partial path, calculate the cost of this partial path
        """
        sync_time = sync_time_i
        arrival_time = arrival_time_i
        last_hub = None
        full_cost = cost_i
        for j in range(1, len(partial_path)):
            node_pre = partial_path[j - 1]
            node_j = partial_path[j]
            # update arrival time
            arrival_time = get_arrive_time(arrival_time, node_pre, node_j, last_hub, sync_time, self.net)
            # update sync time
            if node_j in self.net.hubs:
                sync_time = arrival_time
                last_hub = node_j.replace("_prime", "")
            else:
                pass
            # update cost
            if node_j in self.net.customers:
                if "_prime" in node_j:
                    temp_str = node_j.replace("_prime", "")
                    index = self.net.customers.index(temp_str)
                else:
                    index = self.net.customers.index(node_j)
                full_cost += (arrival_time - self.net.a_lb[node_j]) ** 2 - duals["mu"][index]
            elif node_j == self.net.depot_sink:
                full_cost += arrival_time
            else:
                pass

        return full_cost
