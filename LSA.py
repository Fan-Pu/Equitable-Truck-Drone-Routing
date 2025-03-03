import concurrent.futures
import time
import numpy as np
import heapq
import GeneralHelper
from GeneralHelper import *
from LabelForward import LabelForward
from LabelBackward import LabelBackward


class BiDirectionalLabelSetting:
    def __init__(self, duals):
        random.seed(seed)
        self.net = GeneralHelper.transformed_net
        self.forward_labels = {node: set() for node in self.net.all_nodes}
        self.backward_labels = {node: set() for node in self.net.all_nodes}
        self.best_solution = (np.inf, [])
        self.explored_solutions = set()  # explored labels
        self.duals = duals
        self.forward_label_queue = []  # priority queue, ordered by depth
        self.backward_label_queue = []
        self.explored_forward_paths = set()  # elementary paths that have been explored
        self.explored_backward_paths = set()
        self.forward_disposed_labels = set()  # labels dominated by other labels
        self.backward_disposed_labels = set()

    def forward_labeling_one_step(self, farkas, node_info):
        """Forward search from the depot."""
        new_labels_terminations = set()
        new_labels = set()

        _, label = heapq.heappop(self.forward_label_queue)
        node_i = label.path[-1]
        # check extending
        for node_j in self.net.out_arcs[node_i]:
            new_path = label.path + [node_j]
            # avoid re-exploration
            if tuple(new_path) in self.explored_forward_paths:
                continue
            if not label.allow_extend(node_j):
                continue

            label_j = label.extend(node_j, self.duals, farkas)
            # check dominance
            dominated_by_j, other_dominates_j = (
                self.dominance_check(label_j, self.forward_labels[node_j], farkas))
            self.forward_disposed_labels.update(dominated_by_j)
            # remove the labels dominated by j
            self.forward_labels[node_j].difference_update(dominated_by_j)
            # if j is not dominated by others
            if not other_dominates_j:
                self.forward_labels[node_j].add(label_j)
                # only if this label are possible to be extended
                if node_j != self.net.depot_sink and len(self.net.out_arcs[node_j]) > 0:
                    heapq.heappush(self.forward_label_queue, (-label_j.depth, label_j))
                    self.explored_forward_paths.add(tuple(label_j.path))
                # check whether it is completed, update the incumbent
                if node_j == self.net.depot_sink:
                    route_key = "-".join(label_j.path)
                    # do not consider the existing columns
                    if route_key not in node_info.columns:
                        if label_j.cost < self.best_solution[0]:
                            self.best_solution = (label_j.cost, label_j.path)
                        if tuple(label_j.path) not in self.explored_solutions:
                            self.explored_solutions.add(tuple(label_j.path))
                # only append the labels that have not been added
                new_labels.add(label_j)
                new_labels_terminations.add(node_j)

        return new_labels_terminations, new_labels

    def backward_labeling_one_step(self, farkas, node_info):
        """Backward search from the sink."""
        new_labels_terminations = set()
        new_labels = set()

        _, label = heapq.heappop(self.backward_label_queue)
        node_i = label.path[0]
        # check extending
        for node_j in self.net.in_arcs[node_i]:
            new_path = [node_j] + label.path
            # avoid re-exploration
            if tuple(new_path) in self.explored_backward_paths:
                continue

            if not label.allow_extend(node_j):
                continue

            label_j = label.extend(node_j, self.duals, farkas)

            if node_j == self.net.depot_source:  # once it is completed, we need to calculate the cost
                # normal mode
                if not farkas:
                    label_j.cost, arrive_times = cal_label_cost_normal(self.net, 0, 0, 0,
                                                                       -self.duals["nu"], label_j.path, self.duals)
                else:  # Farkas pricing
                    label_j.cost = cal_label_cost_farkas(self.net, label_j.path, self.duals)

            # check dominance
            dominated_by_j, other_dominates_j = (
                self.dominance_check(label_j, self.backward_labels[node_j], farkas))
            self.backward_disposed_labels.update(dominated_by_j)
            # remove the labels dominated by j
            self.backward_labels[node_j].difference_update(dominated_by_j)
            # if j is not dominated by others
            if not other_dominates_j:
                self.backward_labels[node_j].add(label_j)
                # only if this label are possibly to be extended
                if node_j != self.net.depot_source and len(self.net.in_arcs[node_j]) > 0:
                    heapq.heappush(self.backward_label_queue, (-label_j.depth, label_j))

                self.explored_backward_paths.add(tuple(label_j.path))

                # check whether it is completed, update incumbent
                if node_j == self.net.depot_source:
                    route_key = "-".join(label_j.path)
                    # do not consider the existing columns
                    if route_key not in node_info.columns:
                        if label_j.cost < self.best_solution[0]:
                            self.best_solution = (label_j.cost, label_j.path)
                        if tuple(label_j.path) not in self.explored_solutions:
                            self.explored_solutions.add(tuple(label_j.path))
                # only append the labels that have not been added
                new_labels.add(label_j)
                new_labels_terminations.add(node_j)

        return new_labels_terminations, new_labels

    def merge_labels(self, node, new_forward, new_backwards, farkas, node_info):
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
                    if tuple(complete_path) in self.explored_solutions:
                        continue
                    # this is a new unexplored label
                    last_hub = next(
                        (node.replace("_prime", "") for node in reversed(f_label.path) if node in self.net.hubs), None)
                    if not farkas:  # normal pricing
                        total_cost, _ = (
                            cal_label_cost_normal(self.net, f_label.arrival_time, f_label.sync_time, f_label.wait_time,
                                                  f_label.cost, b_label.path, self.duals, last_hub))
                    else:  # Farkas pricing
                        total_cost = f_label.cost + cal_label_cost_farkas(self.net, b_label.path[1:], self.duals)
                    GeneralHelper.label_merge_num += 1
                    # update incumbent
                    route_key = "-".join(complete_path)
                    if route_key not in node_info.columns:  # do not consider the existing columns
                        if total_cost < self.best_solution[0]:
                            self.best_solution = (total_cost, complete_path)
                        if tuple(complete_path) not in self.explored_solutions:
                            self.explored_solutions.add(tuple(complete_path))

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

    def solve(self, farkas, node_info):
        """
        Execute forward, backward, and merge steps.
        """

        # forward label initialization
        if not farkas:  # normal pricing
            heapq.heappush(self.forward_label_queue,
                           (0, LabelForward([self.net.depot_source], 0, 0, 0,
                                            0, 0, -self.duals["nu"], 0)))

        else:  # Farkas pricing
            heapq.heappush(self.forward_label_queue,
                           (0, LabelForward([self.net.depot_source], 0, 0, 0,
                                            0, 0, 0, 0)))

        # backward label initialization
        heapq.heappush(self.backward_label_queue,
                       (0, LabelBackward([self.net.depot_sink], 0, 0, [0],
                                         [self.net.max_timespan], 0, 0)))

        s_time = time.time()
        while True:
            # combined mode
            if LSA_mode == 0:
                # forward and backward labeling are both feasible
                if (self.best_solution[0] + close_tolerance >= 0
                        and self.forward_label_queue and self.backward_label_queue):
                    # two thread to paralleling handle bi-direction labeling
                    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
                        # Submit the tasks
                        future_forward = executor.submit(self.forward_labeling_one_step, farkas, node_info)
                        future_backward = executor.submit(self.backward_labeling_one_step, farkas, node_info)
                        # Wait for both tasks to complete and get results
                        forward_new_labels_terminations, new_f_labels = future_forward.result()
                        backward_new_labels_terminations, new_b_labels = future_backward.result()
                        # check label merges
                        nodes_updated = set(forward_new_labels_terminations) | set(backward_new_labels_terminations)
                        for node in nodes_updated:
                            self.merge_labels(node, new_f_labels, new_b_labels, farkas, node_info)
                # at least one direction is finished
                else:
                    self.print_runtime_info(s_time)
                    return self.best_solution
            # forward only
            elif LSA_mode == 1:
                while self.best_solution[0] + close_tolerance >= 0 and self.forward_label_queue:
                    self.forward_labeling_one_step(farkas, node_info)
                else:
                    self.print_runtime_info(s_time)
                    return self.best_solution
            # backward only
            elif LSA_mode == 2:
                while self.best_solution[0] + close_tolerance >= 0 and self.backward_label_queue:
                    self.backward_labeling_one_step(farkas, node_info)
                else:
                    self.print_runtime_info(s_time)
                    return self.best_solution

    def dominance_check(self, label_j, other_labels, farkas):
        """Check if label_j dominates any other label in a parallelized manner."""
        dominated_by_j = set()
        other_dominates_j = False

        for other in other_labels:
            j_dominates = label_j.dominates(other, farkas, self.duals)
            other_dominates = other.dominates(label_j, farkas, self.duals)

            if j_dominates:
                dominated_by_j.add(other)
            if other_dominates:
                other_dominates_j = True
                break  # Stop checking if label_j is already dominated

        return dominated_by_j, other_dominates_j

    def print_runtime_info(self, start_time):
        arrival_times, sync_times, wait_times = [0], [0], [0]
        last_hub = None
        obj, path = self.best_solution

        # for j in range(1, len(path)):
        #     node_i, node_j = path[j - 1], path[j]
        #     arrive_time = get_arrive_time(arrival_times[-1], node_i, node_j, last_hub, sync_times[-1], wait_times[-1],
        #                                   self.net)
        #
        #     sync_time = arrive_time if node_j in self.net.hubs else sync_times[-1]
        #     wait_time = max(wait_times[-1], arrive_time - sync_time) if ((node_i, node_j) in self.net.arcs_3
        #                                                                  or (node_i, node_j) in self.net.arcs_4) else 0
        #     last_hub = node_j.replace("_prime", "") if node_j in self.net.hubs else last_hub
        #
        #     arrival_times.append(arrive_time)
        #     sync_times.append(sync_time)
        #     wait_times.append(wait_time)
        # print(f"forward dominance check passed: {GeneralHelper.forward_dominance_num}")
        # print(f"backward dominance check passed: {GeneralHelper.backward_dominance_num}")
        # print(f"label merge found unexplored solutions: {GeneralHelper.label_merge_num}")
        # print(f"best solution: {self.best_solution[0]:.2f}, {self.best_solution[1]}")
        # print(f"runtime: {time.time() - start_time:.4f}s")
