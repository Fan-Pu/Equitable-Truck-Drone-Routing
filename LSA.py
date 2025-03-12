import concurrent.futures
import threading
import time
import numpy as np
import queue
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
        self.best_solution = (np.inf, None, -1)  # the third element: 0 from merge, 1 from forward, 2 from backward
        self.explored_solutions = set()  # explored labels
        self.duals = duals
        self.forward_label_queue = queue.PriorityQueue()  # priority queue, ordered by depth
        self.backward_label_queue = queue.PriorityQueue()
        self.forward_disposed_labels = set()  # labels dominated by other labels
        self.backward_disposed_labels = set()
        self.parallel_lock = threading.Lock()
        self.print_lock = threading.Lock()
        self.termination_event = threading.Event()  # Shared flag for stopping threads

    def forward_labeling_one_step(self, farkas, node_info):
        """Forward search from the depot."""
        new_labels = defaultdict(list)

        _, label = self.forward_label_queue.get()
        # check extending
        available_extensions = set(label.alternative_extensions)
        for node_j in available_extensions:
            label.alternative_extensions.remove(node_j)
            if not label.allow_extend(node_j):
                continue

            label_j = label.extend(node_j, self.duals, farkas)
            # check dominance
            dominated_by_j, other_dominates_j = (
                self.dominance_check(label_j, self.forward_labels[node_j], farkas))
            self.forward_disposed_labels.update(dominated_by_j)
            # remove the labels dominated by j
            with self.parallel_lock:
                self.forward_labels[node_j].difference_update(dominated_by_j)
            # if j is not dominated by others
            if not other_dominates_j:
                is_new_path = True if label_j not in self.forward_labels[node_j] else False
                if not is_new_path:
                    continue
                # is a new path
                with self.parallel_lock:
                    self.forward_labels[node_j].add(label_j)
                # only if this label are possible to be extended
                if len(label_j.alternative_extensions) > 0:
                    self.forward_label_queue.put((-label_j.depth, label_j))
                # check whether it is completed, update the incumbent
                if node_j == self.net.depot_sink:
                    hashable_path = truck_drone_path_to_hashable(label_j.truck_path, label_j.drone_flights)
                    # do not consider the existing columns
                    if hashable_path not in node_info.columns:
                        if label_j.cost < self.best_solution[0]:
                            self.best_solution = (label_j.cost, hashable_path, 1)
                        if hashable_path not in self.explored_solutions:
                            self.explored_solutions.add(hashable_path)
                # only append the labels that have not been added
                new_labels[node_j].append(label_j)

        return new_labels

    def backward_labeling_one_step(self, farkas, node_info):
        """Backward search from the sink."""
        new_labels = defaultdict(list)

        _, label = self.backward_label_queue.get()
        available_extensions = set(label.alternative_extensions)
        for node_j in available_extensions:
            label.alternative_extensions.remove(node_j)
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

            current_sol = (tuple(label_j.truck_path), label_j.drone_flights)
            if current_sol == test_path:
                sdas = 0

            if is_route_subset(test_path, (label_j.truck_path, label_j.drone_flights)):
                sdas = 0

            # check dominance
            dominated_by_j, other_dominates_j = (
                self.dominance_check(label_j, self.backward_labels[node_j], farkas))
            self.backward_disposed_labels.update(dominated_by_j)
            # remove the labels dominated by j
            with self.parallel_lock:
                self.backward_labels[node_j].difference_update(dominated_by_j)
            # if j is not dominated by others
            if not other_dominates_j:
                is_new_path = True if label_j not in self.backward_labels[node_j] else False
                if not is_new_path:
                    continue
                # is a new path
                with self.parallel_lock:
                    self.backward_labels[node_j].add(label_j)
                # only if this label are possibly to be extended
                if len(label_j.alternative_extensions) > 0:
                    self.backward_label_queue.put((-label_j.depth, label_j))
                # check whether it is completed, update incumbent
                if node_j == self.net.depot_source:
                    hashable_path = truck_drone_path_to_hashable(label_j.truck_path, label_j.drone_flights)
                    # do not consider the existing columns
                    if hashable_path not in node_info.columns:
                        if label_j.cost < self.best_solution[0]:
                            self.best_solution = (label_j.cost, hashable_path, 2)
                        if hashable_path not in self.explored_solutions:
                            self.explored_solutions.add(hashable_path)
                # only append the labels that have not been added
                new_labels[node_j].append(label_j)

        return new_labels

    def merge_labels(self, node, new_labels, farkas, node_info, mode):
        """Merge forward and backward labels at common nodes."""
        with self.parallel_lock:
            if mode == 0:
                label_pairs = [(f_label, b_label) for f_label in new_labels for b_label in
                               list(self.backward_labels[node])]
            else:
                label_pairs = [(f_label, b_label) for f_label in list(self.forward_labels[node]) for b_label in
                               new_labels]

        for f_label, b_label in label_pairs:
            if not self.check_merge_feasibility(f_label, b_label, f_label.path[-1]):
                continue

            complete_path = f_label.path + b_label.path[1:]

            # Avoid duplicate computations
            complete_path_tuple = tuple(complete_path)
            if complete_path_tuple in self.explored_solutions:
                continue

            # Find last hub in the forward path
            last_hub = next(
                (node.replace("_prime", "") for node in reversed(f_label.path) if node in self.net.hubs),
                None
            )

            # Compute total cost
            if not farkas:  # Normal pricing
                total_cost, _ = cal_label_cost_normal(
                    self.net, f_label.arrival_time, f_label.sync_time,
                    f_label.wait_time, f_label.cost, b_label.path, self.duals, last_hub
                )
            else:  # Farkas pricing
                total_cost = f_label.cost + cal_label_cost_farkas(self.net, b_label.path[1:], self.duals)

            # Update solution count
            GeneralHelper.label_merge_num += 1

            # Update incumbent solution
            route_key = "-".join(complete_path)
            if route_key not in node_info.columns:  # Ignore existing columns
                if total_cost < self.best_solution[0]:
                    self.best_solution = (total_cost, complete_path, 0)

                # Store newly explored solution
                self.explored_solutions.add(complete_path_tuple)

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
            self.forward_label_queue.put((
                0,
                LabelForward([self.net.depot_source], 0, 0, 0, 0,
                             0, -self.duals["nu"], 0, {}, [self.net.depot_source])
            ))
        else:  # Farkas pricing
            self.forward_label_queue.put((
                0,
                LabelForward([self.net.depot_source], 0, 0, 0, 0,
                             0, 0, 0, {}, [self.net.depot_source])
            ))

        # backward label initialization
        self.backward_label_queue.put((
            0,
            LabelBackward([self.net.depot_sink], 0, 0, [0],
                          [self.net.max_timespan], 0, 0, {},
                          [self.net.depot_sink], set())
        ))

        s_time = time.time()

        # parallel mode
        if LSA_mode == 0:
            with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
                # Submit the tasks
                future_forward = executor.submit(self.forward_thread, farkas, node_info)
                future_backward = executor.submit(self.backward_thread, farkas, node_info)

                # Wait for either thread to finish
                done, _ = concurrent.futures.wait(
                    [future_forward, future_backward], return_when=concurrent.futures.FIRST_COMPLETED
                )

                # Stop the other thread as soon as one is done
                self.termination_event.set()

                # Ensure both threads terminate
                future_forward.result()
                future_backward.result()

                self.print_runtime_info(s_time)

                return self.best_solution

        while True:
            # forward only
            if LSA_mode == 1:
                while self.best_solution[0] + close_tolerance >= 0 and self.forward_label_queue.qsize() > 0:
                    self.forward_labeling_one_step(farkas, node_info)
                else:
                    self.print_runtime_info(s_time)
                    return self.best_solution
            # backward only
            elif LSA_mode == 2:
                while self.best_solution[0] + close_tolerance >= 0 and self.backward_label_queue.qsize() > 0:
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
        obj, path, where = self.best_solution

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
        # print(f"backward cost dominance check passed: {GeneralHelper.backward_cost_dominance_num}")
        # print(f"backward arrival dominance check passed: {GeneralHelper.backward_arrival_dominance_num}")
        # print(f"label merge found unexplored solutions: {GeneralHelper.label_merge_num}")
        # print(f"best solution: {self.best_solution[0]:.2f}, {self.best_solution[1]}")
        # print(f"runtime: {time.time() - start_time:.4f}s")
        # print()

    def forward_thread(self, farkas, node_info):
        """
        A separate thread for solving the forward labeling
        """
        while (not self.termination_event.is_set() and
               self.best_solution[0] + close_tolerance >= 0 and self.forward_label_queue.qsize() > 0):

            new_f_labels = self.forward_labeling_one_step(farkas, node_info)
            for node, labels in new_f_labels.items():
                self.merge_labels(node, labels, farkas, node_info, 0)

        # Signal the other thread to stop
        self.termination_event.set()

        with self.print_lock:
            sdas = 0
            # print(f"****** forward terminate")
            # print(f"forward queue: {self.forward_label_queue.qsize()}")
            # print(f"backward queue: {self.backward_label_queue.qsize()}")
            # print()
        return None

    def backward_thread(self, farkas, node_info):
        """
        A separate thread for solving the backward labeling
        """
        while (not self.termination_event.is_set() and
               self.best_solution[0] + close_tolerance >= 0 and self.backward_label_queue.qsize() > 0):
            new_b_labels = self.backward_labeling_one_step(farkas, node_info)
            for node, labels in new_b_labels.items():
                self.merge_labels(node, labels, farkas, node_info, 1)

        # Signal the other thread to stop
        self.termination_event.set()

        with self.print_lock:
            ssdsa = 0
            # print(f"****** backward terminate")
            # print(f"forward queue: {self.forward_label_queue.qsize()}")
            # print(f"backward queue: {self.backward_label_queue.qsize()}")
            # print()
        return None
