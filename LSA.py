import concurrent.futures
import threading
import time
import numpy as np
import queue
import GeneralHelper
from GeneralHelper import *
from LabelForward import LabelForward
from LabelBackward import LabelBackward
import itertools
from collections import OrderedDict, defaultdict
from NodeInfo import NodeInfo


class BiDirectionalLabelSetting:
    def __init__(self, duals):
        random.seed(seed)
        self.net = GeneralHelper.transformed_net
        # the ordered dict is used to ensure the exact visit sequence of the dict (code reproduction)
        self.forward_labels = {node: OrderedDict() for node in self.net.all_nodes}  # save forward label keys
        self.backward_labels = {node: OrderedDict() for node in self.net.all_nodes}  # save backward label keys
        # store the labels which will non-empty pending flights. Pending_flights + hash_path can be used to check "="
        self.backward_pending_labels = defaultdict(OrderedDict)  # key: pending flights, values: labels

        # self.forward_label_keys = {node: OrderedDict() for node in self.net.all_nodes}  # save forward label keys
        # self.backward_label_keys = {node: OrderedDict() for node in self.net.all_nodes}  # save backward label keys
        # self.forward_label_dict = defaultdict()  # save all forward labels
        # self.backward_label_dict = defaultdict()  # save all backward labels

        # the third element: 0 from merge, 1 from forward, 2 from backward
        self.best_solution = (np.inf, None, None, -1)
        self.explored_solutions = SortedSet()  # explored labels
        self.duals = duals
        self.forward_label_queue = queue.PriorityQueue()  # priority queue, ordered by depth
        self.backward_label_queue = queue.PriorityQueue()
        self.forward_label_counter = itertools.count()
        self.backward_label_counter = itertools.count()
        self.forward_disposed_labels = SortedSet()  # labels dominated by other labels
        self.backward_disposed_labels = SortedSet()
        # self.node_locks = defaultdict(threading.Lock)
        # self.best_sol_lock = threading.Lock()
        self.print_lock = threading.Lock()
        self.termination_event = threading.Event()  # Shared flag for stopping threads
        self.forward_label_locks = defaultdict(threading.Lock)
        self.backward_label_locks = defaultdict(threading.Lock)
        self.backward_pending_label_lock = threading.Lock()
        self.forward_update_cond = threading.Condition()  # the condition signaling the update of forward labels
        self.backward_update_cond = threading.Condition()  # the condition signaling the update of backward labels

    def forward_labeling_one_step(self, farkas, node_info: NodeInfo):
        """Forward search from the depot."""
        new_labels = defaultdict(list)
        awaiting_labels = defaultdict(dict)  # the new labels awaiting to be appended, key: node, value: dict

        _, _, label = self.forward_label_queue.get()
        # check extending
        available_extensions = SortedSet(label.alternative_extensions)
        for node_j in available_extensions:
            label.alternative_extensions.remove(node_j)

            if not label.allow_extend(node_j, node_info):
                continue

            label_j = label.extend(node_j, self.duals, farkas)

            # check dominance
            dominated_by_j, other_dominates_j, other_dom_label = (
                self.dominance_check(label_j, self.forward_labels[node_j].values(), farkas))
            # remove the labels dominated by j
            # with self.forward_label_locks[node_j]:
            #     self.forward_labels[node_j] -= dominated_by_j
            for key in dominated_by_j.keys():
                self.forward_labels[node_j].pop(key, None)
            # self.forward_labels[node_j].difference_update(dominated_by_j)
            # with self.parallel_lock:
            #     self.forward_labels[node_j].difference_update(dominated_by_j)

            # if j is not dominated by others
            if not other_dominates_j:
                is_new_path = True if tuple(label_j.path) not in self.forward_labels[node_j].keys() else False
                if not is_new_path:
                    continue
                # is a new path
                awaiting_labels[node_j][tuple(label_j.path)] = label_j
                # only if this label are possible to be extended
                if len(label_j.alternative_extensions) > 0:
                    self.forward_label_queue.put((-label_j.depth, next(self.forward_label_counter), label_j))
                # check whether it is completed, update the incumbent
                if node_j == self.net.depot_sink:
                    hashable_path = label_j.hash_path
                    # do not consider the existing columns
                    if hashable_path not in node_info.columns:
                        # with self.best_sol_lock:
                        #     if label_j.cost < self.best_solution[0]:
                        #         self.best_solution = (label_j.cost, hashable_path, 1)
                        if label_j.cost < self.best_solution[0]:
                            self.best_solution = (label_j.cost, hashable_path, label_j.path, 1)
                        if hashable_path not in self.explored_solutions:
                            self.explored_solutions.add(hashable_path)
                # only append the labels that have not been added
                new_labels[node_j].append(label_j)
        # update the forward_labels at once
        for node, labels in awaiting_labels.items():
            with self.forward_label_locks[node]:
                self.forward_labels[node].update(labels)

        if len(awaiting_labels) > 0:
            with self.forward_update_cond:
                self.forward_update_cond.notify_all()

        return new_labels

    def backward_labeling_one_step(self, farkas, node_info):
        """Backward search from the sink."""
        new_labels = defaultdict(list)
        awaiting_labels = defaultdict(dict)  # the new labels awaiting to be appended
        awaiting_labels_non_empty_pending = defaultdict(
            dict)  # same as awaiting_labels but for labels with non-empty pending flights

        _, _, label = self.backward_label_queue.get()
        available_extensions = SortedSet(label.alternative_extensions)

        for node_j in available_extensions:
            label.alternative_extensions.remove(node_j)

            if not label.allow_extend(node_j, node_info):
                continue

            label_j = label.extend(node_j, self.duals, farkas)

            if node_j == self.net.depot_source:  # once it is completed, we need to calculate the cost
                # normal mode
                if not farkas:
                    label_j.cost, arrive_times = cal_label_cost_normal(self.net, 0, 0, 0,
                                                                       -self.duals["constant_term"], label_j.path,
                                                                       self.duals)
                else:  # Farkas pricing
                    label_j.cost = cal_label_cost_farkas(self.net, label_j.path, self.duals)

            # check dominance
            dominated_by_j, other_dominates_j, other_dom_label = (
                self.dominance_check(label_j, self.backward_labels[node_j].values(), farkas))
            self.backward_disposed_labels.update(dominated_by_j)
            # remove the labels dominated by j
            # with self.parallel_lock:
            for key in dominated_by_j.keys():
                self.backward_labels[node_j].pop(key, None)
            # if j is not dominated by others
            if not other_dominates_j:
                # check whether it is a new path
                is_new_path = self.check_back_label_existence(label_j)
                # skip the existing path
                if not is_new_path:
                    continue
                # is a new path
                if len(label_j.pending_flights) == 0:
                    awaiting_labels[node_j][label_j.hash_path] = label_j
                else:
                    awaiting_labels_non_empty_pending[tuple(label_j.pending_flights)][label_j.hash_path] = label_j
                # only if this label is possibly to be extended
                if len(label_j.alternative_extensions) > 0:
                    self.backward_label_queue.put((-label_j.depth, next(self.backward_label_counter), label_j))
                # check whether it is completed, update incumbent
                if node_j == self.net.depot_source:
                    hashable_path = label_j.hash_path
                    # do not consider the existing columns
                    if hashable_path not in node_info.columns:
                        if label_j.cost < self.best_solution[0]:
                            self.best_solution = (label_j.cost, hashable_path, label_j.path, 2)
                        if hashable_path not in self.explored_solutions:
                            self.explored_solutions.add(hashable_path)
                # only append the labels that have not been added
                new_labels[node_j].append(label_j)

        # update the backward_labels at once
        for node, labels in awaiting_labels.items():
            with self.backward_label_locks[node]:
                self.backward_labels[node].update(labels)
        with self.backward_pending_label_lock:
            for pending_flights, labels in awaiting_labels_non_empty_pending.items():
                self.backward_pending_labels[tuple(pending_flights)].update(labels)
        if len(awaiting_labels) > 0 or len(awaiting_labels_non_empty_pending) > 0:
            with self.backward_update_cond:
                self.backward_update_cond.notify_all()

        return new_labels

    def merge_labels(self, new_labels, farkas, node_info, mode, opposite_labels):
        """
        Merge forward and backward labels at common nodes.
        mode: 0 for merging new forward labels with existing backward labels, 1 otherwise
        """

        # Determine the label sources based on mode

        if mode == 0:
            label_pairs = [(f_label, b_label) for f_label in new_labels for b_label in opposite_labels]
        else:
            label_pairs = [(f_label, b_label) for f_label in opposite_labels for b_label in new_labels]

        for f_label, b_label in label_pairs:
            if not self.check_merge_feasibility(f_label, b_label, f_label.path[-1]):
                continue

            complete_path = f_label.path + b_label.path[1:]
            if len(b_label.pending_flights) > 0:
                complete_truck_path = f_label.truck_path + b_label.truck_path
            else:
                complete_truck_path = f_label.truck_path + b_label.truck_path[1:]

            complete_drone_flights = {
                k: f_label.drone_flights.get(k, SortedSet()).union(b_label.drone_flights.get(k, SortedSet()))
                for k in f_label.drone_flights.keys() | b_label.drone_flights.keys()}
            if len(b_label.pending_flights) > 0:
                complete_drone_flights[f_label.latest_hub].update(b_label.pending_flights)

            hashable_path = truck_drone_path_to_hashable(complete_truck_path, complete_drone_flights)

            # Avoid duplicate computations
            if hashable_path in self.explored_solutions or hashable_path in node_info.columns:
                continue

            # Find last hub in the forward path
            last_hub = f_label.latest_hub

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
            # with self.best_sol_lock:
            #     if total_cost < self.best_solution[0]:
            #         self.best_solution = (total_cost, hashable_path, 0)
            if total_cost < self.best_solution[0]:
                self.best_solution = (total_cost, hashable_path, 0)

            # Store newly explored solution
            self.explored_solutions.add(hashable_path)

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
                0, next(self.forward_label_counter),
                LabelForward([self.net.depot_source], 0, 0, 0, 0,
                             0, self.duals["constant_term"], 0, {}, [self.net.depot_source])
            ))
        else:  # Farkas pricing
            self.forward_label_queue.put((
                0, next(self.forward_label_counter),
                LabelForward([self.net.depot_source], 0, 0, 0, 0,
                             0, 0, 0, {}, [self.net.depot_source])
            ))

        # backward label initialization
        self.backward_label_queue.put((
            0, next(self.backward_label_counter),
            LabelBackward([self.net.depot_sink], 0, 0, [0],
                          [self.net.max_timespan], 0, 0, {},
                          [self.net.depot_sink], SortedSet())
        ))

        s_time = time.time()

        # parallel mode
        if LSA_mode == 0:
            with concurrent.futures.ThreadPoolExecutor(max_workers=3) as executor:
                # # Submit the tasks
                # futures = [
                #     executor.submit(self.forward_thread, farkas, node_info),
                #     executor.submit(self.backward_thread, farkas, node_info),
                #     executor.submit(self.label_merge_thread, farkas, node_info)
                # ]
                #
                # # Ensure all threads complete before proceeding
                # concurrent.futures.wait(futures)
                #
                # # 🚀 Set termination flag to stop all waiting threads
                # self.termination_event.set()
                #
                # # 🚀 Ensure no thread is stuck waiting on conditions
                # with self.forward_update_cond:
                #     self.forward_update_cond.notify_all()
                #
                # with self.backward_update_cond:
                #     self.backward_update_cond.notify_all()
                #
                # # 🚀 Ensure all threads exit cleanly before continuing
                # for future in futures:
                #     future.result()  # Handle any exceptions properly

                # Submit the tasks
                future_forward = executor.submit(self.forward_thread, farkas, node_info)
                future_backward = executor.submit(self.backward_thread, farkas, node_info)
                future_merge = executor.submit(self.label_merge_thread, farkas, node_info)

                # if len(node_info.columns) > 100:
                #     lp.add_function(concurrent.futures.wait)
                #     lp.enable()

                for future in concurrent.futures.as_completed([future_forward, future_backward, future_merge]):
                    future.result()  # This prevents exceptions from being swallowed

                self.termination_event.set()

                # if len(node_info.columns) > 100:
                #     lp.disable()
                #     lp.print_stats()
                #     input("")

                self.print_runtime_info(s_time)

                return self.best_solution

        while True:
            # forward only
            if LSA_mode == 1:
                while self.best_solution[0] + close_tolerance > 0 and self.forward_label_queue.qsize() > 0:
                    self.forward_labeling_one_step(farkas, node_info)
                else:
                    self.print_runtime_info(s_time)
                    return self.best_solution
            # backward only
            elif LSA_mode == 2:
                while self.best_solution[0] + close_tolerance > 0 and self.backward_label_queue.qsize() > 0:
                    self.backward_labeling_one_step(farkas, node_info)
                else:
                    self.print_runtime_info(s_time)
                    return self.best_solution

    def dominance_check(self, label_j, other_labels, farkas):
        """Check if label_j dominates any other label in a parallelized manner."""
        dominated_by_j = {}
        other_dominates_j = False
        other_dom_label = None

        for other in other_labels:
            j_dominates = label_j.dominates(other, farkas, self.duals)
            if other.hash_path == test_dom_path:
                sdas = 0
            other_dominates = other.dominates(label_j, farkas, self.duals)

            if j_dominates:
                dominated_by_j[tuple(other.path)] = other
            if other_dominates:
                other_dominates_j = True
                other_dom_label = other
                break  # Stop checking if label_j is already dominated

        return dominated_by_j, other_dominates_j, other_dom_label

    def print_runtime_info(self, start_time):
        arrival_times, sync_times, wait_times = [0], [0], [0]
        last_hub = None
        obj, path, element_path, where = self.best_solution

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
        while True:
            # termination condition 1
            if not self.termination_event.is_set():
                break
            # termination condition 2
            # with self.best_sol_lock:
            #     if self.best_solution[0] + close_tolerance < 0:
            #         break
            if self.best_solution[0] + close_tolerance < 0:
                break
            # termination condition 3
            if self.forward_label_queue.qsize() == 0:
                break

            new_f_labels = self.forward_labeling_one_step(farkas, node_info)

        # Signal the other thread to stop
        self.termination_event.set()

        # with self.print_lock:
        #     sdas = 0
        #     print(f"****** forward terminate")
        #     print(f"forward queue: {self.forward_label_queue.qsize()}")
        #     print(f"backward queue: {self.backward_label_queue.qsize()}")
        #     print()
        return None

    def backward_thread(self, farkas, node_info):
        """
        A separate thread for solving the backward labeling
        """
        while True:
            # termination condition 1
            if not self.termination_event.is_set():
                break
            # termination condition 2
            # with self.best_sol_lock:
            #     if self.best_solution[0] + close_tolerance < 0:
            #         break
            if self.best_solution[0] + close_tolerance < 0:
                break
            # termination condition 3
            if self.backward_label_queue.qsize() == 0:
                break

            new_b_labels = self.backward_labeling_one_step(farkas, node_info)

        # Signal the other thread to stop
        self.termination_event.set()

        # with self.print_lock:
        #     ssdsa = 0
        #     print(f"****** backward terminate")
        #     print(f"forward queue: {self.forward_label_queue.qsize()}")
        #     print(f"backward queue: {self.backward_label_queue.qsize()}")
        #     print()
        return None

    def label_merge_thread(self, farkas, node_info):
        forward_seen_labels = defaultdict(set)
        backward_seen_labels = defaultdict(set)

        while not self.termination_event.is_set():
            # wait for forward updates
            with self.forward_update_cond:
                if self.forward_update_cond.wait(timeout=1) or self.termination_event.is_set():
                    break

                if self.termination_event.is_set():
                    print("[DEBUG] Termination event set. Exiting label_merge_thread.")
                    return

                keys_in_forward_labels = self.forward_labels.keys()
                for node in keys_in_forward_labels:
                    new_forward_labels = self.forward_labels[node] - forward_seen_labels[node]
                    self.merge_labels(new_forward_labels, farkas, node_info, 0, backward_seen_labels[node])
                    forward_seen_labels[node].update(new_forward_labels)

            # wait for backward updates
            with self.backward_update_cond:
                while not self.termination_event.is_set():
                    notified = self.backward_update_cond.wait(timeout=1)
                    if notified or self.termination_event.is_set():
                        break

                if self.termination_event.is_set():
                    print("[DEBUG] Termination event set. Exiting label_merge_thread.")
                    return  # ✅ Exit safely

                keys_in_backward_labels = self.backward_labels.keys()
                for node in keys_in_backward_labels:
                    new_backward_labels = self.backward_labels[node] - backward_seen_labels[node]
                    self.merge_labels(new_backward_labels, farkas, node_info, 1, forward_seen_labels[node])
                    backward_seen_labels[node].update(new_backward_labels)

        sdsa = 0

    def check_back_label_existence(self, label):
        """
        check whether it is a new path
        """

        node_j = label.path[0]
        if len(label.pending_flights) == 0:
            # if it has an empty pending_flights, check self.backward_labels[node_j]
            is_new_path = True if label.hash_path not in self.backward_labels[node_j].keys() else False
        else:  # check backward_pending_labels
            is_new_path = True if label.hash_path not in self.backward_pending_labels[
                tuple(label.pending_flights)].keys() else False
        return is_new_path
