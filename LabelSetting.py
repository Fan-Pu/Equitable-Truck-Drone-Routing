import queue
import time

import numpy as np
from sortedcontainers import SortedDict

import CommonHelper
from LabelForward import LabelForward
from NodeInfo import NodeInfo


class LabelSetting:
    def __init__(self, duals, thread_id):
        # self.net = CommonHelper.transformed_net
        # self.ini_routes = CommonHelper.initial_routes
        self.net = CommonHelper.transformed_net
        self.ini_routes = CommonHelper.initial_routes
        # the ordered dict is used to ensure the exact visit sequence of the dict (code reproduction)
        self.forward_labels = {node: SortedDict() for node in self.net.all_nodes}  # save forward label keys
        # self.forward_labels = {node: OrderedDict() for node in self.net.all_nodes}  # save forward label keys
        # cost, elem path
        self.best_solution = (np.inf, [])
        self.duals = duals
        self.forward_label_queue = queue.PriorityQueue()  # priority queue
        self.forward_label_counter = 0
        self.N_hat = set()  # the set for DSS
        self.thread_id = thread_id
        self.print_text = ""

    def __getstate__(self):
        # called when pickling: drop the unpicklable bits
        state = self.__dict__.copy()
        state.pop('forward_label_queue', None)
        return state

    def __setstate__(self, state):
        # called when unpickling: restore everything else…
        self.__dict__.update(state)
        # …then rebuild the queue and counter
        self.forward_label_queue = queue.PriorityQueue()

    def forward_labeling_one_step(self, farkas, node_info: NodeInfo):
        """Forward search from the depot."""
        new_labels = {}  # the new labels awaiting to be appended, key: node, value: new_label

        _, _, _, label = self.forward_label_queue.get()

        # check extending
        # print(
        #     f"current path: {label.path}, feasible extension: {list(available_extensions)}, "
        #     f"queue size: {self.forward_label_queue.qsize()}")

        for node_j in label.alternative_extensions:
            if not label.allow_extend(node_j, node_info, self.N_hat):
                continue

            label_j = label.extend(node_j, self.duals, farkas, node_info)

            # this is an existing column
            if tuple(label_j.path) in self.ini_routes:
                continue

            # check whether it is completed, update the incumbent
            if node_j == self.net.depot_sink:
                if label_j.cost < self.best_solution[0]:
                    if CommonHelper.enable_DSS:
                        # check revisit
                        repeated_cus = CommonHelper.get_revisit(label_j.path, self.net)
                        if len(repeated_cus) > 0:
                            for cus in repeated_cus:
                                self.N_hat.update((cus, cus + "_T"))
                        else:  # no revisit, update the best solution
                            self.best_solution = (label_j.cost, label_j.path)
                    else:
                        self.best_solution = (label_j.cost, label_j.path)
            else:
                # check dominance
                dominated_by_j, other_dominates_j, other_dom_label = (
                    self.dominance_check(label_j, self.forward_labels[node_j].values(), farkas, node_info))

                # dominated_by_j = None
                # other_dominates_j = False
                # other_dom_label = None

                for key in dominated_by_j.keys():
                    self.forward_labels[node_j].pop(key)

                # if j is not dominated by others
                if not other_dominates_j:
                    # is a new path
                    new_labels[node_j] = label_j
                    # only if this label is possible to be extended
                    if len(label_j.alternative_extensions) > 0:
                        self.forward_label_counter += 1
                        self.forward_label_queue.put(
                            (-label_j.depth, label_j.cost, self.forward_label_counter, label_j))

        # update the forward_labels at once
        for node, label in new_labels.items():
            if len(self.forward_labels[node]) >= CommonHelper.max_node_label_num:
                (worst_val, worst_path), temp_label = self.forward_labels[node].peekitem(-1)
                if label.cost + CommonHelper.close_tolerance > worst_val:
                    continue
                # pops out the worst element
                self.forward_labels[node].pop((worst_val, worst_path))
                self.forward_labels[node][(label.cost, tuple(label.path))] = label
            else:
                self.forward_labels[node][(label.cost, tuple(label.path))] = label

        return new_labels

    def solve(self, farkas, node_info: NodeInfo, stop_event):
        """
        Execute forward, backward, and merge steps.
        """

        # forward label initialization
        if not farkas:  # normal pricing
            initial_cost = CommonHelper.truck_cost + self.duals["constant_term"]
            self.forward_label_queue.put((
                0, initial_cost, self.forward_label_counter,
                LabelForward(path=[self.net.depot_source],
                             truck_load=0,
                             drones_used=0,
                             arrival_time=0,
                             sync_time=0,
                             wait_time=0,
                             psi_set={pi: 0 for pi in node_info.added_SR_keys},
                             cost=initial_cost,
                             depth=0,
                             alternative_extensions=self.net.start_node_dict[self.thread_id],
                             trans_net=self.net)
            ))
        else:  # Farkas pricing
            initial_cost = self.duals["constant_term"]
            self.forward_label_queue.put((
                0, initial_cost, self.forward_label_counter,
                LabelForward(path=[self.net.depot_source],
                             truck_load=0,
                             drones_used=0,
                             arrival_time=0,
                             sync_time=0,
                             wait_time=0,
                             psi_set={pi: 0 for pi in node_info.added_SR_keys},
                             cost=initial_cost,
                             depth=0,
                             alternative_extensions=self.net.start_node_dict[self.thread_id],
                             trans_net=self.net)
            ))

        s_time = time.time()

        # forward only
        while self.best_solution[0] + CommonHelper.close_tolerance > 0 and self.forward_label_queue.qsize() > 0:
            if time.time() - s_time >= CommonHelper.max_run_time or stop_event.is_set():
                break
            self.forward_labeling_one_step(farkas, node_info)

        # self.print_runtime_info(s_time, node_info)
        return self.print_text, self.best_solution

    def dominance_check(self, label_j, other_labels, farkas, node_info: NodeInfo):
        """Check if label_j dominates any other label in a parallelized manner."""
        dominated_by_j = {}  # key: (cost,path)
        other_dominates_j = False
        other_dom_label = None

        for other in other_labels:
            j_dominates = label_j.dominates(other, node_info, farkas, self.duals)

            if j_dominates:
                dominated_by_j[(other.cost, tuple(other.path))] = other
            else:
                other_dominates = other.dominates(label_j, node_info, farkas, self.duals)

                if other_dominates:
                    other_dominates_j = True
                    other_dom_label = other
                    break  # Stop checking if label_j is already dominated

        return dominated_by_j, other_dominates_j, other_dom_label

    def print_runtime_info(self, start_time, node_info: NodeInfo):
        arrival_times, sync_times, wait_times = [0], [0], [0]
        last_hub = None
        obj, element_path = self.best_solution

        if element_path is None:
            return

        # for j in range(1, len(element_path)):
        #     node_i, node_j = element_path[j - 1], element_path[j]
        #     arrive_time = get_arrive_time(arrival_times[-1], node_i, node_j, last_hub, sync_times[-1], wait_times[-1],
        #                                   self.net)
        #
        #     sync_time = arrive_time if node_j in self.net.hubs else sync_times[-1]
        #     wait_time = max(wait_times[-1], arrive_time - sync_time) if ((node_i, node_j) in self.net.arcs_scp
        #                                                                  or (
        #                                                                      node_i,
        #                                                                      node_j) in self.net.arcs_cpcp) else 0
        #     last_hub = node_j.replace("_T", "") if node_j in self.net.hubs else last_hub
        #
        #     arrival_times.append(arrive_time)
        #     sync_times.append(sync_time)
        #     wait_times.append(wait_time)

        runtime = time.time() - start_time

        self.print_text = (f"node info: {node_info.id}, thread: {self.thread_id}, num cols: {len(node_info.columns)},"
                           f" obj:{obj:.4f}, ") + f"runtime: {runtime:.4f}s"

        sdas = 0

        # print(
        #     f"node info: {node_info.id}, thread: {self.thread_id}, num cols: {len(node_info.columns)}, obj:{obj:.4f}, "
        #     f"runtime: {runtime:.4f}s")
        # print()
