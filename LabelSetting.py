import time

import numpy as np
import queue
import GeneralHelper
from GeneralHelper import *
from LabelForward import LabelForward
import itertools
from collections import OrderedDict, defaultdict
from NodeInfo import NodeInfo
import cProfile, pstats, io


class LabelSetting:
    def __init__(self, duals):
        self.net = GeneralHelper.transformed_net
        # the ordered dict is used to ensure the exact visit sequence of the dict (code reproduction)
        self.forward_labels = {node: OrderedDict() for node in self.net.all_nodes}  # save forward label keys
        # cost, elem path
        self.best_solution = (np.inf, [])
        self.duals = duals
        self.forward_label_queue = queue.PriorityQueue()  # priority queue, ordered by depth
        self.forward_label_counter = itertools.count()
        self.N_hat = set()  # the set for DSS

    def forward_labeling_one_step(self, farkas, node_info: NodeInfo):
        """Forward search from the depot."""
        new_labels = {}  # the new labels awaiting to be appended, key: node, value: new_label

        _, _, label = self.forward_label_queue.get()

        # check extending
        # print(
        #     f"current path: {label.path}, feasible extension: {list(available_extensions)}, "
        #     f"queue size: {self.forward_label_queue.qsize()}")

        for node_j in label.alternative_extensions:

            if node_info.id == 1 and len(node_info.columns) == 88:
                if label.path == ['Source'] and node_j == 'C13':
                    sdsa = 0
                if label.path == ['Source', 'C13'] and node_j == 'H1':
                    sdsa = 0
                if label.path == ['Source', 'C13', 'H1'] and node_j == 'C8_T':
                    sdsa = 0
                if label.path == ['Source', 'C13', 'H1', 'C8_T'] and node_j == 'C9_T':
                    sdsa = 0
                if label.path == ['Source', 'C13', 'H1', 'C8_T', 'C9_T'] and node_j == 'C15_T':
                    sdsa = 0
                if label.path == ['Source', 'C13', 'H1', 'C8_T', 'C9_T', 'C15_T'] and node_j == 'C4':
                    sdsa = 0
                if label.path == ['Source', 'C13', 'H1', 'C8_T', 'C9_T', 'C15_T', 'C4'] and node_j == 'H2':
                    sdsa = 0
                if label.path == ['Source', 'C13', 'H1', 'C8_T', 'C9_T', 'C15_T', 'C4', 'H2'] and node_j == 'Sink':
                    sdsa = 0

            if not label.allow_extend(node_j, node_info, self.N_hat):
                continue

            label_j = label.extend(node_j, self.duals, farkas, node_info)

            if node_info.id == 1:
                if label_j.path == ['Source', 'C13', 'H1', 'C8_T', 'C9_T', 'C15_T', 'C4', 'H2',
                                    'Sink'] and label_j.cost + close_tolerance < 0:
                    sada = 0

                test_path = ['Source', 'C13', 'H1', 'C8_T', 'C9_T', 'C15_T', 'C4', 'H2', 'Sink']
                test_route = elem_path_to_route(test_path, 999, GeneralHelper.net, GeneralHelper.transformed_net)
                cost = route_get_cost(test_route, GeneralHelper.net, GeneralHelper.transformed_net)
                reduced_cost = cost + self.duals['constant_term'] - self.duals['mu'][12] - self.duals['mu'][7] - \
                               self.duals['mu'][8] - self.duals['mu'][14] - self.duals['mu'][3]
                if label_j.path == ['Source', 'C13', 'H1', 'C8_T', 'C9_T', 'C15_T', 'C4', 'H2',
                                    'Sink'] and reduced_cost + close_tolerance < 0:
                    dsdsa = 0

            if node_info.id == 3:
                if label_j.path == ['Source', 'C13', 'H1', 'C8_T', 'C9_T', 'C15_T', 'C4', 'H2',
                                    'Sink'] and label_j.cost + close_tolerance < 0:
                    sada = 0

                test_path = ['Source', 'C13', 'H1', 'C8_T', 'C9_T', 'C15_T', 'C4', 'H2', 'Sink']
                test_route = elem_path_to_route(test_path, 999, GeneralHelper.net, GeneralHelper.transformed_net)
                cost = route_get_cost(test_route, GeneralHelper.net, GeneralHelper.transformed_net)
                reduced_cost = cost + self.duals['constant_term'] - self.duals['mu'][12] - self.duals['mu'][7] - \
                               self.duals['mu'][8] - self.duals['mu'][14] - self.duals['mu'][3]
                if label_j.path == ['Source', 'C13', 'H1', 'C8_T', 'C9_T', 'C15_T', 'C4', 'H2',
                                    'Sink'] and reduced_cost + close_tolerance < 0:
                    dsdsa = 0

            # this is an existing column
            if tuple(label_j.path) in GeneralHelper.initial_routes:
                continue

            # check whether it is completed, update the incumbent
            if node_j == self.net.depot_sink:
                if label_j.cost < self.best_solution[0]:
                    if enable_DSS:
                        # check revisit
                        repeated_cus = get_revisit(label_j.path, GeneralHelper.transformed_net)
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
                    if node_info.id == 3 and key == debug_route:
                        sdas = 0
                    self.forward_labels[node_j].pop(key, None)

                # if j is not dominated by others
                if not other_dominates_j:
                    # is a new path
                    new_labels[node_j] = label_j
                    # only if this label is possible to be extended
                    if len(label_j.alternative_extensions) > 0:
                        self.forward_label_queue.put((-label_j.depth, next(self.forward_label_counter), label_j))

        # update the forward_labels at once
        for node, label in new_labels.items():
            self.forward_labels[node][tuple(label.path)] = label

        return new_labels

    def solve(self, farkas, node_info: NodeInfo):
        """
        Execute forward, backward, and merge steps.
        """

        # forward label initialization
        if not farkas:  # normal pricing
            self.forward_label_queue.put((
                0, next(self.forward_label_counter),
                LabelForward(path=[self.net.depot_source],
                             truck_load=0,
                             drones_used=0,
                             arrival_time=0,
                             sync_time=0,
                             wait_time=0,
                             psi_set={pi: 0 for pi in node_info.added_SR_keys},
                             cost=truck_cost + self.duals["constant_term"],
                             depth=0)
            ))
        else:  # Farkas pricing
            self.forward_label_queue.put((
                0, next(self.forward_label_counter),
                LabelForward(path=[self.net.depot_source],
                             truck_load=0,
                             drones_used=0,
                             arrival_time=0,
                             sync_time=0,
                             wait_time=0,
                             psi_set={pi: 0 for pi in node_info.added_SR_keys},
                             cost=self.duals["constant_term"],
                             depth=0)
            ))

        s_time = time.time()
        GeneralHelper.allow_extend_checks_passed = 0
        GeneralHelper.forward_dominance_num = 0

        while True:
            # forward only
            while self.best_solution[0] + close_tolerance > 0 and self.forward_label_queue.qsize() > 0:
                # while self.forward_label_queue.qsize() > 0:
                self.forward_labeling_one_step(farkas, node_info)
            # self.print_runtime_info(s_time, node_info)
            return self.best_solution

    def dominance_check(self, label_j, other_labels, farkas, node_info: NodeInfo):
        """Check if label_j dominates any other label in a parallelized manner."""
        dominated_by_j = {}
        other_dominates_j = False
        other_dom_label = None

        for other in other_labels:
            j_dominates = label_j.dominates(other, node_info, farkas, self.duals)

            if j_dominates:
                dominated_by_j[tuple(other.path)] = other
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

        for j in range(1, len(element_path)):
            node_i, node_j = element_path[j - 1], element_path[j]
            arrive_time = get_arrive_time(arrival_times[-1], node_i, node_j, last_hub, sync_times[-1], wait_times[-1],
                                          self.net)

            sync_time = arrive_time if node_j in self.net.hubs else sync_times[-1]
            wait_time = max(wait_times[-1], arrive_time - sync_time) if ((node_i, node_j) in self.net.arcs_scp
                                                                         or (
                                                                             node_i,
                                                                             node_j) in self.net.arcs_cpcp) else 0
            last_hub = node_j.replace("_T", "") if node_j in self.net.hubs else last_hub

            arrival_times.append(arrive_time)
            sync_times.append(sync_time)
            wait_times.append(wait_time)
            runtime = time.time() - start_time

        print(
            f"node info: {node_info.id}, num cols: {len(node_info.columns)}, runtime: {runtime:.4f}s")
        print()
