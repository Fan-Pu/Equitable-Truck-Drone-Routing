import logging
import time
from queue import PriorityQueue
from colorama import Fore, Style
from BP.rmp_node import RMPNode
from GeneralHelper import *
import GeneralHelper
from NodeInfo import NodeInfo
from gurobipy import GRB

node_id_counter = 0
node_infos = {}
route_dict = {}  # key (truck_route, drone_route)
route_key_id_pairs = {}  # key: route key. value: route id


class BranchAndPrice:
    def __init__(self, tolerance=1e-6, max_runtime=float('inf')):
        self.tolerance = tolerance
        self.max_runtime = max_runtime
        self.branch_queue = PriorityQueue()
        self.global_upper_bound = float('inf')  # the cost of incumbent solution
        self.best_solution_node: RMPNode = None
        # add initial routes
        self.initial_routes = []
        routes, element_paths = find_initial_routes(GeneralHelper.net)
        for i in range(len(routes)):
            route_key, route = routes[i]
            element_path = element_paths[route_key]
            route_dict[route_key] = route
            route_key_id_pairs[route_key] = route['id']
            self.initial_routes.append(route_key)
        self.initial_element_paths = dict(element_paths)

    def solve(self):
        """Main branch-and-price loop"""
        start_time = time.time()
        global node_id_counter
        # add root node
        node_id_counter += 1
        node_id = node_id_counter
        node_infos[node_id] = NodeInfo(-1, node_id)
        node_infos[node_id].columns = self.initial_routes
        node_infos[node_id].column_elementary_paths = self.initial_element_paths

        self.branch_queue.put(node_id)

        # branch
        while not self.branch_queue.empty():
            if time.time() - start_time > self.max_runtime:
                logging.info("Terminated due to reaching maximum runtime.")
                break

            node_id = self.branch_queue.get()
            # solve the current node
            is_integer_sol, branch_candidates, var_vals, lp_iters, lp_obj_val = self.solve_node(node_infos[node_id])
            # if LP_obj is higher that global upper bound
            if is_integer_sol is not None:
                if not is_integer_sol:
                    pruned_by_ub = True if lp_obj_val >= self.global_upper_bound else False
                else:
                    pruned_by_ub = True if lp_obj_val > self.global_upper_bound else False
                if not is_integer_sol and not pruned_by_ub:
                    self.branch(node_id, branch_candidates, var_vals)

                ub_print = "-" if self.global_upper_bound == float('inf') else self.global_upper_bound
                local_lb_print = round(lp_obj_val, 2) if not is_integer_sol else "-"
                log_text = (
                    f"LP iters: {lp_iters}, solved node id: {node_id}, queue size: {self.branch_queue.qsize()}, "
                    f"columns: {len(node_infos[node_id].columns)}, ub: {ub_print}, "
                    f"local lb: {local_lb_print}")
                if is_integer_sol:  # find integer solution
                    log_text += f", integer obj: {round(lp_obj_val, 2)}"
                if pruned_by_ub:
                    log_text += f", pruned by UB"
            else:
                log_text = (
                    f"LP iters: {lp_iters}, solved node id: {node_id}, queue size: {self.branch_queue.qsize()}, "
                    f"infeasible node")

            print(Fore.RED + log_text + Style.RESET_ALL)
            if node_id == 1:
                node_info = node_infos[node_id]
                sdsa = 0

        # terminated
        final_solution, cost = self.construct_final_route()

        return time.time() - start_time, final_solution, cost

    def branch(self, current_node_id, branch_candidates, var_vals: dict):
        global node_id_counter
        current_node: NodeInfo = node_infos[current_node_id]
        # left node
        node_id_counter += 1
        node_left_id = node_id_counter
        node_left = NodeInfo(current_node_id, node_left_id)
        node_infos[node_left_id] = node_left
        node_left.as_child(current_node)
        # right node
        node_id_counter += 1
        node_right_id = node_id_counter
        node_right = NodeInfo(current_node_id, node_right_id)
        node_infos[node_right_id] = node_right
        node_right.as_child(current_node)

        current_node.child_ids = [node_left_id, node_right_id]

        ori_net = GeneralHelper.net
        trans_net = GeneralHelper.transformed_net
        # this requires to branch on vehicle fleet
        sum_vals = sum(var_vals.values())
        if not is_integer(sum_vals):
            # if False:
            fleet_ub, fleet_lb = int(sum_vals), int(sum_vals) + 1
            # update node info
            if node_left.vehicle_fleet_branch_ub > fleet_ub:
                node_left.vehicle_fleet_branch_ub = fleet_ub
            if node_right.vehicle_fleet_branch_lb < fleet_lb:
                node_right.vehicle_fleet_branch_lb = fleet_lb
        else:
            # exam flows on all arcs, two types of arc can be revisited: (Source, hub) and (hub, Sink)
            truck_arc_flows = {(i, j): 0.0 for i, j in ori_net.truck_arcs if
                               i != ori_net.depot_source and j != ori_net.depot_sink}
            drone_arc_flows = {(i, j): 0.0 for i, j in ori_net.drone_arcs if
                               i != ori_net.depot_source and j != ori_net.depot_sink}
            arc_travel_column_keys = {(i, j): set() for i, j in ori_net.truck_arcs + ori_net.drone_arcs}
            for column_key in current_node.columns:
                flow_num = var_vals[column_key]
                truck_path = list(column_key[0])
                drone_path = column_key[-1]
                for i in range(len(truck_path) - 1):
                    node_i = truck_path[i]
                    node_j = truck_path[i + 1]
                    # if (node_i == ori_net.depot_source and node_j in ori_net.hubs) or (
                    #         node_i in ori_net.hubs and node_j == ori_net.depot_sink):
                    #     continue
                    arc = (node_i, node_j)  # truck arc
                    if arc in truck_arc_flows.keys():
                        truck_arc_flows[arc] += flow_num
                        arc_travel_column_keys[arc].add(column_key)
                    # also checks the drone paths
                    if node_i in ori_net.hubs:
                        for launch_hub, visits in drone_path:
                            if launch_hub == node_i:
                                for visit in visits:
                                    drone_arc = (launch_hub, visit.replace("_prime", ""))
                                    if drone_arc in drone_arc_flows.keys():
                                        drone_arc_flows[drone_arc] += flow_num
                                        arc_travel_column_keys[drone_arc].add(column_key)
                                break
            # sorted in descending order of the distance between the fractional flow to 0.5
            truck_binary_flows = [(k, v) for k, v in truck_arc_flows.items() if 0 < v < 1 and not is_integer(v)]
            drone_binary_flows = [(k, v) for k, v in drone_arc_flows.items() if 0 < v < 1 and not is_integer(v)]
            # branch on the binary flow (original network)
            arc = flow = where = None
            # we prioritize branching on drone flows
            if len(drone_binary_flows) > 0:
                arc, flow = min(drone_binary_flows, key=lambda x: abs(x[1] - 0.5))
                where = 'drone'
            elif len(truck_binary_flows) > 0:
                arc, flow = min(truck_binary_flows, key=lambda x: abs(x[1] - 0.5))
                where = 'truck'
            if arc is not None and flow is not None:
                node_i, node_j = arc

                new_disabled_arcs_left = set()
                new_disabled_arcs_right = set()

                # down branch (left)
                if where == 'drone':
                    node_left.disabled_arcs_drones_left.add(arc)
                else:
                    node_left.disabled_arcs.add(arc)
                new_disabled_arcs_left.add(arc)

                # up branch (right) must travel arc
                node_right.must_visit_arcs.add(arc)
                if where == 'truck':
                    for node_next in ori_net.truck_out_arcs[node_i]:
                        if node_next != node_j:
                            node_right.disabled_arcs.add((node_i, node_next))
                            new_disabled_arcs_right.add((node_i, node_next))
                else:
                    for node_pre in ori_net.truck_in_arcs[node_j]:
                        # disable the entrance from any other hub except node_i
                        if node_pre != node_i and node_pre in ori_net.hubs:
                            node_right.disabled_arcs.add((node_pre, node_j))
                            new_disabled_arcs_right.add((node_pre, node_j))
                    for node_next in ori_net.truck_out_arcs[node_i]:
                        # must immediately visit node_j if node_i is visited
                        if node_next != node_j:
                            node_right.disabled_arcs.add((node_i, node_next))
                            new_disabled_arcs_right.add((node_i, node_next))

                # remove columns
                remove_col_keys_left = set()
                remove_col_keys_right = set()
                for column_key in current_node.columns:
                    truck_path = list(column_key[0])
                    drone_path = column_key[-1]
                    left_break = right_break = False
                    for i in range(len(truck_path) - 1):
                        if left_break and right_break:
                            break
                        node_i = truck_path[i]
                        node_j = truck_path[i + 1]
                        temp_arc = (node_i, node_j)  # truck arc
                        if temp_arc in new_disabled_arcs_left and not left_break:
                            remove_col_keys_left.add(column_key)
                            left_break = True
                        if temp_arc in new_disabled_arcs_right and not right_break:
                            remove_col_keys_right.add(column_key)
                            right_break = True
                        # also checks the drone paths
                        if node_i not in ori_net.hubs:
                            continue
                        for launch_hub, visits in drone_path:
                            if launch_hub != node_i:
                                continue
                            for visit in visits:
                                if left_break and right_break:
                                    break
                                drone_arc = (launch_hub, visit.replace("_prime", ""))
                                if drone_arc in new_disabled_arcs_left and not left_break:
                                    remove_col_keys_left.add(column_key)
                                    left_break = True
                                if drone_arc in new_disabled_arcs_right and not right_break:
                                    remove_col_keys_right.add(column_key)
                                    right_break = True
                            break

                node_left.removed_columns_keys = remove_col_keys_left
                node_right.removed_columns_keys = remove_col_keys_right
                # remove the columns
                for column_key in remove_col_keys_left:
                    node_left.columns.remove(column_key)
                    del node_left.column_elementary_paths[column_key]
                for column_key in remove_col_keys_right:
                    node_right.columns.remove(column_key)
                    del node_right.column_elementary_paths[column_key]
            # branch on the transformed network
            else:
                # exam flows on all arcs, two types of arc can be revisited: (Source, hub) and (hub, Sink)
                arc_flows = {(i, j): 0.0 for i, j in trans_net.arcs if
                             i != trans_net.depot_source and j != trans_net.depot_sink}
                arc_travel_column_keys = {(i, j): set() for i, j in trans_net.arcs if
                                          i != trans_net.depot_source and j != trans_net.depot_sink}
                for column_key in current_node.columns:
                    flow_num = var_vals[column_key]
                    element_path = current_node.column_elementary_paths[column_key]
                    for i in range(len(element_path) - 1):
                        node_i = element_path[i]
                        node_j = element_path[i + 1]
                        arc = (node_i, node_j)  # truck arc
                        if arc in arc_flows.keys():
                            arc_flows[arc] += flow_num
                            arc_travel_column_keys[arc].add(column_key)
                binary_flows = [(k, v) for k, v in arc_flows.items() if 0 < v < 1 and not is_integer(v)]
                # branch on the binary flow (original network)
                if len(binary_flows) == 0:
                    raise Exception("wrong branching case")
                # branch on the least fractional arc
                arc, flow = min(binary_flows, key=lambda x: abs(x[1] - 0.5))
                node_i, node_j = arc

                new_disabled_arcs_left = set()
                new_disabled_arcs_right = set()

                # down branch (left)
                node_left.disabled_arcs_trans.add(arc)
                new_disabled_arcs_left.add(arc)

                # up branch (right) must travel arc
                node_right.must_visit_arcs_trans.add(arc)
                for node_pre in trans_net.in_arcs[node_j]:
                    if node_pre != node_i:
                        disabled_arc = (node_pre, node_j)
                        node_right.disabled_arcs_trans.add(disabled_arc)
                        new_disabled_arcs_right.add(disabled_arc)
                for node_next in trans_net.out_arcs[node_i]:
                    # must immediately visit node_j if node_i is visited
                    if node_next != node_j:
                        disabled_arc = (node_i, node_next)
                        node_right.disabled_arcs_trans.add(disabled_arc)
                        new_disabled_arcs_right.add(disabled_arc)

                # remove columns
                remove_col_keys_left = set()
                remove_col_keys_right = set()
                for column_key in current_node.columns:
                    element_path = current_node.column_elementary_paths[column_key]
                    left_break = right_break = False
                    for i in range(len(element_path) - 1):
                        if left_break and right_break:
                            break
                        node_i = element_path[i]
                        node_j = element_path[i + 1]
                        temp_arc = (node_i, node_j)  # truck arc
                        if temp_arc in new_disabled_arcs_left and not left_break:
                            remove_col_keys_left.add(column_key)
                            left_break = True
                        if temp_arc in new_disabled_arcs_right and not right_break:
                            remove_col_keys_right.add(column_key)
                            right_break = True

                node_left.removed_columns_keys = remove_col_keys_left
                node_right.removed_columns_keys = remove_col_keys_right
                # remove the columns (fix their ub to 0), this does not introduce new constraints into the node
                for column_key in remove_col_keys_left:
                    node_left.columns.remove(column_key)
                    del node_left.column_elementary_paths[column_key]
                for column_key in remove_col_keys_right:
                    node_right.columns.remove(column_key)
                    del node_right.column_elementary_paths[column_key]
        left_node_include_vector = []
        right_node_include_vector = []
        for path in test_path_list:
            left_node_include_vector.append(path in node_left.columns)
            right_node_include_vector.append(path in node_right.columns)

        self.branch_queue.put(node_left_id)
        self.branch_queue.put(node_right_id)

    def solve_node(self, node_info: NodeInfo):
        """
        iteratively solve the RMP, then solve pricing problem using Benders decomposition (BMP, BSP), until no new
        columns found
        :return: is_integer, branch_candidates, var_vals, lp_iters, lp_obj_val
        """
        rmp_node = RMPNode(node_info)
        if node_info.id == 3:
            sdas = 0
        rmp_node.solve()
        lp_iters = 1

        # column generation
        while True:
            find_new_column = rmp_node.run_pricer(node_info)
            if not find_new_column:
                break
            rmp_node.solve()
            lp_iters += 1

        if node_info.id == 4:
            sdsa = 0
        rmp_node.model.update()
        rmp_node.model.write(f"./BP_nodes/RMP_{node_info.id}.lp")

        # this means the current node is infeasible
        if rmp_node.status == GRB.INFEASIBLE:
            return None, None, None, lp_iters, None

        # check integrality of RMP
        has_integer_solution, branch_candidates = rmp_node.is_integer()
        rmp_obj_val = rmp_node.obj_val
        if has_integer_solution:  # update incumbent
            if rmp_obj_val < self.global_upper_bound:
                self.global_upper_bound = rmp_obj_val
                self.best_solution_node = rmp_node
            return has_integer_solution, None, rmp_node.z_vals, lp_iters, rmp_obj_val  # no need to branch
        else:
            return has_integer_solution, branch_candidates, rmp_node.z_vals, lp_iters, rmp_obj_val  # needs branch

    def construct_final_route(self):
        solution = []
        cost = 0
        for route_key, val in self.best_solution_node.z_vals.items():
            # Check if the variable is close to 1 in the solution
            if is_close(val, 1):
                route = route_dict[route_key]
                solution.append(route)
                cost += route['cost']
        return solution, cost
