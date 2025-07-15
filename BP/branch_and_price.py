import logging
import time
from queue import PriorityQueue

from colorama import Fore, Style
from gurobipy import GRB

import BP.rmp_node
import CommonHelper
from BP.rmp_node import RMPNode
from NodeInfo import NodeInfo

node_id_counter = 0
node_infos = {}
route_dict = {}  # key (truck_route, drone_route)
RMP_nodes = {}


def branch(current_node_id, branch_candidates, var_vals: dict):
    global node_id_counter
    current_node: NodeInfo = node_infos[current_node_id]
    # left node
    node_id_counter += 1
    node_left_id = node_id_counter
    node_left = NodeInfo(current_node_id, node_left_id, current_node.depth + 1)
    node_infos[node_left_id] = node_left
    node_left.as_child(current_node)
    # right node
    node_id_counter += 1
    node_right_id = node_id_counter
    node_right = NodeInfo(current_node_id, node_right_id, current_node.depth + 1)
    node_infos[node_right_id] = node_right
    node_right.as_child(current_node)

    current_node.child_ids = [node_left_id, node_right_id]

    ori_net = CommonHelper.net
    trans_net = CommonHelper.transformed_net
    # branch on vehicle fleet
    sum_vals = sum(var_vals.values())
    if not CommonHelper.is_integer(sum_vals):
        fleet_ub, fleet_lb = int(sum_vals), int(sum_vals) + 1
        # update node info
        if node_left.vehicle_fleet_branch_ub > fleet_ub:
            node_left.vehicle_fleet_branch_ub = fleet_ub
        if node_right.vehicle_fleet_branch_lb < fleet_lb:
            node_right.vehicle_fleet_branch_lb = fleet_lb
    # branch on the arc-flow
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
                arc = (node_i, node_j)  # truck arc
                if arc in truck_arc_flows.keys():
                    truck_arc_flows[arc] += flow_num
                    arc_travel_column_keys[arc].add(column_key)
                # also checks the drone paths
                if node_i in ori_net.hubs:
                    for launch_hub, visits in drone_path:
                        if launch_hub == node_i:
                            for visit in visits:
                                drone_arc = (launch_hub, visit)
                                if drone_arc in drone_arc_flows.keys():
                                    drone_arc_flows[drone_arc] += flow_num
                                    arc_travel_column_keys[drone_arc].add(column_key)
                            break
        # sorted in descending order of the distance between the fractional flow to 0.5
        truck_binary_flows = [(k, v) for k, v in truck_arc_flows.items() if
                              0 < v < 1 and not CommonHelper.is_integer(
                                  v) and k not in current_node.must_visit_arcs_trucks]
        drone_binary_flows = [(k, v) for k, v in drone_arc_flows.items() if
                              0 < v < 1 and not CommonHelper.is_integer(
                                  v) and k not in current_node.must_visit_arcs_drones]
        # branch on the binary flow (original network)
        arc = flow = where = None
        # we prioritize branching on truck flows
        if len(truck_binary_flows) > 0:
            arc, flow = min(truck_binary_flows, key=lambda x: abs(x[1] - 0.5))
            where = 'truck'
        elif len(drone_binary_flows) > 0:
            arc, flow = min(drone_binary_flows, key=lambda x: abs(x[1] - 0.5))
            where = 'drone'

        # branch on original network
        if arc is not None and flow is not None:
            node_i, node_j = arc
            # the arcs disabled in the original network
            new_disabled_arcs_left = set()
            new_disabled_arcs_right = set()

            # down branch (left)
            new_disabled_arcs_left.add(arc)
            if where == 'drone':
                node_left.disabled_arcs_drones.add(arc)
            else:
                node_left.disabled_arcs_trucks.add(arc)

            # up branch (right) must travel arc
            if where == 'truck':
                node_right.must_visit_arcs_trucks.add(arc)
                for node_next in ori_net.truck_out_arcs[node_i]:
                    if node_next != node_j:
                        temp_arc = (node_i, node_next)
                        node_right.disabled_arcs_trucks.add(temp_arc)
                        new_disabled_arcs_right.add(temp_arc)
                for node_pre in ori_net.truck_in_arcs[node_j]:
                    if node_pre != node_i:
                        temp_arc = (node_pre, node_j)
                        node_right.disabled_arcs_trucks.add(temp_arc)
                        new_disabled_arcs_right.add(temp_arc)
            else:  # drone arc
                node_right.must_visit_arcs_drones.add(arc)
                for node_pre in trans_net.in_arcs[node_j]:
                    # disable the entrance from any other hub except node_i
                    if node_pre != node_i and node_pre in trans_net.hubs:
                        node_right.disabled_arcs_trans.add((node_pre, node_j))

                # for node_pre in ori_net.truck_in_arcs[node_j]:
                #     # disable the entrance from any other hub except node_i
                #     if node_pre != node_i and node_pre in ori_net.hubs:
                #         node_right.disabled_arcs_trucks.add((node_pre, node_j))
                #         new_disabled_arcs_right.add((node_pre, node_j))
                # for node_next in ori_net.truck_out_arcs[node_i]:
                #     # must immediately visit node_j if node_i is visited
                #     if node_next != node_j:
                #         node_right.disabled_arcs.add((node_i, node_next))
                #         new_disabled_arcs_right.add((node_i, node_next))

            # remove columns
            remove_col_keys_left = set()
            remove_col_keys_right = set()

            for column_key in current_node.columns:
                route = route_dict[column_key]
                # check left
                for temp_node_i, temp_node_j in new_disabled_arcs_left:
                    # the column travels a removed arc
                    if CommonHelper.if_route_travel_arc(route, temp_node_i, temp_node_j, CommonHelper.net):
                        remove_col_keys_left.add(column_key)
                        break
                # check right
                for temp_node_i, temp_node_j in new_disabled_arcs_right:
                    # the column travels a removed arc
                    if CommonHelper.if_route_travel_arc(route, temp_node_i, temp_node_j, CommonHelper.net):
                        remove_col_keys_right.add(column_key)
                        break

            node_left.removed_columns_keys.update(remove_col_keys_left)
            node_right.removed_columns_keys.update(remove_col_keys_right)
            # remove the columns
            for column_key in remove_col_keys_left:
                node_left.columns.remove(column_key)
                del node_left.column_customer_visits[column_key]
                # check SR infos
                for triple in node_left.column_in_SR_triples[column_key]:
                    node_left.SR_infos[triple].remove(column_key)
                    if node_left.SR_infos[triple] == 0:
                        node_left.added_SR_keys.remove(triple)
                node_left.column_in_SR_triples[column_key].clear()
            for column_key in remove_col_keys_right:
                node_right.columns.remove(column_key)
                del node_right.column_customer_visits[column_key]
                # check SR infos
                for triple in node_right.column_in_SR_triples[column_key]:
                    node_right.SR_infos[triple].remove(column_key)
                    if node_right.SR_infos[triple] == 0:
                        node_right.added_SR_keys.remove(triple)
                node_right.column_in_SR_triples[column_key].clear()
        # branch on the transformed network
        else:
            # exam flows on all arcs, two types of arc can be revisited: (Source, hub) and (hub, Sink)
            arc_flows = {(i, j): 0.0 for i, j in trans_net.arcs if
                         i != trans_net.depot_source and j != trans_net.depot_sink}
            arc_travel_column_keys = {(i, j): set() for i, j in trans_net.arcs if
                                      i != trans_net.depot_source and j != trans_net.depot_sink}
            for column_key in current_node.columns:
                flow_num = var_vals[column_key]
                element_path = list(column_key)
                for i in range(len(element_path) - 1):
                    node_i = element_path[i]
                    node_j = element_path[i + 1]
                    arc = (node_i, node_j)  # truck arc
                    if arc in arc_flows.keys():
                        arc_flows[arc] += flow_num
                        arc_travel_column_keys[arc].add(column_key)
            binary_flows = [(k, v) for k, v in arc_flows.items() if
                            0 < v < 1 and not CommonHelper.is_integer(
                                v) and k not in current_node.must_visit_arcs_trans]
            # branch on the binary flow (original network)
            if len(binary_flows) == 0:
                raise Exception("wrong branching case")
            # branch on the most fractional arc
            arc, flow = min(binary_flows, key=lambda x: abs(x[1] - 0.5))
            node_i, node_j = arc

            new_disabled_arcs_left = set()
            new_disabled_arcs_right = set()

            # down branch (left)
            node_left.disabled_arcs_trans.add(arc)
            new_disabled_arcs_left.add(arc)

            # up branch (right) must travel arc
            node_right.must_visit_arcs_trans.add(arc)
            for node_next in trans_net.out_arcs[node_i]:
                # must immediately visit node_j if node_i is visited
                if node_next != node_j:
                    disabled_arc = (node_i, node_next)
                    node_right.disabled_arcs_trans.add(disabled_arc)
                    new_disabled_arcs_right.add(disabled_arc)
            if node_j not in trans_net.hubs:
                for node_pre in trans_net.in_arcs[node_j]:
                    if node_pre != node_i:
                        disabled_arc = (node_pre, node_j)
                        node_right.disabled_arcs_trans.add(disabled_arc)
                        new_disabled_arcs_right.add(disabled_arc)

            # remove columns
            remove_col_keys_left = set()
            remove_col_keys_right = set()
            for column_key in current_node.columns:
                element_path = list(column_key)
                for temp_arc in new_disabled_arcs_left:
                    if CommonHelper.if_path_travel_arc(element_path, temp_arc):
                        remove_col_keys_left.add(column_key)
                        break
                for temp_arc in new_disabled_arcs_right:
                    if CommonHelper.if_path_travel_arc(element_path, temp_arc):
                        remove_col_keys_right.add(column_key)
                        break

            node_left.removed_columns_keys.update(remove_col_keys_left)
            node_right.removed_columns_keys.update(remove_col_keys_right)
            # remove the columns
            for column_key in remove_col_keys_left:
                node_left.columns.remove(column_key)
                del node_left.column_customer_visits[column_key]
                # check SR infos
                for triple in node_left.column_in_SR_triples[column_key]:
                    node_left.SR_infos[triple].remove(column_key)
                    if node_left.SR_infos[triple] == 0:
                        node_left.added_SR_keys.remove(triple)
                node_left.column_in_SR_triples[column_key].clear()
            for column_key in remove_col_keys_right:
                node_right.columns.remove(column_key)
                del node_right.column_customer_visits[column_key]
                # check SR infos
                for triple in node_right.column_in_SR_triples[column_key]:
                    node_right.SR_infos[triple].remove(column_key)
                    if node_right.SR_infos[triple] == 0:
                        node_right.added_SR_keys.remove(triple)
                node_right.column_in_SR_triples[column_key].clear()

    return node_left_id, node_right_id


def solve_node(node_info: NodeInfo, rmp_node: RMPNode):
    """
    iteratively solve the RMP, then solve pricing problem using Benders decomposition (BMP, BSP), until no new
    columns found
    :return: is_integer, branch_candidates, var_vals, lp_iters, lp_obj_val
    """

    print(f"running pricer at node {node_info.id}")
    s_time = time.time()

    rmp_node.solve()
    lp_iters = 1

    # column generation
    new_col_num = 0
    while True:
        add_new_column = rmp_node.run_pricer(node_info)
        run_time = time.time() - s_time
        new_col_num += 1
        rmp_node.solve()
        if not add_new_column or run_time > CommonHelper.max_run_time:
            break
        lp_iters += 1

    rmp_node.model.update()

    # the current node is infeasible
    if rmp_node.status == GRB.INFEASIBLE:
        return None, None, None, lp_iters, None

    rmp_obj_val = rmp_node.obj_val

    # check integrality of RMP
    is_integer_solution, branch_candidates = rmp_node.is_integer()
    if is_integer_solution:  # update incumbent
        return is_integer_solution, None, rmp_node.z_vals, lp_iters, rmp_obj_val  # no need to branch
    else:
        return is_integer_solution, branch_candidates, rmp_node.z_vals, lp_iters, rmp_obj_val  # needs branch


class BranchAndPrice:
    def __init__(self, max_runtime=float('inf')):
        self.max_runtime = max_runtime
        self.branch_queue = PriorityQueue()  # (lp_obj_val, node_id)
        self.node_solutions = {}  # is_integer_sol, branch_candidates, var_vals, lp_iters, lp_obj_val
        self.node_lp_objs = {}
        self.global_upper_bound = float('inf')  # the cost of incumbent solution
        self.global_lower_bound = -float('inf')
        self.gap = 1  # optimality gap
        self.best_solution_node: RMPNode = None
        self.initial_element_paths = []
        self.s_time = None
        # add initial routes
        routes, element_paths = CommonHelper.find_initial_routes(CommonHelper.net, CommonHelper.transformed_net)
        for i in range(len(routes)):
            route_key, route = routes[i]
            route_dict[route_key] = route
            CommonHelper.initial_routes.append(route_key)
        self.initial_element_paths = list(element_paths.values())

    def solve(self):
        """Main branch-and-price loop"""
        if self.s_time is None:
            self.s_time = time.time()
        global node_id_counter
        # add root node
        node_id_counter += 1
        node_id = node_id_counter
        root_node = NodeInfo(-1, node_id, 1)
        node_infos[node_id] = root_node
        root_node.columns = CommonHelper.initial_routes
        root_node.column_elementary_paths = self.initial_element_paths
        # setup the column customer visits
        for elem_path in root_node.column_elementary_paths:
            route_key = tuple(elem_path)
            root_node.column_customer_visits[route_key].update(
                CommonHelper.get_route_customer_visits(route_dict[route_key]))
        # initialize SR infos
        root_node.init_SR_infos()
        # solve the root node
        RMP_node = RMPNode(root_node)
        RMP_nodes[root_node.id] = RMP_node
        s_t = time.time()

        is_integer_sol, branch_candidates, var_vals, lp_iters, lp_obj_val = solve_node(root_node, RMP_node)
        use_time = time.time() - s_t
        self.node_solutions[node_id] = tuple([is_integer_sol, branch_candidates, var_vals, lp_iters, lp_obj_val])
        self.node_lp_objs[node_id] = lp_obj_val
        # add the column to the queue
        self.branch_queue.put(((1, node_id), node_id))
        # print
        local_lb_print = round(lp_obj_val, 2) if not is_integer_sol else "-"
        log_text = f"solve root node, l_lb: {local_lb_print}, time: {use_time:2f}"
        if is_integer_sol:  # find integer solution
            log_text += f", integer obj: {round(lp_obj_val, 2)}"
        print(Fore.RED + log_text + Style.RESET_ALL)
        CommonHelper.root_node_runtime = use_time

        # main loop
        while not self.branch_queue.empty():
            if time.time() - self.s_time > CommonHelper.max_run_time:
                logging.info("Terminated due to reaching maximum runtime.")
                break

            priority, node_id = self.branch_queue.get()
            # solve the current node
            is_integer_sol, branch_candidates, var_vals, lp_iters, lp_obj_val = self.node_solutions[node_id]

            # feasible node
            if is_integer_sol is not None:
                # run primal heuristic
                if not is_integer_sol and CommonHelper.enable_primal_heuristics:
                    find_integer_sol = RMP_node.solve_IP()
                    if find_integer_sol and self.global_upper_bound > RMP_node.primal_obj_val:
                        self.global_upper_bound = RMP_node.primal_obj_val
                        self.best_solution_node = RMP_nodes[node_id]

                if not is_integer_sol:
                    pruned_by_ub = True if lp_obj_val >= self.global_upper_bound else False
                else:
                    pruned_by_ub = True if lp_obj_val > self.global_upper_bound else False

                if not is_integer_sol:
                    # branch
                    if lp_obj_val < self.global_upper_bound:
                        left_node_id, right_node_id = branch(node_id, branch_candidates, var_vals)
                        # solve the child nodes at once
                        rmp_left_node = RMPNode(node_infos[left_node_id])
                        RMP_nodes[left_node_id] = rmp_left_node
                        self.node_solutions[left_node_id] = solve_node(node_infos[left_node_id], rmp_left_node)
                        rmp_right_node = RMPNode(node_infos[right_node_id])
                        RMP_nodes[right_node_id] = rmp_right_node
                        self.node_solutions[right_node_id] = solve_node(node_infos[right_node_id], rmp_right_node)
                        # if the child node is feasible
                        if self.node_solutions[left_node_id][-1] is not None:
                            self.node_lp_objs[left_node_id] = self.node_solutions[left_node_id][-1]
                        if self.node_solutions[right_node_id][-1] is not None:
                            self.node_lp_objs[right_node_id] = self.node_solutions[right_node_id][-1]

                        # widest
                        # self.branch_queue.put(((node_infos[left_node_id].depth, left_node_id), left_node_id))
                        # self.branch_queue.put(((node_infos[right_node_id].depth, right_node_id), right_node_id))

                        # best-first
                        if self.node_solutions[left_node_id][-1] is not None:
                            self.branch_queue.put(((self.node_solutions[left_node_id][-1], left_node_id), left_node_id))
                        if self.node_solutions[right_node_id][-1] is not None:
                            self.branch_queue.put(
                                ((self.node_solutions[right_node_id][-1], right_node_id), right_node_id))

                        # deepest
                        # self.branch_queue.put(((-node_infos[left_node_id].depth, left_node_id), left_node_id))
                        # self.branch_queue.put(((-node_infos[right_node_id].depth, right_node_id), right_node_id))

                        # update global lower bound
                        best_id, best_lb = min(self.node_lp_objs.items(), key=lambda item: item[1])
                        # best_lb = round(min(self.node_lp_objs.values()), 2)
                        if best_lb > self.global_lower_bound:
                            self.global_lower_bound = best_lb
                            CommonHelper.node_lp_trace.append((best_id, best_lb))
                        # self.global_lower_bound = round(min(self.node_lp_objs.values()), 2)
                else:  # integer solution
                    if lp_obj_val < self.global_upper_bound:
                        self.global_upper_bound = lp_obj_val
                        self.best_solution_node = RMP_nodes[node_id]

                # remove the current node lp info
                del self.node_lp_objs[node_id]

                if self.global_upper_bound < float('inf'):
                    self.gap = (self.global_upper_bound - self.global_lower_bound) / abs(self.global_upper_bound)
                gap_text = round(self.gap * 100, 2) if self.gap < 1 else "-"

                ub_print = "-" if self.global_upper_bound == float('inf') else self.global_upper_bound
                g_lb_print = "-" if self.global_lower_bound == -float('inf') else self.global_lower_bound
                local_lb_print = round(lp_obj_val, 2) if not is_integer_sol else "-"
                log_text = (
                    f"LP iters: {lp_iters}, node id: {node_id}, queue: {self.branch_queue.qsize()}, gap: {gap_text}%, "
                    f"cols: {len(node_infos[node_id].columns)}, g_ub: {ub_print}, g_lb: {g_lb_print}, "
                    f"l_lb: {local_lb_print}")
                if is_integer_sol:  # find integer solution
                    log_text += f", integer obj: {round(lp_obj_val, 2)}"
                if pruned_by_ub:
                    log_text += f", pruned by UB"
            else:  # infeasible node
                log_text = (
                    f"LP iters: {lp_iters}, solved node id: {node_id}, queue size: {self.branch_queue.qsize()}, "
                    f"infeasible node")

            print(Fore.RED + log_text + Style.RESET_ALL)
            CommonHelper.node_log_list[node_id] = log_text

        # terminated
        final_solution, cost = self.construct_final_route()
        print(f"total nodes: {len(node_infos)}")

        BP.rmp_node._pool = None
        
        return time.time() - self.s_time, final_solution, cost

    def construct_final_route(self):
        solution = []
        cost = 0
        if self.best_solution_node is None:
            return None, None
        if not self.best_solution_node.find_primal_solution:
            z_vals = self.best_solution_node.z_vals.items()
        else:
            z_vals = self.best_solution_node.primal_z_vals.items()
        for route_key, val in z_vals:
            # Check if the variable is close to 1 in the solution
            if CommonHelper.is_close(val, 1):
                route = route_dict[route_key]
                solution.append(route)
                cost += route['cost']
        return solution, cost

    def add_warm_start_solution(self, route_list):
        self.s_time = time.time()
        for i in range(len(route_list)):
            route = route_list[i]
            if len(route) == 0:
                continue
            elem_path = CommonHelper.route_to_elem_path(route, CommonHelper.transformed_net)
            route_key = tuple(elem_path)
            route_id = CommonHelper.num_customers + i
            route_cost, _ = CommonHelper.route_get_cost(route, CommonHelper.net, CommonHelper.transformed_net)
            if route_key not in route_dict.keys():
                route['id'] = route_id
                route['cost'] = route_cost
                route_dict[route_key] = route
                CommonHelper.initial_routes.append(route_key)
                self.initial_element_paths.append(elem_path)
