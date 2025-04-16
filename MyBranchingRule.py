from pyscipopt import Branchrule, quicksum, SCIP_RESULT
from pyscipopt.scip import PY_SCIP_NODETYPE

import GeneralHelper
import RMP
from GeneralHelper import *
from NodeInfo import NodeInfo


class MyBranchingRule(Branchrule):

    def branchexeclp(self, allowaddcons):
        """
        Executes the branching rule during LP relaxation.
        """

        branch_cands, branch_cand_sols, branch_cand_fracs, ncands, npriocands, nimplcands = \
            self.model.getLPBranchCands()

        if ncands == 0:
            return {'result': SCIP_RESULT.DIDNOTRUN}

        cols = self.model.getLPColsData()
        node_vars = [col.getVar() for col in cols]
        vals = [col.getPrimsol() for col in cols]
        sum_vals = sum(vals)

        current_node = self.model.getCurrentNode()
        current_id = current_node.getNumber()
        current_node_info = RMP.node_infos[current_id]
        node_left = self.model.createChild(-(current_id + 1), self.model.getLPObjVal())
        node_right = self.model.createChild(-(current_id + 2), self.model.getLPObjVal())
        # update the node info
        node_left_id = node_left.getNumber()
        node_right_id = node_right.getNumber()
        current_node_info.child_ids = [node_left_id, node_right_id]
        # create node info
        RMP.node_infos[node_left_id] = NodeInfo(current_id)
        node_info_left = RMP.node_infos[node_left_id]
        node_info_left.as_child(current_node_info)
        # right
        RMP.node_infos[node_right_id] = NodeInfo(current_id)
        node_info_right = RMP.node_infos[node_right_id]
        node_info_right.as_child(current_node_info)

        ori_net = GeneralHelper.net

        # this requires to branch on all columns
        if not is_integer(sum_vals):
            fleet_ub, fleet_lb = int(sum_vals), int(sum_vals) + 1
            left_cons = self.model.createConsFromExpr(quicksum(node_vars) <= fleet_ub, f"{current_id}-left",
                                                      modifiable=True)
            right_cons = self.model.createConsFromExpr(quicksum(node_vars) >= fleet_lb, f"{current_id}-right",
                                                       modifiable=True)
            # add cons
            self.model.addConsNode(node_left, left_cons)
            self.model.addConsNode(node_right, right_cons)
            node_info_left.vehicle_fleet_branch_ub_cons_list.append(left_cons)
            node_info_right.vehicle_fleet_branch_lb_cons_list.append(right_cons)
            # update node info
            if node_info_left.vehicle_fleet_branch_ub > fleet_ub:
                node_info_left.vehicle_fleet_branch_ub = fleet_ub
                node_info_left.vehicle_fleet_branch_ub_cons = left_cons
            if node_info_right.vehicle_fleet_branch_lb < fleet_lb:
                node_info_right.vehicle_fleet_branch_lb = fleet_lb
                node_info_right.vehicle_fleet_branch_lb_cons = right_cons
        else:
            # exam flows on all arcs, two types of arc can be revisited: (Source, hub) and (hub, Sink)
            truck_arc_flows = {(i, j): 0.0 for i, j in ori_net.truck_arcs}
            drone_arc_flows = {(i, j): 0.0 for i, j in ori_net.drone_arcs}
            arc_travel_column_indices = {(i, j): set() for i, j in ori_net.truck_arcs + ori_net.drone_arcs}
            for col_id in range(len(current_node_info.columns)):
                flow_num = vals[col_id]
                col = current_node_info.columns[col_id]
                truck_path = list(col[0])
                drone_path = col[-1]
                for i in range(len(truck_path) - 1):
                    node_i = truck_path[i]
                    node_j = truck_path[i + 1]
                    # if (node_i == ori_net.depot_source and node_j in ori_net.hubs) or (
                    #         node_i in ori_net.hubs and node_j == ori_net.depot_sink):
                    #     continue
                    arc = (node_i, node_j)  # truck arc
                    truck_arc_flows[arc] += flow_num
                    arc_travel_column_indices[arc].add(col_id)
                    # also checks the drone paths
                    if node_i in ori_net.hubs:
                        for launch_hub, visits in drone_path:
                            if launch_hub == node_i:
                                for visit in visits:
                                    drone_arc = (launch_hub, visit.replace("_prime", ""))
                                    drone_arc_flows[drone_arc] += flow_num
                                    arc_travel_column_indices[drone_arc].add(col_id)
                                break
            # sorted in descending order of the distance between the fractional flow to 0.5
            combined_binary = [(k, v, 'truck') for k, v in truck_arc_flows.items() if 0 < v < 1] + [
                (k, v, 'drone') for k, v in drone_arc_flows.items() if 0 < v < 1]
            # branch on the binary flow
            if len(combined_binary) > 0:
                # branch on the least fractional arc
                arc, flow, best_source = min(combined_binary, key=lambda x: abs(x[1] - 0.5))
                node_i, node_j = arc

                new_disabled_arcs_left = set()
                new_disabled_arcs_right = set()

                # down branch (left)
                node_info_left.disabled_arcs.add(arc)
                new_disabled_arcs_left.add(arc)

                # up branch (right) must travel arc
                node_info_right.must_visit_arcs.add(arc)
                if best_source == 'truck':
                    for node_next in ori_net.truck_out_arcs[node_i]:
                        if node_next != node_j:
                            node_info_right.disabled_arcs.add((node_i, node_next))
                            new_disabled_arcs_right.add((node_i, node_next))
                else:
                    for node_pre in ori_net.truck_in_arcs[node_j]:
                        # disable the entrance from any other hub except node_i
                        if node_pre != node_i and node_pre in ori_net.hubs:
                            node_info_right.disabled_arcs.add((node_pre, node_j))
                            new_disabled_arcs_right.add((node_pre, node_j))
                    for node_next in ori_net.truck_out_arcs[node_i]:
                        # must immediately visit node_j if node_i is visited
                        if node_next != node_j:
                            node_info_right.disabled_arcs.add((node_i, node_next))
                            new_disabled_arcs_right.add((node_i, node_next))

                # remove columns
                remove_col_indices_left = set()
                remove_col_indices_right = set()
                for col_id in range(len(current_node_info.columns)):
                    col = current_node_info.columns[col_id]
                    truck_path = list(col[0])
                    drone_path = col[-1]
                    left_break = right_break = False
                    for i in range(len(truck_path) - 1):
                        if left_break and right_break:
                            break
                        node_i = truck_path[i]
                        node_j = truck_path[i + 1]
                        temp_arc = (node_i, node_j)  # truck arc
                        if temp_arc in new_disabled_arcs_left and not left_break:
                            remove_col_indices_left.add(col_id)
                            left_break = True
                        if temp_arc in new_disabled_arcs_right and not right_break:
                            remove_col_indices_right.add(col_id)
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
                                    remove_col_indices_left.add(col_id)
                                    left_break = True
                                if drone_arc in new_disabled_arcs_right and not right_break:
                                    remove_col_indices_right.add(col_id)
                                    right_break = True
                            break

                node_info_left.removed_columns_indices = remove_col_indices_left
                # remove the columns (fix their ub to 0), this does not introduce new constraints into the node
                for col_id in remove_col_indices_left:
                    node_var = node_vars[col_id]
                    self.model.chgVarUbNode(node_left, node_var, 0)
                for col_id in remove_col_indices_right:
                    node_var = node_vars[col_id]
                    self.model.chgVarUbNode(node_right, node_var, 0)
            else:
                combined_integer = [(k, v, 'truck') for k, v in truck_arc_flows.items() if v > 1] + [
                    (k, v, 'drone') for k, v in drone_arc_flows.items() if v > 1]
                # branch on the least fractional arc, here the elements are larger than 1
                arc, flow, best_source = min(combined_integer, key=lambda x: abs((x % 1) - 0.5))
                node_i, node_j = arc

                select_columns = [node_vars[i] for i in arc_travel_column_indices[arc]]
                left_cons = self.model.createConsFromExpr(quicksum(select_columns) <= int(flow), f"{current_id}-left",
                                                          modifiable=True)
                right_cons = self.model.createConsFromExpr(quicksum(select_columns) >= int(flow) + 1,
                                                           f"{current_id}-right", modifiable=True)
                # add cons
                self.model.addConsNode(node_left, left_cons)
                self.model.addConsNode(node_right, right_cons)
                # update node info
                node_info_left.arc_flow_ub_cons_list.append((node_i, node_j, flow, left_cons))
                node_info_right.arc_flow_lb_cons_list.append((node_i, node_j, flow, left_cons))
                sdsa = 0

        cons1 = node_left.getAddedConss()
        cons2 = node_right.getAddedConss()
        return {'result': SCIP_RESULT.BRANCHED}

    def branchexecps(self, allowaddcons):
        """
        Handles branching for pseudo solutions (no LP relaxation available).
        This method is needed to ensure SCIP can still branch even if LP is skipped.
        """

        # Get variables that can be branched on
        branch_cands, ncands, npriocands = self.model.getPseudoBranchCands()

        if ncands == 0:
            return {'result': SCIP_RESULT.DIDNOTRUN}
            # return SCIP_RESULT.DIDNOTRUN  # No branching needed

        # Choose a variable to branch on (e.g., first unfixed variable)
        for var in branch_cands:
            if var.vtype in ["BINARY", "INTEGER"]:
                var_val = self.model.getVal(var)  # Get the variable's current value

                node_left = self.model.createChild(0.0, self.model.getLocalEstimate())
                node_right = self.model.createChild(0.0, self.model.getLocalEstimate())

                # Branching by forcing the variable to take an integer value
                left_cons = self.model.createConsFromExpr(var <= int(var_val))
                right_cons = self.model.createConsFromExpr(var >= int(var_val) + 1)

                self.model.addConsNode(node_left, left_cons)
                self.model.addConsNode(node_right, right_cons)

                return {'result': SCIP_RESULT.BRANCHED}
                # return SCIP_RESULT.BRANCHED  # SCIP recognizes branching was done

        return {'result': SCIP_RESULT.DIDNOTFIND}
