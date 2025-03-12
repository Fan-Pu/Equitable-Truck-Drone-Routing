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
            # return SCIP_RESULT.DIDNOTRUN

        # rank the candidate solution according to the value of |x-0.5|
        # Compute distance to 0.5
        frac_dist = [(cand, frac, abs(frac - 0.5)) for cand, frac in zip(branch_cands, branch_cand_fracs)]

        # Sort candidates based on proximity to 0.5 (ascending order)
        sorted_cands = sorted(frac_dist, key=lambda x: x[2])

        # branching variables
        remove_var_names = []
        sum_branch_z_val = sum(branch_cand_sols)

        if is_integer(sum_branch_z_val):
            for z, frac_val, _ in reversed(sorted_cands):
                temp_sum_val = sum_branch_z_val - frac_val
                if not is_integer(temp_sum_val):
                    remove_var_names.append(z.name)
                    sum_branch_z_val = temp_sum_val
                    break
            if len(remove_var_names) == 0:
                raise Exception("error in branch")
        branch_z_set = [var for var in branch_cands if var.name not in remove_var_names]
        branch_z_names = [var.name for var in branch_z_set]

        current_node = self.model.getCurrentNode()
        current_id = current_node.getNumber()
        current_node_info = RMP.node_infos[current_id]

        node_left = self.model.createChild(-(current_id + 1), self.model.getLPObjVal())
        node_right = self.model.createChild(-(current_id + 2), self.model.getLPObjVal())
        node_left.getNumber()

        left_cons = self.model.createConsFromExpr(quicksum(branch_z_set) <= int(sum_branch_z_val),
                                                  f"{current_id}-left")
        right_cons = self.model.createConsFromExpr(quicksum(branch_z_set) >= int(sum_branch_z_val) + 1,
                                                   f"{current_id}-right")

        self.model.addConsNode(node_left, left_cons)
        self.model.addConsNode(node_right, right_cons)

        # update the node info
        node_left_id = node_left.getNumber()
        node_right_id = node_right.getNumber()

        if current_id == 1:
            sdsa = 0

        if all(elem in current_node_info.columns for elem in test_path_list):
            test_node_ids.extend([node_left_id, node_right_id])

        # left
        RMP.node_infos[node_left_id] = NodeInfo(current_id)
        RMP.node_infos[node_left_id].columns, RMP.node_infos[
            node_left_id].branches = current_node_info.columns.copy(), current_node_info.branches.copy()
        RMP.node_infos[node_left_id].branches.append((branch_z_names, int(sum_branch_z_val), "<="))
        # right
        RMP.node_infos[node_right_id] = NodeInfo(current_id)
        RMP.node_infos[node_right_id].columns, RMP.node_infos[
            node_right_id].branches = current_node_info.columns.copy(), current_node_info.branches.copy()
        RMP.node_infos[node_right_id].branches.append((branch_z_names, int(sum_branch_z_val) + 1, ">="))

        GeneralHelper.local_estimates[node_left_id] = node_left.getEstimate()
        GeneralHelper.local_estimates[node_right_id] = node_right.getEstimate()

        GeneralHelper.node_dict[node_left_id] = node_left
        GeneralHelper.node_dict[node_right_id] = node_right

        return {'result': SCIP_RESULT.BRANCHED}
        # return SCIP_RESULT.BRANCHED

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
        # return SCIP_RESULT.DIDNOTFIND  # No branching performed
