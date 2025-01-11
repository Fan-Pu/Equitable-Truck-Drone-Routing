import math

import GeneralHelper
from BMP import BMP
from BSP import BSP
from GeneralHelper import *


class PSP:
    """
    pricing subproblem
    """

    def __init__(self, net):
        self.BMP = BMP(net)
        self.BSP = BSP(net)
        # self.IntM = PSPIntM(net)

    def solve(self, net, duals):
        """
        solve the pricing problem
        """
        self.BMP.remove_cuts()  # remove the cuts added in the previous iteration

        # # solve the integrated model
        # self.IntM.update_objective(net, duals)
        # IntM_obj_val = self.IntM.solve()

        iter_num = 0

        # Benders loop
        while True:
            iter_num += 1
            self.BMP.update_objective_and_BSP_Lb(net, duals)
            BMP_obj_val, x_vals, phi_value = self.BMP.solve()
            self.BSP.update_objective(net, duals)
            self.BSP.update_binding_cons(x_vals)
            BSP_LP_obj_val, BSP_duals = self.BSP.solveLP()
            # add subgradient cut
            if phi_value + close_tolerance < BSP_LP_obj_val and not is_close(phi_value, BSP_LP_obj_val):
                self.separate_subgradient_cut(BSP_LP_obj_val, BSP_duals, x_vals)
            # needs to solve IP
            else:
                BSP_IP_obj_val, y_vals, route_cost = self.BSP.solveIP()
                # optimal solution found
                if is_close(BSP_IP_obj_val, phi_value):
                    # if not math.isclose(BMP_obj_val, IntM_obj_val):
                    #     sdsad = 0
                    # negative reduced cost, return the new column
                    if BMP_obj_val + close_tolerance < 0:
                        route = self.get_route(net, x_vals, y_vals, route_cost)
                        return route, True
                    else:
                        return None, False
                elif phi_value + close_tolerance < BSP_IP_obj_val:
                    self.separate_L_shaped_cut(BSP_IP_obj_val, x_vals, GeneralHelper.BSP_LB)
                else:
                    raise Exception("Errors in BSP objective value")

            print()

    def separate_subgradient_cut(self, LP_obj_val, duals, x_vals):
        rhs = LP_obj_val
        for key, val in duals.items():
            rhs += val * (self.BMP.x_dict[key] - x_vals[key])
        cons = self.BMP.model.addConstr(self.BMP.phi >= rhs, f"subgrad_{len(self.BMP.subgradient_cuts)}")
        self.BMP.subgradient_cuts.append(cons)
        self.BMP.constraints.append(cons)
        # check the validness of this constraint
        rhs_val = rhs.getValue()
        phi_val = self.BMP.phi.X
        # the phi_val is wrong
        if phi_val > LP_obj_val:
            sdsa = 0
        # the node is not cut
        if phi_val >= rhs_val:
            sdas = 0

    def separate_L_shaped_cut(self, obj_val, x_vals, L):
        # the set such that x are 1
        S_set = [key for key, val in x_vals.items() if is_close(val, 1)]
        rhs = obj_val + (obj_val - L) * (
                -len(S_set) + sum(self.BMP.x_dict[key] for key in S_set) - sum(
            var for key, var in self.BMP.x_dict.items() if key not in S_set)
        )
        cons = self.BMP.model.addConstr(self.BMP.phi >= rhs, f"Lshaped_{len(self.BMP.Lshaped_cuts)}")
        self.BMP.Lshaped_cuts.append(cons)
        self.BMP.constraints.append(cons)
        # check the correctness of the constraint

    def get_route(self, net, x_vals, y_vals, route_cost):
        """
        given the values of variables, output the route solution
        """
        # for truck
        active_links = {key for key, value in x_vals.items() if is_close(1, value)}
        # Initialize path
        start_node = net.depot_source
        end_node = net.depot_sink
        truck_route = [start_node]
        # Follow the links to construct the path
        current_node = net.all_nodes_indices[start_node]
        while current_node != net.all_nodes_indices[end_node]:
            # Find the next node connected to the current node
            next_node = next((j for i, j in active_links if i == current_node), None)
            if next_node is None:
                raise ValueError(f"No valid path found from node {current_node}.")
            truck_route.append(net.all_nodes[next_node])
            current_node = next_node

        # for drone
        drone_route = [net.all_nodes[j] for (i, j, d), value in y_vals.items() if is_close(1, value)]
        drone_links = [(net.all_nodes[i], net.all_nodes[j]) for (i, j, d), value in y_vals.items() if
                       math.isclose(1, value)]
        route = {'truck': truck_route, 'drone': drone_route, 'cost': route_cost, 'drone links': drone_links}

        return route
