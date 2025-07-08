import CommonHelper
from NodeInfo import NodeInfo
from TransformedNetwork import TransformedNetwork


class LabelForward:
    def __init__(self, path, truck_load, drones_used, arrival_time, sync_time, wait_time, psi_set, cost, depth,
                 trans_net: TransformedNetwork, alternative_extensions=None):
        self.path = path  # Ordered sequence of visited nodes (partial path)
        self.truck_load = truck_load  # Truck loading weight
        self.drones_used = drones_used  # Number of drones used
        self.arrival_time = arrival_time  # Arrival time at last node
        self.sync_time = sync_time  # Arrival time at last synchronization point
        self.wait_time = wait_time  # truck waiting time
        self.psi_set = psi_set.copy()  # for SR inequalities
        self.cost = cost  # Accumulated cost
        # auxiliary components
        self.net = trans_net
        self.depth = depth
        if alternative_extensions is None:
            self.alternative_extensions = [node_j for node_j in self.net.out_arcs[self.path[-1]]]
        else:
            self.alternative_extensions = alternative_extensions.copy()
        self.latest_hub = None
        self.path_set = set()

    def dominates(self, other, node_info: NodeInfo, farkas: bool, duals=None):
        """Check if this label dominates another."""
        strict = False  # check whether contains a strict condition

        # condition subset violated
        if not self.path_set.issubset(other.path_set):
            return False
        elif len(self.path_set) != len(other.path_set):  # strict subset
            strict = True

        # condition truck load violated
        if self.truck_load > other.truck_load:
            return False
        elif self.truck_load < other.truck_load:
            strict = True

        # condition drone fleet violated
        if self.drones_used > other.drones_used:
            return False
        elif self.drones_used < other.drones_used:
            strict = True

        # condition 2, arrival time violated
        if not farkas:
            if self.arrival_time > other.arrival_time:
                return False
            elif self.arrival_time < other.arrival_time:
                strict = True

        # condition 2, sync time violated
        if not farkas:
            if self.sync_time > other.sync_time:
                return False
            elif self.sync_time < other.sync_time:
                strict = True

        # condition 2, waiting time violated
        if not farkas:
            if self.wait_time > other.wait_time:
                return False
            elif self.wait_time < other.wait_time:
                strict = True

        # condition 2, cost violated
        if self.cost > other.cost:
            return False
        else:
            # calculate sum_nu
            sum_nu = 0
            for triple in node_info.added_SR_keys:
                if self.psi_set[triple] in {1, 3} and other.psi_set[triple] in {0, 2}:
                    sum_nu += duals[triple]

            if self.cost - sum_nu + CommonHelper.close_tolerance > other.cost:
                return False
            elif self.cost - sum_nu + CommonHelper.close_tolerance < other.cost:
                strict = True

        # here all conditions are satisfied, we need to ensure that at least one is strict
        return strict

    def allow_extend(self, node_j, node_info: NodeInfo, N_hat: set):
        """
        check whether we can extend the current label to node_j
        """
        node_i = self.path[-1]
        _node_i = node_i.replace("_T", "")
        _node_j = node_j.replace("_T", "")
        arc = (node_i, node_j)

        # check Loading Capacity Constraints
        if self.truck_load + self.net.demand_weights[node_j] > CommonHelper.truck_max_weight:
            return False

        # check Drone Fleet Constraints
        if arc in self.net.arcs_scp or arc in self.net.arcs_cpcp:
            if self.drones_used >= CommonHelper.num_drones_per_truck:
                return False

        # check branch arcs in transformed network
        if arc in node_info.disabled_arcs_trans:
            return False

        # check Customer Visits Constraints
        if not CommonHelper.enable_DSS:
            if node_j in self.net.customers_origin:
                dup_node = node_j + "_T"
                if node_j in self.path or dup_node in self.path:
                    return False
            elif node_j in self.net.customers_prime:
                dup_node = _node_j
                if node_j in self.path or dup_node in self.path:
                    return False
            else:  # not a customer node
                if node_j in self.path:
                    return False
        else:  # enable DSS
            if node_j in N_hat:
                return False

        # check Node Accessibility Check
        if arc in self.net.arcs_cpcp:
            if (self.latest_hub, node_j) not in self.net.arcs_scp:
                return False
        if arc in self.net.arcs_cpc:
            if (self.latest_hub, node_j) not in self.net.arcs_ori:
                return False

        # check Arrival Time Constraints
        if node_j in self.net.customers:
            # get the arrival time at j
            node_j_arrive_t = CommonHelper.get_arrive_time(self.arrival_time, node_i, node_j, self.latest_hub,
                                                           self.sync_time, self.wait_time, self.net)
            if node_j == "C":
                sdas = 0
            if node_j_arrive_t + CommonHelper.close_tolerance < self.net.a_lb[_node_j]:
                return False

        # check disabled truck arcs in the original network
        if arc in self.net.arcs_ori and arc in node_info.disabled_arcs_trucks:
            return False

        # check AD+
        if arc in self.net.arcs_cpc:
            hub_idx = self.path.index(self.latest_hub)
            for s, n in node_info.must_visit_arcs_drones:
                if s != self.latest_hub:
                    continue
                if n not in self.path[hub_idx + 1:]:
                    return False
        elif arc in self.net.arcs_cpcp:
            for s, n in node_info.must_visit_arcs_drones:
                if s == self.latest_hub:
                    continue
                if n == node_j:
                    return False

        # check AD-
        if arc in self.net.arcs_scp and (node_i, node_j) in node_info.disabled_arcs_drones:
            return False
        if arc in self.net.arcs_cpcp and (self.latest_hub, node_j) in node_info.disabled_arcs_drones:
            return False

        # check AK-
        if arc in self.net.arcs_ori and arc in node_info.disabled_arcs_trucks:
            return False
        if arc in self.net.arcs_cpc and (self.latest_hub, node_j) in node_info.disabled_arcs_trucks:
            return False

        return True

    def extend(self, node_j, duals, farkas, node_info: NodeInfo):
        """
        extend the label to j
        """

        node_i = self.path[-1]
        label_j = self.__class__(
            path=self.path[:],
            truck_load=self.truck_load,
            drones_used=self.drones_used,
            arrival_time=self.arrival_time,
            sync_time=self.sync_time,
            wait_time=self.wait_time,
            psi_set=self.psi_set,
            cost=self.cost,
            depth=self.depth + 1,
            trans_net=self.net
        )
        label_j.alternative_extensions = [node for node in self.net.out_arcs[node_j] if node not in self.path]
        label_j.latest_hub = self.latest_hub

        # update path
        label_j.path.append(node_j)
        label_j.path_set = self.path_set.copy()
        label_j.path_set.add(node_j)

        # update latest_hub
        if node_j in self.net.hubs:
            label_j.latest_hub = node_j

        # update truck load
        if node_j in self.net.customers:
            label_j.truck_load = self.truck_load + self.net.demand_weights[node_j]

        # update drone used
        if node_j in self.net.customers_prime:
            label_j.drones_used += 1
        else:
            label_j.drones_used = 0

        # update a
        label_j.arrival_time = CommonHelper.get_arrive_time(self.arrival_time, node_i, node_j, label_j.latest_hub,
                                                            self.sync_time, self.wait_time, self.net)

        # update sync time
        if node_j in self.net.hubs:
            label_j.sync_time = label_j.arrival_time

        # update waiting time
        if node_j in self.net.customers_prime:
            label_j.wait_time = max(self.wait_time, label_j.arrival_time - label_j.sync_time)
        else:
            label_j.wait_time = 0

        # update psi
        sum_nu = 0
        for triple in node_info.added_SR_keys:
            if node_j.replace("_T", "") in set(triple):
                label_j.psi_set[triple] += 1
                if self.psi_set[triple] == 1 and label_j.psi_set[triple] == 2:
                    sum_nu += duals[triple]

        # update cost
        if not farkas:
            if node_j in self.net.customers:
                _node_j = node_j.replace("_T", "")
                index = self.net.customers.index(_node_j)
                label_j.cost += CommonHelper.cw * (label_j.arrival_time - self.net.a_lb[_node_j]) ** 2 - duals["mu"][
                    index] - sum_nu
                if node_j in self.net.customers_prime:
                    label_j.cost += CommonHelper.drone_cost_per_flight
            elif node_j == self.net.depot_sink:
                label_j.cost += CommonHelper.cw * label_j.arrival_time - sum_nu
            else:
                label_j.cost += -sum_nu
        else:
            if node_j in self.net.customers:
                index = self.net.customers.index(node_j.replace("_T", ""))
                label_j.cost += -duals["mu"][index] - sum_nu
            else:
                label_j.cost += -sum_nu

        return label_j

    def __eq__(self, other):
        if isinstance(other, LabelForward):
            return self.path == other.path
            # return self.truck_path == other.truck_path and self.drone_flights == other.drone_flights
        return False

    def __hash__(self):
        return hash(tuple(self.path))  # Hash based on ID so customers with same ID are treated as the same

    def __lt__(self, other):
        """Defines how to compare two LabelForward objects"""
        return self.depth < other.depth  # Compare based on depth
