import random
import re
from line_profiler import LineProfiler
import networkx as nx
from collections import defaultdict
from Network import Network
from TransformedNetwork import TransformedNetwork
from sortedcontainers import SortedSet

# test_path = ['Source', 'H1', 'C1_prime', 'C3_prime', 'Sink']
# test_path = (tuple(['Source', 'H1', 'Sink']), frozenset(
#     {
#         'H1': frozenset({'C1_prime', 'C3_prime'})
#     }.items()
# ))

# test_path = ['Source', 'H1', 'H1_prime', 'C4_prime', 'Sink']
# test_path = ['Source', 'C2', 'Sink']

# test_path_list = [
#     ['Source', 'H1', 'H1_prime', 'C1_prime', 'C3_prime', 'Sink'],
#     ['Source', 'H1', 'H1_prime', 'C4_prime', 'Sink'],
#     ['Source', 'C2', 'Sink']
# ]

columns_list = []

merge_num = 0

final_model = None

# test_path_list = [
#     (tuple(['Source', 'H1', 'Sink']), frozenset(
#         {
#             'H1': frozenset({'C1_prime', 'C3_prime'})
#         }.items()
#     )),
#     (tuple(['Source', 'H1', 'Sink']), frozenset(
#         {
#             'H1': frozenset({'C4_prime'})
#         }.items()
#     ))
# ]


test_path_list = [
    (tuple(['Source', 'H3', 'Sink']), frozenset(
        {
            'H3': frozenset({'C4_prime', 'C8_prime'})
        }.items()
    )),
    (tuple(['Source', 'H3', 'Sink']), frozenset(
        {
            'H3': frozenset({'C1_prime', 'C3_prime', 'C6_prime'})
        }.items()
    ))
]

test_node_ids = []

test_sols = []

forward_dominance_num = 0
backward_cost_dominance_num = 0
backward_arrival_dominance_num = 0
label_merge_num = 0
label_forward_num = 0
label_backward_num = 0

lp = LineProfiler()

LSA_mode = 1  # 0 for combined, 1 for forward, 2 for backward

seed = 2024

net = None  # the network object
transformed_net = None  # the transformed network object

# drone travel time
drone_min_t = 2
drone_max_t = 8

# truck travel time
truck_min_t = 10
truck_max_t = 50

# parcel weights
demand_weight_max = 10
demand_weight_min = 1

# endurance
truck_max_weight = 100
drone_endurance = 150

# for sub-tour elimination
epsilon = 1

# needs to branch
# num_customers = 4
# num_hubs = 3
# num_trucks = 3
# num_drones_per_truck = 2

num_customers = 8
num_hubs = 2
num_trucks = 5
num_drones_per_truck = 3

# # directly goes to second branch rule
# num_customers = 10
# num_hubs = 3
# num_trucks = 5
# num_drones_per_truck = 3

# num_customers = 8
# num_hubs = 1
# num_trucks = 8
# num_drones_per_truck = 3

# route_list = [{'id': 4, 'truck': ['Source', 'C5', 'Sink'], 'drone': [], 'launches': [], 'cost': 550},
#               {'id': 6, 'truck': ['Source', 'C7', 'Sink'], 'drone': [], 'launches': [], 'cost': 745},
#               {'id': 47, 'truck': ['Source', 'C2', 'C9', 'Sink'], 'drone': [], 'launches': [], 'cost': 1586}]

node_ids = set()
node_depth = set()
local_estimates = {}
node_dict = {}
node_visit_list = []

last_node_vars = None
last_node_vals = None

M = 10000

# cost_scale = 0.01
cost_scale = 1

BSP_LB = 0  # lower bound of BSP

close_tolerance = 0.001


def update_arc_infos(location, term, travel_time, is_truck, truck_travel_times, truck_out_arcs, truck_in_arcs,
                     drone_travel_times, drone_out_arcs, drone_in_arcs):
    if is_truck:
        truck_travel_times[(location, term)] = travel_time
        truck_out_arcs[location].append(term)
        truck_in_arcs[term].append(location)
    else:
        drone_travel_times[(location, term)] = travel_time
        drone_out_arcs[location].append(term)
        drone_in_arcs[term].append(location)


def create_random_truck_drone_network():
    """
    Creates a random graph to represent the truck-drone routing problem.
    """
    random.seed(seed)
    truck_net = nx.DiGraph()
    # Generate node names
    depot_source = "Source"
    depot_sink = "Sink"
    customers = [f"C{i + 1}" for i in range(num_customers)]
    hubs = [f"H{i + 1}" for i in range(num_hubs)]

    # Add depot, customers, and hubs as nodes
    truck_net.add_node(depot_source)
    truck_net.add_node(depot_sink)
    for customer in customers:
        truck_net.add_node(customer)
    for hub in hubs:
        truck_net.add_node(hub)

    drone_net = truck_net.copy()

    all_nodes = [depot_source] + customers + hubs + [depot_sink]
    customer_indices = {}
    hub_indices = {}
    all_nodes_indices = {}
    for i in range(len(all_nodes)):
        node_name = all_nodes[i]
        all_nodes_indices[node_name] = i
        if node_name in customers:
            customer_indices[node_name] = i
        elif node_name in hubs:
            hub_indices[node_name] = i

    truck_out_arcs = {n: [] for n in all_nodes}
    truck_in_arcs = {n: [] for n in all_nodes}
    drone_out_arcs = {n: [] for n in all_nodes}
    drone_in_arcs = {n: [] for n in all_nodes}

    truck_travel_times = {}
    drone_travel_times = {}  # for round trip

    # generate truck arcs *************************************************************

    # Ensure that each customer and hub has a path from depot_source
    for location in customers + hubs:
        travel_time = random.randint(truck_min_t, truck_max_t)
        update_arc_infos(depot_source, location, travel_time, True, truck_travel_times, truck_out_arcs, truck_in_arcs,
                         drone_travel_times, drone_out_arcs, drone_in_arcs)
        truck_net.add_edge(depot_source, location)

    # Ensure that each customer and hub has a path to depot_sink
    for location in customers + hubs:
        travel_time = random.randint(truck_min_t, truck_max_t)
        update_arc_infos(location, depot_sink, travel_time, True, truck_travel_times, truck_out_arcs, truck_in_arcs,
                         drone_travel_times, drone_out_arcs, drone_in_arcs)
        truck_net.add_edge(location, depot_sink)

    # randomly generate arcs from hubs (truck)
    for location in hubs:
        # to customers
        for term in customers:
            if random.random() < 0.5:
                continue
            travel_time = random.randint(truck_min_t, truck_max_t)
            update_arc_infos(location, term, travel_time, True, truck_travel_times, truck_out_arcs, truck_in_arcs,
                             drone_travel_times, drone_out_arcs, drone_in_arcs)
            truck_net.add_edge(location, term)

    # randomly generate additional arcs for truck routes (between hubs customers)
    all_locations = customers + hubs
    for i in all_locations:
        for j in all_locations:
            if j == i:
                continue
            if random.random() > 0.5 and i not in hubs and j not in hubs:  # Randomly decide if an arc exists
                travel_time = random.randint(truck_min_t, truck_max_t)
                update_arc_infos(i, j, travel_time, True, truck_travel_times, truck_out_arcs, truck_in_arcs,
                                 drone_travel_times, drone_out_arcs, drone_in_arcs)
                truck_net.add_edge(i, j)

    # add a path visiting all customers
    for i in range(len(customers) - 1):
        n = customers[i]
        n_next = customers[i + 1]
        if (n, n_next) in truck_net.edges:
            continue
        travel_time = random.randint(truck_min_t, truck_max_t)
        update_arc_infos(n, n_next, travel_time, True, truck_travel_times, truck_out_arcs, truck_in_arcs,
                         drone_travel_times, drone_out_arcs, drone_in_arcs)
        truck_net.add_edge(n, n_next)

    # generate arcs for drones ********************************
    for i in hubs:
        for j in customers:
            travel_time = random.randint(drone_min_t, drone_max_t)
            update_arc_infos(i, j, travel_time, False, truck_travel_times, truck_out_arcs, truck_in_arcs,
                             drone_travel_times, drone_out_arcs, drone_in_arcs)
            drone_net.add_edge(i, j)

    truck_net = truck_net
    drone_net = drone_net

    a_lb = {n: 0 for n in all_nodes}

    demand_weights = {}
    for n_name in customers:
        n = all_nodes_indices[n_name]
        demand_weights[n] = random.uniform(demand_weight_min, demand_weight_max)
    for s_name in hubs:
        s = all_nodes_indices[s_name]
        demand_weights[s] = epsilon
    for n_name in [depot_source, depot_sink]:
        n = all_nodes_indices[n_name]
        demand_weights[n] = 0

    global net
    params = {
        "depot_source": depot_source,
        "depot_sink": depot_sink,
        "customers": customers,
        "hubs": hubs,
        "all_nodes": all_nodes,
        "all_nodes_indices": all_nodes_indices,
        "customer_indices": customer_indices,
        "hub_indices": hub_indices,
        "truck_net": truck_net,
        "drone_net": drone_net,
        "truck_out_arcs": truck_out_arcs,
        "truck_in_arcs": truck_in_arcs,
        "drone_out_arcs": drone_out_arcs,
        "drone_in_arcs": drone_in_arcs,
        "truck_travel_times": truck_travel_times,
        "drone_travel_times": drone_travel_times,
        "demand_weights": demand_weights
    }
    net = Network(num_trucks, num_drones_per_truck, a_lb, **params)
    sa = 0


def transform_network():
    """
    Turn the original network into the transformed network
    """
    global net, transformed_net
    transformed_net = TransformedNetwork(net)


def is_close(x, y):
    return True if abs(x - y) < close_tolerance else False


def is_integer(num):
    return abs(num - round(num)) <= close_tolerance


def is_subsequence(sub, full):
    """Check if 'sub' is a subsequence of 'full' while preserving order."""
    it = iter(full)
    return all(node in it for node in sub)


def is_sublist_ordered(sub, main):
    return bool(re.search(r'\b' + r', '.join(map(str, sub)) + r'\b', ', '.join(map(str, main))))


def get_latest_hub(network, path):
    """
    given a path, return the latest arrived hub
    """
    result = None
    idx = -1
    for node in reversed(path):
        if node in network.hubs:
            result = node.replace("_prime", "")
            idx = path.index(node)
            break
    return result, idx


def get_arrive_time(arrival_time, node_i, node_j, last_hub, sync_time, wait_time, network):
    """
    return the arrival time at node_j
    """

    if (node_i, node_j) in network.arcs_1:
        result = arrival_time + network.travel_times[(node_i, node_j)]
    elif (node_i, node_j) in network.arcs_5:
        if last_hub is None:
            travel_time = min(network.travel_times[(node_i, node_j)].values())
        else:
            travel_time = network.travel_times[(node_i, node_j)][last_hub]
        result = sync_time + wait_time + travel_time
    elif (node_i, node_j) in network.arcs_2:
        result = arrival_time
    # arcs 3 and 4
    else:
        if (node_i, node_j) in network.arcs_4:
            if last_hub is None:
                travel_time = min(network.travel_times[(node_i, node_j)].values())
            else:
                travel_time = network.travel_times[(node_i, node_j)][last_hub]
            result = sync_time + travel_time
        else:  # arcs 3
            result = sync_time + network.travel_times[(node_i, node_j)]
    return result


def elementary_path_to_route(path, idx, original_net, trans_net):
    """
    convert the elementary path derived from LSA to path stored in solution pool
    """
    truck_route = []
    launches = []
    drone_route = []
    for i in range(len(path) - 1):
        j = i + 1
        node_i = path[i]
        node_j = path[j]
        arc = (node_i, node_j)
        if node_i == original_net.depot_source:
            truck_route.append(node_i)
        # black arc
        if arc in trans_net.arcs_1:
            truck_route.append(node_j)
        # blue arc
        elif arc in trans_net.arcs_2:
            launches.append(node_i)
            drone_route.append(set())
        # purple arc
        elif arc in trans_net.arcs_5:
            truck_route.append(node_j)
        # orange and green arc
        else:
            drone_route[-1].add(node_j.replace("_prime", ""))
    route_key = "-".join(path)

    # calculate the route cost
    sync_time = 0
    arrival_time = 0
    wait_time = 0
    cost = 0
    last_hub = None
    for j in range(1, len(path)):
        node_pre = path[j - 1]
        node_j = path[j]
        # update arrival time
        arrival_time = get_arrive_time(arrival_time, node_pre, node_j, last_hub, sync_time, wait_time, trans_net)
        # update sync time
        if node_j in trans_net.hubs:
            sync_time = arrival_time
            last_hub = node_j.replace("_prime", "")
        # update wait time
        if (node_pre, node_j) in trans_net.arcs_3 or (node_pre, node_j) in trans_net.arcs_4:
            wait_time = max(wait_time, arrival_time - sync_time)
        else:
            wait_time = 0
        # update cost
        if node_j in trans_net.customers:
            cost += (arrival_time - trans_net.a_lb[node_j]) ** 2
        elif node_j == trans_net.depot_sink:
            cost += arrival_time

    route = {'id': idx, 'truck': truck_route, 'drone': drone_route, 'launches': launches, 'cost': cost}
    return route_key, route


def hashable_path_to_route(path, idx, trans_net):
    """
    convert the hashable path derived from LSA to path stored in solution pool
    """
    truck_route = path[0]
    drone_route = {key: value for key, value in path[-1]}
    launches = [key.replace("_prime", "") for key in drone_route.keys()]

    # calculate the route cost
    arrival_time = 0
    wait_time = 0
    cost = 0
    for j in range(1, len(truck_route)):
        node_pre = truck_route[j - 1]
        node_j = truck_route[j]
        arrival_time += trans_net.travel_times[(node_pre, node_j)] + wait_time
        wait_time = 0
        # update sync time
        if node_j in launches:
            sync_time = arrival_time
            drone_travel_times = []
            for drone_visit in drone_route[node_j]:
                drone_travel_time = trans_net.travel_times[(node_j + "_prime", drone_visit)]
                drone_travel_times.append(drone_travel_time)
                cost += (sync_time + drone_travel_time - trans_net.a_lb[drone_visit]) ** 2
            wait_time = max(drone_travel_times)
        elif node_j in trans_net.customers:
            cost += (arrival_time - trans_net.a_lb[node_j]) ** 2
        elif node_j == trans_net.depot_sink:
            cost += arrival_time

    route = {'id': idx, 'truck': truck_route, 'drone': drone_route, 'launches': launches, 'cost': cost}

    return path, route


def find_initial_routes(original_net):
    routes = []
    truck_travel_times = original_net.truck_travel_times
    a_lb = original_net.a_lb
    depot_source, depot_sink = original_net.depot_source, original_net.depot_sink
    customers = original_net.customers
    hubs = original_net.hubs

    # # Route that visits all customers
    # truck_route = [depot_source, customers[0]]
    # arrive_time = truck_travel_times[(depot_source, customers[0])]
    # cost = (arrive_time - a_lb[customers[0]]) ** 2
    # return_time = arrive_time  # Tracks total time back to depot
    # route_key = [str(depot_source), str(customers[0])]
    #
    # for i in range(len(customers) - 1):
    #     n, n_next = customers[i], customers[i + 1]
    #     travel_time = truck_travel_times[(n, n_next)]
    #     arrive_time += travel_time
    #     cost += (arrive_time - a_lb[n_next]) ** 2
    #     return_time += travel_time
    #     truck_route.append(n_next)
    #     route_key.append(str(n_next))
    #
    # return_time += truck_travel_times[(customers[-1], depot_sink)]
    # cost += return_time
    # truck_route.append(depot_sink)
    # route_key.append(str(depot_sink))
    # key = "-".join(route_key)
    #
    # route = {'id': len(routes), 'truck': truck_route, 'drone': [], 'launches': [], 'cost': cost}
    # routes.append((key, route))

    # Routes that visit only one node
    for n_name in customers:
        truck_route = [depot_source, n_name, depot_sink]
        arrive_time = truck_travel_times[(depot_source, n_name)]
        cost = (arrive_time - a_lb[n_name]) ** 2
        return_time = arrive_time + truck_travel_times[(n_name, depot_sink)]
        cost += return_time

        route = {'id': len(routes), 'truck': truck_route, 'drone': {}, 'launches': [], 'cost': cost}
        key = truck_drone_path_to_hashable(truck_route, {})
        routes.append((key, route))

    return routes


def cal_reduced_cost(route, duals, original_net):
    """
    Given a route and duals, calculate its reduced cost
    """

    route_cost, truck_route, drone_route = route['cost'], route['truck'], route['drone']
    reduced_cost = route_cost  # Initial reduced cost
    for n_idx, n_name in enumerate(original_net.customers):
        dual = duals["mu"][n_idx]  # Retrieve dual value
        if n_name in truck_route or any(n_name in d_set for d_set in drone_route):
            reduced_cost -= dual
    reduced_cost -= duals["cons_term"]  # Subtract nu
    return reduced_cost


def cal_label_cost_normal(trans_net, arrival_time_i, sync_time_i, wait_time_i, cost_i, partial_path, duals,
                          last_hub=None):
    """
    node_i is the first point on the partial path, calculate the reduced cost of this partial path
    """

    sync_time = sync_time_i
    arrival_time = arrival_time_i
    wait_time = wait_time_i
    full_cost = cost_i
    arrive_times = [arrival_time]
    for j in range(1, len(partial_path)):
        node_pre = partial_path[j - 1]
        node_j = partial_path[j]
        # update arrival time
        arrival_time = get_arrive_time(arrival_time, node_pre, node_j, last_hub, sync_time, wait_time, trans_net)
        # update sync time and last_hub
        if node_j in trans_net.hubs:
            sync_time = arrival_time
            last_hub = node_j.replace("_prime", "")
        # update cost
        if node_j in trans_net.customers:
            index = trans_net.customers.index(node_j.replace("_prime", ""))
            full_cost += (arrival_time - trans_net.a_lb[node_j]) ** 2 - duals["mu"][index]
        elif node_j == trans_net.depot_sink:
            full_cost += arrival_time
        # update waiting time
        if (node_pre, node_j) in trans_net.arcs_3 or (node_pre, node_j) in trans_net.arcs_4:
            wait_time = max(wait_time, arrival_time - sync_time)
        else:
            wait_time = 0
        arrive_times.append(arrival_time)

    return full_cost, arrive_times


def cal_label_cost_farkas(trans_net, partial_path, duals):
    """
    node_i is the first point on the partial path, calculate the reduced cost of this partial path
    """

    full_cost = 0
    for j in range(len(partial_path)):
        node_j = partial_path[j]
        # update cost
        if node_j in trans_net.customers:
            index = trans_net.customers.index(node_j.replace("_prime", ""))
            full_cost -= duals["mu"][index]

    return full_cost


def find_prefix(path, trans_net):
    prefix = [path[0]]
    node_m_next = None
    for i, (node, node_next) in enumerate(zip(path, path[1:])):
        if (node, node_next) in trans_net.arcs_5:
            node_m_next = node_next
            break
        prefix.append(node_next)
    return prefix, node_m_next


def truck_drone_path_to_hashable(truck_path, drone_flights):
    truck_path_tuple = tuple(truck_path)
    drone_flight_frozen = {k: frozenset(v) for k, v in drone_flights.items()}
    return truck_path_tuple, frozenset(drone_flight_frozen.items())


def is_route_subset(route_1, route_2):
    """
    check whether route_2 is subset of route_1
    """
    route_truck_1, route_drone_1 = route_1
    route_truck_2, route_drone_2 = route_2
    route_drone_1 = {k: set(v) for k, v in route_drone_1}
    route_drone_2 = {k: set(v) for k, v in route_drone_2}
    if not set(route_truck_2).issubset(set(route_truck_1)):
        return False
    for key, visit_set in route_drone_2.items():
        if key not in route_drone_1.keys() or route_drone_1[key] != visit_set:
            return False
    return True
