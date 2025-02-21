import random
import re

import networkx as nx

from Network import Network
from TransformedNetwork import TransformedNetwork

test_path = ["H2", "H2_prime", "C1_prime", "Sink"]

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

num_customers = 3
num_hubs = 2
num_trucks = 2
num_drones_per_truck = 2

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
    for node in reversed(path):
        if node in network.hubs:
            result = node.replace("_prime", "")
            break
    return result


def get_arrive_time(arrival_time, node_i, node_j, last_hub, sync_time, network):
    """
    return the arrival time at node_j
    """
    if (node_i, node_j) in network.arcs_1:
        arrival_time += network.travel_times[(node_i, node_j)]
    elif (node_i, node_j) in network.arcs_5:
        if last_hub is None:
            travel_time = min(network.travel_times[(node_i, node_j)].values())
        else:
            travel_time = network.travel_times[(node_i, node_j)][last_hub]
        arrival_time += travel_time
    elif (node_i, node_j) in network.arcs_2:
        pass
    # arcs 2 and 4
    else:
        if (node_i, node_j) in network.arcs_4:
            if last_hub is None:
                travel_time = min(network.travel_times[(node_i, node_j)].values())
            else:
                travel_time = network.travel_times[(node_i, node_j)][last_hub]
            arrival_time += travel_time
        else:
            arrival_time = sync_time + network.travel_times[(node_i, node_j)]
    return arrival_time
