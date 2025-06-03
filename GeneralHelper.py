import math
import random
import re
from line_profiler import LineProfiler
import networkx as nx
from collections import defaultdict
from Network import Network
from TransformedNetwork import TransformedNetwork
from sortedcontainers import SortedSet
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans

M = 10000
close_tolerance = 0.001
# cost_scale = 0.01

max_columns_num = 9900

cw = 0.25

arc_gen_prob = 0.1  # the probability of generating an arc

test_route_list = []

columns_list = []

SR_num = 0  # the maximum number of SR inequalities
SR_num_each_run = 0

node_lp_trace = []

max_runtime = 0
which_node_col_num = 0

max_time = 0
max_num = 0
max_id = 0

allow_extend_checks_passed = 0

forward_dominance_num = 0

lp = LineProfiler()

seed = 2024

net: Network = None  # the network object
transformed_net: TransformedNetwork = None  # the transformed network object

num_trucks = 2  # 2, 4, 6
num_customers = 25  # 5, 25, 50

custom_dist = "PS"  # customer distribution "PS", "PC", "mixed"

area_side = 14  # coordinates in [0,14]×[0,14] ⇒ 200 km²

# cost in dollars
truck_cost = 20
drone_cost_per_flight = 6

# travel time
truck_speed = 40
drone_speed = 40

# parcel weights
demand_weight_mean = 1
demand_weight_std = 5
demand_weight_min = 0.5

truck_max_weight = 450
drone_max_weight = 2.3

# flight endurance
drone_endurance = 30

# for sub-tour elimination
epsilon = 0.1

num_drones_per_truck = 4

num_hubs = int(math.floor(num_customers / 5))

locations = {}


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
            if random.random() <= arc_gen_prob:
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
    net = Network(num_trucks, num_drones_per_truck, **params)
    sa = 0


def create_original_network():
    random.seed(seed)
    np.random.seed(seed)
    truck_net = nx.DiGraph()
    # Generate node names
    depot_source = "Source"
    depot_sink = "Sink"
    customers = [f"C{i + 1}" for i in range(num_customers)]
    hubs = [f"H{i + 1}" for i in range(num_hubs)]

    # 1) sample customer & hub locations
    customer_locations = _sample_customers(num_customers, custom_dist)
    for i in range(len(customers)):
        locations[customers[i]] = tuple(customer_locations[i])
    # locations[depot_source] = tuple(customer_locations.mean(axis=0))
    locations[depot_source] = tuple(np.array([0, 0]))
    locations[depot_sink] = locations[depot_source]
    hubs_locations = choose_hubs_by_kmeans(np.array(list(locations.values())), num_hubs)
    for i in range(len(hubs)):
        locations[hubs[i]] = tuple(hubs_locations[i])

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
    for node in customers + hubs:  # ensure that each hub/customer has a path from depot_source and a path to depot_sink
        travel_time = _gen_truck_travel_time(*locations[depot_source], *locations[node])
        # from source
        update_arc_infos(depot_source, node, travel_time, True, truck_travel_times, truck_out_arcs,
                         truck_in_arcs, drone_travel_times, drone_out_arcs, drone_in_arcs)
        truck_net.add_edge(depot_source, node)
        # to sink
        update_arc_infos(node, depot_sink, travel_time, True, truck_travel_times, truck_out_arcs,
                         truck_in_arcs, drone_travel_times, drone_out_arcs, drone_in_arcs)
        truck_net.add_edge(node, depot_sink)

    # randomly generate additional arcs for truck routes (between hubs and customers)
    all_locations = customers + hubs
    for i in all_locations:
        for j in all_locations:
            if j == i:
                continue
            if random.random() <= arc_gen_prob and (i in customers or j in customers):
                travel_time = _gen_truck_travel_time(*locations[i], *locations[j])
                update_arc_infos(i, j, travel_time, True, truck_travel_times, truck_out_arcs, truck_in_arcs,
                                 drone_travel_times, drone_out_arcs, drone_in_arcs)
                truck_net.add_edge(i, j)

    # generate arcs for drones ********************************
    for i in hubs:
        for j in customers:
            if random.random() <= arc_gen_prob:
                travel_time = _gen_drone_travel_time(*locations[i], *locations[j])
                update_arc_infos(i, j, travel_time, False, truck_travel_times, truck_out_arcs, truck_in_arcs,
                                 drone_travel_times, drone_out_arcs, drone_in_arcs)
                drone_net.add_edge(i, j)

    demand_weights = {}
    temp_demands = np.clip(np.random.normal(loc=demand_weight_mean, scale=demand_weight_std, size=num_customers),
                           demand_weight_min, None)
    for n_name in customers:
        n = all_nodes_indices[n_name]
        demand_weights[n] = round(temp_demands[customers.index(n_name)], 2)
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
    net = Network(num_trucks, num_drones_per_truck, **params)


def _sample_customers(n, dist):
    if dist == 'PS':
        # purely sparse: uniform over the square
        return np.random.uniform(0, area_side, size=(n, 2))

    elif dist == 'PC':
        # purely clustered: generate cluster centers, then normal around them
        k = max(1, n // 5)
        centers = np.random.uniform(0, area_side, size=(k, 2))
        pts = []
        for i in range(n):
            center = centers[i % k]
            pt = np.random.normal(loc=center, scale=area_side * 0.05, size=2)
            pts.append(np.clip(pt, 0, area_side))
        return np.vstack(pts)

    elif dist == 'mixed':
        # half uniform, half clustered
        n_clust = n // 2
        n_unif = n - n_clust
        unif_pts = _sample_customers(n_unif, 'PS')
        cluster_pts = _sample_customers(n_clust, 'PC')
        return np.vstack((unif_pts, cluster_pts))

    else:
        raise ValueError(f'Unknown distribution type: {dist}')


def choose_hubs_by_kmeans(cust, k):
    """
    Given an array of customer coordinates cust (shape: n x 2),
    compute k hub coordinates using k-means clustering.
    """

    # initialize and fit k-means
    kmeans = KMeans(n_clusters=k, random_state=seed)
    kmeans.fit(cust)

    # centroids is an array of shape (k, 2)
    hubs = kmeans.cluster_centers_
    return hubs


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
    i = 0
    n = len(sub)
    if n == 0:
        return True
    for x in full:
        if x == sub[i]:
            i += 1
            if i == n:
                return True
    return False


def get_arrive_time(arrival_time, node_i, node_j, last_hub, sync_time, wait_time, network: TransformedNetwork):
    """
    return the arrival time at node_j
    """
    arc = (node_i, node_j)
    if arc in network.arcs_ori:
        result = arrival_time + network.travel_times[arc]
    elif arc in network.arcs_cpc:
        if last_hub is None:
            travel_time = min(network.travel_times[arc].values())
        else:
            travel_time = network.travel_times[arc][last_hub]
        result = sync_time + wait_time + travel_time
    # arcs_cpcp and arcs_SC'
    else:
        if arc in network.arcs_cpcp:
            result = sync_time + network.travel_times[arc][last_hub]
        else:  # arcs_SC'
            result = sync_time + network.travel_times[arc]
    return result


def hashable_path_to_route(path, idx, trans_net):
    """
    convert the hashable path derived from LSA to path stored in solution pool
    """
    truck_route = path[0]
    drone_route = {key: value for key, value in path[-1]}
    launches = [key for key in drone_route.keys()]

    # calculate the route cost
    arrival_time = 0
    wait_time = 0
    cost = truck_cost
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
                cost += drone_cost_per_flight
                drone_travel_time = trans_net.travel_times[(node_j, drone_visit)]
                drone_travel_times.append(drone_travel_time)
                cost += (sync_time + drone_travel_time - trans_net.a_lb[drone_visit.replace("_prime", "")]) ** 2
            if len(drone_travel_times) > 0:
                wait_time = max(drone_travel_times)
        elif node_j in trans_net.customers:
            cost += (arrival_time - trans_net.a_lb[node_j.replace("_prime", "")]) ** 2
        elif node_j == trans_net.depot_sink:
            cost += arrival_time

    route = {'id': idx, 'truck': truck_route, 'drone': drone_route, 'launches': launches, 'cost': cost}

    return path, route


def find_initial_routes(original_net, trans_net):
    routes = []
    element_paths = {}
    truck_travel_times = original_net.truck_travel_times
    a_lb = trans_net.a_lb
    depot_source, depot_sink = original_net.depot_source, original_net.depot_sink
    customers = original_net.customers

    # Routes that visit only one node
    for n_name in customers:
        truck_route = [depot_source, n_name, depot_sink]
        arrive_time = truck_travel_times[(depot_source, n_name)]
        cost = (arrive_time - a_lb[n_name]) ** 2 + truck_cost
        return_time = arrive_time + truck_travel_times[(n_name, depot_sink)]
        cost += return_time

        route = {'id': len(routes), 'truck': truck_route, 'drone': {}, 'launches': [], 'cost': cost}
        key = truck_drone_path_to_hashable(truck_route, {})
        routes.append((key, route))
        element_paths[key] = truck_route

    return routes, element_paths


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


def cal_label_cost_normal(trans_net: TransformedNetwork, arrival_time_i, sync_time_i, wait_time_i, cost_i, partial_path,
                          duals, last_hub=None):
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
        if (node_pre, node_j) in trans_net.arcs_scp or (node_pre, node_j) in trans_net.arcs_cpcp:
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


def find_prefix(path, trans_net: TransformedNetwork):
    prefix = [path[0]]
    node_m_next = None
    for i, (node, node_next) in enumerate(zip(path, path[1:])):
        arc = (node, node_next)
        if arc in trans_net.arcs_cpc:
            node_m_next = node_next
            break
        prefix.append(node_next)
    return prefix, node_m_next


def truck_drone_path_to_hashable(truck_path, drone_flights):
    truck_path_tuple = tuple(truck_path)
    drone_flight_frozen = {k: frozenset(v) for k, v in drone_flights.items() if len(v) > 0}
    return truck_path_tuple, frozenset(drone_flight_frozen.items())


def get_route_customer_visits(route):
    """
    given an element path, returns the customer nodes visited by the path
    """

    result = []
    for node in route['truck']:
        node = node.replace("_prime", "")
        if node in transformed_net.customers_origin:
            result.append(node)
    for hub, nodes in route['drone'].items():
        result.extend(node.replace("_prime", "") for node in nodes)
    return result


def if_route_travel_arc(route, i, j, original_net):
    """
    check whether the given route travels the arc (i,j)
    :param route:
    :param i:
    :param j:
    :param original_net:
    :return:
    """
    if (i, j) in zip(route['truck'], route['truck'][1:]):
        return True
    elif i in original_net.hubs and i in route['drone'].keys() and j in route['drone'][i]:
        return True
    return False


def if_route_visit_node(route, i):
    """
    check whether a route visit the given node, customer and customer' are considered the same
    """

    if i in route['truck']:
        return True
    else:
        for key, node_set in route['drone'].items():
            if i + "_prime" in node_set:
                return True
    return False


def if_path_travel_arc(path: list, arc):
    """
    given an elementary path, check whether it travels the arc
    """

    i, j = arc
    return any(x == i and y == j for x, y in zip(path, path[1:]))


def _gen_truck_travel_time(xi, yi, xj, yj):
    man_dist = abs(xi - xj) + abs(yi - yj)  # km
    t_truck = man_dist / truck_speed * 60  # to minutes
    return t_truck


def _gen_drone_travel_time(xi, yi, xj, yj):
    euc_dist = math.hypot(xi - xj, yi - yj)  # km
    t_drone = euc_dist / drone_speed * 60  # to minutes
    return t_drone


def visualize_network():
    # pos = nx.spring_layout(self.truck_net, seed=self.seed)
    pos = {
        node: locations[node]
        for node in net.all_nodes
    }
    fig, ax = plt.subplots()
    plt.sca(ax)

    # Color mapping: Different colors for depot, customers, and hubs
    node_colors = []
    for node in net.truck_net.nodes():
        if node == net.depot_source or node == net.depot_sink:
            node_colors.append('orange')  # Red for depot
        elif node in net.customers:
            node_colors.append('lightblue')  # Yellow for customers
        elif node in net.hubs:
            node_colors.append('lightgreen')  # Green for hubs
        else:
            node_colors.append('gray')  # Default color for others (if any)

    # Draw nodes
    nx.draw_networkx_nodes(net.truck_net, pos, node_color=node_colors, node_size=500)
    nx.draw_networkx_labels(net.truck_net, pos, font_size=10, font_weight='bold')

    # Draw curved edges with varying curvature to avoid overlaps
    edge_curvatures = [0.2, 0.4, -0.2, -0.4]  # Example curvatures
    for i, (u, v) in enumerate(net.truck_net.edges()):
        curvature = edge_curvatures[i % len(edge_curvatures)]  # Cycle through curvatures
        nx.draw_networkx_edges(
            net.truck_net, pos, edgelist=[(u, v)], edge_color='gray', arrowsize=15, width=1,
            connectionstyle=f"arc3,rad={curvature}"
        )
    for i, (u, v) in enumerate(net.drone_net.edges()):
        curvature = edge_curvatures[i % len(edge_curvatures)]  # Cycle through curvatures
        nx.draw_networkx_edges(
            net.drone_net, pos, edgelist=[(u, v)], edge_color='pink', arrowsize=15, width=1,
            connectionstyle=f"arc3,rad={curvature}"
        )

    # # Create custom labels for nodes
    # node_labels = {}
    # for node in net.truck_net.nodes():
    #     if node in net.customers:
    #         n = net.all_nodes_indices[node]
    #         node_labels[node] = f"w:{net.demand_weights[n]:.2f}"
    #
    # label_pos = {node: (x, y + 0.05) for node, (x, y) in pos.items()}  # Adjust 0.05 to control the offset
    # # Draw the node labels
    # nx.draw_networkx_labels(net.truck_net, label_pos, labels=node_labels, font_size=10)

    plt.axis('off')  # Turn off the axis
    plt.tight_layout()
    plt.show()
