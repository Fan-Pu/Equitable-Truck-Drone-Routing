import math
import random
import re
from line_profiler import LineProfiler
import networkx as nx
from collections import defaultdict

from streamlit import selectbox

from Network import Network
from TransformedNetwork import TransformedNetwork
from sortedcontainers import SortedSet
import numpy as np
import matplotlib

matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans
from collections import Counter

M = 10000
close_tolerance = 0.001

enable_DSS = False

root_node_max_col_num = 999

cw = 0.25
# cw = 1

arc_gen_prob = 0.1  # the probability of generating a truck arc
hub_arc_gen_prob = 0.5  # the probability of generating a truck arc that connects a hub
drone_arc_gen_prob = 0.2  # the probability of generating a drone arc

columns_list = []

SR_num = 0  # the maximum number of SR inequalities
SR_num_each_run = 0

node_lp_trace = []

allow_extend_checks_passed = 0

forward_dominance_num = 0

lp = LineProfiler()

seed = 2024

net: Network = None  # the network object
transformed_net: TransformedNetwork = None  # the transformed network object

num_trucks = 4  # 2, 4, 6
num_customers = 15  # 5, 25, 50

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

truck_max_weight = 50
drone_max_weight = 3

# flight endurance
drone_endurance = 30

# for sub-tour elimination
epsilon = 0.1

num_drones_per_truck = 4

# num_hubs = int(math.floor(num_customers / 5))
num_hubs = 2

locations = {}

initial_routes = []

test_solutions = [
    ['Source', 'C6', 'C7', 'H2', 'Sink'],
    ['Source', 'C13', 'H1', 'C8_T', 'C9_T', 'C15_T', 'C4', 'H2', 'Sink'],
    ['Source', 'C11', 'C5', 'C14', 'H2', 'C3', 'C10', 'Sink'],
    ['Source', 'C2', 'C12', 'H1', 'C1', 'Sink']
]


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
            if i in hubs or j in hubs:
                if random.random() <= hub_arc_gen_prob and (i in customers or j in customers):
                    travel_time = _gen_truck_travel_time(*locations[i], *locations[j])
                    update_arc_infos(i, j, travel_time, True, truck_travel_times, truck_out_arcs, truck_in_arcs,
                                     drone_travel_times, drone_out_arcs, drone_in_arcs)
                    truck_net.add_edge(i, j)
            else:
                if random.random() <= arc_gen_prob and (i in customers or j in customers):
                    travel_time = _gen_truck_travel_time(*locations[i], *locations[j])
                    update_arc_infos(i, j, travel_time, True, truck_travel_times, truck_out_arcs, truck_in_arcs,
                                     drone_travel_times, drone_out_arcs, drone_in_arcs)
                    truck_net.add_edge(i, j)

    # generate arcs for drones ********************************
    for i in hubs:
        for j in customers:
            if random.random() <= drone_arc_gen_prob:
                travel_time = _gen_drone_travel_time(*locations[i], *locations[j])
                if 2 * travel_time > drone_endurance:  # the drone's battery is not sufficient
                    continue
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


def is_subsequence(sub, full_index_map):
    """
    Return True if `sub` is a subsequence of the original `full` list.
    Because there are no duplicates, we can simply compare indices.
    """
    # Keep track of the index in `full` that we matched most recently.
    prev_index = -1
    for value in sub:
        # If `value` does not appear in `full`, it cannot match.
        if value not in full_index_map:
            return False
        current_index = full_index_map[value]
        # If indices do not increase, the order is wrong.
        if current_index <= prev_index:
            return False
        prev_index = current_index
    return True


def get_arrive_time(arrival_time, node_i, node_j, last_hub, sync_time, wait_time, network: TransformedNetwork):
    """
    return the arrival time at node_j
    """

    arc = (node_i, node_j)
    if arc in network.arcs_ori:
        result = arrival_time + network.travel_times[arc]
    elif arc in network.arcs_cpc:
        travel_time = network.travel_times[arc][last_hub]
        result = sync_time + wait_time + travel_time
    # arcs_cpcp and arcs_SC'
    else:
        if arc in network.arcs_cpcp:
            result = sync_time + network.travel_times[arc][last_hub]
        else:  # arcs_SC'
            result = sync_time + network.travel_times[arc]
    return result


def elem_path_to_route(path, idx, ori_net: Network, trans_net: TransformedNetwork):
    """
    convert the elementary path derived from LSA to path stored in solution pool
    """

    truck_route = []
    drone_route = defaultdict(list)
    launches = []
    for node in path:
        if node in trans_net.hubs:
            launches.append(node)
        if node in trans_net.customers_prime:
            drone_route[launches[-1]].append(node)
        else:
            truck_route.append(node)

    route = {'id': idx, 'truck': truck_route, 'drone': drone_route, 'cost': 0}
    route['cost'] = route_get_cost(route, ori_net, trans_net)

    return route


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
        return_time = arrive_time + truck_travel_times[(n_name, depot_sink)]
        cost = cw * (arrive_time - a_lb[n_name]) ** 2 + truck_cost
        cost += cw * return_time
        route = {'id': len(routes), 'truck': truck_route, 'drone': {}, 'cost': cost}

        key = tuple(truck_route)
        routes.append((key, route))
        element_paths[key] = truck_route

    return routes, element_paths


def get_route_customer_visits(route):
    """
    given an element path, returns the customer nodes visited by the path
    """

    result = []
    for node in route['truck']:
        node = node.replace("_T", "")
        if node in transformed_net.customers_origin:
            result.append(node)
    for hub, nodes in route['drone'].items():
        result.extend(node.replace("_T", "") for node in nodes)
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
            if i + "_T" in node_set:
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


def get_revisit(element_path, trans_net: TransformedNetwork):
    counts = Counter()
    for node in element_path:
        if node in trans_net.customers_origin:
            counts[node] += 1
        elif node in trans_net.customers_prime:
            counts[node.replace("_T", "")] += 1
    repeated = {cust for cust, freq in counts.items() if freq > 1}
    return repeated


def route_get_cost(route, ori: Network, trans: TransformedNetwork):
    truck_route = route['truck']
    if len(truck_route) == 0:
        return 0
    drone_route = route['drone']
    cost = truck_cost
    truck_arr_time = 0
    pre_node = 'Source'
    wait_time = 0
    for node in truck_route[1:]:
        truck_arr_time += ori.truck_travel_times[(pre_node, node)] + wait_time
        wait_time = 0
        if node in ori.customers:
            cost += cw * (truck_arr_time - trans.a_lb[node]) ** 2
        elif node == trans.depot_sink:
            cost += cw * truck_arr_time
        elif node in ori.hubs:
            hub = node
            if hub in drone_route.keys():
                drone_visits = drone_route[hub]
                for visit in drone_visits:
                    temp_visit = visit.replace("_T", "")
                    drone_arr_time = truck_arr_time + ori.drone_travel_times[(hub, temp_visit)]
                    cost += cw * (drone_arr_time - trans.a_lb[temp_visit]) ** 2
                    wait_time = max(wait_time, ori.drone_travel_times[hub, temp_visit])
                cost += drone_cost_per_flight * len(drone_visits)
        pre_node = node
    return cost
