import inspect
import json
import math
import random
import re
from collections import defaultdict
from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
from sklearn.cluster import KMeans

from Network import Network
from TransformedNetwork import TransformedNetwork

sensitivity_analysis = True

# 1,2,3
seed = 1

# feasible combinations (2,5), (5, 15), (8,25),  (5,20)
num_trucks, num_customers = (2, 5)
custom_dist = "PS"  # customer distribution "PS", "PC", "mixed"
# num_drones_per_truck = 4
num_drones_per_truck = 3
num_hubs = 2

# arc generation
truck_arc_gen_prob = 0.2  # the probability of generating a truck arc
hub_arc_gen_prob = 0.5  # the probability of generating a truck arc that connects a hub

enable_DSS = False

enable_warm_start = True
enable_primal_heuristics = True
# enable_warm_start = False
# enable_primal_heuristics = False
# SR_num = 0  # the maximum number of SR inequalities, 10
# SR_num_each_run = 0
SR_num = 10  # the maximum number of SR inequalities, 10
SR_num_each_run = 2

num_threads = 4  # if this value exceeds the maximum number N of logic processors, change it to N

warm_start_MIP_no_improve = 30  # seconds
primal_heuristic_time = 30

M = 1000
close_tolerance = 0.001

max_node_label_num = 9999  # the maximum number of labels kept in a physical node

# cw = 0.25
cw = 9
alpha = 0.4
# cw = (1 - alpha) / alpha

max_run_time = 1800  # in seconds

node_lp_trace = []

net: Network = None  # the network object
transformed_net: TransformedNetwork = None  # the transformed network object

area_side = 14  # coordinates in [0,14]×[0,14] ⇒ 200 km²

# cost in dollars
truck_cost = 20
drone_cost_per_flight = 6
# drone_cost_per_flight = 0 * truck_cost

# travel time
truck_speed = 40
drone_speed = 40

# parcel weights
low_demand_customer_ratio = 0.5
low_demand_weight_mean = 1
low_demand_weight_std = 5
low_demand_weight_min = 0.5
# heavy parcels
high_demand_weight_mean = 10
high_demand_weight_std = 5
high_demand_weight_min = 6

truck_max_weight = 50
drone_max_weight = 2.3

# flight endurance
drone_endurance = 30

# for sub-tour elimination
epsilon = 0.001

locations = {}  # the position of nodes on the canvas

initial_routes = []

# solution visualization
solution_to_show = [
    {
        "id": 11,
        "truck": [
            "Source",
            "C4",
            "C3",
            "Sink"
        ],
        "drone": {},
        "cost": 46.353421514412354
    },
    {
        "id": 19,
        "truck": [
            "Source",
            "C2",
            "C5",
            "H2",
            "Sink"
        ],
        "drone": {
            "H2": [
                "C1_T"
            ]
        },
        "cost": 56.8971511692913
    }
]
truck_colors = [
    '#9aab4b',
    '#febd2b',
    '#d94f21',
    '#82cbec',
    '#a291c7',
    '#5f4e94',
    '#182b55',
    '#900c3f'
]

# logs
node_log_list = {}
num_of_arcs = -1
root_node_runtime = -1
BP_runtime = -1
solver_sol = None
BP_sol = None
sol_summary = []


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
    locations.clear()
    truck_net = nx.DiGraph()
    # Generate node names
    depot_source = "Source"
    depot_sink = "Sink"
    customers = [f"C{i + 1}" for i in range(num_customers)]
    hubs = [f"H{i + 1}" for i in range(num_hubs)]

    # sample customer & hub locations
    customer_locations = _sample_customers(num_customers, custom_dist)
    for i in range(len(customers)):
        locations[customers[i]] = tuple(customer_locations[i])
    # locations[depot_source] = tuple(customer_locations.mean(axis=0))
    locations[depot_source] = tuple(np.array([0, 0]))
    locations[depot_sink] = locations[depot_source]
    hubs_locations = choose_hubs_by_kmeans(
        [val for key, val in locations.items() if key not in [depot_source, depot_sink]], num_hubs)
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
    for node in customers + hubs:
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
                if random.random() <= truck_arc_gen_prob and (i in customers or j in customers):
                    travel_time = _gen_truck_travel_time(*locations[i], *locations[j])
                    update_arc_infos(i, j, travel_time, True, truck_travel_times, truck_out_arcs, truck_in_arcs,
                                     drone_travel_times, drone_out_arcs, drone_in_arcs)
                    truck_net.add_edge(i, j)

    # generate arcs for drones ********************************
    for i in hubs:
        for j in customers:
            travel_time = _gen_drone_travel_time(*locations[i], *locations[j])
            if 2 * travel_time > drone_endurance:  # the drone's battery is not sufficient
                continue
            update_arc_infos(i, j, travel_time, False, truck_travel_times, truck_out_arcs, truck_in_arcs,
                             drone_travel_times, drone_out_arcs, drone_in_arcs)
            drone_net.add_edge(i, j)

    demand_weights = {}
    # low demands
    low_dem_customers = random.sample(customers, int(low_demand_customer_ratio * len(customers)))
    low_demands = np.clip(
        np.random.normal(loc=low_demand_weight_mean, scale=low_demand_weight_std, size=len(low_dem_customers)),
        low_demand_weight_min, None)
    high_dem_customers = [cus for cus in customers if cus not in low_dem_customers]
    high_demands = np.clip(
        np.random.normal(loc=high_demand_weight_mean, scale=high_demand_weight_std,
                         size=num_customers - len(low_dem_customers)), high_demand_weight_min, None)
    for n_name in low_dem_customers:
        n = all_nodes_indices[n_name]
        demand_weights[n] = round(low_demands[low_dem_customers.index(n_name)], 2)
    for n_name in high_dem_customers:
        n = all_nodes_indices[n_name]
        demand_weights[n] = round(high_demands[high_dem_customers.index(n_name)], 2)
    for s_name in hubs:
        s = all_nodes_indices[s_name]
        demand_weights[s] = epsilon
    for n_name in [depot_source, depot_sink]:
        n = all_nodes_indices[n_name]
        demand_weights[n] = 0

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
    global net, cw
    net = Network(num_trucks, num_drones_per_truck, **params)
    print(f"cw is {cw}")


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


def choose_hubs_by_kmeans(cust, k, min_dist=0.5):
    """
    Given an array of customer coordinates cust (shape: n x 2),
    compute k hub coordinates using k-means clustering.
    Ensure no hub exactly overlaps a customer by applying a small random shift.
    """

    # fit k-means
    kmeans = KMeans(n_clusters=k, random_state=seed)
    kmeans.fit(cust)
    hubs = kmeans.cluster_centers_.copy()

    # for each hub, if it matches any customer (within min_dist), jitter it
    for i, hub in enumerate(hubs):
        # repeat until hub is at least min_dist away from every customer
        while np.any(np.linalg.norm(cust - hub, axis=1) < min_dist):
            hub += np.random.uniform(-min_dist, min_dist, size=2)
        hubs[i] = hub

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
    route['cost'], _ = route_get_cost(route, ori_net, trans_net)

    return route


def route_to_elem_path(route, trans_net: TransformedNetwork):
    truck_route = route['truck']
    drone_route = route['drone']
    elem_path = []
    for node in truck_route:
        elem_path.append(node)
        if node in trans_net.hubs and node in drone_route.keys():
            visits = sorted(drone_route[node], key=lambda s: int(s.split("_")[0][1:]))
            for visit in visits:
                elem_path.append(visit)
    return elem_path


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
    euc_dist = _get_euc_dist(xi, yi, xj, yj)
    t_drone = euc_dist / drone_speed * 60  # to minutes
    return t_drone


def _get_euc_dist(xi, yi, xj, yj):
    return math.hypot(xi - xj, yi - yj)


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
    labels = {
        n: "Depot" if n == "Source" else n
        for n in net.truck_net.nodes()
        if n != "Sink"
    }
    nx.draw_networkx_labels(
        net.truck_net,
        pos,
        labels=labels,
        font_size=10,
        font_weight="bold"
    )
    # nx.draw_networkx_labels(net.truck_net, pos, font_size=10, font_weight='bold')

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
            net.drone_net, pos, edgelist=[(u, v)], edge_color='red', arrowsize=15, width=1,
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
    counts = {}
    for node in element_path:
        if node in trans_net.customers_origin:
            cust = node
        elif node in trans_net.customers_prime:
            cust = node.replace("_T", "")
        else:
            continue
        counts[cust] = counts.get(cust, 0) + 1

    repeated = {cust for cust, freq in counts.items() if freq > 1}
    return repeated


def route_get_cost(route, ori: Network, trans: TransformedNetwork):
    arrive_times = {}
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
            arrive_times[node] = (truck_arr_time, trans.a_lb[node])
            if arrive_times[node][0] + close_tolerance < arrive_times[node][1]:
                sdas = 0
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
                    arrive_times[temp_visit] = (drone_arr_time, trans.a_lb[temp_visit])
                    if arrive_times[temp_visit][0] + close_tolerance < arrive_times[temp_visit][1]:
                        sdas = 0
                    wait_time = max(wait_time, ori.drone_travel_times[hub, temp_visit])
                cost += drone_cost_per_flight * len(drone_visits)
        pre_node = node
    return cost, arrive_times


def export_class_attributes(cls, file_path):
    """Write all JSON-serializable public attributes of *cls* to *file_path*."""
    attrs = {}
    for name, value in cls.__dict__.items():
        if name.startswith("__") or inspect.isroutine(value):
            continue
        try:
            json.dumps(value)  # test if value is JSON-serializable
            attrs[name] = value
        except TypeError:
            continue  # skip non-serializable attributes

    # ensure the folder exists
    Path(file_path).expanduser().parent.mkdir(parents=True, exist_ok=True)

    # write to JSON
    with open(file_path, "w", encoding="utf-8") as f:
        json.dump(attrs, f, indent=4, ensure_ascii=False)


def to_json_safe(obj):
    """Return *obj* converted to something json can write."""
    # atoms
    if isinstance(obj, (int, float, str, bool)) or obj is None:
        return obj

    # tuples, sets → list            (lists are valid JSON arrays)
    if isinstance(obj, (tuple, set)):
        return [to_json_safe(x) for x in obj]

    # NumPy array → list
    if isinstance(obj, np.ndarray):
        return obj.tolist()

    # dict: convert keys to strings, values recursively
    if isinstance(obj, dict):
        new_dict = {}
        for k, v in obj.items():
            # keys that are already str stay as they are, else stringify
            if isinstance(k, str):
                key = k
            else:
                key = json.dumps(to_json_safe(k))  # e.g. tuple → "[\"Source\", …]"
            new_dict[key] = to_json_safe(v)
        return new_dict

    # fallback: stringify anything unknown
    return str(obj)


def save_complex_dict(data, file_path):
    cleaned = to_json_safe(data)

    path = Path(file_path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)

    with path.open("w", encoding="utf-8") as f:
        json.dump(cleaned, f, indent=4, ensure_ascii=False)


def visualize_solution():
    positions = {
        node: locations[node]
        for node in net.all_nodes
    }
    fig, ax = plt.subplots()
    plt.sca(ax)

    truck_num = len(solution_to_show)

    depots = [transformed_net.depot_source, transformed_net.depot_sink]
    customers = transformed_net.customers_origin
    hubs = transformed_net.hubs

    node_size = 800
    customer_node_size = 800
    label_size = 14
    arc_size = 2

    nx.draw_networkx_nodes(
        net.truck_net, positions,
        nodelist=depots,
        node_shape='*',
        node_size=node_size,
        node_color='none',
        edgecolors='#9f1b25',
        linewidths=2
    )

    nx.draw_networkx_nodes(
        net.truck_net, positions,
        nodelist=customers,
        node_shape='o',  # circle
        node_size=customer_node_size,
        node_color='none',
        edgecolors='#202228',
        linewidths=2
    )

    nx.draw_networkx_nodes(
        net.truck_net, positions,
        nodelist=hubs,
        node_shape='s',
        node_size=node_size,
        node_color='none',
        edgecolors='#9f1b25',
        linewidths=2
    )

    labels = {
        n: "" if n == "Source" else n
        for n in net.truck_net.nodes()
        if n != "Sink"
    }
    nx.draw_networkx_labels(
        net.truck_net,
        positions,
        labels=labels,
        font_size=label_size
    )

    for k in range(truck_num):
        route = solution_to_show[k]
        truck_path = route['truck']
        for u, v in zip(truck_path[:], truck_path[1:]):
            nx.draw_networkx_edges(
                net.truck_net, positions, edgelist=[(u, v)], edge_color=truck_colors[k], width=arc_size,
                arrows=True, arrowsize=20, arrowstyle='-|>', connectionstyle='arc3,rad=0.4'
            )
        for hub, customers in route['drone'].items():
            for customer in customers:
                nx.draw_networkx_edges(
                    net.truck_net, positions, edgelist=[(hub, customer.replace('_T', ''))], edge_color=truck_colors[k],
                    width=arc_size, arrows=True, arrowsize=20, style='dashed', arrowstyle='-|>',
                    connectionstyle='arc3,rad=0.4'
                )

    plt.axis('off')  # Turn off the axis
    plt.tight_layout()
    plt.savefig("solution_visual.pdf", dpi=300)


def plot_solution_waiting_times(original_net: Network, trans_net: TransformedNetwork):
    arrive_time_dict = {}
    for route in solution_to_show:
        _, arrive_times = route_get_cost(route, original_net, trans_net)
        arrive_time_dict.update(arrive_times)
    categories = trans_net.customers_origin
    arrive_time_values = [arrive_time_dict[cus][0] for cus in categories]
    lower_bounds = [arrive_time_dict[cus][1] for cus in categories]
    # x‐positions and bar width
    x = np.arange(len(categories))
    width = 0.6

    colors = ['#74c239', '#c5c5c5']
    fig, ax = plt.subplots()
    bars1 = ax.bar(x, arrive_time_values, width=width,
                   label='Actual arrival time', alpha=1, color=colors[0])
    bars2 = ax.bar(x, lower_bounds, width=width,
                   label='Earliest arrival time', alpha=1, color=colors[1])

    label_size = ax.yaxis.label.get_size()
    # annot_size = label_size - 6
    annot_size = label_size
    # annotate each bar
    for bar in bars2:
        h = bar.get_height()
        ax.text(
            bar.get_x() + bar.get_width() / 2,
            h - 0.1,
            f'{h:.1f}',
            ha='center',
            va='top',  # align the text’s top at that y
            color='black',
            fontsize=annot_size
        )

    for bar in bars1:
        h = bar.get_height()
        ax.text(
            bar.get_x() + bar.get_width() / 2,
            h + 0.2,
            f'{h:.1f}',
            ha='center',
            va='bottom',  # align the text’s bottom at that y,
            color='black',
            fontsize=annot_size
        )

    # give 10% headroom so labels are never cut off
    all_heights = [b.get_height() for b in bars1] + [b.get_height() for b in bars2]
    ax.set_ylim(0, max(all_heights) * 1.10)

    # labels and legend
    ax.set_xticks(x)
    ax.set_xticklabels(categories)
    ax.set_xlabel('Customer name')
    ax.set_ylabel('Arrival time (min)')
    ax.legend(loc='upper right', frameon=True, framealpha=1)  # no box around legend
    fig.tight_layout()

    # save as PDF to include in your INFORMS submission
    fig.savefig('arrival_times.pdf')
    sdas = 0


def plot_set_style():
    # Informs‐style settings
    mpl.rcParams['font.family'] = 'serif'
    mpl.rcParams['font.serif'] = ['Times New Roman']
    # mpl.rcParams['text.usetex'] = True
    mpl.rcParams['font.size'] = 20  # base font size for text
    mpl.rcParams['axes.labelsize'] = 20  # x- and y-axis labels
    mpl.rcParams['xtick.labelsize'] = 18  # x-tick labels
    mpl.rcParams['ytick.labelsize'] = 18  # y-tick labels
    mpl.rcParams['legend.fontsize'] = 18  # legend text
    mpl.rcParams['axes.linewidth'] = 0.5
    mpl.rcParams['lines.linewidth'] = 0.8
    mpl.rcParams['savefig.dpi'] = 300
    mpl.rcParams['savefig.format'] = 'pdf'
    # mpl.rcParams['figure.figsize'] = (10, 5)  # width = 10", height = 5"
    # mpl.rcParams['figure.figsize'] = (15, 5)  # width = 10", height = 5"
    mpl.rcParams['figure.autolayout'] = True


def get_SA_infos(BP_solutions, trans_net: TransformedNetwork):
    # get the information for the sensitivity analysis
    truck_customer_visit_num = 0
    truck_customer_visits = {}
    drone_launch_times = {key: 0 for key in trans_net.hubs}
    total_drone_flights = 0
    total_cost = 0
    num_truck_dispatched = 0
    delay_times = {}
    for solution in BP_solutions:
        # get the delay info
        _, arrive_times = route_get_cost(solution, net, transformed_net)
        for cust, vals in arrive_times.items():
            arr_t, arr_t_lb = vals
            delay_times[cust] = float(arr_t - arr_t_lb)

        truck_route = solution['truck']
        total_cost += solution['cost']
        truck_customer_visit = []
        if len(truck_route) > 0:
            num_truck_dispatched += 1
        for node in truck_route:
            if node in trans_net.customers_origin:
                truck_customer_visit.append(node)
        if len(truck_customer_visit) > 0:
            truck_customer_visit_num += len(truck_customer_visit)
            truck_customer_visits[solution['id']] = truck_customer_visit
        drone_routes = solution['drone']
        for hub, visits in drone_routes.items():
            drone_launch_times[hub] += len(visits)

    f_cost = num_truck_dispatched * truck_cost + drone_cost_per_flight * total_drone_flights
    f_time = (total_cost - f_cost) / cw
    drone_flight_num = sum(drone_launch_times.values())
    drone_ratio = drone_flight_num / num_customers * 100

    return num_truck_dispatched, truck_customer_visit_num, drone_flight_num, drone_ratio, f_time, f_cost, delay_times


def read_sol_get_delay_times():
    # point this at your “debug” folder
    base_path = Path('debug')

    # match names like "alpha 0.1" or "alpha0.1"
    alpha_re = re.compile(r'alpha\s*(\d*\.?\d+)')
    seed_re = re.compile(r'param-seed=(\d+)')

    # map each alpha (as float) to a list of BP_sol values
    bp_by_alpha = defaultdict(list)

    # loop over seed folders
    for seed_dir in base_path.iterdir():
        if not seed_dir.is_dir() or not seed_dir.name.startswith('seed'):
            continue

        # loop over alpha subfolders
        for alpha_dir in seed_dir.iterdir():
            if not alpha_dir.is_dir():
                continue
            m = alpha_re.fullmatch(alpha_dir.name)
            if not m:
                continue

            alpha_val = float(m.group(1))
            # find every JSON whose name starts with param-seed=
            for json_file in alpha_dir.glob('param-seed=*.json'):
                # extract seed as int
                mseed = seed_re.match(json_file.stem)
                if not mseed:
                    continue
                temp_seed = int(mseed.group(1))

                try:
                    data = json.loads(json_file.read_text())
                except Exception as e:
                    # skip files that fail to load
                    continue
                bp_sol = data.get('BP_sol')
                bp_by_alpha[alpha_val].append((temp_seed, bp_sol))

    delays_by_alpha = defaultdict(list)
    for alpha, solutions in bp_by_alpha.items():
        for temp_seed, solution in solutions:
            global seed, net, transformed_net
            seed = temp_seed
            create_original_network()
            transform_network()
            for route in solution:
                _, arrive_times = route_get_cost(route, net, transformed_net)
                delays = [float(arrive_times[cus][0] - arrive_times[cus][1]) for cus in arrive_times.keys()]
                delays_by_alpha[alpha].extend(delays)
    for alpha, delays in delays_by_alpha.items():
        print(delays)
        print()
    for alpha, sols in sorted(bp_by_alpha.items()):
        print(f'alpha = {alpha!s}: found {len(sols)} entries →', sols)


def update_inputs(_seed, _num_trucks, _num_customers, _custom_dist, _drone_num, _cw):
    global seed, num_trucks, num_customers, custom_dist, num_drones_per_truck, cw
    seed = _seed
    num_trucks = _num_trucks
    num_customers = _num_customers
    custom_dist = _custom_dist
    num_drones_per_truck = _drone_num
    cw = _cw
