import heapq
import os
import networkx as nx
import osmnx as ox
from geopy.distance import geodesic
import math

# Where our map data is from
MAP_LOCATION = "Dehradun, Uttarakhand, India"

# Our street graph
our_map_graph = None

def load_our_map():
    # Load map data or download it
    global our_map_graph
    map_file = f"{MAP_LOCATION.replace(' ', '_').replace(',', '')}_network.graphml"

    if os.path.exists(map_file):
        print(f"Loading map for {MAP_LOCATION} from file.")
        our_map_graph = ox.load_graphml(map_file)
        # Add road details
        our_map_graph = add_road_details(our_map_graph)
        ox.save_graphml(our_map_graph, map_file) # Save if new details added

    else:
        print(f"Getting street map for {MAP_LOCATION}...")
        try:
            our_map_graph = ox.graph_from_place(MAP_LOCATION, network_type="drive")
            # Add road details (length, speed, time)
            our_map_graph = add_road_details(our_map_graph)
            ox.save_graphml(our_map_graph, map_file)
            print(f"Got and saved map for {MAP_LOCATION}.")
        except Exception as e:
            print(f"Error getting map: {e}")
            our_map_graph = None

def add_road_details(graph_data):
    # Add length, speed, and travel time to each road segment
    if graph_data is None: return None

    # How fast we assume cars go on different roads (km/h)
    speed_limits = {
        'motorway': 100, 'trunk': 80, 'primary': 60, 'secondary': 50,
        'tertiary': 40, 'unclassified': 30, 'residential': 25,
        'service': 15, 'living_street': 10, 'cycleway': 20, 'footway': 5,
        'track': 20, 'steps': 3, 'path': 10
    }

    for u, v, key, road_info in graph_data.edges(keys=True, data=True):
        # Calculate road length
        if 'length' not in road_info or road_info['length'] is None:
            lat1, lon1 = graph_data.nodes[u]['y'], graph_data.nodes[u]['x']
            lat2, lon2 = graph_data.nodes[v]['y'], graph_data.nodes[v]['x']
            road_info['length'] = geodesic((lat1, lon1), (lat2, lon2)).meters
        
        # Determine speed for road type
        if 'speed_kph' not in road_info or road_info['speed_kph'] is None:
            road_type = road_info.get('highway')
            if isinstance(road_type, list): road_type = road_type[0]
            elif not road_type: road_type = 'residential'
            road_info['speed_kph'] = speed_limits.get(road_type, 30)

        # Calculate time to travel this road
        if 'travel_time' not in road_info or road_info['travel_time'] is None:
            length_km = road_info.get('length', 0) / 1000
            speed_kph = road_info.get('speed_kph', 0)     
            if speed_kph > 0 and length_km >= 0:
                road_info['travel_time'] = (length_km / speed_kph) * 3600 # seconds
            else:
                road_info['travel_time'] = float('inf') # Can't travel

    return graph_data

def get_graph():
    # Give back the map, or load it if it's not ready
    if our_map_graph is None:
        load_our_map()
    return our_map_graph

def a_star_search(start_coords, goal_coords):
    # Find shortest path using A*
    current_graph = get_graph()
    if current_graph is None:
        print("Error: Map not loaded for A*.")
        return [], 0, 0

    try:
        # Find closest points on map for start/end
        start_node = ox.distance.nearest_nodes(current_graph, start_coords[1], start_coords[0])
        end_node = ox.distance.nearest_nodes(current_graph, goal_coords[1], goal_coords[0])
    except Exception as e:
        print(f"Error finding closest map points: {e}.")
        return [], 0, 0

    if start_node is None or end_node is None:
        print("Couldn't find points on map.")
        return [], 0, 0

    # How we guess distance to goal (straight line)
    def guess_dist_to_goal(node1_id, node2_id):
        node1_info = current_graph.nodes[node1_id]
        node2_info = current_graph.nodes[node2_id]
        point1 = (node1_info['y'], node1_info['x'])
        point2 = (node2_info['y'], node2_info['x'])
        return geodesic(point1, point2).meters

    # A* setup
    check_nodes = [] # Nodes to check
    heapq.heappush(check_nodes, (guess_dist_to_goal(start_node, end_node), start_node))

    path_history = {} # How we got to each node
    cost_so_far = {node: float('inf') for node in current_graph.nodes}
    cost_so_far[start_node] = 0
    total_estimated_cost = {node: float('inf') for node in current_graph.nodes}
    total_estimated_cost[start_node] = guess_dist_to_goal(start_node, end_node)

    while check_nodes:
        # Get node with lowest estimated total cost
        current_est_cost, current_node = heapq.heappop(check_nodes)
        if current_est_cost > total_estimated_cost[current_node]: continue
        if current_node == end_node:
            # Path found! Build it
            path_nodes = []
            temp_node = current_node
            while temp_node in path_history:
                path_nodes.append(temp_node)
                temp_node = path_history[temp_node]
            path_nodes.append(start_node)
            path_nodes.reverse()

            final_coords = []
            total_dist_meters = 0
            total_time_seconds = 0

            # Calculate path details
            for i in range(len(path_nodes) - 1):
                u = path_nodes[i]
                v = path_nodes[i+1]
                road_options = current_graph.get_edge_data(u, v)
                if not road_options: continue

                best_road_data = None
                smallest_cost_diff = float('inf')
                for road_key, road_details in road_options.items():
                    road_time = road_details.get('travel_time')
                    if road_time is None or road_time == float('inf'): continue
                    current_cost_diff = abs(cost_so_far[v] - (cost_so_far[u] + road_time))
                    if current_cost_diff < smallest_cost_diff:
                        smallest_cost_diff = current_cost_diff
                        best_road_data = road_details

                if best_road_data:
                    total_dist_meters += best_road_data.get('length', 0)
                    total_time_seconds += best_road_data.get('travel_time', 0)
                else:
                    # Fallback if no specific edge matched
                    first_road_details = next(iter(road_options.values()))
                    total_dist_meters += first_road_details.get('length', 0)
                    total_time_seconds += first_road_details.get('travel_time', 0)

            # Get coordinates for the path
            for node_id in path_nodes:
                node_info = current_graph.nodes[node_id]
                final_coords.append([node_info['y'], node_info['x']])

            return final_coords, total_dist_meters, total_time_seconds

        # Check neighbors
        for neighbor_node in current_graph.neighbors(current_node):
            min_time_to_neighbor = float('inf')
            road_options = current_graph.get_edge_data(current_node, neighbor_node)
            if not road_options: continue

            for key, road_details in road_options.items():
                road_time = road_details.get('travel_time')
                if road_time is None or road_time == float('inf'): continue
                min_time_to_neighbor = min(min_time_to_neighbor, road_time)
            
            if min_time_to_neighbor == float('inf'): continue # No valid path this way

            new_cost_to_neighbor = cost_so_far[current_node] + min_time_to_neighbor
            
            if new_cost_to_neighbor < cost_so_far[neighbor_node]:
                path_history[neighbor_node] = current_node
                cost_so_far[neighbor_node] = new_cost_to_neighbor
                total_estimated_cost[neighbor_node] = new_cost_to_neighbor + guess_dist_to_goal(neighbor_node, end_node)
                heapq.heappush(check_nodes, (total_estimated_cost[neighbor_node], neighbor_node))

    return [], 0, 0 # No path found