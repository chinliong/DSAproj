import heapq
import osmnx as ox
from geopy.distance import geodesic
import folium
import requests
from itertools import permutations

# Load the graph from OpenStreetMap
G = ox.graph_from_place('Singapore', network_type='drive')

# Function to find the nearest node in the graph to a given coordinate
def get_nearest_node(graph, point):
    return ox.distance.nearest_nodes(graph, point[1], point[0])

# Heuristic function for A*
def heuristic(node1, node2, graph):
    coords_1 = (graph.nodes[node1]['y'], graph.nodes[node1]['x'])
    coords_2 = (graph.nodes[node2]['y'], graph.nodes[node2]['x'])
    h = geodesic(coords_1, coords_2).meters
    return h

# Custom A* search algorithm
def a_star_search(graph, start, goal):
    pq = [(0, start, [])]  # Priority queue as (cost, current_node, path)
    costs = {start: 0}
    visited = set()
    
    while pq:
        cost, current, path = heapq.heappop(pq)
        
        if current in visited:
            continue
        
        path = path + [current]
        if current == goal:
            return path
        
        visited.add(current)
        
        for neighbor in graph.neighbors(current):
            if neighbor in visited:
                continue
            
            new_cost = costs[current] + graph[current][neighbor][0]['length']
            if neighbor not in costs or new_cost < costs[neighbor]:
                costs[neighbor] = new_cost
                priority = new_cost + heuristic(neighbor, goal, graph)
                heapq.heappush(pq, (priority, neighbor, path))
    
    return None

# Function to fetch real-time traffic data from the LTA API
def fetch_traffic_data(api_key):
    url = "http://datamall2.mytransport.sg/ltaodataservice/v3/TrafficSpeedBands"
    headers = {
        'AccountKey': api_key,
        'accept': 'application/json'
    }
    response = requests.get(url, headers=headers)
    
    if response.status_code == 200:
        try:
            return response.json()
        except requests.exceptions.JSONDecodeError as e:
            print(f"JSON decode error: {e}")
            print(f"Response content: {response.content}")
    else:
        print(f"Failed to fetch data: {response.status_code}")
        print(f"Response content: {response.content}")
    
    return None

# Function to calculate fuel consumption in liters
def calculate_fuel_consumption(distance_km, fuel_efficiency_l_per_100km, traffic_factor=1.0):
    adjusted_fuel_efficiency = fuel_efficiency_l_per_100km * traffic_factor
    fuel_consumption = (distance_km / 100) * adjusted_fuel_efficiency
    return fuel_consumption

# Function to determine traffic factor based on fetched traffic data
def determine_traffic_factor(traffic_data):
    try:
        average_speed = sum(item['SpeedBand'] for item in traffic_data['value']) / len(traffic_data['value'])
        if average_speed < 2:
            return 1.5  # Heavy traffic
        elif average_speed < 4:
            return 1.2  # Moderate traffic
        else:
            return 1.0  # Normal traffic
    except Exception as e:
        print(f"Error in determining traffic factor: {e}")
        return 1.0  # Default to normal traffic factor if any error occurs

# Input coordinates (latitude, longitude)
start_coords = (1.3321, 103.8934)  # [Ubi Challenger warehouse]
destination_coords = [
    (1.2936, 103.8319),     # GWC [Furthest]
    (1.3507, 103.8488),     # ION orchard [middle]
    (1.3002, 103.8552)      # Bishan [closest]
]

# Find the nearest nodes in the graph to the given coordinates
start_node = get_nearest_node(G, start_coords)
destination_nodes = [get_nearest_node(G, coords) for coords in destination_coords]

# Function to calculate the total path distance for a given order of nodes
def calculate_total_distance(order):
    total_distance = 0
    current_node = start_node
    for node in order:
        segment = a_star_search(G, current_node, node)
        if segment:
            segment_distance = sum(G[segment[i]][segment[i + 1]][0]['length'] for i in range(len(segment) - 1))
            total_distance += segment_distance
        current_node = node
    return total_distance

# Define fuel efficiency of the vehicle and fuel price
fuel_efficiency = 8.7  # in liters per 100 kilometers
fuel_price_per_liter = 2.30  # in SGD

# Realistic average speed assumption for travel time calculation (40 km/h in urban areas)
average_speed_kmh = 40

def main():
    """
    This is the main function. Code starts executing here
    """
    # Fetch real-time traffic data from LTA
    api_key = 'o2oSSMCJSUOkZQxWvyAjsA=='  # Replace with your actual LTA API key
    traffic_data = fetch_traffic_data(api_key)
    traffic_factor = determine_traffic_factor(traffic_data) if traffic_data else 1.0

    # Find the optimal order using a brute-force approach
    optimal_order = min(permutations(destination_nodes), key=calculate_total_distance)

    # Define colors for segments
    colors = ['green', 'blue', 'purple', 'orange']

    # Calculate the route based on the optimal order
    route_nodes = [start_node]
    route_segments = []
    total_distance = 0
    total_fuel_cost = 0
    total_duration = 0

    for i, dest_node in enumerate(optimal_order):
        segment = a_star_search(G, route_nodes[-1], dest_node)
        if segment:
            route_segments.append((segment, colors[i % len(colors)]))
            route_nodes.extend(segment[1:])  # Avoid duplicating nodes

            # Calculate distance, fuel consumption, and duration
            segment_distance = sum(G[segment[j]][segment[j + 1]][0]['length'] for j in range(len(segment) - 1)) / 1000  # Convert to km
            total_distance += segment_distance
            fuel_consumption = calculate_fuel_consumption(segment_distance, fuel_efficiency, traffic_factor)
            fuel_cost = fuel_consumption * fuel_price_per_liter
            total_fuel_cost += fuel_cost
            # Calculate travel time assuming an average speed in km/h
            segment_duration = (segment_distance / average_speed_kmh) * 60  # Convert to minutes
            total_duration += segment_duration

    # Create Folium map
    route_map = folium.Map(location=start_coords, zoom_start=13)

    # Add route segments to map
    for segment, color in route_segments:
        segment_coords = [(G.nodes[node]['y'], G.nodes[node]['x']) for node in segment]
        folium.PolyLine(segment_coords, color=color, weight=2.5, opacity=1).add_to(route_map)

    # Add markers for start and destinations
    folium.Marker(location=start_coords, popup="Start", icon=folium.Icon(color='green')).add_to(route_map)
    for coords in destination_coords:
        folium.Marker(location=coords, popup="Destination", icon=folium.Icon(color='red')).add_to(route_map)

    # Add total distance, duration, and fuel cost to the map
    folium.Marker(location=start_coords, popup=f"Total Distance: {total_distance:.2f} km\nTotal Duration: {total_duration:.2f} minutes\nTotal Fuel Cost: ${total_fuel_cost:.2f}", icon=folium.Icon(color='blue')).add_to(route_map)

    # Save the map to an HTML file
    map_path = 'route_map.html'
    route_map.save(map_path)

    print(f"Map saved to {map_path}. Open this file in a web browser to view the route.")
    print(f"Total Distance: {total_distance:.2f} km")
    print(f"Total Duration: {total_duration:.2f} minutes")
    print(f"Total Fuel Cost: ${total_fuel_cost:.2f}")

if __name__ == "__main__":
    main()
