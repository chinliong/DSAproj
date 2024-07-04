import osmnx as ox
import networkx as nx
import folium
from itertools import permutations

def download_and_project_graph():
    print("Downloading OSM data for Singapore...")
    try:
        G = ox.graph_from_place('Singapore', network_type='drive')
        print("Data downloaded.")
        return G
    except Exception as e:
        print(f"Error during download: {e}")
        return None

def calculate_distance_matrix(G, nodes):
    distance_matrix = {}
    for i, source in enumerate(nodes):
        distance_matrix[source] = {}
        for j, destination in enumerate(nodes):
            if i != j:
                distance_matrix[source][destination] = nx.shortest_path_length(G, source=source, target=destination, weight='length')
            else:
                distance_matrix[source][destination] = 0
    return distance_matrix

def solve_tsp(start_node, nodes, distance_matrix):
    other_nodes = [node for node in nodes if node != start_node]
    all_permutations = permutations(other_nodes)
    min_path = None
    min_distance = float('inf')
    for perm in all_permutations:
        current_distance = distance_matrix[start_node][perm[0]]
        for i in range(len(perm) - 1):
            current_distance += distance_matrix[perm[i]][perm[i+1]]
        if current_distance < min_distance:
            min_distance = current_distance
            min_path = (start_node,) + perm
    return min_path, min_distance

def find_shortest_paths(G, nodes):
    paths = []
    for i in range(len(nodes) - 1):
        source = nodes[i]
        destination = nodes[i+1]
        shortest_path = nx.shortest_path(G, source=source, target=destination, weight='length')
        paths.append(shortest_path)
    return paths

def visualize_paths(G, paths, nodes):
    try:
        route_map = folium.Map(location=[1.3521, 103.8198], zoom_start=12)

        for path in paths:
            ox.plot_route_folium(G, path, route_map=route_map, route_color='blue', opacity=0.7)

        # Add markers to the map
        for idx, node in enumerate(nodes):
            lat_lon = (G.nodes[node]['y'], G.nodes[node]['x'])
            color = 'red' if idx == 0 else 'green' if idx == len(nodes) - 1 else 'orange'
            folium.Marker(lat_lon, popup=f'Location {idx+1}', icon=folium.Icon(color=color)).add_to(route_map)

        route_map.save("dijkstra.html")
        print("TSP path visualization saved as 'tsp_shortest_path_singapore.html'.")
    except Exception as e:
        print(f"Error visualizing paths: {e}")

def main():
    G = download_and_project_graph()
    if G is None:
        print("Failed to download the graph. Exiting...")
        return

    locations = [
        (1.3521, 103.8198),  # Marina Bay Sands
        (1.2839, 103.8601),  # Singapore Sports Hub
        (1.2806, 103.8500)   # Raffles Place
    ]

    # Find nearest nodes for the locations
    nodes = [ox.distance.nearest_nodes(G, loc[1], loc[0]) for loc in locations]
    
    # Ensure the first node is Marina Bay Sands
    start_node = nodes[0]
    
    # Calculate the distance matrix using Dijkstra's algorithm
    distance_matrix = calculate_distance_matrix(G, nodes)
    
    # Solve the TSP problem
    tsp_path, tsp_distance = solve_tsp(start_node, nodes, distance_matrix)
    print(f"TSP path: {tsp_path}, Distance: {tsp_distance}")
    
    # Find the shortest paths for the TSP route
    shortest_paths = find_shortest_paths(G, tsp_path)
    
    # Visualize the paths on the map
    visualize_paths(G, shortest_paths, tsp_path)
    
if __name__ == "__main__":
    main()
