
import sys

from enum import Enum
# Python3 implementation to build a
# graph using Dictonaries

from collections import defaultdict

# Function to build the graph
def build_graph():
	edges = [
		["A", "B",5], ["A", "E"],
		["A", "C"], ["B", "D"],
		["B", "E"], ["C", "F"],
		["C", "G"], ["D", "E"]
	]
	graph = defaultdict(list)
	
	# Loop to iterate over every
	# edge of the graph
	for edge in edges:
		a, b = edge[0], edge[1]
		
		# Creating the graph
		# as adjacency list
		graph[a].append(b)
		graph[b].append(a)
	return graph

if __name__ == "__main__":
	graph = build_graph()
	
	print(graph)


class DijkstrasAlg:
    """Implements Dijkstra's algorithm and stores the shortest paths from all nodes to all reachable nodes."""

    """
    'shortest_dists' is a dictionary storing the shortest distances between any two cities.
    In the example below, the shortest distance from A to B is 5.0, from E to D is 15.0, etc.
    Note that a city is not at distance 0 from itself; at least one edge must be traversed to create a route.
    Destinations that can't be reached will not appear; for example from A to A.

    {'A': {'B': 5.0, 'C': 9.0, 'D': 1.0, 'E': 3.0},
     'B': {'B': 9.0, 'C': 4.0, 'D': 12.0, 'E': 6.0},
     'C': {'B': 5.0, 'C': 9.0, 'D': 8.0, 'E': 2.0},
     'D': {'B': 5.0, 'C': 8.0, 'D': 16.0, 'E': 2.0},
     'E': {'B': 3.0, 'C': 7.0, 'D': 15.0, 'E': 9.0}}
    """


    def __init__(self, graph_dict):
        """Find and store all shortest routes for this graph."""
        self.shortest_dists = {}

        # Run Dijkstra's algorithm with each node as the start node.
        for start_node in graph_dict.keys():
            self.shortest_dists[start_node] = self._compute_shotest_paths(graph_dict, start_node)


    def _compute_shotest_paths(self, graph_dict, start_node):
        """Use Dijkstra's algorithm to compute all shortest paths."""
        completed_nodes = {}
        visited_nodes = {start_node: 0}

        while visited_nodes:
            min_value_key = min(visited_nodes, key=visited_nodes.get)
            cost_to_min_node = visited_nodes.pop(min_value_key)

            for k, v in graph_dict[min_value_key].items():
                if k not in completed_nodes:
                    comparison_min = cost_to_min_node + v
                    if k not in visited_nodes:
                        visited_nodes[k] = comparison_min
                    else:
                        visited_nodes[k] = min(comparison_min, visited_nodes[k])

            # Don't mark start node completed at a distance of 0
            if min_value_key != start_node or cost_to_min_node > 0:
                completed_nodes[min_value_key] = cost_to_min_node
        return completed_nodes


    def get_distance(self, start_node, end_node):
        if end_node in self.shortest_dists.get(start_node, {}):
            return self.shortest_dists[start_node][end_node]
        return "NO SUCH ROUTE"



class TrainGraph:
    """
    Takes a string representing a directed graph of city-to-city connections and their distances of
    the format "AB5, BC4, CD8, DC8, DE6, AD5, CE2, EB3, AE7" and allows querying of 
    - shortest routes,
    - the distance of specific routes,
    - the number of routes from one city to another with a max number of stops,
    - the number of routes from one city to another with an exact number of stops,
    - the number of routes from one city to another with less than a specified distance.

    Formatting of the graph string and the submitted routes is assumed to be valid.
    All distances are required to be greater than 0 so that the algorithms used have valid answers.
    """

    TripType = Enum('TripType', 'exact_stops max_stops')


    def __init__(self, graph):
        self.graph = self._convert_graph_to_dict(graph)
        self.dijkstras_alg = DijkstrasAlg(self.graph)


    def _convert_graph_to_dict(self, graph):
        """Convert the 'graph' string to a dictionary."""
        graph_dict = {}
        connection_list = [x.strip() for x in graph.split(',')]
        for connection in connection_list:
            start_city = connection[0]
            next_city = connection[1]
            distance = float(connection[2:])

            if start_city not in graph_dict:
                graph_dict[start_city] = {}

            graph_dict[start_city][next_city] = distance
        return graph_dict


    def get_distance(self, route):
        """Calculate distance for a specific route, where 'route' is a string of format "A-B-D" """
        if route == "":
            return "NO SUCH ROUTE"

        route_cities = route.split('-')
        current_city = route_cities[0]
        total_distance = 0
        # Iterate through cities on the route and sum their distances.
        for i in range(1, len(route_cities)):
            next_city = route_cities[i]
            if current_city in self.graph and next_city in self.graph[current_city]:
                total_distance += self.graph[current_city][next_city]
            else:
                return "NO SUCH ROUTE"
            current_city = next_city

        return total_distance


    def get_number_trips(self, start_city, end_city, num_stops, trip_type):
        """
        Get the number of possible trips from start_city to end_city.
        If trip_type is TripType.max_stops, all trips with stops <= num_stops are counted.
        Otherwise only trips with exactly num_stops are counted.
        """
        sum_routes = 0
        if num_stops == 0:
            return 0

        # iterate through all connections from start_city
        for next_city in self.graph[start_city]:
            if next_city == end_city and (trip_type == self.TripType.max_stops or num_stops == 1):
                sum_routes += 1

            sum_routes += self.get_number_trips(next_city, end_city, num_stops - 1, trip_type)

        return sum_routes
my_routes = TrainGraph("AB5, BC4, CD8, DC8, DE6, AD5, CE2, EB3, AE7")
distance =my_routes.get_distance("A-B-C")
print("the distance of the route A-B-C output#1:",distance)

tripnumber=my_routes.get_number_trips("C", "C", 3, TrainGraph.TripType.max_stops)
print("The number of trips starting at C and ending at C  output#6:",tripnumber)

