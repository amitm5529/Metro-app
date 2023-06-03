#include <iostream>
#include <vector>
#include <queue>
#include <climits>

// Structure to represent a station in the Metro network
struct Station {
    int id;             // Unique identifier for the station
    std::string name;   // Name of the station

    Station(int _id, std::string _name) : id(_id), name(_name) {}
};

// Structure to represent an edge between two stations in the Metro network
struct Edge {
    int source;         // ID of the source station
    int destination;    // ID of the destination station
    int weight;         // Weight of the edge (distance or travel time)

    Edge(int _source, int _destination, int _weight) : source(_source), destination(_destination), weight(_weight) {}
};

// Graph class to represent the Metro network
class MetroGraph {
private:
    std::vector<Station> stations;               // List of all stations
    std::vector<std::vector<Edge>> adjacencyList; // Adjacency list representation of the graph

public:
    MetroGraph(const std::vector<Station>& _stations) : stations(_stations) {
        int numStations = stations.size();
        adjacencyList.resize(numStations);
    }

    // Function to add an edge between two stations
    void addEdge(int source, int destination, int weight) {
        adjacencyList[source].push_back(Edge(source, destination, weight));
        adjacencyList[destination].push_back(Edge(destination, source, weight));
    }

    // Function to find the shortest path between two stations using Dijkstra's algorithm
    std::vector<int> findShortestPath(int source, int destination) {
        std::vector<int> distance(stations.size(), INT_MAX); // Distance array to store the shortest distances
        std::vector<int> parent(stations.size(), -1);        // Parent array to store the shortest path tree
        std::vector<bool> visited(stations.size(), false);   // Visited array to track visited stations

        // Priority queue to store the next station to visit based on minimum distance
        std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<std::pair<int, int>>> pq;

        // Set the distance of the source station to 0 and add it to the priority queue
        distance[source] = 0;
        pq.push(std::make_pair(0, source));

        while (!pq.empty()) {
            int u = pq.top().second;
            pq.pop();

            visited[u] = true;

            for (const Edge& edge : adjacencyList[u]) {
                int v = edge.destination;
                int weight = edge.weight;

                if (!visited[v] && distance[u] + weight < distance[v]) {
                    distance[v] = distance[u] + weight;
                    parent[v] = u;
                    pq.push(std::make_pair(distance[v], v));
                }
            }
        }

        // Reconstruct the shortest path from source to destination
        std::vector<int> shortestPath;
        int current = destination;

        while (current != -1) {
            shortestPath.insert(shortestPath.begin(), current);
            current = parent[current];
        }

        return shortestPath;
    }
};

int main() {
    std::vector<Station> stations = {
        Station(0, "A"),
        Station(1, "B"),
        Station(2, "C"),
        Station(3, "D"),
        Station(4, "E"),
        Station(5, "F"),
        Station(6, "G")
    };

    MetroGraph metro(stations);

    // Adding edges between stations
    metro.addEdge(0, 1, 5);
    metro.addEdge(0, 2, 4);
    metro.addEdge(1, 2, 2);
    metro.addEdge(1, 3, 6);
    metro.addEdge(2, 4, 3);
    metro.addEdge(3, 4, 7);
    metro.addEdge(3, 5, 8);
    metro.addEdge(4, 5, 1);
    metro.addEdge(4, 6, 9);
    metro.addEdge(5, 6, 5);

    int source = 0;
    int destination = 6;

    std::vector<int> shortestPath = metro.findShortestPath(source, destination);

    std::cout << "Shortest path from " << stations[source].name << " to " << stations[destination].name << ":\n";
    for (int station : shortestPath) {
        std::cout << stations[station].name << " ";
    }
    std::cout << std::endl;

    return 0;
}
