#include <iostream>
#include <vector>
#include <unordered_map>
#include <limits>
#include <set>
#include <algorithm> // Include this header for the reverse function

using namespace std;

// Define an adjacency list representation for the graph
unordered_map<char, unordered_map<char, int>> graph = {
    {'A', {{'B', 10}, {'E', 3}}},
    {'B', {{'C', 2}, {'E', 4}}},
    {'C', {{'D', 9}}},
    {'D', {{'C', 7}}},
    {'E', {{'B', 1}, {'C', 8}}}
};

// Function to find the node with the minimum distance which is not yet visited
char get_min_distance_node(unordered_map<char, int>& distances, set<char>& visited) {
    int min_distance = numeric_limits<int>::max();
    char min_node = '\0';

    for (const auto& pair : distances) {
        char node = pair.first;
        int distance = pair.second;
        if (visited.find(node) == visited.end() && distance < min_distance) {
            min_distance = distance;
            min_node = node;
        }
    }
    return min_node;
}

// Dijkstra's algorithm implementation
pair<int, vector<char>> dijkstra(char start_node, char end_node) {
    // Step 1: Initialize distances and visited set
    unordered_map<char, int> distances;
    unordered_map<char, char> previous_nodes;
    set<char> visited;
    
    for (const auto& node : graph) {
        distances[node.first] = numeric_limits<int>::max();
    }
    
    distances[start_node] = 0;

    // Step 2: Iterate over all nodes until all are processed
    while (visited.size() < graph.size()) {
        char current_node = get_min_distance_node(distances, visited);
        
        if (current_node == '\0') {
            break; // No more reachable nodes
        }

        visited.insert(current_node);

        // Step 3: Update distances to neighbors
        for (const auto& neighbor : graph[current_node]) {
            char neighbor_node = neighbor.first;
            int edge_weight = neighbor.second;

            if (visited.find(neighbor_node) == visited.end()) {
                int new_distance = distances[current_node] + edge_weight;
                if (new_distance < distances[neighbor_node]) {
                    distances[neighbor_node] = new_distance;
                    previous_nodes[neighbor_node] = current_node;
                }
            }
        }

        if (current_node == end_node) {
            break; // Stop early if we've reached the destination
        }
    }

    // Step 4: Retrieve the shortest path and cost
    vector<char> path;
    int total_cost = distances[end_node];
    for (char at = end_node; at != start_node; at = previous_nodes[at]) {
        path.push_back(at);
    }
    path.push_back(start_node);
    reverse(path.begin(), path.end()); // Reverse the path to get the correct order

    return {total_cost, path};
}

int main() {
    char start_node, end_node;

    // Step 5: Get user input for start and end nodes
    cout << "Enter the start node (A, B, C, D, E): ";
    cin >> start_node;
    cout << "Enter the end node (A, B, C, D, E): ";
    cin >> end_node;

    // Convert inputs to uppercase
    start_node = toupper(start_node);
    end_node = toupper(end_node);

    // Check if start and end nodes exist in the graph
    if (graph.find(start_node) == graph.end() || graph.find(end_node) == graph.end()) {
        cout << "Error: One or both of the nodes are not present in the graph." << endl;
        return 1;
    }

    // Step 6: Apply Dijkstra's algorithm and output the result
    pair<int, vector<char>> result = dijkstra(start_node, end_node);

    int shortest_distance = result.first;
    vector<char> shortest_path = result.second;

    cout << "Shortest path from " << start_node << " to " << end_node << ": ";
    for (char node : shortest_path) {
        cout << node << " ";
    }
    cout << "\nTotal cost: " << shortest_distance << endl;

    return 0;
}
