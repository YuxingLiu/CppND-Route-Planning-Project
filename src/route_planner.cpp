#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Find the closest nodes to the inputs:
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    // Compute the h value of node as the distance to the end_node.
    return end_node->distance(*node);
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    // Find all the unvisited neighbors of current_node.
    current_node->FindNeighbors();
    for(auto it:current_node->neighbors) {
        // Set the parent, the h_value, the g_value.
        it->parent = current_node;
        it->h_value = CalculateHValue(it);
        it->g_value = current_node->g_value + current_node->distance(*it);

        // Add the neighbor to open_list and set the node's visited attribute to true.
        open_list.push_back(it);
        it->visited = true;
    }
}


bool Compare(const RouteModel::Node *a, const RouteModel::Node *b) {
    float f1 = a->g_value + a->h_value;
    float f2 = b->g_value + b->h_value;
    return f1 > f2;
}

RouteModel::Node *RoutePlanner::NextNode() {
    // Sort the open_list in descending order of f value (g+h)
    std::sort(open_list.begin(), open_list.end(), Compare);
    RouteModel::Node *cur = open_list.back();
    open_list.pop_back();
    return cur;
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // Take the current (final) node as an argument and iteratively follow the chain of parents
    // of nodes until the starting node is found.
    path_found.push_back(*current_node);
    while(current_node != start_node){
        distance += current_node->distance(*current_node->parent);
        current_node = current_node->parent;
        path_found.push_back(*current_node);
    }

    // Restore the correct order: the start node is the first element of the vector.
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}


void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // Add the start_node to the open_list.
    start_node->visited = true;
    AddNeighbors(start_node);

    while(!open_list.empty()) {
        // Sort the open list by f value, and pop the smallest one.
        current_node = NextNode();

        // If the end_node is reached, return the final path.
        if(current_node == end_node){
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }

        // Else, expand the search to the current_node's neighbors.
        AddNeighbors(current_node);
    }

    std::cout << "No path found!" << "\n";
}