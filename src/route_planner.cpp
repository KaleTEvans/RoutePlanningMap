#include "route_planner.h"
#include <algorithm>
#include <vector>
using namespace std;

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


// Implement the CalculateHValue method.
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return end_node->distance(* node);
}


// Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    // Find neighbors method
    current_node->FindNeighbors();
    // loop through the node's potential neighbors
    for (auto node : current_node->neighbors) {
        // set the parent
        node->parent = current_node;
        // set h value 
        node->h_value = CalculateHValue(node);
        // increment the g value
        auto CurrentGValue = current_node->distance(*node);
        node->g_value = current_node->g_value + CurrentGValue;
        // using emplace_back here for an object
        open_list.emplace_back(node);
        // set the node as visited
        node->visited = true;
    }
}


// Complete the NextNode method to sort the open list and return the next node.
RouteModel::Node *RoutePlanner::NextNode() {
    // sort the list according to the sum of the h and g value
    // using lambda expressions for simpler code
    sort(open_list.begin(), open_list.end(), [](auto& a, auto& b) {
        return a->h_value + a->g_value > b->h_value + b-> g_value;
    });

    // add a pointer to the end of the list, which has the smallest sum
    auto *node = open_list.back();
    // remove the node from the list
    open_list.pop_back();
    // return the pointer
    return node;
}


// Complete the ConstructFinalPath method to return the final path found from your A* search.
vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    vector<RouteModel::Node> path_found;

    // run while loop running until starting node is found
    while (current_node != start_node) {
        // add nodes to path found
        path_found.insert(path_found.begin(), *current_node);
        // iincrement the distance
        distance += current_node->distance(*current_node->parent);
        // update current node 
        current_node = current_node->parent;
    }
    path_found.insert(path_found.begin(), *start_node);

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// Write the A* Search algorithm here.
void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // set the current node value to that of the start node and set visited to true
    current_node = start_node;
    current_node->visited = true;
    // add the starting node to the open list
    open_list.emplace_back(start_node);
    
    // use loop to iterate over nodes while the open list has values
    while (open_list.size() != 0) {
        // if statement to exit if end node is founde
        if (current_node->x && current_node->y == end_node->y) {
            m_Model.path = ConstructFinalPath(current_node);
            break;
        }

        // add all of the neighbors of the current node to the open list
        AddNeighbors(current_node);
        // go to the next node
        current_node = NextNode();
    }
}