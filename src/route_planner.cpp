#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return(node->distance(*end_node));
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    // This will update the current_node's neighbors vector
    current_node->FindNeighbors();

    // Loop through all neighbors
    for( RouteModel::Node *n : current_node->neighbors ) {
        // Assign values
        n->parent = current_node;
        n->g_value = current_node->g_value + n->distance(*current_node);
        n->h_value = CalculateHValue(n);

        // Add to open_list
        open_list.emplace_back(n);

        // Set neighbor's visitied attribute to True
        n->visited = true; 
    }
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {
    // Sort open_list
    std::sort(open_list.begin(), open_list.end(), [](const RouteModel::Node *i, const RouteModel::Node *j){return (i->g_value + i->h_value) < (j->g_value + j->h_value);});

    // Pointer to lowest node
    RouteModel::Node *lowest_node = open_list[0];
    // Remove node
    open_list.erase(open_list.begin());

    return(lowest_node);
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
    while(current_node != start_node) {
        // Add the distance from current node to parent node to distance
        distance += current_node->distance(*current_node->parent);

        // Add current_node to path_found vector will get all but start_node
        path_found.push_back(*current_node);

        // Replace current_node with parent node
        current_node = current_node->parent;
    }

    // Add start_node to path_found
    path_found.push_back(*start_node);

    // Reverse order of path_found to get correct order
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    // Add the start_node 
    RouteModel::Node *current_node = start_node;

    // TODO: Implement your solution here.
    // Add the start_node to the openlist
    open_list.emplace_back(current_node);
    // set visited to true so that FindNeighbors works on the first iteration 
    start_node->visited = true;

    // Loop through open_list till end is found or it's empty and we failed
    while(open_list.size() > 0) {
        // Check that the current_node is not the end
        if(current_node == end_node) {
            // Add found path to model
            m_Model.path = ConstructFinalPath(current_node);
            // End A* search
            return;
        }

        // Find the next node with lowest value
        current_node = NextNode();
        
        // Exand our seach for neighbor nodes of current_node
        AddNeighbors(current_node);
    }
}