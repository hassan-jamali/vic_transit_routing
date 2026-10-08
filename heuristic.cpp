#include "heuristic.h"
#include <cmath>

double Heuristic::calculate_distance(double latitude_one, double longitude_one, double latitude_two, double longitude_two) {
    // haversine distance in meters
    const double earth_radius_meters = 6371000.0;
    const double radians_per_degree = M_PI / 180.0;
    
    double delta_latitude = (latitude_two - latitude_one) * radians_per_degree;
    double delta_longitude = (longitude_two - longitude_one) * radians_per_degree;
    
    double haversine_a = std::sin(delta_latitude / 2) * std::sin(delta_latitude / 2) +
                         std::cos(latitude_one * radians_per_degree) * std::cos(latitude_two * radians_per_degree) *
                         std::sin(delta_longitude / 2) * std::sin(delta_longitude / 2);
               
    return earth_radius_meters * 2 * std::atan2(std::sqrt(haversine_a), std::sqrt(1 - haversine_a));
}

int Heuristic::estimate_time(const TransitGraph& graph, const std::string& current_id, const std::string& goal_id) {
    const auto& current_node = graph.nodes.at(current_id);
    const auto& goal_node = graph.nodes.at(goal_id);
    
    double distance_meters = calculate_distance(current_node.latitude, current_node.longitude, goal_node.latitude, goal_node.longitude);
    return static_cast<int>(distance_meters / 16.67); 
}