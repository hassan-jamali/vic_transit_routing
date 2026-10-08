#include "a_star_router.h"
#include "heuristic.h"
#include "metal_heuristic.h"
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <algorithm>
#include <iostream>

RouteResult AStarRouter::find_route(const TransitGraph& graph, const std::string& start_id, const std::string& goal_id, int start_time_seconds, bool use_gpu) {
    RouteResult route_result;
    route_result.is_found = false;

    if (!graph.nodes.count(start_id) || !graph.nodes.count(goal_id)) return route_result;

    double goal_latitude = graph.nodes.at(goal_id).latitude;
    double goal_longitude = graph.nodes.at(goal_id).longitude;

    std::unordered_map<std::string, double> precalculated_heuristics;
    if (use_gpu) {
        std::vector<std::string> node_ids;
        std::vector<double> node_latitudes;
        std::vector<double> node_longitudes;
        
        node_ids.reserve(graph.nodes.size());
        node_latitudes.reserve(graph.nodes.size());
        node_longitudes.reserve(graph.nodes.size());

        for (const auto& [node_id, node_data] : graph.nodes) {
            node_ids.push_back(node_id);
            node_latitudes.push_back(node_data.latitude);
            node_longitudes.push_back(node_data.longitude);
        }

        std::vector<double> heuristic_array = MetalHeuristic::calculate_all_distances(node_latitudes, node_longitudes, goal_latitude, goal_longitude);
        for (size_t index = 0; index < node_ids.size(); ++index) {
            precalculated_heuristics[node_ids[index]] = heuristic_array[index];
        }
    }

    std::priority_queue<QueueNode, std::vector<QueueNode>, std::greater<QueueNode>> open_set;
    std::unordered_map<std::string, int> best_arrival_times;
    std::unordered_map<std::string, std::string> parent_nodes;

    open_set.push({start_id, start_time_seconds, 0.0});
    best_arrival_times[start_id] = start_time_seconds;

    while (!open_set.empty()) {
        QueueNode current_node = open_set.top();
        open_set.pop();

        if (current_node.node_id == goal_id) {
            route_result.is_found = true;
            route_result.arrival_time = current_node.time_seconds;

            std::string trace_id = goal_id;
            while (trace_id != start_id) {
                route_result.path.push_back(trace_id);
                trace_id = parent_nodes[trace_id];
            }
            route_result.path.push_back(start_id);
            std::reverse(route_result.path.begin(), route_result.path.end());
            return route_result;
        }

        if (current_node.time_seconds > best_arrival_times[current_node.node_id]) continue;

        if (graph.edges.count(current_node.node_id)) {
            for (const auto& [neighbor_id, trips] : graph.edges.at(current_node.node_id)) {
                for (const auto& trip : trips) {
                    if (trip.is_blocked) continue;
                    if (trip.departure_time >= current_node.time_seconds) {
                        
                        if (!best_arrival_times.count(neighbor_id) || trip.arrival_time < best_arrival_times[neighbor_id]) {
                            best_arrival_times[neighbor_id] = trip.arrival_time;
                            parent_nodes[neighbor_id] = current_node.node_id;
                            
                            double heuristic_distance = 0.0;
                            if (use_gpu) {
                                heuristic_distance = precalculated_heuristics[neighbor_id];
                            } else {
                                double neighbor_latitude = graph.nodes.at(neighbor_id).latitude;
                                double neighbor_longitude = graph.nodes.at(neighbor_id).longitude;
                                heuristic_distance = Heuristic::calculate_distance(neighbor_latitude, neighbor_longitude, goal_latitude, goal_longitude);
                            }

                            double f_score = trip.arrival_time + heuristic_distance;
                            open_set.push({neighbor_id, trip.arrival_time, f_score});
                        }
                    }
                }
            }
        }
    }

    return route_result;
}