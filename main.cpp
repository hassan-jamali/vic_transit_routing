#include "transit_graph.h"
#include "a_star_router.h"
#include "metal_heuristic.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <cstdlib>
#include <nlohmann/json.hpp>

int main(int argument_count, char* argument_values[]) {
    if (argument_count < 3) return 1;

    std::vector<std::string> route_nodes;
    std::vector<std::string> blocked_nodes;
    int start_time_seconds = 18000;
    bool use_gpu = false;
    int batch_size = 0; 

    std::string first_argument = argument_values[1];
    bool uses_flag_format = (first_argument.rfind("--", 0) == 0);

    // parse command line arguments
    if (uses_flag_format) {
        std::string current_mode = "";
        for (int index = 1; index < argument_count; ++index) {
            std::string argument = argument_values[index];

            if (argument == "--gpu") {
                use_gpu = true;
                continue;
            }
            if (argument == "--batch") {
                if (index + 1 < argument_count) {
                    try { batch_size = std::stoi(argument_values[++index]); }
                    catch (...) { batch_size = 10000; }
                }
                continue;
            }
            if (argument == "--time") {
                if (index + 1 < argument_count) {
                    try { start_time_seconds = std::stoi(argument_values[++index]); }
                    catch (...) { start_time_seconds = 18000; }
                }
                continue;
            }
            if (argument == "--route" || argument == "--block") {
                current_mode = argument;
                continue;
            }
            if (current_mode == "--route") {
                route_nodes.push_back(argument);
            } else if (current_mode == "--block") {
                blocked_nodes.push_back(argument);
            }
        }
    } else {
        // support positional argument fallback
        route_nodes.push_back(argument_values[1]);
        route_nodes.push_back(argument_values[2]);
        if (argument_count >= 4) {
            try { start_time_seconds = std::stoi(argument_values[3]); }
            catch (...) { start_time_seconds = 18000; }
        }
        for (int index = 4; index < argument_count; ++index) {
            blocked_nodes.push_back(argument_values[index]);
        }
    }

    // load network graph from json
    TransitGraph graph;
    if (!graph.load_from_json("transit_graph.json")) return 1;

    // execute massive batch heuristic simulation if flag is set
    if (batch_size > 0) {
        std::vector<double> node_latitudes;
        std::vector<double> node_longitudes;
        for (const auto& [id, node] : graph.nodes) {
            node_latitudes.push_back(node.latitude);
            node_longitudes.push_back(node.longitude);
        }

        size_t node_count = node_latitudes.size();
        std::vector<double> goal_latitudes;
        std::vector<double> goal_longitudes;
        
        // randomly distribute commuters across the network
        srand(42); 
        for (int index = 0; index < batch_size; ++index) {
            int random_index = rand() % node_count;
            goal_latitudes.push_back(node_latitudes[random_index]);
            goal_longitudes.push_back(node_longitudes[random_index]);
        }

        std::cout << "rush hour simulation: " << batch_size << " commuters across " << node_count << " stations\n";
        std::cout << "total calculations: " << (node_count * batch_size) << "\n";

        if (use_gpu) {
            // compute distances in parallel on gpu
            auto start_time = std::chrono::high_resolution_clock::now();
            auto matrix = MetalHeuristic::calculate_batch_distances(node_latitudes, node_longitudes, goal_latitudes, goal_longitudes);
            auto end_time = std::chrono::high_resolution_clock::now();
            double elapsed_milliseconds = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000.0;
            std::cout << "gpu parallel execution: " << elapsed_milliseconds << " ms\n";
        } else {
            // compute distances sequentially on cpu
            auto start_time = std::chrono::high_resolution_clock::now();
            double earth_radius_meters = 6371000.0;
            double degrees_to_radians = M_PI / 180.0;
            std::vector<std::vector<double>> matrix(batch_size, std::vector<double>(node_count, 0.0));

            for (size_t goal_index = 0; goal_index < batch_size; ++goal_index) {
                for (size_t node_index = 0; node_index < node_count; ++node_index) {
                    double latitude_1 = node_latitudes[node_index] * degrees_to_radians;
                    double longitude_1 = node_longitudes[node_index] * degrees_to_radians;
                    double latitude_2 = goal_latitudes[goal_index] * degrees_to_radians;
                    double longitude_2 = goal_longitudes[goal_index] * degrees_to_radians;
                    double delta_latitude = latitude_2 - latitude_1;
                    double delta_longitude = longitude_2 - longitude_1;
                    double a = sin(delta_latitude / 2.0) * sin(delta_latitude / 2.0) +
                               cos(latitude_1) * cos(latitude_2) *
                               sin(delta_longitude / 2.0) * sin(delta_longitude / 2.0);
                    matrix[goal_index][node_index] = earth_radius_meters * (2.0 * atan2(sqrt(a), sqrt(1.0 - a)));
                }
            }
            auto end_time = std::chrono::high_resolution_clock::now();
            double elapsed_milliseconds = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000.0;
            std::cout << "cpu sequential execution: " << elapsed_milliseconds << " ms\n";
        }
        return 0;
    }

    // validate route waypoint sequence
    if (route_nodes.size() < 2) return 1;

    // sever edges connected to blocked stations
    for (const auto& blocked_node_id : blocked_nodes) {
        if (graph.edges.count(blocked_node_id)) {
            for (auto& [destination_id, trips] : graph.edges[blocked_node_id]) {
                for (auto& trip : trips) trip.is_blocked = true;
            }
        }
        for (auto& [origin_id, destinations] : graph.edges) {
            if (destinations.count(blocked_node_id)) {
                for (auto& trip : destinations[blocked_node_id]) trip.is_blocked = true;
            }
        }
    }

    // execute multi segment pathfinding
    RouteResult combined_route;
    combined_route.is_found = true;
    int current_time_seconds = start_time_seconds;

    auto route_start_time = std::chrono::high_resolution_clock::now();

    for (size_t segment_index = 0; segment_index < route_nodes.size() - 1; ++segment_index) {
        std::string segment_origin = route_nodes[segment_index];
        std::string segment_destination = route_nodes[segment_index + 1];

        RouteResult segment_result = AStarRouter::find_route(graph, segment_origin, segment_destination, current_time_seconds, use_gpu);

        if (!segment_result.is_found) {
            combined_route.is_found = false;
            break;
        }

        size_t start_offset = (segment_index == 0) ? 0 : 1;
        for (size_t path_index = start_offset; path_index < segment_result.path.size(); ++path_index) {
            combined_route.path.push_back(segment_result.path[path_index]);
        }
        current_time_seconds = segment_result.arrival_time;
    }
    combined_route.arrival_time = current_time_seconds;

    auto route_end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::micro> elapsed_time_microseconds = route_end_time - route_start_time;

    // write final route output to json
    nlohmann::json json_data = {
        {"found", combined_route.is_found},
        {"arrival_time", combined_route.arrival_time},
        {"path", combined_route.path},
        {"execution_time_microseconds", elapsed_time_microseconds.count()}
    };

    std::ofstream output_file("route_result.json");
    output_file << json_data.dump(4) << "\n";

    std::cout << "route exported to route_result.json\n";
    return 0;
}