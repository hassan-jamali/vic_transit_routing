#include "transit_graph.h"
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

bool TransitGraph::load_from_json(const std::string& file_path) {
    // read and parse json
    std::ifstream file_stream(file_path);
    json json_data;
    
    try {
        file_stream >> json_data;
    } catch (const json::parse_error& error) {
        std::cerr << "json parse error: " << error.what() << "\n";
        return false;
    }
    
    // load nodes
    for (auto& [stop_id, node_data] : json_data["nodes"].items()) {
        nodes[stop_id] = {
            node_data.value("name", "unknown"),
            node_data.value("latitude", 0.0),
            node_data.value("longitude", 0.0)
        };
    }
    
    // load edges
    for (auto& [origin_id, destinations] : json_data["edges"].items()) {
        for (auto& [destination_id, trips] : destinations.items()) {
            for (auto& trip : trips) {
                edges[origin_id][destination_id].push_back({
                    trip.value("trip_id", "unknown"),
                    trip.value("departure_time", 0),
                    trip.value("arrival_time", 0),
                    trip.value("travel_time", 0)
                });
            }
        }
    }
    
    return true;
}

Trip* TransitGraph::find_trip(const std::string& origin, const std::string& destination, const std::string& trip_id) {
    // dry trip lookup for dynamic updates
    for (auto& trip : edges[origin][destination]) {
        if (trip.trip_id == trip_id) return &trip;
    }
    return nullptr;
}

void TransitGraph::set_trip_block(const std::string& origin, const std::string& destination, const std::string& trip_id, bool block) {
    if (auto* trip = find_trip(origin, destination, trip_id)) {
        trip->is_blocked = block;
    }
}

void TransitGraph::apply_delay(const std::string& origin, const std::string& destination, const std::string& trip_id, int delay_seconds) {
    if (auto* trip = find_trip(origin, destination, trip_id)) {
        trip->departure_time += delay_seconds;
        trip->arrival_time += delay_seconds;
    }
}