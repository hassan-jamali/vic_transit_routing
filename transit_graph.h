#pragma once
#include <string>
#include <vector>
#include <unordered_map>

struct Trip {
    std::string trip_id;
    int departure_time;
    int arrival_time;
    int travel_time;
    bool is_blocked = false; 
};

struct Node {
    std::string name;
    double latitude;
    double longitude;
};

class TransitGraph {
public:
    std::unordered_map<std::string, Node> nodes;
    std::unordered_map<std::string, std::unordered_map<std::string, std::vector<Trip>>> edges;

    bool load_from_json(const std::string& file_path);
    
    // dynamic edge updates
    void set_trip_block(const std::string& origin, const std::string& destination, const std::string& trip_id, bool block);
    void apply_delay(const std::string& origin, const std::string& destination, const std::string& trip_id, int delay_seconds);
    
private:
    // helper to find a trip
    Trip* find_trip(const std::string& origin, const std::string& destination, const std::string& trip_id);
};