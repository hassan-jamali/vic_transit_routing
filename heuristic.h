#pragma once
#include "transit_graph.h"
#include <string>

class Heuristic {
public:
    static double calculate_distance(double latitude_one, double longitude_one, double latitude_two, double longitude_two);
    static int estimate_time(const TransitGraph& graph, const std::string& current_id, const std::string& goal_id);
};