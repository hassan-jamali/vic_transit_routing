#pragma once
#include "transit_graph.h"
#include <string>
#include <vector>

struct RouteResult {
    bool is_found;
    int arrival_time;
    std::vector<std::string> path;
};

struct QueueNode {
    std::string node_id;
    int time_seconds;
    double f_score;
    // min heap comparison (smallest f_score comes first)
    bool operator>(const QueueNode& other) const {
        return f_score > other.f_score;
    }
};

class AStarRouter {
public:
    static RouteResult find_route(const TransitGraph& graph, const std::string& start_id, const std::string& goal_id, int start_time_seconds, bool use_gpu = false);
};