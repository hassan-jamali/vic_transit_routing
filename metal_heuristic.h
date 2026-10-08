#pragma once
#include <vector>

namespace MetalHeuristic {
    std::vector<double> calculate_all_distances(
        const std::vector<double>& node_latitudes,
        const std::vector<double>& node_longitudes,
        double goal_latitude,
        double goal_longitude
    );

    // batch processes multiple goal destinations simultaneously
    std::vector<std::vector<double>> calculate_batch_distances(
        const std::vector<double>& node_latitudes,
        const std::vector<double>& node_longitudes,
        const std::vector<double>& goal_latitudes,
        const std::vector<double>& goal_longitudes
    );
}