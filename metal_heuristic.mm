#include "metal_heuristic.h"
#import <Metal/Metal.h>
#import <Foundation/Foundation.h>
#include <iostream>

namespace MetalHeuristic {

    // metal shader source embedded directly as a string
    static NSString* const kBatchShaderSource = @R"(
        #include <metal_stdlib>
        using namespace metal;

        kernel void calculate_haversine_batch(
            device const double* node_latitudes [[buffer(0)]],
            device const double* node_longitudes [[buffer(1)]],
            device const double* goal_latitudes [[buffer(2)]],
            device const double* goal_longitudes [[buffer(3)]],
            device double* heuristic_matrix [[buffer(4)]],
            constant uint& num_goals [[buffer(5)]],
            uint2 thread_position [[thread_position_in_grid]]
        ) {
            uint node_index = thread_position.x;
            uint goal_index = thread_position.y;

            double earth_radius_meters = 6371000.0;
            double degrees_to_radians = M_PI_F / 180.0;

            double latitude_1 = node_latitudes[node_index] * degrees_to_radians;
            double longitude_1 = node_longitudes[node_index] * degrees_to_radians;
            double latitude_2 = goal_latitudes[goal_index] * degrees_to_radians;
            double longitude_2 = goal_longitudes[goal_index] * degrees_to_radians;

            double delta_latitude = latitude_2 - latitude_1;
            double delta_longitude = longitude_2 - longitude_1;

            double a = sin(delta_latitude / 2.0) * sin(delta_latitude / 2.0) +
                       cos(latitude_1) * cos(latitude_2) *
                       sin(delta_longitude / 2.0) * sin(delta_longitude / 2.0);

            double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));

            uint output_index = node_index * num_goals + goal_index;
            heuristic_matrix[output_index] = earth_radius_meters * c;
        }
    )";

    struct MetalContext {
        id<MTLDevice> device = nil;
        id<MTLCommandQueue> queue = nil;
        id<MTLComputePipelineState> batchPipelineState = nil;
    };

    static MetalContext& get_metal_context() {
        static MetalContext context = []() {
            MetalContext ctx;
            
            // firstly, initialize the default metal device hardware
            ctx.device = MTLCreateSystemDefaultDevice();
            if (!ctx.device) return ctx;
            
            ctx.queue = [ctx.device newCommandQueue];
            
            // in addition, compile the shader source library at runtime
            NSError* error = nil;
            id<MTLLibrary> library = [ctx.device newLibraryWithSource:kBatchShaderSource options:nil error:&error];
            if (!library) {
                std::cerr << "error compiling metal shader: " << [[error localizedDescription] UTF8String] << "\n";
                return ctx;
            }
            
            // moreover, create the compute pipeline state for batch processing
            id<MTLFunction> function = [library newFunctionWithName:@"calculate_haversine_batch"];
            ctx.batchPipelineState = [ctx.device newComputePipelineStateWithFunction:function error:&error];
            return ctx;
        }();
        return context;
    }

    std::vector<double> calculate_all_distances(
        const std::vector<double>& node_latitudes,
        const std::vector<double>& node_longitudes,
        double goal_latitude,
        double goal_longitude
    ) {
        // firstly, map single goal parameters into vectors
        std::vector<double> goal_latitudes_vector = {goal_latitude};
        std::vector<double> goal_longitudes_vector = {goal_longitude};
        
        // in addition, evaluate through the batch distance engine
        auto matrix = calculate_batch_distances(node_latitudes, node_longitudes, goal_latitudes_vector, goal_longitudes_vector);
        if (matrix.empty()) return std::vector<double>(node_latitudes.size(), 0.0);
        return matrix[0];
    }

    std::vector<std::vector<double>> calculate_batch_distances(
        const std::vector<double>& node_latitudes,
        const std::vector<double>& node_longitudes,
        const std::vector<double>& goal_latitudes,
        const std::vector<double>& goal_longitudes
    ) {
        size_t node_count = node_latitudes.size();
        size_t goal_count = goal_latitudes.size();
        
        std::vector<std::vector<double>> results(goal_count, std::vector<double>(node_count, 0.0));
        if (node_count == 0 || goal_count == 0) return results;

        MetalContext& ctx = get_metal_context();
        if (!ctx.device || !ctx.batchPipelineState) return results;

        size_t node_buffer_size = node_count * sizeof(double);
        size_t goal_buffer_size = goal_count * sizeof(double);
        size_t matrix_buffer_size = node_count * goal_count * sizeof(double);

        // firstly, allocate shared memory buffers for nodes and goals
        id<MTLBuffer> buffer_node_latitudes = [ctx.device newBufferWithBytes:node_latitudes.data() length:node_buffer_size options:MTLResourceStorageModeShared];
        id<MTLBuffer> buffer_node_longitudes = [ctx.device newBufferWithBytes:node_longitudes.data() length:node_buffer_size options:MTLResourceStorageModeShared];
        id<MTLBuffer> buffer_goal_latitudes = [ctx.device newBufferWithBytes:goal_latitudes.data() length:goal_buffer_size options:MTLResourceStorageModeShared];
        id<MTLBuffer> buffer_goal_longitudes = [ctx.device newBufferWithBytes:goal_longitudes.data() length:goal_buffer_size options:MTLResourceStorageModeShared];
        id<MTLBuffer> buffer_heuristic_matrix = [ctx.device newBufferWithLength:matrix_buffer_size options:MTLResourceStorageModeShared];

        uint32_t num_goals_val = static_cast<uint32_t>(goal_count);

        // in addition, set up the command encoder and thread configurations
        id<MTLCommandBuffer> command_buffer = [ctx.queue commandBuffer];
        id<MTLComputeCommandEncoder> compute_encoder = [command_buffer computeCommandEncoder];

        [compute_encoder setComputePipelineState:ctx.batchPipelineState];
        [compute_encoder setBuffer:buffer_node_latitudes offset:0 atIndex:0];
        [compute_encoder setBuffer:buffer_node_longitudes offset:0 atIndex:1];
        [compute_encoder setBuffer:buffer_goal_latitudes offset:0 atIndex:2];
        [compute_encoder setBuffer:buffer_goal_longitudes offset:0 atIndex:3];
        [compute_encoder setBuffer:buffer_heuristic_matrix offset:0 atIndex:4];
        [compute_encoder setBytes:&num_goals_val length:sizeof(uint32_t) atIndex:5];

        MTLSize grid_size = MTLSizeMake(node_count, goal_count, 1);
        NSUInteger width = ctx.batchPipelineState.threadExecutionWidth;
        NSUInteger height = ctx.batchPipelineState.maxTotalThreadsPerThreadgroup / width;
        MTLSize thread_group_dimensions = MTLSizeMake(width, height, 1);

        [compute_encoder dispatchThreads:grid_size threadsPerThreadgroup:thread_group_dimensions];
        [compute_encoder endEncoding];

        // moreover, commit work and wait for parallel execution to finish
        [command_buffer commit];
        [command_buffer waitUntilCompleted];

        // finally, extract computed matrix results back into container structures
        double* matrix_results = (double*)[buffer_heuristic_matrix contents];
        for (size_t goal_index = 0; goal_index < goal_count; ++goal_index) {
            for (size_t node_index = 0; node_index < node_count; ++node_index) {
                results[goal_index][node_index] = matrix_results[node_index * goal_count + goal_index];
            }
        }

        return results;
    }
}