#pragma once

#include <vector>
#include <cmath>
#include <Eigen/Dense>

#include "../core/types.hpp"
#include "line_optimizer.hpp"
#include "../control/mpc_controller.hpp"
#include "velocity_profiler.hpp"
#include "../math/bspline.hpp"

/**
 * @brief Handles the Lap 2+
 * It calculates the global optimal racing line, physical speed limits, and uses
 * an MPC controller to push the vehicle to those limits.
 */
class OptimizationPlanner {
private:
    PathOptimizer optimizer;
    VelocityProfiler profiler;
    MPCController mpc;

    // Store track limits computed at the start of the lap 2
    std::vector<Eigen::Vector2d> optimized_path;
    std::vector<double> speed_profile;
    bool is_ready = false;
public:
    OptimizationPlanner() = default;

    /**
     * @brief Computes the global trajectory
     * @param final_slam_gates The closed-loop track boundaries from the ExplorationPlanner
     */
    void initialize_track(const std::vector<Edge> &final_slam_gates, const VehiclePose &start_pos);

    /** @brief Control loop utilizing the MPC */
    ControlCommand compute_command(const VehiclePose &current_pose, double current_speed);
};

void OptimizationPlanner::initialize_track(const std::vector<Edge> &final_slam_gates, const VehiclePose &start_pos) {
    Eigen::Vector2d start(start_pos.x, start_pos.y);

    // Calculate the optimized path
    optimizer.load_gates(final_slam_gates, start_pos, 1.6, 0.5);
    optimizer.calculate_optimized_path();

    // Generate smooth points using a Catmull-Rom spline
    // optimized_path = CatmullRomGenerator::generate_closed_spline(optimizer.get_optimized_path());
    optimized_path = BSplineGenerator::generate_spline(optimizer.get_optimized_path());

    // Compute absolute speed limits
    speed_profile = profiler.generate_profile(optimized_path, 0.6, 14.0, 12.0);

    MPCState initial_state{start_pos.x, start_pos.y, start_pos.yaw, 0.0};
    mpc.init(initial_state, optimized_path, speed_profile);

    is_ready = true;
}

ControlCommand OptimizationPlanner::compute_command(const VehiclePose &current_pose, double current_speed) {
    // Brake if planner is called before the initialization
    if (!is_ready) return {0.0, 0.0, 1.0, {0.0, 0.0, ConeColor::UNKNOWN}, 0.0, {}};

    MPCState current_state{current_pose.x, current_pose.y, current_pose.yaw, current_speed};

    // Take the control output from the MPC
    MPCResult optimal_cmd = mpc.solve(current_state, optimized_path, speed_profile);
    ControlCommand cmd = {0.0, 0.0, 0.0, {0.0, 0.0, ConeColor::UNKNOWN}, 0.0, {}};

    // Take the steering from the MPC
    cmd.steering = -optimal_cmd.delta/1.22;

    // Normalize acceleration from the MPC
    if (optimal_cmd.accel > 0.05) {
        cmd.throttle = std::min(1.0, optimal_cmd.accel/8.0);
        cmd.brake = 0.0;
    } else if (optimal_cmd.accel < -0.25) {
        cmd.throttle = 0.0;
        cmd.brake = std::min(1.0, std::abs(optimal_cmd.accel)/12.0);
    }

    // Export the MPC's predicted path for visualization
    for (const auto &p : optimal_cmd.predicted_path) cmd.mpc_trajectory.push_back({p.x(), p.y(), ConeColor::UNKNOWN});

    return cmd;
}