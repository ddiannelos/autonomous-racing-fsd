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
 * @brief Configuration struct for Optimization tuning parameters
 */
struct OptimizationConfig {
    double car_width = 1.6;
    double safety_margin = 0.5;
    double friction_mu = 0.6;
    double max_speed = 14.0;
    double max_accel = 8.0;
    double max_decel = 12.0;
    double max_steer = 1.22;
    double wheelbase = 1.6;

    // MPC Controller Weights
    int mpc_N = 10;
    double mpc_dt = 0.1;
    double q_x = 250.0;
    double q_y = 250.0;
    double q_yaw = 100.0;
    double q_v = 50.0;
    double r_steer = 100.0;
    double r_accel = 10.0;
    double r_steer_rate = 20000.0;
    double r_accel_rate = 25.0;
};

/**
 * @brief Handles the Lap 2+
 * It calculates the global optimal racing line, physical speed limits, and uses
 * an MPC controller to push the vehicle to those limits.
 */
class OptimizationPlanner {
private:
    OptimizationConfig config_;
    PathOptimizer optimizer;
    VelocityProfiler profiler;
    MPCController mpc;

    // Store track limits computed at the start of the lap 2
    std::vector<Eigen::Vector2d> optimized_path;
    std::vector<double> speed_profile;
    bool is_ready = false;
public:
    OptimizationPlanner(const OptimizationConfig& config = OptimizationConfig()) : config_(config) {
        // Pass the YAML limits directly into the MPC engine
        mpc.configure(config.mpc_N, config.mpc_dt, config.wheelbase,
                      config.max_steer, config.max_accel, config.max_decel,
                      config.q_x, config.q_y, config.q_yaw, config.q_v,
                      config.r_steer, config.r_accel,
                      config.r_steer_rate, config.r_accel_rate);
    }

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

    // Calculate the optimized path using physical parameter boundaries
    optimizer.load_gates(final_slam_gates, start_pos, config_.car_width, config_.safety_margin);
    optimizer.calculate_optimized_path();

    // Generate smooth points using a B-spline
    optimized_path = BSplineGenerator::generate_spline(optimizer.get_optimized_path());

    // Compute absolute speed limits using physical grip limits
    speed_profile = profiler.generate_profile(optimized_path, config_.friction_mu, config_.max_speed, config_.max_decel);

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

    // Normalize the steering using the physical maximum steer
    cmd.steering = -optimal_cmd.delta / config_.max_steer;

    // Normalize acceleration and braking outputs from the MPC
    if (optimal_cmd.accel > 0.05) {
        cmd.throttle = std::min(1.0, optimal_cmd.accel / config_.max_accel);
        cmd.brake = 0.0;
    } else if (optimal_cmd.accel < -0.25) {
        cmd.throttle = 0.0;
        cmd.brake = std::min(1.0, std::abs(optimal_cmd.accel) / config_.max_decel);
    }

    // Export the MPC's predicted path for visualization
    for (const auto &p : optimal_cmd.predicted_path) cmd.mpc_trajectory.push_back({p.x(), p.y(), ConeColor::UNKNOWN});

    return cmd;
}