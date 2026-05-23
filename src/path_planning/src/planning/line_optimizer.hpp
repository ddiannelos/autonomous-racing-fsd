#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "../core/types.hpp"

using Eigen::Vector2d;

/** @brief Represents a lateral slice of the track between a left and right cone */
struct OptimizationGate {
    Vector2d midpoint;
    Vector2d lateral_vector;
    double max_shift;
    double shift;
};

/** @brief Calculates the racing line trajectory within the track boundaries */
class PathOptimizer {
private:
    std::vector<OptimizationGate> gates;
    std::vector<Vector2d> optimized_path;

    /** @brief Takes an unordered track edges and sorts them sequentially based on distance and direction heading */
    std::vector<Edge> sort_gates_sequentially(const std::vector<Edge> &raw_gates, const VehiclePose &start_pose);
public:
    PathOptimizer() = default;

    /** @brief Formats raw track boundaries into optimization gates */
    void load_gates(const std::vector<Edge> &raw_gates, const VehiclePose &start_pose, double car_width = 1.5, double safety_margin = 0.3);

    /** @brief Iterative smooths the path by pulling points toward the average of their neighbor */
    void calculate_optimized_path(int iterations = 1000, double w_centerline = 0.2, double alpha = 0.1);


    /** @brief Get the optimized path */
    std::vector<Vector2d> get_optimized_path() const { return optimized_path; }
};

std::vector<Edge> PathOptimizer::sort_gates_sequentially(const std::vector<Edge> &raw_gates, const VehiclePose &start_pose) {
    if (raw_gates.empty()) return {};

    std::vector<Edge> sorted;
    std::vector<bool> visited(raw_gates.size(), false);

    Vector2d start_pos(start_pose.x, start_pose.y);
    Vector2d car_dir(std::cos(start_pose.yaw), std::sin(start_pose.yaw));

    int best_start_idx = 0;
    double min_start_score = 1e9;

    // Find the gate closest to the vehicle that is in front of it
    for (size_t i = 0; i < raw_gates.size(); i++) {
        Vector2d mid = { (raw_gates[i].a.x + raw_gates[i].b.x) / 2.0,
                            (raw_gates[i].a.y + raw_gates[i].b.y) / 2.0};

        Vector2d to_gate = mid - start_pos;

        double dist = to_gate.norm();
        double proj = to_gate.normalized().dot(car_dir);

        // Penalize gates that are behind the car's heading
        double score = dist + (proj < 0.2 ? 10000.0 : 0.0);

        if (score < min_start_score) {
            min_start_score = score;
            best_start_idx = i;
        }
    }
    sorted.push_back(raw_gates[best_start_idx]);
    visited[best_start_idx] = true;

    Vector2d current_mid = { (sorted.back().a.x + sorted.back().b.x) / 2.0,
                                (sorted.back().a.y + sorted.back().b.y) / 2.0 };

    // Step to the second gate using the car's initial direction
    int second_idx = -1;
    double min_second_score = 1e9;

    for (size_t j = 0; j < raw_gates.size(); j++) {
        if (!visited[j]) {
            Vector2d candidate_mid = { (raw_gates[j].a.x + raw_gates[j].b.x) / 2.0,
                                        (raw_gates[j].a.y + raw_gates[j].b.y) / 2.0 };
            Vector2d to_candidate = candidate_mid - current_mid;
            double dist = to_candidate.norm();

            double proj = to_candidate.normalized().dot(car_dir);
            double score = dist + (proj < 0.2 ? 10000.0 : 0.0);

            if (score < min_second_score) {
                min_second_score = score;
                second_idx = j;
            }
        }
    }

    if (second_idx != -1) {
        sorted.push_back(raw_gates[second_idx]);
        visited[second_idx] = true;
    }

    // Connect the rest of the gates by following the track direction
    for (size_t i = 2; i < raw_gates.size(); i++) {
        current_mid = { (sorted.back().a.x + sorted.back().b.x) / 2.0,
                        (sorted.back().a.y + sorted.back().b.y) / 2.0 };
        Vector2d prev_mid = { (sorted[sorted.size()-2].a.x + sorted[sorted.size()-2].b.x) / 2.0,
                                (sorted[sorted.size()-2].a.y + sorted[sorted.size()-2].b.y) / 2.0 };

        // Dynamically calculate the track direction based on the last two gates
        Vector2d track_dir = (current_mid - prev_mid).normalized();

        double min_dist = 1e9;
        int best_idx = -1;
        for (size_t j = 0; j < raw_gates.size(); j++) {
            if (!visited[j]) {
                Vector2d candidate_mid = { (raw_gates[j].a.x + raw_gates[j].b.x) / 2.0,
                                            (raw_gates[j].a.y + raw_gates[j].b.y) / 2.0 };
                Vector2d to_candidate = candidate_mid - current_mid;
                double dist = to_candidate.norm();

                double proj = to_candidate.normalized().dot(track_dir);
                double score = dist + (proj < -0.2 ? 10000.0 : 0.0);

                if (score < min_dist) {
                    min_dist = score;
                    best_idx = j;
                }
            }
        }

        if (best_idx != -1) {
            sorted.push_back(raw_gates[best_idx]);
            visited[best_idx] = true;
        }
    }
    return sorted;
}

void PathOptimizer::load_gates(const std::vector<Edge> &raw_gates, const VehiclePose &start_pose, double car_width, double safety_margin) {
    gates.clear();
    optimized_path.clear();
    if (raw_gates.empty()) return;

    std::vector<Edge> sorted_edges = sort_gates_sequentially(raw_gates, start_pose);

    // Calculate the physical buffer to prevent the vehicle from hitting cones
    double required_buffer = (car_width / 2.0) + safety_margin;

    for (const auto &edge : sorted_edges) {
        OptimizationGate gate;

        // Enforce left-to-right vector orientation based on cone colors
        Vector2d left, right;
        if (edge.a.color == ConeColor::BLUE ||
            (edge.a.color == ConeColor::ORANGE && edge.b.color == ConeColor::YELLOW)) {
            left = Vector2d(edge.a.x, edge.a.y);
            right = Vector2d(edge.b.x, edge.b.y);
        } else {
            left = Vector2d(edge.b.x, edge.b.y);
            right = Vector2d(edge.a.x, edge.a.y);
        }

        gate.midpoint = (left + right) / 2.0;
        double track_width = (right - left).norm();

        gate.lateral_vector = (right - left).normalized();
        // Restrict how far the optized point can move off the centerline
        gate.max_shift = std::max(0.0, (track_width / 2.0) - required_buffer);
        gates.push_back(gate);
    }

    int N = gates.size();
    if (N >= 3) {
        for (int i = 0; i < N; i++) {
            int prev = (i - 1 + N) % N;
            int next = (i + 1) % N;

            // Calculate the general forward direction of the track through this gate
            Vector2d forward_dir = (gates[next].midpoint - gates[prev].midpoint).normalized();

            // Calculate the mathematical "Right" direction relative to forward: (y, -x)
            Vector2d expected_right(forward_dir.y(), -forward_dir.x());

            // If the gate's lateral vector points Left, flip it to point Right.
            // This guarantees 100% consistency, even across the start/finish array seam.
            if (gates[i].lateral_vector.dot(expected_right) < 0.0) {
                gates[i].lateral_vector = -gates[i].lateral_vector;
            }
        }
    }
}

void PathOptimizer::calculate_optimized_path(int iterations, double w_centerline, double alpha) {
    if (gates.size() < 3) return;
    int N = gates.size();

    // Tuning parameters
    int lookahead = 18;
    int future_offset = 4;
    double entry_weight = 6.0;  // Pulls the line to the outside before a turn
    double apex_weight = 4.0;   // Pulls the line to the inside during a turn
    double forward_bias = 0.5;  // Smooths the line by looking slightly ahead

    double sharp_turn_threshold = 0.3;
    double safe_w_centerline = std::min(w_centerline, 0.05);

    std::vector<double> gate_dynamic_w(N, 0.0);
    std::vector<Vector2d> target_centers(N);

    // Compute curvature biases
    // Detect where the "ideal" should be based on upcoming corners
    for (int i = 0; i < N; i++) {
        // Calculate localized 2D cross product for current curvature: $v_{in} \times v_{out}$
        Vector2d v_in = (gates[i].midpoint - gates[(i - 2 + N) % N].midpoint).normalized();
        Vector2d v_out = (gates[(i + 2) % N].midpoint - gates[i].midpoint).normalized();
        double local_curvature = (v_in.x() * v_out.y() - v_in.y() * v_out.x());

        // Lookahead to calculate the curvature of the upcoming track segment
        double future_curvature = 0.0;
        for (int k = 0; k < lookahead; k++) {
            int idx1 = (i + future_offset + k) % N;
            int idx2 = (i + future_offset + k + 1) % N;
            int idx3 = (i + future_offset + k + 2) % N;

            Vector2d v1 = (gates[idx2].midpoint - gates[idx1].midpoint).normalized();
            Vector2d v2 = (gates[idx3].midpoint - gates[idx2].midpoint).normalized();

            // Decay the weight of corners that are further away
            double weight = 1.0 - (static_cast<double>(k) / lookahead);
            future_curvature += (v1.x() * v2.y() - v1.y() * v2.x()) * weight;
        }

        // Noise gating
        if (std::abs(local_curvature) < 0.10) local_curvature = 0.0;
        if (std::abs(future_curvature) < 0.15) future_curvature = 0.0;

        // Decrease apex pulling on very sharp turns to prevent the car from cutting corners too deep
        double current_apex_weight = apex_weight;
        if (std::abs(local_curvature) > sharp_turn_threshold) current_apex_weight *= 0.3;

        // Corner entry logic, pull inside only if the vehicle is not deep in the corner
        double corner_depth = std::min(1.0, std::abs(local_curvature) * 12.0);
        double active_entry_weight = entry_weight * (1.0 - corner_depth);

        // Combine the future entry bias (pull outside) and local apex bias (pull inside)
        double combined_bias = (future_curvature * active_entry_weight) - (local_curvature * current_apex_weight);

        // Translate the bias into a physical lateral shift distance
        double bias_shift = combined_bias * gates[i].max_shift;
        bias_shift = std::clamp(bias_shift, -gates[i].max_shift, gates[i].max_shift);

        // Save the dynamic target for the optimization loop
        double active_center_pull = (std::abs(local_curvature) > 0.0) ? safe_w_centerline : 0.0;
        target_centers[i] = gates[i].midpoint + (gates[i].lateral_vector * bias_shift);
        gate_dynamic_w[i] = active_center_pull + std::abs(combined_bias) * 0.8;
    }

    std::vector<double> current_shifts(N, 0.0);
    for (int i = 0; i < N; i++) current_shifts[i] = gates[i].shift;

    // Smooth the targets before passing it to the optimizer
    std::vector<Vector2d> smoothed_targets(N);
    for (int i = 0; i < N; i++) {
        int prev = (i - 1 + N) % N;
        int next = (i + 1) % N;
        smoothed_targets[i] = (target_centers[prev] * 0.25) + (target_centers[i] * 0.5) + (target_centers[next] * 0.25);
    }
    target_centers = smoothed_targets;

    // Elastic band optimizer
    for (int iter = 0; iter < iterations; iter++) {
        std::vector<double> new_shifts(N, 0.0);
        double max_shift_change = 0.0;

        for (int i = 0; i < N; i++) {
            int prev = (i - 1 + N) % N;
            int next = (i + 1) % N;

            Vector2d p_prev = gates[prev].midpoint + (gates[prev].lateral_vector * current_shifts[prev]);
            Vector2d p_next = gates[next].midpoint + (gates[next].lateral_vector * current_shifts[next]);

            // Pulls the point straight between its neighbors
            Vector2d target_smooth = (p_prev * (1.0 - forward_bias)) + (p_next * forward_bias);

            // Blend the elastic target with the pre-calculated racing line target
            Vector2d target = (target_smooth + gate_dynamic_w[i] * target_centers[i]) / (1.0 + gate_dynamic_w[i]);

            // Calculate the error and apply the learning rate (alpha)
            Vector2d diff = target - gates[i].midpoint;
            double ideal_shift = diff.dot(gates[i].lateral_vector);
            double step = alpha * (ideal_shift - current_shifts[i]);

            new_shifts[i] = std::clamp(current_shifts[i] + step, -gates[i].max_shift, gates[i].max_shift);

            max_shift_change = std::max(max_shift_change, std::abs(step));
        }

        // Apply a 3-point moving average filter to smooth the shifts and prevent jitter
        for (int i = 0; i < N; i++) {
            int prev = (i - 1 + N) % N;
            int next = (i + 1) % N;
            // current_shifts[i] = 0.33 * new_shifts[prev] + 0.33 * new_shifts[i] + 0.33 * new_shifts[next];
            current_shifts[i] = 0.25 * new_shifts[prev] + 0.50 * new_shifts[i] + 0.25 * new_shifts[next];
        }

        // Early stopping if the path as converged
        if (max_shift_change < 0.001) break;
    }

    // Save the optimized path
    optimized_path.clear();
    for (int i = 0; i < N; i++) {
        gates[i].shift = current_shifts[i];
        optimized_path.push_back(gates[i].midpoint + (gates[i].lateral_vector * gates[i].shift));
    }
}