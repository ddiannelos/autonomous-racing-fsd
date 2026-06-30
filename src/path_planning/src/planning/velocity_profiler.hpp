#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>

using Eigen::Vector2d;

class VelocityProfiler {
private:
    std::vector<double> speed_profile;

    /** @brief Calculates the Menger curvature of three points (Curvature = 1 / Radius) */
    double calculate_curvature(const Vector2d &p1, const Vector2d &p2, const Vector2d &p3);

    /** @brief Finds a point on the track at a specific distance away  */
    int get_spaced_index(const std::vector<Vector2d> &path, int start_idx, double target_dist, int direction);
public:
    VelocityProfiler() = default;

    /**
     * @brief Computes the maximum safe speed for every point on the track
     * @param mu Coefficient of friction between tires and asphalt
     * @param max_speed Absolute top speed (m/s)
     * @param max_decel Maximum braking capability (m/s^2)
     */
    std::vector<double> generate_profile(const std::vector<Vector2d> &path, double mu = 0.8, double max_speed = 20.0, double max_decel = 8.0);
};

double VelocityProfiler::calculate_curvature(const Vector2d &p1, const Vector2d &p2, const Vector2d &p3) {
    // Area of the triangle formed by the three points
    double area = 0.5 * std::abs(p1.x() * (p2.y() - p3.y()) + p2.x() * (p3.y() - p1.y()) + p3.x() * (p1.y() - p2.y()));

    // Length of the sides of the triangle
    double a = (p1 - p2).norm();
    double b = (p2 - p3).norm();
    double c = (p3 - p1).norm();

    if (a == 0 || b == 0 || c == 0) return 0.0;

    // Menger Curvature formula
    return (4.0 * area) / (a * b * c);
}

int VelocityProfiler::get_spaced_index(const std::vector<Vector2d> &path, int start_idx, double target_dist, int direction) {
    int N = path.size();
    double dist = 0.0;
    int curr = start_idx;

    for (int i = 0; i < 30; i++) { // Look up to 30 points away
        int next = (curr + direction + N) % N;
        dist += (path[next] - path[curr]).norm();
        curr = next;
        if (dist >= target_dist) break;
    }
    return curr;
}

std::vector<double> VelocityProfiler::generate_profile(const std::vector<Vector2d> &path, double mu, double max_speed, double max_decel) {
    int N = path.size();
    if (N < 3) return {};

    speed_profile.assign(N, max_speed);
    double gravity = 9.81;
    double max_accel = 8.0;

    // The absolute maximum total acceleration the tires can handle (Friction Circle radius)
    // The total acceleration the tires can handle
    double max_grip = mu * gravity;

    // Calculate max speed through corners (Lateral limit)
    for (int i = 0; i < N; i++) {
        int prev = get_spaced_index(path, i, 1.5, -1);
        int next = get_spaced_index(path, i, 1.5, 1);

        double kappa = calculate_curvature(path[prev], path[i], path[next]);
        kappa = std::max(kappa, 1e-6); // Prevent divide by zero

        // Limit speed based on lateral centripetal friction v = sqrt(mu * g / kappa)
        double v_corner = std::sqrt(max_grip / kappa);

        speed_profile[i] = std::min(max_speed, v_corner);
    }

    // Moving Average Filter
    std::vector<double> smoothed_profile(N);
    int window = 4; // Average across 9 points total (4 ahead, 4 behind)
    for (int i = 0; i < N; i++) {
        double sum = 0.0;
        for (int j = -window; j <= window; j++) {
            int idx = (i + j + N) % N;
            sum += speed_profile[idx];
        }
        smoothed_profile[i] = sum / (2 * window + 1);
    }
    speed_profile = smoothed_profile;

    // Ensure deceleration/acceleration is physically possible (Friction Circle limits)
    for (int loop = 0; loop < 2; loop++) {

        // BACKWARD PASS: Safe Braking into corners
        for (int i = N - 1; i >= 0; i--) {
            int next = (i + 1) % N;
            double distance = (path[i] - path[next]).norm();

            // Calculate curvature at the 'next' point to find lateral grip usage
            int next_plus_one = (next + 1) % N;
            double kappa = calculate_curvature(path[i], path[next], path[next_plus_one]);

            // a_y = v^2 * kappa
            double a_y = std::pow(speed_profile[next], 2) * kappa;

            // Calculate remaining grip for deceleration
            double available_decel = 0.0;
            if (a_y < max_grip) {
                available_decel = std::sqrt(std::pow(max_grip, 2) - std::pow(a_y, 2));
            }

            // Cap it to the physical limit of your brakes
            available_decel = std::min(available_decel, max_decel);

            // Kinematic equation: v_f^2 = v_i^2 + 2ad
            double safe_braking_speed = std::sqrt(std::pow(speed_profile[next], 2) + (2.0 * available_decel * distance));
            speed_profile[i] = std::min(speed_profile[i], safe_braking_speed);
        }

        // FORWARD PASS: Safe Acceleration out of corners
        for (int i = 0; i < N; i++) {
            int prev = (i - 1 + N) % N;
            double distance = (path[i] - path[prev]).norm();

            // Calculate curvature at the 'prev' point to find lateral grip usage
            int prev_minus_one = (prev - 1 + N) % N;
            double kappa = calculate_curvature(path[prev_minus_one], path[prev], path[i]);

            // a_y = v^2 * kappa
            double a_y = std::pow(speed_profile[prev], 2) * kappa;

            // Calculate remaining grip for acceleration
            double available_accel = 0.0;
            if (a_y < max_grip) {
                available_accel = std::sqrt(std::pow(max_grip, 2) - std::pow(a_y, 2));
            }

            // Cap it to the physical limit of the powertrain
            available_accel = std::min(available_accel, max_accel);

            double safe_accel_speed = std::sqrt(std::pow(speed_profile[prev], 2) + (2.0 * available_accel * distance));
            speed_profile[i] = std::min(speed_profile[i], safe_accel_speed);
        }
    }

    return speed_profile;
}