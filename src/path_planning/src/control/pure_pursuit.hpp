#pragma once

#include <vector>
#include <cmath>
#include <limits>

#include "../core/types.hpp"


/**
 * @brief Implements hte Pure Pursuit path tracking algorithm
 * * Pure Pursuit is a geometric path tracking controller. It works by finding a "target point"
 * on the desired path that is a specific distance (lookahead distance) away from the vehicle, and
 * then calculating the steering angle required to drive the vehicle's rear axle to that point
 */
class PurePursuit {
private:
    double wheelbase;           /**< The distance between the front and the rear axles (L) */
    double lookahead_distance;  /**< The search radius around the car to find the target point */

    Point last_target_point;    /**< The last point found, used for visualization */

    /** @brief Finds the optimal target point on the path for the car to drive towards */
    Point get_target_point(const VehiclePose &car_pose, const std::vector<Point> &path);
public:
    /**
     * @brief Constructor for the PurePursuit Controller
     * @param L The wheelbase of the vehicle
     * @param ld The lookahead distance
     */
    PurePursuit(double L, double ld) : wheelbase(L), lookahead_distance(ld) { last_target_point = {0.0, 0.0, ConeColor::UNKNOWN}; }

    /**
     * @brief Calculates the steering angle required to track the path
     * @param car_pose The current position and heading of the vehicle
     * @param path The smoothed centerline path to follow
     * @return The steering angle in radians
     */
    double calculate_steering(const VehiclePose &car_pose, const std::vector<Point> &path);

    /** @brief Returns the last lookahead point that was found. Used for visualization */
    Point get_last_target() const { return last_target_point; }

    /** @brief Returns the lookahead distance */
    double get_lookahead_distance() const { return lookahead_distance; }
};

Point PurePursuit::get_target_point(const VehiclePose &car_pose, const std::vector<Point> &path) {
    // Find the closest point on the path to the car (to prevent searching backwards)
    int closest_index = 0;
    double closest_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < path.size(); i++) {
        double dist = std::hypot(path[i].x - car_pose.x, path[i].y - car_pose.y);
        if (dist < closest_dist) {
            closest_dist = dist;
            closest_index = i;
        }
    }

    // Search forward for the exact circle-line intersection
    for (size_t i = closest_index; i < path.size() - 1; i++) {
        Point p1 = path[i];
        Point p2 = path[i+1];

        // Shift points so the car is at the origin (0, 0)
        double x1 = p1.x - car_pose.x;
        double y1 = p1.y - car_pose.y;
        double x2 = p2.x - car_pose.x;
        double y2 = p2.y - car_pose.y;

        double dx = x2 - x1;
        double dy = y2 - y1;

        // Quadratic equation coefficients
        double a = dx * dx + dy * dy;
        double b = 2.0 * (x1 * dx + y1 * dy);
        double c = (x1 * x1 + y1 * y1) - lookahead_distance * lookahead_distance;

        double discriminant = b * b - 4.0 * a * c;

        // If discriminant <0, the line misses the circle entirely
        if (discriminant >= 0.0) {
            // Solve for t
            double t1 = (-b - std::sqrt(discriminant)) / (2.0 * a);
            double t2 = (-b + std::sqrt(discriminant)) / (2.0 * a);

            // Collect valid intersections that lie on the line segment
            std::vector<Point> candidates;
            if (t1 >= 0.0 && t1 <= 1) {
                candidates.push_back({p1.x + t1 * dx, p1.y + t1 * dy, ConeColor::UNKNOWN});
            }
            if (t2 >= 0.0 && t2 <= 1.0) {
                candidates.push_back({p1.x + t2 * dx, p1.y + t2 * dy, ConeColor::UNKNOWN});
            }

            // If the segment crosses the circle twice take the furthest one
            if (!candidates.empty()) return candidates.back();
        }
    }

    // If there is no intersection return the last point
    return path.back();
}

double PurePursuit::calculate_steering(const VehiclePose &car_pose, const std::vector<Point> &path) {
    if (path.empty()) return 0.0;

    Point target_point = get_target_point(car_pose, path);

    last_target_point = target_point;

    // Transform target point to the car's local coordinate frame
    double dx = target_point.x - car_pose.x;
    double dy = target_point.y - car_pose.y;

    // 2D Rotation matrix to align with the car's heading
    double local_x = std::cos(-car_pose.yaw) * dx - std::sin(-car_pose.yaw) * dy;
    double local_y = std::sin(-car_pose.yaw) * dx + std::cos(-car_pose.yaw) * dy;

    // Distance to the target
    double distance_to_target = std::hypot(local_x, local_y);

    if (distance_to_target < 0.1) return 0.0;

    // Calculate steering angle delta
    double steering_angle = std::atan2(2.0 * wheelbase * local_y, distance_to_target * distance_to_target);

    return steering_angle;
}

