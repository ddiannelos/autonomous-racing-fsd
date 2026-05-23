#pragma once

#include <vector>
#include <cmath>
#include <Eigen/Dense>

#include "../core/types.hpp"

/** @brief A utility class to generate smooth curves from a set of discrete points using Uniform Cubic B-Splines. */
class BSplineGenerator {
public:
    /**
     * @brief Generates a smoothed B-Splin path from a given set of points
     * @param control_points The raw, jagged points (must have at least 4)
     * @param resolution The number of interpolated points to generate between each set of points, Higher = smoother
     * @return std::vector<Point> The smoothed trajectory
     */
    static std::vector<Point> generate_spline(const std::vector<Point> &control_points, int resolution = 10) {
        std::vector<Point> smoothed_path;

        // Must have at least 4 points
        if (control_points.size() < 4) return control_points;

        // Duplicate the first points to start the spline from the starting position
        std::vector<Point> padded_points;
        padded_points.push_back(control_points.front());
        padded_points.push_back(control_points.front());

        // Add the actual points in the middle
        for (const auto &p : control_points) padded_points.push_back(p);

        // Duplicate the end points to end the spline to the ending position
        padded_points.push_back(control_points.back());
        padded_points.push_back(control_points.back());

        for (size_t i = 1; i < padded_points.size() - 2; i++) {
            Point p0 = padded_points[i-1];
            Point p1 = padded_points[i];
            Point p2 = padded_points[i + 1];
            Point p3 = padded_points[i + 2];

            // Intrpolate 'resolution' number of points within this segment
            for (int step = 0; step < resolution; step++) {
                // 't' goes from 0.0 to 1.0 represetning the progress along the current segment
                double t = static_cast<double>(step) / resolution;

                // Calculate the basis functions
                double b0 = (1.0 / 6.0) * std::pow(1.0 - t, 3);
                double b1 = (1.0 / 6.0) * (3.0 * std::pow(t, 3) - 6.0 * std::pow(t, 2) + 4.0);
                double b2 = (1.0 / 6.0) * (-3.0 * std::pow(t, 3) + 3.0 * std::pow(t, 2) + 3.0 * t + 1.0);
                double b3 = (1.0 / 6.0) * std::pow(t, 3);

                // Apply the weights to the X and Y coordinates to find the exact point on the curve
                Point spline_point;
                spline_point.x = b0 * p0.x + b1 * p1.x + b2 * p2.x + b3 * p3.x;
                spline_point.y = b0 * p0.y + b1 * p1.y + b2 * p2.y + b3 * p3.y;
                spline_point.color = ConeColor::UNKNOWN;

                smoothed_path.push_back(spline_point);
            }
        }

        // Add the absolute final point
        smoothed_path.push_back(control_points.back());

        return smoothed_path;
    }


    /**
     * @brief Generates a smoothed B-Splin path from a given set of Eigen::Vector2d
     * @param control_points The raw, jagged points (must have at least 4)
     * @param resolution The number of interpolated points to generate between each set of points, Higher = smoother
     * @return std::vector<Eigen::Vector2d> The smoothed trajectory
     */
    static std::vector<Eigen::Vector2d> generate_spline(const std::vector<Eigen::Vector2d> &control_points, int resolution = 10) {
        std::vector<Eigen::Vector2d> smoothed_path;

        // Must have at least 4 points to form a cubic spline
        if (control_points.size() < 4) return control_points;

        int N = control_points.size();

        // Iterate through every point to create a continuous closed loop
        for (int i = 0; i < N; i++) {
            // Wrap the indices using modulo (% N) so the end connects seamlessly to the start
            Eigen::Vector2d p0 = control_points[(i - 1 + N) % N];
            Eigen::Vector2d p1 = control_points[i];
            Eigen::Vector2d p2 = control_points[(i + 1) % N];
            Eigen::Vector2d p3 = control_points[(i + 2) % N];

            // Interpolate 'resolution' number of points within this segment
            for (int step = 0; step < resolution; step++) {
                // 't' goes from 0.0 to 1.0 representing the progress along the current segment
                double t = static_cast<double>(step) / resolution;

                // Calculate the basis functions
                double b0 = (1.0 / 6.0) * std::pow(1.0 - t, 3);
                double b1 = (1.0 / 6.0) * (3.0 * std::pow(t, 3) - 6.0 * std::pow(t, 2) + 4.0);
                double b2 = (1.0 / 6.0) * (-3.0 * std::pow(t, 3) + 3.0 * std::pow(t, 2) + 3.0 * t + 1.0);
                double b3 = (1.0 / 6.0) * std::pow(t, 3);

                // Eigen allows us to multiply the scalar weights directly against the vectors!
                Eigen::Vector2d spline_point = (b0 * p0) + (b1 * p1) + (b2 * p2) + (b3 * p3);

                smoothed_path.push_back(spline_point);
            }
        }

        return smoothed_path;
    }
};