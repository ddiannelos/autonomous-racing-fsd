#pragma once

#include <vector>
#include <Eigen/Dense>

/**
 * @brief Generates a smooth continuous curve through a set of points
 * Catmull-Rom splines create a curve that will pass exactly through
 * every provided point.
 */
class CatmullRomGenerator {
public:
    /**
     * @brief Generates the spline points. The track is assumed to be a close loop
     * @param points The original cotnrol points
     * @param points_per_segment The resolution of the interpolation
     */
    static std::vector<Eigen::Vector2d> generate_closed_spline(const std::vector<Eigen::Vector2d>& points, int points_per_segment = 10) {
        std::vector<Eigen::Vector2d> smooth_path;
        // Requires at least 4  points
        if (points.size() < 4) return points;

        int N = points.size();
        for (int i = 0; i < N; i++) {
            Eigen::Vector2d p0 = points[(i - 1 + N) % N]; // Previous Point
            Eigen::Vector2d p1 = points[i];               // Current point
            Eigen::Vector2d p2 = points[(i + 1) % N];     // Next point
            Eigen::Vector2d p3 = points[(i + 2) % N];     // Point after the next

            // Interpolate points between p1-p2
            for (int j = 0; j < points_per_segment; j++) {
                double t = (double)j / points_per_segment;
                double t2 = t * t;
                double t3 = t2 * t;

                // Catmull-Rom cubic polynomial matrix multiplication
                Eigen::Vector2d pt = 0.5 * (
                    (2.0 * p1) +
                    (-p0 + p2) * t +
                    (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2 +
                    (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3
                );
                smooth_path.push_back(pt);
            }
        }
        return smooth_path;
    }
};