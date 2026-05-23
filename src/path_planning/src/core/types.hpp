#pragma once

#include <cmath>
#include <vector>


/** @brief Represents the 2D position of the vehicle */
struct VehiclePose {
    double x;
    double y;
    double yaw; // The heading of the vehicle in radians
};

/**
 * @brief Represents the color of a cone
 * Blue = left boundary, Yellow = right boundary, Orange = Start/Finish
 */
enum class ConeColor {
    YELLOW,
    BLUE,
    ORANGE,
    UNKNOWN
};

/** @brief Represents a 2D coordinate for a cone on the track */
struct Point {
    double x;
    double y;
    ConeColor color;

    // Checks if two points are identical
    bool operator==(const Point &p) const { return std::abs(x - p.x) < 1e-6 && std::abs(y-p.y) < 1e-6; }
};

/** @brief Represents an edge connecting two points */
struct Edge {
    Point a;
    Point b;

    // Checks if two edges connect the same points, regardless of direction
    bool operator==(const Edge &e) const { return (this->a == e.a && this->b == e.b) ||
                                                  (this->b == e.a && this->a == e.b); }
    // Calculates the Euclidean distance between two points
    double length() const { return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2)); }
};

/** @brief Represents a triangle formed by three points, including circumcircle properties */
struct Triangle {
    Point a;
    Point b;
    Point c;

    double circum_x;
    double circum_y;
    double circum_radius_sq;

    /** @brief Constructs a triangle and calculates its circumcenter and circumradius */
    Triangle(const Point &p1, const Point &p2, const Point &p3) : a(p1), b(p2), c(p3) {
        // Denominator for circumcenter formulas
        double D = 2 * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));

        // Handle collinear points (degenerate triangle)
        if (std::abs(D) < 1e-6) {
            circum_radius_sq = -1;
            return;
        }

        double a_sq = a.x * a.x + a.y * a.y;
        double b_sq = b.x * b.x + b.y * b.y;
        double c_sq = c.x * c.x + c.y * c.y;

        // Calculate circumcenter coordinates
        circum_x = (a_sq * (b.y - c.y) + b_sq * (c.y - a.y) + c_sq * (a.y - b.y)) / D;
        circum_y = (a_sq * (c.x - b.x) + b_sq * (a.x - c.x) + c_sq * (b.x - a.x)) / D;

        // Calculate squared circumradius
        circum_radius_sq = (a.x - circum_x) * (a.x - circum_x) + (a.y - circum_y) * (a.y - circum_y);
    }

    /** @brief Checks if a given point lies strictly inside the circumcircle of the triangle. */
    bool contains_in_circumcircle(const Point &p) const {
        double dist_sq = (p.x - circum_x) * (p.x - circum_x) + (p.y - circum_y) * (p.y - circum_y);
        return dist_sq <= circum_radius_sq;
    }

    /** @brief Checks if a given point is one of the vertices of the triangle */
    bool contains_vertex(const Point &p) const {
        return a == p || b == p || c == p;
    }
};

/** @brief Represents the Control command send to the actuator */
struct ControlCommand {
    double steering;
    double throttle;
    double brake;
    Point lookahead_point;              // Point for visualization
    double lookahead_radius;            // Double for visualization
    std::vector<Point> mpc_trajectory;  // Path for visualization
};
