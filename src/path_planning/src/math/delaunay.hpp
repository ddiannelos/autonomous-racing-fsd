#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>

#include "../core/types.hpp"

/** @brief A class to compute the Delaunay Triangulation of the track cones and extract path and boundary data */
class DelaunayTriangulator {
private:
    std::vector<Triangle> triangles;

    /**
     * @brief Computs the cosine of the angle between two vectors formed by a center point and two neighboring points
     * Used as a metric to find the straightest continuous lines during boundary pruning
     */
    double get_angle_metric(const Point &center, const Point &a, const Point &b) const;

    /** @brief Cleans up a set of boundary edges to remove unwanted branches and enforce continuous lap */
    std::vector<Edge> prune_boundary_edges(const std::vector<Edge> &raw_edges) const;

    /** @brief Extracts raw boundary edges based on coor and distance */
    std::vector<Edge> get_boundary(ConeColor target_color, double max_edge_distance) const;
public:
    DelaunayTriangulator() = default;
    ~DelaunayTriangulator() = default;

    /**
     * @brief Computes the Delaunay Triangulation of a set of points using the Bowyer-Watson algorithm.
     * @param raw_points The unordered list of cones detected on the track
     */
    void triangulate(const std::vector<Point> &raw_points);

    /** @brief Returns the raw list of generated Delaunay triangulation */
    const std::vector<Triangle> &get_triangles() const;

    /**
     * @brief Extracts the centerline of the track.
     * It does this by finding edges that connect opposite side of the track and calculating their midpoints
     */
    const std::vector<Point> get_centerline(double max_edge_distance) const;

    /** @brief Extracts and prunes the left boundary */
    std::vector<Edge> get_left_boundary(double max_edge_distance) const;

    /** @brief Extracts and prunes the right boundary */
    std::vector<Edge> get_right_boundary(double max_edge_distance) const;

    /** @brief Extracts the cross-track edges (gates) connecting opposite sides of the track */
    std::vector<Edge> get_track_gates(double max_edge_distance) const;
};


double DelaunayTriangulator::get_angle_metric(const Point &center, const Point &a, const Point &b) const {
    double u_x = a.x - center.x;
    double u_y = a.y - center.y;
    double v_x = b.x - center.x;
    double v_y = b.y - center.y;

    double dot = u_x * v_x + u_y * v_y;
    double mag_u = std::sqrt(u_x * u_x + u_y * u_y);
    double mag_v = std::sqrt(v_x * v_x + v_y * v_y);
    if (mag_u == 0 || mag_v == 0) return 1.0;
    return dot / (mag_u * mag_v);
}

std::vector<Edge> DelaunayTriangulator::prune_boundary_edges(const std::vector<Edge> &raw_edges) const {
    std::vector<std::pair<Point, std::vector<Point>>> graph;

    // Build an adjacency list graph from the raw edges
    auto add_connection = [&graph](const Point &u, const Point &v) {
        bool found = false;
        for (auto &node : graph) {
            if (node.first == u) {
                if (std::find(node.second.begin(), node.second.end(), v) == node.second.end()) {
                    node.second.push_back(v);
                }
                found = true;
                break;
            }
        }
        if (!found) graph.push_back({u, {v}});
    };

    for (const auto &e : raw_edges) {
        add_connection(e.a, e.b);
        add_connection(e.b, e.a);
    }

    std::vector<Edge> rejected_edges;

    // Prune logic: Ensure no node has more than 2 connections (to form a simple line/curve)
    for (const auto &node : graph) {
        const Point &center = node.first;
        std::vector<Point> neighbors = node.second;

        // Resolve small triangular loops created by dense points
        while (neighbors.size() > 2) {
            bool triangle_found = false;
            for (size_t i = 0; i < neighbors.size(); i++) {
                for (size_t j = i + 1; j < neighbors.size(); j++) {
                    Point n1 = neighbors[i];
                    Point n2 = neighbors[j];
                    Edge test_edge = {n1, n2};

                    // If two neighbors are also connected to each other, drop the longest edge to the center
                    if (std::find(raw_edges.begin(), raw_edges.end(), test_edge) != raw_edges.end()) {
                        double dist1 = std::hypot(center.x - n1.x, center.y - n1.y);
                        double dist2 = std::hypot(center.x - n2.x, center.y - n2.y);

                        if (dist1 > dist2) {
                            Edge bad_edge = {center, n1};
                            if (std::find(rejected_edges.begin(), rejected_edges.end(), bad_edge) == rejected_edges.end()) {
                                rejected_edges.push_back(bad_edge);
                            }
                            neighbors.erase(neighbors.begin() + i);
                        } else {
                            Edge bad_edge = {center, n2};
                            if (std::find(rejected_edges.begin(), rejected_edges.end(), bad_edge) == rejected_edges.end()) {
                                rejected_edges.push_back(bad_edge);
                            }
                            neighbors.erase(neighbors.begin() + j);
                        }
                        triangle_found = true;
                        break;
                    }
                }
                if (triangle_found) break;
            }
            if (!triangle_found) break;
        }

        // If a node still has > 2 neighbors (a branch), keep the two that form the straightest line
        if (neighbors.size() > 2) {
            double min_cos = 2.0;
            Point best_n1 = neighbors[0];
            Point best_n2 = neighbors[1];

            for (size_t i = 0; i < neighbors.size(); i++) {
                for (size_t j = i + 1; j < neighbors.size(); j++) {
                    double cos_theta = get_angle_metric(center, neighbors[i], neighbors[j]);
                    if (cos_theta < min_cos) {
                        min_cos = cos_theta;
                        best_n1 = neighbors[i];
                        best_n2 = neighbors[j];
                    }
                }
            }

            // Reject all branches except the two best ones
            for (const auto &n : neighbors) {
                if (!(n == best_n1) && !(n == best_n2)) {
                    Edge bad_edge = {center, n};
                    if (std::find(rejected_edges.begin(), rejected_edges.end(), bad_edge) == rejected_edges.end()) {
                        rejected_edges.push_back(bad_edge);
                    }
                }
            }
        }
    }

    // // Return only the edges that were not rejected
    std::vector<Edge> clean_edges;
    for (const auto &e : raw_edges) {
        if (std::find(rejected_edges.begin(), rejected_edges.end(), e) == rejected_edges.end()) {
            clean_edges.push_back(e);
        }
    }

    return clean_edges;
}

std::vector<Edge> DelaunayTriangulator::get_boundary(ConeColor target_color, double max_edge_distance) const {
    std::vector<Edge> boundary_edges;

    ConeColor opposite_color = (target_color == ConeColor::BLUE) ? ConeColor::YELLOW : ConeColor::BLUE;

    // Lambda function to find distance from a point to the nearest cone of a given color
    auto dist_to_closest_color = [&](const Point &p, ConeColor c) {
        double min_dist = std::numeric_limits<double>::max();
        for (const auto &t : triangles) {
            const Point *pts[3] = {&t.a, &t.b, &t.c};
            for (int k = 0; k < 3; k++) {
                if (pts[k]->color == c) {
                    double dist = std::hypot(p.x - pts[k]->x, p.y - pts[k]->y);
                    if (dist < min_dist) min_dist = dist;
                }
            }
        }
        return min_dist;
    };

    // Inspect all edges in the triangulation
    for (const auto &t : triangles) {
        Edge edges[3] = { {t.a, t.b}, {t.b, t.c}, {t.c, t.a} };

        for (int i = 0; i < 3; i++) {
            const Edge &e = edges[i];

            bool is_boundary = false;

            // Condition 1: Edge connects two cones of the target color
            // Condition 2: Edge connects target color to an orange cone (short edges only)
            // Condition 3: Edge connects two orange cones (very short edges only)
            if (e.a.color == target_color && e.b.color == target_color) {
                is_boundary = true;
            } else if (((e.a.color == target_color && e.b.color == ConeColor::ORANGE) ||
                        (e.a.color == ConeColor::ORANGE && e.b.color == target_color)) &&
                        (e.length() <= 5.0)) {
                // Assign orange cone to this boundary if it's closer to this color than the opposite
                Point orange_cone = (e.a.color == ConeColor::ORANGE) ? e.a : e.b;
                if (dist_to_closest_color(orange_cone, target_color) < dist_to_closest_color(orange_cone, opposite_color)) {
                    is_boundary = true;
                }
            } else if (e.a.color == ConeColor::ORANGE && e.b.color == ConeColor::ORANGE && e.length() <= 1.0) {
                // Assign to this boundary if it's physically closer to the target color side
                if (dist_to_closest_color(e.a, target_color) < dist_to_closest_color(e.a, opposite_color)) {
                    is_boundary = true;
                }
            }

            // Add to boundary list if valid, within distance limits and not a duplicate
            if (is_boundary && e.length() <= max_edge_distance) {
                auto it = std::find(boundary_edges.begin(), boundary_edges.end(), e);
                if (it == boundary_edges.end()) boundary_edges.push_back(e);
            }
        }
    }
    return boundary_edges;
}

void DelaunayTriangulator::triangulate(const std::vector<Point> &raw_points) {
    // Clean previous triangulation
    triangles.clear();

    // Filter out duplicate points that are too close to each other
    std::vector<Point> points;
    for (const auto &new_cone : raw_points) {
        bool is_duplicate = false;
        for (const auto &existing_cone : points) {
            double dist = std::hypot(new_cone.x - existing_cone.x, new_cone.y - existing_cone.y);
            if (dist < 0.3 && new_cone.color == existing_cone.color) {
                is_duplicate = true;
                break;
            }
        }
        if (!is_duplicate) points.push_back(new_cone);
    }

    // Create the Super-Triangle
    Point a_super = {-10000, -10000, ConeColor::UNKNOWN};
    Point b_super = {10000, -10000, ConeColor::UNKNOWN};
    Point c_super = {0, 10000, ConeColor::UNKNOWN};

    triangles.push_back(Triangle(a_super, b_super, c_super));

    // Insert points one by one
    for (const auto &point : points) {
        std::vector<Triangle> bad_triangles;

        // Find triangles whose circumcircle contains the point
        for (const auto &triangle : triangles) {
            if (triangle.contains_in_circumcircle(point)) bad_triangles.push_back(triangle);
        }

        // Find the boundary of the polygon hole
        std::vector<Edge> polygon;
        for (size_t i = 0; i < bad_triangles.size(); i++) {
            Edge edges[3] = {
                {bad_triangles[i].a, bad_triangles[i].b},
                {bad_triangles[i].b, bad_triangles[i].c},
                {bad_triangles[i].c, bad_triangles[i].a}
            };

            // An edge belongs to the boundary if it is not shared by any other bad triangle
            for (int j = 0; j < 3; j++) {
                bool is_shared = false;
                for (size_t k = 0; k < bad_triangles.size(); k++) {
                    if (i == k) continue;

                    Edge other_edges[3] = {
                        {bad_triangles[k].a, bad_triangles[k].b},
                        {bad_triangles[k].b, bad_triangles[k].c},
                        {bad_triangles[k].c, bad_triangles[k].a}
                    };
                    for (int l = 0; l < 3; l++) {
                        if (edges[j] == other_edges[l]) {
                            is_shared = true;
                            break;
                        }
                    }
                    if (is_shared) break;
                }
                if (!is_shared) polygon.push_back(edges[j]);
            }
        }

        // Remove bad triangles from the main triangulation
        triangles.erase(
            std::remove_if(triangles.begin(), triangles.end(),
                [&bad_triangles](const Triangle &t) {
                    for (const auto &bad_t : bad_triangles) {
                        if (std::abs(t.circum_x - bad_t.circum_x) < 1e-6 &&
                            std::abs(t.circum_y - bad_t.circum_y) < 1e-6) {
                            return true;
                        }
                    }
                    return false;
                }
            ), triangles.end()
        );

        // Re-triangulate the polygon hole
        for (const auto &edge : polygon) triangles.push_back(Triangle(edge.a, edge.b, point));
    }

    // Remove triangles containing the Super triangle
    triangles.erase(
        std::remove_if(triangles.begin(), triangles.end(),
            [&a_super, &b_super, &c_super] (const Triangle &t) {
                return t.contains_vertex(a_super) ||
                        t.contains_vertex(b_super) ||
                        t.contains_vertex(c_super);
            }
        ), triangles.end()
    );
}

const std::vector<Triangle> &DelaunayTriangulator::get_triangles() const {
    return triangles;
}

const std::vector<Point> DelaunayTriangulator::get_centerline(double max_edge_distance) const {
    std::vector<Point> centerline;

    std::vector<Edge> gates = get_track_gates(max_edge_distance);

    for (const auto &e : gates) {
        Point midpoint{ (e.a.x + e.b.x) / 2.0, (e.a.y + e.b.y) / 2.0, ConeColor::UNKNOWN};
        centerline.push_back(midpoint);
    }

    return centerline;
}

std::vector<Edge> DelaunayTriangulator::get_left_boundary(double max_edge_distance) const {
    return prune_boundary_edges(get_boundary(ConeColor::BLUE, max_edge_distance));
}

std::vector<Edge> DelaunayTriangulator::get_right_boundary(double max_edge_distance) const {
    return prune_boundary_edges(get_boundary(ConeColor::YELLOW, max_edge_distance));
}

std::vector<Edge> DelaunayTriangulator::get_track_gates(double max_edge_distance) const {
        std::vector<Edge> unique_valid_edges;

        // Lambda function to find distance from a point to the nearest cone of a given color
        auto dist_to_closest_color = [&](const Point &p, ConeColor c) {
            double min_dist = std::numeric_limits<double>::max();
            for (const auto &t : triangles) {
                const Point *pts[3] = {&t.a, &t.b, &t.c};
                for (int k = 0; k < 3; k++) {
                    if (pts[k]->color == c) {
                        double dist = std::hypot(p.x - pts[k]->x, p.y - pts[k]->y);
                        if (dist < min_dist) min_dist = dist;
                    }
                }
            }
            return min_dist;
        };

        for (const auto &t : triangles) {
            // Extract the three edges of the current triangle
            Edge edges[3] = { {t.a, t.b}, {t.b, t.c}, {t.c, t.a} };

            for (int i = 0; i < 3; i++) {
                const Edge &e = edges[i];
                bool is_cross_track = false;

                // Condition 1: An edge crossing the track connects the left boundary to the right boundary
                // Condition 2: An edge crossing the track connects the left side orange and right side orange cones
                // Condition 3: An edge crossing the track connects the right side orange and the left boundary
                // Condition 4: An edge crossing the track connects the left side orange and the right boundary
                if ((e.a.color == ConeColor::BLUE && e.b.color == ConeColor::YELLOW) ||
                    (e.a.color == ConeColor::YELLOW && e.b.color == ConeColor::BLUE)) {
                    is_cross_track = true;
                } else if (e.a.color == ConeColor::ORANGE && e.b.color == ConeColor::ORANGE && e.length() > 1.5) {
                    is_cross_track = true;
                } else if ((e.a.color == ConeColor::BLUE && e.b.color == ConeColor::ORANGE) ||
                           (e.a.color == ConeColor::ORANGE && e.b.color == ConeColor::BLUE)) {
                    Point orange_cone = (e.a.color == ConeColor::ORANGE) ? e.a : e.b;
                    // Check if the orange cone is to the yellow side
                    if (dist_to_closest_color(orange_cone, ConeColor::YELLOW) < dist_to_closest_color(orange_cone, ConeColor::BLUE)) {
                        is_cross_track = true;
                    }
                } else if ((e.a.color == ConeColor::YELLOW && e.b.color == ConeColor::ORANGE) ||
                         (e.a.color == ConeColor::ORANGE && e.b.color == ConeColor::YELLOW)) {
                    Point orange_cone = (e.a.color == ConeColor::ORANGE) ? e.a : e.b;
                    // Check if the orange cone is to the blue side
                    if (dist_to_closest_color(orange_cone, ConeColor::BLUE) < dist_to_closest_color(orange_cone, ConeColor::YELLOW)) {
                        is_cross_track = true;
                    }
                }

                if (is_cross_track && e.length() <= max_edge_distance) {
                    // Check if the edges is already in the vector
                    if (std::find(unique_valid_edges.begin(), unique_valid_edges.end(), e) == unique_valid_edges.end()) {
                        unique_valid_edges.push_back(e);
                    }
                }
            }
        }
        return unique_valid_edges;
    }