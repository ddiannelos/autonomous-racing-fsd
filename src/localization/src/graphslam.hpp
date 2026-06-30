#pragma once

#include <iostream>
#include <vector>
#include <map>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Sparse>

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Triplet;

typedef Eigen::Matrix<double, 5, 1> Vector5d;

/**
 * @brief Configuration struct for GraphSlam tuning parameters
 */
struct GraphSlamConfig {
    double odom_info_x = 1.0;
    double odom_info_y = 1.0;
    double odom_info_yaw = 100.0;
    double min_translation = 0.3;
    double min_rotation = 0.1;
    double huber_odom = 0.3;
    double huber_obs = 1.0;
    int window_size = 30;
};

/**
 * @brief Represents the vehicle's state node in the SLAM pose graph
 */
struct Pose {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int id;
    Vector3d pose;
};

/**
 * @brief Represents a mapped track boundary (cone) in the SLAM graph
 */
struct Landmark {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int id;                            /**< Unique identifier for the landmark */
    int color_id;                      /**< Winning color ID (0: Yellow, 1: Blue, 2: Orange, 3: Big Orange, 4: Unknown) */
    Vector2d position;                 /**< Global (x, y) coordinates of the cone */
    Vector5d color_probability;        /**< Discrete Bayes filter probability distribution for the 5 classes */
};

/**
 * @brief Represents a relative movement constraint between two consecutive poses
 */
struct OdometryEdge {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int from_id;                      /**< ID of the previous pose */
    int to_id;                        /**< ID of the current pose */
    Vector3d measurement;             /**< Relative movement (dx, dy, dyaw) in the local frame */
    Matrix3d information;             /**< Inverse covariance matrix (confidence in the measurement) */
};

/**
 * @brief Represents a measurement constraint between a vehicle pose and a landmark
 */
struct ObservationEdge {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int pose_id;                     /**< ID of the pose from which the cone was observed */
    int landmark_id;                 /**< ID of the matched landmark in the map */
    int color_id;                    /**< Color classification from the percepption */
    double color_confidence;         /**< Confidence score of the color */
    Vector2d measurement;            /**< Relative distance to the cone (dx, dy) in the vehicle frame */
    Matrix2d information;            /**< Inverse covariance matrix (confidence based on distance)*/
};

/**
 * @brief A custom 2D Graph SLAM implementation designed for Autonomous Racing
 * * This class builds a pose graph, matches YOLO cone detections, tracks color
 * probabilities using a recursive Bayes filter, and optimizes the trajectory
 * using a Sparse Cholesky (SimplicialLDLT) solver
 */
class GraphSlam {
private:
    GraphSlamConfig config_;

    std::vector<Pose> poses;
    std::map<int, Landmark> landmarks;

    std::vector<OdometryEdge> odometry_edges;
    std::vector<ObservationEdge> observation_edges;

    Matrix3d odom_info;
    bool is_initialized;
    Vector3d prev_odom;
    int next_landmark_id = 0;

    bool optimization_triggered = false;
    bool localization_mode = false;
    bool new_lap = false;

    int frames_since_last_lap = 0;

    Vector3d map_to_odom_offset;

    /** @brief Normalizes an angle to be strictly within [-pi, pi]*/
    double normalize_angle(double phi);

    /** @brief Inserts a dense matrix block into the sparse triplet list for the Information Matrix (H)*/
    void add_to_H(std::vector<Triplet<double>> &triplets, int row_start, int col_start, const MatrixXd &block);

    /** @brief Initializes the first pose and all visible landmarks on frame 1 */
    void initialize_graph(const Vector3d &current_odom, const std::vector<ObservationEdge> &observations);

    /** @brief Checks if the vehicle has moved past the minimum translation/rotation */
    bool has_moved_enough(const Vector3d &current_odom);

    /** @brief Computes the local odometry delta and adds it to the graph edges */
    void add_odometry_edge(const Vector3d &current_odom, int prev_id, int curr_id);

    /** @brief Calculates a Huber loss weight to mitigate the impact of outliers */
    double weight_huber(const VectorXd &error, double delta = 1.0);

    /** @brief Projects a local sensor measurement to a global map coordinate */
    Vector2d predict_landmark_position(const Vector3d &pose, const Vector2d &measurement);

    /** @brief Generates a 5D probability distribution from a single color confidence */
    Vector5d get_color_probabilities(int color_id, double confidence);

    /** @brief Executes a recursive discrete Bayes filter update to track cone colors */
    void update_landmark_color_bayes(int match_id, int color_id, double confidence);

    /** @brief Truncates the pose graph to a maximum number of recent poses to maintain real-time performance */
    void slide_window(size_t window_size);

    /** @brief Calculates the global rigid-body transform offset between the EKF frame and the SLAM Map frame */
    void calculate_map_to_odom_offset(const Vector3d &latest_optimized_pose, const Vector3d &current_odom);

    /** @brief Predicts the vehicle's new global pose based on odometry and the calculated map offset */
    Vector3d predict_new_pose(const Vector3d &current_odom);

    /** @brief Runs the Gauss-Newton / Levenberg-Marquardt optimizer to solve the pose graph */
    void optimize_graph();
public:
    /** @brief Constructs a new GraphSlam object and initializes the tuning matrices */
    GraphSlam(const GraphSlamConfig& config = GraphSlamConfig());

    /** * @brief Core engine entry point. Process a new odometry state and cone array
     * @param current_odom The current EKF odometry reading (x, y, yaw)
     * @param current_observations A vector of cone observations detected in the current frame
    */
    void process(const Vector3d &current_odom, const std::vector<ObservationEdge> &current_observations);

    /** @brief Fetches the latest optimized global pose of the vehicle */
    Vector3d get_current_pose() const;

    /** @brief Fetches all fully optimized map boundaries */
    std::vector<Landmark> get_optimized_map() const;

    /** @brief Return if loop closure is detected */
    bool loop_closure_detected() {
        if (new_lap) {
            new_lap = false;
            return true;
        }
        return false;
    }
};

GraphSlam::GraphSlam(const GraphSlamConfig& config) : config_(config) {
    is_initialized = false;
    map_to_odom_offset = Vector3d::Zero();
    odom_info = Matrix3d::Zero();

    // Base confidence for odometry
    odom_info(0, 0) = config_.odom_info_x;   // Forward x certainty
    odom_info(1, 1) = config_.odom_info_y;   // Lateral y certainty
    odom_info(2, 2) = config_.odom_info_yaw; // Yaw certainty
}

double GraphSlam::normalize_angle(double phi) {
    return std::atan2(std::sin(phi), std::cos(phi));
}

void GraphSlam::add_to_H(std::vector<Triplet<double>> &triplets, int row_start, int col_start, const MatrixXd &block) {
    for (int r = 0; r < block.rows(); r++) {
        for (int c = 0; c < block.cols(); c++) {
            triplets.push_back(Triplet<double>(row_start + r, col_start + c, block(r, c)));
        }
    }
}

void GraphSlam::initialize_graph(const Vector3d &current_odom, const std::vector<ObservationEdge> &observations) {
    poses.push_back({0, current_odom});
    prev_odom = current_odom;
    is_initialized = true;

    // Initialize all seen cones on the first frame
    for (const auto &obs : observations) {
        ObservationEdge new_edge = obs;
        new_edge.pose_id = 0;

        // Dynamic confidence: trust closes cones more than distant ones
        double distance = new_edge.measurement.norm();
        double confidence = std::max(1.0, 100.0 / (distance+1.0));

        new_edge.information = Matrix2d::Identity() * confidence;
        new_edge.landmark_id = next_landmark_id++;

        Vector2d global_lm_pred = predict_landmark_position(current_odom, new_edge.measurement);

        landmarks[new_edge.landmark_id] = {new_edge.landmark_id, new_edge.color_id, global_lm_pred};
        observation_edges.push_back(new_edge);
    }
}

bool GraphSlam::has_moved_enough(const Vector3d &current_odom) {
    double dx = current_odom(0) - prev_odom(0);
    double dy = current_odom(1) - prev_odom(1);
    double dtheta_check = std::abs(normalize_angle(current_odom(2) - prev_odom(2)));
    double distance_traveled = std::sqrt(dx * dx + dy * dy);

    // Drop a new graph node based on parameterized movement thresholds
    return (distance_traveled >= config_.min_translation || dtheta_check >= config_.min_rotation);
}

void GraphSlam::add_odometry_edge(const Vector3d &current_odom, int prev_id, int curr_id) {
    // Calculate global odometry delta
    double global_dx = current_odom(0) - prev_odom(0);
    double global_dy = current_odom(1) - prev_odom(1);
    double dtheta = normalize_angle(current_odom(2) - prev_odom(2));

    // Rotate global delta into the vehicle's local frame
    double c_prev = std::cos(prev_odom(2));
    double s_prev = std::sin(prev_odom(2));
    double local_dx =  c_prev * global_dx + s_prev * global_dy;
    double local_dy = -s_prev * global_dx + c_prev * global_dy;

    Vector3d odom_delta(local_dx, local_dy, dtheta);
    odometry_edges.push_back({prev_id, curr_id, odom_delta, odom_info});
}

double GraphSlam::weight_huber(const VectorXd &error, double delta) {
    // Flattens the loss curve for errors larger than the delta to ignore outliers
    double e_norm = error.norm();
    if (e_norm <= delta) return 1.0;
    else return delta / e_norm;
}

Vector2d GraphSlam::predict_landmark_position(const Vector3d &pose, const Vector2d &measurement) {
    double c = std::cos(pose(2));
    double s = std::sin(pose(2));
    Vector2d global_lm_pred;

    // Apply 2D body rotation and translation
    global_lm_pred(0) = pose(0) + c * measurement(0) - s * measurement(1);
    global_lm_pred(1) = pose(1) + s * measurement(0) + c * measurement(1);
    return global_lm_pred;
}

Vector5d GraphSlam::get_color_probabilities(int color_id, double confidence) {
    // Cap confidence to prevent the Bayes filter from permanently locking a color
    double p_match = std::min(0.97, std::max(0.20, confidence));
    double p_other = (1.0 - p_match) / 4.0; // Distribute remaining doubt among the 4 other classes

    Vector5d prob = Vector5d::Constant(p_other);
    prob(color_id) = p_match;
    return prob;
}

void GraphSlam::update_landmark_color_bayes(int match_id, int color_id, double confidence) {
    Vector5d measurement_prob = get_color_probabilities(color_id, confidence);

    // Discrete Bayes update (Prior * likelihood)
    landmarks[match_id].color_probability = landmarks[match_id].color_probability.cwiseProduct(measurement_prob);

    // Normalize probabilities sum to 1.0
    landmarks[match_id].color_probability /= landmarks[match_id].color_probability.sum();

    // Search the new winning color
    double best_prob = 0.0;
    for (int i = 0; i < 5; i++) {
        if (landmarks[match_id].color_probability[i] > best_prob) {
            landmarks[match_id].color_id = i;
            best_prob = landmarks[match_id].color_probability[i];
        }
    }
}

void GraphSlam::slide_window(size_t window_size) {
    if (poses.size() <= window_size) return;

    int poses_to_drop = poses.size() - window_size;

    // Remove the oldest poses
    poses.erase(poses.begin(), poses.begin() + poses_to_drop);

    // Reassign sequential IDs to the remaining poses
    for (size_t i = 0; i < poses.size(); i++) poses[i].id = i;

    // Purge orphaned Odometry Edges and shift remaining IDs
    odometry_edges.erase(
        std::remove_if(odometry_edges.begin(), odometry_edges.end(),
                       [poses_to_drop](const OdometryEdge& e) { return e.from_id < poses_to_drop; }),
        odometry_edges.end()
    );
    for (auto& edge : odometry_edges) {
        edge.from_id -= poses_to_drop;
        edge.to_id -= poses_to_drop;
    }

    // Purge orphaned Observation Edges and shift remaining IDs
    observation_edges.erase(
        std::remove_if(observation_edges.begin(), observation_edges.end(),
                       [poses_to_drop](const ObservationEdge& e) { return e.pose_id < poses_to_drop; }),
        observation_edges.end()
    );
    for (auto& edge : observation_edges) {
        edge.pose_id -= poses_to_drop;
    }
}

void GraphSlam::calculate_map_to_odom_offset(const Vector3d &latest_optimized_pose, const Vector3d &current_odom) {
    // Find the difference between the pure EKF and slam corrected position
    map_to_odom_offset(2) = normalize_angle(latest_optimized_pose(2) - current_odom(2));
    double c_new_off = std::cos(map_to_odom_offset(2));
    double s_new_off = std::sin(map_to_odom_offset(2));
    map_to_odom_offset(0) = latest_optimized_pose(0) - (current_odom(0) * c_new_off - current_odom(1) * s_new_off);
    map_to_odom_offset(1) = latest_optimized_pose(1) - (current_odom(0) * s_new_off + current_odom(1) * c_new_off);
}

Vector3d GraphSlam::predict_new_pose(const Vector3d &current_odom) {
    Vector3d new_pose;
    double c_off = std::cos(map_to_odom_offset(2));
    double s_off = std::sin(map_to_odom_offset(2));

    // Generate the new pose strictly from the smooth EKF + the map offset
    new_pose(0) = map_to_odom_offset(0) + current_odom(0) * c_off - current_odom(1) * s_off;
    new_pose(1) = map_to_odom_offset(1) + current_odom(0) * s_off + current_odom(1) * c_off;
    new_pose(2) = normalize_angle(current_odom(2) + map_to_odom_offset(2));

    return new_pose;
}

void GraphSlam::optimize_graph() {
    if (poses.size() < 2) return;

    int num_poses = poses.size();
    int num_landmarks = landmarks.size();

    // Dynamic state: Shrink the matrix size if in localization_mode
    int state_dim = localization_mode ? (num_poses * 3) : num_poses * 3 + num_landmarks * 2;

    std::map<int, int> lm_idx_map;
    if (!localization_mode) {
        int lm_counter = 0;
        for (auto &pair : landmarks) {
            lm_idx_map[pair.first] = num_poses * 3 + lm_counter *2;
            lm_counter++;
        }
    }

    std::vector<Triplet<double>> triplets;
    VectorXd b = VectorXd::Zero(state_dim);

    // Anchor the first pose to prevent the entire map from floating away
    double anchor_weight = 1e6;
    triplets.push_back(Triplet<double>(0, 0, anchor_weight));
    triplets.push_back(Triplet<double>(1, 1, anchor_weight));
    triplets.push_back(Triplet<double>(2, 2, anchor_weight));

    // Process Odometry constraints
    for (const auto &edge : odometry_edges) {
        int idx_i = edge.from_id * 3;
        int idx_j = edge.to_id * 3;

        Vector3d xi = poses[edge.from_id].pose;
        Vector3d xj = poses[edge.to_id].pose;
        Vector3d u = edge.measurement;

        // Compute deltas
        double dx = xj(0) - xi(0);
        double dy = xj(1) - xi(1);
        double c = std::cos(xi(2));
        double s = std::sin(xi(2));

        // Error function
        Vector3d e;
        e(0) =  c * dx + s * dy - u(0);
        e(1) = -s * dx + c * dy - u(1);
        e(2) = normalize_angle(xj(2) - xi(2) - u(2));

        // Jacobians with respect to pose i (A) and pose j (B)
        Matrix3d A;
        A << -c, -s, -s * dx + c * dy,
                s, -c, -c * dx - s * dy,
                0,  0, -1;

        Matrix3d B;
        B << c, s, 0,
            -s, c, 0,
                0, 0, 1;

        // Apply Huber loss to down-weight outliers
        double weight = weight_huber(e, config_.huber_odom);
        Matrix3d omega = edge.information * weight;

        // Add blocks to Information Matrix H
        add_to_H(triplets, idx_i, idx_i, A.transpose() * omega * A);
        add_to_H(triplets, idx_i, idx_j, A.transpose() * omega * B);
        add_to_H(triplets, idx_j, idx_i, B.transpose() * omega * A);
        add_to_H(triplets, idx_j, idx_j, B.transpose() * omega * B);

        // Add to information vector b
        b.segment<3>(idx_i) += A.transpose() * omega * e;
        b.segment<3>(idx_j) += B.transpose() * omega * e;
    }

    // Process Observation Constraints
    for (const auto &edge : observation_edges) {
        int idx_i = edge.pose_id * 3;

        Vector3d xi = poses[edge.pose_id].pose;
        Vector2d lm = landmarks[edge.landmark_id].position;
        Vector2d z = edge.measurement;

        double dx = lm(0) - xi(0);
        double dy = lm(1) - xi(1);
        double c = std::cos(xi(2));
        double s = std::sin(xi(2));

        // Transform landmark position into the vehicle's local frame
        Vector2d z_pred;
        z_pred(0) = c * dx + s * dy;
        z_pred(1) = -s * dx + c * dy;

        // Calculate measurement error
        Vector2d e = z_pred - z;

        // Jacobians with respect to the pose A and the landmark B
        Eigen::Matrix<double, 2, 3> A;
        A << -c, -s, -s * dx + c * dy,
                s, -c, -c * dx - s * dy;

        Matrix2d B;
        B << c, s,
            -s, c;

        // Apply Huber loss
        double weight = weight_huber(e, config_.huber_obs);
        Matrix2d omega = edge.information * weight;

        // Optimize the pose
        add_to_H(triplets, idx_i, idx_i, A.transpose() * omega * A);

        // Only optimize the landmark block if the map is still unlocked
        if (!localization_mode) {
            int idx_l = lm_idx_map[edge.landmark_id];

            add_to_H(triplets, idx_i, idx_l, A.transpose() * omega * B);
            add_to_H(triplets, idx_l, idx_i, B.transpose() * omega * A);
            add_to_H(triplets, idx_l, idx_l, B.transpose() * omega * B);
            b.segment<2>(idx_l) += B.transpose() * omega * e;
        }

        b.segment<3>(idx_i) += A.transpose() * omega * e;
    }

    // Construct sparse matrix and solve H * delta = -b using
    // Cholesky factorization
    Eigen::SparseMatrix<double> H(state_dim, state_dim);
    H.setFromTriplets(triplets.begin(), triplets.end());
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
    solver.compute(H);

    // Fails silently if the matrix is underconstrained
    if (solver.info() != Eigen::Success) return;

    Eigen::VectorXd delta = solver.solve(-b);

    // Apply calculated deltas to update poses
    for (int i = 0; i < num_poses; i++) {
        poses[i].pose(0) += delta(i * 3 + 0);
        poses[i].pose(1) += delta(i * 3 + 1);
        poses[i].pose(2) = normalize_angle(poses[i].pose(2) + delta(i * 3 + 2));
    }

    // Apply calculated deltas to update landmarks
    if (!localization_mode) {
        for (auto &pair : landmarks) {
            int idx = lm_idx_map[pair.first];
            pair.second.position(0) += delta(idx + 0);
            pair.second.position(1) += delta(idx + 1);
        }
    }
}

void GraphSlam::process(const Vector3d &current_odom, const std::vector<ObservationEdge> &current_observations) {
    if (!is_initialized) {
        initialize_graph(current_odom, current_observations);
        return;
    }

    frames_since_last_lap++;

    if (!has_moved_enough(current_odom)) return;

    int prev_id = poses.size() - 1;
    int curr_id = poses.size();

    // Odometry prediction phase
    add_odometry_edge(current_odom, prev_id, curr_id);
    Vector3d new_pose = predict_new_pose(current_odom);
    poses.push_back({curr_id, new_pose});

    // Data association and mapping phase
    for (const auto &obs : current_observations) {
        ObservationEdge new_edge = obs;
        new_edge.pose_id = curr_id;

        // Calculate dynamic confidence for this measurement
        double distance = new_edge.measurement.norm();
        double confidence = std::max(1.0, 100.0 / (distance+1.0));
        new_edge.information = Matrix2d::Identity() * confidence;

        Vector2d global_lm_pred = predict_landmark_position(new_pose, new_edge.measurement);

        int match_id = -1;
        double min_dist = 1e9;
        bool is_orange_cone = (new_edge.color_id == 2 || new_edge.color_id == 3);
        bool mapped_enough_cones = (landmarks.size() > 100);
        bool is_close_enough = (new_edge.measurement(0)-1.6 < 1.5 && new_edge.measurement(0) > -3.0);
        double match_threshold = (is_orange_cone && mapped_enough_cones) ? 3.0 : 0.6;


        // Nearest Neighbor data association
        for (const auto &pair : landmarks) {
            double dist = (global_lm_pred - pair.second.position).norm();
            if (dist < min_dist && dist < match_threshold) {
                min_dist = dist;
                match_id = pair.first;
            }
        }

        if (match_id != -1) {
            new_edge.landmark_id = match_id;
            update_landmark_color_bayes(match_id, new_edge.color_id, new_edge.color_confidence);

            // Orange Cone Loop closure trigger
            bool is_mapped_orange = (landmarks[match_id].color_id == 2 || landmarks[match_id].color_id == 3);

            // Check if the lap has finished
            if (is_orange_cone && is_mapped_orange && mapped_enough_cones && is_close_enough && frames_since_last_lap > 150) {
                new_lap = true;
                frames_since_last_lap = 0;

                // If not in localization mode, optimize the graph
                if (!localization_mode) optimization_triggered = true;
            }
        } else {
            // Drop ghost cones on Lap 2
            if (localization_mode) continue;

            new_edge.landmark_id = next_landmark_id++;
            int id = new_edge.landmark_id;

            Vector5d color_prob = get_color_probabilities(new_edge.color_id, new_edge.color_confidence);
            landmarks[id] = {id, new_edge.color_id, global_lm_pred, color_prob};
        }

        observation_edges.push_back(new_edge);

    }

    prev_odom = current_odom;

    // If in localization mode, cap the graph to the last X poses
    if (localization_mode) {
        slide_window(config_.window_size);
    }

    // Graph Optimization phase
    if (optimization_triggered || localization_mode || curr_id % 10 == 0) {
        // If the optimization is triggered then optimize the graph more times
        int max_iters = (optimization_triggered) ? 3 : 1;
        for (int iter = 0; iter < max_iters; iter++) {
            optimize_graph();
        }
        // Synchronize the EKF-to-Map transform using the newly optimized trajectory tail
        calculate_map_to_odom_offset(poses.back().pose, current_odom);

        // Switch to localization mode if loop closure finished
        if (optimization_triggered) {
            localization_mode = true;
            optimization_triggered = false;
        }
    }
}

Vector3d GraphSlam::get_current_pose() const {
    if (poses.empty()) return Vector3d::Zero();
    return poses.back().pose;
}

std::vector<Landmark> GraphSlam::get_optimized_map() const {
    std::vector<Landmark> optimized_map;
    optimized_map.reserve(landmarks.size());
    for (const auto &pair : landmarks) optimized_map.push_back(pair.second);
    return optimized_map;
}