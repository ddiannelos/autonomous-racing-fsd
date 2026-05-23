#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <OsqpEigen/OsqpEigen.h>

struct MPCState {
    double x;
    double y;
    double yaw;
    double v;
};

struct MPCInput {
    double delta; // Steering angle
    double accel; // Acceleration/Braking
};

struct MPCResult {
    double delta;
    double accel;
    std::vector<Eigen::Vector2d> predicted_path; // Used for visualization
};

class MPCController {
private:
    int N = 10;                             /**< Prediction horizon (Looking 1.0 seconds ahead) */
    double dt = 0.1;                        /**< Timestep */
    double L = 1.6;                        /**< Wheelbase of the vehicle */

    // Physical limits of the car
    double max_steer = 1.22;
    double max_accel = 8.0;
    double max_decel = -12.0;

    Eigen::DiagonalMatrix<double, 4> Q;     /**< Penalizes deviation from the target state [x, y, yaw, velocity] */
    Eigen::DiagonalMatrix<double, 2> R;     /**< Penalizes high actuator values [steer, accel]*/

    // Penalties for changing the inputs too fast
    double R_steer_rate = 20000.0;
    double R_accel_rate = 25.0;

    int last_nearest_idx = -1;

    OsqpEigen::Solver solver;
    Eigen::SparseMatrix<double> P;
    Eigen::SparseMatrix<double> A_cons;
    Eigen::VectorXd q;
    Eigen::VectorXd lower_bound;
    Eigen::VectorXd upper_bound;
    std::vector<Eigen::Triplet<double>> A_triplets;
    bool is_initialized = false;
    int num_variables;
    int num_constraints;

    /** @brief Locates the closest point on the reference path in front of the vehicle */
    int find_nearest_index(const MPCState &current_state, const std::vector<Eigen::Vector2d> &path);
public:
    MPCController() {
        Q.diagonal() << 250.0, 250.0, 100.0, 50.0;
        // Q.diagonal() << 100, 100, 100, 50.0;
        R.diagonal() << 100.0, 10.0;
    }

    /** @brief Allocates memory and factorizes the QP matrices */
    bool init(const MPCState& initial_state, const std::vector<Eigen::Vector2d>& opt_path, const std::vector<double>& speed_profile);

    /** @brief Constructs and solves the Quadratic Program to find the optimal control inputs */
    MPCResult solve(const MPCState& current_state, const std::vector<Eigen::Vector2d>& opt_path, const std::vector<double>& speed_profile);
};

int MPCController::find_nearest_index(const MPCState &current_state, const std::vector<Eigen::Vector2d> &path) {
    int path_len = path.size();
    if (path_len == 0) return 0;

    // Helper to calculate squared distance (No expensive std::hypot or sqrt!)
    auto get_sq_dist = [&](int idx) {
        double dx = path[idx].x() - current_state.x;
        double dy = path[idx].y() - current_state.y;
        return (dx * dx) + (dy * dy);
    };

    // Lap 1 Initialization: Global Search
    if (last_nearest_idx == -1) {
        double min_dist_sq = 1e15;
        int best_idx = 0;
        for (int i = 0; i < path_len; i++) {
            double dist_sq = get_sq_dist(i);
            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                best_idx = i;
            }
        }
        last_nearest_idx = best_idx;
        return best_idx;
    }

    // Local Forward Search
    double min_dist_sq = 1e15;
    int best_idx = last_nearest_idx;

    // Since the array is now ordered correctly, a small window is plenty.
    // The car won't skip 50 spline points in a single 0.1s timestep!
    int search_window = std::min(50, path_len);

    for (int i = 0; i < search_window; i++) {
        // Only look forward in the array
        int check_idx = (last_nearest_idx + i) % path_len;
        double dist_sq = get_sq_dist(check_idx);

        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            best_idx = check_idx;
        }
    }

    last_nearest_idx = best_idx;
    return best_idx;
}

bool MPCController::init(const MPCState& initial_state, const std::vector<Eigen::Vector2d>& opt_path, const std::vector<double>& speed_profile) {
    int nx = 4; // [x, y, yaw, v]
    int nu = 2; // [steer, accel]
    num_variables = nx * (N + 1) + nu * N;
    num_constraints = nx * (N + 1) + nu * N;

    P.resize(num_variables, num_variables);
    A_cons.resize(num_constraints, num_variables);
    q.resize(num_variables);
    lower_bound.resize(num_constraints);
    upper_bound.resize(num_constraints);
    A_triplets.reserve(num_constraints * 5);

    // Build the Constant P Matrix
    std::vector<Eigen::Triplet<double>> P_triplets;
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < nx; ++j) P_triplets.push_back({i * nx + j, i * nx + j, Q.diagonal()[j]});
        for (int j = 0; j < nu; ++j) P_triplets.push_back({nx * (N + 1) + i * nu + j, nx * (N + 1) + i * nu + j, R.diagonal()[j]});
    }
    for (int i = 0; i < N - 1; ++i) {
        int u_idx = nx * (N + 1) + i * nu;
        int next_u_idx = nx * (N + 1) + (i + 1) * nu;
        P_triplets.push_back({u_idx + 0, u_idx + 0, R_steer_rate});
        P_triplets.push_back({next_u_idx + 0, next_u_idx + 0, R_steer_rate});
        P_triplets.push_back({u_idx + 0, next_u_idx + 0, -R_steer_rate});
        P_triplets.push_back({next_u_idx + 0, u_idx + 0, -R_steer_rate});
        P_triplets.push_back({u_idx + 1, u_idx + 1, R_accel_rate});
        P_triplets.push_back({next_u_idx + 1, next_u_idx + 1, R_accel_rate});
        P_triplets.push_back({u_idx + 1, next_u_idx + 1, -R_accel_rate});
        P_triplets.push_back({next_u_idx + 1, u_idx + 1, -R_accel_rate});
    }
    for (int j = 0; j < nx; ++j) P_triplets.push_back({N * nx + j, N * nx + j, Q.diagonal()[j]});
    P.setFromTriplets(P_triplets.begin(), P_triplets.end());

    // To initialize the solver, A_cons and q must be populated at least once.
    // solve() internally to do the math, but we catch the setup before it runs the actual solver.
    is_initialized = true; // Temporarily trick solve() so it populates the matrices
    solve(initial_state, opt_path, speed_profile);
    is_initialized = false;

    // Configure and factorize the solver
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);
    solver.data()->setNumberOfVariables(num_variables);
    solver.data()->setNumberOfConstraints(num_constraints);

    if (!solver.data()->setHessianMatrix(P)) return false;
    if (!solver.data()->setGradient(q)) return false;
    if (!solver.data()->setLinearConstraintsMatrix(A_cons)) return false;
    if (!solver.data()->setLowerBound(lower_bound)) return false;
    if (!solver.data()->setUpperBound(upper_bound)) return false;

    if (!solver.initSolver()) return false;

    is_initialized = true;
    return true;
}

MPCResult MPCController::solve(const MPCState& current_state, const std::vector<Eigen::Vector2d>& opt_path, const std::vector<double>& speed_profile) {
    if (opt_path.empty() || speed_profile.empty()) return {0.0, 0.0, {}};

    int nearest_idx = find_nearest_index(current_state, opt_path);
    int path_len = opt_path.size();

    int nx = 4; // [x, y, yaw, v]
    int nu = 2; // [steer, accel]

    // Calculates smooth orientation by looking ahead slightly
    auto get_smooth_heading = [&](int start_idx) {
        int curr = start_idx;
        double dist = 0.0;
        for (int k = 0; k < 20; ++k) {
            int next = (curr + 1) % path_len;
            dist += std::hypot(opt_path[next].x() - opt_path[curr].x(), opt_path[next].y() - opt_path[curr].y());
            curr = next;
            if (dist >= 0.5) break;
        }
        return std::atan2(opt_path[curr].y() - opt_path[start_idx].y(), opt_path[curr].x() - opt_path[start_idx].x());
    };

    // Project forward N steps along the path based on the target velocity to create a reference horizon
    std::vector<int> ref_indices(N + 1);
    ref_indices[0] = nearest_idx;
    double accumulated_dist = 0.0;
    int curr_idx = nearest_idx;

    for (int i = 1; i <= N; ++i) {
        double v_r = std::max(3.0, speed_profile[curr_idx]);
        double target_dist = accumulated_dist + (v_r * dt);

        int emergency_break = 0;
        while (emergency_break < 200) {
            int next_idx = (curr_idx + 1) % path_len;
            double dist_to_next = std::hypot(opt_path[next_idx].x() - opt_path[curr_idx].x(),
                                                opt_path[next_idx].y() - opt_path[curr_idx].y());
            if (accumulated_dist + dist_to_next >= target_dist) break;

            accumulated_dist += dist_to_next;
            curr_idx = next_idx;
            emergency_break++;
        }
        ref_indices[i] = curr_idx;
        accumulated_dist = target_dist;
    }

    // Zero-allocation buffer clearing
    q.setZero();
    A_triplets.clear();

    // Populate the gradient (Q)
    for (int i = 0; i < N; ++i) {
        int ref_idx = ref_indices[i];
        double v_r = speed_profile[ref_idx];
        double theta_r = get_smooth_heading(ref_idx);

        double yaw_diff = theta_r - current_state.yaw;
        while (yaw_diff > M_PI) yaw_diff -= 2.0 * M_PI;
        while (yaw_diff < -M_PI) yaw_diff += 2.0 * M_PI;
        theta_r = current_state.yaw + yaw_diff;

        // Gradient components pushing states toward the reference trajectory
        q(i * nx + 0) = -Q.diagonal()[0] * opt_path[ref_idx].x();
        q(i * nx + 1) = -Q.diagonal()[1] * opt_path[ref_idx].y();
        q(i * nx + 2) = -Q.diagonal()[2] * theta_r;
        q(i * nx + 3) = -Q.diagonal()[3] * v_r;
    }

    // Apply terminal cost to q
    int term_ref = ref_indices[N];
    double terminal_theta = get_smooth_heading(term_ref);
    double term_yaw_diff = terminal_theta - current_state.yaw;
    while (term_yaw_diff > M_PI) term_yaw_diff -= 2.0 * M_PI;
    while (term_yaw_diff < -M_PI) term_yaw_diff += 2.0 * M_PI;
    terminal_theta = current_state.yaw + term_yaw_diff;

    q(N * nx + 0) = -Q.diagonal()[0] * opt_path[term_ref].x();
    q(N * nx + 1) = -Q.diagonal()[1] * opt_path[term_ref].y();
    q(N * nx + 2) = -Q.diagonal()[2] * terminal_theta;
    q(N * nx + 3) = -Q.diagonal()[3] * speed_profile[term_ref];

    // Rebuild dynamics and constraints (A_cons, bounds)
    // Initial state constraint (x0 = current state)
    for (int j = 0; j < nx; ++j) A_triplets.push_back({j, j, 1.0});
    lower_bound.segment(0, nx) << current_state.x, current_state.y, current_state.yaw, current_state.v;
    upper_bound.segment(0, nx) << current_state.x, current_state.y, current_state.yaw, current_state.v;

    // Dynamics constraint: x_{k+1} = A*x_k + B*u_k
    for (int i = 0; i < N; ++i) {
        int ref_idx = ref_indices[i];
        int next_ref_idx = ref_indices[(i + 1) % (N + 1)];

        double v_r = std::max(0.1, speed_profile[ref_idx]);
        double theta_r = get_smooth_heading(ref_idx);

        double yaw_diff = theta_r - current_state.yaw;
        while (yaw_diff > M_PI) yaw_diff -= 2.0 * M_PI;
        while (yaw_diff < -M_PI) yaw_diff += 2.0 * M_PI;
        theta_r = current_state.yaw + yaw_diff;

        double next_theta_r = get_smooth_heading(next_ref_idx);
        double delta_theta = next_theta_r - theta_r;
        while (delta_theta > M_PI) delta_theta -= 2.0 * M_PI;
        while (delta_theta < -M_PI) delta_theta += 2.0 * M_PI;

        double delta_r = std::clamp(std::atan2(L * delta_theta, v_r * dt), -max_steer, max_steer);

        // Fill A and B Jacobians for the Kinematic Bicycle Model
        A_triplets.push_back({nx * (i + 1) + 0, nx * i + 0, -1.0});
        A_triplets.push_back({nx * (i + 1) + 1, nx * i + 1, -1.0});
        A_triplets.push_back({nx * (i + 1) + 2, nx * i + 2, -1.0});
        A_triplets.push_back({nx * (i + 1) + 3, nx * i + 3, -1.0});

        A_triplets.push_back({nx * (i + 1) + 0, nx * i + 2, v_r * std::sin(theta_r) * dt});
        A_triplets.push_back({nx * (i + 1) + 0, nx * i + 3, -std::cos(theta_r) * dt});
        A_triplets.push_back({nx * (i + 1) + 1, nx * i + 2, -v_r * std::cos(theta_r) * dt});
        A_triplets.push_back({nx * (i + 1) + 1, nx * i + 3, -std::sin(theta_r) * dt});
        A_triplets.push_back({nx * (i + 1) + 2, nx * i + 3, -(std::tan(delta_r) / L) * dt});

        double cos_delta_sq = std::max(0.01, std::pow(std::cos(delta_r), 2));
        A_triplets.push_back({nx * (i + 1) + 2, nx * (N + 1) + i * nu + 0, -(v_r * dt) / (L * cos_delta_sq)});
        A_triplets.push_back({nx * (i + 1) + 3, nx * (N + 1) + i * nu + 1, -dt});

        for (int j = 0; j < nx; ++j) A_triplets.push_back({nx * (i + 1) + j, nx * (i + 1) + j, 1.0});

        double x_r = opt_path[ref_idx].x();
        double y_r = opt_path[ref_idx].y();
        double f_x = x_r + v_r * std::cos(theta_r) * dt;
        double f_y = y_r + v_r * std::sin(theta_r) * dt;
        double f_theta = theta_r + (v_r * std::tan(delta_r) / L) * dt;

        double c_x = f_x - (x_r - theta_r * v_r * std::sin(theta_r) * dt + v_r * std::cos(theta_r) * dt);
        double c_y = f_y - (y_r + theta_r * v_r * std::cos(theta_r) * dt + v_r * std::sin(theta_r) * dt);
        double c_theta = f_theta - (theta_r + v_r * (std::tan(delta_r) / L) * dt) - (delta_r * (v_r * dt) / (L * cos_delta_sq));

        lower_bound.segment(nx * (i + 1), nx) << c_x, c_y, c_theta, 0.0;
        upper_bound.segment(nx * (i + 1), nx) << c_x, c_y, c_theta, 0.0;
    }

    // Input Constraints (Physical Limits)
    for (int i = 0; i < N; ++i) {
        A_triplets.push_back({nx * (N + 1) + i * nu + 0, nx * (N + 1) + i * nu + 0, 1.0});
        A_triplets.push_back({nx * (N + 1) + i * nu + 1, nx * (N + 1) + i * nu + 1, 1.0});

        lower_bound(nx * (N + 1) + i * nu + 0) = -max_steer;
        upper_bound(nx * (N + 1) + i * nu + 0) = max_steer;
        lower_bound(nx * (N + 1) + i * nu + 1) = max_decel;
        upper_bound(nx * (N + 1) + i * nu + 1) = max_accel;
    }

    A_cons.setFromTriplets(A_triplets.begin(), A_triplets.end());

    // Fast solver update
    if (is_initialized) {
        solver.updateGradient(q);
        solver.updateBounds(lower_bound, upper_bound);
        solver.updateLinearConstraintsMatrix(A_cons);

        if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return {0.0, 0.0, {}};

        Eigen::VectorXd solution = solver.getSolution();

        // Extract the predicted path for visualization
        std::vector<Eigen::Vector2d> predicted_path;
        for (int i = 0; i <= N; ++i) predicted_path.push_back({solution(i * nx + 0), solution(i * nx + 1)});

        // Return the first optimal control action (u_0)
        return {solution(nx * (N + 1)), solution(nx * (N + 1) + 1), predicted_path};
    }

    // Failsafe return if called during the initialization pass
    return {0.0, 0.0, {}};
}