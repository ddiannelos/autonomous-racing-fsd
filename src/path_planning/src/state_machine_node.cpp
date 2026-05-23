#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <carla_msgs/msg/carla_ego_vehicle_control.hpp>
#include <interfaces/msg/cone.hpp>
#include <interfaces/msg/cone_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <fstream>
#include <iostream>

#include "planning/exploration_planner.hpp"
#include "planning/optimization_planner.hpp"

// ROS2 Topics
const std::string ODOM_TOPIC = "/localization/slam/odom";
const std::string CONE_TOPIC = "/localization/slam/cone_list";
const std::string LAP_TOPIC = "/localization/slam/lap_count";
const std::string CONTROL_TOPIC = "/carla/hero/vehicle_control_cmd";
const std::string POINT_VIZ_TOPIC = "/exploration/viz/point";
const std::string CIRCLE_VIZ_TOPIC = "/exploration/viz/circle";
const std::string MPC_VIZ_TOPIC = "/optimization/viz/mpc_trajectory";

const int MAX_LAPS = 4;

/** @brief The states of the vehicle */
enum class RaceState {
    EXPLORATION,   /**< Lap 1: Driving safely: discovering the track */
    OPTIMIZATION,  /**< Lap 2+: Track is known, racing line computed, driving at limits */
    FINISHED       /**< Max laps reached, bring the car to a stop */
};

class StateMachineNode : public rclcpp::Node {
private:
    RaceState current_state = RaceState::EXPLORATION;
    int lap_count = 1;

    // Planners
    ExplorationPlanner explorer;
    OptimizationPlanner optimizer;

    // Vehicle state
    VehiclePose current_pose;
    double current_speed = 0.0;
    std::vector<Point> global_cones_cache;
    rclcpp::Time prev_time;

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<interfaces::msg::ConeArray>::SharedPtr cone_sub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr lap_sub;

    // Publishers
    rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr control_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr circle_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mpc_pub;

    // Used for visualization and debugging
    std::ofstream telemetry_file;
    double start_time = -1.0;

    /** @brief Quaterion to Yaw converter */
    double extract_yaw(double w, double x, double y, double z) {
        double siny_cosp = 2.0 * (w * z + x * y);
        double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        return atan2(siny_cosp, cosy_cosp);
    }

    /** @brief Monitors the SLAM's pipeline's lap counter and triggers state transition */
    void lap_callback(const std_msgs::msg::Int32::SharedPtr msg);

    /** @brief Cahces the global cone map published by SLAM */
    void cone_callback(const interfaces::msg::ConeArray::SharedPtr msg);

    /** @brief The main control loop, driven by the odometry publisher */
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    /** @brief Publish the control command */
    void publish_to_carla(const ControlCommand &cmd) {
        carla_msgs::msg::CarlaEgoVehicleControl carla_cmd;
        carla_cmd.steer = cmd.steering;
        carla_cmd.throttle = cmd.throttle;
        carla_cmd.brake = cmd.brake;
        control_pub->publish(carla_cmd);
    }

    /** @brief Publishes the visualizations */
    void publish_visualizations(const Point &target, double radius) {
        publish_lookahead_point(target);
        publish_lookahead_radius(radius);
    }

    /** @brief Visualizes the lookahead point */
    void publish_lookahead_point(const Point &target);

    /** @brief Visualizes the lookahead radius */
    void publish_lookahead_radius(double lookahead_radius);

    /** @brief Visualizes the MPC trajectory path */
    void publish_mpc_trajectory(const std::vector<Point> &trajectory);
public:
    StateMachineNode();
};

void StateMachineNode::lap_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    lap_count = msg->data;

    // Trasition: Exploration -> Optimization
    if (lap_count == 2 && current_state == RaceState::EXPLORATION) {
        RCLCPP_INFO(this->get_logger(), "LAP 2 TRIGGERED: Passing map to Optimizer...");

        current_state = RaceState::OPTIMIZATION;

        // Full stop so, the optimization strategy can be initialized
        publish_to_carla(ControlCommand{0.0, 0.0, 1.0, {0.0, 0.0, ConeColor::UNKNOWN}, 0.0, {}});

        // Extract the global track from the exploration node, and initialize the track
        std::vector<Edge> full_track_gates = explorer.generate_global_gates(global_cones_cache);
        optimizer.initialize_track(full_track_gates, current_pose);

        // Save the final map to a csv file for visualization
        std::string filename = "/home/sir/ros2_ws/scripts/csv/cones.csv";
        std::ofstream csv_file(filename);

        if (csv_file.is_open()) {
            for (const auto &c : global_cones_cache) {
                int color;
                switch (c.color) {
                    case ConeColor::YELLOW: color = 0; break;
                    case ConeColor::BLUE: color = 1; break;
                    case ConeColor::ORANGE: color = 2; break;
                    case ConeColor::UNKNOWN: color = 4; break;
                }
                csv_file << c.x << "," << c.y << "," << color << "\n";
            }
            csv_file.close();
        } else std::cerr << "ERROR: Failed to open file";
    // Transition: Optimization -> Finished
    } else if (lap_count >= MAX_LAPS && current_state != RaceState::FINISHED){
        current_state = RaceState::FINISHED;
        RCLCPP_INFO(this->get_logger(), "%d Laps Complete. Shutting down driving.", lap_count);
    }
}

void StateMachineNode::cone_callback(const interfaces::msg::ConeArray::SharedPtr msg) {
    if (current_state == RaceState::OPTIMIZATION) return;
    global_cones_cache.clear();
    for (const auto &c : msg->cones) {
        ConeColor color;
        switch (c.color) {
            case 0: color = ConeColor::YELLOW; break;
            case 1: color = ConeColor::BLUE; break;
            case 2:
            case 3: color = ConeColor::ORANGE; break;
            default: color = ConeColor::UNKNOWN; break;
        }
        global_cones_cache.push_back({c.position.x, c.position.y, color});
    }
}

void StateMachineNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Update vehicle state
    current_pose.x = msg->pose.pose.position.x;
    current_pose.y = msg->pose.pose.position.y;
    current_pose.yaw = extract_yaw(
        msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y, msg->pose.pose.orientation.z
    );
    current_speed = msg->twist.twist.linear.x;

    rclcpp::Time current_time = this->now();
    double dt = (current_time - prev_time).seconds();
    prev_time = current_time;

    if (start_time < 0.0) start_time = current_time.seconds();
    double elapsed_time = current_time.seconds() - start_time;

    ControlCommand cmd;

    // Depending on the state, execute the strategy
    if (current_state == RaceState::EXPLORATION) {
        // Lap 1
        cmd = explorer.compute_command(current_pose, current_speed, dt, global_cones_cache);
        publish_visualizations(cmd.lookahead_point, cmd.lookahead_radius);
    } else if (current_state == RaceState::OPTIMIZATION) {
        // Lap 2+
        cmd = optimizer.compute_command(current_pose, current_speed);
        publish_mpc_trajectory(cmd.mpc_trajectory);
    } else if (current_state == RaceState::FINISHED) {
        // Laps finished
        cmd = {0.0, 0.0, 0.5, {0.0, 0.0, ConeColor::UNKNOWN}, 0.0, {}};

        if (telemetry_file.is_open()) telemetry_file.close();
    }

    // Print results for visualization and debugging
    if (telemetry_file.is_open() && current_state != RaceState::FINISHED) {
        int state_id = (current_state == RaceState::EXPLORATION) ? 0 : 1; // 0 for exploration, 1 for optimization
        telemetry_file << elapsed_time << ","
                       << state_id << ","
                       << current_pose.x << ","
                       << current_pose.y << ","
                       << current_speed << ","
                       << cmd.steering << ","
                       << cmd.throttle << ","
                       << cmd.brake << "\n";
    }

    // Publish control command
    publish_to_carla(cmd);
}

void StateMachineNode::publish_lookahead_point(const Point &target) {
    visualization_msgs::msg::Marker sphere_marker;
    sphere_marker.header.frame_id = "map";
    sphere_marker.header.stamp = this->now();
    sphere_marker.ns = "lookahead";
    sphere_marker.id = 0;
    sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;
    sphere_marker.action = visualization_msgs::msg::Marker::ADD;

    sphere_marker.pose.position.x = target.x;
    sphere_marker.pose.position.y = target.y;
    sphere_marker.pose.position.z = 0.3;

    sphere_marker.scale.x = 0.2;
    sphere_marker.scale.y = 0.2;
    sphere_marker.scale.z = 0.2;

    sphere_marker.color.r = 0.0f;
    sphere_marker.color.g = 1.0f;
    sphere_marker.color.b = 0.0f;
    sphere_marker.color.a = 1.0f;

    marker_pub->publish(sphere_marker);
}

void StateMachineNode::publish_lookahead_radius(double lookahead_radius) {
    visualization_msgs::msg::Marker circle_marker;
    circle_marker.header.frame_id = "map";
    circle_marker.header.stamp = this->now();
    circle_marker.ns = "lookahead_radius";
    circle_marker.id = 1;
    circle_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    circle_marker.action = visualization_msgs::msg::Marker::ADD;

    circle_marker.scale.x = 0.1;
    circle_marker.color.r = 0.0f;
    circle_marker.color.g = 1.0f;
    circle_marker.color.b = 1.0f;
    circle_marker.color.a = 0.8f;

    for (int i = 0; i <= 36; i++) {
        double theta = (double) i * (2.0 * M_PI / 36.0);
        geometry_msgs::msg::Point p;
        p.x = current_pose.x + lookahead_radius * std::cos(theta);
        p.y = current_pose.y + lookahead_radius * std::sin(theta);
        p.z = 0.3;
        circle_marker.points.push_back(p);
    }
    circle_pub->publish(circle_marker);
}

void StateMachineNode::publish_mpc_trajectory(const std::vector<Point> &trajectory) {
    if (trajectory.empty()) return;

    visualization_msgs::msg::Marker line_marker;
    line_marker.header.frame_id = "map";
    line_marker.header.stamp = this->now();
    line_marker.ns = "mpc_prediction";
    line_marker.id = 2;
    line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::msg::Marker::ADD;

    line_marker.scale.x = 0.15;

    line_marker.color.r = 1.0f;
    line_marker.color.g = 0.0f;
    line_marker.color.b = 1.0f;
    line_marker.color.a = 1.0f;

    for (const auto &pt : trajectory) {
        geometry_msgs::msg::Point p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = 0.2;
        line_marker.points.push_back(p);
    }
    mpc_pub->publish(line_marker);
}

StateMachineNode::StateMachineNode() : Node("state_machine_node") {
    prev_time = this->now();

    // Initialize subscribers
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        ODOM_TOPIC, 10,
        std::bind(&StateMachineNode::odom_callback, this, std::placeholders::_1)
    );
    cone_sub = this->create_subscription<interfaces::msg::ConeArray>(
        CONE_TOPIC, 10,
        std::bind(&StateMachineNode::cone_callback, this, std::placeholders::_1)
    );
    lap_sub = this->create_subscription<std_msgs::msg::Int32>(
        LAP_TOPIC, 10,
        std::bind(&StateMachineNode::lap_callback, this, std::placeholders::_1)
    );

    // Initialize publishers
    control_pub = this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>(
        CONTROL_TOPIC, 10
    );
    marker_pub = this->create_publisher<visualization_msgs::msg::Marker>(
        POINT_VIZ_TOPIC, 10
    );
    circle_pub = this->create_publisher<visualization_msgs::msg::Marker>(
        CIRCLE_VIZ_TOPIC, 10
    );
    mpc_pub = this->create_publisher<visualization_msgs::msg::Marker>(
        MPC_VIZ_TOPIC, 10
    );

    telemetry_file.open("/home/sir/ros2_ws/scripts/csv/telemetry.csv");
    if (telemetry_file.is_open()) telemetry_file << "time,state,x,y,speed,steering,throttle,brake\n";

    RCLCPP_INFO(this->get_logger(), "State Machine Initialized");
}


/**
 * @brief Standard ROS 2 entry point
 */
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StateMachineNode>());
    rclcpp::shutdown();
    return 0;
}
