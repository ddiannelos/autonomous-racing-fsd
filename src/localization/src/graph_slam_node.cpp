#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int32.hpp>
#include <interfaces/msg/cone_array.hpp>
#include <interfaces/msg/cone.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "graphslam.hpp"

/**
 * @brief ROS 2 Node that wraps the GraphSlam C++ library
 * * This node subscribes to the vehicle's EKF odometry and the Lidar/Camera cone detection,
 * synchronizes them using an ApproximateTime filter, feeds them into the SLAM engine and
 * publishes the optimized trajectory and track map
 */
class GraphSlamNode : public rclcpp::Node {
private:
    std::unique_ptr<GraphSlam> slam_map;   /**< Pointer to the core Graph Slam engine */
    Vector3d current_odom;                 /**< Vehicle state (x, y, yaw) from the EKF */

    double current_v = 0;                  /**< Cached linear velocity from EKF */
    double current_w = 0;                  /**< Cached angular velocity from EKF */
    int current_lap = 1;                   /**< Lap counter */

    // Topics
    std::string odom_pub_topic_;
    std::string odom_sub_topic_;
    std::string cones_pub_topic_;
    std::string cones_sub_topic_;
    std::string lap_counter_topic_;

    // Offset for the Lidar sensor actual position
    // against the center of the vehicle
    double sensor_offset_x_;

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    rclcpp::Publisher<interfaces::msg::ConeArray>::SharedPtr map_pub;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr lap_pub;

    /**
     * @brief Type definition for the message filter synchroniation policy
     * Synchronizes Odometry and ConeArray messages based on their header timestamps
     */
    typedef message_filters::sync_policies::ApproximateTime<
        nav_msgs::msg::Odometry,
        interfaces::msg::ConeArray
    > SyncPolicy;

    // Subscribers and synchronizer
    message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub;
    message_filters::Subscriber<interfaces::msg::ConeArray> cone_sub;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync;

    /**
     * @brief Helper function to convert a 3D geomtery_msgs Quaternion into a 2D yaw angle
     * @param w Quaternion w component
     * @param x Quaternion x component
     * @param y Quaternion y component
     * @param z Quaternion z component
     */
    double extract_yaw(double w, double x, double y, double z) {
        double siny_cosp = 2.0 * (w * z + x * y);
        double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        return atan2(siny_cosp, cosy_cosp);
    }

    /**
     * @brief Main synchronized callback. Triggered only when matching Odometry, and Cone Data arrive
     * @param odom_msg The vehicle's current odometry estimate
     * @param cone_msg The array of cones currently visible to the perception camera
     */
    void synced_callback(const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg, const interfaces::msg::ConeArray::ConstSharedPtr &cone_msg) {
        // Extract odometry from ros message
        current_odom(0) = odom_msg->pose.pose.position.x;
        current_odom(1) = odom_msg->pose.pose.position.y;
        current_odom(2) = extract_yaw(odom_msg->pose.pose.orientation.w,
                                         odom_msg->pose.pose.orientation.x,
                                         odom_msg->pose.pose.orientation.y,
                                         odom_msg->pose.pose.orientation.z);

        // Extract kinematics
        current_v = odom_msg->twist.twist.linear.x;
        current_w = odom_msg->twist.twist.angular.z;

        // Convert ROS cone messages into SLAM obervation edges
        std::vector<ObservationEdge> observations;
        for (const auto &cone : cone_msg->cones) {
            ObservationEdge edge;
            edge.color_id = cone.color;
            // Apply the parameterized sensor offset to map lidar frame to vehicle center frame
            edge.measurement = Vector2d(cone.position.x + sensor_offset_x_, cone.position.y);
            edge.color_confidence = cone.confidence;

            // Assign a temporary unmapped ID. The SLAM engine handles data association
            edge.landmark_id = -1;
            observations.push_back(edge);
        }

        // Run the slam core
        slam_map->process(current_odom, observations);

        // Publish optimized results
        rclcpp::Time current_time = cone_msg->header.stamp;
        publish_odom(current_time);
        publish_map(cone_msg->header);

        // Check if a new lap has started
        if (slam_map->loop_closure_detected()) {
            current_lap++;
            std_msgs::msg::Int32 lap_msg;
            lap_msg.data = current_lap;
            lap_pub->publish(lap_msg);
        }
    }

    /**
     * @brief Packages the mathematically optimized SLAM pose into a ROS Odometry message
     * @param stamp The timestamp to attach to the published message
     */
    void publish_odom(rclcpp::Time &stamp) {
        // Fetch teh global tracking result
        const auto pose = slam_map->get_current_pose();

        // Create odometry message
        nav_msgs::msg::Odometry slam_odom;
        slam_odom.header.stamp = stamp;
        slam_odom.header.frame_id = "map";
        slam_odom.child_frame_id = "hero";

        // Position
        slam_odom.pose.pose.position.x = pose(0);
        slam_odom.pose.pose.position.y = pose(1);
        slam_odom.pose.pose.position.z = 0.0;

        // Orientation (Yaw to Quaternion)
        slam_odom.pose.pose.orientation.x = 0.0;
        slam_odom.pose.pose.orientation.y = 0.0;
        slam_odom.pose.pose.orientation.z = sin(pose(2) * 0.5);
        slam_odom.pose.pose.orientation.w = cos(pose(2) * 0.5);

        // Velocity
        slam_odom.twist.twist.linear.x = current_v;
        slam_odom.twist.twist.angular.z = current_w;

        odom_pub->publish(slam_odom);
    }

    /**
     * @brief Packages the optimized SLAM landmarks into a ROS ConeArray message
     * @param header The standard ROS header containing the frame ID and timestamp
     */
    void publish_map(std_msgs::msg::Header header) {
        // Fetch the completely optimized map boundaries
        const auto &map = slam_map->get_optimized_map();

        interfaces::msg::ConeArray cone_array;
        cone_array.header = header;

        if (map.empty()) return;

        // Convert internal SLAM Landmark structs back to standard ROS messages
        for (size_t i = 0; i < map.size(); i++) {
            interfaces::msg::Cone cone = get_cone(map[i]);
            cone_array.cones.push_back(cone);
        }

        // Publish messages
        map_pub->publish(cone_array);
    }

    /**
     * @brief Helper function to map a C++ SLAM Landmark struct to a ROS 2 Cone message
     * @param lm The internal SLAM Landmark to convert
     * @return The formatted ROS 2 Cone message
     */
    interfaces::msg::Cone get_cone(const Landmark &lm) {
        interfaces::msg::Cone cone;
        cone.color = lm.color_id;          // The optimized color
        cone.confidence = 1.0;             // Output 100% since the Bayes filter has already smoothed the doubt
        cone.position.x = lm.position(0);
        cone.position.y = lm.position(1);
        cone.position.z = 0.0;
        return cone;
    }
public:
    /**
     * @brief Constructs the GraphSlamNode, initilizing the SLAM engine, publishers and synchronizers
     */
    GraphSlamNode() : Node("slam_node") {
        // Declare Parameters
        this->declare_parameter<std::string>("topics.slam_odom", "/localization/slam/odom");
        this->declare_parameter<std::string>("topics.ekf_odom", "/localization/ekf/odom");
        this->declare_parameter<std::string>("topics.slam_cones", "/localization/slam/cone_list");
        this->declare_parameter<std::string>("topics.perception_cones", "/perception/cone_list");
        this->declare_parameter<std::string>("topics.lap_count", "/localization/slam/lap_count");

        this->declare_parameter<double>("sensor_offset_x", 1.6);

        this->declare_parameter<double>("tuning.odom_info_x", 1.0);
        this->declare_parameter<double>("tuning.odom_info_y", 1.0);
        this->declare_parameter<double>("tuning.odom_info_yaw", 100.0);
        this->declare_parameter<double>("tuning.min_translation", 0.3);
        this->declare_parameter<double>("tuning.min_rotation", 0.1);
        this->declare_parameter<double>("tuning.huber_odom", 0.3);
        this->declare_parameter<double>("tuning.huber_obs", 1.0);
        this->declare_parameter<int>("tuning.window_size", 30);

        // Read Parameters
        odom_pub_topic_ = this->get_parameter("topics.slam_odom").as_string();
        odom_sub_topic_ = this->get_parameter("topics.ekf_odom").as_string();
        cones_pub_topic_ = this->get_parameter("topics.slam_cones").as_string();
        cones_sub_topic_ = this->get_parameter("topics.perception_cones").as_string();
        lap_counter_topic_ = this->get_parameter("topics.lap_count").as_string();

        sensor_offset_x_ = this->get_parameter("sensor_offset_x").as_double();

        // Populate Config Struct for the Core Engine
        GraphSlamConfig config;
        config.odom_info_x = this->get_parameter("tuning.odom_info_x").as_double();
        config.odom_info_y = this->get_parameter("tuning.odom_info_y").as_double();
        config.odom_info_yaw = this->get_parameter("tuning.odom_info_yaw").as_double();
        config.min_translation = this->get_parameter("tuning.min_translation").as_double();
        config.min_rotation = this->get_parameter("tuning.min_rotation").as_double();
        config.huber_odom = this->get_parameter("tuning.huber_odom").as_double();
        config.huber_obs = this->get_parameter("tuning.huber_obs").as_double();
        config.window_size = this->get_parameter("tuning.window_size").as_int();

        // Instantiate SLAM Engine with the configuration
        slam_map = std::make_unique<GraphSlam>(config);

        // Create publisher using dynamic topics
        odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(odom_pub_topic_, 10);
        map_pub = this->create_publisher<interfaces::msg::ConeArray>(cones_pub_topic_, 10);
        lap_pub = this->create_publisher<std_msgs::msg::Int32>(lap_counter_topic_, 10);

        // Create subscribers
        odom_sub.subscribe(this, odom_sub_topic_);
        cone_sub.subscribe(this, cones_sub_topic_);

        // Setup the sync policy (Queue size 50)
        sync = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(50), odom_sub, cone_sub);
        sync->registerCallback(std::bind(&GraphSlamNode::synced_callback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Slam Node Initialized");
    }
};

/**
 * @brief Standard ROS 2 entry point
 */
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GraphSlamNode>());
    rclcpp::shutdown();
    return 0;
}