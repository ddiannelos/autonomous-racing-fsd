#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <interfaces/msg/cone_array.hpp>
#include <interfaces/msg/cone.hpp>

const std::string ODOM_SUB_TOPIC = "/localization/slam/odom";
const std::string CONES_SUB_TOPIC = "/localization/slam/cone_list";
const std::string PATH_PUB_TOPIC = "/localization/viz/slam_path";
const std::string CLOUD_PUB_TOPIC = "/localization/viz/cone_cloud";

/**
 * @brief Utility node to visualize custom SLAM data in RViz
 * * Subscribes to the vehicle's optimized odometry and custom cone arrays and
 * translates them into standard nav_msgs::Path andsensor_msgs::PointCloud2
 * messages for 3D rendering in RViz
 */
class SlamVisualizationNode : public rclcpp::Node {
private:
    nav_msgs::msg::Path slam_path; /**< Accumulates historical poses to draw the trajectory line */

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<interfaces::msg::ConeArray>::SharedPtr cone_sub;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub;

    /**
     * @brief Accumulates incoming SLAM odometry into a continuous historical path
     * @param msg The latest optimized odometry from the GraphSlam engine
     */
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (slam_path.header.frame_id.empty()) {
            slam_path.header.frame_id = "map";
        }
        slam_path.header.stamp = msg->header.stamp;

        // Convert Odometry pose to PoseStamped for the Path Array
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = msg->header;
        pose_stamped.pose = msg->pose.pose;

        slam_path.poses.push_back(pose_stamped);

        if (slam_path.poses.size() > 1000) slam_path.poses.erase(slam_path.poses.begin());

        path_pub->publish(slam_path);
    }

    /**
     * @brief Converts the custom ConeArray into an RGB PointCloud2
     * * PointClouds are used instead of MarkerArrays becasue RViz can render them faster than mesh markers
     * @param msg The custom array of optimized cone positions and colors
     */
    void cone_callback(const interfaces::msg::ConeArray::SharedPtr msg) {
        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header = msg->header;
        cloud_msg.header.frame_id = "map";

        // Setup the PointCloud2 binary data structure for XYZ and RGB
        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
        modifier.resize(msg->cones.size());

        // Iterators allow safe memory writing directly into the raw byte array
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_msg, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_msg, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_msg, "b");

        for (const auto &cone : msg->cones) {
            // Write spatial coordinates
            *iter_x = cone.position.x;
            *iter_y = cone.position.y;
            *iter_z = 0.0;

            // Map custom color IDs to exact RGB rendering values
            if (cone.color == 0) {                             // Yellow
                *iter_r = 255; *iter_g = 255; *iter_b = 0;
            } else if (cone.color == 1) {                      // Blue
                *iter_r = 0; *iter_g = 0; *iter_b = 255;
            } else if (cone.color == 2 || cone.color == 3) {   // Orange
                *iter_r = 255; *iter_g = 128; *iter_b = 0;
            } else {                                           // Unknown
                *iter_r = 100; *iter_g = 100; *iter_b = 100;
            }

            // Advance all memory points to the next point
            ++iter_x; ++iter_y; ++iter_z;
            ++iter_r; ++iter_g; ++iter_b;
        }

        cloud_pub->publish(cloud_msg);
    }
public:
    /**
     * @brief Constructs the Vizualization node, initilizing publishers and subscribers
     */
    SlamVisualizationNode() : Node("slam_visualization_node") {
        path_pub = this->create_publisher<nav_msgs::msg::Path>(PATH_PUB_TOPIC, 10);
        cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(CLOUD_PUB_TOPIC, 10);

        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            ODOM_SUB_TOPIC, 10,
            std::bind(&SlamVisualizationNode::odom_callback, this, std::placeholders::_1)
        );

        cone_sub = this->create_subscription<interfaces::msg::ConeArray>(
            CONES_SUB_TOPIC, 10,
            std::bind(&SlamVisualizationNode::cone_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Slam Visualization Node started");
    }
};

/**
 * @brief Standard ROS2 entry point.
 */
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SlamVisualizationNode>());
    rclcpp::shutdown();
    return 0;
}
