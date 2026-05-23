#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <Eigen/Dense>

const std::string IMU_TOPIC = "/carla/hero/imu";
const std::string GNSS_TOPIC = "/carla/hero/gnss";
const std::string ODOM_TOPIC = "/localization/ekf/odom";

const float INITIAL_X = 194.5;
const float INITIAL_Y = -296.3;
const float INITIAL_VEL = 0.0;
const float INITIAL_YAW = M_PI;

/**
 * @brief Extended Kalman Filter (EKF) Node for Vehicle Localization
 * * Fuses high-frequency IMU data (linear acceleration and yaw rate) for the
 * Preddiction step, and low-frequency GNSS data (Latitude, Longitude) for th
 * Update step. Estimates the 4D state vector: [X, Y, Velocity, Yaw]
 */
class EKFNode : public rclcpp::Node {
private:
    bool initialized = false;

    // GNSS Local Tangent Plane Origin
    double origin_lat = 0.0;
    double origin_lon = 0.0;
    bool origin_set = false;

    int origin_samples = 0;
    const int MAX_ORIGIN_SAMPLES = 20;
    double sum_lat = 0.0;
    double sum_lon = 0.0;

    Eigen::Vector4d x;    /**< State Vector [pos_x, pos_y, velocity, yaw]^T */
    Eigen::Matrix4d P;    /**< State Covariance: Trakces the uncertainty of the state  */
    Eigen::Matrix4d Q;    /**< Process Noise: IMU prediction distrust */
    Eigen::Matrix2d R;    /**< Measurement Noise: GNSS reading distrust */

    rclcpp::Time last_imu_time;

    // ROS2 Subscribers and Publisher
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gnss;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;

    /**
     * @brief Normalizes an angle to be strictly within [-pi, pi]
     */
    void normalize_angle(double &phi);

    /**
     * @brief Converts spherical GPS coordinates (Lat/Lon) into Cartesian (X/Y) meters
     * relative to the established local tangent plane origin
     */
    void lat_lon_to_xy(double lat, double lon, double &x, double &y);

    /**
     * @brief EKF Prediction Step. Uses IMU kinematics to push the state vector forward in time
     * @param dt Time delta since the last IMU message in seconds
     * @param ax Linear acceleration from the IMU
     * @param yaw_rate Angular velocity from the IMU
     */
    void predict(double dt, double ax, double yaw_rate);

    /**
     * @brief EKF Update step. Corrects the predicted state using an absolute GNSS measurement
     * @param meas_x The measured X position from the GNSS
     * @param meas_y The measured Y position from the GNSS
     */
    void update(double meas_x, double meas_y);

    /** @brief High-frequency callback for IMU data. Drives the prediction step */
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    /** @brief Low-frequency callback for GNSS data. Drives the update step */
    void gnss_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
public:
    /** @brief Constructs the EKF Node, initializing system matrices and ROS interfaces */
    EKFNode();
};

EKFNode::EKFNode() : Node("ekf_node") {
    // Initialize State Vector
    x.setZero();
    x(0) = INITIAL_X;
    x(1) = INITIAL_Y;
    x(2) = INITIAL_VEL;
    x(3) = INITIAL_YAW;

    // Initialize state covariance (P)
    P = Eigen::Matrix4d::Identity();
    P(0, 0) = 10.0;
    P(1, 1) = 10.0;

    // Initialize Process Noise (Q) - System physics uncertainty
    Q = Eigen::Matrix4d::Identity();
    Q(0, 0) = 0.1; // 0.005;     // 0.01 0.05
    Q(1, 1) = 0.1; // 0.005;     // 0.01 0.05
    Q(2, 2) = 0.1; // 0.001;    // 0.005 0.1
    Q(3, 3) = 0.05; // 0.0005;    // 0.001 0.01

    // Initialize Measurement Noise (R) - GNSS sensor uncertainty
    R = Eigen::Matrix2d::Identity();
    R(0, 0) = 1.0; //5.0;
    R(1, 1) = 1.0; //5.0;

    // ROS Interfaces
    sub_imu = this->create_subscription<sensor_msgs::msg::Imu>(
        IMU_TOPIC, 10, std::bind(&EKFNode::imu_callback, this, std::placeholders::_1)
    );
    sub_gnss = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        GNSS_TOPIC, 10, std::bind(&EKFNode::gnss_callback, this, std::placeholders::_1)
    );

    pub_odom = this->create_publisher<nav_msgs::msg::Odometry>(ODOM_TOPIC, 10);

    RCLCPP_INFO(this->get_logger(), "Extended Kalman Filter Node Started");
}

void EKFNode::normalize_angle(double &phi) {
    phi = atan2(sin(phi), cos(phi));
}

void EKFNode::lat_lon_to_xy(double lat, double lon, double &x, double &y) {
    double R_earth = 6378137.0;
    double d_lat = (lat - origin_lat) * M_PI / 180.0;
    double d_lon = (lon - origin_lon) * M_PI / 180.0;

    // Equirectangular approximation for fast local distance calculation
    x = d_lon * R_earth * cos(origin_lat * M_PI / 180.0);
    y = -d_lat * R_earth;
}

void EKFNode::predict(double dt, double ax, double yaw_rate) {
    double v = x(2);
    double theta = x(3);

    // Jacobian
    Eigen::Matrix4d F = Eigen::Matrix4d::Identity();

    // Derivative of pos_x with respect to theta
    F(0, 3) = -v * sin(theta) * dt;
    F(0, 2) = cos(theta) * dt;

    // Derivative of pos_y with respect to theta
    F(1, 3) = v * cos(theta) * dt;
    F(1, 2) = sin(theta) * dt;

    // State transition x = x + v*cos(theta)*dt
    x(0) = x(0) + v * cos(theta) * dt;
    x(1) = x(1) + v * sin(theta) * dt;
    x(2) = x(2) + ax * dt;
    x(3) = x(3) + yaw_rate * dt;

    // Zero Velocity Update
    if (std::abs(ax) < 0.1 && std::abs(x(2)) < 0.2) {
        x(2) = 0.0;

        // Sever the mathematical link between position and velocity
        // so GNSS noise doesn't drag the velocity around while parked
        P(0, 2) = 0.0; P(2, 0) = 0.0;
        P(1, 2) = 0.0; P(2, 1) = 0.0;
        P(2, 2) = 0.01; // Velocity uncertainty
    }

    normalize_angle(x(3));

    // Covariance update (P = F * P * F^t + Q)
    P = F * P * F.transpose() + (Q * dt);
}

void EKFNode::update(double meas_x, double meas_y) {
    Eigen::Vector2d z(meas_x, meas_y);

    Eigen::Matrix<double, 2, 4> H;
    H << 1, 0, 0, 0,
         0, 1, 0, 0;

    // Predicted measurement based on current state
    Eigen::Vector2d z_pred = H * x;
    // Innovation (measurement residual)
    Eigen::Vector2d y = z - z_pred;
    // Inovation Covariance
    Eigen::Matrix2d S = H * P * H.transpose() + R;
    // Kalman Gain
    Eigen::Matrix<double, 4, 2> K = P * H.transpose() * S.inverse();

    // State update
    x = x + K * y;

    normalize_angle(x(3));

    // Covariance update
    // P = (I - K * H) * P;
    // Joseph Form: P = (I - K*H) * P * (I - K*H)^T + K*R*K^T
    Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d I_KH = I - K * H;

    P = I_KH * P * I_KH.transpose() + K * R * K.transpose();
}

void EKFNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    if (!initialized) {
        last_imu_time = msg->header.stamp;
        initialized = true;
        return;
    }

    // Calculate Delta Time
    rclcpp::Time current_time = msg->header.stamp;
    double dt = (current_time - last_imu_time).seconds();
    last_imu_time = current_time;

    // Inputs from Imu sensor
    double ax = msg->linear_acceleration.x;
    double yaw_rate = msg->angular_velocity.z;

    // Prediction
    predict(dt, ax, yaw_rate);

    // Send odom message
    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = current_time;
    odom.header.frame_id = "map";
    odom.child_frame_id = "hero";

    // Position
    odom.pose.pose.position.x = x(0);
    odom.pose.pose.position.y = x(1);
    odom.pose.pose.position.z = 0.0;

    // Orientation (Yaw to Quaternion)
    // We dont have roll and pitch
    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = sin(x(3) * 0.5);
    odom.pose.pose.orientation.w = cos(x(3) * 0.5);

    // Velocity
    odom.twist.twist.linear.x = x(2);
    odom.twist.twist.angular.z = msg->angular_velocity.z;

    pub_odom->publish(odom);
}

void EKFNode::gnss_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    if (!initialized) return;

    // Wait for a stable block of samples to set the local tangent plane
    if (!origin_set) {
        sum_lat += msg->latitude;
        sum_lon += msg->longitude;
        origin_samples++;

        if (origin_samples >= MAX_ORIGIN_SAMPLES) {
            origin_lat = sum_lat / MAX_ORIGIN_SAMPLES;
            origin_lon = sum_lon / MAX_ORIGIN_SAMPLES;
            origin_set = true;
        }

        return;
    }

    // Covert absolute Lat/Lon to local X/Y offset
    double meas_x, meas_y;
    lat_lon_to_xy(msg->latitude, msg->longitude, meas_x, meas_y);

    // Apply starting position bias
    meas_x += INITIAL_X;
    meas_y += INITIAL_Y;

    update(meas_x, meas_y);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EKFNode>());
    rclcpp::shutdown();
    return 0;
}
