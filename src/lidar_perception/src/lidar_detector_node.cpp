// Libraries
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl-1.12/pcl/point_cloud.h>
#include <pcl-1.12/pcl/point_types.h>
#include <pcl-1.12/pcl/filters/passthrough.h>
#include <pcl-1.12/pcl/filters/extract_indices.h>
#include <pcl-1.12/pcl/segmentation/sac_segmentation.h>
#include <pcl-1.12/pcl/segmentation/extract_clusters.h>
#include <pcl-1.12/pcl/common/centroid.h>


/**
 * @brief LidarDetector Node
 * * This node processes raw LiDAR data to detect cones.
 * Pipeline:
 * 1. Filter ROI (PassThrough)
 * 2. Remove Ground Plane (RANSAC)
 * 3. Cluster remaining objects (Euclidean Clustering)
 * 4. Calculate Centroids and Publish
 */
class LidarDetector : public rclcpp::Node {
    private:
        // Subscriber that "hears" PointCloud2 messages
        // from the lidar
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar;

        // Publisher that sends PointCloud2 messages
        // that contain the positions of the cones that
        // were processed
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cones;

        // Topics
        std::string sub_topic_;
        std::string pub_topic_;

        // ROI (Region of Interest) limits to crop the PointCloud
        float x_filter_min_, x_filter_max_;
        float y_filter_min_, y_filter_max_;
        float z_filter_min_, z_filter_max_;

        // Ground Removal Threshold
        // Threshold distance (meters) to the model for a point
        // to be considered an inlier
        float distance_threshold_;

        // Clustering
        // Tolerance: Maximum distance between points to be considered
        //            part of the same cluster
        // Min/Max:   Filters out noise (too small) or walls/large obstacles
        float cluster_tolerance_;
        int min_cluster_size_, max_cluster_size_;

        /**
         * @brief Filters the point cloud to keep only points within specific limits
         * Useful for removing points that are not on the track (e.g. walls or trees)
         * @param cloud The input point cloud (modified in place)
         */
        void filter_coordinates(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

        /**
         * @brief Segments and removes the ground plane using RANSAC
         * @param cloud Input cloud containing ground and obstacles
         * @return pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud containing non ground points
         */
        pcl::PointCloud<pcl::PointXYZ>::Ptr ground_removal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

        /**
         * @brief Groups points into clusters based on spatial proximity
         * @param cloud_obstacles Cloud containing only obstacles
         * @return std::vector<pcl::PointIndices> Vector of indices, where each element represents one cluster
         */
        std::vector<pcl::PointIndices> euclidean_clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_obstacles);

        /**
         * @brief Calculates the geometric center (centroid) of each cluster
         * @param cluster_indices The indices of points belonging to each cluster
         * @param cloud_obstacles The actual point cloud data
         * @return pcl::PointCloud<pcl::PointXYZ>::Ptr A point cloud where each point is a center of a cone
         */
        pcl::PointCloud<pcl::PointXYZ>::Ptr find_centroids(std::vector<pcl::PointIndices> cluster_indices, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_obstacles);

        // Callback for PointCloud message
        void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    public:
        LidarDetector() : Node("lidar_detector") {
            // Declare parameters
            this->declare_parameter<std::string>("topics.sub_lidar", "/carla/hero/lidar");
            this->declare_parameter<std::string>("topics.pub_lidar", "/perception/lidar");

            this->declare_parameter<float>("roi.x_min" , 0.0);
            this->declare_parameter<float>("roi.x_max" , 25.0);
            this->declare_parameter<float>("roi.y_min" , -5.0);
            this->declare_parameter<float>("roi.y_max" , 5.0);
            this->declare_parameter<float>("roi.z_min" , -1.0);
            this->declare_parameter<float>("roi.z_max" , 1.0);

            this->declare_parameter<float>("ransac.distance_threshold", 0.15);

            this->declare_parameter<float>("clustering.tolerance", 0.3);
            this->declare_parameter<int>("clustering.min_size", 3);
            this->declare_parameter<int>("clustering.max_size", 150);

            // Read parameters into member variables
            sub_topic_  = this->get_parameter("topics.sub_lidar").as_string();
            pub_topic_ = this->get_parameter("topics.pub_lidar").as_string();

            x_filter_min_ = this->get_parameter("roi.x_min").as_double();
            x_filter_max_ = this->get_parameter("roi.x_max").as_double();
            y_filter_min_ = this->get_parameter("roi.y_min").as_double();
            y_filter_max_ = this->get_parameter("roi.y_max").as_double();
            z_filter_min_ = this->get_parameter("roi.z_min").as_double();
            z_filter_max_ = this->get_parameter("roi.z_max").as_double();

            distance_threshold_ = this->get_parameter("ransac.distance_threshold").as_double();

            cluster_tolerance_ = this->get_parameter("clustering.tolerance").as_double();
            min_cluster_size_ = this->get_parameter("clustering.min_size").as_int();
            max_cluster_size_ = this->get_parameter("clustering.max_size").as_int();

            // Create subscriber
            sub_lidar = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                sub_topic_,
                10,
                std::bind(&LidarDetector::lidar_callback, this, std::placeholders::_1)
            );

            // Create publisher
            pub_cones = this->create_publisher<sensor_msgs::msg::PointCloud2>(pub_topic_, 10);

            RCLCPP_INFO(this->get_logger(), "Cone Detector Node Started");
        }
};

void LidarDetector::filter_coordinates(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x"); pass.setFilterLimits(x_filter_min_, x_filter_max_); pass.filter(*cloud);
    pass.setFilterFieldName("y"); pass.setFilterLimits(y_filter_min_, y_filter_max_); pass.filter(*cloud);
    pass.setFilterFieldName("z"); pass.setFilterLimits(z_filter_min_, z_filter_max_); pass.filter(*cloud);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LidarDetector::ground_removal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    // Configure RANSAC for Plane fitting
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(distance_threshold_);
    seg.setInputCloud(cloud);

    // Execute segmentation
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) return NULL;

    // Extract indices: Keep points that are not the ground
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_obstacles(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_obstacles);

    return cloud_obstacles;
}

std::vector<pcl::PointIndices> LidarDetector::euclidean_clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_obstacles) {
    // KdTree object for fast search of nearest neighbors
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_obstacles);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclidean_extraction;

    euclidean_extraction.setClusterTolerance(cluster_tolerance_);
    euclidean_extraction.setMinClusterSize(min_cluster_size_);
    euclidean_extraction.setMaxClusterSize(max_cluster_size_);
    euclidean_extraction.setSearchMethod(tree);
    euclidean_extraction.setInputCloud(cloud_obstacles);

    euclidean_extraction.extract(cluster_indices);

    return cluster_indices;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LidarDetector::find_centroids(std::vector<pcl::PointIndices> cluster_indices, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_obstacles) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cone_centers(new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto& cluster : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZ> single_cone_cluster;

        // Populate the temporary cloud using the indices
        // from the main obstacle
        for (const auto& index : cluster.indices) {
            single_cone_cluster.push_back((*cloud_obstacles)[index]);
        }

        // Calculate Centroid  (x, y, z)
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(single_cone_cluster, centroid);

        cone_centers->push_back(pcl::PointXYZ(centroid[0], centroid[1], centroid[2]));
    }

    return cone_centers;
}

void LidarDetector::lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // Convert PointCloud Ros to PCL points (x, y, z)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    // Crop ROI
    filter_coordinates(cloud);

    // Remove Ground
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_obstacles = ground_removal(cloud);

    if (cloud_obstacles == NULL) {
        RCLCPP_INFO(this->get_logger(), "Could not detect ground plane.");
        return;
    }

    // Euclidean Clustering
    std::vector<pcl::PointIndices> cluster_indices = euclidean_clustering(cloud_obstacles);

    // Find Centroids
    pcl::PointCloud<pcl::PointXYZ>::Ptr cone_centers = find_centroids(cluster_indices, cloud_obstacles);

    // Publish results
    sensor_msgs::msg::PointCloud2 cones_msg;
    pcl::toROSMsg(*cone_centers, cones_msg);
    cones_msg.header = msg->header;
    pub_cones->publish(cones_msg);

    // RCLCPP_INFO(this->get_logger(), "Send Cones Centroids");
}


/**
 * @brief Standard ROS 2 entry point
 */
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarDetector>());
    rclcpp::shutdown();
    return 0;
}