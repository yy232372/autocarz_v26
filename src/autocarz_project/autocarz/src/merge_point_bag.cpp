#include <memory>
#include <string>
#include <functional>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PointCloudMerger : public rclcpp::Node
{
public:
    PointCloudMerger()
        : Node("pointcloud_merger_bag")
    {
        livox_x_offset_ = this->declare_parameter<double>("livox_x_offset", 0.3);
        const auto vlp_topic = this->declare_parameter<std::string>(
            "topics.vlp", "/velodyne_points");
        const auto livox_topic = this->declare_parameter<std::string>(
            "topics.livox", "/livox/lidar");
        const auto merged_topic = this->declare_parameter<std::string>(
            "topics.merged", "merged_pointcloud");

        vlp_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            vlp_topic, rclcpp::SensorDataQoS(),
            std::bind(&PointCloudMerger::vlpCallback, this, std::placeholders::_1));
        livox_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            livox_topic, rclcpp::SensorDataQoS(),
            std::bind(&PointCloudMerger::livoxCallback, this, std::placeholders::_1));
        merged_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            merged_topic, rclcpp::SensorDataQoS());

        RCLCPP_INFO(this->get_logger(), "PointCloudMerger initialized.");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr vlp_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr livox_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merged_pub_;
    sensor_msgs::msg::PointCloud2::ConstSharedPtr vlp_msg_;
    sensor_msgs::msg::PointCloud2::ConstSharedPtr livox_msg_;
    double livox_x_offset_;

    void vlpCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
    {
        vlp_msg_ = msg;
        processPointClouds();
    }

    void livoxCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
    {
        livox_msg_ = msg;
        processPointClouds();
    }

    void processPointClouds()
    {
        if (!vlp_msg_ || !livox_msg_) {
            return;
        }

        pcl::PointCloud<pcl::PointXYZI> vlp_cloud;
        pcl::PointCloud<pcl::PointXYZI> livox_cloud;
        pcl::PointCloud<pcl::PointXYZI> merged_cloud;
        pcl::fromROSMsg(*vlp_msg_, vlp_cloud);
        pcl::fromROSMsg(*livox_msg_, livox_cloud);

        for (auto & point : livox_cloud) {
            point.x += static_cast<float>(livox_x_offset_);
        }

        merged_cloud = vlp_cloud + livox_cloud;
        sensor_msgs::msg::PointCloud2 merged_msg;
        pcl::toROSMsg(merged_cloud, merged_msg);
        merged_msg.header.stamp = this->now().to_msg();
        merged_msg.header.frame_id = "base_link";
        merged_pub_->publish(merged_msg);

        vlp_msg_.reset();
        livox_msg_.reset();
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudMerger>());
    rclcpp::shutdown();
    return 0;
}
