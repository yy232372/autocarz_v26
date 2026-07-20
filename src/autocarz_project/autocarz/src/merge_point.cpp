#include <cmath>
#include <memory>
#include <queue>
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
        : Node("pointcloud_merger"), livox_base_time_set_(false), livox_time_offset_(0.0)
    {
        livox_x_offset_ = this->declare_parameter<double>("livox_x_offset", 0.3);
        livox_theta_ = this->declare_parameter<double>(
            "livox_y_rotation_theta", -1.0 * 3.14 / 180.0);
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
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr vlp_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr livox_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merged_pub_;

    double livox_x_offset_;
    double livox_theta_;
    std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> vlp_queue_;
    std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> livox_queue_;
    bool livox_base_time_set_;
    double livox_time_offset_;

    void vlpCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr vlp_msg)
    {
        vlp_queue_.push(vlp_msg);
        processPointClouds();
    }

    void livoxCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr livox_msg)
    {
        const double livox_stamp = rclcpp::Time(livox_msg->header.stamp).seconds();
        if (!livox_base_time_set_) {
            livox_time_offset_ = this->now().seconds() - livox_stamp;
            livox_base_time_set_ = true;
        }
        livox_queue_.push(livox_msg);
        processPointClouds();
    }

    void processPointClouds()
    {
        while (!vlp_queue_.empty() && !livox_queue_.empty()) {
            const auto vlp_msg = vlp_queue_.front();
            const auto livox_msg = livox_queue_.front();

            const double vlp_time = rclcpp::Time(vlp_msg->header.stamp).seconds();
            const double livox_time =
                rclcpp::Time(livox_msg->header.stamp).seconds() + livox_time_offset_;
            const double time_diff = std::fabs(vlp_time - livox_time);

            if (time_diff < 0.1) {
                mergePointClouds(vlp_msg, livox_msg);
                vlp_queue_.pop();
                livox_queue_.pop();
            } else if (vlp_time < livox_time) {
                vlp_queue_.pop();
            } else {
                livox_queue_.pop();
            }
        }
    }

    void mergePointClouds(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr vlp_msg,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr livox_msg)
    {
        pcl::PointCloud<pcl::PointXYZI> vlp_cloud;
        pcl::PointCloud<pcl::PointXYZI> livox_cloud;
        pcl::PointCloud<pcl::PointXYZI> merged_cloud;

        pcl::fromROSMsg(*vlp_msg, vlp_cloud);
        pcl::fromROSMsg(*livox_msg, livox_cloud);

        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.rotate(Eigen::AngleAxisf(
            static_cast<float>(livox_theta_), Eigen::Vector3f::UnitY()));

        for (auto & point : livox_cloud) {
            Eigen::Vector3f point_vector(point.x, point.y, point.z);
            point_vector = transform * point_vector;
            point.x = point_vector.x() + static_cast<float>(livox_x_offset_);
            point.y = point_vector.y();
            point.z = point_vector.z() - 0.1F;
        }

        merged_cloud = vlp_cloud + livox_cloud;
        sensor_msgs::msg::PointCloud2 merged_msg;
        pcl::toROSMsg(merged_cloud, merged_msg);
        merged_msg.header.stamp = this->now().to_msg();
        merged_msg.header.frame_id = "base_link";
        merged_pub_->publish(merged_msg);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudMerger>());
    rclcpp::shutdown();
    return 0;
}
