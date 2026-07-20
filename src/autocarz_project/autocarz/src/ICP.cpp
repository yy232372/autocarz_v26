#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

class ICPNode : public rclcpp::Node
{
public:
    ICPNode()
        : Node("icp_node"),
          source_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
          target_cloud_(new pcl::PointCloud<pcl::PointXYZ>)
    {
        const auto source_topic = this->declare_parameter<std::string>(
            "topics.source", "/velodyne_points");
        const auto target_topic = this->declare_parameter<std::string>(
            "topics.target", "/livox/lidar");
        const auto filtered_topic = this->declare_parameter<std::string>(
            "topics.filtered", "/filtered_velodyne_points");

        source_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            source_topic,
            rclcpp::SensorDataQoS(),
            std::bind(&ICPNode::sourceCallback, this, std::placeholders::_1));
        target_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            target_topic,
            rclcpp::SensorDataQoS(),
            std::bind(&ICPNode::targetCallback, this, std::placeholders::_1));
        filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            filtered_topic, rclcpp::SensorDataQoS());
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr source_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr target_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pub_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_;

    void sourceCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(
            new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *source_cloud_);
        manualFilterPointCloud(source_cloud_, filtered_cloud);

        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*filtered_cloud, output);
        output.header = cloud_msg->header;
        filtered_pub_->publish(output);

        if (!target_cloud_->empty()) {
            runICP(filtered_cloud);
        }
    }

    void targetCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
    {
        pcl::fromROSMsg(*cloud_msg, *target_cloud_);
        if (!source_cloud_->empty()) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(
                new pcl::PointCloud<pcl::PointXYZ>);
            manualFilterPointCloud(source_cloud_, filtered_cloud);
            runICP(filtered_cloud);
        }
    }

    static void manualFilterPointCloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr & output_cloud)
    {
        output_cloud->clear();
        const double angle_min = -M_PI / 4.0;
        const double angle_max = M_PI / 4.0;

        for (const auto & point : input_cloud->points) {
            const double angle = std::atan2(point.y, point.x);
            if (angle >= angle_min && angle <= angle_max) {
                output_cloud->points.push_back(point);
            }
        }

        output_cloud->width = static_cast<std::uint32_t>(output_cloud->points.size());
        output_cloud->height = 1;
        output_cloud->is_dense = true;
    }

    void runICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr & filtered_cloud)
    {
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(filtered_cloud);
        icp.setInputTarget(target_cloud_);

        pcl::PointCloud<pcl::PointXYZ> final_cloud;
        icp.align(final_cloud);

        if (icp.hasConverged()) {
            RCLCPP_INFO(this->get_logger(),
                        "ICP converged. fitness_score=%.6f",
                        icp.getFitnessScore());
            const Eigen::Matrix4f transformation = icp.getFinalTransformation();
            std::cout << "Transformation matrix:\n" << transformation << std::endl;
        } else {
            RCLCPP_WARN(this->get_logger(), "ICP did not converge.");
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ICPNode>());
    rclcpp::shutdown();
    return 0;
}
