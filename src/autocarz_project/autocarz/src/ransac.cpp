#include <memory>
#include <string>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

class PointCloudProcessor : public rclcpp::Node
{
public:
    PointCloudProcessor()
        : Node("point_cloud_processor")
    {
        z_threshold_ = this->declare_parameter<double>("z_threshold", 0.1);
        topic_input_ = this->declare_parameter<std::string>("topics.input", "/roi/roi");
        topic_output_ = this->declare_parameter<std::string>("topics.output", "ransac_output");

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic_input_,
            rclcpp::SensorDataQoS(),
            std::bind(&PointCloudProcessor::cloudCallback, this, std::placeholders::_1));
        pub_output_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            topic_output_, rclcpp::SensorDataQoS());

        RCLCPP_INFO(this->get_logger(),
                    "PointCloudProcessor initialized with z_threshold: %.3f",
                    z_threshold_);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_output_;
    double z_threshold_;
    std::string topic_input_;
    std::string topic_output_;

    void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*input, cloud);

        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setAxis(Eigen::Vector3f(0.0F, 0.0F, 1.0F));
        seg.setEpsAngle(0.45);
        seg.setDistanceThreshold(z_threshold_);
        seg.setMaxIterations(1000);
        seg.setInputCloud(cloud.makeShared());
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty()) {
            RCLCPP_ERROR(this->get_logger(),
                         "Could not estimate a planar model for the given dataset.");
            return;
        }

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud.makeShared());
        extract.setIndices(inliers);
        extract.setNegative(true);

        pcl::PointCloud<pcl::PointXYZ> non_ground_cloud;
        extract.filter(non_ground_cloud);

        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(non_ground_cloud, output);
        output.header = input->header;
        pub_output_->publish(output);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudProcessor>());
    rclcpp::shutdown();
    return 0;
}
