#include <ctime>
#include <iostream>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

float z_threshold;

void
cloud_cb (const sensor_msgs::msg::PointCloud2::ConstSharedPtr& input)
{

    clock_t start_t, looptime;
    double loop;
    start_t = clock();

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg (*input, cloud);



    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (z_threshold);
    seg.setMaxIterations(800);

    seg.setAxis({0,0,1.0});
    seg.setEpsAngle(0.3); 

    seg.setInputCloud (cloud.makeShared());
    seg.segment (*inliers, *coefficients);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud.makeShared());
    extract.setIndices (inliers);
    extract.setNegative(true);


    // pcl::PointCloud<pcl::PointXYZI>::Ptr XYZ_RANSAC(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZ> XYZ_RANSAC;
    extract.filter(XYZ_RANSAC);

    looptime = clock() - start_t;
    loop = float(looptime)/CLOCKS_PER_SEC;

    std::cout <<"ransac time: " << loop << std::endl;

    pcl::PCLPointCloud2 output_ransac;
    sensor_msgs::msg::PointCloud2 output_ransac_complete;
    pcl::toPCLPointCloud2(XYZ_RANSAC, output_ransac);
    pcl_conversions::fromPCL(output_ransac,output_ransac_complete);

    output_ransac_complete.header.frame_id = "velodyne";
    pub_output->publish(output_ransac_complete);
    //sensor_msgs::msg::PointCloud2 outmsg;
    //pcl::toROSMsg(XYZ_RANSAC, outmsg);
    //pub.publish(outmsg);
}



class RansacNode : public rclcpp::Node
{
public:
    RansacNode()
        : Node("ransac_out")
    {
        z_threshold = static_cast<float>(this->declare_parameter<double>("z_threshold", 0.1));
        const auto input_topic = this->declare_parameter<std::string>("topics.input", "roi_out");
        const auto output_topic = this->declare_parameter<std::string>("topics.output", "ransac_out");
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic, rclcpp::SensorDataQoS(), cloud_cb);
        pub_output = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            output_topic, rclcpp::SensorDataQoS());
    }
private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RansacNode>());
    rclcpp::shutdown();
    return 0;
}
