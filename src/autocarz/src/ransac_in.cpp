#include <algorithm>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <iostream>
#include <memory>
#include <string>

#include <Eigen/Eigenvalues>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_output;

float z_threshold;


void
cloud_cb (const sensor_msgs::msg::PointCloud2::ConstSharedPtr& input)
{

    const auto declared_points =
        static_cast<std::uint64_t>(input->width) * input->height;
    if (declared_points < 3 || input->data.empty()) {
        RCLCPP_DEBUG(rclcpp::get_logger("ransac_in"),
            "Skipping RANSAC: PointCloud2 contains fewer than 3 points");
        return;
    }

    clock_t start_t, looptime;
    double loop;
    start_t = clock();

    pcl::PointCloud<pcl::PointXYZI> input_cloud;
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg (*input, input_cloud);

    // PCL의 SACSegmentation은 빈 점군, NaN 또는 일직선뿐인 점군을 받으면
    // stderr에 "No samples could be selected"를 직접 출력한다. 이는 센서
    // 고장이 아니라 ROI 결과가 희소한 정상 프레임일 수 있으므로 선검사한다.
    cloud.reserve(input_cloud.size());
    for (const auto & point : input_cloud.points) {
        if (std::isfinite(point.x) && std::isfinite(point.y) &&
            std::isfinite(point.z)) {
            cloud.push_back(point);
        }
    }
    cloud.width = static_cast<std::uint32_t>(cloud.size());
    cloud.height = 1;
    cloud.is_dense = true;

    if (cloud.size() < 3) {
        RCLCPP_DEBUG(rclcpp::get_logger("ransac_in"),
            "Skipping RANSAC: only %zu finite ROI points", cloud.size());
        return;
    }

    Eigen::Vector4f centroid;
    Eigen::Matrix3f covariance;
    pcl::computeMeanAndCovarianceMatrix(cloud, covariance, centroid);
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance);
    if (eigen_solver.info() != Eigen::Success ||
        eigen_solver.eigenvalues()(2) <= 0.0F ||
        eigen_solver.eigenvalues()(1) /
            eigen_solver.eigenvalues()(2) < 1.0e-6F) {
        RCLCPP_DEBUG(rclcpp::get_logger("ransac_in"),
            "Skipping RANSAC: ROI points are geometrically degenerate");
        return;
    }



    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setAxis({0,0,1.0});
    seg.setEpsAngle(0.3); 
    seg.setDistanceThreshold (z_threshold);
    seg.setMaxIterations(800);
    const auto cloud_ptr = cloud.makeShared();
    seg.setInputCloud (cloud_ptr);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
        // 빈/희소 프레임은 처리 생략 대상이며 프로세스 오류가 아니다.
        RCLCPP_DEBUG(rclcpp::get_logger("ransac_in"),
            "Could not estimate a planar model for this frame");
        return ;
    }

    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(cloud_ptr);
    extract.setIndices (inliers);
    extract.setNegative(true);

    pcl::PointCloud<pcl::PointXYZI> XYZ_RANSAC;
    extract.filter(XYZ_RANSAC);

    looptime = clock() - start_t;
    loop = float(looptime)/CLOCKS_PER_SEC;

    RCLCPP_DEBUG(rclcpp::get_logger("ransac_in"), "ransac time: %.6f", loop);

    pcl::PCLPointCloud2 output_ransac;
    sensor_msgs::msg::PointCloud2 output_ransac_complete;
    pcl::toPCLPointCloud2(XYZ_RANSAC, output_ransac);
    pcl_conversions::fromPCL(output_ransac,output_ransac_complete);

    output_ransac_complete.header = input->header;
    pub_output->publish(output_ransac_complete);
}



class RansacNode : public rclcpp::Node
{
public:
    RansacNode()
        : Node("ransac_in")
    {
        z_threshold = static_cast<float>(this->declare_parameter<double>("z_threshold", 0.1));
        const auto input_topic = this->declare_parameter<std::string>("topics.input", "roi_in");
        const auto output_topic = this->declare_parameter<std::string>("topics.output", "ransac_in");
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic, rclcpp::SensorDataQoS(), cloud_cb);
        pub_output = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            output_topic,
            rclcpp::QoS(rclcpp::KeepLast(5)).reliable().durability_volatile());
    }
private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RansacNode>();
    rclcpp::spin(node);
    pub_output.reset();
    node.reset();
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    return 0;
}
