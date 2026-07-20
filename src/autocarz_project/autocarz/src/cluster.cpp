#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <vector>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "dbscan.h"

#define MINIMUM_POINTS 4
#define EPSILON 1.5
#define SAME_Z_THRESHOLD 0.20

class PointCloudCluster : public rclcpp::Node
{
public:
    PointCloudCluster()
        : Node("cluster"), checkOdom_(false)
    {
        lane_ = this->declare_parameter<int>("lane", 1);
        const auto pointcloud_topic = this->declare_parameter<std::string>(
            "topics.pointcloud", "/vlp/ransac/ransac_output");
        const auto odom_topic = this->declare_parameter<std::string>(
            "topics.odom", "/odom");
        const auto cluster_topic = this->declare_parameter<std::string>(
            "topics.cluster", "cluster");
        const auto obstacle_topic = this->declare_parameter<std::string>(
            "topics.obstacle", "obstacle");
        const auto pose_array_topic = this->declare_parameter<std::string>(
            "topics.pose_array", "pose_array");
        const auto central_pose_topic = this->declare_parameter<std::string>(
            "topics.central_pose", "central_pose");

        const auto control_qos = rclcpp::QoS(rclcpp::KeepLast(10))
            .reliable().durability_volatile();

        pcSub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            pointcloud_topic, rclcpp::SensorDataQoS(),
            std::bind(&PointCloudCluster::pointCloudCallback, this, std::placeholders::_1));
        odomSub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, control_qos,
            std::bind(&PointCloudCluster::odomCallback, this, std::placeholders::_1));
        clusterPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            cluster_topic, rclcpp::SensorDataQoS());
        obstaclePub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            obstacle_topic, control_qos);
        poseArrayPub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
            pose_array_topic, control_qos);
        centralPosePub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            central_pose_topic, control_qos);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcSub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clusterPub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr obstaclePub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr poseArrayPub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr centralPosePub_;
    int lane_;
    bool checkOdom_;
    float htMat_[12] = {0.0F};

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & inputPclRaw) {
        if (checkOdom_) {
            std::cout<< "cluster" << std::endl;
            pcl::PointCloud<pcl::PointXYZI> inputCloud, outputCloud;
            pcl::fromROSMsg(*inputPclRaw, inputCloud);
            std::vector<Point> points(inputCloud.points.size());
            for (size_t i = 0; i < inputCloud.points.size(); ++i) {
                points[i].x = inputCloud.points[i].x;
                points[i].y = inputCloud.points[i].y;
                points[i].z = inputCloud.points[i].z;
                points[i].clusterID = -1;
            }
            DBSCAN ds(MINIMUM_POINTS, EPSILON, SAME_Z_THRESHOLD, points);
            ds.run();

            processClusters(ds, inputCloud, outputCloud);

            sensor_msgs::msg::PointCloud2 outputPclComplete;
            pcl::toROSMsg(outputCloud, outputPclComplete);
            outputPclComplete.header.frame_id = "base_link";
            clusterPub_->publish(outputPclComplete);

            publishPoseArray(outputCloud);
        }
    }

    void processClusters(DBSCAN& ds, const pcl::PointCloud<pcl::PointXYZI>& inputCloud, pcl::PointCloud<pcl::PointXYZI>& outputCloud) {
        std_msgs::msg::Float32MultiArray obstacleArray;
        unsigned int maxID = 0;

        // 클러스터의 min/max 좌표를 추적하기 위한 맵
        std::map<int, std::pair<std::pair<float, float>, std::pair<float, float>>> clusterBounds;

        for (size_t i = 0; i < inputCloud.points.size(); ++i) {
            bool is_same_z = std::find(ds.same_z_clusterID.begin(), ds.same_z_clusterID.end(), ds.m_points[i].clusterID) != ds.same_z_clusterID.end();
            if (ds.m_points[i].clusterID > 0 && !is_same_z) {
                pcl::PointXYZI point = inputCloud.points[i];
                point.intensity = ds.m_points[i].clusterID;
                outputCloud.push_back(point);

                // 클러스터 ID가 처음 등장한 경우에만 초기화
                if (clusterBounds.find(ds.m_points[i].clusterID) == clusterBounds.end()) {
                    clusterBounds[ds.m_points[i].clusterID] = {
                        {point.x, point.x}, // min_x, max_x 초기화
                        {point.y, point.y}  // min_y, max_y 초기화
                    };
                } else {
                    auto& bounds = clusterBounds[ds.m_points[i].clusterID];
                    bounds.first.first = std::min(bounds.first.first, point.x);   // min_x
                    bounds.first.second = std::max(bounds.first.second, point.x); // max_x
                    bounds.second.first = std::min(bounds.second.first, point.y); // min_y
                    bounds.second.second = std::max(bounds.second.second, point.y); // max_y
                }
            }
        }

        for (const auto& cluster : clusterBounds) {
            // 각 클러스터의 중간값 계산
            float mid_x = (cluster.second.first.first + cluster.second.first.second) / 2.0;  // (min_x + max_x) / 2
            float mid_y = (cluster.second.second.first + cluster.second.second.second) / 2.0; // (min_y + max_y) / 2

            pcl::PointXYZI midPoint, transformedMidPoint;
            midPoint.x = mid_x;
            midPoint.y = mid_y;
            midPoint.z = 0.0; // 클러스터 중심의 z값이 필요하면 추가

            transformPoint(midPoint, transformedMidPoint);

            // 클러스터의 최소 및 최대 x, y 값 출력
            //         cluster.first, 
            //         cluster.second.first.first,  // min_x
            //         cluster.second.first.second, // max_x
            //         cluster.second.second.first, // min_y
            //         cluster.second.second.second // max_y
            // );

            // 결과를 obstacleArray에 저장
            // obstacleArray.data.push_back(mid_x);
            // obstacleArray.data.push_back(mid_y);
            obstacleArray.data.push_back(transformedMidPoint.x);
            obstacleArray.data.push_back(transformedMidPoint.y);
            obstacleArray.data.push_back(cluster.second.first.first);  // min_x
            obstacleArray.data.push_back(cluster.second.second.first); // min_y
            obstacleArray.data.push_back(cluster.second.first.second); // max_x
            obstacleArray.data.push_back(cluster.second.second.second); // max_y

            // 중앙값을 PoseStamped 메시지로 퍼블리시
            geometry_msgs::msg::PoseStamped centralPose;
            centralPose.header.stamp = this->now().to_msg();  // 타임스탬프 추가
            centralPose.header.frame_id = "base_link";  // 프레임 ID 설정
            centralPose.pose.position.x = mid_x;
            centralPose.pose.position.y = mid_y;
            centralPose.pose.position.z = 0.0;  // z 축에 대한 정보가 필요하면 설정

            // 회전은 없으므로 단위 쿼터니언을 사용
            centralPose.pose.orientation.x = 0.0;
            centralPose.pose.orientation.y = 0.0;
            centralPose.pose.orientation.z = 0.0;
            centralPose.pose.orientation.w = 1.0;

            centralPosePub_->publish(centralPose);
        }
        obstaclePub_->publish(obstacleArray);
    }


    void publishPoseArray(const pcl::PointCloud<pcl::PointXYZI>& cloud) {
        geometry_msgs::msg::PoseArray poseArrayMsg;
        poseArrayMsg.header.stamp = this->now().to_msg();
        poseArrayMsg.header.frame_id = "base_link";  // Change this to the appropriate frame

        for (const auto& point : cloud.points) {
            geometry_msgs::msg::Pose pose;
            pose.position.x = point.x;
            pose.position.y = point.y;
            pose.position.z = point.z; // Use point.z if it's relevant, or set it to 0 for a 2D plane

            // Set orientation to identity (no rotation)
            pose.orientation.x = 0.0;
            pose.orientation.y = 0.0;
            pose.orientation.z = 0.0;
            pose.orientation.w = 1.0;

            poseArrayMsg.poses.push_back(pose);
        }

        poseArrayPub_->publish(poseArrayMsg);
    }

    void transformPoint(const pcl::PointXYZI& inputPoint, pcl::PointXYZI& outputPoint) {
        outputPoint.x = htMat_[0] * inputPoint.x + htMat_[1] * inputPoint.y + htMat_[2] * inputPoint.z + htMat_[3];
        outputPoint.y = htMat_[4] * inputPoint.x + htMat_[5] * inputPoint.y + htMat_[6] * inputPoint.z + htMat_[7];
        outputPoint.z = htMat_[8] * inputPoint.x + htMat_[9] * inputPoint.y + htMat_[10] * inputPoint.z + htMat_[11];
    }

    void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& odometry) {
        const auto& pose = odometry->pose.pose;
        const auto& orientation = pose.orientation;

        htMat_[0] = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z);
        htMat_[1] = 2.0 * (orientation.x * orientation.y - orientation.w * orientation.z);
        htMat_[2] = 2.0 * (orientation.x * orientation.z + orientation.w * orientation.y);
        htMat_[3] = pose.position.x;
        htMat_[4] = 2.0 * (orientation.x * orientation.y + orientation.w * orientation.z);
        htMat_[5] = 1.0 - 2.0 * (orientation.x * orientation.x + orientation.z * orientation.z);
        htMat_[6] = 2.0 * (orientation.y * orientation.z - orientation.w * orientation.x);
        htMat_[7] = pose.position.y;
        htMat_[8] = 2.0 * (orientation.x * orientation.z - orientation.w * orientation.y);
        htMat_[9] = 2.0 * (orientation.y * orientation.z + orientation.w * orientation.x);
        htMat_[10] = 1.0 - 2.0 * (orientation.x * orientation.x + orientation.y * orientation.y);
        htMat_[11] = pose.position.z;

        checkOdom_ = true;
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudCluster>());
    rclcpp::shutdown();
    return 0;
}
