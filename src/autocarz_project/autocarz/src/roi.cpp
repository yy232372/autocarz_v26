#include <algorithm>
#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#define MAX_BUFF 300
#define WORDS_PER_LINE 5

class PointCloudProcessor : public rclcpp::Node
{
public:
    PointCloudProcessor()
        : Node("roi")
    {
        globalPathFileIn_ = this->declare_parameter<std::string>("path_in", "");
        globalPathFileOut_ = this->declare_parameter<std::string>("path_out", "");
        pcTopic_ = this->declare_parameter<std::string>("topics.pointcloud", "");
        roiNum_ = this->declare_parameter<int>("roi_idxlen", 0);
        roiNumStart_ = this->declare_parameter<int>("roi_idx_start", 0);
        isLivox_ = this->declare_parameter<bool>("is_livox", false);
        radiusDim_ = this->declare_parameter<double>("radius_dim", 0.0);
        interval_ = this->declare_parameter<int>("interval", 6);

        const auto odom_topic = this->declare_parameter<std::string>("topics.odom", "/odom");
        const auto in_index_topic = this->declare_parameter<std::string>(
            "topics.cur_waypoint_idx_in", "/curWaypointIdxIn");
        const auto out_index_topic = this->declare_parameter<std::string>(
            "topics.cur_waypoint_idx_out", "/curWaypointIdxOut");
        const auto roi_topic = this->declare_parameter<std::string>("topics.roi", "roi");
        const auto voxel_topic = this->declare_parameter<std::string>(
            "topics.voxel_grid", "voxel_grid");
        const auto max_roi_topic = this->declare_parameter<std::string>(
            "topics.max_roi", "max_roi_dis_in");

        zThreshold_ = isLivox_ ? 0.4F : 0.2F;
        readPathIn(globalPathFileIn_);
        readPathOut(globalPathFileOut_);

        const auto control_qos = rclcpp::QoS(rclcpp::KeepLast(10))
            .reliable().durability_volatile();
        pcSub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            pcTopic_, rclcpp::SensorDataQoS(),
            std::bind(&PointCloudProcessor::pointCloudCallback, this, std::placeholders::_1));
        odomSub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, control_qos,
            std::bind(&PointCloudProcessor::odomCallback, this, std::placeholders::_1));
        InwaypointInfoSub_ = this->create_subscription<std_msgs::msg::Int32>(
            in_index_topic, control_qos,
            std::bind(&PointCloudProcessor::InwaypointInfoCallback, this, std::placeholders::_1));
        OutwaypointInfoSub_ = this->create_subscription<std_msgs::msg::Int32>(
            out_index_topic, control_qos,
            std::bind(&PointCloudProcessor::OutwaypointInfoCallback, this, std::placeholders::_1));

        clusterPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            roi_topic, rclcpp::SensorDataQoS());
        voxelPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            voxel_topic, rclcpp::SensorDataQoS());
        if (isLivox_) {
            maxRoiPub_ = this->create_publisher<std_msgs::msg::Float32>(
                max_roi_topic, control_qos);
        }
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcSub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr InwaypointInfoSub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr OutwaypointInfoSub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clusterPub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxelPub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr maxRoiPub_;

    std::string globalPathFileIn_;
    std::string globalPathFileOut_;
    std::string pcTopic_;
    int roiNum_;
    int roiNumStart_;
    int interval_;
    bool isLivox_;
    float zThreshold_;
    double radiusDim_;
    std::vector<float> globalInPathX_;
    std::vector<float> globalInPathY_;
    std::vector<float> globalInPathRadius_;
    std::vector<float> globalOutPathX_;
    std::vector<float> globalOutPathY_;
    std::vector<float> globalOutPathRadius_;
    bool checkReadPath_ = false;
    bool checkOdom_ = false;
    bool IncheckCurWaypointIndx_ = false;
    bool OutcheckCurWaypointIndx_ = false;
    float maxRoiX_ = 0.0F;
    int wayIndx_[MAX_BUFF] = {0};
    float htMat_[12] = {0.0F};

    void readPathIn(const std::string& globalPathFile)
    {
        std::ifstream fin(globalPathFile);
        if (!fin.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open the file: %s", globalPathFile.c_str());
            return;
        }

        float value;
        int indx = 0;
        while (fin >> value)
        {
            if (indx % WORDS_PER_LINE == 0) globalInPathX_.push_back(value);
            else if (indx % WORDS_PER_LINE == 1) globalInPathY_.push_back(value);
            else if (indx % WORDS_PER_LINE == 3) globalInPathRadius_.push_back(value);
            indx++;
        }
        if (indx / WORDS_PER_LINE != globalInPathX_.size()) {
            RCLCPP_WARN(this->get_logger(), "Path lines: %lu did not match globalNum: %lu", globalInPathX_.size(), indx / WORDS_PER_LINE);
        }
        checkReadPath_ = true;
    }

    void readPathOut(const std::string& globalPathFile)
    {
        std::ifstream fin(globalPathFile);
        if (!fin.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open the file: %s", globalPathFile.c_str());
            return;
        }

        float value;
        int indx = 0;
        while (fin >> value)
        {
            if (indx % WORDS_PER_LINE == 0) globalOutPathX_.push_back(value);
            else if (indx % WORDS_PER_LINE == 1) globalOutPathY_.push_back(value);
            else if (indx % WORDS_PER_LINE == 3) globalOutPathRadius_.push_back(value);
            indx++;
        }
        if (indx / WORDS_PER_LINE != globalOutPathX_.size()) {
            RCLCPP_WARN(this->get_logger(), "Path lines: %lu did not match globalNum: %lu", globalOutPathX_.size(), indx / WORDS_PER_LINE);
        }
        checkReadPath_ = true;
    }

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& inputPclRaw)
    {
        if (checkReadPath_ && (IncheckCurWaypointIndx_ || OutcheckCurWaypointIndx_))
        {
            // std::cout << "roi" << std::endl;
            pcl::PointCloud<pcl::PointXYZI> inputCloud, voxelCloud, roiCloud;
            pcl::fromROSMsg(*inputPclRaw, inputCloud);


            pcl::VoxelGrid<pcl::PointXYZI> voxelFilter;
            voxelFilter.setInputCloud(inputCloud.makeShared());
            voxelFilter.setLeafSize(0.1f, 0.1f, 0.1f);
            voxelFilter.filter(voxelCloud);

            sensor_msgs::msg::PointCloud2 voxelOutput;
            pcl::toROSMsg(voxelCloud, voxelOutput);
            voxelOutput.header.frame_id = "base_link";
            voxelPub_->publish(voxelOutput);


            // std::cout<< IncheckCurWaypointIndx_ << std:: endl;
            if (IncheckCurWaypointIndx_) {
                processPointCloudIn(voxelCloud, roiCloud);    // 복셀화
                // processPointCloudIn(inputCloud, roiCloud);    // 복셀화 X
                std::cout << "roi_in" << std::endl;
            }
            else {
                processPointCloudOut(voxelCloud, roiCloud);    // 복셀화
                // processPointCloudOut(inputCloud, roiCloud);    // 복셀화 X
                std::cout << "roi_out" << std::endl;
            }
            sensor_msgs::msg::PointCloud2 outputPclComplete;
            pcl::toROSMsg(roiCloud, outputPclComplete);
            outputPclComplete.header.frame_id = "base_link";
            clusterPub_->publish(outputPclComplete);

            checkOdom_ = false;
            IncheckCurWaypointIndx_ = false;
            OutcheckCurWaypointIndx_ = false;
            maxRoiX_ = 0;

        }
    }

    void processPointCloudIn(const pcl::PointCloud<pcl::PointXYZI>& inputCloud, pcl::PointCloud<pcl::PointXYZI>& outputCloud)
    {
        float rsquare, x, y, z;
        int l, wIndx;
        bool boolCheck;

        for (const auto& point : inputCloud) {
            if (point.z < zThreshold_ && (fabs(point.y) > 0.5 || fabs(point.x) > 0.7))
            {
                pcl::PointXYZI transformedPoint;
                transformPoint(point, transformedPoint);

                for (l = 0; wayIndx_[l] != -100 && l < MAX_BUFF; l++)
                {
                    wIndx = wayIndx_[l];
                    rsquare = pow(globalInPathRadius_[wIndx] - radiusDim_, 2);
                    boolCheck = pow(globalInPathX_[wIndx] - transformedPoint.x, 2) +
                                pow(globalInPathY_[wIndx] - transformedPoint.y, 2) <= rsquare;

                    if (boolCheck) {
                        outputCloud.push_back(point);
                        maxRoiX_ = std::max(maxRoiX_, point.x);
                        break;
                    }
                }
            }
        }
    }

    void processPointCloudOut(const pcl::PointCloud<pcl::PointXYZI>& inputCloud, pcl::PointCloud<pcl::PointXYZI>& outputCloud)
    {
        float rsquare, x, y, z;
        int l, wIndx;
        bool boolCheck;

        for (const auto& point : inputCloud) {
            if (point.z < zThreshold_ && (fabs(point.y) > 0.5 || fabs(point.x) > 0.7))
            {
                pcl::PointXYZI transformedPoint;
                transformPoint(point, transformedPoint);

                for (l = 0; wayIndx_[l] != -100 && l < MAX_BUFF; l++)
                {
                    wIndx = wayIndx_[l];
                    rsquare = pow(globalOutPathRadius_[wIndx] - radiusDim_, 2);
                    boolCheck = pow(globalOutPathX_[wIndx] - transformedPoint.x, 2) +
                                pow(globalOutPathY_[wIndx] - transformedPoint.y, 2) <= rsquare;

                    if (boolCheck) {
                        outputCloud.push_back(point);
                        maxRoiX_ = std::max(maxRoiX_, point.x);
                        break;
                    }
                }
            }
        }
    }

    void transformPoint(const pcl::PointXYZI& inputPoint, pcl::PointXYZI& outputPoint) {
        outputPoint.x = htMat_[0] * inputPoint.x + htMat_[1] * inputPoint.y + htMat_[2] * inputPoint.z + htMat_[3];
        outputPoint.y = htMat_[4] * inputPoint.x + htMat_[5] * inputPoint.y + htMat_[6] * inputPoint.z + htMat_[7];
        outputPoint.z = htMat_[8] * inputPoint.x + htMat_[9] * inputPoint.y + htMat_[10] * inputPoint.z + htMat_[11];
    }

    void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& odometry) {
        if (checkReadPath_) {
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
    }

    // PP전용
    // void waypointInfoCallback(const autocarz::msg::WaypointInfo::SharedPtr info) {
    //     if (checkReadPath_) {
    //         int curWay = info->indx_in;
    //         // std::cout<< curWay << std::endl;
    //         int i = 0;

    //         if (isLivox_) {
    //             wayIndx_[0] = curWay + roiNumStart_;
    //             if (wayIndx_[0] >= globalPathX_.size()) {
    //                 wayIndx_[0] -= globalPathX_.size();
    //             }

    //             for (i = 1; i < roiNum_ / interval_ + 1; i++) {
    //                 wayIndx_[i] = wayIndx_[i - 1] + interval_;
    //                 if (wayIndx_[i] >= globalPathX_.size()) {
    //                     wayIndx_[i] -= globalPathX_.size();
    //                 }
    //             }
    //         }
    //         else {
    //             int b = 0;
    //             wayIndx_[0] = curWay - roiNum_*3;
    //             if (wayIndx_[0] < 0) {
    //                 wayIndx_[0] += globalPathX_.size();
    //             }

    //             for (i = 1; i < 2 * roiNum_*3 / interval_ + 1; i++) {
    //                 wayIndx_[i] = wayIndx_[i - 1] + interval_;
    //                 if (wayIndx_[i] >= globalPathX_.size()) {
    //                     wayIndx_[i] -= globalPathX_.size();
    //                 }
    //             }
    //         }
            
    //         // Ensure we don't exceed the array bounds
    //         if (i + 1 < MAX_BUFF) {
    //             wayIndx_[i + 1] = -100;
    //         } else {
    //             RCLCPP_WARN(this->get_logger(), "Index exceeds MAX_BUFF limit, skipping assignment to wayIndx_");
    //         }

    //         checkCurWaypointIndx_ = true;
    //     }
    // }

    // MPC전용
    void InwaypointInfoCallback(const std_msgs::msg::Int32::ConstSharedPtr& info) {
        if (checkReadPath_) {
            int curWay = static_cast<int>(info->data);
            int i = 0;
            if (isLivox_) {
                wayIndx_[0] = curWay + roiNumStart_;
                if (wayIndx_[0] >= globalInPathX_.size()) {
                    wayIndx_[0] -= globalInPathX_.size();
                }

                for (i = 1; i < roiNum_ / interval_ + 1; i++) {
                    wayIndx_[i] = wayIndx_[i - 1] + interval_;
                    if (wayIndx_[i] >= globalInPathX_.size()) {
                        wayIndx_[i] -= globalInPathX_.size();
                    }
                }
            }
            else {
                int b = 0;
                wayIndx_[0] = curWay - roiNum_*3;
                if (wayIndx_[0] < 0) {
                    wayIndx_[0] += globalInPathX_.size();
                }

                for (i = 1; i < 2 * roiNum_*3 / interval_ + 1; i++) {
                    wayIndx_[i] = wayIndx_[i - 1] + interval_;
                    if (wayIndx_[i] >= globalInPathX_.size()) {
                        wayIndx_[i] -= globalInPathX_.size();
                    }
                }
            }
            
            // Ensure we don't exceed the array bounds
            if (i + 1 < MAX_BUFF) {
                wayIndx_[i + 1] = -100;
            } else {
                RCLCPP_WARN(this->get_logger(), "Index exceeds MAX_BUFF limit, skipping assignment to wayIndx_");
            }

            IncheckCurWaypointIndx_ = true;
        }
    }

    void OutwaypointInfoCallback(const std_msgs::msg::Int32::ConstSharedPtr& info) {
        if (checkReadPath_) {
            int curWay = static_cast<int>(info->data);
            // std::cout<< curWay << std::endl;
            int i = 0;
            if (isLivox_) {
                wayIndx_[0] = curWay + roiNumStart_;
                if (wayIndx_[0] >= globalOutPathX_.size()) {
                    wayIndx_[0] -= globalOutPathX_.size();
                }

                for (i = 1; i < roiNum_ / interval_ + 1; i++) {
                    wayIndx_[i] = wayIndx_[i - 1] + interval_;
                    if (wayIndx_[i] >= globalOutPathX_.size()) {
                        wayIndx_[i] -= globalOutPathX_.size();
                    }
                }
            }
            else {
                int b = 0;
                wayIndx_[0] = curWay - roiNum_*3;
                if (wayIndx_[0] < 0) {
                    wayIndx_[0] += globalOutPathX_.size();
                }

                for (i = 1; i < 2 * roiNum_*3 / interval_ + 1; i++) {
                    wayIndx_[i] = wayIndx_[i - 1] + interval_;
                    if (wayIndx_[i] >= globalOutPathX_.size()) {
                        wayIndx_[i] -= globalOutPathX_.size();
                    }
                }
            }
            
            // Ensure we don't exceed the array bounds
            if (i + 1 < MAX_BUFF) {
                wayIndx_[i + 1] = -100;
            } else {
                RCLCPP_WARN(this->get_logger(), "Index exceeds MAX_BUFF limit, skipping assignment to wayIndx_");
            }

            OutcheckCurWaypointIndx_ = true;
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudProcessor>());
    rclcpp::shutdown();
    return 0;
}
