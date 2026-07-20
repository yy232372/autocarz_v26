#include <algorithm>
#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <autocarz/msg/waypoint_info.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

int globalNum = 0;
#define MAX_BUFF 200
#define WORDS_PER_LINE 4

#define MINIMUM_POINTS 10
#define EPSILON 0.75


rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_pub_in;
rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr max_roi_pub;


typedef pcl::PointXYZI PointXYZI;


int roiNum;
int roiNum_start;
bool is_livox;
float radius=1.20;
int interval=2;
int way_indx[MAX_BUFF]={0};  //roi의 waypoint를 담는다 -100은 waypoint의 끝을 의미
// int roiWayNum=0; 이거 대신 -100으로 끝을 표현함
float htMat[12]={};
int lane;
float z_threshold = 0.1; //  VLP 16 0.1 livox 0.4
float point_step = 0.3; ///[m]
float max_roi_x = 0;
double radius_dim = 0.0;


std::vector<float> globalPath_x;
std::vector<float> globalPath_y;
std::vector<float> globalPath_radius;

bool checkReadPath=false;
bool checkOdom=false;
bool checkCurWaypointIndx=false;


void readPath(std::string globalPath_file) {
    std::ifstream fin(globalPath_file);
    if (!fin.is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("roi_in"), "Could not open the path file: %s", globalPath_file.c_str());
        return;
    }

    globalPath_x.clear();
    globalPath_y.clear();
    globalPath_radius.clear();

    float x = 0.0f;
    float y = 0.0f;
    float unused = 0.0f;
    float radius_value = 0.0f;
    while (fin >> x >> y >> unused >> radius_value) {
        globalPath_x.push_back(x);
        globalPath_y.push_back(y);
        globalPath_radius.push_back(radius_value);
    }

    globalNum = static_cast<int>(globalPath_x.size());
    if (globalNum == 0) {
        RCLCPP_ERROR(rclcpp::get_logger("roi_in"), "Path file contains no valid 4-column rows: %s", globalPath_file.c_str());
        return;
    }

    RCLCPP_INFO(rclcpp::get_logger("roi_in"), "[roi_in.cpp] loaded %d path points", globalNum);
    checkReadPath = true;
}

void pc_cb(const sensor_msgs::msg::PointCloud2::SharedPtr input_pcl_raw){
    // RCLCPP_INFO(rclcpp::get_logger("roi_in"), "pc_cb");
    // if(checkReadPath && checkOdom && checkCurWaypointIndx) {
    if(checkReadPath && checkCurWaypointIndx) {
        
        pcl::PCLPointCloud2 input_pcl;
        pcl::PCLPointCloud2 output_pcl;
        pcl::PCLPointCloud2 output_map;
        pcl::PointCloud<pcl::PointXYZI> XYZ_array;
        pcl::PointCloud<pcl::PointXYZI> XYZ_array_output;
        pcl::PointXYZI M;
        pcl_conversions::toPCL(*input_pcl_raw, input_pcl);       // ros_pointcloud2  -->  pcl_pointcloud2
        pcl::fromPCLPointCloud2(input_pcl, XYZ_array);        // pcl_pointcloud2  -->  pcl_xyzrgb array

        // ---------------------------------------------------------------------
        int l=0;
        bool bool_check=0;
        float rsquare=radius*radius;

        // ---------------------------------------------------------------------
        float x, y, z;
        int w_indx=0;

        for(unsigned int i=0; i<XYZ_array.size(); i++) {
            if ( XYZ_array.points[i].z < z_threshold){
                if ( fabs(XYZ_array.points[i].y) >0.5 ||  fabs (XYZ_array.points[i].x) > 0.7) {


                    x=XYZ_array.points[i].x;
                    y=XYZ_array.points[i].y;
                    z=XYZ_array.points[i].z;

                    M.x = htMat[0]*x+htMat[1]*y+htMat[2]*z+htMat[3];
                    M.y = htMat[4]*x+htMat[5]*y+htMat[6]*z+htMat[7];
                    M.z = htMat[8]*x+htMat[9]*y+htMat[10]*z+htMat[11];

                    for (l = 0 ;  way_indx[l]!=-100 && l < MAX_BUFF ; l++) {
                        w_indx=way_indx[l];

                        rsquare=(globalPath_radius[w_indx]-radius_dim)*(globalPath_radius[w_indx]-radius_dim);
                        bool_check=(globalPath_x[w_indx]-M.x)*(globalPath_x[w_indx]-M.x)+(globalPath_y[w_indx]-M.y)*(globalPath_y[w_indx]-M.y)<=rsquare;

                        if(bool_check>0) {
                            // XYZ_array.points[i].intensity=1;
                            XYZ_array_output.push_back(XYZ_array.points[i]);
                            bool_check=0;
                            max_roi_x = std::max(max_roi_x, XYZ_array.points[i].x);
                            break;
                        }
                        
                    }
                }
            }   

        }



    // ---------------------------------------------------------------------------------------------
        sensor_msgs::msg::PointCloud2 output_pcl_complete;
        pcl::toPCLPointCloud2(XYZ_array_output, output_pcl);
        pcl_conversions::fromPCL(output_pcl, output_pcl_complete); 
        output_pcl_complete.header.frame_id = "velodyne";
        cluster_pub_in->publish(output_pcl_complete);

        if (is_livox){
            std_msgs::msg::Float32 max_roi_dis;
            max_roi_dis.data = max_roi_x;
            max_roi_pub->publish(max_roi_dis);
        }

        checkOdom=false;
        checkCurWaypointIndx=false;
        max_roi_x = 0;
    }
}

void odom_cb(const nav_msgs::msg::Odometry::SharedPtr odometry){
    if(checkReadPath) {

        float qx=odometry->pose.pose.orientation.x;
        float qy=odometry->pose.pose.orientation.y;
        float qz=odometry->pose.pose.orientation.z;
        float qw=odometry->pose.pose.orientation.w;

        htMat[0]=1.-2.*qy*qy-2.*qz*qz;
        htMat[1]=2.*qx*qy-2.*qw*qz;
        htMat[2]=2.*qx*qz+2.*qw*qy;
        htMat[3]=odometry->pose.pose.position.x;
        htMat[4]=2.*qx*qy+2.*qw*qz;
        htMat[5]=1.-2.*qx*qx-2.*qz*qz;
        htMat[6]=2.*qy*qz-2.*qw*qx;
        htMat[7]=odometry->pose.pose.position.y;
        htMat[8]=2.*qx*qz-2.*qw*qy;
        htMat[9]=2.*qy*qz+2.*qw*qx;
        htMat[10]=1.-2.*qx*qx-2.*qy*qy;
        htMat[11]=odometry->pose.pose.position.z;

        checkOdom=true;
    }
}

void waypointInfo_cb(const autocarz::msg::WaypointInfo::SharedPtr info){
    int i=0;
    lane = info->lane;
    if(checkReadPath) {
        int cur_way=info->indx_in;
        if(info->lane==1 || is_livox) {   //forward
            way_indx[0]=cur_way+roiNum_start;
            if(way_indx[0]>globalNum-1)
                way_indx[0]-=globalNum;
            for(i=1; i<roiNum/interval + 1; i++) {
                way_indx[i]=way_indx[i-1]+interval;
                if(way_indx[i]>globalNum-1)
                    way_indx[i]-=globalNum;
            }
        }
        else {             //forward && back
            way_indx[0]=cur_way-roiNum;
            if(way_indx[0]<0)
                way_indx[0]+=globalNum;
            for(i=1; i<2*roiNum/interval +1; i++) {
                way_indx[i]=way_indx[i-1]+interval;
                if(way_indx[i]>globalNum-1)
                    way_indx[i]-=globalNum;
            }
        }
        way_indx[i+1]=-100; // for marking where way_indx ends
        checkCurWaypointIndx=true;

    }
}


class RoiNode : public rclcpp::Node
{
public:
    RoiNode()
        : Node("roi_in")
    {
        std::string path_value = this->declare_parameter<std::string>("path_in", "");
        const auto package_share = ament_index_cpp::get_package_share_directory("autocarz");
        const std::string global_path_file = path_value.empty() || path_value.front() != '/'
            ? package_share + "/" + path_value
            : path_value;
        const auto pointcloud_topic = this->declare_parameter<std::string>("topics.pointcloud", "");
        const auto odom_topic = this->declare_parameter<std::string>("topics.odom", "/odom");
        const auto waypoint_topic = this->declare_parameter<std::string>("topics.waypoint_info", "/waypointInfo");
        const auto roi_output_topic = this->declare_parameter<std::string>("topics.roi", "roi_in");
        const auto max_roi_topic = this->declare_parameter<std::string>("topics.max_roi", "max_roi_dis_in");
        roiNum = this->declare_parameter<int>("roi_idxlen", 0);
        roiNum_start = this->declare_parameter<int>("roi_idx_start", 0);
        is_livox = this->declare_parameter<bool>("is_livox", false);
        radius_dim = this->declare_parameter<double>("radius_dim", 0.0);
        if (is_livox) { z_threshold = 0.4F; }

        readPath(global_path_file);
        const auto control_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
        pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            pointcloud_topic, rclcpp::SensorDataQoS(), pc_cb);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, control_qos, odom_cb);
        waypoint_sub_ = this->create_subscription<autocarz::msg::WaypointInfo>(
            waypoint_topic, control_qos, waypointInfo_cb);
        cluster_pub_in = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            roi_output_topic, rclcpp::SensorDataQoS());
        if (is_livox) {
            max_roi_pub = this->create_publisher<std_msgs::msg::Float32>(
                max_roi_topic, control_qos);
        }
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<autocarz::msg::WaypointInfo>::SharedPtr waypoint_sub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoiNode>());
    rclcpp::shutdown();
    return 0;
}
