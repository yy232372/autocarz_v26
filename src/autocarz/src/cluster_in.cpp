#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <typeinfo>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <autocarz/msg/waypoint_info.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "dbscan.h"

#define MINIMUM_POINTS 10
#define EPSILON 0.75
#define SAME_Z_THRESHOLD 0.25

std::string LANE;

rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr side_back_distance;
rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr side_front_distance;
rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr front_distance;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_pub_in;

typedef pcl::PointXYZI PointXYZI;


float radius=1.20;
int interval=3;

int lane;
bool checkCurWaypointIndx=false;


void waypointInfo_cb(const autocarz::msg::WaypointInfo::SharedPtr info){
    lane = info->lane;

    checkCurWaypointIndx=true;
    
}


void pc_cb(const sensor_msgs::msg::PointCloud2::SharedPtr input_pcl_raw){

    if ( checkCurWaypointIndx ){
        pcl::PCLPointCloud2 input_pcl;
        pcl::PCLPointCloud2 output_pcl;
        pcl::PCLPointCloud2 output_map;
        pcl::PointCloud<pcl::PointXYZI> XYZ_array;
        pcl::PointCloud<pcl::PointXYZI> XYZ_array_output;
        pcl::PointXYZI M;
        pcl_conversions::toPCL(*input_pcl_raw, input_pcl);       // ros_pointcloud2  -->  pcl_pointcloud2
        pcl::fromPCLPointCloud2(input_pcl, XYZ_array);        // pcl_pointcloud2  -->  pcl_xyzrgb array

        // ---------------------------------------------------------------------
        clock_t start, finish;
        double duration_time;
        start = clock();
// ----------------- Clustering --------//
        std::vector<Point> points;
        std::size_t num_points;
        int Max_ID;
        std_msgs::msg::Float32 min_x, min_front, min_back;
        num_points = XYZ_array.points.size();
        RCLCPP_DEBUG(rclcpp::get_logger("cluster_in"), "input points: %zu", num_points);

        Point *p = (Point *)calloc(num_points, sizeof(Point));
        

        for(std::size_t i=0; i<num_points ; i++) {
            
            p[i].x=XYZ_array.points[i].x;
            p[i].y=XYZ_array.points[i].y;
            p[i].z=XYZ_array.points[i].z;
            p[i].clusterID = -1;
            points.push_back(p[i]);

        }
        free(p);

        DBSCAN ds( MINIMUM_POINTS , EPSILON, SAME_Z_THRESHOLD, points);


        ds.run();
        
       
        Max_ID =0;
        min_x.data = 30.0  ;
        min_front.data = 30.0 ;
        min_back.data = - 15.0;

        if (lane == 1 ){
            for(std::size_t i=0; i<num_points ; i++){
                bool is_same_z = (std::find(ds.same_z_clusterID.begin(), ds.same_z_clusterID.end(), ds.m_points[i].clusterID) != ds.same_z_clusterID.end());
                if (ds.m_points[i].clusterID > 0 && !is_same_z){
                    XYZ_array.points[i].intensity = ds.m_points[i].clusterID  ;
                    XYZ_array_output.push_back(XYZ_array.points[i]);
                    if (Max_ID < ds.m_points[i].clusterID){
                        Max_ID = ds.m_points[i].clusterID;
                    }
                    //std::cout<<"x,y,z"<<","<< ds.m_points[i].x <<"," << ds.m_points[i].y << ""<< ds.m_points[i].z << "," << ds.m_points[i].clusterID<<std::endl;
                    //std::cout<<"Intensity"<<XYZ_array.points[i].intensity<<std::endl;
                    //std::cout << "x,y " << ds.m_points[i].x <<", "<< ds.m_points[i].y<<std::endl;

                    if (ds.m_points[i].x > 0 && ds.m_points[i].x < min_x.data){
                        min_x.data = ds.m_points[i].x;
                        //lidar location calculate

                    }
                }
            }
        }
        else{
            for(std::size_t i=0; i<num_points ; i++){
                bool is_same_z = (std::find(ds.same_z_clusterID.begin(), ds.same_z_clusterID.end(), ds.m_points[i].clusterID) != ds.same_z_clusterID.end());

                if (ds.m_points[i].clusterID > 0 && !is_same_z){

                    XYZ_array.points[i].intensity = ds.m_points[i].clusterID  ;
                    XYZ_array_output.push_back(XYZ_array.points[i]);
                    
                    if (Max_ID < ds.m_points[i].clusterID){
                        Max_ID = ds.m_points[i].clusterID;
                    }
                    if ( ds.m_points[i].x > 0 && ds.m_points[i].x < min_front.data){
                        
                        min_front.data = ds.m_points[i].x;

                    }
                    else if(ds.m_points[i].x < 0 && ds.m_points[i].x > min_back.data){
                        min_back.data = ds.m_points[i].x;
                    }
                }
            }
        }
        finish = clock();
        duration_time = double(finish-start) / CLOCKS_PER_SEC;
        RCLCPP_DEBUG(rclcpp::get_logger("cluster_in"),
            "cluster time: %.6f, max ID: %d", duration_time, Max_ID);


    // ---------------------------------------------------------------------------------------------
        sensor_msgs::msg::PointCloud2 output_pcl_complete;
        pcl::toPCLPointCloud2(XYZ_array_output, output_pcl);
        pcl_conversions::fromPCL(output_pcl, output_pcl_complete); 
        output_pcl_complete.header = input_pcl_raw->header;
        cluster_pub_in->publish(output_pcl_complete);
        if (lane==1){
            front_distance->publish(min_x);
        }
        else{
            side_front_distance->publish(min_front);
            side_back_distance->publish(min_back);
        }

        checkCurWaypointIndx=false;
    }
}






class ClusterNode : public rclcpp::Node
{
public:
    ClusterNode()
        : Node("cluster_in")
    {
        const auto input_topic = this->declare_parameter<std::string>("topics.input", "ransac_in");
        const auto waypoint_topic = this->declare_parameter<std::string>("topics.waypoint_info", "/waypointInfo");
        const auto front_topic = this->declare_parameter<std::string>("topics.distance_front", "distance_front");
        const auto side_front_topic = this->declare_parameter<std::string>("topics.distance_side_front", "distance_side_front");
        const auto side_back_topic = this->declare_parameter<std::string>("topics.distance_side_back", "distance_side_back");
        const auto cluster_topic = this->declare_parameter<std::string>("topics.cluster", "cluster_in");

        const auto control_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
        pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic, rclcpp::SensorDataQoS(), pc_cb);
        waypoint_sub_ = this->create_subscription<autocarz::msg::WaypointInfo>(
            waypoint_topic, control_qos, waypointInfo_cb);
        front_distance = this->create_publisher<std_msgs::msg::Float32>(front_topic, control_qos);
        side_front_distance = this->create_publisher<std_msgs::msg::Float32>(side_front_topic, control_qos);
        side_back_distance = this->create_publisher<std_msgs::msg::Float32>(side_back_topic, control_qos);
        cluster_pub_in = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            cluster_topic,
            rclcpp::QoS(rclcpp::KeepLast(5)).reliable().durability_volatile());
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
    rclcpp::Subscription<autocarz::msg::WaypointInfo>::SharedPtr waypoint_sub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ClusterNode>();
    rclcpp::spin(node);
    side_back_distance.reset();
    side_front_distance.reset();
    front_distance.reset();
    cluster_pub_in.reset();
    node.reset();
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    return 0;
}
