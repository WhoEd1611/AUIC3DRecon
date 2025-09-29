#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/registration/icp.h>

#include <Eigen/Dense>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class PointCloudBlender : public rclcpp:: Node{
    public:
        PointCloudBlender()
        : Node("icp_blender")
        {            
            // Init pub
            ang_pub_ = this->create_publisher<std_msgs::msg::Int32>("angle", 10);
            pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("merge_pc", 10);
            
            // Init sub
            bool_sub_ = this->create_subscription<std_msgs::msg::Header>("motion_bool", 10, std::bind(&PointCloudBlender::updateFlag, this, _1));
            pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("camera/camera/depth/color/points", 10, std::bind(&PointCloudBlender::updatePCL, this, _1));

            // Init timer
            timer_ = this->create_wall_timer(1s, std::bind(&PointCloudBlender::publishCloud, this));

            // Init direction, angle and point cloud
            dirFlag = 1;
            angle = 0;
            captFlag = true;
            memoryCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
            initFlag = true;
            offsetAng = 45;
            dAng = 30;
            prevAng = 0;

            // Init publish angle to force rotation there
            this->publishAngle();
        }

    private:
        void updateFlag(const std_msgs::msg::Header::ConstSharedPtr &bool_msg)
        {
            if (bool_msg->frame_id == "1")
            {
                RCLCPP_INFO(this->get_logger(), "Angle updated");
                captFlag = true;
            }
        }

        void updatePCL(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pc_msg)
        {
            if (captFlag) // Means only when we want to take photos will we update the PC
            {
                RCLCPP_INFO(this->get_logger(), "PCL Received");
                captFlag = false;
   
                // Get point cloud
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
                pcl::fromROSMsg(*pc_msg, *cloud);

                if (initFlag){
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr trans_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
                    RCLCPP_INFO(this->get_logger(), "PC Initialised");
                    Eigen::Matrix4f t_matrix = this->calcTransform(offsetAng, 0);
                    pcl::transformPointCloud(*cloud, *trans_cloud, t_matrix);
                    // Default as first cloud
                    *memoryCloud = *trans_cloud;
                    initFlag = false;
                }

                else
                {
                    // Transform new point cloud
                    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
                    pcl::PointCloud<pcl::PointXYZRGB> cloud_aligned;
                    icp.setInputSource(cloud);
                    icp.setInputTarget(memoryCloud);

                    icp.align(cloud_aligned);

                    // Add new point cloud to old point cloud
                    *memoryCloud = *memoryCloud + cloud_aligned;

                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterCloud(new pcl::PointCloud<pcl::PointXYZRGB>); // temp filtered cloud


                    // Radius Outlier Removal
                    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> ror;
                    ror.setInputCloud(memoryCloud);
                    ror.setRadiusSearch(0.001);   // 1 mm radius
                    ror.setMinNeighborsInRadius(1);  // keep if at least one neighbor
                    ror.filter(*filterCloud);

                    // // Voxel Filter cloud
                    // pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
                    // voxel.setInputCloud(memoryCloud);
                    // voxel.setLeafSize(0.01f, 0.01f, 0.01f); // 1cm voxels

                    // voxel.filter(*filterCloud);

                    *memoryCloud = *filterCloud; // Replace with new cloud
                    RCLCPP_INFO(this->get_logger(), "Cloud updated");
                }

                // Update direction of rotation
                if (angle >= 180){
                    prevAng = 180;
                    angle = 0;
                }
                else {
                    // Update new angle
                    prevAng = angle;
                    angle = angle + dAng;
                }
                // Send new angle to Arduino, needs to be blocking
                this->publishAngle();
            }

            // this->publishCloud();
        }
    
        void publishAngle()
        {
            auto message = std_msgs::msg::Int32();
            message.data = angle;
            RCLCPP_INFO(this->get_logger(), "Angle Published");
            ang_pub_->publish(message);
        }

        void publishCloud()
        {
            auto message = sensor_msgs::msg::PointCloud2();
            pcl::toROSMsg(*memoryCloud, message);
            message.header.stamp = this->now();       // sets consistent timestamp
            message.header.frame_id = "camera_depth_optical_frame"; 
            RCLCPP_INFO(this->get_logger(), "Cloud sent");
            pc_pub_->publish(message);
        }
        
        Eigen::Matrix4f calcTransform(int phi_deg, int gamma_deg)
        {
            float phi = phi_deg * M_PI / 180.0f;    // pitch
            float gamma = gamma_deg * M_PI / 180.0f; // yaw / rotation around axis

            // Rotation around camera x-axis (pitch)
            Eigen::Matrix3f Rx;
            Rx << 1, 0, 0,
                0, cos(phi), -sin(phi),
                0, sin(phi), cos(phi);

            // Rotation around z-axis (gamma)
            Eigen::Matrix3f Rz;
            Rz << cos(gamma), -sin(gamma), 0,
                sin(gamma),  cos(gamma), 0,
                0,           0,          1;

            // Optional flips if using optical frame conventions
            Eigen::Matrix3f Rzneg;
            Rzneg << 1, 0, 0,
                    0, 1, 0,
                    0, 0,-1;

            Eigen::Matrix3f Ryneg;
            Ryneg << 1, 0, 0,
                    0,-1, 0,
                    0, 0, 1;

            // Total rotation
            Eigen::Matrix3f R = Rz * Ryneg * Rzneg * Rx;
            Eigen::Matrix4f T1 = Eigen::Matrix4f::Identity();
            T1.block<3,3>(0,0) = R; 

            // Offset from rotation axis (3 cm along X)
            Eigen::Vector3f offsetp(0.03f, 0.0f, 0.0f); // 3 cm in global X
            Eigen::Matrix4f T0 = Eigen::Matrix4f::Identity();
            T0.block<3,1>(0,3) = offsetp;

            // Offset from rotation axis (-3 cm along X)
            Eigen::Vector3f offsetn(-0.03f, 0.0f, 0.0f); // 3 cm in global X
            Eigen::Matrix4f T2 = Eigen::Matrix4f::Identity();
            T2.block<3,1>(0,3) = offsetn;


            // Build 4x4 homogeneous transform
            Eigen::Matrix4f transform = T1*T0;

            return transform;
        }


        // Publishers
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr ang_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;

        // Subscribers
        rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr bool_sub_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
        
        // Constant
        int angle;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr memoryCloud;
        int dirFlag;
        bool captFlag;
        int initFlag;

        int offsetAng;
        int dAng;
        int prevAng;

        // Timer
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudBlender>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}