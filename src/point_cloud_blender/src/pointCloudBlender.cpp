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

#include <Eigen/Dense>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class PointCloudBlender : public rclcpp:: Node{
    public:
        PointCloudBlender()
        : Node("pc_blender")
        {            
            // Init pub
            ang_pub_ = this->create_publisher<std_msgs::msg::Int32>("angle", 10);
            pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("merge_pc", 10);
            
            // Init sub
            bool_sub_ = this->create_subscription<std_msgs::msg::Header>("motion_bool", 10, std::bind(&PointCloudBlender::updateFlag, this, _1));
            pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("camera/camera/depth/color/points", 10, std::bind(&PointCloudBlender::updatePCL, this, _1));

            // Init timer
            timer_ = this->create_wall_timer(200ms, std::bind(&PointCloudBlender::publishCloud, this));

            // Init direction, angle and point cloud
            dirFlag = 1;
            angle = 0;
            captFlag = false;
            memoryCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();


            // Init publish angle to force rotation there
            this->publishAngle();
        }

    private:
        void updateFlag(const std_msgs::msg::Header::ConstSharedPtr &bool_msg)
        {
            if (bool_msg->frame_id == "1")
            {
                captFlag = true;
            }
        }

        void updatePCL(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pc_msg)
        {
            if (captFlag) // Means only when we want to take photos will we update the PC
            {
                captFlag = false;
                RCLCPP_INFO(this->get_logger(), "PCL Received");
                
                // Get point cloud
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
                pcl::fromROSMsg(*pc_msg, *cloud);
                pcl::PointCloud<pcl::PointXYZ>::Ptr trans_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

                if (memoryCloud->empty()){
                    RCLCPP_INFO(this->get_logger(), "PC Initialised");
                    Eigen::Matrix4f t_matrix = this->calcTransform(angle, 0);
                    pcl::transformPointCloud(*cloud, *trans_cloud, t_matrix);
                    // Default as first cloud
                    *memoryCloud = *cloud;
                }

                else
                {
                    // Transform new point cloud
                    Eigen::Matrix4f t_matrix = this->calcTransform(angle, 30);
                    pcl::transformPointCloud(*cloud, *trans_cloud, t_matrix);

                    // Add new point cloud to old point cloud
                    *memoryCloud += *trans_cloud;

                    // Filter cloud
                    pcl::VoxelGrid<pcl::PointXYZ> voxel;
                    voxel.setInputCloud(memoryCloud);
                    voxel.setLeafSize(0.05f, 0.05f, 0.05f); // 5cm voxels

                    pcl::PointCloud<pcl::PointXYZ>::Ptr filterCloud(new pcl::PointCloud<pcl::PointXYZ>); // temp filtered cloud
                    voxel.filter(*filterCloud);
                    memoryCloud = filterCloud; // Replace with new cloud
                }

                // Update direction of rotation
                if (angle >= 180){
                    dirFlag = -1;
                }
                else if (angle <= 0) {
                    dirFlag = 1;
                }

                // Update new angle
                angle = angle + dirFlag * 30;
                // Send new angle to Arduino, needs to be blocking
                this->publishAngle();
            }
        }
    
        void publishAngle()
        {
            auto message = std_msgs::msg::Int32();
            message.data = angle;
            RCLCPP_INFO(this->get_logger(), "Angle Published");
            ang_pub_->publish(message);

            captFlag = true;
        }

        void publishCloud()
        {
            auto message = sensor_msgs::msg::PointCloud2();
            pcl::toROSMsg(*memoryCloud, message);
            message.header.stamp = this->now();       // sets consistent timestamp
            message.header.frame_id = "map"; 
            RCLCPP_INFO(this->get_logger(), "Cloud updated");
            pc_pub_->publish(message);
        }
        
        Eigen::Matrix4f calcTransform(int phi, int gamma)
        {
            // // Get rotation matrix
            // Eigen::Matrix3f R0;
            // R0 << 1, 0, 0,
            //     0, cos(-1*gamma), -sin(-1*gamma),
            //     0, sin(-1*gamma), cos(-1*gamma);

            Eigen::Matrix3f R1;
            R1 << cos(phi), 0, sin(phi),
                0, 1, 0,
                -sin(phi), 0, cos(phi);

            Eigen::Matrix3f R2;
            R2 << cos(gamma), -sin(gamma), 0,
                sin(gamma), cos(gamma), 0,
                0, 0, 1;

            Eigen::Matrix3f R = R1 * R2;
            
            // Get transform matrix
            Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
            transform.block<3,3>(0,0) = R;

            return transform;
        };

        // Publishers
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr ang_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;

        // Subscribers
        rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr bool_sub_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
        
        // Constant
        int angle;
        pcl::PointCloud<pcl::PointXYZ>::Ptr memoryCloud;
        int dirFlag;
        bool captFlag;

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