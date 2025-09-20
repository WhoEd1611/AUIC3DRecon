#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
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
            bool_sub_.subscribe(this, "motion_bool");
            pc_sub_.subscribe(this, "camera/camera/depth/color/points");

            sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
                SyncPolicy(10), bool_sub_, pc_sub_);

            // Register callback
            sync_->registerCallback(std::bind(&PointCloudBlender::confirmAngle, this, _1, _2));

            // Init direction, angle and point cloud
            dirFlag = 1;
            angle = 0;
        }

    private:
        void publishAngle()
        {
            auto message = std_msgs::msg::Int32();
            message.data = angle;
            RCLCPP_INFO(this->get_logger(), "Angle: '%f'", angle);
            ang_pub_->publish(message);
        }

        void publishCloud()
        {
            auto message = sensor_msgs::msg::PointCloud2();
            pcl::toROSMsg(*memoryCloud, message);
            RCLCPP_INFO(this->get_logger(), "Cloud updated");
            pc_pub_->publish(message);
        }

        void confirmAngle(const std_msgs::msg::Bool::SharedPtr bool_msg,  
                          const sensor_msgs::msg::PointCloud2::ConstSharedPtr pc_msg)
        {
            if (bool_msg->data) {
                // Get point cloud
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
                pcl::fromROSMsg(*pc_msg, *cloud);
                if (memoryCloud->empty()){
                    // Default as first cloud
                    *memoryCloud = *cloud;
                }
                else
                {
                    // Transform new point cloud
                    pcl::PointCloud<pcl::PointXYZ>::Ptr trans_cloud;
                    Eigen::Matrix4f t_matrix = calcTransform(angle, 30);
                    pcl::transformPointCloud(*cloud, *trans_cloud, t_matrix);

                    // Add new point cloud to old point cloud
                    *memoryCloud += *trans_cloud;

                    // Filter cloud
                    pcl::VoxelGrid<pcl::PointXYZ> voxel;
                    voxel.setInputCloud(memoryCloud);
                    voxel.setLeafSize(0.01f, 0.01f, 0.01f); // 1cm voxels

                    pcl::PointCloud<pcl::PointXYZ>::Ptr filterCloud(new pcl::PointCloud<pcl::PointXYZ>); // temp filtered cloud
                    voxel.filter(*filterCloud);
                    memoryCloud = filterCloud; // Replace with new cloud

                    // Publish point cloud
                    this->publishCloud();
                }
                
                // Update new angle
                if (angle >= 180){
                    dirFlag = -1;
                }
                else if (angle <= 0) {
                    dirFlag = 1;
                }
                
                angle = angle + dirFlag * 30;
                
                // Publish new angle
                this->publishAngle();
            }

        };
        
        Eigen::Matrix4f calcTransform(int phi, int gamma)
        {
            // Get rotation matrix
            Eigen::Matrix3f R0;
            R0 << 1, 0, 0,
                0, cos(-1*gamma), -sin(-1*gamma),
                0, sin(-1*gamma), cos(-1*gamma);

            Eigen::Matrix3f R1;
            R1 << cos(phi), 0, sin(phi),
                0, 1, 0,
                -sin(phi), 0, cos(phi);

            Eigen::Matrix3f R2;
            R2 << cos(gamma), -sin(gamma), 0,
                sin(gamma), cos(gamma), 0,
                0, 0, 1;

            Eigen::Matrix3f R = R2 * R1 * R0;
            
            // Get transform matrix
            Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
            transform.block<3,3>(0,0) = R;

            return transform;
        };

        // Publishers
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr ang_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;

        // Subscribers
        message_filters::Subscriber<std_msgs::msg::Bool> bool_sub_;
        message_filters::Subscriber<sensor_msgs::msg::PointCloud2> pc_sub_;

        // Synchronizer
        typedef message_filters::sync_policies::ApproximateTime<
        std_msgs::msg::Bool,
        sensor_msgs::msg::PointCloud2> SyncPolicy;

        std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
        
        // Constant
        int angle;
        pcl::PointCloud<pcl::PointXYZ>::Ptr memoryCloud;
        int dirFlag;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudBlender>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}