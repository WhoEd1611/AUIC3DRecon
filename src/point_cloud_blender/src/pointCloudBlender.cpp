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

using std::placeholders::_1;
using std::placeholders::_2;


using namespace std::chrono_literals;

class PointCloudBlender : public rclcpp:: Node{
    public:
        PointCloudBlender()
        : Node("pc_blender"), angle(0)
        {
        ang_pub_ = this->create_publisher<std_msgs::msg::Int32>("angle", 10);
        pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("merge_pc", 10);
        bool_sub_.subscribe(this, "motion_bool");
        pc_sub_.subscribe(this, "camera/image_raw");

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

        void confirmAngle(const std_msgs::msg::Bool::SharedPtr bool_msg,  
                          const sensor_msgs::msg::PointCloud2::ConstSharedPtr pc_msg)
        {
            if (bool_msg->data) {
                // Get point cloud
                pcl::PointCloud<pcl::PointXYZ> cloud;
                pcl::fromROSMsg(*pc_msg, cloud);
                if (memoryCloud.empty()){
                    memoryCloud = cloud;
                }
                else
                {
                // Transform new point cloud
                // Merge point clouds together
                // Publish point cloud
                this->publishCloud();
                }
                
                // Publish new angle
                if (angle >= 180){
                    dirFlag = -1;
                }
                else if (angle <= 0) {
                    dirFlag = 1;
                }
                
                angle = angle + dirFlag * 30;
                
                this->publishAngle();
            }

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
        pcl::PointCloud<pcl::PointXYZ> memoryCloud;
        int dirFlag;
};