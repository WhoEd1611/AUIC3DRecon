#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <boost/asio.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include <Eigen/Dense>

using std::placeholders::_1;
using std::placeholders::_2;

class ControlBlender : public rclcpp:: Node{
    public:
        ControlBlender()
        : Node("pc_blender"), serial_(io_)
        {            
            // Init arduino communication
            try {
                serial_.open("/dev/ttyACM0");
                serial_.set_option(boost::asio::serial_port_base::baud_rate(9600));
            }
            catch (boost::system::system_error &e) 
            {
                RCLCPP_ERROR(this->get_logger(), "Could not open serial port: %s", e.what());
                return;
            }

            
            // Init pub
            pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("merge_pc", 10);
            
            // Init sub
            pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "camera/camera/depth/color/points", 10, std::bind(&ControlBlender::blendPC, this, _1));

            // Init direction, angle and point cloud
            dirFlag = 1;
            angle = 0;
        }

    private:
        void blendPC(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pc_msg)
        {
            if (captFlag) // Means only when we want to take photos will we update the PC
            {
                captFlag = false;
                RCLCPP_INFO(this->get_logger(), "PC Received");
                
                // Get point cloud
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
                pcl::fromROSMsg(*pc_msg, *cloud);
                pcl::PointCloud<pcl::PointXYZ>::Ptr trans_cloud;

                if (memoryCloud->empty()){
                    Eigen::Matrix4f t_matrix = calcTransform(angle, 0);
                    pcl::transformPointCloud(*cloud, *trans_cloud, t_matrix);
                    // Default as first cloud
                    *memoryCloud = *cloud;
                    RCLCPP_INFO(this->get_logger(), "PC Initialised");
                }

                else
                {
                    // Transform new point cloud
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
                }
                // Publish point cloud
                this->publishCloud();

                // Send new angle to Arduino, needs to be blocking
                this->publishAngle();
            }
            else 
            {   
                // Publish point cloud
                this->publishCloud();
            }
        }

        void publishAngle()
        {
            // Send to Arduino angle
            if (serial_.is_open()) {
                
                // Update new angle
                if (angle >= 180){
                    dirFlag = -1;
                }
                else if (angle <= 0) {
                    dirFlag = 1;
                }
                
                angle = angle + dirFlag * 30;
                boost::asio::write(serial_, boost::asio::buffer(&angle, sizeof(angle)));
                RCLCPP_INFO(this->get_logger(), "Arduino messaged");

                // Wait for reply from Arduino
                boost::asio::streambuf buf;
                boost::asio::read_until(serial_, buf, "\n"); // Message recieved
                
                // Toogle boolean message because finished moving
                RCLCPP_INFO(this->get_logger(), "Arduino finished moving");
                captFlag = true;
            }
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
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;

        // Subscribers
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
        
        // Serial
        boost::asio::io_service io_;
        boost::asio::serial_port serial_;

        // Constant
        int angle;
        pcl::PointCloud<pcl::PointXYZ>::Ptr memoryCloud;
        int dirFlag;
        bool captFlag = true;
    };

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlBlender>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}