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
using namespace std::chrono_literals;

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

            // Init timer
            timer_ = this->create_wall_timer(200ms, std::bind(&ControlBlender::publishCloud, this));
            
            // Init direction, angle and point cloud
            dirFlag = 1;
            angle = 0;
            memoryCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

            // Delay init so arduino can init
            rclcpp::Rate rate(1); // 1 Hz = 1 second per cycle
            RCLCPP_INFO(this->get_logger(), "Sleeping for 1 second...");
            rate.sleep(); // pauses for 1 second
        }

    private:
        void blendPC(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pc_msg)
        {
            if (captFlag) // Means only when we want to take photos will we update the PC
            {
                captFlag = false;
                RCLCPP_INFO(this->get_logger(), "PC Received");
                
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
                    voxel.setLeafSize(0.05f, 0.05f, 0.05f); // 10cm voxels

                    pcl::PointCloud<pcl::PointXYZ>::Ptr filterCloud(new pcl::PointCloud<pcl::PointXYZ>); // temp filtered cloud
                    voxel.filter(*filterCloud);
                    memoryCloud = filterCloud; // Replace with new cloud
                }

                // Send new angle to Arduino, needs to be blocking
                this->publishAngle();

                while (!captFlag){
                    ;
                }
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
                RCLCPP_INFO(this->get_logger(), "Angle: %d", angle);
                
                boost::asio::write(serial_, boost::asio::buffer(&angle, sizeof(angle)));
                RCLCPP_INFO(this->get_logger(), "Arduino messaged");

                // // Wait for reply from Arduino
                // boost::asio::streambuf buf;
                // std::string line;
                // while (true)
                // {
                //     boost::asio::read_until(serial_, buf, "\n"); // Message recieved
                //     std::istream is(&buf);
                //     std::getline(is, line);

                //     // Trim leading/trailing whitespace
                //     line.erase(line.begin(), std::find_if(line.begin(), line.end(), [](int ch){ return !std::isspace(ch); }));
                //     line.erase(std::find_if(line.rbegin(), line.rend(), [](int ch){ return !std::isspace(ch); }).base(), line.end());

                //     RCLCPP_INFO(this->get_logger(), "Monitor: %s", line.c_str());

                //     if (line == "CMD: 1") 
                //     {
                //         // Toogle boolean message because finished moving
                //         RCLCPP_INFO(this->get_logger(), "Arduino finished moving");
                //         captFlag = true;
                
                //     }
                // }     

                // Delay init so arduino can init
                rclcpp::Rate rate(1); // 1 Hz = 1 second per cycl
                rate.sleep(); // pauses for 0.5 second
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
        
        // Timer
        rclcpp::TimerBase::SharedPtr timer_;

        // Serial
        boost::asio::io_service io_;
        boost::asio::serial_port serial_;

        // Constant
        int angle;
        pcl::PointCloud<pcl::PointXYZ>::Ptr memoryCloud;
        int dirFlag;
        bool captFlag = true;

        // Multithread
        std::mutex pcl_mutex_;
    };

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlBlender>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    
    executor.spin();
    executor.remove_node(node);
    node.reset();  // destroy shared_ptr
    rclcpp::shutdown();
    return 0;
}