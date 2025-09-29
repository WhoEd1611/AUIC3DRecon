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
#include <pcl/registration/transformation_estimation_2D.h>

#include <Eigen/Dense>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

// Hash function for std::pair<int,int>
struct pair_hash {
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2> &p) const {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);
        return h1 ^ (h2 << 1);  // combine the two hashes
    }
};

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
            // timer_ = this->create_wall_timer(1s, std::bind(&PointCloudBlender::publishCloud, this));

            // Init direction, angle and point cloud
            dirFlag = 1;
            angle = 0;
            captFlag = false;
            memoryCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
            initFlag = true;
            offsetAng = 0;
            dAng = 30;
            prevAng = 0;
            // if (!t_matrix.allFinite()) {
            //     RCLCPP_ERROR(this->get_logger(), "ICP transformation contains NaN/Inf!");
            // } 
            // else 
            // {
            //     RCLCPP_INFO(this->get_logger(), "ICP transformation matrix:");
            //     RCLCPP_INFO(this->get_logger(), 
            //                 "[%f, %f, %f, %f]\n"
            //                 "[%f, %f, %f, %f]\n"
            //                 "[%f, %f, %f, %f]\n"
            //                 "[%f, %f, %f, %f]",
            //                 t_matrix(0,0), t_matrix(0,1), t_matrix(0,2), t_matrix(0,3),
            //                 t_matrix(1,0), t_matrix(1,1), t_matrix(1,2), t_matrix(1,3),
            //                 t_matrix(2,0), t_matrix(2,1), t_matrix(2,2), t_matrix(2,3),
            //                 t_matrix(3,0), t_matrix(3,1), t_matrix(3,2), t_matrix(3,3));
            // }

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

                // Clean pcl
                std::vector<int> indices;  // Not used
                pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
                // pcl::transformPointCloud(*cloud, *cloud, t_matrix);
                
                // Voxel Filter cloud
                pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
                voxel.setInputCloud(cloud);
                voxel.setLeafSize(0.001f, 0.001f, 0.001f); // 1cm voxels
                voxel.filter(*cloud);

                // Radius Outlier Removal
                pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> ror;
                ror.setInputCloud(cloud);
                ror.setRadiusSearch(0.001);   // 1 mm radius
                ror.setMinNeighborsInRadius(1);  // keep if at least one neighbor
                ror.filter(*cloud);

                if (initFlag){
                    // Default as first cloud
                    // pcl::transformPointCloud(*cloud, *cloud, t_matrix);
                    *memoryCloud = *cloud;
                    initFlag = false;
                    RCLCPP_INFO(this->get_logger(), "PCL Initialised");
                }

                else
                {
                    // Transform new point cloud
                    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_aligned = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
                    icp.setInputSource(cloud);
                    icp.setInputTarget(memoryCloud);

                    // // ICP parameters
                    // icp.setMaxCorrespondenceDistance(1);
                    // icp.setMaximumIterations(50);
                    // icp.setTransformationEpsilon(1e-8);
                    // icp.setEuclideanFitnessEpsilon(1e-6);

                    // icp.align(*cloud_aligned, t_matrix);
                    icp.align(*cloud_aligned);


                    if (icp.hasConverged()) {
                        RCLCPP_INFO(this->get_logger(), "ICP converged!");
                        RCLCPP_INFO(this->get_logger(), "Fitness score: %f", icp.getFitnessScore());
                    } else {
                        RCLCPP_WARN(this->get_logger(), "ICP did NOT converge.");                    
                    }

                    // Add new point cloud to old point cloud
                    *memoryCloud = *memoryCloud + *cloud_aligned;
                    // *memoryCloud = *memoryCloud + *cloud;


                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterCloud(new pcl::PointCloud<pcl::PointXYZRGB>); // temp filtered cloud

                    // Voxel Filter cloud
                    pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
                    voxel.setInputCloud(memoryCloud);
                    voxel.setLeafSize(0.001f, 0.001f, 0.001f); // 1cm voxels
                    voxel.filter(*filterCloud);

                    // Radius Outlier Removal
                    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> ror;
                    ror.setInputCloud(filterCloud);
                    ror.setRadiusSearch(0.001);   // 1 mm radius
                    ror.setMinNeighborsInRadius(1);  // keep if at least one neighbor
                    ror.filter(*filterCloud);


                    // // Custom filter method to keep 2.5D
                    // float grid_size = 0.001f; // 1 mm grid
                    // std::unordered_map<std::pair<int,int>, pcl::PointXYZRGB, pair_hash> grid;

                    // for (const auto &p : cloud_aligned->points) {
                    //     int gx = static_cast<int>(p.x / grid_size);
                    //     int gy = static_cast<int>(p.y / grid_size);
                    //     std::pair<int,int> key = {gx, gy};
                    //     if (grid.find(key) == grid.end()) {
                    //         grid[key] = p; // keep first point in this cell
                    //     }
                    // }
                    
                    // for (auto &kv : grid) filterCloud->points.push_back(kv.second); // Convert back to point cloud
    
                    float threshold = 0.8f;
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_thresh(new pcl::PointCloud<pcl::PointXYZRGB>);


                    for (auto& p : *filterCloud) {
                        float r = std::sqrt(p.x * p.x + p.y * p.y); // distance from origin in XY plane
                        if (r <= threshold) {                       // keep points inside threshold
                            cloud_thresh->points.emplace_back(p);
                        }
                    }

                    cloud_thresh->width = cloud_thresh->points.size();
                    cloud_thresh->height = 1;
                    cloud_thresh->is_dense = true;

                    *memoryCloud = *cloud_thresh; // Replace with new cloud
                    pcl::removeNaNFromPointCloud(*memoryCloud, *memoryCloud, indices);
                    RCLCPP_INFO(this->get_logger(), "Cloud updated");
                }

                this->publishCloud();

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
        }
    
        void publishAngle()
        {
            t_matrix = this->calcTransform(offsetAng, angle);
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
        Eigen::Matrix4f t_matrix;

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