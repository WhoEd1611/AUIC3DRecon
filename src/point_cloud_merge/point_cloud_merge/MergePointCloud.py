# Created by: Edric Lay
# Date Created: 18/09/25
import rclpy
from rclpy.node import Node
from message_filters import Subscriber, ApproximateTimeSynchronizer
import open3d
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32, Bool, Header

class MergePointCloud(Node):
    def __init__(self):
        super().__init__("point_cloud_merge_node")

        #### Publisher
        self.pc_pub = self.create_publisher(PointCloud2,"merged_Point_Cloud",1) # Will publish full point_cloud for rviz3 to watch

        ### Subscribers
        self.point_sub = Subscriber(self, PointCloud2, "/camera/camera/depth/color/points") # Gets point cloud from current view
        self.angle_sub = Subscriber(self, Float32, "published_Angle") # Gets current viewing angle
        self.updateFlag_sub = Subscriber(self, Bool, "updateFlag") # Gets flag to update at current angle or not. Does not matter what value this is.

        ### Will only update point_cloud when to
        self.ats = ApproximateTimeSynchronizer([self.point_sub, self.angle_sub, self.updateFlag_sub], queue_size=5, slop=0.1)
        self.ats.registerCallback(self.synchronised_callback)

        ### Current point cloud
        self.point_cloud = None

    def synchronised_callback(self, pc_msg, angle_msg, bool_msg):
        # If no current point_cloud exists, init here
        if self.point_cloud == None:
            self.point_cloud = pc_msg.data

        # Populate new PointCloud message
        new_pc_msg = PointCloud2()
        new_pc_msg.header = Header()
        new_pc_msg.header.frame_id = "frame"

        self.pc_pub.publish(new_pc_msg)



def main(args = None):
    rclpy.init(args=args)
    node = MergePointCloud()
    rclpy.spin(node) # This node will be kept alive independently until is killed.
    rclpy.shutdown() 