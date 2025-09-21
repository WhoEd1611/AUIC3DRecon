import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Header, Int32
from sensor_msgs.msg import PointCloud2

from message_filters import Subscriber, ApproximateTimeSynchronizer

import numpy as np
import open3d as o3d

class PointCloudBlender(Node):
    def __init__(self):
        super().__init__("pc_blender")
        
        # Init pub
        self.ang_pub_ = self.create_publisher(Int32, "angle", 10)
        self.pc_pub_ = self.create_publisher(PointCloud2, "merge_pc", 10)

        # Init sub
        self.bool_sub_ = Subscriber(self, "motion_bool", Header)
        self.pc_sub_ = Subscriber(self, "camera/camera/depth/color/points", PointCloud2)

        self.ts = ApproximateTimeSynchronizer(
            [self.bool_sub_, self.pc_sub_],
            queue_size=10,
            slop=0.1
            )
        
        self.ts.registerCallback(self.confirmAngle)
    
    def confirmAngle(self, bool_msg: Header, pc_msg: PointCloud2):
        if bool_msg.frame_id == "1":
            # Get point cloud and reformat
            cloud = pc_msg.data
            cloud = np.array(cloud)
            cloud = cloud.reshape