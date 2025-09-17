# Created by: Edric Lay
# Date Created: 18/09/25
import rclpy
from rclpy.node import Node
import open3d

class MergePointCloud(Node):
    def __init__(self):
        super().__init__("point_cloud_merge_node")

def main(args = None):
    rclpy.init(args=args)
    node = MergePointCloud()
    rclpy.spin(node) # This node will be kept alive independently until is killed.
    rclpy.shutdown() 