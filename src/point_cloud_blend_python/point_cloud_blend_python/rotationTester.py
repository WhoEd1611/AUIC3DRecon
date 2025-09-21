import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Header, Int32
import serial

class Rotation(Node):
    def __init__(self):
        super().__init__("rotation_node")

        self.angle_pub_ = self.create_publisher(Int32, "angle", 10)
        self.get_logger().info("Node initialised")
        self.timer_ = self.create_timer(3, self.sendAngle)
        self.angle = 0
        self.dirFlag = 1
    
    def sendAngle(self):
        if self.angle >= 180:
            self.dirFlag = -1

        elif self.angle <= 0:
            self.dirFlag = 1
                
        self.angle = self.angle + self.dirFlag * 30

        msg = Int32()
        msg.data = self.angle

        self.get_logger().info(f"Angle Sent {self.angle}")

        self.angle_pub_.publish(msg)


def main(args = None):
    rclpy.init(args=args)
    node = Rotation()
    rclpy.spin(node)
    rclpy.shutdown()