import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Header, Int32
import serial

class ArduinoSerialNode(Node):
    def __init__(self):
        super().__init__("arduino_serial_node")

        self.serial_ = serial.Serial("/dev/ttyACM0", 9600)

        self.stat_pub_ = self.create_publisher(Header, "motion_bool", 10)
        self.ang_sub_ = self.create_subscription(Int32, "angle", self.sendToArduino, 10)
        self.get_logger().info("Node initialised")
    
    def sendToArduino(self, ang_msg: Int32):
        self.get_logger().info("Angle Recieved")

        move_flag = False # Makes it so that other node doesn't same data until this is finished
        
        # Send to Arduino angle
        if self.serial_.is_open:
            cmd = str(ang_msg.data)+'\n'
            self.serial_.write(cmd.encode('utf-8'))
            self.get_logger().info(f"Angle to go to: {cmd}")

            # Wait for reply from Arduino
            while move_flag == False:
                line = self.serial_.readline() # Message recieved
                line = line.decode("utf-8").strip()
                self.get_logger().info(f"Serial message: {line}")
                if line == "CMD: 1":
                    move_flag = True

            # Publish message because finished moving
            pub_msg = Header()
            pub_msg.stamp = self.get_clock().now().to_msg()
            pub_msg.frame_id = "1" # Hold Bool
            self.stat_pub_.publish(pub_msg)
            self.get_logger().info(f"Arduino finished moving to: {cmd}")


def main(args = None):
    rclpy.init(args=args)
    node = ArduinoSerialNode()
    rclpy.spin(node)
    rclpy.shutdown()