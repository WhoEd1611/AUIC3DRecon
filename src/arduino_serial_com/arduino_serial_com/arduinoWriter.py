import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Header, Int32
import serial

class ArduinoSerialNode(Node):
    def __init__(self):
        super.__init__("arduino_serial_node")

        self.ser = serial.Serial("/dev/ttyACM0", 9600)

        self.stat_pub_ = self.create_publisher(Header, "motion_bool", 10)
        self.ang_sub_ = self.create_subscription(Int32, "angle", self.sendToArduino, 10)
    
    def sendToArduino(self, ang_msg):
        