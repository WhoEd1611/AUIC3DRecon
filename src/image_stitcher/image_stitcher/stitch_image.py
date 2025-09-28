import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Header, Int32
from sensor_msgs.msg import Image

class ImageStitcherNode(Node):
    def __init__(self):
        super().__init__("image_stitcher")

        # Subscribers
        self.rgb_sub = self.create_subscription(Image, "camera/camera/aligned_depth_to_color/image_raw", self.stitchCallback) # Reads the depth aligned images

        # Publishers
        self.stitch_pub = self.create_publisher(Image, "stitched_Image") # Publishes the final stitched image

        # Data
        self.stitched_Image = None
    
    def stitchCallback(self, RGBD_msg):
        self.stitch_pub.publish(self.stitched_Image)