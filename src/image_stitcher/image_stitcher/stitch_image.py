import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Header, Int32
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge

class ImageStitcherNode(Node):
    def __init__(self):
        super().__init__("image_stitcher")

        # Subscribers
        self.rgb_sub = self.create_subscription(Image, "camera/camera/aligned_depth_to_color/image_raw", self.stitchCallback, 10) # Reads the depth aligned images

        # Publishers
        self.stitch_pub = self.create_publisher(Image, "stitched_Image", 10) # Publishes the final stitched image
        
        # Data
        self.stitched_Image = None

        # Other
        self.br = CvBridge()
    
    def stitchCallback(self, RGBD_msg: Image):
        self.get_logger().info('Receiving video frame')
        
        # Initial frame
        if not self.stitched_Image: # check if image is empty
            pass

        # Other frame
        else:
            pass

        self.stitch_pub.publish(self.stitched_Image)
