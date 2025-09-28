import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Header, Int32
from sensor_msgs.msg import Image, JointState

from message_filters import ApproximateTimeSynchronizer, Subscriber

import cv2
from cv_bridge import CvBridge

import numpy as np

class ImageStitcherNode(Node):
    def __init__(self):
        super().__init__("image_stitcher")

        # Parameters
        self.declare_parameter('max_distance', 0.8)

        # Subscribers
        # self.rgb_sub = self.create_subscription(Image, "camera/camera/aligned_depth_to_color/image_raw", self.stitchCallback, 10) # Reads the depth aligned images
        # self.depth_sub = self.create_subscription(Image, "camera/camera/depth/image_rect_raw", self.stitchCallback, 10) # Reads the depth images

        self.rgb_sub = Subscriber(Image, "camera/camera/aligned_depth_to_color/image_raw") # Reads the depth aligned images
        self.depth_sub = Subscriber(Image, "camera/camera/depth/image_rect_raw") # Reads the depth images

        self.ats = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], 10, 0.1)
        self.ats.registerCallback(self.stitchCallback)

        # Publishers
        self.stitch_pub = self.create_publisher(Image, "stitched_Image", 10) # Publishes the final stitched image
        self.update_pub = self.create_publisher(JointState, "update_PCL", 10) # Communicates to the ICP_blender on when and where to update the point cloud
        
        # Data
        self.stitchedImage = None
        self.maxDist = self.get_parameter('max_distance').get_parameter_value().double_value

        # Other
        self.br = CvBridge()
        self.stitcher = cv2.Stitcher.create()
    
    def stitchCallback(self, RGBD_msg: Image, depth_msg: Image):
        self.get_logger().info('Receiving video frame')

        status = False # Used to prevent failed stitches from being published
        

        # Extract to cv2
        rgbFrame = self.br.imgmsg_to_cv2(RGBD_msg)
        depthFrame = self.br.imgmsg_to_cv2(depth_msg)

        filteredFrame = self.filterImage(rgbFrame, depthFrame)
        
        # Initial frame
        if not self.stitchedImage: # check if image is empty
            status = True
            self.stitchedImage = filteredFrame

        # Other frame
        else:
            imgs = [self.stitchedImage, filteredFrame]
            
            notReplace = True # Two conditions: need to replace or not?

            # Doesn't need to replace image + pcl
            if notReplace:
                pass

            # Does need to replace image + PCL
            else:
                status, stitchedImage = self.stitcher.stitch(imgs)
                if status:
                    self.stitchedImage = stitchedImage
                    
                    # Tell PCL to update point cloud
                    update_msg = JointState()
                    update_msg.name = ["angle"]
                    # update_msg.position = [angle] # Need to tell where to update PCL
                    update_msg.effort = [1] # Used to tell ICP blender to update PCL
                    self.update_pub.publish(update_msg)
                else:
                    self.get_logger().info('Issue with image stitching')

        # Always publish stitched image
        self.stitch_pub.publish(self.stitchedImage)

    def filterImage(self, RGB_image, depth_image):
        # Filter using numpy
        depthRGB = depth_image[:, :, None]
        filtered = np.where(depthRGB>self.maxDist, RGB_image, [0,0,0])

        return filtered