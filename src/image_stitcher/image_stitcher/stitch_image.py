import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Header, Int32
from sensor_msgs.msg import Image, JointState, CameraInfo

from message_filters import ApproximateTimeSynchronizer, Subscriber

import cv2
from cv_bridge import CvBridge

import numpy as np

class ImageStitcherNode(Node):
    def __init__(self):
        super().__init__("image_stitcher")

        # Parameters
        self.declare_parameter('max_distance', 0.8)
        self.declare_parameter('sweep_range', 180)

        # Subscribers
        # self.rgb_sub = self.create_subscription(Image, "camera/camera/aligned_depth_to_color/image_raw", self.stitchCallback, 10) # Reads the depth aligned images
        # self.depth_sub = self.create_subscription(Image, "camera/camera/depth/image_rect_raw", self.stitchCallback, 10) # Reads the depth images

        self.rgb_sub = Subscriber(Image, "camera/camera/aligned_depth_to_color/image_raw") # Reads the depth aligned images
        self.depth_sub = Subscriber(Image, "camera/camera/depth/image_rect_raw") # Reads the depth images
        self.angle_sub = Subscriber(Int32, "angle") # Reads the current image angle
        self.cam_sub = self.create_subscription(CameraInfo, "/camera/camera/aligned_depth_to_color/camera_info", self.setStitchedImageSize) # Reads the camera intrinsics

        self.ats = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub, self.angle_sub], 10, 0.1)
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
    
    def setStitchedImageSize(self, cam_msg: CameraInfo):
        # Determine how much camera sweeps
        self.sweep = self.get_parameter("sweep_range").get_parameter_value().integer_value

        # Determine hfov
        self.img_width = cam_msg.width
        self.img_height = cam_msg.height
        self.img_intrinsic = cam_msg.k
        self.img_fx = self.img_intrinsic[0]
        self.img_hfov = 2 * np.rad2deg(np.arctan(self.img_width/(2*self.img_fx)))

        # Check if full hfov >= 360
        self.full_hfov = self.sweep + self.hfov

        if self.full_hfov >= 360:        
            # Determine stitch image size
            pan_width = self.img_width * (360/self.img_hfov)
            self.stitchedImage = np.zeros((pan_width, self.img_height, 3))
        
        else:
            # Determine stitch image size
            pan_width = self.img_width * (1 + self.sweep/self.hfov)
            self.stitchedImage = np.zeros((pan_width, self.img_height, 3))
        
        del self.cam_sub
    
    def stitchCallback(self, RGBD_msg: Image, depth_msg: Image, angle_msg: Int32):
        self.get_logger().info('Receiving video frame')

        status = False # Used to prevent failed stitches from being published

        # Extract to cv2
        rgbFrame = self.br.imgmsg_to_cv2(RGBD_msg)
        depthFrame = self.br.imgmsg_to_cv2(depth_msg)

        # Get angle data
        angle = angle_msg.data

        filteredFrame = self.filterImage(rgbFrame, depthFrame) # Filters image to remove far objects
        imgs = [self.stitchedImage, filteredFrame]
        
        # Checks to see if anything has changed in the image to see if need to update pcl
        notReplace = self.compareFrame(filteredFrame, angle) # Compares the captured image with the current frame

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
    
    def compareFrame(self, image_frame, angle):
        if self.full_hfov < 360:
            # Step 1: Extract from self.stitchedImage equivalent image
            width = self.stitchedImage.shape[0]
            centre = (width-self.img_width) * angle/self.sweep
            old_image = self.stitchedImage[centre-self.img_width/2:centre+self.img_width/2+1,:,:]
        
        else:
            pass

        # Step 2: Compare two images
        can_old = cv2.Canny(old_image, 100, 200)
        can_new = cv2.Canny(image_frame, 100, 200)

        diff = cv2.subtract(can_new, can_old)
        pix_diff_count = cv2.countNonZero(diff)
        self.get_logger().info(f"Difference in pixel count: {pix_diff_count}")

        if pix_diff_count > 0.05 * self.img_width * self.img_height: # Pass some minimum threshold of difference
            return True
        else:
            return False
        
def main(args = None):
    rclpy.init(args=args)
    node = ImageStitcherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()