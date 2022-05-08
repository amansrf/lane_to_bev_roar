import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge
import numpy as np
from sensor_msgs.msg import Image

import time

color_mask_minima = {
    "red_l": np.array([  0, 100,  200]),
    "red_u": np.array([160, 100,  200]),
    "white": np.array([   0,  0, 210]),
    "blue" : np.array([100,150,0]),
    "yellow" : np.array([ 50, 50, 170])
}
color_mask_maxima = {
    "red_l": np.array([ 10, 255, 255]),
    "red_u": np.array([179, 255, 255]),
    "white": np.array([ 255, 25, 255]),
    "blue" : np.array([ 40,255,255]),
    "yellow" : np.array([ 255, 255, 190])
}

LANE_COLOR = "white"
CHKPT_COLOR = "red"

class MaskPublisher(Node):

    def __init__(self):
        super().__init__('lane_mask_publisher')
        self.image_sub = self.create_subscription(Image, '/rgb_streamer/rgb_image', self.image_callback, 1)
        self.masked_img_pub = self.create_publisher(Image, 'masked_image', 1)
        self.bridge = CvBridge()
        self.denoising_kernel = np.ones((3,3), dtype=np.uint8)
        # Use a horizontal kernel to accentuate horizontal waypoint lines
        self.horizontal_kernel = np.array(
            [
                [0, 0, 0, 0],
                [1, 1, 1, 1],
                [1, 1, 1, 1],
                [0, 0, 0, 0]
            ],
            dtype = np.uint8
        )

        # Use a vertical kernel to accentuate vertical lane lines
        self.vertical_kernel = np.array(
            [
                [0, 1, 1, 0],
                [0, 1, 1, 0],
                [0, 1, 1, 0]
                [0, 1, 1, 0]
            ],
            dtype = np.uint8
        )

    
    def color_mask(self, img, color):
        '''Since cv2 has two ranges for red in the HSV Color Space,
            we define two different masks and combine them.'''
        if(color=='red'):
            lower_red_minima = color_mask_minima['red_l']
            lower_red_maxima = color_mask_maxima['red_l']
            upper_red_minima = color_mask_minima['red_u']
            upper_red_maxima = color_mask_maxima['red_u']
            # color_list = [lower_red_minima,lower_red_maxima,upper_red_minima,upper_red_maxima]
            lower_mask = cv2.inRange(img, lower_red_minima, lower_red_maxima)
            upper_mask = cv2.inRange(img, upper_red_minima, upper_red_maxima)
            mask = cv2.bitwise_or(lower_mask, upper_mask)

        else:
            minima = color_mask_minima[color]
            maxima = color_mask_maxima[color]
            # color_list = [minima,maxima]
            mask = cv2.inRange(img, minima, maxima)
        return mask

    def image_callback(self, msg):
        
        # Converting ROS image message to CV2 BGR8
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

        # Converting BGR to HSV
        converted_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        
        
        # --------------------------------- Checkpoint mask --------------------------------- #
        checkpoint_mask_original = self.color_mask(img = converted_image, color = CHKPT_COLOR)
        checkpoint_mask = checkpoint_mask_original
        # checkpoint_mask = cv2.medianBlur(checkpoint_mask_original, 1)
        # cv2.imshow("Checkpoint Mask", checkpoint_mask_original)
        # cv2.waitKey(1)

        # First Erode and then Dilate using the horizontal_kernel to remove noise
        checkpoint_mask = cv2.morphologyEx(checkpoint_mask, cv2.MORPH_OPEN, self.horizontal_kernel)

        # Now dilate and then erode the image to join any internal noise or line breakages
        checkpoint_mask = cv2.morphologyEx(checkpoint_mask, cv2.MORPH_CLOSE, np.ones((5,5)))
        

        # -------------------------------- Lane Mask -------------------------------- #
        # Crop out top third of image for mask
        converted_image[:converted_image.shape[0]//3,:] = 0
        lane_mask_original = self.color_mask(img = converted_image, color = LANE_COLOR)
        lane_mask = cv2.medianBlur(lane_mask_original, 5)
        
        cv2.imshow("Lane Mask", lane_mask)
        # cv2.waitKey(1)

        # First Erode and then Dilate using the vertical_kernel to remove noise
        lane_mask = cv2.morphologyEx(lane_mask, cv2.MORPH_OPEN, self.vertical_kernel)
        # cv2.imshow("lane_mask_without_noise", lane_mask)

        # Now dilate and then erode the image to join any internal noise or line breakages
        lane_mask = cv2.morphologyEx(lane_mask, cv2.MORPH_CLOSE, np.ones((5,5)))
        # cv2.imshow("lane_mask_closed", lane_mask)
        # cv2.waitKey(1)

        # Halving the mask value from 255 because pointcloud overflows in some unexplainable way otherwise
        checkpoint_mask = checkpoint_mask/2
        lane_mask = lane_mask/2

        # Stack the masks in bgr format.
        mask_stack = np.array(
            np.stack((lane_mask, checkpoint_mask, np.zeros(checkpoint_mask.shape)), axis=-1),
            dtype=np.uint8,
        )
        cv2.imshow("combined_mask", (mask_stack*2)%255)
        cv2.waitKey(1)


        # Convert to ros_msg
        masked_image = self.bridge.cv2_to_imgmsg(mask_stack, encoding='bgr8')
        masked_image.header = msg.header

        # Publish Masked Image
        self.masked_img_pub.publish(masked_image)

    


def main(args=None):
    rclpy.init(args=args)

    mask_publisher = MaskPublisher()

    try:
        rclpy.spin(mask_publisher)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    mask_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
