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
}
class MaskPublisher(Node):

    def __init__(self):
        super().__init__('lane_mask_publisher')
        self.image_sub = self.create_subscription(Image, '/rgb_streamer/rgb_image', self.image_callback, 1)
        self.masked_img_pub = self.create_publisher(Image, 'masked_image', 1)
        self.bridge = CvBridge()
        self.denoising_kernel = np.ones((3,3), dtype=np.uint8)

    def image_callback(self, msg):

        # Converting ROS image message to CV2 BGR8
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

        # Converting BGR to HSV
        converted_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        '''Since cv2 has two ranges for red in the HSV Color Space,
            we define two different masks and combine them.'''
        
        # --------------------------------- Red Mask --------------------------------- #
        # Creating mask for the lower end of HSV red
        lower_red_minima = np.array([  0, 100,  200])
        lower_red_maxima = np.array([ 10, 255, 255])
        lower_mask = cv2.inRange(converted_image, lower_red_minima, lower_red_maxima)

        # Creating mask for the lower end of HSV red
        upper_red_minima = np.array([160, 100,  200])
        upper_red_maxima = np.array([179, 255, 255])
        upper_mask = cv2.inRange(converted_image, upper_red_minima, upper_red_maxima)

        # Combining the red masks
        red_mask = cv2.bitwise_or(lower_mask, upper_mask)
        # cv2.imshow("Red Mask", red_mask)
        # cv2.waitKey(1)

        # Use a horizontal kernel to accentuate horizontal waypoint lines
        horizontal_kernel = np.array(
            [
                [0, 0, 0],
                [1, 1, 1],
                [0, 0, 0]
            ],
            dtype = np.uint8
        )

        vertical_kernel = np.array(
            [
                [0, 1, 0],
                [0, 1, 0],
                [0, 1, 0]
            ],
            dtype = np.uint8
        )

        # First Erode and then Dilate using the horizontal_kernel to remove noise
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, horizontal_kernel)

        # Now dilate and then erode the image to join any internal noise or line breakages
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, np.ones((5,5)))
        

        # -------------------------------- White Mask -------------------------------- #
        # Creating mask for HSV white
        white_minima = np.array([   0,  0, 210])
        white_maxima = np.array([ 255, 25, 255])

        # Crop out top third of image for mask
        converted_image[:converted_image.shape[0]//3,:] = 0

        # Finish white mask
        white_mask = cv2.inRange(converted_image, white_minima, white_maxima)
        # cv2.imshow("white_mask", white_mask)
        # cv2.waitKey(1)

        # Use a horizontal kernel to accentuate horizontal waypoint lines
        # horizontal_kernel = np.array(
        #     [
        #         [0, 0, 0],
        #         [1, 1, 1],
        #         [0, 0, 0]
        #     ],
        #     dtype = np.uint8
        # )

        # First Erode and then Dilate using the horizontal_kernel to remove noise
        # white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, vertical_kernel)
        # cv2.imshow("white_mask_without_noise", white_mask)

        # Now dilate and then erode the image to join any internal noise or line breakages
        # white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, np.ones((5,5)))
        # cv2.imshow("white_mask_closed", white_mask)
        # cv2.waitKey(1)

        # Halving the mask value from 255 because pointcloud overflows in some unexplainable way otherwise
        red_mask = red_mask/2
        white_mask = white_mask/2

        # Stack the masks in bgr format.
        mask_stack = np.array(
            np.stack((white_mask, np.zeros(red_mask.shape), red_mask), axis=-1),
            dtype=np.uint8,
        )
        cv2.imshow("combined_mask", (mask_stack*2)%255)
        cv2.waitKey(1)


        # Convert to ros_msg
        masked_image = self.bridge.cv2_to_imgmsg(mask_stack, encoding='bgr8')
        masked_image.header = msg.header

        # Publish Masked Image
        self.masked_img_pub.publish(masked_image)

    def color_mask(self, color):


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
