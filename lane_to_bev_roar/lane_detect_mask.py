import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge
import numpy as np
from sensor_msgs.msg import Image


class MaskPublisher(Node):

    def __init__(self):
        super().__init__('lane_mask_publisher')
        self.image_sub = self.create_subscription(Image, '/rgb_streamer/rgb_image', self.image_callback, 1)
        self.masked_img_pub = self.create_publisher(Image, 'masked_image', 1)
        self.bridge = CvBridge()
        self.denoising_kernel = np.ones((3,3), dtype=np.uint8)

    def image_callback(self, msg):

        # Converting ROS image message to RGB
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

        # Converting BGR to HSV
        converted_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        '''Since cv2 has two ranges for red in the HSV Color Space,
            we define two different masks and combine them.'''
        
        # Creating mask for the lower end of HSV red
        lower_hue_minima = np.array([  0, 100,  200])
        lower_hue_maxima = np.array([ 10, 255, 255])
        lower_mask = cv2.inRange(converted_image, lower_hue_minima, lower_hue_maxima)

        # Creating mask for the lower end of HSV red
        upper_hue_minima = np.array([160, 100,  200])
        upper_hue_maxima = np.array([179, 255, 255])
        upper_mask = cv2.inRange(converted_image, upper_hue_minima, upper_hue_maxima)

        # Combining the masks
        mask = cv2.bitwise_or(lower_mask, upper_mask)
        cv2.imshow("mask", mask)
        cv2.waitKey(1)

        # # Remove noisy detections from mask
        # denoised_mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.denoising_kernel)
        # cv2.imshow("denoised_mask", denoised_mask)
        # cv2.waitKey(1)

        # Halving the mask value from 255 because pointcloud overflows in some unexplainable way otherwise
        mask = mask/2

        # Stack the black and white image thrice like in RGB format.
        mask_stack = np.array(
            np.stack((mask, mask, mask), axis=-1),
            dtype=np.uint8,
        )

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
