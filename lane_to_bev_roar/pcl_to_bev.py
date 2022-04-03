import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy, QoSLivelinessPolicy

import cv2
from cv_bridge import CvBridge
import numpy as np
import math

from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
import sensor_msgs_py.point_cloud2 as pc2
# from pcl_msgs.msg import PCLPointCloud2
# import ros2_numpy

import ctypes
import struct



class BEVPublisher(Node):

    def __init__(self):
        super().__init__('bev_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            liveliness=QoSLivelinessPolicy.RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
            depth=1,
        )
        self.pointcloud_sub = self.create_subscription(PointCloud2, '/pointcloud', self.pcl_callback, qos_profile)
        print("haha1")
        self.bev_img_pub = self.create_publisher(Image, 'bev_image', 1)
        self.bridge = CvBridge()
        self.bev_shape = (256,256)
        self.denoising_kernel = np.ones((25,25), dtype=np.uint8)
        self.bev_image = Image()

    def pcl_callback(self, msg):
        bev_image = np.zeros(self.bev_shape)

        cloud_list_rgb = pc2.read_points_list(msg, skip_nans=True)
        cl_list_copy = cloud_list_rgb.copy()

        max_x, max_z = -np.float('inf'), -np.float('inf')
        min_x, min_z = np.float('inf'), np.float('inf')
        for _ in cl_list_copy:
            x,y,z,rgb = _.x, _.y, _.z, _.rgb

            s = struct.pack('>f', rgb)
            i = struct.unpack('>l', s)[0]

            pack = ctypes.c_uint32(i).value
            r = (pack & 0x00FF0000)>> 16
            g = (pack & 0x0000FF00)>> 8
            b = (pack & 0x000000FF)

            # if (r+g+b>300):
            if max_x < x:
                max_x = x
            if max_z < z:
                max_z = z
            if min_x > x:
                min_x = x
            if min_z > z:
                min_z = z

        for _ in cl_list_copy:
            x,y,z,rgb = _.x, _.y, _.z, _.rgb

            s = struct.pack('>f', rgb)
            i = struct.unpack('>l', s)[0]

            pack = ctypes.c_uint32(i).value
            r = (pack & 0x00FF0000)>> 16
            g = (pack & 0x0000FF00)>> 8
            b = (pack & 0x000000FF)

            if (r+g+b>300):
                u = math.floor(self.bev_shape[1]*(x-min_x)/(max_x-min_x + 1e-10))
                v = math.floor(self.bev_shape[0]*(z-min_z)/(max_z-min_z + 1e-10))
                print(u,v)
                bev_image[self.bev_shape[0]-int(v)-1,int(u)] = 255
        cv2.imshow("bev", bev_image)
        cv2.waitKey(1)

        # Remove noisy detections from mask
        denoised_image = cv2.morphologyEx(bev_image, cv2.MORPH_CLOSE, self.denoising_kernel)

        cv2.imshow("bev_noiseless", denoised_image)
        cv2.waitKey(1)

        denoised_image = np.array(
            np.stack((denoised_image, denoised_image, denoised_image), axis=-1),
            dtype=np.uint8,
        )

        self.bev_image = self.bridge.cv2_to_imgmsg(denoised_image, encoding='bgr8')
        self.bev_image.header.frame_id = 'base_link'
        self.bev_image.header.stamp = self.get_clock().now().to_msg()
        self.bev_img_pub.publish(self.bev_image)




        
            


    


def main(args=None):
    rclpy.init(args=args)

    bev_publisher = BEVPublisher()

    rclpy.spin(bev_publisher)

    # Destroy the node explicitly
    bev_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
