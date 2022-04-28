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
from .point_cloud2 import pointcloud2_to_array, split_rgb_field

import ctypes
import struct

MAX_Z = 3.65 #metres
MIN_Z = 0.2 #metres
MAX_X = (MAX_Z - MIN_Z)/2 #metres
MIN_X = -MAX_X #metres



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

        pcl_arr = pointcloud2_to_array(msg, squeeze=False)
        pcl_arr = split_rgb_field(pcl_arr)
        pcl_arr = get_xyzrgb_points(pcl_arr)

        lane_coods = pcl_arr[np.argwhere(pcl_arr[...,3]>100)[:,0]]
        lane_coods_xz = lane_coods[:,(0,2)]
        
        self.get_logger().info("# ------------------------------------ END ----------------------------------- #")
        # self.get_logger().info(f"{(lane_coods).shape}")
        # self.get_logger().info(f"{(lane_coods_xz).shape}")

        # lane_coods_xz = np.argwhere((lane_coods_xz[...,1]<MAX_Z))

        # self.get_logger().info(f"{(pcl_arr).shape}")
        # self.get_logger().info(f"{(lane_coods_xz).shape}")

        lane_coods_uv = np.zeros(lane_coods_xz.shape)
        lane_coods_uv[:,0] = np.floor( self.bev_shape[1] * (lane_coods_xz[:,0] - MIN_X)/(MAX_X-MIN_X + 1e-10))
        lane_coods_uv[:,1] = np.floor( self.bev_shape[0] * (lane_coods_xz[:,1] - MIN_Z)/(MAX_Z-MIN_Z + 1e-10))

        self.get_logger().info(f"{lane_coods_uv[:5]}")
        self.get_logger().info(f"{lane_coods_xz[:5]}")
        lane_coods_uv = np.array(lane_coods_uv, dtype=np.uint)
        bev_image[ self.bev_shape[0]-lane_coods_uv[:,1]-1 , lane_coods_uv[:,0] ] = 1


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



def get_xyzrgb_points(cloud_array, remove_nans=True, dtype=np.float):
    '''Pulls out x, y, and z columns from the cloud recordarray, and returns
    a 3xN matrix.
    '''
    # remove crap points
    if remove_nans:
        mask = np.isfinite(cloud_array['x']) & \
               np.isfinite(cloud_array['y']) & \
               np.isfinite(cloud_array['z'])
        cloud_array = cloud_array[mask]

    # pull out x, y, and z values
    points = np.zeros(cloud_array.shape + (4,), dtype=dtype)
    points[...,0] = cloud_array['x']
    points[...,1] = cloud_array['y']
    points[...,2] = cloud_array['z']
    points[...,3] = cloud_array['r']
    # points[...,4] = cloud_array['g']
    # points[...,5] = cloud_array['b']
    
    return points

        
            


    


def main(args=None):
    rclpy.init(args=args)

    bev_publisher = BEVPublisher()

    rclpy.spin(bev_publisher)

    # Destroy the node explicitly
    bev_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
