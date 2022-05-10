import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy, QoSLivelinessPolicy

import cv2
from cv_bridge import CvBridge
import numpy as np
import math
import time

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

        self.bev_img_pub = self.create_publisher(Image, 'bev_image', 1)
        self.bridge = CvBridge()
        self.bev_shape = (84,84,3)
        self.denoising_kernel = np.ones((25,25), dtype=np.uint8)
        self.bev_image = Image()

    def pcl_callback(self, msg):

        # Initialize empty bev_image
        bev_image = np.zeros(self.bev_shape)
        # self.get_logger().info(f"BEV Shape is:{bev_image.shape}")

        # Convert pcl2 message to numpy array
        pcl_arr = pointcloud2_to_array(msg, squeeze=False)
        # self.get_logger().info(f"after pcl2_to_arr:{pcl_arr.shape}")

        # Split the 32 bit rgb field into individual r,g,b values
        pcl_arr = split_rgb_field(pcl_arr)
        # self.get_logger().info(f"after split_rgb_field:{pcl_arr.shape}")

        # Convert pcl_arr to 6xN array
        pcl_arr = get_xyzrgb_points(pcl_arr)
        # self.get_logger().info(f"after get_xyzrgb_points:{pcl_arr.shape}")

        # Filter locations of masked points only
        # lane_coods = pcl_arr[np.argwhere(pcl_arr[:,3]>100)[:,0]]
        lane_coods = pcl_arr[np.where( (pcl_arr[:,3]>100) | (pcl_arr[:,5]>100))[0]]
        # self.get_logger().info(f"Lane_Coods after mask filter:{lane_coods.shape}")

        # self.get_logger().info(f"Lane_Coods after x filter:{np.where( (lane_coods[:,0] > MIN_X) & (lane_coods[:,0] < MAX_X))[0].shape}")
        # Filter points by x range
        lane_coods = lane_coods[
            np.where(
                (lane_coods[:,0] > MIN_X) & (lane_coods[:,0] < MAX_X)
            )[0],
            :
        ]
        # self.get_logger().info(f"Lane_Coods after x filter:{lane_coods.shape}")

        # Filter points by z range
        lane_coods = lane_coods[
            np.where(
                (lane_coods[:,2] > MIN_Z) & (lane_coods[:,2] < MAX_Z)
            )[0],
            :
        ]
        # self.get_logger().info(f"Lane_Coods after z filter:{lane_coods.shape}")


        # Extract x and z coods of filtered points
        # lane_coods_x = lane_coods[:,0]
        lane_coods_x = lane_coods[np.where(lane_coods[:,3]>100)[0], 0]
        # self.get_logger().info(f"Max x:{np.max(lane_coods_x)}, Min x:{np.min(lane_coods_x)}")

        # lane_coods_z = lane_coods[:,2]
        lane_coods_z = lane_coods[np.where(lane_coods[:,3]>100)[0], 2]
        # self.get_logger().info(f"Max z:{np.max(lane_coods_z)}, Min z:{np.min(lane_coods_z)}")

        lane_coods_uv = np.zeros( (lane_coods_x.shape[0], 2) )

        lane_coods_uv[:,0] = np.floor( self.bev_shape[1] * (lane_coods_x[:] - MIN_X)/(MAX_X-MIN_X + 1e-10))
        # self.get_logger().info(f"Max u:{np.max(lane_coods_uv[:,0])}, Min u:{np.min(lane_coods_uv[:,0])}")

        lane_coods_uv[:,1] = np.floor( self.bev_shape[0] * (lane_coods_z[:] - MIN_Z)/(MAX_Z-MIN_Z + 1e-10))
        # self.get_logger().info(f"Max v:{np.max(lane_coods_uv[:,1])}, Min v:{np.min(lane_coods_uv[:,1])}")


        lane_coods_uv = np.array(lane_coods_uv, dtype=np.uint)
        # self.get_logger().info(f"{( (self.bev_shape[0]-lane_coods_uv[:,1]-1)%255 , lane_coods_uv[:,0]%255 )}")
        bev_image[ (self.bev_shape[0]-lane_coods_uv[:,1]-1) , lane_coods_uv[:,0], 2] = 1

        # Extract x and z coods of filtered points
        # lane_coods_x = lane_coods[:,0]
        lane_coods_x = lane_coods[np.where(lane_coods[:,5]>100)[0], 0]
        # self.get_logger().info(f"Max x:{np.max(lane_coods_x)}, Min x:{np.min(lane_coods_x)}")

        # lane_coods_z = lane_coods[:,2]
        lane_coods_z = lane_coods[np.where(lane_coods[:,5]>100)[0], 2]
        # self.get_logger().info(f"Max z:{np.max(lane_coods_z)}, Min z:{np.min(lane_coods_z)}")

        lane_coods_uv = np.zeros( (lane_coods_x.shape[0], 2) )

        lane_coods_uv[:,0] = np.floor( self.bev_shape[1] * (lane_coods_x[:] - MIN_X)/(MAX_X-MIN_X + 1e-10))
        # self.get_logger().info(f"Max u:{np.max(lane_coods_uv[:,0])}, Min u:{np.min(lane_coods_uv[:,0])}")

        lane_coods_uv[:,1] = np.floor( self.bev_shape[0] * (lane_coods_z[:] - MIN_Z)/(MAX_Z-MIN_Z + 1e-10))
        # self.get_logger().info(f"Max v:{np.max(lane_coods_uv[:,1])}, Min v:{np.min(lane_coods_uv[:,1])}")


        lane_coods_uv = np.array(lane_coods_uv, dtype=np.uint)
        # self.get_logger().info(f"{( (self.bev_shape[0]-lane_coods_uv[:,1]-1)%255 , lane_coods_uv[:,0]%255 )}")
        bev_image[ (self.bev_shape[0]-lane_coods_uv[:,1]-1) , lane_coods_uv[:,0], 0] = 1

        # Show bev
        cv2.imshow("bev", bev_image)
        cv2.waitKey(1)

        # Remove noisy detections from mask

        # vertical_kernel = np.array(
        #     [
        #         [0, 0, 1, 1, 1, 0, 0],
        #         [0, 0, 1, 1, 1, 0, 0],
        #         [0, 0, 1, 1, 1, 0, 0],
        #         [0, 0, 1, 1, 1, 0, 0],
        #         [0, 0, 1, 1, 1, 0, 0],
        #         [0, 0, 1, 1, 1, 0, 0],
        #         [0, 0, 1, 1, 1, 0, 0],
        #     ],
        #     dtype = np.uint8
        # )
        vertical_kernel = np.array(
            [
                [0, 1, 0],
                [0, 1, 0],
                [0, 1, 0],
            ],
            dtype = np.uint8
        )
        horizontal_kernel = np.array(
            [
                [0, 0, 0],
                [1, 1, 1],
                [0, 0, 0],
            ],
            dtype = np.uint8
        )

        denoised_image = cv2.morphologyEx(bev_image, cv2.MORPH_CLOSE, vertical_kernel)
        # denoised_image = cv2.dilate(bev_image, vertical_kernel, iterations=2)
        # denoised_image = cv2.erode(denoised_image, horizontal_kernel, iterations=1)
        denoised_image = cv2.morphologyEx(denoised_image, cv2.MORPH_CLOSE, horizontal_kernel)

        # Show bev
        cv2.imshow("bev_denoised", denoised_image)
        cv2.waitKey(1)

        assert denoised_image.shape == self.bev_shape, "Denoised Image not of correct shape"

        denoised_image = np.array(denoised_image, dtype=np.uint8)
        self.bev_image = self.bridge.cv2_to_imgmsg(denoised_image, encoding='bgr8')
        self.bev_image.header.frame_id = 'base_link'
        self.bev_image.header.stamp = msg.header.stamp
        self.get_logger().info(f"Time gap:{self.bev_image.header.stamp.sec, self.bev_image.header.stamp.nanosec}")
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
    points = np.zeros(cloud_array.shape + (6,), dtype=dtype)
    points[...,0] = cloud_array['x']
    points[...,1] = cloud_array['y']
    points[...,2] = cloud_array['z']
    points[...,3] = cloud_array['r']
    points[...,4] = cloud_array['g']
    points[...,5] = cloud_array['b']
    
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
