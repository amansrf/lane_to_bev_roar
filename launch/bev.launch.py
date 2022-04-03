from launch import LaunchDescription
import launch_ros
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lane_to_bev_roar',
            namespace='lane_mask_publisher',
            executable='lane_mask_publisher',
            name='lane_mask_publisher'
        ),
        launch_ros.actions.ComposableNodeContainer(
                name="container",
                namespace="",
                package="rclcpp_components",
                executable="component_container",
                composable_node_descriptions=[
                    # Driver itself
                    launch_ros.descriptions.ComposableNode(
                        package="depth_image_proc",
                        plugin="depth_image_proc::PointCloudXyzrgbNode",
                        name="point_cloud_xyzrgb_node",
                        remappings=[
                            ("rgb/camera_info", "/depth_streamer/camera_info"),
                            ("rgb/image_rect_color", "/lane_mask_publisher/masked_image"),
                            (
                                "depth_registered/image_rect",
                                "/depth_streamer/depth_image",
                            ),
                            ("points", "/pointcloud"),
                        ],
                    ),
                ],
                output="screen",
            ),
    ]
)