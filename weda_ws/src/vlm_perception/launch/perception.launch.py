"""Launch the full VLM perception pipeline.

    ros2 launch vlm_perception perception.launch.py
    ros2 launch vlm_perception perception.launch.py config:=/path/to/vlm_app.yaml

The vlm-depth node acts as the standalone camera driver using pyorbbecsdk2 natively.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_CONFIG = "/opt/weda-release/config/vlm_app.yaml"


def generate_launch_description():
    config = LaunchConfiguration("config")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=DEFAULT_CONFIG),
        # Static TF with the standard optical rotation (z-forward camera ->
        # x-forward camera_link) so the 3D pointcloud appears upright in rviz.
        Node(package="tf2_ros", executable="static_transform_publisher",
             name="tf_camera_color", output="log",
             arguments=["--roll", "-1.5707963", "--yaw", "-1.5707963",
                        "--frame-id", "camera_link",
                        "--child-frame-id", "camera_color_optical_frame"]),
        Node(package="tf2_ros", executable="static_transform_publisher",
             name="tf_camera_depth", output="log",
             arguments=["--roll", "-1.5707963", "--yaw", "-1.5707963",
                        "--frame-id", "camera_link",
                        "--child-frame-id", "camera_depth_optical_frame"]),
        Node(package="vlm_perception", executable="vlm-depth",
             name="vlm_depth_processor", output="screen",
             arguments=["--config", config]),
        Node(package="vlm_perception", executable="vlm-detector",
             name="vlm_detector", output="screen",
             arguments=["--config", config]),
        Node(package="vlm_perception", executable="vlm-perception",
             name="vlm_perception", output="screen",
             arguments=["--config", config]),
    ])
