from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="v4l2_camera",
                executable="v4l2_camera_node",
                name="camera_publisher",
                parameters=[{"video_device":"/dev/video0","image_size": [480,480]}],
            ),
            Node(
                package="motion_recognition",
                executable="motion_recognition",
                name="motion_recognition",
            ),
        ]
    )
