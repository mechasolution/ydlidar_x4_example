import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace, LifecycleNode


def generate_launch_description():
    # LiDAR 드라이버
    ydlidar_x4_param = LaunchConfiguration(
        "ydlidar_x4_param",
        default=os.path.join(
            get_package_share_directory("ydlidar_x4_example"),
            "param",
            "ydlidar_x4_param.yaml",
        ),
    )
    lidar_driver_node = LifecycleNode(
        package="ydlidar_ros2_driver",
        executable="ydlidar_ros2_driver_node",
        name="ydlidar_ros2_driver_node",
        output="screen",
        emulate_tty=True,
        parameters=[ydlidar_x4_param],
    )
    # LiDAR TF2
    lidar_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_pub_laser",
        arguments=["0", "0", "0.02", "0", "0", "0", "1", "base_link", "laser_frame"],
    )

    return LaunchDescription([lidar_driver_node, lidar_tf_node])
