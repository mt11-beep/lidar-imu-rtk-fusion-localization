from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value="~/workspace/project/config/example.yaml",
                description="Public demo parameter file. Replace with your ROS2 package path.",
            ),
            Node(
                package="spark_fast_lio",
                executable="spark_lio_mapping",
                name="lidar_imu_rtk_fusion",
                output="screen",
                remappings=[
                    ("/lidar", "/points_raw"),
                    ("/imu", "/imu/data"),
                    ("/fix", "/fix"),
                ],
                parameters=[config_file],
            ),
        ]
    )
