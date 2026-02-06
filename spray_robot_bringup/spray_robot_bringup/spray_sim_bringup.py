#!/usr/bin/env python3

import os
import sys

from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    world_default = os.path.expanduser("~/spray_sim/worlds/spray_lidar_world.sdf")
    rviz_default = os.path.expanduser("~/ros2_ws/spray_robot-ws/rviz/spray.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value=world_default),
            DeclareLaunchArgument("rviz_config", default_value=rviz_default),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("fixed_frame", default_value="world"),
            DeclareLaunchArgument("base_frame", default_value="spray_base/base_link"),
            DeclareLaunchArgument("top_frame", default_value="spray_base/base_link/lidar_top"),
            DeclareLaunchArgument("bottom_frame", default_value="spray_base/base_link/lidar_bottom"),
            DeclareLaunchArgument("base_x", default_value="-40"),
            DeclareLaunchArgument("base_y", default_value="0"),
            DeclareLaunchArgument("base_z", default_value="10"),
            DeclareLaunchArgument("base_roll", default_value="0"),
            DeclareLaunchArgument("base_pitch", default_value="0"),
            DeclareLaunchArgument("base_yaw", default_value="0"),
            DeclareLaunchArgument("top_x", default_value="0"),
            DeclareLaunchArgument("top_y", default_value="0"),
            DeclareLaunchArgument("top_z", default_value="6"),
            DeclareLaunchArgument("top_roll", default_value="0"),
            DeclareLaunchArgument("top_pitch", default_value="0"),
            DeclareLaunchArgument("top_yaw", default_value="0"),
            DeclareLaunchArgument("bottom_x", default_value="0"),
            DeclareLaunchArgument("bottom_y", default_value="0"),
            DeclareLaunchArgument("bottom_z", default_value="0"),
            DeclareLaunchArgument("bottom_roll", default_value="0"),
            DeclareLaunchArgument("bottom_pitch", default_value="0"),
            DeclareLaunchArgument("bottom_yaw", default_value="0"),
            ExecuteProcess(
                cmd=["ign", "gazebo", "-r", LaunchConfiguration("world")],
                output="screen",
            ),
            Node(
                package="ros_ign_bridge",
                executable="parameter_bridge",
                arguments=["/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock"],
                output="screen",
            ),
            Node(
                package="ros_ign_bridge",
                executable="parameter_bridge",
                arguments=[
                    "/lidar/top/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked"
                ],
                output="screen",
            ),
            Node(
                package="ros_ign_bridge",
                executable="parameter_bridge",
                arguments=[
                    "/lidar/bottom/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked"
                ],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--x",
                    LaunchConfiguration("base_x"),
                    "--y",
                    LaunchConfiguration("base_y"),
                    "--z",
                    LaunchConfiguration("base_z"),
                    "--roll",
                    LaunchConfiguration("base_roll"),
                    "--pitch",
                    LaunchConfiguration("base_pitch"),
                    "--yaw",
                    LaunchConfiguration("base_yaw"),
                    "--frame-id",
                    LaunchConfiguration("fixed_frame"),
                    "--child-frame-id",
                    LaunchConfiguration("base_frame"),
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--x",
                    LaunchConfiguration("top_x"),
                    "--y",
                    LaunchConfiguration("top_y"),
                    "--z",
                    LaunchConfiguration("top_z"),
                    "--roll",
                    LaunchConfiguration("top_roll"),
                    "--pitch",
                    LaunchConfiguration("top_pitch"),
                    "--yaw",
                    LaunchConfiguration("top_yaw"),
                    "--frame-id",
                    LaunchConfiguration("base_frame"),
                    "--child-frame-id",
                    LaunchConfiguration("top_frame"),
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--x",
                    LaunchConfiguration("bottom_x"),
                    "--y",
                    LaunchConfiguration("bottom_y"),
                    "--z",
                    LaunchConfiguration("bottom_z"),
                    "--roll",
                    LaunchConfiguration("bottom_roll"),
                    "--pitch",
                    LaunchConfiguration("bottom_pitch"),
                    "--yaw",
                    LaunchConfiguration("bottom_yaw"),
                    "--frame-id",
                    LaunchConfiguration("base_frame"),
                    "--child-frame-id",
                    LaunchConfiguration("bottom_frame"),
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", LaunchConfiguration("rviz_config")],
                parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
                output="screen",
            ),
        ]
    )


def main() -> int:
    launch_service = LaunchService(argv=sys.argv[1:])
    launch_service.include_launch_description(generate_launch_description())
    return launch_service.run()


if __name__ == "__main__":
    raise SystemExit(main())
