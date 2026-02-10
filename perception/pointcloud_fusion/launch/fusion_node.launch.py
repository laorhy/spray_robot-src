from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_fusion',
            executable='pointcloud_fusion_node',
            name='pointcloud_fusion_node',
            output='screen',
            parameters=[{"use_sim_time": True}],
        ),
    ])