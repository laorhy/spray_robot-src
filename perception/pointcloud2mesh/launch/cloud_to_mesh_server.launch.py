from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud2mesh',
            executable='cloud_to_mesh_server',
            name='cloud_to_mesh_server',
            output='screen',
            parameters=[{
                'default_output_dir': '/tmp/pointcloud_mesh',
                'default_filename_prefix': 'roi_mesh',
                'default_save_ply': True,
                'default_save_stl': False,
                'use_sim_time': False,
            }]
        )
    ])
