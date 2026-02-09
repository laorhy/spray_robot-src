from setuptools import find_packages, setup

package_name = 'spray_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/worlds', ['worlds/spray_lidar_world.sdf', 'worlds/spray_lidar_world_simple.sdf']),
        ('share/' + package_name + '/meshes', ['meshes/ship_hull.STL']),
        ('share/' + package_name + '/rviz', ['rviz/spray.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lao',
    maintainer_email='lao@todo.todo',
    description='Spray robot simulation package',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
)