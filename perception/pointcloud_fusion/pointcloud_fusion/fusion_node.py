#!/usr/bin/env python3

from itertools import chain

import rclpy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException
from tf2_ros.transform_listener import TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


class PointCloudFusionNode(Node):
    def __init__(self) -> None:
        super().__init__("pointcloud_fusion_node")

        self.declare_parameter("top_topic", "/lidar/top/points")
        self.declare_parameter("bottom_topic", "/lidar/bottom/points")
        self.declare_parameter("target_frame", "spray_base/base_link")
        self.declare_parameter("fused_topic", "/lidar/fused/points")
        self.declare_parameter("roi_topic", "/lidar/fused/roi")
        self.declare_parameter("roi.min_x", float('-inf'))
        self.declare_parameter("roi.max_x", float('inf'))
        self.declare_parameter("roi.min_y", -3.0)
        self.declare_parameter("roi.max_y", 3.0)
        self.declare_parameter("roi.min_z", 0.0)
        self.declare_parameter("roi.max_z", 6.0)

        top_topic = self.get_parameter("top_topic").get_parameter_value().string_value
        bottom_topic = (
            self.get_parameter("bottom_topic").get_parameter_value().string_value
        )
        self.target_frame = (
            self.get_parameter("target_frame").get_parameter_value().string_value
        )
        fused_topic = self.get_parameter("fused_topic").get_parameter_value().string_value
        roi_topic = self.get_parameter("roi_topic").get_parameter_value().string_value

        self.roi_limits = {
            "min_x": self.get_parameter("roi.min_x").value,
            "max_x": self.get_parameter("roi.max_x").value,
            "min_y": self.get_parameter("roi.min_y").value,
            "max_y": self.get_parameter("roi.max_y").value,
            "min_z": self.get_parameter("roi.min_z").value,
            "max_z": self.get_parameter("roi.max_z").value,
        }

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.fused_pub = self.create_publisher(
            PointCloud2, fused_topic, qos_profile_sensor_data
        )
        self.roi_pub = self.create_publisher(
            PointCloud2, roi_topic, qos_profile_sensor_data
        )

        top_sub = Subscriber(self, PointCloud2, top_topic, qos_profile=qos_profile_sensor_data)
        bottom_sub = Subscriber(self, PointCloud2, bottom_topic, qos_profile=qos_profile_sensor_data)

        self.sync = ApproximateTimeSynchronizer(
            [top_sub, bottom_sub], queue_size=10, slop=0.1, allow_headerless=False
        )
        self.sync.registerCallback(self._sync_callback)

        self.get_logger().info(
            "PointCloud fusion node started. "
            f"Target frame: {self.target_frame}, "
            f"Top topic: {top_topic}, Bottom topic: {bottom_topic}"
        )

    def _sync_callback(self, top_msg: PointCloud2, bottom_msg: PointCloud2) -> None:
        try:
            timeout = rclpy.duration.Duration(seconds=0.2)
            top_stamp = rclpy.time.Time.from_msg(top_msg.header.stamp)
            bottom_stamp = rclpy.time.Time.from_msg(bottom_msg.header.stamp)

            top_transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                top_msg.header.frame_id,
                top_stamp,
                timeout=timeout,
            )
            bottom_transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                bottom_msg.header.frame_id,
                bottom_stamp,
                timeout=timeout,
            )
        except TransformException as exc:
            self.get_logger().warn(f"TF transform failed: {exc}")
            return

        # 3. 仅保留 xyz 字段，避免点云字段/填充导致的 tf2 断言错误
        top_xyz_points = list(
            point_cloud2.read_points(
                top_msg, field_names=("x", "y", "z"), skip_nans=True
            )
        )
        bottom_xyz_points = list(
            point_cloud2.read_points(
                bottom_msg, field_names=("x", "y", "z"), skip_nans=True
            )
        )
        top_xyz_cloud = point_cloud2.create_cloud_xyz32(top_msg.header, top_xyz_points)
        bottom_xyz_cloud = point_cloud2.create_cloud_xyz32(
            bottom_msg.header, bottom_xyz_points
        )

        transformed_top_msg = do_transform_cloud(top_xyz_cloud, top_transform)
        transformed_bottom_msg = do_transform_cloud(
            bottom_xyz_cloud, bottom_transform
        )

        # 4. 读取转换后的点云数据
        top_points = list(
            point_cloud2.read_points(
                transformed_top_msg, field_names=("x", "y", "z"), skip_nans=True
            )
        )
        bottom_points = list(
            point_cloud2.read_points(
                transformed_bottom_msg, field_names=("x", "y", "z"), skip_nans=True
            )
        )

        # 5. 合并点云数据
        fused_points = list(chain(top_points, bottom_points))

        # 6. 创建新的点云消息
        header = top_msg.header
        header.frame_id = self.target_frame
        header.stamp = self.get_clock().now().to_msg()
        fused_msg = point_cloud2.create_cloud_xyz32(header, fused_points)
        self.fused_pub.publish(fused_msg)

        # 7. 感兴趣区域过滤
        roi_points = self._filter_roi(fused_points)
        roi_msg = point_cloud2.create_cloud_xyz32(header, roi_points)
        self.roi_pub.publish(roi_msg)

    def _filter_roi(self, points):
        limits = self.roi_limits
        min_x = limits["min_x"]
        max_x = limits["max_x"]
        min_y = limits["min_y"]
        max_y = limits["max_y"]
        min_z = limits["min_z"]
        max_z = limits["max_z"]

        return [
            (x, y, z)
            for x, y, z in points
            if min_x <= x <= max_x
            and min_y <= y <= max_y
            and min_z <= z <= max_z
        ]


def main() -> None:
    rclpy.init()
    node = PointCloudFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
