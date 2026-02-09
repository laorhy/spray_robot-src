#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
升级版：两路雷达点云融合（喷涂场景友好）
功能：
1) TF 到统一 target_frame
2) 仅保留 xyz（避开 tf2 对字段/填充的坑）
3) 体素( VoxelGrid )融合：重叠区域不“加厚”，点数下降，网格更干净
4)（可选）离群点剔除：统计离群 SOR（优先）/ 半径离群 ROR（降级）
5) ROI 过滤
6) 发布 fused 与 roi

依赖：
- 必需：numpy
- 可选：scipy（用于 KDTree，加速 SOR/ROR；没有也能跑，只是离群点剔除会自动降级/关闭）

运行：
ros2 run <your_pkg> pointcloud_fusion_node.py
参数示例：
ros2 run <your_pkg> pointcloud_fusion_node.py --ros-args \
  -p voxel.size:=0.03 \
  -p outlier.enable:=true -p outlier.method:=sor -p outlier.mean_k:=24 -p outlier.std_mul:=1.0 \
  -p roi.min_x:=-2.0 -p roi.max_x:=6.0 -p roi.min_y:=-3.0 -p roi.max_y:=3.0 -p roi.min_z:=0.0 -p roi.max_z:=6.0
"""

from itertools import chain
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy

from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from tf2_ros import Buffer, TransformException
from tf2_ros.transform_listener import TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


# ----------------------------
# KDTree（scipy 可用则启用，否则降级）
# ----------------------------
try:
    from scipy.spatial import cKDTree as KDTree  # type: ignore
    _HAS_KDTREE = True
except Exception:
    KDTree = None
    _HAS_KDTREE = False


def _pc2_to_xyz_np(msg: PointCloud2, skip_nans=True) -> np.ndarray:
    """PointCloud2 -> Nx3 float32 numpy"""
    pts = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=skip_nans)
    arr = np.fromiter(chain.from_iterable(pts), dtype=np.float32)
    if arr.size == 0:
        return np.empty((0, 3), dtype=np.float32)
    return arr.reshape((-1, 3))


def _xyz_np_to_pc2(header, xyz: np.ndarray) -> PointCloud2:
    """Nx3 numpy -> PointCloud2 (xyz32)"""
    if xyz.size == 0:
        return point_cloud2.create_cloud_xyz32(header, [])
    # create_cloud_xyz32 接受 iterable[tuple]
    return point_cloud2.create_cloud_xyz32(header, xyz.astype(np.float32).tolist())


def voxel_downsample_centroid(xyz: np.ndarray, voxel: float) -> np.ndarray:
    """
    体素融合（质心）：把落在同一体素内的点聚成一个点（质心），避免重叠区域“加厚”
    """
    if xyz.shape[0] == 0 or voxel <= 0.0:
        return xyz

    # 体素索引
    idx = np.floor(xyz / voxel).astype(np.int32)

    # 用结构化数组做 unique（比 python dict 更快一些）
    key = idx.view(dtype=np.dtype((np.void, idx.dtype.itemsize * idx.shape[1])))
    uniq_key, inv = np.unique(key, return_inverse=True)

    # 对每个体素求质心：sum / count
    counts = np.bincount(inv)
    sums_x = np.bincount(inv, weights=xyz[:, 0])
    sums_y = np.bincount(inv, weights=xyz[:, 1])
    sums_z = np.bincount(inv, weights=xyz[:, 2])

    centroids = np.vstack((sums_x / counts, sums_y / counts, sums_z / counts)).T.astype(np.float32)
    return centroids


def filter_roi_np(xyz: np.ndarray, limits: dict) -> np.ndarray:
    if xyz.shape[0] == 0:
        return xyz
    min_x, max_x = limits["min_x"], limits["max_x"]
    min_y, max_y = limits["min_y"], limits["max_y"]
    min_z, max_z = limits["min_z"], limits["max_z"]

    m = (
        np.isfinite(xyz[:, 0]) & np.isfinite(xyz[:, 1]) & np.isfinite(xyz[:, 2]) &
        (xyz[:, 0] >= min_x) & (xyz[:, 0] <= max_x) &
        (xyz[:, 1] >= min_y) & (xyz[:, 1] <= max_y) &
        (xyz[:, 2] >= min_z) & (xyz[:, 2] <= max_z)
    )
    return xyz[m]


def remove_outliers_sor(xyz: np.ndarray, mean_k: int, std_mul: float) -> np.ndarray:
    """
    统计离群（SOR）：对每个点，计算 kNN 平均距离；大于 mean + std_mul*std 的剔除
    需要 KDTree（scipy）
    """
    if xyz.shape[0] == 0:
        return xyz
    if not _HAS_KDTREE:
        return xyz  # 自动降级：没 KDTree 就不做
    k = max(4, int(mean_k))
    tree = KDTree(xyz)
    # query 返回 (dist, idx)，dist shape (N, k+1)（包含自身 0 距离）
    dists, _ = tree.query(xyz, k=min(k + 1, xyz.shape[0]))
    if dists.ndim == 1:
        return xyz
    # 去掉自身
    d = dists[:, 1:]
    mean_dist = np.mean(d, axis=1)
    mu = float(np.mean(mean_dist))
    sigma = float(np.std(mean_dist) + 1e-12)
    thresh = mu + float(std_mul) * sigma
    keep = mean_dist <= thresh
    return xyz[keep]


def remove_outliers_ror(xyz: np.ndarray, radius: float, min_neighbors: int) -> np.ndarray:
    """
    半径离群（ROR）：半径内邻居数 < min_neighbors 的点剔除
    需要 KDTree（scipy）
    """
    if xyz.shape[0] == 0:
        return xyz
    if not _HAS_KDTREE:
        return xyz
    r = float(radius)
    mn = max(1, int(min_neighbors))
    tree = KDTree(xyz)
    # query_ball_point 返回每个点邻居索引列表
    neighbors = tree.query_ball_point(xyz, r=r)
    counts = np.fromiter((len(n) - 1 for n in neighbors), dtype=np.int32)  # -1 排除自身
    keep = counts >= mn
    return xyz[keep]


class PointCloudFusionNode(Node):
    def __init__(self) -> None:
        super().__init__("pointcloud_fusion_node_upgraded")

        # ---- topics / frames
        self.declare_parameter("top_topic", "/lidar/top/points")
        self.declare_parameter("bottom_topic", "/lidar/bottom/points")
        self.declare_parameter("target_frame", "spray_base/base_link")
        self.declare_parameter("fused_topic", "/lidar/fused/points")
        self.declare_parameter("roi_topic", "/lidar/fused/roi")

        # ---- voxel fusion
        self.declare_parameter("voxel.enable", True)
        self.declare_parameter("voxel.size", 0.03)  # 3cm：船体大面一般够用；需要更细可 0.01~0.02

        # ---- outlier removal
        self.declare_parameter("outlier.enable", True)
        self.declare_parameter("outlier.method", "sor")  # "sor" or "ror" or "none"
        self.declare_parameter("outlier.mean_k", 24)
        self.declare_parameter("outlier.std_mul", 1.0)
        self.declare_parameter("outlier.radius", 0.08)
        self.declare_parameter("outlier.min_neighbors", 6)

        # ---- ROI
        self.declare_parameter("roi.min_x", float("-inf"))
        self.declare_parameter("roi.max_x", float("inf"))
        self.declare_parameter("roi.min_y", -3.0)
        self.declare_parameter("roi.max_y", 3.0)
        self.declare_parameter("roi.min_z", 0.0)
        self.declare_parameter("roi.max_z", 6.0)

        top_topic = self.get_parameter("top_topic").value
        bottom_topic = self.get_parameter("bottom_topic").value
        self.target_frame = self.get_parameter("target_frame").value
        fused_topic = self.get_parameter("fused_topic").value
        roi_topic = self.get_parameter("roi_topic").value

        self.roi_limits = {
            "min_x": float(self.get_parameter("roi.min_x").value),
            "max_x": float(self.get_parameter("roi.max_x").value),
            "min_y": float(self.get_parameter("roi.min_y").value),
            "max_y": float(self.get_parameter("roi.max_y").value),
            "min_z": float(self.get_parameter("roi.min_z").value),
            "max_z": float(self.get_parameter("roi.max_z").value),
        }

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # QoS
        self.pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.fused_pub = self.create_publisher(PointCloud2, fused_topic, self.pub_qos)
        self.roi_pub = self.create_publisher(PointCloud2, roi_topic, self.pub_qos)

        top_sub = Subscriber(self, PointCloud2, top_topic, qos_profile=qos_profile_sensor_data)
        bottom_sub = Subscriber(self, PointCloud2, bottom_topic, qos_profile=qos_profile_sensor_data)

        self.sync = ApproximateTimeSynchronizer(
            [top_sub, bottom_sub],
            queue_size=10,
            slop=0.10,
            allow_headerless=False,
        )
        self.sync.registerCallback(self._sync_callback)

        if self.get_parameter("outlier.enable").value and not _HAS_KDTREE:
            self.get_logger().warn(
                "scipy 不可用：离群点剔除（SOR/ROR）将自动降级为关闭。"
            )

        self.get_logger().info(
            f"Upgraded fusion node started. target_frame={self.target_frame}, "
            f"top={top_topic}, bottom={bottom_topic}"
        )

    def _lookup(self, target: str, source: str, stamp) :
        timeout = rclpy.duration.Duration(seconds=0.2)
        return self.tf_buffer.lookup_transform(target, source, stamp, timeout=timeout)

    def _sync_callback(self, top_msg: PointCloud2, bottom_msg: PointCloud2) -> None:
        # 1) TF 查询
        try:
            top_stamp = rclpy.time.Time.from_msg(top_msg.header.stamp)
            bottom_stamp = rclpy.time.Time.from_msg(bottom_msg.header.stamp)
            top_tf = self._lookup(self.target_frame, top_msg.header.frame_id, top_stamp)
            bottom_tf = self._lookup(self.target_frame, bottom_msg.header.frame_id, bottom_stamp)
        except TransformException as exc:
            self.get_logger().warn(f"TF transform failed: {exc}")
            return

        # 2) 仅保留 xyz 字段，避免 tf2 对字段断言问题
        top_xyz = _pc2_to_xyz_np(top_msg, skip_nans=True)
        bottom_xyz = _pc2_to_xyz_np(bottom_msg, skip_nans=True)

        # 空保护
        if top_xyz.shape[0] == 0 and bottom_xyz.shape[0] == 0:
            return

        # 3) 变换到 target_frame（PointCloud2 -> transform -> 再读回 numpy）
        #    这里仍用 create_cloud_xyz32 + do_transform_cloud，最稳
        top_xyz_cloud = _xyz_np_to_pc2(top_msg.header, top_xyz)
        bottom_xyz_cloud = _xyz_np_to_pc2(bottom_msg.header, bottom_xyz)

        top_tf_msg = do_transform_cloud(top_xyz_cloud, top_tf)
        bottom_tf_msg = do_transform_cloud(bottom_xyz_cloud, bottom_tf)

        top_tf_xyz = _pc2_to_xyz_np(top_tf_msg, skip_nans=True)
        bottom_tf_xyz = _pc2_to_xyz_np(bottom_tf_msg, skip_nans=True)

        # 4) 拼接（只是中间态；后面会做体素融合/去噪）
        fused_xyz = (
            top_tf_xyz if bottom_tf_xyz.shape[0] == 0
            else (bottom_tf_xyz if top_tf_xyz.shape[0] == 0 else np.vstack((top_tf_xyz, bottom_tf_xyz)))
        )

        # 5) 体素融合（关键：避免重叠区域“加厚”）
        if bool(self.get_parameter("voxel.enable").value):
            voxel = float(self.get_parameter("voxel.size").value)
            fused_xyz = voxel_downsample_centroid(fused_xyz, voxel)

        # 6) 离群点剔除（可选）
        if bool(self.get_parameter("outlier.enable").value) and _HAS_KDTREE:
            method = str(self.get_parameter("outlier.method").value).lower()
            if method == "sor":
                mean_k = int(self.get_parameter("outlier.mean_k").value)
                std_mul = float(self.get_parameter("outlier.std_mul").value)
                fused_xyz = remove_outliers_sor(fused_xyz, mean_k=mean_k, std_mul=std_mul)
            elif method == "ror":
                radius = float(self.get_parameter("outlier.radius").value)
                min_n = int(self.get_parameter("outlier.min_neighbors").value)
                fused_xyz = remove_outliers_ror(fused_xyz, radius=radius, min_neighbors=min_n)
            # "none" 则跳过

        # 7) 发布 fused
        header = top_msg.header
        header.frame_id = self.target_frame
        header.stamp = self.get_clock().now().to_msg()  # 实时可视化/下游消费更直观
        fused_msg = _xyz_np_to_pc2(header, fused_xyz)
        self.fused_pub.publish(fused_msg)

        # 8) ROI 过滤并发布
        roi_xyz = filter_roi_np(fused_xyz, self.roi_limits)
        roi_msg = _xyz_np_to_pc2(header, roi_xyz)
        self.roi_pub.publish(roi_msg)


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
