#!/usr/bin/env bash
set -e

# ========================
# ROS2 环境（按你的目录结构）
# ========================
source /opt/ros/humble/setup.bash

if [ -f "$HOME/ros2_ws/spray_robot-ws/install/setup.bash" ]; then
  source "$HOME/ros2_ws/spray_robot-ws/install/setup.bash"
fi
if [ -f "$HOME/ros2_ws/ws_livox/install/setup.bash" ]; then
  source "$HOME/ros2_ws/ws_livox/install/setup.bash"
fi

# ========================
# Gazebo world
# ========================
WORLD_SDF="$HOME/spray_sim/worlds/spray_lidar_world.sdf"

# ========================
# TF frame 命名（你已确认 PointCloud2 frame_id）
# ========================
FIXED_FRAME="world"
BASE_FRAME="spray_base/base_link"
TOP_FRAME="$BASE_FRAME/lidar_top"
BOT_FRAME="$BASE_FRAME/lidar_bottom"

# ========================
# 位姿（务必改成你 SDF 里的真实值，单位：m, rad）
# ========================
# world -> base_link
BASE_X=-40; BASE_Y=0; BASE_Z=10; BASE_ROLL=0; BASE_PITCH=0; BASE_YAW=0

# base_link -> lidar_top
TOP_X=0; TOP_Y=0; TOP_Z=6; TOP_ROLL=0; TOP_PITCH=0; TOP_YAW=0

# base_link -> lidar_bottom
BOT_X=0; BOT_Y=0; BOT_Z=0; BOT_ROLL=0; BOT_PITCH=0; BOT_YAW=0

# ========================
# 进程管理
# ========================
PIDS=()

start_bg () {
  echo "[start] $*"
  "$@" &
  PIDS+=($!)
}

cleanup () {
  echo ""
  echo "[stop] killing processes: ${PIDS[*]}"
  for pid in "${PIDS[@]}"; do
    kill "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

# ========================
# 0) 启动 Gazebo
# ========================
if [ ! -f "$WORLD_SDF" ]; then
  echo "❌ world sdf not found: $WORLD_SDF"
  exit 1
fi

start_bg ign gazebo -r "$WORLD_SDF"

# 给 Gazebo 起起来一点时间（否则桥接可能先起来但还没 topic）
sleep 2

# ========================
# 1) 桥接仿真时间
# ========================
start_bg ros2 run ros_ign_bridge parameter_bridge \
  /clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock

# ========================
# 2) 桥接点云
# ========================
start_bg ros2 run ros_ign_bridge parameter_bridge \
  /lidar/top/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked
start_bg ros2 run ros_ign_bridge parameter_bridge \
  /lidar/bottom/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked

sleep 1

# ========================
# 3) 静态 TF：把点云 frame_id 接入 TF 树
# ========================
start_bg ros2 run tf2_ros static_transform_publisher \
  --x "$BASE_X" --y "$BASE_Y" --z "$BASE_Z" \
  --roll "$BASE_ROLL" --pitch "$BASE_PITCH" --yaw "$BASE_YAW" \
  --frame-id "$FIXED_FRAME" --child-frame-id "$BASE_FRAME"

start_bg ros2 run tf2_ros static_transform_publisher \
  --x "$TOP_X" --y "$TOP_Y" --z "$TOP_Z" \
  --roll "$TOP_ROLL" --pitch "$TOP_PITCH" --yaw "$TOP_YAW" \
  --frame-id "$BASE_FRAME" --child-frame-id "$TOP_FRAME"

start_bg ros2 run tf2_ros static_transform_publisher \
  --x "$BOT_X" --y "$BOT_Y" --z "$BOT_Z" \
  --roll "$BOT_ROLL" --pitch "$BOT_PITCH" --yaw "$BOT_YAW" \
  --frame-id "$BASE_FRAME" --child-frame-id "$BOT_FRAME"

sleep 1

# ========================
# 4) RViz2（自动 use_sim_time）
# ========================
RVIZ_CFG="$HOME/ros2_ws/spray_robot-ws/rviz/spray.rviz"
if [ -f "$RVIZ_CFG" ]; then
  start_bg rviz2 -d "$RVIZ_CFG" --ros-args -p use_sim_time:=true
else
  start_bg rviz2 --ros-args -p use_sim_time:=true
fi

echo ""
echo "✅ FULL bringup done."
echo "  Gazebo: $WORLD_SDF"
echo "  Clock : /clock"
echo "  PC2   : /lidar/top/points , /lidar/bottom/points"
echo "  Scan  : /lidar/top , /lidar/bottom"
echo "  TF    : $FIXED_FRAME -> $BASE_FRAME -> {$TOP_FRAME, $BOT_FRAME}"
echo ""
echo "按 Ctrl+C 结束（会自动关闭 Gazebo/bridge/tf/rviz）"
wait

