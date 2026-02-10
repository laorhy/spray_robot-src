# pointcloud2mesh 包

## 功能概述

`pointcloud2mesh` 是一个 ROS 2 包，用于将点云数据转换为网格模型。它提供了一个 ROS 2 服务接口，可以将缓存的 ROI（感兴趣区域）点云数据转换为 PLY 或 STL 格式的网格文件。

### 主要功能：

- 将点云数据转换为网格模型（使用 PCL 的 Poisson 重建算法）
- 支持 PLY 和 STL 格式的输出
- 提供丰富的参数配置选项
- 增强的错误处理和日志记录
- 详细的服务响应信息

## 系统要求

- ROS 2 Humble 或更高版本
- PCL (Point Cloud Library)
- C++17 兼容的编译器
- 可选：OpenMP 支持（加速法线估计）

## 安装

1. 将包克隆到您的 ROS 2 工作空间：

   ```bash
   cd ~/ros2_ws/src
   git clone <repository_url>
   ```

2. 安装依赖：

   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. 编译包：

   ```bash
   colcon build --packages-select pointcloud2mesh
   ```

4. 激活环境：

   ```bash
   source install/setup.bash
   ```

## 使用方法

### 1. 启动服务节点

使用 launch 文件启动服务节点：

```bash
ros2 launch pointcloud2mesh cloud_to_mesh_server.launch.py
```

或者直接运行节点：

```bash
ros2 run pointcloud2mesh cloud_to_mesh_server
```

### 2. 调用服务

使用 `ros2 service call` 命令调用服务：

```bash
ros2 service call /save_mesh pointcloud2mesh/srv/SaveMesh "{
  output_dir: '/tmp/meshes',
  filename_prefix: 'ship_hull',
  save_ply: true,
  save_stl: true,
  poisson_depth: 10,
  scale: 1.1,
  samples_per_node: 1.5,
  point_weight: 4.0,
  normal_k: 30,
  wait_timeout_sec: 5.0
}"
```

### 3. 服务响应示例
success: True
mesh_path: "/tmp/meshes/ship_hull_20260210_123456.ply"
num_points: 125000
num_triangles: 245000
message: "Mesh saved successfully (125000 points, 245000 triangles). Multiple outputs: /tmp/meshes/ship_hull_20260210_123456.ply, /tmp/meshes/ship_hull_20260210_123456.stl"


## 参数说明

### 服务请求参数

| 参数 | 类型 | 描述 | 默认值 | 推荐值 |
|------|------|------|--------|--------|
| output_dir | string | 输出目录路径 | /tmp/pointcloud_mesh | 任何可写目录 |
| filename_prefix | string | 文件名前缀 | roi_mesh | 描述性名称 |
| save_ply | bool | 是否保存为 PLY 格式 | true | true |
| save_stl | bool | 是否保存为 STL 格式 | false | false |
| poisson_depth | int | Poisson 重建深度 | 10 | 8-12（值越大细节越多但速度越慢） |
| scale | float | 缩放因子 | 1.1 | 1.1 |
| samples_per_node | float | 每个节点的样本数 | 1.5 | 1.0-2.0 |
| point_weight | float | 点权重 | 4.0 | 2.0-6.0 |
| normal_k | int | 法线估计的最近邻数 | 30 | 20-50 |
| wait_timeout_sec | float | 等待点云数据的超时时间 | 1.0 | 1.0-5.0 |

### 节点参数

可以在启动时通过 `--ros-args` 设置以下参数：

| 参数 | 类型 | 描述 | 默认值 |
|------|------|------|--------|
| default_output_dir | string | 默认输出目录 | /tmp/pointcloud_mesh |
| default_filename_prefix | string | 默认文件名前缀 | roi_mesh |
| default_save_ply | bool | 默认是否保存为 PLY 格式 | true |
| default_save_stl | bool | 默认是否保存为 STL 格式 | false |
| use_sim_time | bool | 是否使用仿真时间 | false |

## 工作原理

1. **点云订阅**：节点订阅 `/lidar/fused/roi` 话题获取点云数据
2. **点云缓存**：缓存最新的点云数据
3. **服务处理**：当收到 `/save_mesh` 服务请求时：
   - 等待并获取最新的点云数据
   - 估计点云法线
   - 使用 Poisson 算法重建网格
   - 保存网格为指定格式
   - 返回详细的服务响应

## 故障排除

### 常见问题及解决方案

1. **"No fresh cached point cloud available before timeout"**
   - 原因：没有收到点云数据
   - 解决方案：确保点云发布者正在运行，并且话题名称正确

2. **"Poisson reconstruction produced an empty mesh"**
   - 原因：点云数据不足或 Poisson 参数不合适
   - 解决方案：增加点云密度或调整 Poisson 参数（如减小 depth）

3. **"Failed to create output directory"**
   - 原因：权限不足或路径不存在
   - 解决方案：确保输出目录路径正确，并且有写入权限

4. **"Failed to save PLY/STL file"**
   - 原因：磁盘空间不足或权限问题
   - 解决方案：确保有足够的磁盘空间和写入权限

### 日志级别

可以通过设置日志级别来获取更详细的信息：

```bash
ros2 run pointcloud2mesh cloud_to_mesh_server --ros-args --log-level debug
```

## 性能优化

1. **使用 OpenMP**：确保 PCL 编译时启用了 OpenMP 支持，以加速法线估计
2. **调整 Poisson 参数**：根据点云大小和所需细节调整 `poisson_depth` 参数
3. **减少点云密度**：对于大型点云，可以先进行下采样以提高处理速度

## 最佳实践

1. **点云预处理**：在转换前对点云进行滤波和下采样，以提高重建质量和速度
2. **参数调优**：
   - 对于大型点云：使用较小的 `poisson_depth`（如 8-9）
   - 对于小型点云：使用较大的 `poisson_depth`（如 10-12）
3. **文件格式选择**：
   - PLY：保留更多信息，适合进一步处理
   - STL：更通用，适合 3D 打印和 CAD 软件

## 示例

### 基本用法示例

```bash
# 启动服务
ros2 launch pointcloud2mesh cloud_to_mesh_server.launch.py

# 调用服务（基本配置）
ros2 service call /save_mesh pointcloud2mesh/srv/SaveMesh "{
  output_dir: '/home/user/meshes',
  filename_prefix: 'test',
  save_ply: true,
  save_stl: false
}"
```

### 高级用法示例

```bash
# 启动服务（自定义默认参数）
ros2 run pointcloud2mesh cloud_to_mesh_server --ros-args \
  -p default_output_dir:=/home/user/meshes \
  -p default_filename_prefix:=ship \
  -p default_save_ply:=true \
  -p default_save_stl:=true

# 调用服务（自定义重建参数）
ros2 service call /save_mesh pointcloud2mesh/srv/SaveMesh "{
  poisson_depth: 11,
  normal_k: 40,
  wait_timeout_sec: 3.0
}"
```

## 相关包

- **pointcloud_fusion**：用于融合多个激光雷达的点云数据
- **pcl_ros**：ROS 2 与 PCL 的接口包

## 许可证

Apache-2.0