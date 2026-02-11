# Spray Robot System  

## 单块区域自动喷涂机器人系统（ROS2）

---

## 一、项目简介

本项目为一套基于 **ROS2** 构建的自动喷涂机器人系统，面向单块区域（例如 3m × 3m）自动喷涂作业场景。

系统采用**分布式部署架构**：

- **喷涂控制工控机（长臂末端）**  
  运行机器人控制系统，负责感知、建模、路径规划与执行。

- **HMI工控机（移动底盘上）**  
  运行人机交互界面（UI），负责任务下发与状态显示。

系统当前设计目标为：

> 在人工将移动底盘与长臂调整至作业区域后，自动完成该单块区域的喷涂全过程。

---

## 二、系统总体架构

### 1. 分布式通信结构

```markdown
HMI工控机  <--- hmi_msgs --->  喷涂控制工控机
                                   │
                                   │
                          spray_robot_interfaces
                                   │
         ┌──────── perception ─────────────┐
         │                                 │
   pointcloud_fusion                pointcloud2mesh
         │                                 │
         └──────── planning ───────────────┘
                       │
              spray_path_planner
                       │
                    MoveIt
                       │
                motion_coordinator
                       │
         arm_command + slider_command

```

安全监控模块独立运行，可随时打断执行。

---

## 三、系统功能流程

功能：

- 单块区域自动喷涂  
  - 双激光雷达曲面建模  
  - Noether喷涂路径生成  
  - 七轴运动规划  
  - 喷枪协同执行  
  - 激光安全距离监测  
- 分布式HMI显示  

流程：

1. 操作人员手动将移动底盘与长臂调整到目标区域
2. 在HMI界面点击“开始喷涂”
3. 系统自动执行以下流程：
   - 安全检测
   - 双激光雷达感知喷涂曲面
   - 喷涂曲面网格生成
   - Noether喷涂路径生成
   - MoveIt七轴运动规划
   - 机械臂+滑台协同喷涂执行
4. HMI实时显示：
   - 当前阶段
   - 作业进度
   - 预计剩余时间
   - 安全状态
   - 告警信息
5. 区域喷涂完成后停止任务，等待下一次手动触发

---

## 四、目录结构说明

### 1. 核心系统模块

#### spray_robot_bringup

系统总启动入口，负责启动：

- 雷达厂家ROS2节点
- 点云处理链
- Noether服务
- MoveIt
- 任务执行模块
- 安全监控模块
- HMI后端接口

---

#### spray_robot_description

包含：

- 机械臂 + 滑台 URDF / Xacro
- TF结构
- MoveIt依赖配置

---

#### spray_robot_interfaces

机器人内部通信接口定义，包括：

- action：任务执行
- srv：路径规划、触发控制
- msg：状态与错误码

该接口包仅用于喷涂控制系统内部节点通信。

---

### 2. 传感器模块（sensor）

#### range_sensor_driver

- 通过 Modbus RTU 读取4个激光位移传感器
- 发布距离数据
- 提供通信状态监测

#### camera_bridge

- 连接 RTSP 视频流
- 提供拍照触发服务
- 发布拍照结果信息
- 视频流可供外部客户端查看

> 激光雷达采用厂家提供的ROS2节点，直接在bringup中启动。

---

### 3. 感知模块（perception）

#### pointcloud_fusion

功能：

- 订阅两路激光雷达点云
- TF坐标统一
- 轻量滤波与裁剪
- 融合输出统一点云

#### pointcloud2mesh

功能：

- 根据融合点云生成曲面网格
- 输出网格供 Noether 使用

---

### 4. 规划模块（planning）

#### spray_path_planner

功能：

- 调用 Noether 服务生成喷涂路径

#### motion_planner

功能：

- 调用 MoveIt 进行7轴（6DOF机械臂 + 单轴滑台）运动规划
- 输出可执行轨迹

---

### 5. 执行模块（execution）

#### arm_command

发送机械臂轨迹指令。

#### slider_command

发送滑台轨迹指令。

#### motion_coordinator

执行协调核心模块：

- 同步机械臂与滑台运动
- 控制喷枪开关
- 管理执行节奏
- 处理中断与异常

---

### 6. 任务模块（task）

#### task_manager

单块区域喷涂流程控制器（状态机）：

执行流程：

1. 安全检查
2. 点云融合
3. 曲面建模
4. Noether路径生成
5. MoveIt规划
6. 执行喷涂
7. 完成或失败处理

---

### 7. 安全模块（safety）

#### spray_safety_monitor

功能：

- 监听4个激光测距传感器
- 判断安全距离
- 发布安全状态
- 必要时触发紧急停止

---

### 8. HMI模块（hmi）

#### hmi_backend

运行于喷涂控制工控机：

- 作为跨机通信网关
- 对外使用 hmi_msgs 协议
- 对内调用 spray_robot_interfaces

---

#### hmi_msgs

跨机通信接口定义，包括：

- RunSingleBlock.action
- HmiStatus.msg
- 告警与状态信息

用于UI与喷涂控制系统之间通信。

---

## 五、接口分层设计原则

### 内部接口

- spray_robot_interfaces
- 用于机器人内部节点之间通信
- 可包含复杂结构与详细数据

### 外部接口

- hmi_msgs
- 字段精简
- 面向跨机通信
- 强调稳定性与兼容性

---

## 六、系统设计原则

- 单块闭环优先
- 模块清晰解耦
- 分布式部署
- 内外接口分层
- 安全优先
- 可扩展但不过度设计

---

## 七、后续扩展方向

- 多块区域连续喷涂
- 自动底盘定位与联动
- 喷涂参数自适应
- 图像辅助质量检测
- 作业日志与数据追溯
- 故障自动恢复策略

---

## 八、运行方式

```bash
source /opt/ros/humble/setup.bash
cd ~/spray_robot_ws
colcon build
source install/setup.bash
ros2 launch spray_robot_bringup bringup.launch.py
```
