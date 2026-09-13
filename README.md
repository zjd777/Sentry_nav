# ROS 2 导航与建图工作空间

这个工作空间面向 ROS 2 Humble，整合了 Livox 雷达驱动、FAST_LIO 建图、地面分割、点云转 2D LaserScan、small_gicp 重定位、Nav2 导航、IMU 滤波和 CH343 串口底盘通信。

## 环境

推荐环境：

- Ubuntu 22.04
- ROS 2 Humble
- `colcon`
- `gnome-terminal`
- Livox MID360/HAP 相关 SDK 与驱动依赖

## 编译

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

只重编某些包可以这样用：

```bash
COLCON_BUILD_ARGS="--packages-select fast_lio" ./mapping.sh
```

## 启动脚本

`mapping.sh` 用于启动建图链路，`nav2.sh` 用于启动导航、重定位和 RViz。

两个脚本都会：

- 自动 source `/opt/ros/${ROS_DISTRO:-humble}/setup.bash`
- 默认执行 `colcon build --symlink-install`
- source `install/setup.bash`
- 用多个 `gnome-terminal` 分别启动各节点

常用环境变量：

```bash
SKIP_BUILD=1 ./nav2.sh
ROS_DISTRO=humble ./mapping.sh
LAUNCH_DELAY=1 ./nav2.sh
```

## 主要链路

当前 3D 点云转 2D 栅格地图的链路是：

```text
/cloud_registered_body
  -> linefit_ground_segmentation_ros
  -> /segmentation/obstacle
  -> pointcloud_to_laserscan
  -> /scan
  -> slam_toolbox / Nav2
```

这里特意使用 `/cloud_registered_body` 做地面分割，而不是 `/cloud_registered`。原因是 linefit 地面分割按“以传感器/车体为中心”的径向模型工作，用世界系点云会让机器人移动后分割效果变差。

## 关键参数改动

近期已经做过这些优化：

- 地面分割输入改为 `/cloud_registered_body`，并用 `camera_init` 做重力对齐。
- 点云转 LaserScan 输出 frame 改为 `base_link`，方便 Nav2 和 AMCL 直接使用。
- LaserScan 有效范围统一到 10 m，角分辨率改为约 1 度。
- 新增 `min_points_per_bin`，默认每个角度桶至少 2 个点才认为是障碍，用于抑制孤立噪点。
- 高度裁剪改为 `min_height=-0.35`、`max_height=0.85`，减少地面残留和高处杂点被投到 2D 地图。
- slam_toolbox、Nav2 costmap、AMCL 的 scan range 已对齐，避免 scan 只有 4 m 但代价地图按 10/20 m 处理。
- local costmap 使用 `camera_init`，global costmap 使用 `map`。
- small_gicp 重定位的先验 PCD 路径改成 launch 参数，不再硬编码某台机器的 `/home/.../ros_ws` 路径。
- FAST_LIO 的 PCD 输出路径改成可配置参数，`pcd_save.pcd_save_path` 现在会被代码实际读取。
- CH343 串口驱动修过发送缓冲区、无设备忙等、关闭线程和 fd 失败处理等问题。

## FAST_LIO 地图保存

默认情况下，FAST_LIO 使用包内的 `PCD/scans.pcd`。也可以不改 YAML，直接通过 launch 参数指定保存目录：

```bash
ros2 launch fast_lio mapping.launch.py pcd_dir:=/tmp/fast_lio_pcd
```

指定完整地图文件：

```bash
ros2 launch fast_lio mapping.launch.py \
  pcd_dir:=/tmp/fast_lio_pcd \
  map_file:=/tmp/fast_lio_pcd/scans.pcd
```

## 重定位

small_gicp 重定位默认读取 FAST_LIO 的 `PCD/scans.pcd`，也可以手动指定：

```bash
ros2 launch small_gicp_relocalization small_gicp_relocalization_launch.py \
  prior_pcd_file:=/tmp/fast_lio_pcd/scans.pcd
```

重定位相关改动：

- `map_transform` 参数替代了原来写死在代码里的地图旋转偏置。
- 发布 `/initialpose` 时修正了 TF 查找方向。
- 默认累计一小段点云再配准，减少单帧稀疏点云带来的抖动。
- 跳变抑制只在初始定位成功后启用，避免初始收敛时自己把有效更新拦掉。

## 调参建议

如果 2D 地图仍然有散点：

- 把 `pointcloud_to_laserscan` 的 `min_points_per_bin` 从 `2` 调到 `3`。
- 把 `min_height` 从 `-0.35` 调到 `-0.25`，减少地面残留。
- 把地面分割的 `max_dist_to_line` 从 `0.08` 调到 `0.06`，让地面判定更严格。

如果障碍物漏检：

- 把 `min_points_per_bin` 从 `2` 降到 `1`。
- 把 `max_height` 从 `0.85` 提到 `1.0`。
- 检查 `/segmentation/obstacle` 是否已经把真实障碍物过滤掉。

如果重定位不稳定：

- 先确认 `prior_pcd_file` 指向的 PCD 和当前场地一致。
- 调整 `map_transform` 的 yaw，确保 PCD 地图方向与 Nav2 地图方向一致。
- 初始偏差较大时，适当放宽 `max_dist_sq` 或给更接近的 `init_pose`。

## 主要包

- `livox_ros_driver2`：Livox 雷达驱动和自定义消息。
- `fast_lio`：激光惯性里程计与点云地图构建。
- `linefit_ground_segmentation_ros`：地面/障碍物点云分割。
- `pointcloud_to_laserscan`：点云转 2D LaserScan。
- `small_gicp` / `small_gicp_relocalization`：点云配准与重定位。
- `nav2_bringup`：Nav2 启动和参数配置。
- `imu_tools`：IMU 互补滤波和 Madgwick 滤波。
- `serial_driver_ch343`：CH343 UART 底盘串口桥接。

## 常用检查命令

```bash
ros2 topic echo /scan --once
ros2 topic hz /scan
ros2 topic hz /segmentation/obstacle
ros2 run tf2_ros tf2_echo camera_init body
ros2 run tf2_ros tf2_echo body base_link
ros2 service call /map_save std_srvs/srv/Trigger {}
```
