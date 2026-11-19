# DART 进入与使用说明

本文档面向当前 `green_light_cpp` 验证分支。项目目录为：

```bash
/home/keylinkatasga/dev/doing/DART
```

当前推荐使用 `sp_vpie:dart` Docker 镜像运行 ROS 2 Humble、OpenCV、Eigen、Ceres、
大恒相机 SDK 和项目节点。所有命令均在本机执行，除非特别说明。

## 1. 进入开发环境

先确认镜像、相机和 MCU：

```bash
docker image inspect sp_vpie:dart >/dev/null
lsusb
ls -l /dev/ttyACM0
```

MCU 没有连接时，`/dev/ttyACM0` 不存在是正常的；此时只能运行无 MCU 的验证模式。

启动临时开发容器：

```bash
cd /home/keylinkatasga/dev/doing/DART

docker run --rm -it \
  --name dart-dev \
  --network host \
  --ipc host \
  --privileged \
  --user "$(id -u):$(id -g)" \
  -e HOME=/tmp/dart-home \
  -v /home/keylinkatasga/dev/doing/DART:/workspace/DART \
  -w /workspace/DART \
  sp_vpie:dart bash
```

容器是临时的，退出后会删除；源代码、`build/`、`install/` 和 `log/` 位于宿主机
仓库中，不会随容器删除。`--privileged` 用于访问 USB 相机和串口，只应在可信的本机
开发镜像中使用。

进入容器后加载 ROS：

```bash
source /opt/ros/humble/setup.bash
```

需要第二个终端观察 ROS topic 时，在宿主机的新终端执行：

```bash
docker exec -it dart-dev bash
cd /workspace/DART
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 2. 编译与测试

在容器中的 `/workspace/DART` 执行：

```bash
colcon build --symlink-install --event-handlers console_cohesion+
source install/setup.bash
colcon test --event-handlers console_cohesion+
colcon test-result --verbose
```

每次打开新终端或新容器，都需要重新执行：

```bash
source /opt/ros/humble/setup.bash
cd /workspace/DART
source install/setup.bash
```

只编译 MCU 节点时：

```bash
colcon build --packages-select mcu_node --symlink-install
source install/setup.bash
```

## 3. 首次建议：无硬件算法自检

先运行完整的合成输入闭环，不访问真实相机、串口或舵机：

```bash
python3 tools/filter_chain_smoke.py
```

正常结果包含：

```text
PASS: circle PnP -> Singer -> self EKF -> safe fin command
```

验证简化后的 MCU 字节协议和伪串口收发：

```bash
python3 tools/dart_fin_mcu_smoke.py
```

正常结果包含：

```text
PASS: simplified MCU RX/TX
```

这个测试使用 PTY 伪串口，不会驱动真实舵机。

验证 MCU RX 到坐标/目标估计节点的数据交接：

```bash
python3 tools/mcu_estimate_smoke.py
```

## 4. 仅验证相机与绿灯检测

当前标准相机 SN 为 `KE0210080605`，配置位于
`/workspace/DART/config/standard3.yaml`。

启动相机和检测节点，关闭 MCU、滤波和控制节点：

```bash
ros2 launch dart_bringup dart.launch.py \
  config:=/workspace/DART/config/standard3.yaml \
  mcu:=false \
  dart_coordinate:=false \
  dart_estimate:=false \
  dart_self_estimate:=false \
  dart_calculate:=false
```

另开一个进入同一 Docker 环境的终端，加载环境后检查：

```bash
ros2 node list
ros2 topic hz /camera_node/image_raw
ros2 topic echo /detect_node/detection
ros2 topic hz /detect_node/debug_image
```

也可以等待一次有效绿灯识别：

```bash
python3 tools/probe_green_light.py --timeout 20 --require-delta
```

输出会包含：

- 圆心 `center_px`；
- 半径 `radius_px`；
- 帧间像素变化 `delta_px`；
- 像素速度 `velocity_px_s`；
- 根据相机内参计算的偏转角 `bearing_rad`；
- 角速度 `bearing_velocity_rad_s`。

当前场景没有绿灯时，`detected: false` 是正确结果，不代表相机或检测节点故障。

## 5. 相机与 MCU 的目标估计链路

MCU 默认串口为 `/dev/ttyACM0`，波特率为 115200。当前协议为：

- MCU → 主机：30 字节 payload，包含 IMU 四元数和比赛状态；
- 主机 → MCU：22 字节 payload，包含 `angle1` 到 `angle4` 及控制标志；
- 四个舵机从左上角开始顺时针编号，呈 X 型分布；
- CRC16 与 `ros_sp` 一致。

当前 MCU RX 帧没有原始加速度计、角速度和舵角反馈，因此实机视觉/PnP/Singer 验证时，
先关闭自身 EKF 和控制节点：

```bash
ros2 launch dart_bringup dart.launch.py \
  config:=/workspace/DART/config/standard3.yaml \
  dart_self_estimate:=false \
  dart_calculate:=false
```

在另一个已加载环境的终端中观察各级输出：

```bash
ros2 topic echo /mcu_node/message/rx
ros2 topic echo /detect_node/detection
ros2 topic echo /dart_coordinate_node/observation
ros2 topic echo /dart_estimate_node/target_state
ros2 topic echo /diagnostics
```

若只想验证 MCU 接收，可仅启动 MCU 节点：

```bash
ros2 run mcu_node mcu_node --ros-args \
  --params-file /workspace/DART/config/standard3.yaml
```

## 6. 完整估计与控制框架

只有在系统已经提供标准 `/imu/data`（`sensor_msgs/msg/Imu`，包含原始加速度和角速度）后，
才启动自身 EKF 和计算节点：

```bash
ros2 launch dart_bringup dart.launch.py \
  config:=/workspace/DART/config/standard3.yaml
```

检查：

```bash
ros2 topic hz /imu/data
ros2 topic echo /dart_self_estimate_node/state
ros2 topic echo /dart_calculate_node/guidance
ros2 topic echo /mcu_node/fin_command
```

当前配置中有两级执行安全开关，并且都默认为关闭：

```yaml
mcu_node:
  ros__parameters:
    actuator_output_enabled: false

dart_calculate_node:
  ros__parameters:
    safety:
      output_enabled: false
```

因此默认启动只计算和发布安全的中性命令，不会向真实执行机构发送舵角。

> 不要在当前占位参数下打开执行输出。质量、惯量、舵面面积、力臂、法向、铰链轴、
> 舵机方向、零位和限位全部实测并完成台架限幅验证后，才能同时打开上述两个开关。

## 7. 常用配置位置

唯一正式运行配置：

```text
/workspace/DART/config/standard3.yaml
```

常改项目：

- `camera_node`：相机 SN、曝光、增益、ROI；
- `detect_node.threshold`：HSV 绿色范围；
- `detect_node.filter`：面积、半径、圆度和填充率；
- `dart_coordinate_node.solution_mode`：`known_circle_pnp` 或 `bearing_only`；
- `dart_coordinate_node.target_diameter_m`：绿灯实际发光直径；
- `dart_estimate_node.prediction.model`：`singer`、`static` 或
  `sinusoidal_velocity`；
- `dart_calculate_node.model.airframe`：`four_independent`、
  `diagonal_coupled` 或 `fixed_movable`；
- `dart_calculate_node.model.controller`：当前使用 `pid`，`mpc` 仍为保留项。

因为启动命令显式传入根目录配置，修改 `config/standard3.yaml` 后重新启动节点即可，
不需要为参数修改重新编译。

## 8. 常见问题

### 相机打不开

```bash
lsusb
```

确认相机已被宿主机识别、SN 与配置一致，并确认容器使用了 `--privileged`。不要同时运行
两个占用同一相机的进程。

### MCU 串口打不开

```bash
ls -l /dev/ttyACM0
```

确认设备名与 `config/standard3.yaml` 一致，关闭可能占用串口的串口助手或旧 ROS 进程。
重新插拔后设备名可能变成 `/dev/ttyACM1`，此时修改配置再启动。

### 检测不到绿灯

先确认图像持续发布：

```bash
ros2 topic hz /camera_node/image_raw
```

然后检查环境中的实际 HSV、曝光、目标面积和圆度是否落在
`detect_node` 的配置范围内。没有目标时持续输出 `detected: false` 是设计行为。

### PnP 距离明显不正确

必须填写真实的 `target_diameter_m`，并使用对应分辨率下的真实相机标定参数。当前
`0.1 m` 是验证占位值，不能作为实物测距精度依据。

### 自身 EKF 一直等待

检查：

```bash
ros2 topic hz /imu/data
```

MCU 的简化 RX 帧只有四元数，不会自动生成 `/imu/data`。在原始加速度和角速度链路完成前，
应使用 `dart_self_estimate:=false dart_calculate:=false` 启动。

## 9. 停止与退出

在启动节点的终端按 `Ctrl+C`。退出容器：

```bash
exit
```

容器使用 `--rm`，退出后自动删除；仓库文件仍保留在
`/home/keylinkatasga/dev/doing/DART`。
