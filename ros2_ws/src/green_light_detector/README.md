# green_light_detector

ROS 2 Humble C++/OpenCV 圆形绿光检测节点。节点只输出视觉测量，不输出执行机构或控制指令。

## 接口

- 输入：`/camera_node/image_raw` (`bgr8`)
- 输入：`/camera_node/camera_info`
- 输出：`/green_light_detector/detection` (`dart_interfaces/msg/GreenLight`)
- 输出：`/green_light_detector/debug_image` (`bgr8`，有订阅者时才生成)
- 输出：`/diagnostics`

`center_x_px/center_y_px/radius_px` 是当前帧检测结果。`delta_x_px/delta_y_px`
是相邻连续有效帧的圆心差；检测丢失或时间间隔超过 `max_track_gap_s` 时
`delta_valid=false`，重新捕获不会跨丢失帧计算差分。

偏转角以相机光轴为零点，右为正 yaw，上为正 pitch：

```text
bearing_x = atan((u - cx) / fx)
bearing_y = atan((cy - v) / fy)
```

若 CameraInfo 带畸变参数，节点先用 `undistortPoints` 得到归一化坐标。配置中的
standard3 内参仅作为 CameraInfo 尚未到达时的回退值。

## 构建与运行

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch green_light_detector green_light.launch.py
```

查看输出：

```bash
ros2 topic echo /green_light_detector/detection
ros2 run rqt_image_view rqt_image_view /green_light_detector/debug_image
```

阈值位于 `config/standard3_green_light.yaml`。发光体过曝时应先缩短曝光，再调整
`saturation_min`、`value_min` 和形态学闭运算次数。
