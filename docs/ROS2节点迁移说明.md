# ROS 2 节点迁移说明

迁移来源为只读的 `/home/keylinkatasga/dev/doing/ros_sp`。DART 采用同一套 ROS 2 Humble
接口约定，但拥有独立工作区、配置和 Git 历史；迁移过程没有修改来源工作区。

| DART 节点 | 作用 | 来源/适配 |
| --- | --- | --- |
| `camera_node` | Daheng 相机采集 | 迁移已验证实现；Docker 下优先使用挂载的 Daheng SDK 头文件路径。 |
| `detect_node` | HSV、形态学、圆度和填充率筛选的绿圆检测 | DART 专用 OpenCV 实现，取代早期 `green_light_detector` 包名。 |
| `mcu_node` | gimbal 串口协议解析与 Rx/Tx 消息 | 迁移已验证实现和协议单元测试。 |
| `estimate_node` | 绿灯测量与最近 IMU 样本配对 | DART 专用轻量估计输出；不进行控制。 |

统一入口为：

```bash
ros2 launch dart_bringup dart.launch.py
```

启用状态下的数据流为：

```text
camera_node -> detect_node -> estimate_node
mcu_node --------------------^ 
```

`estimate_node` 仅保留 MCU 原始 IMU 字段；协议尚未在 DART 内定义物理单位和机体系轴向时，
会发布 `imu_units_verified=false`，因此不会擅自进行角度或加速度换算。
