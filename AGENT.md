# Repository working agreement

- 保持代码优雅简洁：遵循 ROS 2 包结构，明确模块职责，避免重复实现和无用输出。
- 每次改动必须具有 Git 可追溯性：检查现有改动，按逻辑提交代码与文档，提交说明包含目的和验证结果。
- 运行参数只维护根目录 `config/standard3.yaml`；启动包只读取这一份配置，不在各业务包复制参数文件。
- ROS 包名使用 snake_case。相机 SDK 采集归 `camera_node`，圆形绿灯二维检测归 `detect_node`，MCU 字节协议、串口与通信消息归 `mcu_node`，视觉与 IMU 时间邻近观测归 `estimate_node`。
- 头文件放 `include/<包名>/`，实现放 `src/`，消息放 `msg/`，统一启动归 `dart_bringup`。
- 各业务节点只发布 ROS 消息与低频 diagnostics，不得自行实现 UDP 曲线输出。
- `mcu_node` 保留 `ros_sp` 已验证的线上字节协议。协议变更必须增加 golden-frame 测试；不得为对齐计数伪造 TX 或丢弃正常 RX。
- `detect_node` 的边界固定为 ROS Image/CameraInfo → 二维圆形绿灯观测，不混入相机 SDK、串口或执行机构控制。
- `estimate_node` 不猜测 MCU IMU 的单位与轴向；在单位和坐标系经实机确认前只原样携带并明确标记为未验证。
- 所有 ROS 2 代码、文档、构建和测试都属于本仓库。`/home/keylinkatasga/dev/doing/ros_sp` 是只读的已验证来源，不得在其中生成或修改文件。
- 本仓库规范远端为 `https://github.com/KeylinKatasga/DART.git`，不得把 DART 改动提交到 `ros_sp`。
- 保持各包可由 `colcon` 独立构建和测试。
