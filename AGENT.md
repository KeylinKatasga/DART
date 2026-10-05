# Repository working agreement

- 保持代码优雅简洁：遵循 ROS 2 包结构，明确模块职责，避免重复实现和无用输出。
- 每次改动必须具有 Git 可追溯性：检查现有改动，按逻辑提交代码与文档，提交说明包含目的和验证结果。
- 运行参数只维护根目录正式 profile（当前为 `config/standard3.yaml`）；启动包在内存中严格映射为各节点 ROS 参数，不复制或生成第二份配置。
- 运行资源归根目录 `assets/`；配置使用相对路径时由启动加载器解析为绝对资源路径，不依赖终端目录或上游仓库挂载。
- ROS 包名使用 snake_case 且节点包以 `_node` 结尾；当前为 `mcu_node`、`camera_node`、`detect_node`、`dart_estimate_node`、`dart_calculate_node`、`dart_interfaces` 与 `dart_bringup`。头文件放 `include/<包名>/`，实现放 `src/`，消息放 `msg/`，启动归 `launch/`。
- MCU 字节协议、串口与通信消息归 `mcu_node`；各业务节点只发布 ROS 消息与低频 diagnostics，不得自行实现 UDP 曲线输出。默认关闭执行器输出；不得为对齐计数而伪造 TX 或丢弃正常 RX。
- 相机 SDK 采集、标准图像/标定发布和相机 diagnostics 归 `camera_node`；`detect_node` 的边界固定为 ROS Image/CameraInfo → 二维圆形绿灯观测，不混入相机 SDK、串口、运动预测或执行机构控制。
- `dart_estimate_node` 将绿灯观测和 IMU 组合为带时间戳的目标状态与预测；不得猜测 MCU IMU 的单位与轴向，未经实机确认的字段必须明确标记。
- `dart_calculate_node` 只负责绿灯预测状态到抽象攻角/绕角修正、控制器和尾翼分配；不得直接访问串口。尾翼机构、控制器和预测模型必须可由根目录配置选择。
- 所有 ROS 2 代码、构建和测试都属于本仓库。历史 `docs/` 日志不属于当前业务重构范围。`ros_sp` 是只读的已验证来源，不得在其中生成或修改文件。
- 本仓库规范远端为 `https://github.com/KeylinKatasga/DART.git`，不得把 DART 改动提交到 `ros_sp`。
- 保持迁移包模块化，使每个 ROS 包均可由 `colcon` 独立构建和测试。协议变更必须增加 golden-frame 测试后再做实机发送。
