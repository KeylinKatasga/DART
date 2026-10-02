# camera_node

ROS 2 Humble 单大恒相机驱动。默认发布：

- `/camera_node/image_raw`：`sensor_msgs/msg/Image`，`bgr8`，Bayer 解码后但未去畸变。
- `/camera_node/camera_info`：`sensor_msgs/msg/CameraInfo`，与图像共用时间戳和 frame_id。
- `/diagnostics`：1 Hz 连接、帧率、超时、帧号间隙、转换耗时和时间来源。

驱动迁移自 `sp_-vpie` 提交 `5fc6239` 的 `io/camera.hpp`、
`io/daheng/{daheng.cpp,gx_camera.cpp}`。保留连续采集、ROI、曝光/增益、白平衡和
Bayer8→BGR 方法；ROS 适配修正了错误返回、清理遗漏、latch 调用、接收时间采样点、
ticks 下溢和 SDK 已知频率不启用统计等边界。构建不链接上游 `io` 库。

## 构建约束

镜像通过 `GALAXY_SDK_ROOT` 声明 Galaxy SDK 根目录。CMake 从该根目录的 `include/`
和常见 `lib/` 子目录查找 `GxIAPI.h`、`DxImageProc.h` 与 `libgxiapi.so`，不依赖某台
开发机或某个工作区的源码路径。镜像外构建时可通过 CMake cache 或同名环境变量指定
SDK 根目录。

所有硬件参数第一版仅在启动时应用。空 `camera_sn` 只适用于单相机环境；多相机必须指定
序列号。默认配置来自根目录 `config/standard3.yaml`，保留当前已验证设备序列号。
实际有效 ROI、曝光和增益由 SDK 回读后在启动日志与 diagnostics 中报告。

标定使用标准 ROS camera calibration YAML：
在 `standard3.yaml` 中设置 `camera.calibration_url: file:///absolute/path/calibration.yaml`。
标定尺寸与输出尺寸不一致时按未标定发布（K[0] 为 0）。`flip_image=true` 与上游
`sp_vpie/io/daheng/daheng.cpp` 一致，在 Bayer→BGR 后执行 `cv::flip(image, image, -1)`，
可与 `camera_info_url` 同时使用。翻转不自动生成标定或手眼外参：对应 profile 必须使用
与相机实际安装方向一致的标定和外参。

`header.stamp` 优先由 Galaxy 硬件 timestamp latch 映射到主机 steady 时钟，再相对映射到
ROS 时钟；缺少设备 tick、频率或 latch 时使用成功取帧后、转换前采样的主机接收时间。
diagnostics 的 `time_source` 会明确显示来源。该时间戳不声明为曝光开始或曝光结束。

## 查看、记录与故障定位

本机图像工具订阅 `/camera_node/image_raw`。PlotJuggler 的 ROS 2 数据源订阅
`/diagnostics` 查看窗口 capture_fps、窗口 publish_fps、processing_ms、conversion_ms、arrival_jitter_ms、timeouts
和 frame_gaps；不新增 UDP 9870 发送端。

`logger_node/record_bag` 的第三个参数是可选 MCAP 分卷字节数，0 表示不按大小分卷。
camera 模式记录 image_raw、camera_info 和 diagnostics；all 再加入 MCU RX/TX 观测，
永不记录 `/mcu_node/command`。

无设备时节点保持运行、每 2 秒重试，重复错误最多每 5 秒打印一次。常见检查：容器是否
挂载 `/dev`、是否 privileged、Galaxy 动态库能否由 `ldconfig -p` 找到、序列号是否正确、
相机是否已被其他进程独占。

统一构建、启动、录包和回放命令见根目录 [`USE.md`](../../USE.md)。

## 验证状态

已由软件/模拟验证：参数拒绝、BGR8 内容与尺寸、Image/CameraInfo 时间戳和 frame_id、
低噪声采集、三包联合编译、临时 MCAP 的相机记录与无硬件回放。

等待大恒实机验证：设备型号与权限、真实颜色/方向、ROI/曝光/增益/白平衡回读、标定、
硬件 tick 语义与频率、10 分钟持续采集、USB 拔插恢复、退出后立即重启、与 MCU 联合运行
以及目标分辨率/帧率下的磁盘吞吐。

