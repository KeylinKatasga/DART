# dart_estimate_node

订阅 `/detect_node/detection` 与 `/mcu_node/message/rx`，发布
`/dart_estimate_node/target_state`。当前阶段完成时间邻近性检查并把绿灯视线观测与最近 MCU IMU
样本组合为一个结构化消息。

MCU 协议桥保持线上数值原样，固件的角速度/角加速度单位与轴向尚未在 DART 实机记录中
确认，因此 `imu_units_verified=false`，本节点不会擅自做单位换算或视线去旋转。完成固件
约定核验后，才可在这里加入 IMU 稳像与滤波。
