# dart_estimate_node

订阅 `/dart_coordinate_node/observation`，发布 `/dart_estimate_node/target_state`。
当坐标节点提供三维位置时，节点使用 9 维 Singer 模型估计目标位置、速度和加速度，并支持
预测时域、过程噪声、量测噪声和马氏距离门限配置。没有三维位置时仍保留纯方位字段，供
降级链路使用。

自身姿态与角速度不在本节点估计，由 `dart_self_estimate_node` 的 7 维刚体 EKF 独立完成。
两套滤波器分离，避免把目标机动噪声与飞镖气动模型耦合到同一协方差中。
