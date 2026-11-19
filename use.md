# 运算端启动

相机、IMU 和舵机接好后，在宿主机执行：

```bash
cd /home/keylinkatasga/dev/doing/DART
./start_compute.sh
```

入口会自动进入 `sp_vpie:dart` Docker 环境；首次运行或构建不存在时自动编译，随后启动：

```text
camera_node -> detect_node -> dart_coordinate_node -> dart_estimate_node
                                      + dart_self_estimate_node
                                      -> dart_calculate_node -> mcu_node
```

运行参数只修改：

```text
/home/keylinkatasga/dev/doing/DART/config/standard3.yaml
```

停止运行按 `Ctrl+C`。默认执行器输出仍由配置中的两级安全开关控制：

```yaml
mcu_node.ros__parameters.actuator_output_enabled: false
dart_calculate_node.ros__parameters.safety.output_enabled: false
```

完成舵机零位、方向、限位及动力学参数确认前，不要开启它们。
