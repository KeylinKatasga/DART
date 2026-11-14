# dart_algorithm_c

纯 C11 算法迁移库，不依赖 ROS、OpenCV、Eigen 或 Ceres。当前包含：

- 四元数与刚体 7 维自身 EKF；
- 9 维 Singer 目标滤波器；
- 视线误差到三轴修正、四舵分配与限幅；
- 图像帧和视觉处理回调接口，图像算法由部署端后续注入。

所有数组均为固定尺寸，库内部不申请动态内存。参数默认值只用于单元测试，实际质量、
惯量、气动分配和舵面分配必须由部署配置填写。

```bash
cmake -S algorithm_c -B build/algorithm_c
cmake --build build/algorithm_c
ctest --test-dir build/algorithm_c --output-on-failure
```
