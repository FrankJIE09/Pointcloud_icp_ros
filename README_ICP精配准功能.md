# ICP精配准功能说明

## 功能概述
本ROS服务现已集成完整的点云配准流程，包含：
1. **粗配准**：PyTorch实现的初始对齐
2. **ICP精配准**：PyTorch实现的精确配准
3. **结果保存**：自动保存配准结果点云文件

## 配准流程

### 步骤1：粗配准
- 使用PyTorch优化器进行初始对齐
- 通过最小化最近邻距离优化旋转和平移
- 可配置迭代次数和学习率

### 步骤2：ICP精配准
- 支持两种ICP方法：
  - `point_to_point`：点到点ICP（默认）
  - `point_to_plane`：点到平面ICP（需要法向量）
- 迭代优化变换矩阵直到收敛
- 多种收敛条件检查

## 配置参数

在 `pose_estimation_ros_config.yaml` 中新增的ICP参数：

```yaml
# ICP精配准配置
use_icp: true                       # 是否启用ICP精配准
icp_max_iterations: 100             # ICP最大迭代次数
icp_distance_threshold: 0.1         # ICP对应点距离阈值
icp_rmse_threshold: 1e-6            # ICP收敛RMSE变化阈值
icp_transform_threshold: 1e-5       # ICP收敛变换矩阵变化阈值
icp_estimation_method: 'point_to_point'  # ICP估计方法
```

## 输出文件

每次运行后会自动保存以下点云文件：

1. **`registration_result_scene_YYYYMMDD_HHMMSS.pcd`**
   - 配准后的场景点云（黄色）
   - 已应用最终变换矩阵

2. **`registration_result_target_YYYYMMDD_HHMMSS.pcd`**
   - 目标模型点云（蓝色）
   - 用于比较和验证配准效果

3. **`registration_result_combined_YYYYMMDD_HHMMSS.pcd`**
   - 合并的点云文件
   - 包含配准后的场景点云和目标模型点云

## 使用方法

### 1. 启动ROS服务
```bash
rosrun your_package pose_estimation_ros_service.py
```

### 2. 调用服务
```bash
rosservice call /estimate_pose "{}"
```

### 3. 查看结果
- 检查终端输出的姿态估计结果
- 查看保存的点云文件验证配准效果
- 使用点云查看器（如PCL Viewer、Open3D）打开结果文件

## 日志输出示例

```
=== 开始姿态估计 ===
输入点云: data/test.pcd
目标模型: stp/part2_rude.STL
使用直接配准方法
场景点云: 5000 点
目标模型: 20480 点

=== 步骤1：粗配准 ===
Running PyTorch coarse alignment...
Iteration 50/200, Loss: 0.025431
Iteration 100/200, Loss: 0.018234
...
粗配准完成

=== 步骤2：ICP精配准 ===
Running PyTorch ICP (point_to_point)...
ICP iter 10/100: RMSE: 0.045123, Fitness: 0.892456, Corr: 4125
ICP iter 20/100: RMSE: 0.032187, Fitness: 0.905234, Corr: 4234
...
ICP converged at iter 35 due to RMSE change.
ICP完成 - Fitness: 0.912345, RMSE: 0.028456

保存配准后场景点云: registration_result_scene_20241210_143052.pcd
保存目标模型点云: registration_result_target_20241210_143052.pcd
保存合并点云: registration_result_combined_20241210_143052.pcd

=== 姿态估计结果 ===
位置 (XYZ): [0.123456, -0.234567, 0.345678]
姿态 (RPY弧度): [0.012345, -0.023456, 0.034567]
姿态 (RPY度数): [0.707°, -1.344°, 1.980°]
四元数 (XYZW): [0.006172, -0.011728, 0.017284, 0.999756]

=== 姿态估计完成 ===
配准结果点云已保存到当前目录
```

## 注意事项

1. **内存使用**：ICP精配准需要更多GPU/CPU内存
2. **计算时间**：精配准会增加整体计算时间
3. **参数调优**：根据具体场景调整ICP参数以获得最佳效果
4. **文件管理**：定期清理保存的点云文件以节省磁盘空间

## 故障排除

- 如果ICP收敛困难，尝试增加 `icp_distance_threshold`
- 如果配准结果不理想，检查粗配准参数设置
- 如果内存不足，减少点云采样点数或使用CPU模式 