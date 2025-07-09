# ROS姿态估计系统使用指南

这是一个基于ROS的点云姿态估计服务，使用直接配准方法实现3D物体姿态估计。

## 🛠️ 环境安装

### 方法1：使用conda环境文件（推荐）

1. **创建conda环境**
```bash
conda env create -f pose_estimation_ros_environment.yml
```

2. **激活环境**
```bash
conda activate pose_estimation_ros
```

3. **设置ROS环境**
```bash
source /opt/ros/noetic/setup.bash
```

### 方法2：手动安装

1. **创建conda环境**
```bash
conda create -n pose_estimation_ros python=3.8 -y
conda activate pose_estimation_ros
```

2. **安装PyTorch (CUDA版本)**
```bash
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
```

3. **安装其他依赖**
```bash
pip install open3d scipy pyyaml numpy scikit-learn
pip install rospkg catkin_pkg
conda install -c conda-forge python-orocos-kdl -y
```

4. **设置ROS环境**
```bash
source /opt/ros/noetic/setup.bash
```

## 📁 项目文件结构

```
Pointcloud_icp_sage/
├── pose_estimation_ros_service.py      # 主服务程序
├── pose_estimation_ros_config.yaml     # 配置文件
├── pose_estimation_ros_environment.yml # conda环境文件
├── data/
│   └── test.pcd                        # 测试点云数据
└── stp/
    └── part2_rude.STL                  # 目标CAD模型
```

## 🚀 启动服务

### 第一步：启动ROS核心
在终端1中启动：
```bash
source /opt/ros/noetic/setup.bash
roscore
```

### 第二步：启动姿态估计服务
在终端2中启动：
```bash
conda activate pose_estimation_ros
source /opt/ros/noetic/setup.bash
cd /path/to/Pointcloud_icp_sage
python pose_estimation_ros_service.py
```

成功启动会看到：
```
[INFO] [时间戳]: Loaded configuration from pose_estimation_ros_config.yaml
[INFO] [时间戳]: Using device: cuda
[INFO] [时间戳]: Using direct alignment method
[INFO] [时间戳]: Pose estimation service ready
[INFO] [时间戳]: Pose estimation service is running...
```

## 📞 调用服务

### 方法1：命令行调用（快速测试）

```bash
# 新开终端3
conda activate pose_estimation_ros
source /opt/ros/noetic/setup.bash
rosservice call /estimate_pose "{}"
```

### 方法2：设置参数后调用

```bash
# 设置自定义参数
rosparam set /pose_estimation_service/input_point_cloud_file "data/your_scene.pcd"
rosparam set /pose_estimation_service/model_file "stp/your_model.STL"

# 调用服务
rosservice call /estimate_pose "{}"
```

### 方法3：Python代码调用

```python
#!/usr/bin/env python3
import rospy
from std_srvs.srv import Empty

def call_pose_estimation():
    rospy.init_node('pose_estimation_client')
    
    # 设置参数（可选）
    rospy.set_param('/pose_estimation_service/input_point_cloud_file', 'data/test.pcd')
    rospy.set_param('/pose_estimation_service/model_file', 'stp/part2_rude.STL')
    
    # 等待服务可用
    rospy.wait_for_service('/estimate_pose')
    
    try:
        # 调用服务
        estimate_pose = rospy.ServiceProxy('/estimate_pose', Empty)
        response = estimate_pose()
        print("姿态估计完成！")
        
    except rospy.ServiceException as e:
        print(f"服务调用失败: {e}")

if __name__ == '__main__':
    call_pose_estimation()
```

## 📊 查看结果

### 查看TF变换
```bash
# 查看变换树
rosrun tf tf_monitor

# 查看具体变换（map -> estimated_pose）
rosrun tf tf_echo map estimated_pose

# 一次性查看
rosrun tf tf_echo map estimated_pose -n 1
```

### 查看服务状态
```bash
# 查看可用服务
rosservice list | grep estimate

# 查看运行节点
rosnode list

# 查看服务信息
rosservice info /estimate_pose
```

## ⚙️ 配置参数

主要配置在 `pose_estimation_ros_config.yaml` 文件中：

```yaml
# 输入输出配置
input_point_cloud_file: 'data/test.pcd'          # 输入点云文件
model_file: 'stp/part2_rude.STL'               # 目标CAD模型

# 目标模型采样
model_sample_points: 20480                     # CAD模型采样点数

# 点云预处理
preprocess_voxel_size: 0.0                     # 体素下采样（0禁用）
preprocess_fps_n_points: 2048                  # 最远点采样点数

# 配准参数
coarse_iterations: 200                         # 粗配准迭代次数
coarse_lr: 0.01                               # 学习率
coarse_align_scene_points: 1024               # 用于配准的场景点数
coarse_align_model_points: 1024               # 用于配准的模型点数

# 系统配置
no_cuda: false                                 # 是否禁用CUDA

# TF发布
target_frame: 'map'                            # 父坐标系
estimated_frame: 'estimated_pose'              # 估计坐标系
```

### 动态参数设置
也可以通过ROS参数服务器动态设置：
```bash
rosparam set /pose_estimation_service/coarse_iterations 300
rosparam set /pose_estimation_service/coarse_lr 0.005
```

## 📈 输出结果

成功运行后，控制台会显示：
```
=== 姿态估计结果 ===
位置 (XYZ): [-902.144589, -68.280014, -195.150906]
姿态 (RPY弧度): [0.094322, -0.077401, 0.002517]
姿态 (RPY度数): [5.404°, -4.435°, 0.144°]
四元数 (XYZW): [0.047084, -0.038678, 0.001259, 0.998139]
```

同时会发布TF变换：`map` → `estimated_pose`

## 🐛 故障排除

### 1. 服务启动失败

**问题**: ImportError 或模块找不到
```bash
# 解决方法：确认环境激活
conda activate pose_estimation_ros
source /opt/ros/noetic/setup.bash

# 验证依赖
python -c "import torch, open3d, rospy; print('依赖正常')"
```

**问题**: CUDA相关错误
```bash
# 检查CUDA状态
python -c "import torch; print('CUDA可用:', torch.cuda.is_available())"

# 如果CUDA不可用，修改配置文件
# no_cuda: true
```

### 2. 文件路径错误

**问题**: 点云文件或模型文件找不到
```bash
# 检查文件是否存在
ls -la data/test.pcd
ls -la stp/part2_rude.STL

# 使用绝对路径
rosparam set /pose_estimation_service/input_point_cloud_file "/完整路径/data/test.pcd"
```

### 3. ROS连接问题

**问题**: 无法连接到ROS master
```bash
# 检查roscore是否运行
rosnode list

# 检查ROS环境变量
echo $ROS_MASTER_URI

# 重启roscore
pkill -f roscore
roscore
```

### 4. 服务调用超时

**问题**: 服务调用没有响应
```bash
# 检查服务状态
rosservice list | grep estimate

# 检查节点状态
rosnode info /pose_estimation_service

# 监控CPU使用（配准过程会占用较高CPU）
htop
```

### 5. 内存不足

**问题**: 大点云处理时内存不足
```yaml
# 在配置文件中减少点数
model_sample_points: 10240      # 减少模型采样点
preprocess_fps_n_points: 1024   # 减少预处理点数
coarse_align_scene_points: 512  # 减少配准用点数
```

## 📝 支持的文件格式

### 输入点云格式
- **PCD**: Point Cloud Data格式
- **PLY**: Polygon格式  
- **TXT**: 纯文本格式（XYZ或XYZRGB，空格或逗号分隔）

### 目标模型格式
- **STL**: 立体光刻格式
- **PLY**: Polygon格式
- **OBJ**: Wavefront OBJ格式

## 🔧 性能优化建议

1. **GPU加速**: 确保使用CUDA版本的PyTorch
2. **点云预处理**: 适当的下采样可提高速度
3. **配准参数**: 根据精度需求调整迭代次数
4. **内存管理**: 大模型时适当减少采样点数

## 📚 API参考

### ROS服务
- **服务名**: `/estimate_pose`
- **服务类型**: `std_srvs/Empty`
- **输入**: 无（参数通过ROS参数服务器设置）
- **输出**: 无（结果通过TF发布）

### ROS参数
- `input_point_cloud_file`: 输入点云文件路径
- `model_file`: 目标模型文件路径
- `target_frame`: TF父坐标系名称
- `estimated_frame`: TF子坐标系名称
- 其他配准参数见配置文件

### TF发布
- **父坐标系**: `map` (可配置)
- **子坐标系**: `estimated_pose` (可配置)
- **更新频率**: 按需更新（服务调用时）

---

## 📧 技术支持

如遇问题，请检查：
1. conda环境是否正确激活
2. ROS环境是否正确设置
3. 文件路径是否正确
4. CUDA环境是否可用

创建时间：2025年1月9日 