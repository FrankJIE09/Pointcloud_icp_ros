#!/usr/bin/env python3
"""
ROS服务：点云姿态估计
整合直接粗配准
"""

import rospy
import sys
import os
import datetime
import copy
import numpy as np
import torch
import yaml
import json
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion
from std_srvs.srv import Empty, EmptyResponse
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from scipy.spatial.transform import Rotation as R

# 自定义服务消息类型需要先定义，这里用字典作为替代
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

# --- 导入点云处理库 ---
try:
    import open3d as o3d
    OPEN3D_AVAILABLE = True
except ImportError:
    rospy.logerr("Open3D not found. Please install Open3D")
    sys.exit(1)

# 删除了语义分割相关的导入

# --- PyTorch相关导入 ---
import torch.nn as nn
import torch.optim as optim


class PoseEstimationService:
    def __init__(self):
        rospy.init_node('pose_estimation_service')
        
        # 加载配置文件
        config_file = rospy.get_param('~config_file', 'pose_estimation_ros_config.yaml')
        self.config = self.load_config(config_file)
        
        # 初始化设备
        self.device = torch.device("cuda" if torch.cuda.is_available() and not self.config.get('no_cuda', False) else "cpu")
        rospy.loginfo(f"Using device: {self.device}")
        
        # 直接配准方法，无需语义分割模型
        rospy.loginfo("Using direct alignment method")
        
        # TF广播器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # 创建服务
        self.service = rospy.Service('estimate_pose', Empty, self.estimate_pose_callback)
        rospy.loginfo("Pose estimation service ready")

    def load_config(self, config_path):
        """加载YAML配置文件"""
        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
            rospy.loginfo(f"Loaded configuration from {config_path}")
            return config
        except Exception as e:
            rospy.logwarn(f"Could not load config {config_path}: {e}. Using defaults.")
            return {}



    def normalize_point_cloud_xyz(self, points_xyz):
        """XYZ坐标归一化"""
        if points_xyz.shape[0] == 0:
            return points_xyz
        centroid = np.mean(points_xyz, axis=0)
        points_centered = points_xyz - centroid
        max_dist = np.max(np.sqrt(np.sum(points_centered ** 2, axis=1)))
        if max_dist < 1e-6:
            return points_centered
        points_normalized = points_centered / max_dist
        return points_normalized.astype(np.float32)

    def axis_angle_to_matrix(self, axis_angle):
        """轴角向量转旋转矩阵"""
        theta = torch.norm(axis_angle)
        if theta < 1e-6:
            return torch.eye(3, device=axis_angle.device, dtype=axis_angle.dtype)
        
        k = axis_angle / theta
        K = torch.as_tensor([
            [0, -k[2], k[1]],
            [k[2], 0, -k[0]],
            [-k[1], k[0], 0]
        ], dtype=axis_angle.dtype, device=axis_angle.device)
        
        R = torch.eye(3, device=axis_angle.device, dtype=axis_angle.dtype) + \
            torch.sin(theta) * K + (1 - torch.cos(theta)) * (K @ K)
        return R

    def pytorch_coarse_alignment(self, source_points, target_points, iterations=200, lr=0.01):
        """PyTorch粗配准"""
        rospy.loginfo("Running PyTorch coarse alignment...")
        
        source_tensor = torch.from_numpy(source_points.astype(np.float32)).to(self.device)
        target_tensor = torch.from_numpy(target_points.astype(np.float32)).to(self.device)
        
        rot_vec = nn.Parameter(torch.zeros(3, device=self.device))
        translation = nn.Parameter(torch.zeros(3, device=self.device))
        optimizer = optim.Adam([rot_vec, translation], lr=lr)
        
        for i in range(iterations):
            optimizer.zero_grad()
            R_matrix = self.axis_angle_to_matrix(rot_vec)
            transformed_source = (R_matrix @ source_tensor.T).T + translation
            dists = torch.cdist(transformed_source, target_tensor)
            loss = torch.min(dists, dim=1)[0].mean()
            loss.backward()
            optimizer.step()
            
            if (i + 1) % 50 == 0:
                rospy.loginfo(f"Iteration {i+1}/{iterations}, Loss: {loss.item():.6f}")
        
        final_R = self.axis_angle_to_matrix(rot_vec).detach().cpu().numpy()
        final_t = translation.detach().cpu().numpy()
        
        transform = np.identity(4)
        transform[:3, :3] = final_R
        transform[:3, 3] = final_t
        
        return transform

    def find_nearest_neighbors_torch(self, source_points, target_points):
        """查找最近邻对应点"""
        dist_sq = torch.cdist(source_points, target_points).pow(2)
        min_dist_sq, indices = torch.min(dist_sq, dim=1)
        return indices, min_dist_sq

    def estimate_point_to_point_svd_torch(self, P, Q):
        """点到点SVD变换估计"""
        centroid_P = torch.mean(P, dim=0, keepdim=True)
        centroid_Q = torch.mean(Q, dim=0, keepdim=True)
        P_centered = P - centroid_P
        Q_centered = Q - centroid_Q
        H = P_centered.T @ Q_centered

        try:
            U, S, Vt = torch.linalg.svd(H)
        except torch.linalg.LinAlgError as e:
            rospy.logwarn(f"SVD failed: {e}. Returning identity transform.")
            T = torch.eye(4, device=P.device, dtype=P.dtype)
            return T

        R = Vt.T @ U.T
        if torch.linalg.det(R) < 0:
            Vt_copy = Vt.clone()
            Vt_copy[2, :] *= -1
            R = Vt_copy.T @ U.T

        t = centroid_Q.T - R @ centroid_P.T

        T = torch.eye(4, device=P.device, dtype=P.dtype)
        T[:3, :3] = R
        T[:3, 3] = t.squeeze()
        return T

    def estimate_point_to_plane_least_squares_torch(self, P_transformed, Q_corr, Q_normals_corr, device):
        """点到平面最小二乘变换估计"""
        K = P_transformed.shape[0]
        A = torch.zeros(K, 6, device=device, dtype=P_transformed.dtype)
        b = torch.zeros(K, device=device, dtype=P_transformed.dtype)

        cross_prod_P_n = torch.cross(P_transformed, Q_normals_corr, dim=1)

        A[:, 0] = cross_prod_P_n[:, 0]  # alpha
        A[:, 1] = cross_prod_P_n[:, 1]  # beta  
        A[:, 2] = cross_prod_P_n[:, 2]  # gamma

        A[:, 3] = Q_normals_corr[:, 0]  # tx
        A[:, 4] = Q_normals_corr[:, 1]  # ty
        A[:, 5] = Q_normals_corr[:, 2]  # tz

        b = torch.sum((Q_corr - P_transformed) * Q_normals_corr, dim=1)

        try:
            x = torch.linalg.lstsq(A, b).solution
        except torch.linalg.LinAlgError as e:
            rospy.logwarn(f"Point-to-plane lstsq failed: {e}. Returning identity delta transform.")
            return torch.eye(4, device=device, dtype=P_transformed.dtype)

        omega_vec = x[:3]
        angle = torch.norm(omega_vec)
        if angle > 1e-9:
            axis = omega_vec / angle
            K_axis = torch.zeros((3, 3), device=device, dtype=P_transformed.dtype)
            K_axis[0, 1] = -axis[2]
            K_axis[0, 2] = axis[1]
            K_axis[1, 0] = axis[2]
            K_axis[1, 2] = -axis[0]
            K_axis[2, 0] = -axis[1]
            K_axis[2, 1] = axis[0]
            R_delta = torch.eye(3, device=device, dtype=P_transformed.dtype) + \
                      torch.sin(angle) * K_axis + \
                      (1 - torch.cos(angle)) * (K_axis @ K_axis)
        else:
            R_delta = torch.eye(3, device=device, dtype=P_transformed.dtype)

        delta_transform = torch.eye(4, device=device, dtype=P_transformed.dtype)
        delta_transform[:3, :3] = R_delta
        delta_transform[:3, 3] = x[3:]

        return delta_transform

    def pytorch_icp_registration(self, source_points_tensor, target_points_tensor, 
                               initial_transform_guess, max_iterations=100,
                               distance_threshold=0.1, rmse_change_threshold=1e-6,
                               transform_change_threshold=1e-5, 
                               estimation_method='point_to_point',
                               target_normals_tensor=None):
        """PyTorch ICP精配准"""
        rospy.loginfo(f"Running PyTorch ICP ({estimation_method})...")
        
        if estimation_method == 'point_to_plane' and target_normals_tensor is None:
            raise ValueError("Point-to-plane ICP requires target normals")

        current_transform = initial_transform_guess.clone().to(device=self.device, dtype=source_points_tensor.dtype)
        source_homo = torch.cat([source_points_tensor, torch.ones(source_points_tensor.shape[0], 1, 
                                                                 device=self.device, dtype=source_points_tensor.dtype)], dim=1).T

        prev_rmse = float('inf')
        prev_fitness = 0.0

        for i in range(max_iterations):
            transformed_source_points = (current_transform @ source_homo)[:3, :].T

            # 查找对应点
            corr_indices_target, dist_sq = self.find_nearest_neighbors_torch(transformed_source_points, target_points_tensor)

            # 过滤对应点
            valid_mask = dist_sq < (distance_threshold ** 2)
            num_correspondences = valid_mask.sum().item()

            if num_correspondences < 10:
                rospy.logwarn(f"ICP iter {i + 1}: Too few correspondences ({num_correspondences}). Stopping.")
                break

            P_corr_transformed = transformed_source_points[valid_mask]
            Q_corr_target = target_points_tensor[corr_indices_target[valid_mask]]

            current_rmse = torch.sqrt(torch.mean(dist_sq[valid_mask])).item()
            fitness = num_correspondences / source_points_tensor.shape[0]

            transform_update_matrix = torch.eye(4, device=self.device, dtype=source_points_tensor.dtype)
            
            if estimation_method == 'point_to_point':
                P_orig_corr = source_points_tensor[valid_mask]
                new_total_transform = self.estimate_point_to_point_svd_torch(P_orig_corr, Q_corr_target)
                transform_update_matrix = new_total_transform @ torch.linalg.inv(current_transform)
                current_transform = new_total_transform
            elif estimation_method == 'point_to_plane':
                Q_normals_corr = target_normals_tensor[corr_indices_target[valid_mask]]
                transform_update_matrix = self.estimate_point_to_plane_least_squares_torch(
                    P_corr_transformed, Q_corr_target, Q_normals_corr, self.device)
                current_transform = transform_update_matrix @ current_transform

            # 检查收敛
            delta_transform_norm = torch.norm(
                transform_update_matrix - torch.eye(4, device=self.device, dtype=source_points_tensor.dtype)).item()
            rmse_diff = abs(prev_rmse - current_rmse)

            if (i + 1) % 10 == 0:
                rospy.loginfo(f"ICP iter {i + 1}/{max_iterations}: RMSE: {current_rmse:.6f}, Fitness: {fitness:.6f}, Corr: {num_correspondences}")

            if i >= 10:  # 至少运行10次迭代
                if rmse_diff < rmse_change_threshold:
                    rospy.loginfo(f"ICP converged at iter {i + 1} due to RMSE change.")
                    break
                if delta_transform_norm < transform_change_threshold:
                    rospy.loginfo(f"ICP converged at iter {i + 1} due to transform change.")
                    break

            prev_rmse = current_rmse
            prev_fitness = fitness

        # 计算最终指标
        final_transformed_source = (current_transform @ source_homo)[:3, :].T
        final_corr_indices, final_dist_sq = self.find_nearest_neighbors_torch(final_transformed_source, target_points_tensor)
        final_valid_mask = final_dist_sq < (distance_threshold ** 2)

        final_num_correspondences = final_valid_mask.sum().item()
        final_fitness = final_num_correspondences / source_points_tensor.shape[0]
        if final_num_correspondences > 0:
            final_inlier_rmse = torch.sqrt(torch.mean(final_dist_sq[final_valid_mask])).item()
        else:
            final_inlier_rmse = float('inf')

        return {
            "transformation": current_transform.cpu().numpy(),
            "fitness": final_fitness,
            "inlier_rmse": final_inlier_rmse,
            "correspondence_set_size": final_num_correspondences
        }

    def load_point_cloud_from_file(self, file_path):
        """从文件加载点云"""
        if not os.path.exists(file_path):
            raise FileNotFoundError(f"Point cloud file not found: {file_path}")
        
        file_extension = os.path.splitext(file_path)[1].lower()
        
        if file_extension == '.txt':
            # 处理TXT文件
            with open(file_path, 'r') as f:
                first_line = f.readline()
            delimiter = ',' if ',' in first_line else None
            data = np.loadtxt(file_path, delimiter=delimiter, dtype=np.float32)
            if data.ndim == 1:
                data = data.reshape(1, -1)
            if data.shape[1] < 3:
                raise ValueError("TXT file must have at least 3 columns for XYZ")
            
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(data[:, :3])
            if data.shape[1] >= 6:
                colors = data[:, 3:6]
                if np.any(colors > 1.0) and np.max(colors) <= 255.0:
                    colors = colors / 255.0
                pcd.colors = o3d.utility.Vector3dVector(np.clip(colors, 0.0, 1.0))
                
        elif file_extension in ['.pcd', '.ply']:
            pcd = o3d.io.read_point_cloud(file_path)
        else:
            raise ValueError(f"Unsupported file extension: {file_extension}")
        
        if not pcd.has_points():
            raise ValueError("Loaded point cloud is empty")
            
        return pcd

    def load_target_model(self, model_file):
        """加载目标CAD模型"""
        if not os.path.exists(model_file):
            raise FileNotFoundError(f"Model file not found: {model_file}")
        
        # 尝试作为网格加载
        mesh = o3d.io.read_triangle_mesh(model_file)
        if mesh.has_vertices():
            sample_points = self.config.get('model_sample_points', 20480)
            target_pcd = mesh.sample_points_uniformly(number_of_points=sample_points)
        else:
            # 作为点云加载
            target_pcd = o3d.io.read_point_cloud(model_file)
        
        if not target_pcd.has_points():
            raise ValueError("Target model is empty")
            
        return target_pcd

    def preprocess_point_cloud(self, pcd, config_prefix='preprocess_'):
        """点云预处理"""
        processed_pcd = copy.deepcopy(pcd)
        
        # Voxel下采样
        voxel_size = self.config.get(f'{config_prefix}voxel_size', 0.0)
        if voxel_size > 0:
            rospy.loginfo(f"Applying voxel downsampling: {voxel_size}")
            processed_pcd = processed_pcd.voxel_down_sample(voxel_size)
        
        # 统计离群点移除
        sor_k = self.config.get(f'{config_prefix}sor_k', 0)
        sor_std_ratio = self.config.get(f'{config_prefix}sor_std_ratio', 1.0)
        if sor_k > 0:
            rospy.loginfo(f"Applying SOR: k={sor_k}, std_ratio={sor_std_ratio}")
            processed_pcd, _ = processed_pcd.remove_statistical_outlier(sor_k, sor_std_ratio)
        
        # 最远点采样
        fps_points = self.config.get(f'{config_prefix}fps_n_points', 0)
        if fps_points > 0 and len(processed_pcd.points) > fps_points:
            rospy.loginfo(f"Applying FPS to {fps_points} points")
            processed_pcd = processed_pcd.farthest_point_down_sample(fps_points)
        
        return processed_pcd



    def direct_alignment_method(self, scene_pcd, target_pcd):
        """直接对齐方法 - 包含粗配准和ICP精配准"""
        rospy.loginfo("Using direct alignment method")
        
        # 预处理场景点云
        scene_preprocessed = self.preprocess_point_cloud(scene_pcd)
        
        # 中心化点云
        scene_centroid = scene_preprocessed.get_center()
        scene_centered = copy.deepcopy(scene_preprocessed).translate(-scene_centroid)
        
        target_centroid = target_pcd.get_center()
        target_centered = copy.deepcopy(target_pcd).translate(-target_centroid)
        
        # === 步骤1：粗配准 ===
        rospy.loginfo("=== 步骤1：粗配准 ===")
        
        # 对齐点数下采样
        coarse_scene_points = self.config.get('coarse_align_scene_points', 1024)
        coarse_model_points = self.config.get('coarse_align_model_points', 1024)
        
        source_for_align = scene_centered
        target_for_align = target_centered
        
        if coarse_scene_points > 0 and len(source_for_align.points) > coarse_scene_points:
            source_for_align = source_for_align.farthest_point_down_sample(coarse_scene_points)
        
        if coarse_model_points > 0 and len(target_for_align.points) > coarse_model_points:
            target_for_align = target_for_align.farthest_point_down_sample(coarse_model_points)
        
        # 粗配准
        coarse_iterations = self.config.get('coarse_iterations', 200)
        coarse_lr = self.config.get('coarse_lr', 0.01)
        
        coarse_transform = self.pytorch_coarse_alignment(
            np.asarray(source_for_align.points),
            np.asarray(target_for_align.points),
            iterations=coarse_iterations,
            lr=coarse_lr
        )
        
        rospy.loginfo("粗配准完成")
        
        # === 步骤2：ICP精配准 ===
        use_icp = self.config.get('use_icp', True)
        if use_icp:
            rospy.loginfo("=== 步骤2：ICP精配准 ===")
            
            # ICP参数
            icp_max_iterations = self.config.get('icp_max_iterations', 100)
            icp_distance_threshold = self.config.get('icp_distance_threshold', 0.1)
            icp_rmse_threshold = self.config.get('icp_rmse_threshold', 1e-6)
            icp_transform_threshold = self.config.get('icp_transform_threshold', 1e-5)
            icp_estimation_method = self.config.get('icp_estimation_method', 'point_to_point')
            
            # 转换为tensor
            source_points_tensor = torch.from_numpy(np.asarray(scene_centered.points).astype(np.float32)).to(self.device)
            target_points_tensor = torch.from_numpy(np.asarray(target_centered.points).astype(np.float32)).to(self.device)
            initial_transform_tensor = torch.from_numpy(coarse_transform.astype(np.float32)).to(self.device)
            
            # 如果使用点到平面ICP，计算法向量
            target_normals_tensor = None
            if icp_estimation_method == 'point_to_plane':
                rospy.loginfo("计算目标点云法向量...")
                target_centered.estimate_normals(
                    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=icp_distance_threshold * 2.0, max_nn=30))
                if target_centered.has_normals():
                    target_normals_tensor = torch.from_numpy(np.asarray(target_centered.normals).astype(np.float32)).to(self.device)
                    rospy.loginfo("法向量计算完成")
                else:
                    rospy.logwarn("法向量计算失败，使用点到点ICP")
                    icp_estimation_method = 'point_to_point'
            
            # 运行ICP
            icp_result = self.pytorch_icp_registration(
                source_points_tensor=source_points_tensor,
                target_points_tensor=target_points_tensor,
                initial_transform_guess=initial_transform_tensor,
                max_iterations=icp_max_iterations,
                distance_threshold=icp_distance_threshold,
                rmse_change_threshold=icp_rmse_threshold,
                transform_change_threshold=icp_transform_threshold,
                estimation_method=icp_estimation_method,
                target_normals_tensor=target_normals_tensor
            )
            
            # 使用ICP结果
            icp_transform = icp_result["transformation"]
            rospy.loginfo(f"ICP完成 - Fitness: {icp_result['fitness']:.6f}, RMSE: {icp_result['inlier_rmse']:.6f}")
            
            # 保存配准结果点云
            self.save_registration_result(scene_centered, target_centered, icp_transform, scene_centroid, target_centroid)
            
        else:
            rospy.loginfo("跳过ICP精配准")
            icp_transform = coarse_transform
        
        # 计算最终变换
        T_scene_to_origin = np.identity(4)
        T_scene_to_origin[:3, 3] = -scene_centroid
        
        T_origin_to_target = np.identity(4)
        T_origin_to_target[:3, 3] = target_centroid
        
        final_transform = T_origin_to_target @ icp_transform @ T_scene_to_origin
        
        return final_transform

    def save_registration_result(self, scene_centered, target_centered, transform_matrix, scene_centroid, target_centroid):
        """保存配准结果点云 - 支持多种格式和配置控制"""
        # 检查是否启用保存
        if not self.config.get('save_registration_results', True):
            rospy.loginfo("配准结果保存已禁用")
            return
            
        try:
            # 获取保存配置
            output_dir = self.config.get('registration_output_dir', './registration_results')
            save_formats = self.config.get('save_formats', ['pcd'])
            save_scene = self.config.get('save_scene_cloud', True)
            save_target = self.config.get('save_target_cloud', True)
            save_combined = self.config.get('save_combined_cloud', True)
            add_timestamp = self.config.get('add_timestamp_to_filename', True)
            
            # 创建输出目录
            os.makedirs(output_dir, exist_ok=True)
            
            # 生成时间戳
            timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S') if add_timestamp else ""
            timestamp_suffix = f"_{timestamp}" if timestamp else ""
            
            # 应用变换到场景点云
            scene_transformed = copy.deepcopy(scene_centered)
            scene_transformed.transform(transform_matrix)
            
            # 恢复到原始坐标系
            scene_transformed.translate(target_centroid)
            target_result = copy.deepcopy(target_centered)
            target_result.translate(target_centroid)
            
            # 设置颜色
            scene_transformed.paint_uniform_color([1, 0.706, 0])  # 黄色
            target_result.paint_uniform_color([0, 0.651, 0.929])  # 蓝色
            
            saved_files = []
            
            # 保存配准后的场景点云
            if save_scene:
                for fmt in save_formats:
                    scene_filename = f"registration_result_scene{timestamp_suffix}.{fmt}"
                    scene_filepath = os.path.join(output_dir, scene_filename)
                    
                    if self.save_point_cloud_with_format(scene_transformed, scene_filepath, fmt):
                        saved_files.append(scene_filepath)
                        rospy.loginfo(f"保存配准后场景点云: {scene_filepath}")
            
            # 保存目标模型点云
            if save_target:
                for fmt in save_formats:
                    target_filename = f"registration_result_target{timestamp_suffix}.{fmt}"
                    target_filepath = os.path.join(output_dir, target_filename)
                    
                    if self.save_point_cloud_with_format(target_result, target_filepath, fmt):
                        saved_files.append(target_filepath)
                        rospy.loginfo(f"保存目标模型点云: {target_filepath}")
            
            # 保存合并点云
            if save_combined:
                combined_pcd = scene_transformed + target_result
                for fmt in save_formats:
                    combined_filename = f"registration_result_combined{timestamp_suffix}.{fmt}"
                    combined_filepath = os.path.join(output_dir, combined_filename)
                    
                    if self.save_point_cloud_with_format(combined_pcd, combined_filepath, fmt):
                        saved_files.append(combined_filepath)
                        rospy.loginfo(f"保存合并点云: {combined_filepath}")
            
            rospy.loginfo(f"共保存 {len(saved_files)} 个配准结果文件到目录: {output_dir}")
            
        except Exception as e:
            rospy.logerr(f"保存配准结果失败: {e}")
            import traceback
            traceback.print_exc()

    def save_point_cloud_with_format(self, pcd, filepath, format_type):
        """根据格式保存点云文件"""
        try:
            format_type = format_type.lower()
            
            if format_type == 'pcd':
                return o3d.io.write_point_cloud(filepath, pcd)
            elif format_type == 'ply':
                return o3d.io.write_point_cloud(filepath, pcd)
            elif format_type == 'txt':
                return self.save_point_cloud_as_txt(pcd, filepath)
            else:
                rospy.logwarn(f"不支持的点云格式: {format_type}")
                return False
                
        except Exception as e:
            rospy.logerr(f"保存点云文件失败 {filepath}: {e}")
            return False

    def save_point_cloud_as_txt(self, pcd, filepath):
        """将点云保存为TXT格式 (X Y Z R G B)"""
        try:
            points = np.asarray(pcd.points)
            has_colors = pcd.has_colors()
            
            if has_colors:
                colors = np.asarray(pcd.colors)
                # 将颜色从[0,1]范围转换为[0,255]范围
                colors_255 = (colors * 255).astype(np.uint8)
                data = np.hstack([points, colors_255])
                header = "X Y Z R G B"
                fmt = "%.6f %.6f %.6f %d %d %d"
            else:
                data = points
                header = "X Y Z"
                fmt = "%.6f %.6f %.6f"
            
            np.savetxt(filepath, data, fmt=fmt, header=header, comments='')
            return True
            
        except Exception as e:
            rospy.logerr(f"保存TXT格式点云失败: {e}")
            return False

    def matrix_to_xyz_rpy(self, transformation_matrix):
        """从4x4变换矩阵提取xyz位置和rpy欧拉角"""
        xyz = transformation_matrix[:3, 3]
        rotation_matrix = transformation_matrix[:3, :3]
        r = R.from_matrix(rotation_matrix)
        rpy = r.as_euler('xyz', degrees=False)
        return xyz, rpy

    def matrix_to_quaternion(self, transformation_matrix):
        """从4x4变换矩阵提取位置和四元数"""
        xyz = transformation_matrix[:3, 3]
        rotation_matrix = transformation_matrix[:3, :3]
        r = R.from_matrix(rotation_matrix)
        quat = r.as_quat()  # [x, y, z, w]
        return xyz, quat

    def publish_tf(self, transformation_matrix, frame_id="map", child_frame_id="estimated_pose"):
        """发布TF变换"""
        xyz, quat = self.matrix_to_quaternion(transformation_matrix)
        
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = frame_id
        t.child_frame_id = child_frame_id
        
        t.transform.translation.x = xyz[0]
        t.transform.translation.y = xyz[1] 
        t.transform.translation.z = xyz[2]
        
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        
        self.tf_broadcaster.sendTransform(t)
        
        # 打印结果
        xyz_print, rpy_print = self.matrix_to_xyz_rpy(transformation_matrix)
        rospy.loginfo("=== 姿态估计结果 ===")
        rospy.loginfo(f"位置 (XYZ): [{xyz_print[0]:.6f}, {xyz_print[1]:.6f}, {xyz_print[2]:.6f}]")
        rospy.loginfo(f"姿态 (RPY弧度): [{rpy_print[0]:.6f}, {rpy_print[1]:.6f}, {rpy_print[2]:.6f}]")
        rospy.loginfo(f"姿态 (RPY度数): [{np.degrees(rpy_print[0]):.3f}°, {np.degrees(rpy_print[1]):.3f}°, {np.degrees(rpy_print[2]):.3f}°]")
        rospy.loginfo(f"四元数 (XYZW): [{quat[0]:.6f}, {quat[1]:.6f}, {quat[2]:.6f}, {quat[3]:.6f}]")

    def estimate_pose_callback(self, req):
        """服务回调函数"""
        try:
            rospy.loginfo("=== 开始姿态估计 ===")
            
            # 获取参数
            input_point_cloud_file = rospy.get_param('~input_point_cloud_file', 
                                                   self.config.get('input_point_cloud_file', 'data/test.pcd'))
            model_file = rospy.get_param('~model_file', 
                                       self.config.get('model_file', 'stp/part2_rude.STL'))
            
            rospy.loginfo(f"输入点云: {input_point_cloud_file}")
            rospy.loginfo(f"目标模型: {model_file}")
            rospy.loginfo("使用直接配准方法")
            
            # 加载数据
            scene_pcd = self.load_point_cloud_from_file(input_point_cloud_file)
            target_pcd = self.load_target_model(model_file)
            
            rospy.loginfo(f"场景点云: {len(scene_pcd.points)} 点")
            rospy.loginfo(f"目标模型: {len(target_pcd.points)} 点")
            
            # 使用直接配准方法（包含粗配准和ICP精配准）
            final_transform = self.direct_alignment_method(scene_pcd, target_pcd)
            
            # 发布TF
            frame_id = str(rospy.get_param('~target_frame', 'map'))
            child_frame_id = str(rospy.get_param('~estimated_frame', 'estimated_pose'))
            self.publish_tf(final_transform, frame_id, child_frame_id)
            
            rospy.loginfo("=== 姿态估计完成 ===")
            rospy.loginfo("配准结果点云已保存到当前目录")
            return EmptyResponse()
            
        except Exception as e:
            rospy.logerr(f"姿态估计失败: {e}")
            import traceback
            traceback.print_exc()
            return EmptyResponse()


def main():
    try:
        service = PoseEstimationService()
        rospy.loginfo("Pose estimation service is running...")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Service failed: {e}")


if __name__ == '__main__':
    main() 