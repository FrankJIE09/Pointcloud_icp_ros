#!/usr/bin/env python3
"""
ROS姿态估计服务客户端示例
展示如何调用姿态估计服务
"""

import rospy
import sys
from std_srvs.srv import Empty, EmptyRequest
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped
import numpy as np
from scipy.spatial.transform import Rotation as R


class PoseEstimationClient:
    def __init__(self):
        rospy.init_node('pose_estimation_client')
        
        # 等待服务可用
        rospy.loginfo("等待姿态估计服务...")
        rospy.wait_for_service('estimate_pose')
        
        # 创建服务代理
        self.estimate_pose_service = rospy.ServiceProxy('estimate_pose', Empty)
        
        # TF监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        rospy.loginfo("姿态估计客户端准备就绪")

    def call_pose_estimation(self, input_file=None, model_file=None, method=None):
        """
        调用姿态估计服务
        
        Args:
            input_file: 输入点云文件路径
            model_file: 目标模型文件路径  
            method: 估计方法 ('direct' 或 'semantic_clustering')
        """
        try:
            # 设置参数（如果提供的话）
            if input_file:
                rospy.set_param('/pose_estimation_service/input_point_cloud_file', input_file)
                rospy.loginfo(f"设置输入点云文件: {input_file}")
            
            if model_file:
                rospy.set_param('/pose_estimation_service/model_file', model_file)
                rospy.loginfo(f"设置目标模型文件: {model_file}")
            
            # 注意：当前版本只支持直接配准方法
            if method and method != 'direct':
                rospy.logwarn(f"当前版本只支持'direct'方法，忽略参数: {method}")
            # 总是使用direct方法
            rospy.loginfo("使用直接配准方法")
            
            rospy.loginfo("调用姿态估计服务...")
            
            # 调用服务
            request = EmptyRequest()
            response = self.estimate_pose_service(request)
            
            rospy.loginfo("姿态估计服务调用成功")
            
            # 等待一段时间让TF传播
            rospy.sleep(1.0)
            
            # 获取估计的变换
            self.get_estimated_transform()
            
            return True
            
        except rospy.ServiceException as e:
            rospy.logerr(f"服务调用失败: {e}")
            return False

    def get_estimated_transform(self, target_frame='map', source_frame='estimated_pose'):
        """
        获取估计的TF变换
        """
        try:
            # 获取最新的变换
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rospy.Time(0), rospy.Duration(5.0)
            )
            
            # 提取位置和旋转
            trans = transform.transform.translation
            rot = transform.transform.rotation
            
            # 转换为numpy格式
            xyz = np.array([trans.x, trans.y, trans.z])
            quat = np.array([rot.x, rot.y, rot.z, rot.w])
            
            # 转换为欧拉角
            r = R.from_quat(quat)
            rpy = r.as_euler('xyz', degrees=False)
            rpy_deg = np.degrees(rpy)
            
            # 构造4x4变换矩阵
            transformation_matrix = np.eye(4)
            transformation_matrix[:3, :3] = r.as_matrix()
            transformation_matrix[:3, 3] = xyz
            
            # 打印结果
            rospy.loginfo("=== 获取到的姿态估计结果 ===")
            rospy.loginfo(f"TF: {source_frame} -> {target_frame}")
            rospy.loginfo(f"位置 (XYZ): [{xyz[0]:.6f}, {xyz[1]:.6f}, {xyz[2]:.6f}]")
            rospy.loginfo(f"姿态 (RPY弧度): [{rpy[0]:.6f}, {rpy[1]:.6f}, {rpy[2]:.6f}]")
            rospy.loginfo(f"姿态 (RPY度数): [{rpy_deg[0]:.3f}°, {rpy_deg[1]:.3f}°, {rpy_deg[2]:.3f}°]")
            rospy.loginfo(f"四元数 (XYZW): [{quat[0]:.6f}, {quat[1]:.6f}, {quat[2]:.6f}, {quat[3]:.6f}]")
            
            print("\n4x4变换矩阵:")
            print(transformation_matrix)
            
            return transformation_matrix
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"获取TF变换失败: {e}")
            return None

    def run_interactive_mode(self):
        """
        交互模式，允许用户输入参数并调用服务
        """
        rospy.loginfo("进入交互模式...")
        
        while not rospy.is_shutdown():
            print("\n=== 姿态估计服务客户端 ===")
            print("1. 使用默认参数调用服务")
            print("2. 使用自定义参数调用服务")
            print("3. 仅获取当前TF变换")
            print("4. 退出")
            
            try:
                choice = input("请选择操作 (1-4): ").strip()
                
                if choice == '1':
                    rospy.loginfo("使用默认参数调用服务...")
                    self.call_pose_estimation()
                    
                elif choice == '2':
                    print("\n请输入参数 (留空使用当前设置):")
                    input_file = input("输入点云文件路径: ").strip()
                    model_file = input("目标模型文件路径: ").strip()
                    method = input("估计方法 (仅支持direct): ").strip()
                    
                    self.call_pose_estimation(
                        input_file if input_file else None,
                        model_file if model_file else None,
                        method if method else None
                    )
                    
                elif choice == '3':
                    rospy.loginfo("获取当前TF变换...")
                    self.get_estimated_transform()
                    
                elif choice == '4':
                    rospy.loginfo("退出客户端")
                    break
                    
                else:
                    print("无效选择，请重试")
                    
            except KeyboardInterrupt:
                rospy.loginfo("收到中断信号，退出...")
                break
            except EOFError:
                rospy.loginfo("收到EOF，退出...")
                break


def main():
    """主函数"""
    if len(sys.argv) > 1:
        # 命令行模式
        client = PoseEstimationClient()
        
        # 解析命令行参数
        input_file = sys.argv[1] if len(sys.argv) > 1 else None
        model_file = sys.argv[2] if len(sys.argv) > 2 else None
        method = sys.argv[3] if len(sys.argv) > 3 else None
        
        rospy.loginfo("命令行模式")
        success = client.call_pose_estimation(input_file, model_file, method)
        
        if success:
            rospy.loginfo("姿态估计完成")
        else:
            rospy.logerr("姿态估计失败")
            sys.exit(1)
    else:
        # 交互模式
        try:
            client = PoseEstimationClient()
            client.run_interactive_mode()
        except rospy.ROSInterruptException:
            rospy.loginfo("ROS节点被中断")


if __name__ == '__main__':
    main() 