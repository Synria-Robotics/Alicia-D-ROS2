#!/usr/bin/env python3
"""
标定位姿生成器

根据机械臂工作空间和 ArUco 标记位置，生成适合手眼标定的多个位姿。
确保标记在相机视野内，并有足够的旋转多样性。

作者: Synria Robotics
日期: 2026-01
"""

import numpy as np
from scipy.spatial.transform import Rotation as R


class CalibrationPoseGenerator:
    """标定位姿生成器"""
    
    def __init__(self, marker_distance=0.32, num_poses=20):
        """
        初始化位姿生成器
        
        Args:
            marker_distance: ArUco标记距离base_link的距离 (米)
            num_poses: 要生成的位姿数量
        """
        self.marker_distance = marker_distance
        self.num_poses = num_poses
        
        # 机械臂关节限制 (弧度)
        self.joint_limits = {
            'Joint1': (-2.967, 2.967),   # ±170°
            'Joint2': (-2.094, 2.094),   # ±120°
            'Joint3': (-2.617, 2.617),   # ±150°
            'Joint4': (-2.967, 2.967),   # ±170°
            'Joint5': (-2.094, 2.094),   # ±120°
            'Joint6': (-6.283, 6.283),   # ±360°
        }
    
    def generate_poses_for_eye_in_hand(self):
        """
        为 Eye-in-Hand 配置生成标定位姿
        
        位姿设计原则:
        1. 确保 ArUco 标记在相机视野内
        2. 有足够的旋转多样性 (覆盖多个旋转轴)
        3. 避免奇异点和关节限位
        4. 位姿之间有足够的差异
        
        Returns:
            list: 位姿列表，每个位姿是6个关节角度 (弧度)
        """
        poses = []
        
        # 基础位姿参数 (确保标记在视野内)
        # 这些值针对标记在base_link正前方30-35cm处进行了优化
        
        base_j2 = -0.3   # Joint2 基础值 (轻微前倾)
        base_j3 = 0.6    # Joint3 基础值 (抬起)
        base_j5 = -0.9   # Joint5 基础值 (相机朝向前方)
        
        # 策略1: 主要改变 Joint1 (水平旋转)
        for j1 in np.linspace(-0.25, 0.25, 5):
            poses.append([j1, base_j2, base_j3, 0.0, base_j5, 0.0])
        
        # 策略2: 主要改变 Joint5 (俯仰角)
        for j5 in np.linspace(-0.7, -1.1, 4):
            poses.append([0.0, base_j2, base_j3, 0.0, j5, 0.0])
        
        # 策略3: 主要改变 Joint6 (末端旋转)
        for j6 in np.linspace(-0.5, 0.5, 4):
            poses.append([0.0, base_j2, base_j3, 0.0, base_j5, j6])
        
        # 策略4: 改变 Joint2 和 Joint3 (前后/上下)
        for j2, j3 in [(-0.4, 0.7), (-0.2, 0.5), (-0.35, 0.65)]:
            poses.append([0.0, j2, j3, 0.0, base_j5, 0.0])
        
        # 策略5: 组合旋转 (增加多样性)
        combinations = [
            [0.15, -0.35, 0.65, 0.1, -0.85, 0.25],
            [-0.15, -0.25, 0.55, -0.1, -0.95, -0.25],
            [0.2, -0.3, 0.6, 0.15, -0.9, 0.4],
            [-0.2, -0.3, 0.6, -0.15, -0.9, -0.4],
            [0.1, -0.38, 0.68, 0.08, -0.82, 0.3],
            [-0.1, -0.22, 0.52, -0.08, -0.98, -0.3],
        ]
        poses.extend(combinations)
        
        # 确保生成足够的位姿
        if len(poses) < self.num_poses:
            # 添加更多随机组合
            np.random.seed(42)  # 可重复性
            while len(poses) < self.num_poses:
                j1 = np.random.uniform(-0.25, 0.25)
                j2 = np.random.uniform(-0.4, -0.2)
                j3 = np.random.uniform(0.5, 0.7)
                j4 = np.random.uniform(-0.15, 0.15)
                j5 = np.random.uniform(-1.1, -0.7)
                j6 = np.random.uniform(-0.5, 0.5)
                poses.append([j1, j2, j3, j4, j5, j6])
        
        return poses[:self.num_poses]
    
    def validate_pose(self, pose):
        """
        验证位姿是否在关节限制内
        
        Args:
            pose: 6个关节角度 (弧度)
        
        Returns:
            bool: 是否有效
        """
        joint_names = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
        for i, (name, angle) in enumerate(zip(joint_names, pose)):
            min_val, max_val = self.joint_limits[name]
            if angle < min_val or angle > max_val:
                return False
        return True
    
    def filter_valid_poses(self, poses):
        """
        过滤出有效的位姿
        
        Args:
            poses: 位姿列表
        
        Returns:
            list: 有效位姿列表
        """
        return [p for p in poses if self.validate_pose(p)]
    
    def compute_pose_diversity(self, poses):
        """
        计算位姿集合的多样性分数
        
        Args:
            poses: 位姿列表
        
        Returns:
            float: 多样性分数 (越高越好)
        """
        if len(poses) < 2:
            return 0.0
        
        poses_array = np.array(poses)
        
        # 计算所有位姿对之间的距离
        total_distance = 0.0
        count = 0
        for i in range(len(poses)):
            for j in range(i + 1, len(poses)):
                distance = np.linalg.norm(poses_array[i] - poses_array[j])
                total_distance += distance
                count += 1
        
        return total_distance / count if count > 0 else 0.0
    
    def optimize_pose_selection(self, poses, target_count):
        """
        从候选位姿中选择最优的子集
        
        使用贪心算法选择多样性最大的位姿
        
        Args:
            poses: 候选位姿列表
            target_count: 目标数量
        
        Returns:
            list: 优化后的位姿列表
        """
        if len(poses) <= target_count:
            return poses
        
        poses_array = np.array(poses)
        selected_indices = [0]  # 从第一个位姿开始
        
        while len(selected_indices) < target_count:
            max_min_dist = -1
            best_idx = -1
            
            for i in range(len(poses)):
                if i in selected_indices:
                    continue
                
                # 计算到已选位姿的最小距离
                min_dist = float('inf')
                for j in selected_indices:
                    dist = np.linalg.norm(poses_array[i] - poses_array[j])
                    min_dist = min(min_dist, dist)
                
                if min_dist > max_min_dist:
                    max_min_dist = min_dist
                    best_idx = i
            
            if best_idx >= 0:
                selected_indices.append(best_idx)
        
        return [poses[i] for i in selected_indices]


def main():
    """测试位姿生成器"""
    generator = CalibrationPoseGenerator(marker_distance=0.32, num_poses=20)
    
    poses = generator.generate_poses_for_eye_in_hand()
    valid_poses = generator.filter_valid_poses(poses)
    
    print(f'生成 {len(poses)} 个位姿')
    print(f'有效位姿: {len(valid_poses)} 个')
    print(f'多样性分数: {generator.compute_pose_diversity(valid_poses):.4f}')
    
    print('\n生成的位姿 (关节角度，弧度):')
    for i, pose in enumerate(valid_poses):
        angles_deg = [np.degrees(a) for a in pose]
        print(f'  位姿 {i+1}: {[f"{a:.3f}" for a in pose]} (度: {[f"{a:.1f}" for a in angles_deg]})')


if __name__ == '__main__':
    main()
