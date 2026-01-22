#!/usr/bin/env python3
"""
Alicia D机械臂 Pick and Place 演示程序

运行前需要先启动(注意夹爪类型50/100mm):
    cd ~/alicia_ws
    source install/setup.bash
    ros2 launch alicia_d_moveit real_robot.launch.py gripper_type:=50mm

然后在另一个终端运行此脚本:
    cd ~/alicia_ws
    python3 ./src/examples/01_moveit_pick_and_place.py
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
import time
from rclpy.duration import Duration


class PickAndPlaceDemo(Node):
    """Pick and Place 演示节点"""
    
    def __init__(self):
        super().__init__('pick_and_place_demo')
        
        # 创建Action客户端
        self.arm_action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/Alicia_controller/follow_joint_trajectory'
        )
        
        self.gripper_action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/Gripper_controller/follow_joint_trajectory'
        )
        
        # 等待Action服务器
        self.get_logger().info('等待Action服务器...')
        self.arm_action_client.wait_for_server()
        self.gripper_action_client.wait_for_server()
        self.get_logger().info('Action服务器已连接!')
        
        # 关节名称
        self.arm_joint_names = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
        self.gripper_joint_names = ['Gripper']
        
        # HOME位置
        self.home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Position A 
        # 根据实际机械臂和工作空间调整
        self.position_a = [-0.3, -0.279, 0.349, -0.105, -0.715, 0.314]
        self.position_a_above = [-0.32, -0.052, 0.471, -0.087, -1.134, 0.297]
        
        # Position B 
        # self.position_b = [1.2, -0.5, 0.8, 0.0, 0.7, 0.0]
        self.position_b = [0.3, -0.279, 0.349, -0.105, -0.715, 0.314]
        self.position_b_above = [0.32, -0.052, 0.471, -0.087, -1.134, 0.297]
        
        # 夹爪位置
        self.gripper_open = [0.0]  # 张开
        self.gripper_close = [0.025]   # 闭合
        
    def move_arm_to_joint_positions(self, joint_positions, duration_sec=3.0):
        """
        移动机械臂到指定关节位置
        
        Args:
            joint_positions: 关节位置列表
            duration_sec: 运动时长(秒)
        """
        goal_msg = FollowJointTrajectory.Goal()
        
        # 构建轨迹消息
        trajectory = JointTrajectory()
        trajectory.joint_names = self.arm_joint_names
        
        # 添加目标点
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = Duration(seconds=duration_sec).to_msg()
        
        trajectory.points.append(point)
        goal_msg.trajectory = trajectory
        
        # 发送目标
        self.get_logger().info(f'移动机械臂到: {joint_positions}......')
        send_goal_future = self.arm_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('目标被拒绝')
            return False
        
        # 等待执行完成
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        if result.result.error_code == 0:
            self.get_logger().info('到达目标位置')
            return True
        else:
            self.get_logger().error(f'运动失败，错误码: {result.result.error_code}')
            return False
    
    def move_gripper(self, gripper_position, duration_sec=1.0):
        """
        控制夹爪
        
        Args:
            gripper_position: 夹爪位置列表
            duration_sec: 运动时长(秒)
        """
        goal_msg = FollowJointTrajectory.Goal()
        
        # 构建轨迹消息
        trajectory = JointTrajectory()
        trajectory.joint_names = self.gripper_joint_names
        
        # 添加目标点
        point = JointTrajectoryPoint()
        point.positions = gripper_position
        point.time_from_start = Duration(seconds=duration_sec).to_msg()
        
        trajectory.points.append(point)
        goal_msg.trajectory = trajectory
        
        # 发送目标
        gripper_state = "闭合" if gripper_position[0] > 0.02 else "张开"
        # self.get_logger().info(f'夹爪{gripper_state}')
        send_goal_future = self.gripper_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('夹爪目标被拒绝')
            return False
        
        # 等待执行完成
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        if result.result.error_code == 0:
            self.get_logger().info(f'夹爪{gripper_state}完成')
            return True
        else:
            self.get_logger().error(f'夹爪运动失败，错误码: {result.result.error_code}')
            return False
    
    def run_demo(self):
        """执行完整的pick and place演示"""
        self.get_logger().info('=' * 50)
        self.get_logger().info('开始 Pick and Place 演示')
        self.get_logger().info('=' * 50)
        
        try:
            # 步骤1: 移动到HOME位置
            self.get_logger().info('等待移动到HOME位置......')
            if not self.move_arm_to_joint_positions(self.home_position, 3.0):
                return False
            time.sleep(1)
            
            # 步骤2: 夹爪张开
            self.get_logger().info('等待夹爪张开......')
            if not self.move_gripper(self.gripper_open, 1.0):
                return False
            time.sleep(1)
            
            # 步骤3: 移动到Position A上方
            self.get_logger().info('等待移动到Position A上方......')
            if not self.move_arm_to_joint_positions(self.position_a_above, 2.0):
                return False
            time.sleep(1)
            
            # 步骤4: 下降到Position A
            self.get_logger().info('等待下降到Position A......')
            if not self.move_arm_to_joint_positions(self.position_a, 2.0):
                return False
            time.sleep(1)
            
            # 步骤5: 夹爪闭合（抓取）
            self.get_logger().info('等待夹爪闭合......')
            if not self.move_gripper(self.gripper_close, 1.0):
                return False
            time.sleep(1)
            
            # 步骤6: 上升5cm
            self.get_logger().info('等待上升......')
            if not self.move_arm_to_joint_positions(self.position_a_above, 2.0):
                return False
            time.sleep(1)
            
            # 步骤7: 回到HOME位置
            self.get_logger().info('等待回到HOME位置......')
            if not self.move_arm_to_joint_positions(self.home_position, 2.0):
                return False
            time.sleep(1)
            
            # 步骤8: 移动到Position B上方
            self.get_logger().info('等待移动到Position B上方......')
            if not self.move_arm_to_joint_positions(self.position_b_above, 2.0):
                return False
            time.sleep(1)
            
            # 步骤9: 下降到Position B
            self.get_logger().info('等待下降到Position B......')
            if not self.move_arm_to_joint_positions(self.position_b, 2.0):
                return False
            time.sleep(1)
            
            # 步骤10: 夹爪张开（放置物体）
            self.get_logger().info('等待夹爪张开......')
            if not self.move_gripper(self.gripper_open, 1.0):
                return False
            time.sleep(1)
            
            # 步骤11: 上升5cm
            self.get_logger().info('等待上升......')
            if not self.move_arm_to_joint_positions(self.position_b_above, 2.0):
                return False
            time.sleep(1)
            
            # 步骤12: 回到HOME位置
            self.get_logger().info('等待回到HOME位置......')
            if not self.move_arm_to_joint_positions(self.home_position, 3.0):
                return False
            
            self.get_logger().info('\n' + '=' * 50)
            self.get_logger().info('Pick and Place 演示完成!')
            self.get_logger().info('=' * 50)
            return True
            
        except Exception as e:
            self.get_logger().error(f'演示过程中发生错误: {str(e)}')
            return False


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    # 创建节点
    demo_node = PickAndPlaceDemo()
    
    # 等待一下确保系统初始化完成
    time.sleep(2)
    
    # 运行演示
    success = demo_node.run_demo()
    
    # 清理
    demo_node.destroy_node()
    rclpy.shutdown()
    
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())

