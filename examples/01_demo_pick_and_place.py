#!/usr/bin/env python3
"""
Alicia D机械臂 Pick and Place 演示程序

运行前需要先启动(注意夹爪类型50/100mm):
    cd ~/alicia_ws
    source install/setup.bash
    ros2 launch alicia_d_driver alicia_d_driver.launch.py gripper_type:=50mm

然后在另一个终端运行此脚本:
    cd ~/alicia_ws
    python3 ./src/examples/01_demo_pick_and_place.py
"""

import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import math


class PickAndPlaceDemo(Node):
    """Pick and Place 演示节点"""
    
    def __init__(self):
        super().__init__('pick_and_place_demo')
        
        # 创建发布者用于发送关节命令
        self.joint_command_pub = self.create_publisher(
            JointState,
            '/joint_commands',
            10
        )
        
        # 创建订阅者用于获取当前关节状态
        self.current_joint_positions = None
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.get_logger().info('节点已启动，等待关节状态...')
        
        # 关节名称
        self.arm_joint_names = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
        
        # HOME位置
        self.home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Position A 
        # 根据实际机械臂和工作空间调整
        self.position_a = [-0.3, -0.279, 0.349, -0.105, -0.715, 0.314]
        self.position_a_above = [-0.32, -0.052, 0.471, -0.087, -1.134, 0.297]
        
        # Position B 
        self.position_b = [0.3, -0.279, 0.349, -0.105, -0.715, 0.314]
        self.position_b_above = [0.32, -0.052, 0.471, -0.087, -1.134, 0.297]
        
        # 夹爪位置 (0-1000)
        self.gripper_open = 1000.0      # 张开
        self.gripper_close = 0.0  # 闭合
        
        # 默认运动速度（度/秒）
        self.default_speed_deg_s = 30.0
        
    def joint_state_callback(self, msg):
        """关节状态回调函数"""
        joint_positions = {}
        for i, name in enumerate(msg.name):
            if name in self.arm_joint_names:
                joint_positions[name] = msg.position[i]
        
        if len(joint_positions) == len(self.arm_joint_names):
            self.current_joint_positions = [joint_positions[name] for name in self.arm_joint_names]
    
    def wait_for_joint_states(self, timeout=5.0):
        """等待接收关节状态"""
        start_time = time.time()
        while self.current_joint_positions is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                self.get_logger().error('等待关节状态超时')
                return False
        self.get_logger().info('已接收关节状态')
        return True
    
    def move_arm_to_joint_positions(self, target_positions, speed_deg_s=None):
        """
        移动机械臂到指定关节位置
        
        Args:
            target_positions: 目标关节位置列表（弧度）
            speed_deg_s: 运动速度（度/秒）
        """
        if speed_deg_s is None:
            speed_deg_s = self.default_speed_deg_s
        
        # 构建关节命令消息
        cmd_msg = JointState()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.name = self.arm_joint_names
        cmd_msg.position = target_positions
        # 设置速度（rad/s）
        velocity_rad_s = speed_deg_s * math.pi / 180.0
        cmd_msg.velocity = [velocity_rad_s] * len(self.arm_joint_names)
        
        # 发布命令
        self.get_logger().info(f'移动机械臂到: {[f"{p:.3f}" for p in target_positions]}')
        self.joint_command_pub.publish(cmd_msg)
        
        # 计算预估运动时间并等待
        if self.current_joint_positions is not None:
            max_diff = 0.0
            for i in range(len(target_positions)):
                diff = abs(target_positions[i] - self.current_joint_positions[i])
                max_diff = max(max_diff, diff)
            # 预估时间 = 最大角度差 / 角速度
            estimated_time = (max_diff * 180.0 / math.pi) / speed_deg_s
            wait_time = estimated_time + 0.5  # 额外等待0.5秒确保到位
        else:
            wait_time = 3.0  # 默认等待时间
        
        # 等待运动完成，同时处理回调
        start_time = time.time()
        while time.time() - start_time < wait_time:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.get_logger().info('到达目标位置')
        return True
    
    def move_gripper(self, gripper_value, wait_time=2.0):
        """
        控制夹爪
        
        Args:
            gripper_value: 夹爪位置（0-1000）
            wait_time: 等待时间（秒）
        """
        # 构建关节命令消息（同时发送当前机械臂位置和夹爪命令，避免机械臂移动）
        cmd_msg = JointState()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        
        # 同时发送机械臂当前位置和夹爪命令
        cmd_msg.name = self.arm_joint_names + ['Gripper']
        if self.current_joint_positions is not None:
            cmd_msg.position = self.current_joint_positions + [gripper_value]
        else:
            cmd_msg.position = self.home_position + [gripper_value]
        
        gripper_state = "闭合" if gripper_value < 100 else "张开"
        self.get_logger().info(f'夹爪{gripper_state}...')
        self.joint_command_pub.publish(cmd_msg)
        
        # 等待夹爪动作完成
        start_time = time.time()
        while time.time() - start_time < wait_time:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.get_logger().info(f'夹爪{gripper_state}完成')
        return True
    
    def run_demo(self):
        """执行完整的pick and place演示"""
        self.get_logger().info('=' * 20)
        self.get_logger().info('开始 Pick and Place 演示')
        self.get_logger().info('=' * 20)
        
        try:
            # 等待关节状态
            if not self.wait_for_joint_states():
                return False
            
            # 步骤1: 移动到HOME位置
            self.get_logger().info('[步骤1] 移动到HOME位置')
            if not self.move_arm_to_joint_positions(self.home_position):
                return False
            time.sleep(0.5)
            
            # 步骤2: 夹爪张开
            self.get_logger().info('[步骤2] 夹爪张开')
            if not self.move_gripper(self.gripper_open):
                return False
            time.sleep(1.0)
            
            # 步骤3: 移动到Position A上方
            self.get_logger().info('[步骤3] 移动到Position A上方')
            if not self.move_arm_to_joint_positions(self.position_a_above):
                return False
            time.sleep(0.5)
            
            # 步骤4: 下降到Position A
            self.get_logger().info('[步骤4] 下降到Position A')
            if not self.move_arm_to_joint_positions(self.position_a):
                return False
            time.sleep(0.5)
            
            # 步骤5: 夹爪闭合（抓取）
            self.get_logger().info('[步骤5] 夹爪闭合（抓取）')
            if not self.move_gripper(self.gripper_close):
                return False
            time.sleep(0.5)
            
            # 步骤6: 上升
            self.get_logger().info('[步骤6] 上升')
            if not self.move_arm_to_joint_positions(self.position_a_above):
                return False
            time.sleep(0.5)
            
            # 步骤7: 回到HOME位置
            self.get_logger().info('[步骤7] 回到HOME位置')
            if not self.move_arm_to_joint_positions(self.home_position):
                return False
            time.sleep(0.5)
            
            # 步骤8: 移动到Position B上方
            self.get_logger().info('[步骤8] 移动到Position B上方')
            if not self.move_arm_to_joint_positions(self.position_b_above):
                return False
            time.sleep(0.5)
            
            # 步骤9: 下降到Position B
            self.get_logger().info('[步骤9] 下降到Position B')
            if not self.move_arm_to_joint_positions(self.position_b):
                return False
            time.sleep(0.5)
            
            # 步骤10: 夹爪张开（放置物体）
            self.get_logger().info('[步骤10] 夹爪张开（放置物体）')
            if not self.move_gripper(self.gripper_open):
                return False
            time.sleep(0.5)
            
            # 步骤11: 上升
            self.get_logger().info('[步骤11] 上升')
            if not self.move_arm_to_joint_positions(self.position_b_above):
                return False
            time.sleep(0.5)
            
            # 步骤12: 回到HOME位置
            self.get_logger().info('[步骤12] 回到HOME位置')
            if not self.move_arm_to_joint_positions(self.home_position):
                return False
            
            self.get_logger().info('=' * 20)
            self.get_logger().info('Pick and Place 演示完成!')
            self.get_logger().info('=' * 20)
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
