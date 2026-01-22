#!/usr/bin/env python3
"""
Alicia D机械臂 拖动示教演示程序

该脚本实现拖动示教功能：
1. 用户按Enter后，机械臂关闭力矩，进入示教模式
2. 用户可以手动拖动机械臂，同时记录轨迹
3. 用户再次按Enter，机械臂恢复力矩
4. 询问用户是否回放轨迹，按Enter确认后机械臂按记录的轨迹运动

运行前需要先启动(注意夹爪类型50/100mm):
    cd ~/alicia_ws
    source install/setup.bash
    ros2 launch alicia_d_driver alicia_d_driver.launch.py gripper_type:=50mm

然后在另一个终端运行此脚本:
    cd ~/alicia_ws
    python3 ./src/examples/02_demo_drag_teaching.py
"""

import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import time
import threading


class DragTeachingDemo(Node):
    """拖动示教演示节点"""
    
    def __init__(self):
        super().__init__('drag_teaching_demo')
        
        # 创建发布者用于控制示教模式（力矩开关）
        self.demo_mode_pub = self.create_publisher(
            Bool,
            '/demonstration',
            10
        )
        
        # 创建发布者用于发送关节命令（回放轨迹）
        self.joint_command_pub = self.create_publisher(
            JointState,
            '/joint_commands',
            10
        )
        
        # 创建订阅者用于获取关节状态
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.get_logger().info('节点已启动，等待关节状态...')
        
        # 关节名称
        self.arm_joint_names = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
        
        # 轨迹记录相关
        self.is_recording = False
        self.recorded_trajectory = []  # 存储 (timestamp, positions) 元组
        self.recording_start_time = None
        self.current_joint_positions = None
        self.record_lock = threading.Lock()
        
        # 记录间隔（秒）
        self.record_interval = 0.05  # 20Hz采样率
        self.last_record_time = 0
        
    def joint_state_callback(self, msg):
        """关节状态回调函数，用于记录轨迹"""
        # 提取机械臂关节位置
        joint_positions = {}
        for i, name in enumerate(msg.name):
            if name in self.arm_joint_names:
                joint_positions[name] = msg.position[i]
        
        # 按顺序整理关节位置
        if len(joint_positions) == len(self.arm_joint_names):
            positions = [joint_positions[name] for name in self.arm_joint_names]
            self.current_joint_positions = positions
            
            # 如果正在记录，保存轨迹点
            if self.is_recording:
                current_time = time.time()
                if current_time - self.last_record_time >= self.record_interval:
                    with self.record_lock:
                        elapsed_time = current_time - self.recording_start_time
                        self.recorded_trajectory.append((elapsed_time, positions.copy()))
                        self.last_record_time = current_time
    
    def set_demonstration_mode(self, enable: bool):
        """设置示教模式（控制力矩开关）"""
        msg = Bool()
        msg.data = enable
        self.demo_mode_pub.publish(msg)
        
        if enable:
            self.get_logger().info('已关闭力矩，进入示教模式')
        else:
            self.get_logger().info('已恢复力矩，退出示教模式')
    
    def start_recording(self):
        """开始记录轨迹"""
        with self.record_lock:
            self.recorded_trajectory = []
            self.recording_start_time = time.time()
            self.last_record_time = 0
            self.is_recording = True
        self.get_logger().info('开始记录轨迹...')
    
    def stop_recording(self):
        """停止记录轨迹"""
        self.is_recording = False
        with self.record_lock:
            trajectory_length = len(self.recorded_trajectory)
        self.get_logger().info(f'停止记录，共记录了 {trajectory_length} 个轨迹点')
        return trajectory_length > 0
    
    def playback_trajectory(self, speed_deg_s=30.0):
        """
        回放记录的轨迹
        
        Args:
            speed_deg_s: 回放速度（度/秒）
        """
        with self.record_lock:
            if len(self.recorded_trajectory) < 2:
                self.get_logger().error('轨迹点太少，无法回放')
                return False
            
            trajectory_copy = self.recorded_trajectory.copy()
        
        self.get_logger().info(f'开始回放轨迹，共 {len(trajectory_copy)} 个点...')
        
        # 对轨迹进行降采样，减少轨迹点数量
        sample_interval = max(1, len(trajectory_copy) // 100)  # 最多100个点
        sampled_trajectory = trajectory_copy[::sample_interval]
        
        # 确保最后一个点被包含
        if trajectory_copy[-1] not in sampled_trajectory:
            sampled_trajectory.append(trajectory_copy[-1])
        
        self.get_logger().info(f'降采样后 {len(sampled_trajectory)} 个点')
        
        # 逐点发送关节命令
        for i, (timestamp, positions) in enumerate(sampled_trajectory):
            # 构建关节命令消息
            cmd_msg = JointState()
            cmd_msg.header.stamp = self.get_clock().now().to_msg()
            cmd_msg.name = self.arm_joint_names
            cmd_msg.position = positions
            # 设置速度（rad/s）
            velocity_rad_s = speed_deg_s * 3.14159 / 180.0
            cmd_msg.velocity = [velocity_rad_s] * len(self.arm_joint_names)
            
            # 发布命令
            self.joint_command_pub.publish(cmd_msg)
            
            # 计算等待时间
            if i < len(sampled_trajectory) - 1:
                next_timestamp = sampled_trajectory[i + 1][0]
                wait_time = (next_timestamp - timestamp)
                if wait_time > 0:
                    time.sleep(wait_time)
            
            # 处理回调
            rclpy.spin_once(self, timeout_sec=0.01)
        
        self.get_logger().info('轨迹回放完成!')
        return True
    
    def run_demo(self):
        """执行拖动示教演示"""
        self.get_logger().info('=' * 20)
        self.get_logger().info('Alicia D 拖动示教演示')
        self.get_logger().info('=' * 20)
        
        try:
            # 等待用户输入以开始示教
            print('\n' + '=' * 20)
            input('按 Enter 键开始拖动示教\n（机械臂将关闭力矩，请用手托住机械臂避免突然掉落）...')
            
            # 关闭力矩，进入示教模式
            self.set_demonstration_mode(True)
            time.sleep(0.5)  # 等待力矩完全关闭
            
            # 开始记录轨迹
            self.start_recording()
            
            print('\n' + '-' * 20)
            print('现在可以手动拖动机械臂了！')
            print('拖动完成后，按 Enter 键结束示教...')
            print('正在记录轨迹...')
            print('-' * 20)
            
            # 在后台继续spin以接收关节状态
            # 使用一个简单的方式等待用户输入
            input_received = False
            while not input_received:
                # 在后台处理一些回调
                rclpy.spin_once(self, timeout_sec=0.1)
                
                # 检查是否有用户输入（非阻塞）
                import select
                if select.select([sys.stdin], [], [], 0.0)[0]:
                    sys.stdin.readline()
                    input_received = True
            
            # 停止记录
            has_trajectory = self.stop_recording()
            
            # 恢复力矩
            self.set_demonstration_mode(False)
            time.sleep(0.5)  # 等待力矩恢复
            
            if has_trajectory:
                # 询问是否回放
                print('\n' + '-' * 20)
                user_input = input('按 Enter 键回放刚才的轨迹，输入 q 跳过回放: ')
                
                if user_input.strip().lower() != 'q':
                    self.playback_trajectory(speed_deg_s=30.0)
                else:
                    self.get_logger().info('跳过轨迹回放')
            else:
                self.get_logger().warning('没有记录到有效轨迹')
            
            self.get_logger().info('\n' + '=' * 20)
            self.get_logger().info('拖动示教演示结束!')
            self.get_logger().info('=' * 20)
            return True
            
        except KeyboardInterrupt:
            self.get_logger().info('\n用户中断，正在退出...')
            # 确保力矩恢复
            self.set_demonstration_mode(False)
            return False
        except Exception as e:
            self.get_logger().error(f'演示过程中发生错误: {str(e)}')
            # 确保力矩恢复
            self.set_demonstration_mode(False)
            return False


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    # 创建节点
    demo_node = DragTeachingDemo()
    
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
