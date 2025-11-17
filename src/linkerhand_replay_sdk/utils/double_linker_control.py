#!/usr/bin/env python3
# dual_arm_subscriber.py - 基础双臂数据订阅

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import threading
import time

class DualArmSubscriber(Node):
    def __init__(self):
        super().__init__('dual_arm_subscriber')
        
        # 存储最新数据
        self.left_arm_data = None
        self.right_arm_data = None
        self.last_update_time = None
        
        # 创建订阅者 - 左臂
        self.left_arm_subscription = self.create_subscription(
            JointState,
            '/left_arm_joint_control', # left_arm_joint_state/left_arm_joint_control
            self.left_arm_callback,
            10  # 队列大小
        )
        
        # 创建订阅者 - 右臂
        self.right_arm_subscription = self.create_subscription(
            JointState,
            '/right_arm_joint_control', # right_arm_joint_state/right_arm_joint_control
            self.right_arm_callback,
            10
        )
        
        # 数据同步锁
        self.data_lock = threading.Lock()
        
        # 统计信息
        self.left_msg_count = 0
        self.right_msg_count = 0
        
        self.get_logger().info('双臂数据订阅器已启动')
        self.get_logger().info('等待左右臂数据...')
    
    def left_arm_callback(self, msg):
        """左臂数据回调函数"""
        with self.data_lock:
            self.left_arm_data = msg
            self.left_msg_count += 1
            self.last_update_time = self.get_clock().now()
        
        # 实时显示数据
        self.display_realtime_data('LEFT', msg)
    
    def right_arm_callback(self, msg):
        """右臂数据回调函数"""
        with self.data_lock:
            self.right_arm_data = msg
            self.right_msg_count += 1
            self.last_update_time = self.get_clock().now()
        
        # 实时显示数据
        self.display_realtime_data('RIGHT', msg)
    
    def display_realtime_data(self, arm, msg):
        """实时显示数据（简化版）"""
        positions = list(msg.position) if msg.position else []
        velocities = list(msg.velocity) if msg.velocity else []
        efforts = list(msg.effort) if msg.effort else []
        
        print(f"\n[{arm} ARM] 时间: {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}")
        
        if positions:
            print(f"  位置: {[f'{p:.3f}' for p in positions]}")
        if velocities:
            print(f"  速度: {[f'{v:.3f}' for v in velocities]}")
        if efforts:
            print(f"  力矩: {[f'{e:.3f}' for e in efforts]}")
    
    def get_synchronized_data(self):
        """获取同步的双臂数据"""
        with self.data_lock:
            return {
                'left': self.left_arm_data,
                'right': self.right_arm_data,
                'timestamp': self.last_update_time
            }
    
    def print_statistics(self):
        """打印统计信息"""
        with self.data_lock:
            print(f"\n=== 统计信息 ===")
            print(f"左臂消息数: {self.left_msg_count}")
            print(f"右臂消息数: {self.right_msg_count}")
            print(f"最后更新: {self.last_update_time}")

#!/usr/bin/env python3
# joint_command_publisher.py - 基础关节控制发布器

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import time
from threading import Thread

class JointCommandPublisher(Node):
    def __init__(self):
        super().__init__('joint_command_publisher')
        
        # 创建发布者 - 左臂
        self.left_publisher = self.create_publisher(
            JointState, 
            '/left_arm_joint_control', 
            10
        )
        
        # 创建发布者 - 右臂
        self.right_publisher = self.create_publisher(
            JointState, 
            '/right_arm_joint_control', 
            10
        )
        
        # 关节名称（假设7个关节）
        self.joint_names = [
            'shoulder_pan_joint',    # 肩部平移
            'shoulder_lift_joint',   # 肩部抬升
            'elbow_joint',          # 肘部
            'wrist_1_joint',        # 腕部1
            'wrist_2_joint',        # 腕部2
            'wrist_3_joint',        # 腕部3
            'gripper_joint'         # 夹爪
        ]
        
        # 关节数量
        self.num_joints = len(self.joint_names)
        
        # 当前目标位置和速度
        self.left_target_positions = [0.0] * self.num_joints
        self.left_target_velocities = [0.0] * self.num_joints
        
        self.right_target_positions = [0.0] * self.num_joints
        self.right_target_velocities = [0.0] * self.num_joints
        
        # 发布定时器
        self.timer = self.create_timer(0.1, self.publish_commands)  # 10Hz
        
        self.get_logger().info(f'关节命令发布器已启动，控制{self.num_joints}个关节')
    
    def set_left_arm_target(self, positions, velocities=None):
        """设置左臂目标位置和速度"""
        if len(positions) != self.num_joints:
            self.get_logger().error(f'位置数据长度应为{self.num_joints}，实际为{len(positions)}')
            return False
        
        self.left_target_positions = list(positions)
        
        if velocities is not None:
            if len(velocities) != self.num_joints:
                self.get_logger().error(f'速度数据长度应为{self.num_joints}，实际为{len(velocities)}')
                return False
            self.left_target_velocities = list(velocities)
        else:
            # 如果没有提供速度，设置为零
            self.left_target_velocities = [0.0] * self.num_joints
        
        self.get_logger().info(f'设置左臂目标位置: {positions}')
        return True
    
    def set_right_arm_target(self, positions, velocities=None):
        """设置右臂目标位置和速度"""
        if len(positions) != self.num_joints:
            self.get_logger().error(f'位置数据长度应为{self.num_joints}，实际为{len(positions)}')
            return False
        
        self.right_target_positions = list(positions)
        
        if velocities is not None:
            if len(velocities) != self.num_joints:
                self.get_logger().error(f'速度数据长度应为{self.num_joints}，实际为{len(velocities)}')
                return False
            self.right_target_velocities = list(velocities)
        else:
            self.right_target_velocities = [0.0] * self.num_joints
        
        self.get_logger().info(f'设置右臂目标位置: {positions}')
        return True
    
    def create_joint_state_message(self, positions, velocities, efforts=None, arm_name=""):
        """创建关节状态消息"""
        msg = JointState()
        
        # 设置时间戳
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'{arm_name}_arm_base'
        
        # 设置关节名称
        msg.name = self.joint_names
        
        # 设置位置、速度、力矩
        msg.position = [float(pos) for pos in positions]
        msg.velocity = [float(vel) for vel in velocities]
        
        if efforts is not None and len(efforts) == self.num_joints:
            msg.effort = [float(eff) for eff in efforts]
        else:
            msg.effort = [0.0] * self.num_joints  # 默认零力矩
        
        return msg
    
    def publish_commands(self):
        """发布关节命令"""
        # 发布左臂命令
        left_msg = self.create_joint_state_message(
            self.left_target_positions,
            self.left_target_velocities,
            arm_name="left"
        )
        self.left_publisher.publish(left_msg)
        
        # 发布右臂命令
        right_msg = self.create_joint_state_message(
            self.right_target_positions,
            self.right_target_velocities,
            arm_name="right"
        )
        self.right_publisher.publish(right_msg)
        
        # 可选：记录发布状态
        if hasattr(self, 'publish_count'):
            self.publish_count += 1
            if self.publish_count % 50 == 0:  # 每5秒记录一次（10Hz * 5）
                self.get_logger().info('持续发布关节命令...')
        else:
            self.publish_count = 1

def main():
    rclpy.init()
    
    # 创建发布器
    publisher = JointCommandPublisher()
    
    # 示例：设置初始位置
    # 左臂：初始位置
    left_init_pos = [-3.8672375000000017,2.2848593749999964,1.0547125000000008,-5.800753125,-2.1090937499999995,-21.972484374999993,17.75371875000002]  # 弧度
    left_init_vel = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]     # 弧度/秒
    
    # 右臂：初始位置
    right_init_pos = [16.0643125,2.7244999999999777,-2.8120156249999866,-2.5489531250000255,26.894953124999972,10.722681249999994,-6.592234375000004]
    right_init_vel = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    
    publisher.set_left_arm_target(left_init_pos, left_init_vel)
    publisher.set_right_arm_target(right_init_pos, right_init_vel)
    
    print("关节命令发布器已启动")
    print("按 Ctrl+C 停止")

    
    try:
        # 在后台线程中运行ROS2
        import threading
        
        def spin_node():
            rclpy.spin(publisher)
        
        spin_thread = threading.Thread(target=spin_node, daemon=True)
        spin_thread.start()
        
        # 主线程：交互式控制
        interactive_control(publisher)
        
    except KeyboardInterrupt:
        print("\n收到停止信号...")
    finally:
        publisher.destroy_node()
        rclpy.shutdown()

def interactive_control(publisher):
    """交互式控制界面"""

    # 后台线程中运行ROS2
    import threading
    # 创建订阅器节点
    subscriber = DualArmSubscriber()
    
    def spin_node():
        rclpy.spin(subscriber)
    
    spin_thread = threading.Thread(target=spin_node, daemon=True)
    spin_thread.start()
    
    print("双臂数据监控已启动")
    print("按 Ctrl+C 停止监控")
    print("=" * 50)
    
   
    flag = 0

    try:
        while True:
            print("\n=== 双臂控制选项 ===")

            # 左臂：初始位置
            left_init_pos = [-3.8672375000000017,2.2848593749999964,1.0547125000000008,-5.800753125,-2.1090937499999995,-21.972484374999993,17.75371875000002]  # 弧度
            left_init_vel = [20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0]     # 弧度/秒
            # 右臂：初始位置
            right_init_pos = [6.0643125,2.7244999999999777,-2.8120156249999866,-2.5489531250000255,26.894953124999972,10.722681249999994,-6.592234375000004]
            right_init_vel = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0] 
            success = publisher.set_left_arm_target(left_init_pos, left_init_vel)
            success = publisher.set_right_arm_target(right_init_pos, right_init_vel)
            time.sleep(1)
        
            # 左臂：初始位置
            left_init_pos = [-13.8672375000000017,2.2848593749999964,1.0547125000000008,-5.800753125,-2.1090937499999995,-21.972484374999993,17.75371875000002]  # 弧度
            left_init_vel = [20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0]     # 弧度/秒
            # 右臂：初始位置
            right_init_pos = [10.0643125,2.7244999999999777,-2.8120156249999866,-2.5489531250000255,26.894953124999972,10.722681249999994,-6.592234375000004]
            right_init_vel = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0] 
            success = publisher.set_left_arm_target(left_init_pos, left_init_vel)
            success = publisher.set_right_arm_target(right_init_pos, right_init_vel)
            time.sleep(1)

            # 左臂：初始位置
            left_init_pos = [-20.8672375000000017,2.2848593749999964,1.0547125000000008,-5.800753125,-2.1090937499999995,-21.972484374999993,17.75371875000002]  # 弧度
            left_init_vel = [20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0]     # 弧度/秒
            # 右臂：初始位置
            right_init_pos = [20.0643125,2.7244999999999777,-2.8120156249999866,-2.5489531250000255,26.894953124999972,10.722681249999994,-6.592234375000004]
            right_init_vel = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0] 
            success = publisher.set_left_arm_target(left_init_pos, left_init_vel)
            success = publisher.set_right_arm_target(right_init_pos, right_init_vel)
            time.sleep(1)

            # 左臂：初始位置
            left_init_pos = [-13.8672375000000017,2.2848593749999964,1.0547125000000008,-5.800753125,-2.1090937499999995,-21.972484374999993,17.75371875000002]  # 弧度
            left_init_vel = [20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0]     # 弧度/秒
            # 右臂：初始位置
            right_init_pos = [10.0643125,2.7244999999999777,-2.8120156249999866,-2.5489531250000255,26.894953124999972,10.722681249999994,-6.592234375000004]
            right_init_vel = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0] 
            success = publisher.set_left_arm_target(left_init_pos, left_init_vel)
            success = publisher.set_right_arm_target(right_init_pos, right_init_vel)
            time.sleep(1)

            flag = flag+1

                
    except KeyboardInterrupt:
        print("\n退出交互控制")

def set_arm_position(publisher, arm):
    """设置单臂位置"""
    print(f"\n设置{arm}臂7个关节的位置 (单位: 弧度):")
    
    positions = []
    for i in range(7):
        while True:
            try:
                pos = float(input(f"关节{i+1}位置: "))
                positions.append(pos)
                break
            except ValueError:
                print("请输入有效数字")
    
    velocities = []
    use_velocity = input("是否设置速度? (y/N): ").strip().lower() == 'y'
    if use_velocity:
        print("设置7个关节的速度 (单位: 弧度/秒):")
        for i in range(7):
            while True:
                try:
                    vel = float(input(f"关节{i+1}速度: "))
                    velocities.append(vel)
                    break
                except ValueError:
                    print("请输入有效数字")
    
    if arm == 'left':
        success = publisher.set_left_arm_target(positions, velocities if use_velocity else None)
    else:
        success = publisher.set_right_arm_target(positions, velocities if use_velocity else None)
    
    if success:
        print(f"{arm}臂目标位置设置成功")
    else:
        print("设置失败")

def set_both_arms_same(publisher):
    """设置双臂相同位置"""
    print("\n设置双臂7个关节的相同位置:")
    
    positions = []
    for i in range(7):
        while True:
            try:
                pos = float(input(f"关节{i+1}位置: "))
                positions.append(pos)
                break
            except ValueError:
                print("请输入有效数字")
    
    velocities = []
    use_velocity = input("是否设置速度? (y/N): ").strip().lower() == 'y'
    if use_velocity:
        print("设置7个关节的速度:")
        for i in range(7):
            while True:
                try:
                    vel = float(input(f"关节{i+1}速度: "))
                    velocities.append(vel)
                    break
                except ValueError:
                    print("请输入有效数字")
    
    publisher.set_left_arm_target(positions, velocities if use_velocity else None)
    publisher.set_right_arm_target(positions, velocities if use_velocity else None)
    print("双臂目标位置设置成功")

def execute_predefined_trajectory(publisher):
    """执行预定义轨迹"""
    print("\n=== 预定义轨迹 ===")
    print("1. 归零位置")
    print("2. 准备位置")
    print("3. 伸展位置")
    print("4. 自定义正弦波")
    
    choice = input("选择轨迹 (1-4): ").strip()
    
    if choice == '1':
        # 归零位置
        zero_pos = [0.0] * 7
        zero_vel = [0.0] * 7
        publisher.set_left_arm_target(zero_pos, zero_vel)
        publisher.set_right_arm_target(zero_pos, zero_vel)
        print("归零位置设置完成")
        
    elif choice == '2':
        # 准备位置
        ready_pos = [0.0, -1.0, 1.5, 0.5, 0.0, 0.0, 0.0]
        ready_vel = [0.1] * 7
        publisher.set_left_arm_target(ready_pos, ready_vel)
        publisher.set_right_arm_target(ready_pos, ready_vel)
        print("准备位置设置完成")
        
    elif choice == '3':
        # 伸展位置
        extend_pos = [0.5, -1.2, 2.0, 0.8, 0.3, 0.1, 0.0]
        extend_vel = [0.05] * 7
        publisher.set_left_arm_target(extend_pos, extend_vel)
        publisher.set_right_arm_target(extend_pos, extend_vel)
        print("伸展位置设置完成")
        
    elif choice == '4':
        # 正弦波轨迹
        execute_sine_wave_trajectory(publisher)
        
    else:
        print("无效选择")

def execute_sine_wave_trajectory(publisher):
    """执行正弦波轨迹"""
    print("执行正弦波轨迹...")
    
    import math
    import time
    
    duration = 10.0  # 10秒
    steps = 100
    dt = duration / steps
    
    for step in range(steps):
        t = step * dt
        # 每个关节有不同的频率和幅度
        frequencies = [0.5, 0.7, 0.9, 1.1, 1.3, 1.5, 0.3]
        amplitudes = [0.5, 0.4, 0.6, 0.3, 0.2, 0.1, 0.05]
        
        positions = []
        velocities = []
        
        for i in range(7):
            # 位置：正弦波
            pos = amplitudes[i] * math.sin(2 * math.pi * frequencies[i] * t)
            positions.append(pos)
            
            # 速度：导数（余弦波）
            vel = amplitudes[i] * 2 * math.pi * frequencies[i] * math.cos(2 * math.pi * frequencies[i] * t)
            velocities.append(vel)
        
        publisher.set_left_arm_target(positions, velocities)
        publisher.set_right_arm_target(positions, velocities)
        
        time.sleep(dt)
    
    print("正弦波轨迹完成")

def show_current_targets(publisher):
    """显示当前目标"""
    print("\n当前目标位置:")
    print(f"左臂: {publisher.left_target_positions}")
    print(f"右臂: {publisher.right_target_positions}")
    print(f"左臂速度: {publisher.left_target_velocities}")
    print(f"右臂速度: {publisher.right_target_velocities}")

if __name__ == '__main__':
    main()


