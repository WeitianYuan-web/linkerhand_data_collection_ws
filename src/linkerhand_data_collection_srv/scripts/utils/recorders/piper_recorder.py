#!/usr/bin/env python3
# -*-coding:utf8-*-
import os
import sys
import time

try:
    from piper_sdk import *
except ImportError as e:
    print(f"Warning: piper_sdk not available: {e}")
    # Create a dummy class for when piper_sdk is not available
    class C_PiperInterface_V2:
        def __init__(self, can_name):
            self.can_name = can_name
            print(f"Warning: Using dummy Piper interface for {can_name}")
        
        def ConnectPort(self):
            print(f"Warning: Dummy ConnectPort called for {self.can_name}")
        
        def EnablePiper(self):
            print(f"Warning: Dummy EnablePiper called for {self.can_name}")
            return True
        
        def GetArmJointMsgs(self):
            class DummyJointState:
                class DummyJoint:
                    def __init__(self):
                        self.joint_1 = 0
                        self.joint_2 = 0
                        self.joint_3 = 0
                        self.joint_4 = 0
                        self.joint_5 = 0
                        self.joint_6 = 0
                joint_state = DummyJoint()
            return DummyJointState()
        
        def GetArmHighSpdInfoMsgs(self):
            class DummyInfo:
                class DummyMotor:
                    def __init__(self):
                        self.effort = 0
                        self.motor_speed = 0
                motor_1 = DummyMotor()
                motor_2 = DummyMotor()
                motor_3 = DummyMotor()
                motor_4 = DummyMotor()
                motor_5 = DummyMotor()
                motor_6 = DummyMotor()
            return DummyInfo()
        
        def GetArmJointCtrl(self):
            class DummyCtrl:
                class DummyJointCtrl:
                    def __init__(self):
                        self.joint_1 = 0
                        self.joint_2 = 0
                        self.joint_3 = 0
                        self.joint_4 = 0
                        self.joint_5 = 0
                        self.joint_6 = 0
                joint_ctrl = DummyJointCtrl()
            return DummyCtrl()
        
        def GetArmEndPoseMsgs(self):
            class DummyEndPose:
                class DummyPose:
                    def __init__(self):
                        self.X_axis = 0
                        self.Y_axis = 0
                        self.Z_axis = 0
                        self.RX_axis = 0
                        self.RY_axis = 0
                        self.RZ_axis = 0
                end_pose = DummyPose()
            return DummyEndPose()
        
        def GetFK(self, mode="feedback"):
            return [[0, 0, 0, 0, 0, 0]] * 6
        
        def JointCtrl(self, *args):
            print(f"Warning: Dummy JointCtrl called for {self.can_name} with args: {args}")


class Recorder:
    def __init__(self, arm="left"):
        if arm == "left":
            # self.can_name = "can_left"
            self.can_name = "arm_left"  # Changed from "can_left" to "can0"
        elif arm == "right":
            # self.can_name = "can_right"
            self.can_name = "arm_right"  # Changed from "can_right" to "can1"
        self.piper = C_PiperInterface_V2(self.can_name)
        self.piper.ConnectPort()
        self._enabled = False  # Track if arm is enabled for inference mode
        
        # Store latest timestamps for synchronization
        self._latest_joint_timestamp = None
        self._latest_effort_timestamp = None
        self._latest_velocity_timestamp = None
        self._latest_control_timestamp = None

    def enable_piper_for_inference(self):
        '''Enable the Piper arm for inference mode (command sending)'''
        if not self._enabled:
            import time
            while not self.piper.EnablePiper():
                time.sleep(0.01)
            self._enabled = True
            print(f"✓ Piper arm {self.can_name} enabled for inference mode")

    def get_piper_slave_qpos_arc_list(self) -> list:
        '''获取从动机械臂关节qpos角度
        Args:
            self.piper (C_PiperInterface): 机械臂实例
        Returns:
            list: 机械臂关节角度
        '''
        joint_msgs = self.piper.GetArmJointMsgs()
        joint_state = joint_msgs.joint_state
        
        # Extract timestamp if available from SDK
        try:
            if hasattr(joint_msgs, 'timestamp'):
                self._latest_joint_timestamp = joint_msgs.timestamp
            elif hasattr(joint_state, 'timestamp'):
                self._latest_joint_timestamp = joint_state.timestamp
            else:
                # Fallback to current time if no timestamp available
                self._latest_joint_timestamp = time.time()
        except Exception:
            self._latest_joint_timestamp = time.time()
        
        return [
            round(joint_state.joint_1 * 0.001, 3),
            round(joint_state.joint_2 * 0.001, 3),
            round(joint_state.joint_3 * 0.001, 3),
            round(joint_state.joint_4 * 0.001, 3),
            round(joint_state.joint_5 * 0.001, 3),
            round(joint_state.joint_6 * 0.001, 3)
        ]
    
    def get_piper_slave_effort_arc_list(self) -> list:
        '''获取从动机械臂关节effort力矩
        Args:
            self.piper (C_PiperInterface): 机械臂实例
        Returns:
            list: 机械臂关节effort力矩
        '''
        info_msgs = self.piper.GetArmHighSpdInfoMsgs()
        
        # Extract timestamp if available from SDK
        try:
            if hasattr(info_msgs, 'timestamp'):
                self._latest_effort_timestamp = info_msgs.timestamp
            else:
                # Fallback to current time if no timestamp available
                self._latest_effort_timestamp = time.time()
        except Exception:
            self._latest_effort_timestamp = time.time()
        
        return [
            round(info_msgs.motor_1.effort * 0.001, 3),
            round(info_msgs.motor_2.effort * 0.001, 3),
            round(info_msgs.motor_3.effort * 0.001, 3),
            round(info_msgs.motor_4.effort * 0.001, 3),
            round(info_msgs.motor_5.effort * 0.001, 3),
            round(info_msgs.motor_6.effort * 0.001, 3)
        ]
    
    def get_piper_slave_qvel_arc_list(self) -> list:
        '''获取从动机械臂关节qvel速度
        Args:
            self.piper (C_PiperInterface): 机械臂实例
        Returns:
            list: 机械臂关节速度
        '''
        info_msgs = self.piper.GetArmHighSpdInfoMsgs()
        
        # Extract timestamp if available from SDK (reuse effort timestamp since it's from same message)
        try:
            if hasattr(info_msgs, 'timestamp'):
                self._latest_velocity_timestamp = info_msgs.timestamp
            else:
                # Fallback to current time if no timestamp available
                self._latest_velocity_timestamp = time.time()
        except Exception:
            self._latest_velocity_timestamp = time.time()
        
        return [
            round(info_msgs.motor_1.motor_speed * 0.001, 3),
            round(info_msgs.motor_2.motor_speed * 0.001, 3),
            round(info_msgs.motor_3.motor_speed * 0.001, 3),
            round(info_msgs.motor_4.motor_speed * 0.001, 3),
            round(info_msgs.motor_5.motor_speed * 0.001, 3),
            round(info_msgs.motor_6.motor_speed * 0.001, 3)
        ]
    
    def get_piper_master_ctrl_arc_list(self) -> list:
        '''获取主动机械臂关节qpos角度
        Args:
            self.piper (C_PiperInterface): 机械臂实例
        Returns:
            list: 机械臂关节角度
        '''
        ctrl_msgs = self.piper.GetArmJointCtrl()
        joints_ctrl = ctrl_msgs.joint_ctrl
        
        # Extract timestamp if available from SDK
        try:
            if hasattr(ctrl_msgs, 'timestamp'):
                self._latest_control_timestamp = ctrl_msgs.timestamp
            elif hasattr(joints_ctrl, 'timestamp'):
                self._latest_control_timestamp = joints_ctrl.timestamp
            else:
                # Fallback to current time if no timestamp available
                self._latest_control_timestamp = time.time()
        except Exception:
            self._latest_control_timestamp = time.time()
        
        return [
            round(joints_ctrl.joint_1 * 0.001, 3),
            round(joints_ctrl.joint_2 * 0.001, 3),
            round(joints_ctrl.joint_3 * 0.001, 3),
            round(joints_ctrl.joint_4 * 0.001, 3),
            round(joints_ctrl.joint_5 * 0.001, 3),
            round(joints_ctrl.joint_6 * 0.001, 3)
        ]
    
    def get_piper_slave_end_pose(self):
        '''获取从动臂末端执行器位姿
        Args:
            self.piper (C_PiperInterface): 机械臂实例
        Returns:
            list: 末端执行器位姿
        '''
        # slave_end_pose = self.piper.GetFK(mode="feedback")
        # return slave_end_pose[5]
        slave_end_pose = self.piper.GetArmEndPoseMsgs()
        return [
            round(slave_end_pose.end_pose.X_axis * 0.001, 3),
            round(slave_end_pose.end_pose.Y_axis * 0.001, 3),
            round(slave_end_pose.end_pose.Z_axis * 0.001, 3),
            round(slave_end_pose.end_pose.RX_axis * 0.001, 3),
            round(slave_end_pose.end_pose.RY_axis * 0.001, 3),
            round(slave_end_pose.end_pose.RZ_axis * 0.001, 3)
        ]
    
    def get_piper_master_end_pose(self):
        '''获取主动臂末端执行器位姿
        Args:
            self.piper (C_PiperInterface): 机械臂实例
        Returns:
            list: 末端执行器位姿
        '''
        master_end_pose = self.piper.GetFK(mode="control")
        return master_end_pose[5]

    def send_piper_slave_qpos_arc_list(self, qpos_list):
        '''发送关节位置到从动机械臂
        Args:
            self.piper (C_PiperInterface): 机械臂实例
            qpos_list (list): 关节角度列表 [6个关节]
        '''
        if len(qpos_list) != 6:
            raise ValueError(f"Expected 6 joint positions, got {len(qpos_list)}")
        
        # Enable the arm for inference mode if not already enabled
        self.enable_piper_for_inference()
        
        # Convert from radians to the units expected by Piper SDK (multiply by 1000)
        joint_positions = [int(pos * 1000) for pos in qpos_list]
        
        # Use the correct JointCtrl method with individual arguments
        try:
            self.piper.JointCtrl(joint_positions[0], joint_positions[1], joint_positions[2], 
                                joint_positions[3], joint_positions[4], joint_positions[5])
        except Exception as e:
            print(f"Warning: JointCtrl failed for Piper arm {self.can_name}: {e}")

    def send_piper_master_qpos_arc_list(self, qpos_list):
        '''发送关节位置到主动机械臂
        Args:
            self.piper (C_PiperInterface): 机械臂实例
            qpos_list (list): 关节角度列表 [6个关节]
        '''
        if len(qpos_list) != 6:
            raise ValueError(f"Expected 6 joint positions, got {len(qpos_list)}")
        
        # Enable the arm for inference mode if not already enabled
        self.enable_piper_for_inference()
        
        # Convert from radians to the units expected by Piper SDK (multiply by 1000)
        joint_positions = [int(pos * 1000) for pos in qpos_list]
        
        # Use the same JointCtrl method for both master and slave
        try:
            self.piper.JointCtrl(joint_positions[0], joint_positions[1], joint_positions[2], 
                                joint_positions[3], joint_positions[4], joint_positions[5])
        except Exception as e:
            print(f"Warning: JointCtrl failed for Piper arm {self.can_name}: {e}")
    
    def get_latest_timestamps(self) -> dict:
        """Get the latest timestamps from all data sources.
        
        Returns:
            dict: Dictionary containing timestamps for joint states, effort, velocity, and control
        """
        return {
            'joint_timestamp': self._latest_joint_timestamp,
            'effort_timestamp': self._latest_effort_timestamp,
            'velocity_timestamp': self._latest_velocity_timestamp,
            'control_timestamp': self._latest_control_timestamp
        }
    
    def get_representative_timestamp(self) -> float:
        """Get the most representative timestamp for this arm.
        
        Returns:
            float: The most recent timestamp available, or current time if none available
        """
        timestamps = [
            self._latest_joint_timestamp,
            self._latest_effort_timestamp,
            self._latest_velocity_timestamp,
            self._latest_control_timestamp
        ]
        
        # Return the most recent non-None timestamp
        valid_timestamps = [ts for ts in timestamps if ts is not None]
        if valid_timestamps:
            return max(valid_timestamps)
        else:
            return time.time()
