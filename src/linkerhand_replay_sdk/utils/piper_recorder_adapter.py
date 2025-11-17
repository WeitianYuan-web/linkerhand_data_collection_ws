#!/usr/bin/env python3
# -*-coding:utf8-*-
"""
Adapter to make PiperRecorder compatible with ArmRecorder interface.
This allows seamless use of Piper arms in the data collection pipeline.
"""

import os
import yaml
import time
from pathlib import Path

# Import the Piper SDK directly
try:
    from piper_sdk import C_PiperInterface_V2
    PIPER_SDK_AVAILABLE = True
except ImportError:
    try:
        # Try alternative import path
        from piper_sdk.interface.piper_interface_v2 import C_PiperInterface_V2
        PIPER_SDK_AVAILABLE = True
    except ImportError:
        PIPER_SDK_AVAILABLE = False
        print("Warning: Piper SDK not available")
        C_PiperInterface_V2 = None


class PiperRecorderAdapter:
    """
    Adapter class that wraps Piper SDK to provide ArmRecorder-compatible interface.
    """
    
    def __init__(self, side: str = 'left'):
        """
        Initialize Piper arm recorder adapter.
        
        Args:
            side: 'left' or 'right'
        """
        if not PIPER_SDK_AVAILABLE:
            raise ImportError("Piper SDK is not available")
        
        self.side = side
        self._dof = 6  # Piper arm always has 6 DoF
        self._enabled = False  # Track if arm is enabled for inference mode
        
        # Store latest timestamps for synchronization
        self._latest_joint_timestamp = None
        self._latest_effort_timestamp = None
        self._latest_velocity_timestamp = None
        self._latest_control_timestamp = None
        
        # Load CAN configuration from hand_can_config.yaml
        can_name = self._load_can_config(side)
        
        # Initialize Piper SDK directly with the correct CAN name
        print(f"✓ Initializing Piper {side} arm with CAN: {can_name}")
        self.piper = C_PiperInterface_V2(can_name)
        self.piper.ConnectPort()
    
    def _load_can_config(self, side: str) -> str:
        """
        Load CAN configuration from hand_can_config.yaml.
        
        Returns:
            CAN device name (e.g., 'can0', 'can1')
        """
        try:
            # Find workspace root
            current_file = Path(__file__).resolve()
            for parent in current_file.parents:
                config_file = parent / 'scripts' / 'hand_can_config.yaml'
                if config_file.exists():
                    with open(config_file, 'r') as f:
                        config = yaml.safe_load(f)
                    
                    # Get CAN name for the specified side
                    arm_key = f'{side}_arm'
                    if '/**' in config and 'ros__parameters' in config['/**']:
                        params = config['/**']['ros__parameters']
                        can_name = params.get(arm_key, '')
                        if can_name:
                            return can_name
                    
                    break
        except Exception as e:
            print(f"Warning: Could not load CAN config: {e}")
        
        # Fallback to default naming
        return f"arm_{side}"
    
    # ArmRecorder-compatible interface
    def get_qpos(self):
        """Get joint positions (compatible with ArmRecorder interface)"""
        try:
            joint_msgs = self.piper.GetArmJointMsgs()
            joint_state = joint_msgs.joint_state
            
            # Extract timestamp if available from SDK
            try:
                if hasattr(joint_msgs, 'timestamp'):
                    self._latest_joint_timestamp = joint_msgs.timestamp
                elif hasattr(joint_state, 'timestamp'):
                    self._latest_joint_timestamp = joint_state.timestamp
                else:
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
        except Exception as e:
            print(f"Warning: Error getting qpos from Piper: {e}")
            return [0.0] * self._dof
        
        print(self.piper.GetArmStatus())
    
    def get_qvel(self):
        """Get joint velocities (compatible with ArmRecorder interface)"""
        try:
            info_msgs = self.piper.GetArmHighSpdInfoMsgs()
            
            # Extract timestamp if available from SDK
            try:
                if hasattr(info_msgs, 'timestamp'):
                    self._latest_velocity_timestamp = info_msgs.timestamp
                else:
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
        except Exception as e:
            print(f"Warning: Error getting qvel from Piper: {e}")
            return [0.0] * self._dof
    
    def get_effort(self):
        """Get joint efforts/torques (compatible with ArmRecorder interface)"""
        try:
            info_msgs = self.piper.GetArmHighSpdInfoMsgs()
            
            # Extract timestamp if available from SDK
            try:
                if hasattr(info_msgs, 'timestamp'):
                    self._latest_effort_timestamp = info_msgs.timestamp
                else:
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
        except Exception as e:
            print(f"Warning: Error getting effort from Piper: {e}")
            return [0.0] * self._dof
    
    def get_cmd_pos(self):
        """Get command positions (compatible with ArmRecorder interface)"""
        try:
            ctrl_msgs = self.piper.GetArmJointCtrl()
            joint_ctrl = ctrl_msgs.joint_ctrl
            
            # Extract timestamp if available from SDK
            try:
                if hasattr(ctrl_msgs, 'timestamp'):
                    self._latest_control_timestamp = ctrl_msgs.timestamp
                elif hasattr(joint_ctrl, 'timestamp'):
                    self._latest_control_timestamp = joint_ctrl.timestamp
                else:
                    self._latest_control_timestamp = time.time()
            except Exception:
                self._latest_control_timestamp = time.time()
            
            return [
                round(joint_ctrl.joint_1 * 0.001, 3),
                round(joint_ctrl.joint_2 * 0.001, 3),
                round(joint_ctrl.joint_3 * 0.001, 3),
                round(joint_ctrl.joint_4 * 0.001, 3),
                round(joint_ctrl.joint_5 * 0.001, 3),
                round(joint_ctrl.joint_6 * 0.001, 3)
            ]
        except Exception as e:
            # Fallback: use current joint positions if commands not available
            try:
                return self.get_qpos()
            except:
                return [0.0] * self._dof
    
    def get_dof(self) -> int:
        """Get degrees of freedom"""
        return self._dof
    
    def get_representative_timestamp(self) -> float:
        """Get most recent timestamp from any data source"""
        # Return the most recent timestamp from any source
        timestamps = [
            self._latest_joint_timestamp,
            self._latest_effort_timestamp,
            self._latest_velocity_timestamp,
            self._latest_control_timestamp
        ]
        valid_timestamps = [ts for ts in timestamps if ts is not None]
        if valid_timestamps:
            return max(valid_timestamps)
        return time.time()
    
    def enable_for_inference(self):
        """Enable Piper arm for sending commands (inference mode)"""
        if not self._enabled:
            while not self.piper.EnablePiper():
                time.sleep(0.01)
            self._enabled = True
            print(f"✓ Piper arm enabled for inference mode")
    
    def shutdown(self):
        """Shutdown the Piper recorder"""
        # Piper SDK may not need explicit shutdown, but add if needed
        pass


    # 注意！传参时数值必须除以1000
    def set_joints(self, position):
        # position = [0,0,0,0,0,0,0]
        # position = [0.2,0.2,-0.2,0.3,-0.2,0.5,0.08]

        # 如果没有开启松灵机械臂，就循环等待
        while( not self.piper.EnablePiper()):
            time.sleep(0.01)
        self.piper.GripperCtrl(0,1000,0x01, 0)
        # factor = 57295.7795 #1000*180/3.1415926
        # factor =1
        factor = 1000
    
        joint_0 = round(position[0]*factor)
        joint_1 = round(position[1]*factor)
        joint_2 = round(position[2]*factor)
        joint_3 = round(position[3]*factor)
        joint_4 = round(position[4]*factor)
        joint_5 = round(position[5]*factor)
        joint_6 = round(position[6]*1000*1000)
        # piper.MotionCtrl_1()
        self.piper.MotionCtrl_2(0x01, 0x01, 30, 0x00)
        print(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)
        self.piper.JointCtrl(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)
        self.piper.GripperCtrl(abs(joint_6), 1000, 0x01, 0)
        # print(self.piper.GetArmStatus())
        # print(position)
        time.sleep(0.005)
        