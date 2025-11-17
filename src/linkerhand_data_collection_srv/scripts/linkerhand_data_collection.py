#!/usr/bin/env python3
# -*-coding:utf8-*-
import os,sys,time,json,rclpy,random
import threading
import subprocess
import signal
from pathlib import Path
import numpy as np
import argparse
from datetime import datetime
from typing import Optional
from rclpy.node import Node
from utils.color_msg import ColorMsg
# Avoid name collision with module-level main()
from utils.record_episodes import main as record_episodes_main
from utils.record_episodes_video import main as record_episodes_video_main
from utils.config_loader import (
    DATA_DIR,
    get_device_info,
    build_runtime_config,
)
from utils.file_control import FileControl
from utils.task_controller import TaskController
from linkerhand_data_collection_srv.srv import Internal

# Time synchronization imports
try:
    from utils.time_sync import TimeSyncManager
    TIME_SYNC_AVAILABLE = True
except ImportError:
    TIME_SYNC_AVAILABLE = False
    print("Warning: Time synchronization not available")


class LinkerHandDataCollection(Node):
    def __init__(self):
        super().__init__('linkerhand_data_collection_server')
        self.data = {}
        self.data["id"] = 0
        self.data["msg"] = ""
        self.data["code"] = -1
        self.data["params"] = {}
        self.task = TaskController()
        self.file_control = FileControl()
        device_info = get_device_info()
        self.device_number = device_info.get('device_number', '123456')
        
        # RealSense camera process management
        self.realsense_process = None
        self.camera_auto_start = False  # Don't auto-start on init, wait for task info
        self.current_camera_config = None  # Store current camera configuration
        
        # Manual recording control
        self.recording_active = False
        self.recording_stop_flag = False
        self.current_recording_thread = None
        self.recording_lock = threading.Lock()  # 添加线程锁，防止重复启动录制
        
        # Initialize time synchronization (will be updated when task is loaded)
        self.time_sync_manager = None
        self.current_max_timestamp_diff = 0.1  # Default value
        if TIME_SYNC_AVAILABLE:
            # Use default initially, will be updated when task is loaded
            self.time_sync_manager = TimeSyncManager(max_timestamp_diff=self.current_max_timestamp_diff)
            ColorMsg(msg=f"时间同步功能已启用 (默认容差: {self.current_max_timestamp_diff*1000}ms)", color="green")
        else:
            ColorMsg(msg="时间同步功能未启用", color="yellow")
        
        ColorMsg(msg="数据采集服务已准备就绪", color="green")
        self.server = self.create_service(Internal, "linkerhand_data_collection_srv", self.server_cb)

    def update_time_sync_from_task(self, task_name: str):
        """
        从任务配置更新时间同步容差和时间源
        :param task_name: 任务名称
        """
        if not self.time_sync_manager:
            return
        
        try:
            runtime_config = build_runtime_config(task_name)
            max_timestamp_diff = runtime_config.get('collection', {}).get('max_timestamp_diff', 0.1)
            use_system_time = runtime_config.get('collection', {}).get('use_system_time', False)
            
            # Update time sync manager tolerance
            self.time_sync_manager.max_timestamp_diff = max_timestamp_diff
            self.time_sync_manager.use_system_time = use_system_time
            self.current_max_timestamp_diff = max_timestamp_diff
            
            time_source = "系统时间" if use_system_time else "ROS2时间"
            ColorMsg(msg=f"时间同步已更新: 容差={max_timestamp_diff*1000}ms, 时间源={time_source}", color="green")
        except Exception as e:
            ColorMsg(msg=f"更新时间同步配置失败: {e}", color="yellow")
    
    def start_realsense_camera(self, task_name: str = None, camera_config: dict = None):
        """
        Start RealSense camera based on task configuration.
        
        Args:
            task_name: Task name to load camera config from YAML
            camera_config: Direct camera configuration dict (overrides task_name)
        """
        try:
            ColorMsg(msg="正在检查RealSense摄像头状态...", color="blue")
            
            # Check if RealSense camera is already running with correct config
            if self.is_realsense_running():
                ColorMsg(msg="RealSense摄像头已在运行，跳过启动", color="green")
                return True
            
            # Determine camera configuration
            if camera_config is None and task_name:
                runtime_config = build_runtime_config(task_name)
                camera_config = runtime_config.get('cameras', {})
                # backward compatibility for start_realsense_camera expectations
                preset = camera_config.get('preset', 'intel_d455_single_top')
                camera_config = {
                    'preset': preset,
                    'enable_d455': any(cam['name'] == 'cam_top' for cam in camera_config.get('cameras', [])),
                    'enable_d405_left': any(cam['name'] == 'cam_left_wrist' for cam in camera_config.get('cameras', [])),
                    'enable_d405_right': any(cam['name'] == 'cam_right_wrist' for cam in camera_config.get('cameras', [])),
                }
            
            if camera_config is None:
                # Default: single top camera
                camera_config = {
                    'preset': 'intel_d455_single_top',
                    'enable_d455': True,
                    'enable_d405_left': False,
                    'enable_d405_right': False
                }
            
            self.current_camera_config = camera_config
            
            # Build launch command with camera enables
            cmd = [
                "ros2", "launch", 
                "linkerhand_data_collection_srv", "multi_camera_launch.py",
                f"enable_d455:={'true' if camera_config['enable_d455'] else 'false'}",
                f"enable_d405_left:={'true' if camera_config['enable_d405_left'] else 'false'}",
                f"enable_d405_right:={'true' if camera_config['enable_d405_right'] else 'false'}"
            ]
            
            # Display which cameras are being started
            cameras_enabled = []
            if camera_config['enable_d455']:
                cameras_enabled.append("D455顶部")
            if camera_config['enable_d405_left']:
                cameras_enabled.append("D405左腕")
            if camera_config['enable_d405_right']:
                cameras_enabled.append("D405右腕")
            
            if not cameras_enabled:
                ColorMsg(msg="任务配置为无相机模式", color="yellow")
                return True
            
            cameras_str = " + ".join(cameras_enabled)
            ColorMsg(msg=f"正在启动相机: {cameras_str} (预设: {camera_config['preset']})", color="blue")
            
            self.realsense_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid  # Create new process group
            )
            
            # Wait for cameras to initialize
            ColorMsg(msg="等待相机初始化...", color="blue")
            time.sleep(8.0 if len(cameras_enabled) >= 3 else 5.0)
            
            # Check if cameras started successfully
            if self.is_realsense_running():
                ColorMsg(msg=f"相机启动成功，正在预热...", color="green")
                time.sleep(3.0)
                ColorMsg(msg="相机预热完成", color="green")
                return True
            else:
                ColorMsg(msg="相机启动失败", color="red")
                return False
                
        except Exception as e:
            ColorMsg(msg=f"启动相机时出错: {e}", color="red")
            return False

    def stop_realsense_camera(self):
        """Stop RealSense camera."""
        try:
            if self.realsense_process:
                ColorMsg(msg="正在停止RealSense摄像头...", color="blue")
                
                # Kill the process group
                os.killpg(os.getpgid(self.realsense_process.pid), signal.SIGTERM)
                
                # Wait for process to terminate
                try:
                    self.realsense_process.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    # Force kill if it doesn't terminate gracefully
                    os.killpg(os.getpgid(self.realsense_process.pid), signal.SIGKILL)
                
                self.realsense_process = None
                ColorMsg(msg="RealSense摄像头已停止", color="green")
                return True
            else:
                ColorMsg(msg="RealSense摄像头未运行", color="yellow")
                return True
                
        except Exception as e:
            ColorMsg(msg=f"停止RealSense摄像头时出错: {e}", color="red")
            return False

    def is_realsense_running(self):
        """Check if RealSense camera is running by checking for topics."""
        try:
            # Create a temporary node to check topics
            temp_node = rclpy.create_node('realsense_checker')
            topic_names_and_types = temp_node.get_topic_names_and_types()
            temp_node.destroy_node()
            
            # Check for RealSense camera topics
            realsense_topics = [name for name, types in topic_names_and_types 
                              if 'camera' in name.lower() and 'image_raw' in name]
            
            return len(realsense_topics) > 0
            
        except Exception as e:
            print(f"检查RealSense状态时出错: {e}")
            return False

    def server_cb(self, request, response):
        
        result = json.loads(request.req)
        # print(result["method"])
        self.data["method"] = result["method"]
        
        try:
            self.data["params"]["task_id"] =result["params"]["task_id"]
            file_name = result["params"]["name"]
        except:
            pass
        self.data["id"] = result["id"]
        self.data["timestamp"] = time.time()
        self.data["ros_time"] = self.get_clock().now().nanoseconds / 1e9
        # 判断参数是否正确
        if result.get("method") is None or result.get("id") is None or result.get("params") is None:
            self.data["msg"] = "缺少必要参数，请查看手册"
            ColorMsg(msg=self.data["msg"], color="red")
            response.resp = json.dumps(self.data, ensure_ascii=False)
            return response
        ''' ---------------------------Mqtt通讯---START----------------------------- '''
        if result["method"] == "create_task":  # 创建任务
            # rosservice call /linkerhand_data_collection_srv "req: '{\"method\": \"create_task\", \"id\": 121212, \"params\": {\"task_name\":\"data_collection_wear_shoe\",\"task_id\":1,\"task_configs_name\":\"data_collection_wear_shoe\"}}'"
            task_name = result["params"]["task_name"]
            task_id = result["params"]["task_id"]
            t_n = self.task.get_data("task", ["task_name"], "task_name = ?", (task_name,))
            if len(t_n) > 0:
                self.data["id"] = result["id"]
                self.data["params"] = result["params"]
                self.data["code"] = 99
                self.data["msg"] = "要创建的任务已经存在"
                response.resp = json.dumps(self.data, ensure_ascii=False)
                return response
            now = datetime.now()
            session_timestamp = f"{now.year:04d}-{now.month:02d}-{now.day:02d}_{now.hour:02d}-{now.minute:02d}-{now.second:02d}"
            
            # New structure: task_name/session_timestamp/
            task_base_path = task_name  # e.g., "data_collection_wear_shoe"
            session_path = f"session_{session_timestamp}"  # e.g., "session_2025-01-08_14-30-15"
            full_task_path = os.path.join(task_base_path, session_path)  # e.g., "data_collection_wear_shoe/session_2025-01-08_14-30-15"
            
            task_configs_name = result["params"]["task_configs_name"]
            create_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
            
            # Create task base directory and session directory
            task_dir = self.file_control.create_directory_tree(base_path=DATA_DIR, sub_dir_path=full_task_path)
            if task_dir == 1:
                # Store the full path for the database (session path)
                t = self.task.write_data("task", {"task_name": task_name,"task_id":task_id, "task_path": full_task_path, "task_configs_name": task_configs_name, "create_time": create_time, "state": 0})
                if t == 1:
                    self.data["id"] = result["id"]
                    self.data["params"] = result["params"]
                    self.data["code"] = 0
                    self.data["msg"] = "任务目录创建成功"
                    response.resp = json.dumps(self.data, ensure_ascii=False)
                    return response
                else:
                    self.data["id"] = result["id"]
                    self.data["params"] = result["params"]
                    self.data["code"] = 99
                    self.data["msg"] = "任务目录创建失败"
                    response.resp = json.dumps(self.data, ensure_ascii=False)
                    return response
            else:
                self.data["id"] = result["id"]
                self.data["params"] = result["params"]
                self.data["code"] = 99
                self.data["msg"] = "任务目录创建失败"
                response.resp = json.dumps(self.data, ensure_ascii=False)
                return response
        if result["method"] == "get_task_list":
            # rosservice call /linkerhand_data_collection_srv "req: '{\"method\": \"get_task_list\", \"id\": 121212, \"params\": {}}'"
            task_list = self.task.sql.select("task")
            self.data["id"] = result["id"]
            self.data["params"] = result["params"]
            self.data["params"]["task_list"] = task_list
            self.data["code"] = 0
            self.data["msg"] = "任务列表获取成功"
            response.resp = json.dumps(self.data, ensure_ascii=False)
            return response
        if result["method"] == "delete_task":
            # rosservice call /linkerhand_data_collection_srv "req: '{\"method\": \"delete_task\", \"id\": 121212, \"params\": {\"task_name\":\"data_collection_wear_shoe\"}}'"
            task_name = result["params"]["task_name"]
            
            # Check if task exists
            existing_task = self.task.get_data("task", ["task_name", "task_path"], "task_name = ?", (task_name,))
            if len(existing_task) == 0:
                self.data["id"] = result["id"]
                self.data["params"] = result["params"]
                self.data["code"] = 99
                self.data["msg"] = "要删除的任务不存在"
                response.resp = json.dumps(self.data, ensure_ascii=False)
                return response
            
            # Get task path for directory deletion
            task_path = existing_task[0]["task_path"]
            full_path = os.path.join(DATA_DIR, task_path)
            
            try:
                # Delete task from current database
                delete_result = self.task.sql.delete("task", "task_name = ?", (task_name,))
                
                # Also delete from source database if we're running from installed package
                from utils.db.sqlite_manager import SQLiteManager
                from utils.config_loader import workspace_root
                source_db_path = os.path.join(workspace_root, "src", "linkerhand_data_collection_srv", "scripts", "utils", "db", "linkerhand_data_collection_database.db")
                if os.path.exists(source_db_path) and source_db_path != self.task.sql.db_path:
                    source_sql = SQLiteManager(source_db_path)
                    source_sql.delete("task", "task_name = ?", (task_name,))
                    ColorMsg(msg=f"已从源数据库删除任务: {source_db_path}", color="yellow")
                
                # Delete task directory and all its contents
                import shutil
                if os.path.exists(full_path):
                    shutil.rmtree(full_path)
                    ColorMsg(msg=f"已删除任务目录: {full_path}", color="yellow")
                
                self.data["id"] = result["id"]
                self.data["params"] = result["params"]
                self.data["code"] = 0
                self.data["msg"] = f"任务 '{task_name}' 删除成功"
                ColorMsg(msg=self.data["msg"], color="green")
                response.resp = json.dumps(self.data, ensure_ascii=False)
                return response
                
            except Exception as e:
                self.data["id"] = result["id"]
                self.data["params"] = result["params"]
                self.data["code"] = 99
                self.data["msg"] = f"删除任务失败: {str(e)}"
                ColorMsg(msg=self.data["msg"], color="red")
                response.resp = json.dumps(self.data, ensure_ascii=False)
                return response
        if result["method"] == "get_data_list":  # 获取任务下采集的数据
            # rosservice call /linkerhand_data_collection_srv "req: '{\"method\": \"get_data_list\", \"id\": 121212, \"params\": {\"task_name\":\"data_collection_wear_shoe\"}}'"
            task_name = result["params"]["task_name"]
            tmp_d = self.task.get_data("task", ["task_path"], "task_name=?", (task_name,))
            if len(tmp_d) > 0:
                task_path = tmp_d[0]["task_path"]
                base_path=DATA_DIR+"/"+task_path
                all_collection = self.file_control.list_directory_to_json(directory_path=base_path)
                self.data["id"] = result["id"]
                self.data["params"] = result["params"]
                self.data["code"] = 0
                self.data["msg"] = "目录获取成功"
                self.data["params"]["all_collection"] = all_collection
                response.resp = json.dumps(self.data, ensure_ascii=False)
                return response
            else:
                self.data["id"] = result["id"]
                self.data["params"] = result["params"]
                self.data["code"] = 99
                self.data["msg"] = "没有这个任务"
                response.resp = json.dumps(self.data, ensure_ascii=False)
                return response
        if result["method"] == "create_directory":
            # rosservice call /linkerhand_data_collection_srv "req: '{\"method\": \"create_directory\", \"id\": 121212, \"params\": {\"path\":\"\/Python\/LinkerHand_Python_SDK\/example\",\"file\":\"test.yaml\"}}'"
            path = result["params"]["path"]
            file = result["params"]["file"]
            dir = self.file_control.create_directory_tree(sub_dir_path=path,file=file)
            if dir == 99:
                self.data["id"] = result["id"]
                self.data["params"] = result["params"]
                self.data["code"] = 99
                self.data["msg"] = "文件夹创建失败"
            if dir == 1 or dir == 0:
                self.data["id"] = result["id"]
                self.data["params"] = result["params"]
                self.data["code"] = dir
                self.data["msg"] = "文件夹创建成功"
            response.resp = json.dumps(self.data, ensure_ascii=False)
            return response
        if result["method"] == "gather_hdf5":  #开始按照文件夹路径采集hdf5,如果文件夹路径不存在则创建
            # rosservice call /linkerhand_data_collection_srv "req: '{\"method\": \"gather_hdf5\", \"id\": 121212, \"params\": {\"task_name\":\"data_collection_wear_shoe\",\"episode_idx\":8}}'"
            # rosservice call /linkerhand_data_collection_srv "req: '{\"method\": \"gather_hdf5\", \"id\": 121212, \"params\": {\"task_name\":\"data_collection_wear_shoe\"}}'"  # Auto episode numbering
            # rosservice call /linkerhand_data_collection_srv "req: '{\"method\": \"gather_hdf5\", \"id\": 121212, \"params\": {\"task_name\":\"data_collection_wear_shoe\",\"use_timestamp\":true}}'"  # Use timestamp naming
            task_list = self.task.get_data("task", ["task_name"], "task_name=?", (result["params"]["task_name"],))
            if len(task_list) == 0:
                self.data["id"] = result["id"]
                self.data["params"] = result["params"]
                self.data["code"] = 99
                self.data["msg"] = "任务不存在"
                response.resp = json.dumps(self.data, ensure_ascii=False)
                ColorMsg(msg=self.data["msg"], color="red")
                return response
            else:
                task_name = task_list[0]["task_name"]
            
            # Update time sync tolerance from task config
            self.update_time_sync_from_task(task_name)
            
            # Check time synchronization before starting data collection
            if self.time_sync_manager:
                ColorMsg(msg="检查时间同步状态...", color="blue")
                if not self.time_sync_manager.wait_for_synchronized_time(timeout=5.0):
                    ColorMsg(msg="警告: 时间同步未就绪，但仍将继续采集", color="yellow")
                else:
                    ColorMsg(msg="时间同步已就绪", color="green")
            
            # Handle episode_idx - make it optional
            episode_idx = result["params"].get("episode_idx")
            use_timestamp = result["params"].get("use_timestamp", False)
            
            # Determine episode naming strategy
            if use_timestamp:
                # Use timestamp-based naming
                now = datetime.now()
                episode_idx = f"timestamp_{now.strftime('%Y%m%d_%H%M%S_%f')[:-3]}"  # Include milliseconds
                ColorMsg(msg=f"使用时间戳命名: {episode_idx}", color="blue")
            elif episode_idx is not None:
                # Use explicit episode index
                episode_idx = int(episode_idx)
                ColorMsg(msg=f"使用指定episode编号: {episode_idx}", color="blue")
            else:
                # Auto-generate episode index
                episode_idx = None  # Will be auto-generated in the run method
                ColorMsg(msg="使用自动episode编号", color="blue")
            
            path_list = self.task.get_data("task", ["task_path"], "task_name=?", (task_name,))
            task_path = DATA_DIR + "/" + path_list[0]["task_path"]
            # Run capture in background so executor stays free to process image callbacks
            threading.Thread(target=self.run, kwargs={
                'task_name': task_name,
                'task_path': task_path,
                'episode_idx': episode_idx,
                'use_timestamp': use_timestamp
            }, daemon=True).start()
            self.data["id"] = result["id"]
            self.data["params"] = result["params"]
            self.data["code"] = 0
            self.data["msg"] = "开始采集"
            response.resp = json.dumps(self.data, ensure_ascii=False)
            return response
        if result["method"] == "get_all_collection":  #获取指定文件夹下所有目录结构
            # rosservice call /linkerhand_data_collection_srv "req: '{\"method\": \"get_all_collection\", \"id\": 121212, \"params\": {\"path\":\"you path\"}}'"
            base_path=DATA_DIR+result["params"]["path"]
            all_collection = self.file_control.list_directory_to_json(directory_path=base_path)
            self.data["id"] = result["id"]
            self.data["params"] = result["params"]
            
            self.data["params"]["all_collection"] = all_collection
            self.data["code"] = 0
            self.data["msg"] = "目录获取成功"
            #print(json.dumps(self.data, ensure_ascii=False))
            
            response.resp = json.dumps(self.data, ensure_ascii=False)
            return response
        if result["method"] == "upload_file":
            try:
                id = result["id"]
                params = result["params"]
                path = result["params"]["path"] # 要上传的路径
                datasetPath = result["params"]["datasetPath"] # 要上传到华为云的路径
                dic, url = self.file_control.up_load(path=path,datasetPath=datasetPath)
                dic["url"] = url
                response.resp = json.dumps(dic, ensure_ascii=False)
                return response
            except:
                ColorMsg(msg="缺少参数",color="red")
                self.data["msg"] = "缺少参数"
                self.data["code"] = 1
                self.data["params"] = result["params"]
                self.data["id"] = result["id"]
                response.resp = json.dumps(self.data, ensure_ascii=False)
                return response
        if result["method"] == "check_time_sync":  # 检查时间同步状态
            # rosservice call /linkerhand_data_collection_srv "req: '{\"method\": \"check_time_sync\", \"id\": 121212, \"params\": {}}'"
            if self.time_sync_manager:
                is_ready = self.time_sync_manager.wait_for_synchronized_time(timeout=2.0)
                current_time = self.time_sync_manager.get_current_time()
                self.data["id"] = result["id"]
                self.data["params"] = result["params"]
                self.data["params"]["time_sync_ready"] = is_ready
                self.data["params"]["current_time"] = current_time
                self.data["params"]["max_timestamp_diff"] = self.time_sync_manager.max_timestamp_diff
                self.data["code"] = 0
                self.data["msg"] = "时间同步状态检查完成"
                ColorMsg(msg=f"时间同步状态: {'就绪' if is_ready else '未就绪'}", color="green" if is_ready else "yellow")
            else:
                self.data["id"] = result["id"]
                self.data["params"] = result["params"]
                self.data["params"]["time_sync_ready"] = False
                self.data["params"]["current_time"] = time.time()
                self.data["code"] = 99
                self.data["msg"] = "时间同步功能未启用"
                ColorMsg(msg="时间同步功能未启用", color="red")
            response.resp = json.dumps(self.data, ensure_ascii=False)
            return response
        
        if result["method"] == "get_time_sync_stats":  # 获取时间同步统计信息
            # rosservice call /linkerhand_data_collection_srv "req: '{\"method\": \"get_time_sync_stats\", \"id\": 121212, \"params\": {\"task_name\":\"data_collection_wear_shoe\"}}'"
            task_name = result["params"]["task_name"]
            task_list = self.task.get_data("task", ["task_path"], "task_name=?", (task_name,))
            
            if len(task_list) == 0:
                self.data["id"] = result["id"]
                self.data["params"] = result["params"]
                self.data["code"] = 99
                self.data["msg"] = "任务不存在"
                response.resp = json.dumps(self.data, ensure_ascii=False)
                return response
            
            task_path = task_list[0]["task_path"]
            base_path = DATA_DIR + "/" + task_path
            
            # Analyze synchronization data from HDF5 files
            sync_stats = self.analyze_sync_stats(base_path)
            
            self.data["id"] = result["id"]
            self.data["params"] = result["params"]
            self.data["params"]["sync_stats"] = sync_stats
            self.data["code"] = 0
            self.data["msg"] = "时间同步统计信息获取成功"
            response.resp = json.dumps(self.data, ensure_ascii=False)
            return response
        
        if result["method"] == "start_manual_recording":  # 开始手动录制
            # rosservice call /linkerhand_data_collection_srv "req: '{\"method\": \"start_manual_recording\", \"id\": 121212, \"params\": {\"task_name\":\"linkerhand_piper_grasp\", \"use_timestamp\":true}}'"
            task_name = result["params"]["task_name"]
            episode_idx = result["params"].get("episode_idx")
            use_timestamp = result["params"].get("use_timestamp", False)
            
            # 检查任务是否存在
            task_list = self.task.get_data("task", ["task_name"], "task_name=?", (task_name,))
            if len(task_list) == 0:
                self.data["id"] = result["id"]
                self.data["params"] = result["params"]
                self.data["code"] = 99
                self.data["msg"] = "任务不存在"
                response.resp = json.dumps(self.data, ensure_ascii=False)
                return response
            
            # 检查是否已有录制在进行
            if self.recording_active:
                self.data["id"] = result["id"]
                self.data["params"] = result["params"]
                self.data["code"] = 99
                self.data["msg"] = "录制已在进行中，请先停止当前录制"
                response.resp = json.dumps(self.data, ensure_ascii=False)
                return response
            
            # 获取任务路径
            path_list = self.task.get_data("task", ["task_path", "task_configs_name"], "task_name=?", (task_name,))
            task_path = DATA_DIR + "/" + path_list[0]["task_path"]
            task_configs_name = path_list[0].get("task_configs_name", task_name)
            
            # 开始手动录制
            success, msg = self.start_manual_recording(task_name, task_path, episode_idx, use_timestamp)
            self.data["id"] = result["id"]
            self.data["params"] = result["params"]
            self.data["code"] = 0 if success else 99
            self.data["msg"] = msg
            response.resp = json.dumps(self.data, ensure_ascii=False)
            return response
        
        if result["method"] == "stop_manual_recording":  # 停止手动录制
            # rosservice call /linkerhand_data_collection_srv "req: '{\"method\": \"stop_manual_recording\", \"id\": 121212, \"params\": {}}'"
            success, msg = self.stop_manual_recording()
            self.data["id"] = result["id"]
            self.data["params"] = result["params"]
            self.data["code"] = 0 if success else 99
            self.data["msg"] = msg
            response.resp = json.dumps(self.data, ensure_ascii=False)
            return response
        
        if result["method"] == "recording_status":  # 查询录制状态
            # rosservice call /linkerhand_data_collection_srv "req: '{\"method\": \"recording_status\", \"id\": 121212, \"params\": {}}'"
            self.data["id"] = result["id"]
            self.data["params"] = result["params"]
            self.data["params"]["recording_active"] = self.recording_active
            self.data["params"]["recording_stop_flag"] = self.recording_stop_flag
            self.data["code"] = 0
            self.data["msg"] = f"录制状态: {'进行中' if self.recording_active else '未进行'}"
            response.resp = json.dumps(self.data, ensure_ascii=False)
            return response
        
        ''' ---------------------------Mqtt通讯-EDN------------------------------- '''


    def check_episode_idx(self, episode_idx, task_name: str) -> bool:
        """
        检查 episode_idx 是否存在
        :param episode_idx: 需要检查的 episode_idx (可以是int或str)
        :param task_name: 任务名称
        :return: True 如果有效（不存在冲突），否则 False
        """
        # 检查 episode_idx 是否存在
        if isinstance(episode_idx, str) and episode_idx.startswith("timestamp_"):
            # For timestamp-based naming, we don't need to check for conflicts
            # as timestamps are unique by nature
            return True
        
        # For numeric episode indices, check if file already exists
        episode_path = os.path.join(DATA_DIR+"/"+task_name, f"episode_{episode_idx}.hdf5")
        print(episode_path)
        if not os.path.exists(episode_path):
            print(f"Episode index {episode_idx} does not exist.")
            return True

        return False
    
    def analyze_sync_stats(self, base_path: str) -> dict:
        """
        分析HDF5文件中的时间同步统计信息
        :param base_path: 基础路径
        :return: 同步统计信息字典
        """
        import h5py
        import glob
        
        sync_stats = {
            "total_episodes": 0,
            "valid_sync_episodes": 0,
            "invalid_sync_episodes": 0,
            "avg_max_diff": 0.0,
            "max_diff_range": [0.0, 0.0],
            "sync_issues": []
        }
        
        # Find all HDF5 files
        hdf5_files = glob.glob(os.path.join(base_path, "**/*.hdf5"), recursive=True)
        
        if not hdf5_files:
            return sync_stats
        
        total_max_diffs = []
        
        for hdf5_file in hdf5_files:
            try:
                with h5py.File(hdf5_file, 'r') as f:
                    sync_stats["total_episodes"] += 1
                    
                    # Check if sync validation data exists
                    if 'sync_validation' in f:
                        sync_group = f['sync_validation']
                        
                        if 'is_valid' in sync_group and 'max_diff' in sync_group:
                            is_valid = sync_group['is_valid'][:]
                            max_diffs = sync_group['max_diff'][:]
                            
                            # Count valid/invalid episodes
                            episode_valid = np.all(is_valid)
                            if episode_valid:
                                sync_stats["valid_sync_episodes"] += 1
                            else:
                                sync_stats["invalid_sync_episodes"] += 1
                                sync_stats["sync_issues"].append({
                                    "file": os.path.basename(hdf5_file),
                                    "invalid_timesteps": np.sum(~is_valid),
                                    "total_timesteps": len(is_valid)
                                })
                            
                            # Collect max diff statistics
                            episode_max_diff = np.max(max_diffs)
                            total_max_diffs.append(episode_max_diff)
                            
            except Exception as e:
                ColorMsg(msg=f"分析文件 {hdf5_file} 时出错: {str(e)}", color="red")
                continue
        
        # Calculate statistics
        if total_max_diffs:
            sync_stats["avg_max_diff"] = np.mean(total_max_diffs)
            sync_stats["max_diff_range"] = [np.min(total_max_diffs), np.max(total_max_diffs)]
        
        return sync_stats

    def start_manual_recording(self, task_name: str, task_path: str, episode_idx=None, use_timestamp: bool = False):
        """
        开始手动录制（线程安全版本）
        :param task_name: 任务名称
        :param task_path: 任务路径
        :param episode_idx: episode索引
        :param use_timestamp: 是否使用时间戳命名
        :return: (success, message)
        """
        # 使用锁确保线程安全，防止重复启动录制
        with self.recording_lock:
            if self.recording_active:
                return False, "录制已在进行中，请先停止当前录制"
            
            # 立即设置标志，防止其他线程进入
            self.recording_active = True
            self.recording_stop_flag = False
        
        # 在锁外执行耗时操作，避免长时间持有锁
        # Update time sync tolerance from task config
        self.update_time_sync_from_task(task_name)
        
        def stop_callback():
            return self.recording_stop_flag
        
        # 启动录制线程
        self.current_recording_thread = threading.Thread(
            target=self.run_manual_recording,
            kwargs={
                'task_name': task_name,
                'task_path': task_path,
                'episode_idx': episode_idx,
                'use_timestamp': use_timestamp,
                'stop_callback': stop_callback
            },
            daemon=True
        )
        self.current_recording_thread.start()
        ColorMsg(msg="手动录制已开始", color="green")
        return True, "手动录制已开始"
    
    def stop_manual_recording(self):
        """
        停止手动录制
        :return: (success, message)
        """
        if not self.recording_active:
            return False, "没有正在进行的录制"
        
        self.recording_stop_flag = True
        ColorMsg(msg="停止信号已发送", color="yellow")
        return True, "停止信号已发送"
    
    def run_manual_recording(self, task_name: str, task_path: str, episode_idx=None, use_timestamp: bool = False, stop_callback=None):
        """
        运行手动录制
        :param task_name: 任务名称
        :param task_path: 任务路径
        :param episode_idx: episode索引
        :param use_timestamp: 是否使用时间戳命名
        :param stop_callback: 停止回调函数
        :return: None
        """
        try:
            config_list = self.task.get_data("task", ["task_configs_name"], "task_name=?", (task_name,))
            if len(config_list) == 0:
                ColorMsg(msg=f"任务{task_name}不存在", color="red")
                return False
            else:
                task_configs_name = config_list[0]["task_configs_name"]
            
            # Handle episode_idx auto-generation if not provided
            if episode_idx is None and not use_timestamp:
                from utils.record_episodes_video import get_auto_index
                episode_idx = get_auto_index(task_path)
                ColorMsg(msg=f"自动生成episode编号: {episode_idx}", color="green")
            elif use_timestamp and episode_idx is None:
                # Use timestamp-based naming
                now = datetime.now()
                episode_idx = f"timestamp_{now.strftime('%Y%m%d_%H%M%S_%f')[:-3]}"
                ColorMsg(msg=f"使用时间戳命名: {episode_idx}", color="blue")
            
            # 构造参数
            manual_args = {
                "task_name": task_name,
                "task_path": task_path,
                "task_configs_name": task_configs_name,
                "episode_idx": episode_idx,
                "stop_callback": stop_callback
            }
            
            # 模拟 argparse 的返回
            parser = argparse.Namespace(**manual_args)
            
            # 检查任务配置
            task_config = build_runtime_config(task_configs_name)
            use_video_compression = True
            # 选择录制方式
            if use_video_compression:
                ColorMsg(msg="使用视频压缩格式进行手动录制", color="blue")
                rc = record_episodes_video_main(vars(parser), node=self)
            else:
                ColorMsg(msg="使用HDF5格式进行手动录制", color="blue")
                rc = record_episodes_main(vars(parser), node=self)
            
            if rc == True:
                if use_timestamp:
                    ColorMsg(msg=f"任务{task_name} 时间戳数据集 {episode_idx} 录制完成", color="green")
                else:
                    ColorMsg(msg=f"任务{task_name} 第episode_{episode_idx}数据集录制完成", color="green")
            else:
                ColorMsg(msg=f"任务{task_name} 录制失败", color="red")
            
            return rc
            
        except Exception as e:
            ColorMsg(msg=f"手动录制过程中出错: {str(e)}", color="red")
            return False
        finally:
            # 重置录制状态
            self.recording_active = False
            self.recording_stop_flag = False
            self.current_recording_thread = None

    def run(self,task_name: Optional[str] = "data_collection_wear_shoe", task_path: Optional[str] = "",episode_idx: Optional[int] = None, use_timestamp: bool = False):
        """
        运行数据采集
        :param task_name: 任务名称
        :param task_path: 任务路径
        :param episode_idx: 任务索引 (None for auto-generation)
        :param use_timestamp: 是否使用时间戳命名
        :return: None
        """
        config_list = self.task.get_data("task", ["task_configs_name"], "task_name=?", (task_name,))
        if len(config_list) == 0:
            ColorMsg(msg=f"任务{task_name}不存在", color="red")
            return False
        else:
            task_configs_name = config_list[0]["task_configs_name"]
        
        # Handle episode_idx auto-generation if not provided
        if episode_idx is None and not use_timestamp:
            # Import get_auto_index function
            from utils.record_episodes_video import get_auto_index
            episode_idx = get_auto_index(task_path)
            ColorMsg(msg=f"自动生成episode编号: {episode_idx}", color="green")
        
        # 不使用命令行，手动构造参数
        manual_args = {
            "task_name": task_name,
            "task_path": task_path,
            "task_configs_name": task_configs_name,
            "episode_idx": episode_idx
        }

        # 模拟 argparse 的返回
        parser = argparse.Namespace(**manual_args)

        # Check if episode already exists (only for numeric episode_idx)
        if isinstance(episode_idx, int):
            # Check for both HDF5 and video formats
            task_config = build_runtime_config(task_configs_name)
            use_video_compression = task_config.get('use_video_compression', False)
            
            if use_video_compression:
                # Check for video format episode directory
                episode_name = f'episode_{episode_idx:06d}'
                episode_path = os.path.join(task_path, episode_name)
                if os.path.exists(episode_path):
                    ColorMsg(msg=f"任务{task_name}目录下已经存在<{episode_name}>数据集", color="red")
                    return False
            else:
                # Check for HDF5 format
                idx = self.check_episode_idx(episode_idx, task_name)
                if idx == False:
                    ColorMsg(msg=f"任务{task_name}目录下已经存在<episode_{episode_idx}.hdf5>数据集", color="red")
                    return False
        
        # Display appropriate message based on naming strategy
        if use_timestamp:
            ColorMsg(msg=f"开始录制任务{task_name} 时间戳数据集: {episode_idx}", color="green")
        else:
            ColorMsg(msg=f"开始录制任务{task_name} 第episode_{episode_idx}数据集", color="green")
        
        # Pass time sync manager to record_episodes if available
        if self.time_sync_manager:
            ColorMsg(msg="使用时间同步进行数据采集", color="green")
            # The record_episodes function will automatically use the time sync manager
            # since it's already integrated into the RealEnv class
        
        # Check if task config supports video compression
        task_config = build_runtime_config(task_configs_name)
        use_video_compression = task_config.get('use_video_compression', False)
        
        if use_video_compression:
            ColorMsg(msg="使用视频压缩格式进行数据采集", color="blue")
            rc = record_episodes_video_main(vars(parser), node=self)
        else:
            ColorMsg(msg="使用HDF5格式进行数据采集", color="blue")
            rc = record_episodes_main(vars(parser), node=self)
        if rc == True:
            if use_timestamp:
                ColorMsg(msg=f"任务{task_name} 时间戳数据集 {episode_idx} 录制完成", color="green")
            else:
                ColorMsg(msg=f"任务{task_name} 第episode_{episode_idx}数据集录制完成", color="green")
        return rc
        

def main(args=None):
    rclpy.init(args=args)
    # 创建LinkerHandDataCollection实例
    data_collection = LinkerHandDataCollection()

    try:
        # 运行实例
        #data_collection.run()
        rclpy.spin(data_collection)
    except KeyboardInterrupt:
        ColorMsg(msg="收到中断信号，正在关闭服务...", color="yellow")
    finally:
        # 清理资源
        ColorMsg(msg="正在清理资源...", color="blue")
        data_collection.stop_realsense_camera()
        data_collection.destroy_node()
        rclpy.shutdown()
        ColorMsg(msg="服务已关闭", color="green")
    
    # # Use MultiThreadedExecutor to handle callbacks during service calls
    # from rclpy.executors import MultiThreadedExecutor
    # executor = MultiThreadedExecutor()
    # executor.add_node(data_collection)
    
    # try:
    #     executor.spin()
    # except KeyboardInterrupt:
    #     pass
    # finally:
    #     executor.shutdown()
    #     data_collection.destroy_node()
    #     rclpy.shutdown()

if __name__ == "__main__":
    main()