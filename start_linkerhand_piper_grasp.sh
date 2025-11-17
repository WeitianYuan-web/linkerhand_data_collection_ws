#初始化设置
source ~/.venv/data_collection/bin/activate
colcon build --symlink-install
source /opt/ros/jazzy/setup.bash && source install/setup.bash

# 所有配置在task_config.json中
cat task_config.json

# 配置灵巧手型号和左右手根据task_config.json
bash scripts/apply_task_config.sh

# 注意：如果使用ExHand外骨骼手套，需要先启动exhand_read节点：
 ros2 launch exhand_read exhand.launch.py port:=/dev/ttyUSB0 baudrate:=1152000 enable_mapping_push:=true

# 展示目前的can接口
ip link show type can

# 启动linkerhand sdk 正常启动（使用缓存配置scripts/hand_can_config.yaml） <password> = linkerhand
bash scripts/quick_start_hand.sh linkerhand
# 重新插拔CAN设备后：
bash scripts/quick_start_hand.sh linkerhand --force

# 检测摄像头
python3 scripts/detect_cameras.sh
# 启动摄像头
ros2 launch linkerhand_data_collection_srv multi_camera_launch.py
# （可选）启动rviz2查看摄像头
rviz2

# 配置task config
python3 scripts/update_task_configs.py

#启动数据采集后台服务
ros2 run linkerhand_data_collection_srv linkerhand_data_collection.py

#创建piper单手采集任务
ros2 service call /linkerhand_data_collection_srv linkerhand_data_collection_srv/srv/Internal "req: '{\"method\": \"create_task\", \"id\": 121212, \"params\": {\"task_name\":\"linkerhand_piper_grasp\",\"task_id\":2,\"task_configs_name\":\"linkerhand_piper_grasp\"}}'"

# 开始录制（推荐）
bash scripts/manual_recording_control.sh start linkerhand_piper_grasp

# 停止录制（在需要时执行）
bash scripts/manual_recording_control.sh stop

# 查询录制状态
bash scripts/manual_recording_control.sh status

# 调试灵巧手
ros2 launch gui_control gui_control.launch.py 


# ubuntu 24.04遥控操作安装用方法2
https://hs7ghlauag.feishu.cn/docx/JLAsdoZnBohFtHx8NE2cA13hnLd

# 解压数据，进行查看
python3 scripts/extract_episode_data.py collection_data/linkerhand_piper_grasp/session_2025-10-10_20-45-22/episode_000000