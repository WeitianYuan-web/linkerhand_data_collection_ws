#!/bin/bash

# 手动录制控制脚本
# 使用方法：
# ./manual_recording_control.sh camera start <task_name>  - 保留向后兼容提示（已弃用）
# ./manual_recording_control.sh start <task_name> [use_timestamp]  - 开始录制
# ./manual_recording_control.sh stop                      - 停止录制
# ./manual_recording_control.sh status                    - 查询录制状态

# 加载ROS2环境
if [ -f ~/.venv/data_collection/bin/activate ]; then
    source ~/.venv/data_collection/bin/activate
fi

if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi

if [ -f install/setup.bash ]; then
    source install/setup.bash
fi

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 服务名称
SERVICE_NAME="/linkerhand_data_collection_srv"

# 打印带颜色的消息
print_message() {
    local color=$1
    local message=$2
    echo -e "${color}${message}${NC}"
}

# 开始录制
start_recording() {
    local task_name=$1
    local use_timestamp=$2
    
    if [ -z "$task_name" ]; then
        print_message $RED "错误: 请指定任务名称"
        echo "使用方法: $0 start <task_name> [use_timestamp]"
        exit 1
    fi
    
    # 构建请求参数
    local params="{\"task_name\":\"$task_name\""
    if [ "$use_timestamp" = "true" ]; then
        params="${params},\"use_timestamp\":true"
    fi
    params="${params}}"
    
    print_message $BLUE "开始手动录制任务: $task_name"
    if [ "$use_timestamp" = "true" ]; then
        print_message $BLUE "使用时间戳命名"
    fi
    
    # 发送开始录制请求
    local response=$(ros2 service call $SERVICE_NAME linkerhand_data_collection_srv/srv/Internal "req: '{\"method\": \"start_manual_recording\", \"id\": 121212, \"params\": $params}'")
    
    # 解析响应 - 从resp字段中提取JSON
    local json_response=$(echo "$response" | grep -o "resp='[^']*'" | cut -d"'" -f2)
    
    # 使用Python解析JSON
    local code=$(python3 -c "import json, sys; data=json.loads('$json_response'); print(data.get('code', ''))")
    local msg=$(python3 -c "import json, sys; data=json.loads('$json_response'); print(data.get('msg', ''))")
    
    if [ "$code" = "0" ]; then
        print_message $GREEN "✓ $msg"
    else
        print_message $RED "✗ $msg"
        exit 1
    fi
}

# 停止录制
stop_recording() {
    print_message $YELLOW "发送停止录制信号..."
    
    # 发送停止录制请求
    local response=$(ros2 service call $SERVICE_NAME linkerhand_data_collection_srv/srv/Internal "req: '{\"method\": \"stop_manual_recording\", \"id\": 121212, \"params\": {}}'")
    
    # 解析响应 - 从resp字段中提取JSON
    local json_response=$(echo "$response" | grep -o "resp='[^']*'" | cut -d"'" -f2)
    
    # 使用Python解析JSON
    local code=$(python3 -c "import json, sys; data=json.loads('$json_response'); print(data.get('code', ''))")
    local msg=$(python3 -c "import json, sys; data=json.loads('$json_response'); print(data.get('msg', ''))")
    
    if [ "$code" = "0" ]; then
        print_message $GREEN "✓ $msg"
    else
        print_message $RED "✗ $msg"
        exit 1
    fi
}

# 查询录制状态
check_status() {
    print_message $BLUE "查询录制状态..."
    
    # 发送状态查询请求
    local response=$(ros2 service call $SERVICE_NAME linkerhand_data_collection_srv/srv/Internal "req: '{\"method\": \"recording_status\", \"id\": 121212, \"params\": {}}'")
    
    # 解析响应 - 从resp字段中提取JSON
    local json_response=$(echo "$response" | grep -o "resp='[^']*'" | cut -d"'" -f2)
    
    # 使用Python解析JSON
    local code=$(python3 -c "import json, sys; data=json.loads('$json_response'); print(data.get('code', ''))")
    local msg=$(python3 -c "import json, sys; data=json.loads('$json_response'); print(data.get('msg', ''))")
    local active=$(python3 -c "import json, sys; data=json.loads('$json_response'); params=data.get('params', {}); print(params.get('recording_active', ''))")
    
    if [ "$code" = "0" ]; then
        print_message $GREEN "✓ $msg"
        if [ "$active" = "true" ]; then
            print_message $YELLOW "录制状态: 进行中"
        else
            print_message $BLUE "录制状态: 未进行"
        fi
    else
        print_message $RED "✗ $msg"
        exit 1
    fi
}

# 显示帮助信息
show_help() {
    echo "手动录制控制脚本"
    echo ""
    echo "📷 相机控制:"
    echo "  # 请使用 ROS2 Launch:"
    echo "  ros2 launch linkerhand_data_collection_srv multi_camera_launch.py"
    echo ""
    echo "🎬 录制控制:"
    echo "  $0 start <task_name> [use_timestamp]  - 开始手动录制"
    echo "  $0 stop                               - 停止手动录制"
    echo "  $0 status                             - 查询录制状态"
    echo ""
    echo "❓ 帮助:"
    echo "  $0 help                               - 显示帮助信息"
    echo ""
    echo "参数说明:"
    echo "  task_name      - 任务名称 (如: double_linkerhand_grasp)"
    echo "  use_timestamp  - 是否使用时间戳命名 (true/false, 可选)"
    echo ""
    echo "📋 工作流程示例:"
    echo ""
    echo "  # 1. 启动相机（根据任务配置）"
    echo "  $0 camera start double_linkerhand_grasp"
    echo ""
    echo "  # 2. 开始录制"
    echo "  $0 start double_linkerhand_grasp"
    echo ""
    echo "  # 3. 停止录制（相机继续运行）"
    echo "  $0 stop"
    echo ""
    echo "  # 4. 再次录制（无需重启相机）"
    echo "  $0 start double_linkerhand_grasp"
    echo ""
    echo "  # 5. 完成后停止相机"
    echo "  $0 camera stop"
    echo ""
    echo "📝 其他示例:"
    echo "  $0 camera start linkerhand_piper_grasp  # 单臂：仅启动D455"
    echo "  $0 camera start                         # 启动所有相机"
    echo "  $0 start linkerhand_piper_grasp true    # 使用时间戳命名"
    echo "  $0 status                               # 查询录制状态"
    echo "  $0 camera status                        # 查询相机状态"
}

# 主程序
case "$1" in
    "camera")
        # 相机控制子命令
        print_message $YELLOW "[弃用] camera 子命令已取消，请使用 ros2 launch linkerhand_data_collection_srv multi_camera_launch.py"
        ;;
    "start")
        start_recording "$2" "$3"
        ;;
    "stop")
        stop_recording
        ;;
    "status")
        check_status
        ;;
    "help"|"-h"|"--help")
        show_help
        ;;
    *)
        print_message $RED "错误: 未知命令 '$1'"
        echo ""
        show_help
        exit 1
        ;;
esac
