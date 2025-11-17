#!/bin/bash

# ===================================================================
# LinkerHand SDK 统一启动脚本
# 支持：双手双臂模式(4个CAN) 和 Piper单手模式(2个CAN)
# 自动检测模式并配置
# ===================================================================

set -e

# 颜色定义
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; exit 1; }
log_step() { echo -e "${BLUE}[→]${NC} $1"; }

# 检查参数
if [ $# -lt 1 ]; then
    cat << EOF
用法: $0 <sudo密码> [--force]

说明:
  自动检测CAN接口数量，支持两种模式：
  - 4个CAN口 → 双手双臂模式 (double_linkerhand_grasp)
  - 2个CAN口 → Piper单手模式 (linkerhand_piper_grasp)

参数:
  <sudo密码>  : sudo权限密码（必须）
  --force     : 强制重新检测设备（可选）

示例:
  $0 mypassword              # 使用缓存配置（如有）
  $0 mypassword --force      # 强制重新检测

工作流程:
  1. 检查现有配置（如有且有效，跳过检测）
  2. 检测CAN接口并识别设备类型
  3. 生成配置文件 hand_can_config.yaml
  4. 设置udev规则
  5. 配置CAN接口
  6. 启动LinkerHand SDK

EOF
    exit 1
fi

PASSWORD="$1"
FORCE_DETECT=false

# 检查是否有 --force 参数
for arg in "$@"; do
    if [ "$arg" == "--force" ]; then
        FORCE_DETECT=true
    fi
done
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
WORKSPACE_DIR=$(dirname "$SCRIPT_DIR")
CONFIG_FILE="$SCRIPT_DIR/hand_can_config.yaml"
DETECTOR_BIN="$SCRIPT_DIR/detect_hand_config"
DETECTOR_SRC="$SCRIPT_DIR/detect_hand_config.cpp"

log_info "=== LinkerHand SDK 统一启动脚本 ==="
echo ""

cd "$WORKSPACE_DIR"

# 1. 检查是否已有有效配置
SKIP_DETECTION=false
if [ "$FORCE_DETECT" = true ]; then
    log_info "强制重新检测模式"
    rm -f "$CONFIG_FILE"
elif [ -f "$CONFIG_FILE" ]; then
    log_step "检查现有配置..."
    
    # 读取现有配置
    OLD_MODE=$(grep "mode:" "$CONFIG_FILE" | sed 's/.*"\(.*\)".*/\1/' 2>/dev/null || echo "")
    OLD_LEFT_HAND=$(grep "left_hand:" "$CONFIG_FILE" | sed 's/.*"\(can[0-9]*\)".*/\1/' 2>/dev/null || echo "")
    OLD_RIGHT_HAND=$(grep "right_hand:" "$CONFIG_FILE" | sed 's/.*"\(can[0-9]*\)".*/\1/' 2>/dev/null || echo "")
    
    # 检查配置的CAN接口是否仍然存在
    CONFIG_VALID=true
    
    if [ -n "$OLD_LEFT_HAND" ]; then
        if ! ip link show "$OLD_LEFT_HAND" &>/dev/null; then
            CONFIG_VALID=false
        fi
    fi
    
    if [ -n "$OLD_RIGHT_HAND" ]; then
        if ! ip link show "$OLD_RIGHT_HAND" &>/dev/null; then
            CONFIG_VALID=false
        fi
    fi
    
    if [ "$CONFIG_VALID" = true ] && [ -n "$OLD_MODE" ]; then
        log_info "发现有效配置，跳过设备检测"
        log_info "  模式: $OLD_MODE"
        [ -n "$OLD_LEFT_HAND" ]  && log_info "  左手: $OLD_LEFT_HAND"
        [ -n "$OLD_RIGHT_HAND" ] && log_info "  右手: $OLD_RIGHT_HAND"
        SKIP_DETECTION=true
        echo ""
    else
        log_warn "现有配置无效（CAN接口已变化），将重新检测"
        rm -f "$CONFIG_FILE"
    fi
fi

# 2. 如果需要，运行检测程序
if [ "$SKIP_DETECTION" = false ]; then
    # 编译检测程序
    if [ ! -f "$DETECTOR_BIN" ]; then
        log_step "首次运行，编译检测程序..."
        if g++ "$DETECTOR_SRC" -std=c++17 -o "$DETECTOR_BIN" 2>/dev/null; then
            log_info "检测程序编译成功"
        else
            log_error "检测程序编译失败，请安装 g++ 编译器"
        fi
    fi
    
    # 运行检测程序生成配置（需要sudo权限启动CAN）
    log_step "检测CAN接口并识别设备..."
    if ! echo "$PASSWORD" | sudo -S "$DETECTOR_BIN"; then
        log_error "设备检测失败"
    fi
    echo ""
fi

# 3. 读取配置文件
if [ ! -f "$CONFIG_FILE" ]; then
    log_error "配置文件未生成: $CONFIG_FILE"
fi

log_step "读取配置文件..."
MODE=$(grep "mode:" "$CONFIG_FILE" | sed 's/.*"\(.*\)".*/\1/')
LEFT_HAND=$(grep "left_hand:" "$CONFIG_FILE" | sed 's/.*"\(can[0-9]*\)".*/\1/' || echo "")
RIGHT_HAND=$(grep "right_hand:" "$CONFIG_FILE" | sed 's/.*"\(can[0-9]*\)".*/\1/' || echo "")
LEFT_ARM=$(grep "left_arm:" "$CONFIG_FILE" | sed 's/.*"\(can[0-9]*\)".*/\1/' || echo "")
RIGHT_ARM=$(grep "right_arm:" "$CONFIG_FILE" | sed 's/.*"\(can[0-9]*\)".*/\1/' || echo "")

log_info "检测模式: $MODE"
log_info "配置信息："
[ -n "$LEFT_HAND" ]  && log_info "  左手: $LEFT_HAND"
[ -n "$RIGHT_HAND" ] && log_info "  右手: $RIGHT_HAND"
[ -n "$LEFT_ARM" ]   && log_info "  左臂: $LEFT_ARM"
[ -n "$RIGHT_ARM" ]  && log_info "  右臂: $RIGHT_ARM"
echo ""

# 4. 启动CAN接口（使用物理名称）
log_step "启动CAN接口..."

if [ "$MODE" == "double_linkerhand_grasp" ]; then
    log_info "双手双臂模式"
    
    # 启动左手CAN
    if [ -n "$LEFT_HAND" ]; then
        log_info "  启动 $LEFT_HAND (左手)..."
        echo "$PASSWORD" | sudo -S ip link set "$LEFT_HAND" down 2>/dev/null || true
        echo "$PASSWORD" | sudo -S ip link set "$LEFT_HAND" up type can bitrate 1000000
    fi
    
    # 启动右手CAN
    if [ -n "$RIGHT_HAND" ]; then
        log_info "  启动 $RIGHT_HAND (右手)..."
        echo "$PASSWORD" | sudo -S ip link set "$RIGHT_HAND" down 2>/dev/null || true
        echo "$PASSWORD" | sudo -S ip link set "$RIGHT_HAND" up type can bitrate 1000000
    fi
    
elif [ "$MODE" == "linkerhand_piper_grasp" ]; then
    log_info "Piper单手模式"
    
    # 启动手臂CAN
    if [ -n "$LEFT_ARM" ]; then
        log_info "  启动 $LEFT_ARM (左臂)..."
        echo "$PASSWORD" | sudo -S ip link set "$LEFT_ARM" down 2>/dev/null || true
        echo "$PASSWORD" | sudo -S ip link set "$LEFT_ARM" up type can bitrate 1000000
    fi
    if [ -n "$RIGHT_ARM" ]; then
        log_info "  启动 $RIGHT_ARM (右臂)..."
        echo "$PASSWORD" | sudo -S ip link set "$RIGHT_ARM" down 2>/dev/null || true
        echo "$PASSWORD" | sudo -S ip link set "$RIGHT_ARM" up type can bitrate 1000000
    fi
    
    # 启动手CAN
    if [ -n "$LEFT_HAND" ]; then
        log_info "  启动 $LEFT_HAND (左手)..."
        echo "$PASSWORD" | sudo -S ip link set "$LEFT_HAND" down 2>/dev/null || true
        echo "$PASSWORD" | sudo -S ip link set "$LEFT_HAND" up type can bitrate 1000000
    fi
    if [ -n "$RIGHT_HAND" ]; then
        log_info "  启动 $RIGHT_HAND (右手)..."
        echo "$PASSWORD" | sudo -S ip link set "$RIGHT_HAND" down 2>/dev/null || true
        echo "$PASSWORD" | sudo -S ip link set "$RIGHT_HAND" up type can bitrate 1000000
    fi
fi

sleep 1
log_info "CAN接口启动完成"
echo ""

# 5. 确定CAN接口配置（用于启动参数）
log_step "准备启动参数..."

# 6. 启动LinkerHand控制节点（使用linkerhand_cl）
log_step "启动LinkerHand控制节点..."
sleep 1

if [ "$MODE" == "double_linkerhand_grasp" ]; then
    # 双手模式：启动双手控制节点
    # 注意：如果左右手使用不同的CAN接口，需要分别指定
    LEFT_CAN="${LEFT_HAND:-can3}"
    RIGHT_CAN="${RIGHT_HAND:-can1}"
    
    log_info "启动双手控制节点"
    log_info "  左手CAN: $LEFT_CAN"
    log_info "  右手CAN: $RIGHT_CAN"
    
    ros2 launch linkerhand_cl linker_hand_double.launch.py \
        left_can:="$LEFT_CAN" \
        right_can:="$RIGHT_CAN" \
        enable_can:=true \
        can_bitrate:=1000000
    
elif [ "$MODE" == "linkerhand_piper_grasp" ]; then
    # 单手模式：根据配置确定使用左手还是右手
    HAND_CAN=""
    HAND_TYPE=""
    HAND_JOINT="L10"  # 默认型号，可以从task_config.json读取
    
    if [ -n "$LEFT_HAND" ]; then
        HAND_CAN="$LEFT_HAND"
        HAND_TYPE="left"
    elif [ -n "$RIGHT_HAND" ]; then
        HAND_CAN="$RIGHT_HAND"
        HAND_TYPE="right"
    else
        log_error "无法确定手部CAN接口"
    fi
    
    log_info "启动单手控制节点"
    log_info "  手类型: $HAND_TYPE"
    log_info "  CAN接口: $HAND_CAN"
    
    ros2 launch linkerhand_cl linker_hand.launch.py \
        hand_type:="$HAND_TYPE" \
        hand_joint:="$HAND_JOINT" \
        can:="$HAND_CAN" \
        enable_can:=true \
        can_bitrate:=1000000
else
    log_error "未知模式，无法启动控制节点"
fi

log_info "=== 完成 ==="
