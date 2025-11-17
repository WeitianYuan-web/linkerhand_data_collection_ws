#!/bin/bash

# ===================================================================
# CAN状态诊断脚本
# 用途：检查CAN接口状态，推荐启动方式
# ===================================================================

# 颜色定义
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }
log_step() { echo -e "${BLUE}[→]${NC} $1"; }

SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
WORKSPACE_DIR=$(dirname "$SCRIPT_DIR")

log_info "=== CAN状态诊断 ==="
echo ""

# 1. 检查依赖
log_step "检查依赖工具..."
MISSING_TOOLS=()

if ! command -v ip &> /dev/null; then
    log_error "ip (iproute2)"
    MISSING_TOOLS+=("iproute2")
fi

if ! command -v g++ &> /dev/null; then
    log_warn "g++ (build-essential) - 检测程序需要"
    MISSING_TOOLS+=("build-essential")
fi

if [ ${#MISSING_TOOLS[@]} -gt 0 ]; then
    echo ""
    log_warn "缺少依赖: ${MISSING_TOOLS[*]}"
    log_info "安装命令: sudo apt install ${MISSING_TOOLS[*]}"
    echo ""
fi

# 2. 检查CAN内核模块
log_step "检查CAN内核模块..."
if lsmod | grep -q "^can"; then
    log_info "CAN模块已加载 ✓"
else
    log_error "CAN模块未加载"
    log_info "尝试加载: sudo modprobe can"
fi

# 3. 检测CAN接口
log_step "检测CAN接口..."
CAN_LIST=($(ip -br link show type can 2>/dev/null | awk '{print $1}' | grep -E '^can[0-9]+$' | sort))
CAN_COUNT=${#CAN_LIST[@]}

if [ $CAN_COUNT -eq 0 ]; then
    log_error "未检测到任何CAN接口"
    echo ""
    log_info "可能原因："
    log_info "  1. CAN设备未连接"
    log_info "  2. CAN驱动未加载"
    log_info "  3. USB-CAN适配器问题"
    echo ""
    log_info "排查步骤："
    log_info "  lsusb | grep -i can     # 检查USB设备"
    log_info "  dmesg | grep -i can     # 查看内核日志"
    echo ""
    exit 1
fi

log_info "检测到 $CAN_COUNT 个CAN接口: ${CAN_LIST[*]}"
echo ""

# 4. 显示CAN接口状态
log_step "CAN接口详细状态..."
for can in "${CAN_LIST[@]}"; do
    STATUS=$(ip link show "$can" 2>/dev/null)
    if echo "$STATUS" | grep -q "UP"; then
        log_info "$can: UP ✓"
    else
        log_warn "$can: DOWN"
    fi
    echo "$STATUS" | grep -E "state|bitrate" | sed 's/^/    /'
done
echo ""

# 5. 检测模式并给出建议
log_step "推荐启动方式..."
echo ""

if [ $CAN_COUNT -eq 4 ]; then
    log_info "检测到 4个CAN 接口 → 双手双臂模式"
    echo ""
    log_info "启动命令："
    echo "  cd $WORKSPACE_DIR"
    echo "  bash scripts/quick_start_hand.sh <密码>"
    echo ""
    log_info "自动配置："
    log_info "  - 检测并识别左右灵巧手"
    log_info "  - 自动分配机械臂CAN"
    log_info "  - 启动 double_linkerhand_grasp 模式"
    
elif [ $CAN_COUNT -eq 2 ]; then
    log_info "检测到 2个CAN 接口 → Piper单手模式"
    echo ""
    log_info "启动命令："
    echo "  cd $WORKSPACE_DIR"
    echo "  bash scripts/quick_start_hand.sh <密码>"
    echo ""
    log_info "自动配置："
    log_info "  - 检测并识别灵巧手（左/右）"
    log_info "  - 自动识别Piper机械臂"
    log_info "  - 启动 linkerhand_piper_grasp 模式"
    
else
    log_warn "检测到 $CAN_COUNT 个CAN接口"
    log_warn "支持的模式："
    log_warn "  - 4个CAN → 双手双臂模式"
    log_warn "  - 2个CAN → Piper单手模式"
    echo ""
    log_info "当前状态不匹配预期，请检查："
    log_info "  1. 所有设备是否已连接"
    log_info "  2. USB连接是否稳定"
    log_info "  3. 设备是否已通电"
fi

echo ""
log_info "=== 诊断完成 ==="
