/**
 * @file can_sender.cpp
 * @brief CAN发送接口实现
 * 
 * 使用socketcan实现CAN数据发送
 */

#include "linkerhand_cl/can_sender.hpp"
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <cerrno>
#include <cstdlib>
#include <sstream>
#include <cstdio>

namespace linkerhand_cl
{

CanSender::CanSender(const std::string& can_interface, uint32_t bitrate)
    : socket_fd_(-1)
    , can_interface_(can_interface)
    , bitrate_(bitrate)
    , is_open_(false)
{
    // 先设置比特率，再打开接口
    // 如果设置比特率失败，仍然尝试打开接口（可能接口已经配置好）
    int bitrate_ret = setCanBitrate(can_interface, bitrate);
    if (bitrate_ret != 0) {
        // 比特率设置失败，但继续尝试打开接口
        // 实际使用中，接口可能已经由系统配置好
    }
    openCanInterface(can_interface);
}

CanSender::~CanSender()
{
    closeCanInterface();
}

bool CanSender::isOpen() const
{
    return is_open_;
}

int CanSender::setCanBitrate(const std::string& can_interface, uint32_t bitrate)
{
    /**
     * @brief 设置CAN接口比特率
     * 
     * 使用系统命令设置CAN接口的比特率
     * 命令格式：ip link set <interface> type can bitrate <bitrate>
     */
    
    // 先关闭接口（如果已打开）
    std::ostringstream cmd_down;
    cmd_down << "ip link set " << can_interface << " down 2>/dev/null";
    std::system(cmd_down.str().c_str());
    
    // 设置比特率
    std::ostringstream cmd_set;
    cmd_set << "ip link set " << can_interface << " type can bitrate " << bitrate;
    int ret = std::system(cmd_set.str().c_str());
    if (ret != 0) {
        // 如果设置失败，尝试使用sudo（需要配置sudo免密）
        std::ostringstream cmd_sudo;
        cmd_sudo << "sudo ip link set " << can_interface << " type can bitrate " << bitrate;
        ret = std::system(cmd_sudo.str().c_str());
        if (ret != 0) {
            return -1;
        }
    }
    
    // 启动接口
    std::ostringstream cmd_up;
    cmd_up << "ip link set " << can_interface << " up";
    ret = std::system(cmd_up.str().c_str());
    if (ret != 0) {
        std::ostringstream cmd_sudo_up;
        cmd_sudo_up << "sudo ip link set " << can_interface << " up";
        ret = std::system(cmd_sudo_up.str().c_str());
        if (ret != 0) {
            return -1;
        }
    }
    
    return 0;
}

int CanSender::openCanInterface(const std::string& can_interface)
{
    /**
     * @brief 打开CAN接口
     * 
     * 使用socketcan创建RAW socket并绑定到指定的CAN接口
     */
    
    // 创建socket
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
        return -1;
    }
    
    // 获取接口索引
    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, can_interface.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
        ::close(socket_fd_);
        socket_fd_ = -1;
        return -1;
    }
    
    // 绑定socket到CAN接口
    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    
    if (bind(socket_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        ::close(socket_fd_);
        socket_fd_ = -1;
        return -1;
    }
    
    is_open_ = true;
    return 0;
}

void CanSender::closeCanInterface()
{
    /**
     * @brief 关闭CAN接口
     */
    if (socket_fd_ >= 0) {
        ::close(socket_fd_);
        socket_fd_ = -1;
        is_open_ = false;
    }
}

int CanSender::sendFrame(uint32_t can_id, HandFrameType frame_type, const uint8_t* data, uint8_t length)
{
    /**
     * @brief 发送单个CAN帧
     * 
     * 使用socketcan发送CAN数据帧
     * 参考CAN_SendFrame实现：
     * - frame_header放在data[0]
     * - data参数内容从data[1]开始，最多复制dlc-1个字节（最多7个字节）
     * - length参数是总DLC（包括frame_header）
     */
    
    if (!is_open_ || socket_fd_ < 0) {
        return -1;
    }
    
    // 限制DLC最大为8
    uint8_t dlc = length;
    if (dlc > 8) {
        dlc = 8;
    }
    
    // 构建CAN帧
    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    
    // 标准帧，11位标识符
    frame.can_id = can_id & 0x7FF;
    frame.can_dlc = dlc;
    
    // frame_header放在data[0]
    uint8_t frame_header = static_cast<uint8_t>(frame_type);
    frame.data[0] = frame_header;
    
    // data参数内容从data[1]开始，最多复制dlc-1个字节（最多7个字节）
    for (uint8_t i = 0; i < (dlc - 1) && i < 7; i++) {
        frame.data[i + 1] = data[i];
    }
    
    // 发送CAN帧
    ssize_t nbytes = write(socket_fd_, &frame, sizeof(struct can_frame));
    if (nbytes < 0) {
        return -1;
    }
    
    return 0;
}

uint8_t CanSender::calculateChecksum(const uint8_t* data, uint8_t length) const
{
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

int CanSender::sendL10(uint32_t device_id, const HandControlRequest& request)
{
    /**
     * @brief L10协议：单帧7字节协议
     * 
     * 帧格式：
     * 第一帧 (0x01):
     *   [0]=拇指Pitch
     *   [1]=拇指Yaw
     *   [2]=食指Pitch
     *   [3]=中指Pitch
     *   [4]=无名指Pitch
     *   [5]=小指Pitch
     * 
     * 第二帧 (0x04):
     *   [0]=食指Yaw
     *   [1]=无名指Yaw
     *   [2]=小指Yaw
     *   [3]=拇指Roll
     */
    
    constexpr uint8_t L10_FRAME_PITCH = 0x01;
    constexpr uint8_t L10_FRAME_YAW = 0x04;
    
    uint8_t txData[7];
    
    // 第一帧：Pitch帧 (0x01)
    txData[0] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_THUMB)].pitch_angle;
    txData[1] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_THUMB)].yaw_angle;
    txData[2] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_INDEX)].pitch_angle;
    txData[3] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_MIDDLE)].pitch_angle;
    txData[4] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_RING)].pitch_angle;
    txData[5] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_LITTLE)].pitch_angle;
    
    if (sendFrame(device_id, static_cast<HandFrameType>(L10_FRAME_PITCH), txData, 7) != 0) {
        return -1;
    }
    
    // 第一帧和第二帧之间延时1500us
    usleep(1500);
    
    // 第二帧：Yaw帧 (0x04)
    txData[0] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_INDEX)].yaw_angle;
    txData[1] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_RING)].yaw_angle;
    txData[2] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_LITTLE)].yaw_angle;
    txData[3] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_THUMB)].roll_angle;
    
    if (sendFrame(device_id, static_cast<HandFrameType>(L10_FRAME_YAW), txData, 5) != 0) {
        return -1;
    }
    
    return 0;
}

int CanSender::sendL20(uint32_t device_id, const HandControlRequest& request)
{
    /**
     * @brief L20协议：6帧协议
     * 
     * 协议格式：
     * - 0x11: Pitch帧（5个手指）
     * - 0x14: Tip帧（5个手指）
     * - 0x12: Yaw帧（5个手指）
     * - 0x13: Roll帧（5个手指）
     * - 0x15: Speed帧（5个手指）
     * - 0x16: Current帧（5个手指）
     */
    
    constexpr uint8_t L20_FRAME_PITCH = 0x11;
    constexpr uint8_t L20_FRAME_YAW = 0x12;
    constexpr uint8_t L20_FRAME_ROLL = 0x13;
    constexpr uint8_t L20_FRAME_TIP = 0x14;
    constexpr uint8_t L20_FRAME_SPEED = 0x15;
    constexpr uint8_t L20_FRAME_CURRENT = 0x16;
    
    uint8_t data[5];
    
    // 发送Pitch帧 (0x11)
    data[0] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_THUMB)].pitch_angle;
    data[1] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_INDEX)].pitch_angle;
    data[2] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_MIDDLE)].pitch_angle;
    data[3] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_RING)].pitch_angle;
    data[4] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_LITTLE)].pitch_angle;
    if (sendFrame(device_id, static_cast<HandFrameType>(L20_FRAME_PITCH), data, 6) != 0) {
        return -1;
    }
    
    // 帧间延时1500us
    usleep(1500);
    
    // 发送Tip帧 (0x14)
    data[0] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_THUMB)].tip_angle;
    data[1] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_INDEX)].tip_angle;
    data[2] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_MIDDLE)].tip_angle;
    data[3] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_RING)].tip_angle;
    data[4] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_LITTLE)].tip_angle;
    if (sendFrame(device_id, static_cast<HandFrameType>(L20_FRAME_TIP), data, 6) != 0) {
        return -1;
    }
    
    // 帧间延时1500us
    usleep(1500);
    
    // 发送Yaw帧 (0x12)
    data[0] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_THUMB)].yaw_angle;
    data[1] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_INDEX)].yaw_angle;
    data[2] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_MIDDLE)].yaw_angle;
    data[3] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_RING)].yaw_angle;
    data[4] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_LITTLE)].yaw_angle;
    if (sendFrame(device_id, static_cast<HandFrameType>(L20_FRAME_YAW), data, 6) != 0) {
        return -1;
    }
    
    // 帧间延时1500us
    usleep(1500);
    
    // 发送Roll帧 (0x13)
    data[0] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_THUMB)].roll_angle;
    data[1] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_INDEX)].roll_angle;
    data[2] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_MIDDLE)].roll_angle;
    data[3] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_RING)].roll_angle;
    data[4] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_LITTLE)].roll_angle;
    if (sendFrame(device_id, static_cast<HandFrameType>(L20_FRAME_ROLL), data, 6) != 0) {
        return -1;
    }
    
    // 帧间延时1500us
    usleep(1500);
    
    // 发送Speed帧 (0x15)
    data[0] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_THUMB)].speed_angle;
    data[1] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_INDEX)].speed_angle;
    data[2] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_MIDDLE)].speed_angle;
    data[3] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_RING)].speed_angle;
    data[4] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_LITTLE)].speed_angle;
    if (sendFrame(device_id, static_cast<HandFrameType>(L20_FRAME_SPEED), data, 6) != 0) {
        return -1;
    }
    
    // 帧间延时1500us
    usleep(1500);
    
    // 发送Current帧 (0x16)
    data[0] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_THUMB)].over_current;
    data[1] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_INDEX)].over_current;
    data[2] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_MIDDLE)].over_current;
    data[3] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_RING)].over_current;
    data[4] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_LITTLE)].over_current;
    if (sendFrame(device_id, static_cast<HandFrameType>(L20_FRAME_CURRENT), data, 6) != 0) {
        return -1;
    }
    
    return 0;
}

int CanSender::sendL21(uint32_t device_id, const HandControlRequest& request)
{
    /**
     * @brief L21协议：4帧协议（无速度电流）
     * 
     * 协议格式：
     * - 0x01: Roll帧（5个手指）
     * - 0x02: Yaw帧（5个手指）
     * - 0x03: Pitch帧（5个手指）
     * - 0x06: Tip帧（5个手指）
     */
    
    constexpr uint8_t L21_FRAME_ROLL = 0x01;
    constexpr uint8_t L21_FRAME_YAW = 0x02;
    constexpr uint8_t L21_FRAME_PITCH = 0x03;
    constexpr uint8_t L21_FRAME_TIP = 0x06;
    
    uint8_t data[5];
    
    // 发送Roll帧 (0x01)
    data[0] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_THUMB)].roll_angle;
    data[1] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_INDEX)].roll_angle;
    data[2] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_MIDDLE)].roll_angle;
    data[3] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_RING)].roll_angle;
    data[4] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_LITTLE)].roll_angle;
    if (sendFrame(device_id, static_cast<HandFrameType>(L21_FRAME_ROLL), data, 6) != 0) {
        return -1;
    }
    
    // 帧间延时1500us
    usleep(1500);
    
    // 发送Yaw帧 (0x02)
    data[0] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_THUMB)].yaw_angle;
    data[1] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_INDEX)].yaw_angle;
    data[2] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_MIDDLE)].yaw_angle;
    data[3] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_RING)].yaw_angle;
    data[4] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_LITTLE)].yaw_angle;
    if (sendFrame(device_id, static_cast<HandFrameType>(L21_FRAME_YAW), data, 6) != 0) {
        return -1;
    }
    
    // 帧间延时1500us
    usleep(1500);
    
    // 发送Pitch帧 (0x03)
    data[0] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_THUMB)].pitch_angle;
    data[1] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_INDEX)].pitch_angle;
    data[2] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_MIDDLE)].pitch_angle;
    data[3] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_RING)].pitch_angle;
    data[4] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_LITTLE)].pitch_angle;
    if (sendFrame(device_id, static_cast<HandFrameType>(L21_FRAME_PITCH), data, 6) != 0) {
        return -1;
    }
    
    // 帧间延时1500us
    usleep(1500);
    
    // 发送Tip帧 (0x06)
    data[0] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_THUMB)].tip_angle;
    data[1] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_INDEX)].tip_angle;
    data[2] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_MIDDLE)].tip_angle;
    data[3] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_RING)].tip_angle;
    data[4] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_LITTLE)].tip_angle;
    if (sendFrame(device_id, static_cast<HandFrameType>(L21_FRAME_TIP), data, 6) != 0) {
        return -1;
    }
    
    return 0;
}

int CanSender::sendL7(uint32_t device_id, const HandControlRequest& request)
{
    /**
     * @brief L7协议：单帧7字节协议
     * 
     * 帧格式：
     * 单帧 (0x01):
     *   [0]=拇指Pitch
     *   [1]=食指Pitch
     *   [2]=中指Pitch
     *   [3]=无名指Pitch
     *   [4]=小指Pitch
     *   [5]=拇指Yaw
     *   [6]=拇指Roll
     */
    
    constexpr uint8_t L7_FRAME_POSITION = 0x01;
    
    uint8_t txData[7];
    
    // 单帧：位置帧 (0x01)
    txData[0] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_THUMB)].pitch_angle;
    txData[1] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_INDEX)].pitch_angle;
    txData[2] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_MIDDLE)].pitch_angle;
    txData[3] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_RING)].pitch_angle;
    txData[4] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_LITTLE)].pitch_angle;
    txData[5] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_THUMB)].yaw_angle;
    txData[6] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_THUMB)].roll_angle;
    
    if (sendFrame(device_id, static_cast<HandFrameType>(L7_FRAME_POSITION), txData, 8) != 0) {
        return -1;
    }
    
    return 0;
}

int CanSender::sendO6(uint32_t device_id, const HandControlRequest& request)
{
    /**
     * @brief O6协议：单帧6字节协议
     * 
     * 帧格式：
     * 单帧 (0x01):
     *   [0]=拇指Pitch
     *   [1]=食指Pitch
     *   [2]=中指Pitch
     *   [3]=无名指Pitch
     *   [4]=小指Pitch
     *   [5]=拇指Yaw
     */
    
    constexpr uint8_t O6_FRAME_POSITION = 0x01;
    
    uint8_t txData[6];
    
    // 单帧：位置帧 (0x01)
    txData[0] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_THUMB)].pitch_angle;
    txData[1] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_INDEX)].pitch_angle;
    txData[2] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_MIDDLE)].pitch_angle;
    txData[3] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_RING)].pitch_angle;
    txData[4] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_LITTLE)].pitch_angle;
    txData[5] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_THUMB)].yaw_angle;
    
    if (sendFrame(device_id, static_cast<HandFrameType>(O6_FRAME_POSITION), txData, 7) != 0) {
        return -1;
    }
    
    return 0;
}

int CanSender::sendL25(uint32_t device_id, const HandControlRequest& request)
{
    /**
     * @brief L25协议：多帧协议（25自由度）
     * 
     * 协议格式：
     * - 0x03: Root1帧（5个手指根部关节）
     * - 0x02: Yaw帧（5个手指横摆关节）
     * - 0x01: Roll帧（5个手指横滚关节）
     * - 0x04: Root2帧（5个手指第二根部关节）
     * - 0x06: Tip帧（5个手指指尖关节）
     */
    
    constexpr uint8_t L25_FRAME_ROOT1 = 0x03;
    constexpr uint8_t L25_FRAME_YAW = 0x02;
    constexpr uint8_t L25_FRAME_ROLL = 0x01;
    constexpr uint8_t L25_FRAME_ROOT2 = 0x04;
    constexpr uint8_t L25_FRAME_TIP = 0x06;
    
    uint8_t data[5];
    
    // 发送Root1帧 (0x03) - 根部关节
    data[0] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_THUMB)].pitch_angle;
    data[1] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_INDEX)].pitch_angle;
    data[2] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_MIDDLE)].pitch_angle;
    data[3] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_RING)].pitch_angle;
    data[4] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_LITTLE)].pitch_angle;
    if (sendFrame(device_id, static_cast<HandFrameType>(L25_FRAME_ROOT1), data, 6) != 0) {
        return -1;
    }
    
    // 帧间延时1500us
    usleep(1500);
    
    // 发送Yaw帧 (0x02)
    data[0] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_THUMB)].yaw_angle;
    data[1] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_INDEX)].yaw_angle;
    data[2] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_MIDDLE)].yaw_angle;
    data[3] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_RING)].yaw_angle;
    data[4] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_LITTLE)].yaw_angle;
    if (sendFrame(device_id, static_cast<HandFrameType>(L25_FRAME_YAW), data, 6) != 0) {
        return -1;
    }
    
    // 帧间延时1500us
    usleep(1500);
    
    // 发送Roll帧 (0x01)
    data[0] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_THUMB)].roll_angle;
    data[1] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_INDEX)].roll_angle;
    data[2] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_MIDDLE)].roll_angle;
    data[3] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_RING)].roll_angle;
    data[4] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_LITTLE)].roll_angle;
    if (sendFrame(device_id, static_cast<HandFrameType>(L25_FRAME_ROLL), data, 6) != 0) {
        return -1;
    }
    
    // 帧间延时1500us
    usleep(1500);
    
    // 发送Root2帧 (0x04) - 第二根部关节（使用pitch作为第二根部）
    // 注意：L25的第二根部关节可能需要特殊处理，这里使用pitch值
    data[0] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_THUMB)].pitch_angle;
    data[1] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_INDEX)].pitch_angle;
    data[2] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_MIDDLE)].pitch_angle;
    data[3] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_RING)].pitch_angle;
    data[4] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_LITTLE)].pitch_angle;
    if (sendFrame(device_id, static_cast<HandFrameType>(L25_FRAME_ROOT2), data, 6) != 0) {
        return -1;
    }
    
    // 帧间延时1500us
    usleep(1500);
    
    // 发送Tip帧 (0x06)
    data[0] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_THUMB)].tip_angle;
    data[1] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_INDEX)].tip_angle;
    data[2] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_MIDDLE)].tip_angle;
    data[3] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_RING)].tip_angle;
    data[4] = request.fingers[static_cast<size_t>(FingerIndex::FINGER_LITTLE)].tip_angle;
    if (sendFrame(device_id, static_cast<HandFrameType>(L25_FRAME_TIP), data, 6) != 0) {
        return -1;
    }
    
    return 0;
}

int CanSender::send(HandProtocol protocol, uint32_t device_id, const HandControlRequest& request)
{
    switch (protocol) {
        case HandProtocol::HAND_PROTO_L10:
            return sendL10(device_id, request);
        case HandProtocol::HAND_PROTO_L21:
            return sendL21(device_id, request);
        case HandProtocol::HAND_PROTO_L7:
            return sendL7(device_id, request);
        case HandProtocol::HAND_PROTO_L25:
            return sendL25(device_id, request);
        case HandProtocol::HAND_PROTO_O6:
            return sendO6(device_id, request);
        case HandProtocol::HAND_PROTO_L20:
        default:
            return sendL20(device_id, request);
    }
}

}  // namespace linkerhand_cl

