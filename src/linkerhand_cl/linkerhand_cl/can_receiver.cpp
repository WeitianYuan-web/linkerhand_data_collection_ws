/**
 * @file can_receiver.cpp
 * @brief CAN接收接口实现
 * 
 * 使用socketcan实现CAN数据接收，用于获取灵巧手反馈数据
 */

#include "linkerhand_cl/can_receiver.hpp"
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
#include <algorithm>

namespace linkerhand_cl
{

CanReceiver::CanReceiver(const std::string& can_interface, uint32_t bitrate, uint32_t device_id)
    : socket_fd_(-1)
    , can_interface_(can_interface)
    , bitrate_(bitrate)
    , device_id_(device_id)
    , is_open_(false)
    , running_(false)
{
    // 初始化矩阵映射（与Python SDK保持一致）
    matrix_map_[0] = 0;
    matrix_map_[16] = 1;
    matrix_map_[32] = 2;
    matrix_map_[48] = 3;
    matrix_map_[64] = 4;
    matrix_map_[80] = 5;
    matrix_map_[96] = 6;
    matrix_map_[112] = 7;
    matrix_map_[128] = 8;
    matrix_map_[144] = 9;
    matrix_map_[160] = 10;
    matrix_map_[176] = 11;
    
    // 初始化数据缓存
    normal_force_.resize(5, -1.0f);
    tangential_force_.resize(5, -1.0f);
    tangential_force_dir_.resize(5, -1.0f);
    approach_inc_.resize(5, -1.0f);
    
    // 初始化协议特定数据缓存
    l10_x01.resize(6, 0);
    l10_x04.resize(4, 0);
    l10_x05.resize(5, 0);
    l10_x06.resize(5, 0);
    
    l20_x01.resize(5, 0);
    l20_x02.resize(5, 0);
    l20_x03.resize(5, 0);
    l20_x04.resize(5, 0);
    l20_x05.resize(5, 0);
    l20_x06.resize(5, 0);
    l20_x07.resize(5, 0);
    
    l21_x01.resize(5, 0);
    l21_x02.resize(5, 0);
    l21_x03.resize(5, 0);
    l21_x06.resize(5, 0);
    
    l7_x01.resize(7, 0);
    l7_x05.resize(7, 0);
    
    l25_x01.resize(5, 0);
    l25_x02.resize(5, 0);
    l25_x03.resize(5, 0);
    l25_x04.resize(5, 0);
    l25_x06.resize(5, 0);
    
    o6_x01.resize(6, 0);
    o6_x05.resize(6, 0);
    
    // 先设置比特率，再打开接口
    int bitrate_ret = setCanBitrate(can_interface, bitrate);
    if (bitrate_ret != 0) {
        // 比特率设置失败，但继续尝试打开接口
    }
    openCanInterface(can_interface);
}

CanReceiver::~CanReceiver()
{
    stop();
    closeCanInterface();
}

bool CanReceiver::isOpen() const
{
    return is_open_;
}

int CanReceiver::setCanBitrate(const std::string& can_interface, uint32_t bitrate)
{
    /**
     * @brief 设置CAN接口比特率
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
        // 如果设置失败，尝试使用sudo
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

int CanReceiver::openCanInterface(const std::string& can_interface)
{
    /**
     * @brief 打开CAN接口
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

void CanReceiver::closeCanInterface()
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

int CanReceiver::start()
{
    /**
     * @brief 启动接收线程
     */
    if (!is_open_ || socket_fd_ < 0) {
        return -1;
    }
    
    if (running_) {
        return 0;  // 已经在运行
    }
    
    running_ = true;
    receive_thread_ = std::thread(&CanReceiver::receiveThread, this);
    
    return 0;
}

void CanReceiver::stop()
{
    /**
     * @brief 停止接收线程
     */
    if (running_) {
        running_ = false;
        if (receive_thread_.joinable()) {
            receive_thread_.join();
        }
    }
}

void CanReceiver::receiveThread()
{
    /**
     * @brief 接收线程函数
     */
    struct can_frame frame;
    
    while (running_) {
        ssize_t nbytes = read(socket_fd_, &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                // 错误发生，短暂休眠后继续
                usleep(10000);  // 10ms
            }
            continue;
        }
        
        if (nbytes == sizeof(struct can_frame)) {
            // 检查CAN ID是否匹配
            if ((frame.can_id & 0x7FF) == device_id_) {
                processResponse(frame);
            }
        }
    }
}

void CanReceiver::processResponse(const struct can_frame& frame)
{
    /**
     * @brief 处理接收到的CAN消息
     */
    if (frame.can_dlc == 0) {
        return;
    }
    
    uint8_t frame_type = frame.data[0];
    const uint8_t* data = frame.data + 1;
    uint8_t data_length = frame.can_dlc - 1;
    
    // 根据协议类型处理（需要知道当前使用的协议，这里先尝试所有协议）
    // 注意：实际使用时应该根据配置的协议类型来处理
    processL10Response(frame_type, data, data_length);
    processL20Response(frame_type, data, data_length);
    processL21Response(frame_type, data, data_length);
    processL7Response(frame_type, data, data_length);
    processL25Response(frame_type, data, data_length);
    processO6Response(frame_type, data, data_length);
}

void CanReceiver::processL10Response(uint8_t frame_type, const uint8_t* data, uint8_t length)
{
    /**
     * @brief 处理L10协议反馈数据
     */
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    if (frame_type == 0x01) {  // JOINT_POSITION_RCO
        l10_x01.assign(data, data + std::min(length, static_cast<uint8_t>(6)));
    } else if (frame_type == 0x04) {  // JOINT_POSITION2_RCO
        l10_x04.assign(data, data + std::min(length, static_cast<uint8_t>(4)));
    } else if (frame_type == 0x05) {  // JOINT_SPEED_R
        l10_x05.assign(data, data + std::min(length, static_cast<uint8_t>(5)));
    } else if (frame_type == 0x06) {  // JOINT_SPEED2
        l10_x06.assign(data, data + std::min(length, static_cast<uint8_t>(5)));
    } else if (frame_type == 0x20) {  // HAND_NORMAL_FORCE
        for (size_t i = 0; i < std::min(static_cast<size_t>(length), normal_force_.size()); i++) {
            normal_force_[i] = static_cast<float>(data[i]);
        }
    } else if (frame_type == 0x21) {  // HAND_TANGENTIAL_FORCE
        for (size_t i = 0; i < std::min(static_cast<size_t>(length), tangential_force_.size()); i++) {
            tangential_force_[i] = static_cast<float>(data[i]);
        }
    } else if (frame_type == 0x22) {  // HAND_TANGENTIAL_FORCE_DIR
        for (size_t i = 0; i < std::min(static_cast<size_t>(length), tangential_force_dir_.size()); i++) {
            tangential_force_dir_[i] = static_cast<float>(data[i]);
        }
    } else if (frame_type == 0x23) {  // HAND_APPROACH_INC
        for (size_t i = 0; i < std::min(static_cast<size_t>(length), approach_inc_.size()); i++) {
            approach_inc_[i] = static_cast<float>(data[i]);
        }
    } else if (frame_type == 0x33) {  // MOTOR_TEMPERATURE_1
        l10_x33.assign(data, data + std::min(length, static_cast<uint8_t>(5)));
    } else if (frame_type == 0x34) {  // MOTOR_TEMPERATURE_2
        l10_x34.assign(data, data + std::min(length, static_cast<uint8_t>(5)));
    } else if (frame_type == 0x64 || frame_type == 0xC2) {  // VERSION
        feedback_data_.version.assign(data, data + length);
    } else if (frame_type >= 0xb1 && frame_type <= 0xb5) {  // 矩阵触觉数据
        // 处理矩阵触觉数据
        if (length >= 7) {
            uint8_t row_index = data[0];
            auto map_it = matrix_map_.find(row_index);
            if (map_it != matrix_map_.end()) {
                uint8_t finger_idx = frame_type - 0xb1;  // 0=thumb, 1=index, 2=middle, 3=ring, 4=little
                if (finger_idx < 5 && map_it->second < 12) {
                    for (uint8_t col = 0; col < 6 && col < (length - 1); col++) {
                        feedback_data_.matrix_touch[finger_idx][map_it->second][col] = static_cast<float>(data[col + 1]);
                    }
                }
            }
        }
    }
}

void CanReceiver::processL20Response(uint8_t frame_type, const uint8_t* data, uint8_t length)
{
    /**
     * @brief 处理L20协议反馈数据
     */
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    if (frame_type == 0x01) {  // JOINT_PITCH_R
        l20_x01.assign(data, data + std::min(length, static_cast<uint8_t>(5)));
    } else if (frame_type == 0x02) {  // JOINT_YAW_R
        l20_x02.assign(data, data + std::min(length, static_cast<uint8_t>(5)));
    } else if (frame_type == 0x03) {  // JOINT_ROLL_R
        l20_x03.assign(data, data + std::min(length, static_cast<uint8_t>(5)));
    } else if (frame_type == 0x04) {  // JOINT_TIP_R
        l20_x04.assign(data, data + std::min(length, static_cast<uint8_t>(5)));
    } else if (frame_type == 0x05) {  // JOINT_SPEED_R
        l20_x05.assign(data, data + std::min(length, static_cast<uint8_t>(5)));
    } else if (frame_type == 0x06) {  // JOINT_CURRENT_R
        l20_x06.assign(data, data + std::min(length, static_cast<uint8_t>(5)));
    } else if (frame_type == 0x07) {  // JOINT_FAULT_R
        l20_x07.assign(data, data + std::min(length, static_cast<uint8_t>(5)));
    } else if (frame_type == 0x20) {  // HAND_NORMAL_FORCE
        for (size_t i = 0; i < std::min(static_cast<size_t>(length), normal_force_.size()); i++) {
            normal_force_[i] = static_cast<float>(data[i]);
        }
    } else if (frame_type == 0x21) {  // HAND_TANGENTIAL_FORCE
        for (size_t i = 0; i < std::min(static_cast<size_t>(length), tangential_force_.size()); i++) {
            tangential_force_[i] = static_cast<float>(data[i]);
        }
    } else if (frame_type == 0x22) {  // HAND_TANGENTIAL_FORCE_DIR
        for (size_t i = 0; i < std::min(static_cast<size_t>(length), tangential_force_dir_.size()); i++) {
            tangential_force_dir_[i] = static_cast<float>(data[i]);
        }
    } else if (frame_type == 0x23) {  // HAND_APPROACH_INC
        for (size_t i = 0; i < std::min(static_cast<size_t>(length), approach_inc_.size()); i++) {
            approach_inc_[i] = static_cast<float>(data[i]);
        }
    } else if (frame_type >= 0xb1 && frame_type <= 0xb5) {  // 矩阵触觉数据
        if (length >= 7) {
            uint8_t row_index = data[0];
            auto map_it = matrix_map_.find(row_index);
            if (map_it != matrix_map_.end()) {
                uint8_t finger_idx = frame_type - 0xb1;
                if (finger_idx < 5 && map_it->second < 12) {
                    for (uint8_t col = 0; col < 6 && col < (length - 1); col++) {
                        feedback_data_.matrix_touch[finger_idx][map_it->second][col] = static_cast<float>(data[col + 1]);
                    }
                }
            }
        }
    }
}

void CanReceiver::processL21Response(uint8_t frame_type, const uint8_t* data, uint8_t length)
{
    /**
     * @brief 处理L21协议反馈数据
     */
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    if (frame_type == 0x01) {  // ROLL_POS
        l21_x01.assign(data, data + std::min(length, static_cast<uint8_t>(5)));
    } else if (frame_type == 0x02) {  // YAW_POS
        l21_x02.assign(data, data + std::min(length, static_cast<uint8_t>(5)));
    } else if (frame_type == 0x03) {  // ROOT1_POS
        l21_x03.assign(data, data + std::min(length, static_cast<uint8_t>(5)));
    } else if (frame_type == 0x06) {  // TIP_POS
        l21_x06.assign(data, data + std::min(length, static_cast<uint8_t>(5)));
    } else if (frame_type == 0x20) {  // HAND_NORMAL_FORCE
        for (size_t i = 0; i < std::min(static_cast<size_t>(length), normal_force_.size()); i++) {
            normal_force_[i] = static_cast<float>(data[i]);
        }
    } else if (frame_type == 0x21) {  // HAND_TANGENTIAL_FORCE
        for (size_t i = 0; i < std::min(static_cast<size_t>(length), tangential_force_.size()); i++) {
            tangential_force_[i] = static_cast<float>(data[i]);
        }
    } else if (frame_type == 0x22) {  // HAND_TANGENTIAL_FORCE_DIR
        for (size_t i = 0; i < std::min(static_cast<size_t>(length), tangential_force_dir_.size()); i++) {
            tangential_force_dir_[i] = static_cast<float>(data[i]);
        }
    } else if (frame_type == 0x23) {  // HAND_APPROACH_INC
        for (size_t i = 0; i < std::min(static_cast<size_t>(length), approach_inc_.size()); i++) {
            approach_inc_[i] = static_cast<float>(data[i]);
        }
    } else if (frame_type >= 0xb1 && frame_type <= 0xb5) {  // 矩阵触觉数据
        if (length >= 7) {
            uint8_t row_index = data[0];
            auto map_it = matrix_map_.find(row_index);
            if (map_it != matrix_map_.end()) {
                uint8_t finger_idx = frame_type - 0xb1;
                if (finger_idx < 5 && map_it->second < 12) {
                    for (uint8_t col = 0; col < 6 && col < (length - 1); col++) {
                        feedback_data_.matrix_touch[finger_idx][map_it->second][col] = static_cast<float>(data[col + 1]);
                    }
                }
            }
        }
    }
}

void CanReceiver::processL7Response(uint8_t frame_type, const uint8_t* data, uint8_t length)
{
    /**
     * @brief 处理L7协议反馈数据
     */
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    if (frame_type == 0x01) {  // JOINT_POSITION
        l7_x01.assign(data, data + std::min(length, static_cast<uint8_t>(7)));
    } else if (frame_type == 0x05) {  // JOINT_SPEED
        l7_x05.assign(data, data + std::min(length, static_cast<uint8_t>(7)));
    } else if (frame_type == 0x20) {  // HAND_NORMAL_FORCE
        for (size_t i = 0; i < std::min(static_cast<size_t>(length), normal_force_.size()); i++) {
            normal_force_[i] = static_cast<float>(data[i]);
        }
    } else if (frame_type == 0x21) {  // HAND_TANGENTIAL_FORCE
        for (size_t i = 0; i < std::min(static_cast<size_t>(length), tangential_force_.size()); i++) {
            tangential_force_[i] = static_cast<float>(data[i]);
        }
    } else if (frame_type == 0x22) {  // HAND_TANGENTIAL_FORCE_DIR
        for (size_t i = 0; i < std::min(static_cast<size_t>(length), tangential_force_dir_.size()); i++) {
            tangential_force_dir_[i] = static_cast<float>(data[i]);
        }
    } else if (frame_type == 0x23) {  // HAND_APPROACH_INC
        for (size_t i = 0; i < std::min(static_cast<size_t>(length), approach_inc_.size()); i++) {
            approach_inc_[i] = static_cast<float>(data[i]);
        }
    }
}

void CanReceiver::processL25Response(uint8_t frame_type, const uint8_t* data, uint8_t length)
{
    /**
     * @brief 处理L25协议反馈数据
     */
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    if (frame_type == 0x01) {  // ROLL_POS
        l25_x01.assign(data, data + std::min(length, static_cast<uint8_t>(5)));
    } else if (frame_type == 0x02) {  // YAW_POS
        l25_x02.assign(data, data + std::min(length, static_cast<uint8_t>(5)));
    } else if (frame_type == 0x03) {  // ROOT1_POS
        l25_x03.assign(data, data + std::min(length, static_cast<uint8_t>(5)));
    } else if (frame_type == 0x04) {  // ROOT2_POS
        l25_x04.assign(data, data + std::min(length, static_cast<uint8_t>(5)));
    } else if (frame_type == 0x06) {  // TIP_POS
        l25_x06.assign(data, data + std::min(length, static_cast<uint8_t>(5)));
    } else if (frame_type == 0x20) {  // HAND_NORMAL_FORCE
        for (size_t i = 0; i < std::min(static_cast<size_t>(length), normal_force_.size()); i++) {
            normal_force_[i] = static_cast<float>(data[i]);
        }
    } else if (frame_type == 0x21) {  // HAND_TANGENTIAL_FORCE
        for (size_t i = 0; i < std::min(static_cast<size_t>(length), tangential_force_.size()); i++) {
            tangential_force_[i] = static_cast<float>(data[i]);
        }
    } else if (frame_type == 0x22) {  // HAND_TANGENTIAL_FORCE_DIR
        for (size_t i = 0; i < std::min(static_cast<size_t>(length), tangential_force_dir_.size()); i++) {
            tangential_force_dir_[i] = static_cast<float>(data[i]);
        }
    } else if (frame_type == 0x23) {  // HAND_APPROACH_INC
        for (size_t i = 0; i < std::min(static_cast<size_t>(length), approach_inc_.size()); i++) {
            approach_inc_[i] = static_cast<float>(data[i]);
        }
    } else if (frame_type >= 0xb1 && frame_type <= 0xb5) {  // 矩阵触觉数据
        if (length >= 7) {
            uint8_t row_index = data[0];
            auto map_it = matrix_map_.find(row_index);
            if (map_it != matrix_map_.end()) {
                uint8_t finger_idx = frame_type - 0xb1;
                if (finger_idx < 5 && map_it->second < 12) {
                    for (uint8_t col = 0; col < 6 && col < (length - 1); col++) {
                        feedback_data_.matrix_touch[finger_idx][map_it->second][col] = static_cast<float>(data[col + 1]);
                    }
                }
            }
        }
    }
}

void CanReceiver::processO6Response(uint8_t frame_type, const uint8_t* data, uint8_t length)
{
    /**
     * @brief 处理O6协议反馈数据
     */
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    if (frame_type == 0x01) {  // JOINT_POSITION
        o6_x01.assign(data, data + std::min(length, static_cast<uint8_t>(6)));
    } else if (frame_type == 0x05) {  // JOINT_SPEED
        o6_x05.assign(data, data + std::min(length, static_cast<uint8_t>(6)));
    } else if (frame_type == 0x20) {  // HAND_NORMAL_FORCE
        for (size_t i = 0; i < std::min(static_cast<size_t>(length), normal_force_.size()); i++) {
            normal_force_[i] = static_cast<float>(data[i]);
        }
    } else if (frame_type == 0x21) {  // HAND_TANGENTIAL_FORCE
        for (size_t i = 0; i < std::min(static_cast<size_t>(length), tangential_force_.size()); i++) {
            tangential_force_[i] = static_cast<float>(data[i]);
        }
    } else if (frame_type == 0x22) {  // HAND_TANGENTIAL_FORCE_DIR
        for (size_t i = 0; i < std::min(static_cast<size_t>(length), tangential_force_dir_.size()); i++) {
            tangential_force_dir_[i] = static_cast<float>(data[i]);
        }
    } else if (frame_type == 0x23) {  // HAND_APPROACH_INC
        for (size_t i = 0; i < std::min(static_cast<size_t>(length), approach_inc_.size()); i++) {
            approach_inc_[i] = static_cast<float>(data[i]);
        }
    }
}

int CanReceiver::sendRequestFrame(uint8_t frame_type, const uint8_t* data, uint8_t length)
{
    /**
     * @brief 发送请求帧
     */
    if (!is_open_ || socket_fd_ < 0) {
        return -1;
    }
    
    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    
    frame.can_id = device_id_ & 0x7FF;
    frame.can_dlc = length + 1;  // +1 for frame_type
    
    frame.data[0] = frame_type;
    for (uint8_t i = 0; i < length && i < 7; i++) {
        frame.data[i + 1] = data[i];
    }
    
    ssize_t nbytes = write(socket_fd_, &frame, sizeof(struct can_frame));
    if (nbytes < 0) {
        return -1;
    }
    
    return 0;
}

int CanReceiver::requestStatus(HandProtocol protocol)
{
    /**
     * @brief 请求状态数据
     */
    switch (protocol) {
        case HandProtocol::HAND_PROTO_L10: {
            // L10: 请求位置数据
            sendRequestFrame(0x01, nullptr, 0);  // JOINT_POSITION_RCO
            usleep(3000);
            sendRequestFrame(0x04, nullptr, 0);  // JOINT_POSITION2_RCO
            break;
        }
        case HandProtocol::HAND_PROTO_L20: {
            // L20: 请求位置数据
            sendRequestFrame(0x01, nullptr, 0);  // JOINT_PITCH_R
            usleep(10000);
            sendRequestFrame(0x02, nullptr, 0);  // JOINT_YAW_R
            usleep(10000);
            sendRequestFrame(0x03, nullptr, 0);  // JOINT_ROLL_R
            usleep(10000);
            sendRequestFrame(0x04, nullptr, 0);  // JOINT_TIP_R
            break;
        }
        case HandProtocol::HAND_PROTO_L21: {
            // L21: 请求位置数据
            sendRequestFrame(0x01, nullptr, 0);  // ROLL_POS
            usleep(10000);
            sendRequestFrame(0x02, nullptr, 0);  // YAW_POS
            usleep(10000);
            sendRequestFrame(0x03, nullptr, 0);  // ROOT1_POS
            usleep(10000);
            sendRequestFrame(0x06, nullptr, 0);  // TIP_POS
            break;
        }
        case HandProtocol::HAND_PROTO_L7: {
            // L7: 请求位置数据
            sendRequestFrame(0x01, nullptr, 0);
            break;
        }
        case HandProtocol::HAND_PROTO_L25: {
            // L25: 请求位置数据
            sendRequestFrame(0x01, nullptr, 0);  // ROLL_POS
            usleep(10000);
            sendRequestFrame(0x02, nullptr, 0);  // YAW_POS
            usleep(10000);
            sendRequestFrame(0x03, nullptr, 0);  // ROOT1_POS
            usleep(10000);
            sendRequestFrame(0x04, nullptr, 0);  // ROOT2_POS
            usleep(10000);
            sendRequestFrame(0x06, nullptr, 0);  // TIP_POS
            break;
        }
        case HandProtocol::HAND_PROTO_O6: {
            // O6: 请求位置数据
            sendRequestFrame(0x01, nullptr, 0);
            break;
        }
        default:
            return -1;
    }
    
    return 0;
}

std::vector<uint8_t> CanReceiver::getJointPositions(HandProtocol protocol)
{
    /**
     * @brief 获取当前关节位置
     */
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    std::vector<uint8_t> positions;
    
    switch (protocol) {
        case HandProtocol::HAND_PROTO_L10:
            positions = l10_x01;
            positions.insert(positions.end(), l10_x04.begin(), l10_x04.end());
            break;
        case HandProtocol::HAND_PROTO_L20:
            positions = l20_x01;
            positions.insert(positions.end(), l20_x02.begin(), l20_x02.end());
            positions.insert(positions.end(), l20_x03.begin(), l20_x03.end());
            positions.insert(positions.end(), l20_x04.begin(), l20_x04.end());
            break;
        case HandProtocol::HAND_PROTO_L21:
            positions = l21_x01;
            positions.insert(positions.end(), l21_x02.begin(), l21_x02.end());
            positions.insert(positions.end(), l21_x03.begin(), l21_x03.end());
            positions.insert(positions.end(), l21_x06.begin(), l21_x06.end());
            break;
        case HandProtocol::HAND_PROTO_L7:
            positions = l7_x01;
            break;
        case HandProtocol::HAND_PROTO_L25:
            positions = l25_x01;
            positions.insert(positions.end(), l25_x02.begin(), l25_x02.end());
            positions.insert(positions.end(), l25_x03.begin(), l25_x03.end());
            positions.insert(positions.end(), l25_x04.begin(), l25_x04.end());
            positions.insert(positions.end(), l25_x06.begin(), l25_x06.end());
            break;
        case HandProtocol::HAND_PROTO_O6:
            positions = o6_x01;
            break;
        default:
            break;
    }
    
    return positions;
}

std::vector<uint8_t> CanReceiver::getJointVelocities(HandProtocol protocol)
{
    /**
     * @brief 获取当前关节速度
     */
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    switch (protocol) {
        case HandProtocol::HAND_PROTO_L10: {
            std::vector<uint8_t> vel = l10_x05;
            vel.insert(vel.end(), l10_x06.begin(), l10_x06.end());
            return vel;
        }
        case HandProtocol::HAND_PROTO_L20:
            return l20_x05;
        case HandProtocol::HAND_PROTO_L7:
            return l7_x05;
        case HandProtocol::HAND_PROTO_O6:
            return o6_x05;
        default:
            return std::vector<uint8_t>();
    }
}

std::vector<uint8_t> CanReceiver::getJointCurrents(HandProtocol protocol)
{
    /**
     * @brief 获取当前关节电流
     */
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    switch (protocol) {
        case HandProtocol::HAND_PROTO_L20:
            return l20_x06;
        default:
            return std::vector<uint8_t>();
    }
}

std::vector<uint8_t> CanReceiver::getJointTemperatures(HandProtocol protocol)
{
    /**
     * @brief 获取当前关节温度
     */
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    switch (protocol) {
        case HandProtocol::HAND_PROTO_L10: {
            std::vector<uint8_t> temp = l10_x33;
            temp.insert(temp.end(), l10_x34.begin(), l10_x34.end());
            return temp;
        }
        default:
            return std::vector<uint8_t>();
    }
}

std::vector<uint8_t> CanReceiver::getJointFaults(HandProtocol protocol)
{
    /**
     * @brief 获取当前关节故障码
     */
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    switch (protocol) {
        case HandProtocol::HAND_PROTO_L20:
            return l20_x07;
        default:
            return std::vector<uint8_t>();
    }
}

HandFeedbackData CanReceiver::getFeedbackData()
{
    /**
     * @brief 获取力反馈数据
     */
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    HandFeedbackData data = feedback_data_;
    data.normal_force = normal_force_;
    data.tangential_force = tangential_force_;
    data.tangential_force_dir = tangential_force_dir_;
    data.approach_inc = approach_inc_;
    
    return data;
}

HandFeedbackData CanReceiver::getFullFeedbackData()
{
    /**
     * @brief 获取完整反馈数据
     */
    return getFeedbackData();
}

}  // namespace linkerhand_cl

