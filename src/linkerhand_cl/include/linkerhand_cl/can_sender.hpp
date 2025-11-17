/**
 * @file can_sender.hpp
 * @brief CAN发送接口头文件
 * 
 * 提供CAN总线发送功能，支持L10/L20/L21协议
 * 使用socketcan实现底层CAN通信
 */

#ifndef LINKERHAND_CL_CAN_SENDER_HPP_
#define LINKERHAND_CL_CAN_SENDER_HPP_

#include "hand_control.hpp"
#include <cstdint>
#include <string>

namespace linkerhand_cl
{

/**
 * @brief CAN发送类
 * 
 * 提供不同协议的CAN数据发送功能
 * 使用socketcan实现底层CAN通信
 */
class CanSender
{
public:
    /**
     * @brief 构造函数
     * @param can_interface CAN接口名称（如"can0"）
     * @param bitrate CAN比特率（bps），默认1000000（1Mbps）
     */
    explicit CanSender(const std::string& can_interface = "can0", uint32_t bitrate = 1000000);
    
    /**
     * @brief 析构函数
     */
    ~CanSender();
    
    /**
     * @brief 检查CAN接口是否已打开
     * @return true:已打开, false:未打开
     */
    bool isOpen() const;
    
    /**
     * @brief 发送L10协议数据（单帧7字节协议）
     * @param device_id 设备ID
     * @param request 手部控制请求
     * @return 0:成功, -1:失败
     */
    int sendL10(uint32_t device_id, const HandControlRequest& request);
    
    /**
     * @brief 发送L20协议数据（现有多帧协议）
     * @param device_id 设备ID
     * @param request 手部控制请求
     * @return 0:成功, -1:失败
     */
    int sendL20(uint32_t device_id, const HandControlRequest& request);
    
    /**
     * @brief 发送L21协议数据（简化多帧协议）
     * @param device_id 设备ID
     * @param request 手部控制请求
     * @return 0:成功, -1:失败
     */
    int sendL21(uint32_t device_id, const HandControlRequest& request);
    
    /**
     * @brief 发送L7协议数据（7自由度单帧协议）
     * @param device_id 设备ID
     * @param request 手部控制请求
     * @return 0:成功, -1:失败
     */
    int sendL7(uint32_t device_id, const HandControlRequest& request);
    
    /**
     * @brief 发送L25协议数据（25自由度多帧协议）
     * @param device_id 设备ID
     * @param request 手部控制请求
     * @return 0:成功, -1:失败
     */
    int sendL25(uint32_t device_id, const HandControlRequest& request);
    
    /**
     * @brief 发送O6协议数据（6自由度单帧协议）
     * @param device_id 设备ID
     * @param request 手部控制请求
     * @return 0:成功, -1:失败
     */
    int sendO6(uint32_t device_id, const HandControlRequest& request);
    
    /**
     * @brief 根据协议类型发送数据
     * @param protocol 协议类型
     * @param device_id 设备ID
     * @param request 手部控制请求
     * @return 0:成功, -1:失败
     */
    int send(HandProtocol protocol, uint32_t device_id, const HandControlRequest& request);

private:
    /**
     * @brief 设置CAN接口比特率
     * @param can_interface CAN接口名称
     * @param bitrate 比特率（bps）
     * @return 0:成功, -1:失败
     */
    int setCanBitrate(const std::string& can_interface, uint32_t bitrate);
    
    /**
     * @brief 打开CAN接口
     * @param can_interface CAN接口名称
     * @return 0:成功, -1:失败
     */
    int openCanInterface(const std::string& can_interface);
    
    /**
     * @brief 关闭CAN接口
     */
    void closeCanInterface();
    
    /**
     * @brief 发送单个CAN帧
     * @param can_id CAN ID
     * @param frame_type 帧类型
     * @param data 数据
     * @param length 数据长度
     * @return 0:成功, -1:失败
     */
    int sendFrame(uint32_t can_id, HandFrameType frame_type, const uint8_t* data, uint8_t length);
    
    /**
     * @brief 计算校验和
     * @param data 数据指针
     * @param length 数据长度
     * @return 校验和
     */
    uint8_t calculateChecksum(const uint8_t* data, uint8_t length) const;
    
    int socket_fd_;              /* Socket文件描述符 */
    std::string can_interface_;  /* CAN接口名称 */
    uint32_t bitrate_;           /* CAN比特率（bps） */
    bool is_open_;               /* 是否已打开 */
};

}  // namespace linkerhand_cl

#endif  // LINKERHAND_CL_CAN_SENDER_HPP_

