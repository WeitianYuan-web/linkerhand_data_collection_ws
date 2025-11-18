/**
 * @file can_receiver.hpp
 * @brief CAN接收接口头文件
 * 
 * 提供CAN总线接收功能，用于接收灵巧手反馈数据
 * 使用socketcan实现底层CAN通信
 */

#ifndef LINKERHAND_CL_CAN_RECEIVER_HPP_
#define LINKERHAND_CL_CAN_RECEIVER_HPP_

#include "hand_control.hpp"
#include <cstdint>
#include <string>
#include <vector>
#include <array>
#include <thread>
#include <atomic>
#include <mutex>
#include <map>
#include <linux/can.h>

namespace linkerhand_cl
{

/**
 * @brief 手部反馈数据结构
 */
struct HandFeedbackData
{
    std::vector<uint8_t> joint_positions;      /* 关节位置数据 */
    std::vector<uint8_t> joint_velocities;     /* 关节速度数据 */
    std::vector<uint8_t> joint_currents;       /* 关节电流数据 */
    std::vector<uint8_t> joint_temperatures;   /* 关节温度数据 */
    std::vector<uint8_t> joint_faults;          /* 关节故障码 */
    std::vector<float> normal_force;           /* 法向力（5个手指） */
    std::vector<float> tangential_force;      /* 切向力（5个手指） */
    std::vector<float> tangential_force_dir;  /* 切向力方向（5个手指） */
    std::vector<float> approach_inc;          /* 接近增量（5个手指） */
    
    // 矩阵触觉数据（5个手指，每个12x6）
    std::array<std::array<std::array<float, 6>, 12>, 5> matrix_touch;
    
    std::vector<uint8_t> version;             /* 版本信息 */
    
    bool data_valid;                          /* 数据是否有效 */
    
    HandFeedbackData() : data_valid(false) {
        matrix_touch.fill({});
    }
};

/**
 * @brief CAN接收类
 * 
 * 提供不同协议的CAN数据接收功能
 * 使用socketcan实现底层CAN通信
 */
class CanReceiver
{
public:
    /**
     * @brief 构造函数
     * @param can_interface CAN接口名称（如"can0"）
     * @param bitrate CAN比特率（bps），默认1000000（1Mbps）
     * @param device_id 设备ID（用于过滤CAN消息）
     */
    explicit CanReceiver(const std::string& can_interface = "can0", 
                        uint32_t bitrate = 1000000,
                        uint32_t device_id = 0x28);
    
    /**
     * @brief 析构函数
     */
    ~CanReceiver();
    
    /**
     * @brief 检查CAN接口是否已打开
     * @return true:已打开, false:未打开
     */
    bool isOpen() const;
    
    /**
     * @brief 启动接收线程
     * @return 0:成功, -1:失败
     */
    int start();
    
    /**
     * @brief 停止接收线程
     */
    void stop();
    
    /**
     * @brief 请求状态数据（根据协议类型）
     * @param protocol 协议类型
     * @return 0:成功, -1:失败
     */
    int requestStatus(HandProtocol protocol);
    
    /**
     * @brief 获取当前关节位置
     * @param protocol 协议类型
     * @return 关节位置数据（根据协议返回不同长度）
     */
    std::vector<uint8_t> getJointPositions(HandProtocol protocol);
    
    /**
     * @brief 获取当前关节速度
     * @param protocol 协议类型
     * @return 关节速度数据
     */
    std::vector<uint8_t> getJointVelocities(HandProtocol protocol);
    
    /**
     * @brief 获取当前关节电流
     * @param protocol 协议类型
     * @return 关节电流数据
     */
    std::vector<uint8_t> getJointCurrents(HandProtocol protocol);
    
    /**
     * @brief 获取当前关节温度
     * @param protocol 协议类型
     * @return 关节温度数据
     */
    std::vector<uint8_t> getJointTemperatures(HandProtocol protocol);
    
    /**
     * @brief 获取当前关节故障码
     * @param protocol 协议类型
     * @return 关节故障码数据
     */
    std::vector<uint8_t> getJointFaults(HandProtocol protocol);
    
    /**
     * @brief 获取力反馈数据
     * @return 力反馈数据结构
     */
    HandFeedbackData getFeedbackData();
    
    /**
     * @brief 获取完整反馈数据
     * @return 反馈数据结构
     */
    HandFeedbackData getFullFeedbackData();

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
     * @brief 接收线程函数
     */
    void receiveThread();
    
    /**
     * @brief 处理接收到的CAN消息
     * @param frame CAN帧数据
     */
    void processResponse(const struct can_frame& frame);
    
    /**
     * @brief 处理L10协议反馈数据
     * @param frame_type 帧类型
     * @param data 数据
     * @param length 数据长度
     */
    void processL10Response(uint8_t frame_type, const uint8_t* data, uint8_t length);
    
    /**
     * @brief 处理L20协议反馈数据
     * @param frame_type 帧类型
     * @param data 数据
     * @param length 数据长度
     */
    void processL20Response(uint8_t frame_type, const uint8_t* data, uint8_t length);
    
    /**
     * @brief 处理L21协议反馈数据
     * @param frame_type 帧类型
     * @param data 数据
     * @param length 数据长度
     */
    void processL21Response(uint8_t frame_type, const uint8_t* data, uint8_t length);
    
    /**
     * @brief 处理L7协议反馈数据
     * @param frame_type 帧类型
     * @param data 数据
     * @param length 数据长度
     */
    void processL7Response(uint8_t frame_type, const uint8_t* data, uint8_t length);
    
    /**
     * @brief 处理L25协议反馈数据
     * @param frame_type 帧类型
     * @param data 数据
     * @param length 数据长度
     */
    void processL25Response(uint8_t frame_type, const uint8_t* data, uint8_t length);
    
    /**
     * @brief 处理O6协议反馈数据
     * @param frame_type 帧类型
     * @param data 数据
     * @param length 数据长度
     */
    void processO6Response(uint8_t frame_type, const uint8_t* data, uint8_t length);
    
    /**
     * @brief 发送请求帧
     * @param frame_type 帧类型
     * @param data 数据
     * @param length 数据长度
     * @return 0:成功, -1:失败
     */
    int sendRequestFrame(uint8_t frame_type, const uint8_t* data, uint8_t length);
    
    int socket_fd_;              /* Socket文件描述符 */
    std::string can_interface_;  /* CAN接口名称 */
    uint32_t bitrate_;           /* CAN比特率（bps） */
    uint32_t device_id_;         /* 设备ID */
    bool is_open_;               /* 是否已打开 */
    
    std::atomic<bool> running_;  /* 接收线程运行标志 */
    std::thread receive_thread_; /* 接收线程 */
    std::mutex data_mutex_;      /* 数据保护互斥锁 */
    
    // 数据存储（根据协议类型存储）
    HandFeedbackData feedback_data_;  /* 反馈数据 */
    
    // L10数据缓存
    std::vector<uint8_t> l10_x01;  /* L10位置数据1（前6个关节） */
    std::vector<uint8_t> l10_x04;  /* L10位置数据2（后4个关节） */
    std::vector<uint8_t> l10_x02;  /* L10转矩数据1（前5个关节） */
    std::vector<uint8_t> l10_x03;  /* L10转矩数据2（后5个关节） */
    std::vector<uint8_t> l10_x05;  /* L10速度数据1（前5个关节） */
    std::vector<uint8_t> l10_x06;  /* L10速度数据2（后5个关节） */
    std::vector<uint8_t> l10_x33;  /* L10温度数据1 */
    std::vector<uint8_t> l10_x34;  /* L10温度数据2 */
    
    // L20数据缓存
    std::vector<uint8_t> l20_x01;  /* L20 Pitch位置（指根弯曲） */
    std::vector<uint8_t> l20_x02;  /* L20 Yaw位置（侧摆） */
    std::vector<uint8_t> l20_x03;  /* L20 Roll位置（拇指旋转） */
    std::vector<uint8_t> l20_x04;  /* L20 Tip位置（指尖弯曲） */
    std::vector<uint8_t> l20_x05;  /* L20速度 */
    std::vector<uint8_t> l20_x06;  /* L20电流 */
    std::vector<uint8_t> l20_x07;  /* L20故障码 */
    
    // L21数据缓存
    std::vector<uint8_t> l21_x41;  /* L21 拇指位置（6个关节） */
    std::vector<uint8_t> l21_x42;  /* L21 食指位置（6个关节） */
    std::vector<uint8_t> l21_x43;  /* L21 中指位置（6个关节） */
    std::vector<uint8_t> l21_x44;  /* L21 无名指位置（6个关节） */
    std::vector<uint8_t> l21_x45;  /* L21 小指位置（6个关节） */
    std::vector<uint8_t> l21_x49;  /* L21 拇指速度（6个关节） */
    std::vector<uint8_t> l21_x4a;  /* L21 食指速度（6个关节） */
    std::vector<uint8_t> l21_x4b;  /* L21 中指速度（6个关节） */
    std::vector<uint8_t> l21_x4c;  /* L21 无名指速度（6个关节） */
    std::vector<uint8_t> l21_x4d;  /* L21 小指速度（6个关节） */
    std::vector<uint8_t> l21_x51;  /* L21 拇指转矩（6个关节） */
    std::vector<uint8_t> l21_x52;  /* L21 食指转矩（6个关节） */
    std::vector<uint8_t> l21_x53;  /* L21 中指转矩（6个关节） */
    std::vector<uint8_t> l21_x54;  /* L21 无名指转矩（6个关节） */
    std::vector<uint8_t> l21_x55;  /* L21 小指转矩（6个关节） */
    
    // L7数据缓存
    std::vector<uint8_t> l7_x01;   /* L7位置数据（7个关节） */
    std::vector<uint8_t> l7_x02;   /* L7转矩数据（7个关节） */
    std::vector<uint8_t> l7_x05;   /* L7速度数据（7个关节） */
    
    // L25数据缓存
    std::vector<uint8_t> l25_x01;  /* L25 Roll位置 */
    std::vector<uint8_t> l25_x02;  /* L25 Yaw位置 */
    std::vector<uint8_t> l25_x03;  /* L25 Root1位置 */
    std::vector<uint8_t> l25_x04;  /* L25 Root2位置 */
    std::vector<uint8_t> l25_x06;  /* L25 Tip位置 */
    
    // O6数据缓存
    std::vector<uint8_t> o6_x01;   /* O6位置数据（6个关节） */
    std::vector<uint8_t> o6_x02;   /* O6转矩数据（6个关节） */
    std::vector<uint8_t> o6_x05;   /* O6速度数据（6个关节） */
    
    // 通用数据缓存（力反馈、触觉等）
    std::vector<float> normal_force_;          /* 法向力 */
    std::vector<float> tangential_force_;      /* 切向力 */
    std::vector<float> tangential_force_dir_;  /* 切向力方向 */
    std::vector<float> approach_inc_;          /* 接近增量 */
    
    // 矩阵触觉数据映射
    std::map<uint8_t, uint8_t> matrix_map_;  /* 矩阵索引映射 */
};

}  // namespace linkerhand_cl

#endif  // LINKERHAND_CL_CAN_RECEIVER_HPP_

