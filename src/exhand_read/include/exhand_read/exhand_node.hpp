/**
 * @file exhand_node.hpp
 * @brief 手部控制器 ROS2 节点 (C++版本)
 * 
 * 实现手部控制器的 ROS2 接口，发布传感器数据和映射数据。
 */

#ifndef EXHAND_READ_EXHAND_NODE_HPP_
#define EXHAND_READ_EXHAND_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <vector>
#include <string>

#include "exhand_read/uart_frame_command.hpp"
#include "exhand_read/hand_control.hpp"

namespace exhand_read
{

/**
 * @brief 手部控制器 ROS2 节点类
 */
class ExHandNode : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数
     */
    ExHandNode();

    /**
     * @brief 析构函数
     */
    ~ExHandNode();

private:
    /**
     * @brief 启动数据接收线程
     */
    void startDataThread();

    /**
     * @brief 数据接收循环（在独立线程中运行）
     */
    void dataReceiveLoop();

    /**
     * @brief 发布传感器数据
     */
    void publishSensorData(uint8_t hand, const std::vector<uint16_t>& data);

    /**
     * @brief 发布映射数据
     */
    void publishMappingData(uint8_t hand, const std::vector<float>& data);

    /**
     * @brief 发布控制命令到ROS2话题
     * @param device_id 设备ID
     * @param request 控制请求数据
     */
    void publishControlCommand(uint32_t device_id, const HandControlRequest& request);

    /**
     * @brief 从映射数据设置手部控制
     * @param device_id 设备ID
     * @param mapping_data 映射数据（15个值）
     * @param start_finger 起始手指索引
     */
    void setHandControlFromMapping(uint32_t device_id, 
                                    const std::vector<float>& mapping_data,
                                    FingerIndex start_finger);

    /**
     * @brief 中点死区处理函数
     * @details 对于Yaw轴（除了大拇指），实现中点死区处理：
     *   - 当值在0.35-0.65之间时，输出0.5（死区中心）
     *   - 当值在0-0.35之间时，映射到0-0.5
     *   - 当值在0.65-1.0之间时，映射到0.5-1.0
     * @param input_val 输入值（0.0-1.0）
     * @return 处理后的值（0.0-1.0）
     */
    float applyYawDeadzone(float input_val);

    /**
     * @brief 定期发布系统状态
     */
    void publishStatus();

    /**
     * @brief 自动启用设备
     * @param auto_enable 是否自动启用配置
     * @param enable_sensor_push 是否启用传感器数据推送
     * @param enable_mapping_push 是否启用映射数据推送
     * @param protocol 协议类型
     * @param enable_can 是否启用CAN总线控制
     */
    void autoEnable(bool auto_enable, bool enable_sensor_push, bool enable_mapping_push, int protocol, bool enable_can);

    /**
     * @brief 带重试机制的命令发送辅助函数
     * @param command_name 命令名称（用于日志）
     * @param command_func 命令函数
     * @param max_retries 最大重试次数（默认3次）
     * @param retry_delay_ms 重试延迟毫秒数（默认50ms）
     * @return 如果命令成功执行返回true，否则返回false
     */
    template<typename Func>
    bool sendCommandWithRetry(const std::string& command_name, Func command_func, 
                              int max_retries = 3, int retry_delay_ms = 50);

    // ROS2 发布者
    rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr sensor_pub_right_;
    rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr sensor_pub_left_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr mapping_pub_right_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr mapping_pub_left_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr status_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_pub_right_;     /* 右手控制命令发布者 */
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_pub_left_;      /* 左手控制命令发布者 */

    // 定时器
    rclcpp::TimerBase::SharedPtr status_timer_;

    // 串口通信SDK
    std::unique_ptr<UartFrameCommand> sdk_;

    // 手部控制对象
    HandControl hand_control_;

    // 数据缓存
    std::mutex data_mutex_;
    std::vector<uint16_t> sensor_data_cache_[2];  // 0=右手, 1=左手
    std::vector<float> mapping_data_cache_[2];     // 0=右手, 1=左手

    // 线程控制
    std::atomic<bool> running_;
    std::thread data_thread_;

    // 参数
    double publish_rate_;
    uint32_t right_hand_id_;                       /* 右手设备ID */
    uint32_t left_hand_id_;                        /* 左手设备ID */
};

}  // namespace exhand_read

#endif  // EXHAND_READ_EXHAND_NODE_HPP_

