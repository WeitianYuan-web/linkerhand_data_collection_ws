/**
 * @file exhand_node.cpp
 * @brief 手部控制器 ROS2 节点实现 (C++版本)
 */

#include "exhand_read/exhand_node.hpp"

#include <chrono>
#include <thread>
#include <iostream>

using namespace std::chrono_literals;

namespace exhand_read
{

ExHandNode::ExHandNode()
    : Node("exhand_node")
    , running_(false)
    , publish_rate_(20.0)
{
    // 声明参数
    this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    this->declare_parameter<int>("baudrate", 1152000);
    this->declare_parameter<double>("timeout", 1.0);
    // 注意：publish_rate 参数已废弃，现在接收一帧就发布一帧，不控制频率
    // 保留此参数仅为了向后兼容，但不再使用
    this->declare_parameter<double>("publish_rate", 20.0);
    this->declare_parameter<bool>("auto_enable", true);
    this->declare_parameter<bool>("enable_sensor_push", true);
    this->declare_parameter<bool>("enable_mapping_push", true);
    this->declare_parameter<int>("protocol", 0);
    this->declare_parameter<bool>("enable_can", false);

    // 获取参数
    std::string port = this->get_parameter("port").as_string();
    int baudrate = this->get_parameter("baudrate").as_int();
    double timeout = this->get_parameter("timeout").as_double();
    publish_rate_ = this->get_parameter("publish_rate").as_double();
    bool auto_enable = this->get_parameter("auto_enable").as_bool();
    bool enable_sensor_push = this->get_parameter("enable_sensor_push").as_bool();
    bool enable_mapping_push = this->get_parameter("enable_mapping_push").as_bool();
    int protocol = this->get_parameter("protocol").as_int();
    bool enable_can = this->get_parameter("enable_can").as_bool();

    // 初始化SDK
    try
    {
        sdk_ = std::make_unique<UartFrameCommand>(port, static_cast<uint32_t>(baudrate), timeout);
        RCLCPP_INFO(this->get_logger(), "已连接到串口: %s @ %d", port.c_str(), baudrate);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "无法连接串口: %s", e.what());
        throw;
    }

    // 创建发布者
    // 映射/传感器/状态类话题使用 BestEffort
    rclcpp::QoS qos_best_effort(10);
    qos_best_effort.reliability(rclcpp::ReliabilityPolicy::BestEffort);

    sensor_pub_right_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>(
        "exhand/sensor_data_right", qos_best_effort);
    sensor_pub_left_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>(
        "exhand/sensor_data_left", qos_best_effort);
    mapping_pub_right_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "exhand/mapping_data_right", qos_best_effort);
    mapping_pub_left_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "exhand/mapping_data_left", qos_best_effort);
    status_pub_ = this->create_publisher<std_msgs::msg::UInt8>("exhand/status", qos_best_effort);

    // 初始化数据缓存
    sensor_data_cache_[0].resize(15, 0);
    sensor_data_cache_[1].resize(15, 0);
    mapping_data_cache_[0].resize(15, 0.0f);
    mapping_data_cache_[1].resize(15, 0.0f);

    // 等待设备复位完成
    std::this_thread::sleep_for(2s);

    // 自动启用
    if (auto_enable)
    {
        autoEnable(auto_enable, enable_sensor_push, enable_mapping_push, protocol, enable_can);
    }

    // 创建定时器用于定期查询状态
    status_timer_ = this->create_wall_timer(
        1s, std::bind(&ExHandNode::publishStatus, this));

    // 启动数据接收线程
    startDataThread();

    RCLCPP_INFO(this->get_logger(), "手部控制器节点已启动");
}

ExHandNode::~ExHandNode()
{
    RCLCPP_INFO(this->get_logger(), "正在关闭节点...");
    
    // 首先停止定时器，避免在关闭过程中继续访问SDK
    if (status_timer_)
    {
        status_timer_->cancel();
        status_timer_.reset();
    }
    
    // 设置停止标志，通知数据接收线程退出
    running_ = false;

    // 等待数据接收线程退出
    // 由于线程使用10ms超时的非阻塞读取，应该能在很短时间内检测到退出标志
    if (data_thread_.joinable())
    {
        RCLCPP_INFO(this->get_logger(), "等待数据接收线程退出...");
        try
        {
            // 直接 join，线程应该能快速退出
            data_thread_.join();
            RCLCPP_INFO(this->get_logger(), "数据接收线程已正常退出");
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "等待线程退出时出错: %s", e.what());
        }
    }

    // 关闭串口
    if (sdk_)
    {
        try
        {
            RCLCPP_INFO(this->get_logger(), "正在关闭串口...");
            // 直接关闭串口，设备会自动停止推送数据
            sdk_->close();
            RCLCPP_INFO(this->get_logger(), "串口已关闭");
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN(this->get_logger(), "关闭串口时出错: %s", e.what());
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "节点已完全关闭");
}

void ExHandNode::startDataThread()
{
    if (!running_)
    {
        running_ = true;
        data_thread_ = std::thread(&ExHandNode::dataReceiveLoop, this);
        RCLCPP_INFO(this->get_logger(), "数据接收线程已启动");
    }
}

void ExHandNode::dataReceiveLoop()
{
    int error_count = 0;
    constexpr int max_silent_errors = 10;
    int sensor_count = 0;
    int mapping_count = 0;
    auto last_log_time = std::chrono::steady_clock::now();

    // 接收一帧数据就发布一次，不控制发布频率
    // 使用较短的超时时间（10ms），以便快速响应退出信号
    constexpr double receive_timeout = 0.01;  // 10ms超时

    while (running_ && rclcpp::ok())
    {
        // 检查退出条件
        if (!running_ || !rclcpp::ok())
        {
            break;
        }
        
        try
        {
            // 检查串口是否仍然打开
            if (!sdk_->isOpen())
            {
                RCLCPP_WARN(this->get_logger(), "串口已关闭，退出数据接收循环");
                break;
            }

            // 使用统一的帧接收函数，避免跳过数据帧
            // 接收一帧数据就立即发布，不控制频率
            auto frame_data = sdk_->receiveAnyDataFrame(receive_timeout);
            if (frame_data)
            {
                uint8_t hand = frame_data->hand;

                if (frame_data->frame_type == UartFrameCommand::AnyDataFrame::FrameType::SENSOR)
                {
                    RCLCPP_DEBUG(this->get_logger(), "收到传感器数据: hand=%d, 数据长度=%zu",
                                hand, frame_data->sensor_data.size());
                    try
                    {
                        publishSensorData(hand, frame_data->sensor_data);
                        error_count = 0;
                        sensor_count++;
                    }
                    catch (const std::exception& e)
                    {
                        RCLCPP_ERROR(this->get_logger(), "发布传感器数据失败: %s", e.what());
                    }
                }
                else if (frame_data->frame_type == UartFrameCommand::AnyDataFrame::FrameType::MAPPING)
                {
                    RCLCPP_DEBUG(this->get_logger(), "收到映射数据: hand=%d, 数据长度=%zu",
                                hand, frame_data->mapping_data.size());
                    try
                    {
                        publishMappingData(hand, frame_data->mapping_data);
                        error_count = 0;
                        mapping_count++;
                    }
                    catch (const std::exception& e)
                    {
                        RCLCPP_ERROR(this->get_logger(), "发布映射数据失败: %s", e.what());
                    }
                }
            }
            // 如果没有收到数据（超时），继续循环，这样可以快速响应退出信号

            // 每5秒输出一次统计信息
            auto current_log_time = std::chrono::steady_clock::now();
            if (std::chrono::duration<double>(current_log_time - last_log_time).count() >= 5.0)
            {
                RCLCPP_INFO(this->get_logger(), "数据接收统计: 传感器=%d, 映射=%d",
                           sensor_count, mapping_count);
                last_log_time = current_log_time;
            }
        }
        catch (const std::exception& e)
        {
            error_count++;
            if (error_count <= max_silent_errors)
            {
                RCLCPP_ERROR(this->get_logger(), "数据接收错误: %s (错误计数: %d)",
                           e.what(), error_count);
            }
            else if (error_count == max_silent_errors + 1)
            {
                RCLCPP_ERROR(this->get_logger(), "数据接收错误持续发生，后续错误将静默处理");
            }
            // 错误时短暂等待，但也要快速响应退出信号
            constexpr int error_sleep_ms = 10;
            for (int i = 0; i < error_sleep_ms && running_ && rclcpp::ok(); ++i)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "数据接收线程已退出");
}

void ExHandNode::publishSensorData(uint8_t hand, const std::vector<uint16_t>& data)
{
    // 验证数据
    if (hand > 1)
    {
        RCLCPP_WARN(this->get_logger(), "无效的手侧值: %d，跳过发布传感器数据", hand);
        return;
    }

    if (data.size() != 15)
    {
        RCLCPP_WARN(this->get_logger(), "传感器数据长度不正确: %zu，期望15，跳过发布", data.size());
        return;
    }

    // 创建消息
    auto msg = std::make_shared<std_msgs::msg::UInt16MultiArray>();
    msg->data = data;

    // 更新缓存
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        sensor_data_cache_[hand] = data;
    }

    // 根据左右手发布到不同话题
    try
    {
        if (hand == 0)  // 右手
        {
            sensor_pub_right_->publish(*msg);
            RCLCPP_DEBUG(this->get_logger(), "✓ 已发布右手传感器数据到 exhand/sensor_data_right: %zu 个数据点",
                        data.size());
        }
        else if (hand == 1)  // 左手
        {
            sensor_pub_left_->publish(*msg);
            RCLCPP_DEBUG(this->get_logger(), "✓ 已发布左手传感器数据到 exhand/sensor_data_left: %zu 个数据点",
                        data.size());
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "发布传感器数据到话题时出错: %s", e.what());
    }
}

void ExHandNode::publishMappingData(uint8_t hand, const std::vector<float>& data)
{
    // 验证数据
    if (hand > 1)
    {
        RCLCPP_WARN(this->get_logger(), "无效的手侧值: %d，跳过发布映射数据", hand);
        return;
    }

    if (data.size() != 15)
    {
        RCLCPP_WARN(this->get_logger(), "映射数据长度不正确: %zu，期望15，跳过发布", data.size());
        return;
    }

    // 创建消息
    auto msg = std::make_shared<std_msgs::msg::Float32MultiArray>();
    msg->data = data;

    // 更新缓存
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        mapping_data_cache_[hand] = data;
    }

    // 根据左右手发布到不同话题
    try
    {
        if (hand == 0)  // 右手
        {
            mapping_pub_right_->publish(*msg);
            RCLCPP_DEBUG(this->get_logger(), "✓ 已发布右手映射数据到 exhand/mapping_data_right: %zu 个数据点",
                        data.size());
        }
        else if (hand == 1)  // 左手
        {
            mapping_pub_left_->publish(*msg);
            RCLCPP_DEBUG(this->get_logger(), "✓ 已发布左手映射数据到 exhand/mapping_data_left: %zu 个数据点",
                        data.size());
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "发布映射数据到话题时出错: %s", e.what());
    }
}

void ExHandNode::publishStatus()
{
    try
    {
        auto status = sdk_->status();
        if (status)
        {
            // 可以发布状态信息（这里简化处理）
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_DEBUG(this->get_logger(), "状态查询失败: %s", e.what());
    }
}

template<typename Func>
bool ExHandNode::sendCommandWithRetry(const std::string& command_name, Func command_func, 
                                      int max_retries, int retry_delay_ms)
{
    for (int attempt = 1; attempt <= max_retries; ++attempt)
    {
        try
        {
            auto resp = command_func();
            
            // 检查响应
            if (resp && resp->result == 0)
            {
                if (attempt > 1)
                {
                    RCLCPP_INFO(this->get_logger(), "✓ %s 成功（尝试 %d/%d）", 
                               command_name.c_str(), attempt, max_retries);
                }
                return true;
            }
            else if (resp)
            {
                // 收到响应但结果不成功
                if (attempt < max_retries)
                {
                    RCLCPP_WARN(this->get_logger(), "%s 失败（结果码: %d），尝试 %d/%d，等待 %dms 后重试...", 
                               command_name.c_str(), resp->result, attempt, max_retries, retry_delay_ms);
                    std::this_thread::sleep_for(std::chrono::milliseconds(retry_delay_ms));
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "✗ %s 失败（结果码: %d），已达最大重试次数 %d", 
                                command_name.c_str(), resp->result, max_retries);
                }
            }
            else
            {
                // 没有收到响应
                if (attempt < max_retries)
                {
                    RCLCPP_WARN(this->get_logger(), "%s 没有响应，尝试 %d/%d，等待 %dms 后重试...", 
                               command_name.c_str(), attempt, max_retries, retry_delay_ms);
                    std::this_thread::sleep_for(std::chrono::milliseconds(retry_delay_ms));
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "✗ %s 没有响应，已达最大重试次数 %d", 
                                command_name.c_str(), max_retries);
                }
            }
        }
        catch (const std::exception& e)
        {
            if (attempt < max_retries)
            {
                RCLCPP_WARN(this->get_logger(), "%s 出错: %s，尝试 %d/%d，等待 %dms 后重试...", 
                           command_name.c_str(), e.what(), attempt, max_retries, retry_delay_ms);
                std::this_thread::sleep_for(std::chrono::milliseconds(retry_delay_ms));
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "✗ %s 出错: %s，已达最大重试次数 %d", 
                            command_name.c_str(), e.what(), max_retries);
            }
        }
    }
    
    return false;
}

void ExHandNode::autoEnable(bool auto_enable, bool enable_sensor_push, bool enable_mapping_push, int protocol, bool enable_can)
{
    if (!auto_enable) return;

    RCLCPP_INFO(this->get_logger(), "自动启用配置: 传感器推送=%s, 映射推送=%s",
                enable_sensor_push ? "是" : "否", enable_mapping_push ? "是" : "否");

    try
    {
        // 等待串口完全初始化并清空缓冲区
        std::this_thread::sleep_for(500ms);

        // 清空缓冲区
        try
        {
            size_t waiting = sdk_->bytesAvailable();
            if (waiting > 0)
            {
                RCLCPP_INFO(this->get_logger(), "清空缓冲区中的 %zu 字节数据", waiting);
                // 读取并丢弃
                std::vector<uint8_t> buffer(waiting);
                // 注意：这里需要实际读取，但UartFrameCommand没有提供直接读取方法
                // 暂时跳过，在实际使用中可以通过receiveAnyDataFrame来清空
                std::this_thread::sleep_for(100ms);
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN(this->get_logger(), "清空缓冲区失败: %s", e.what());
        }

        // 启用数据帧模式（必须启用）
        RCLCPP_INFO(this->get_logger(), "正在启用数据帧模式...");
        sendCommandWithRetry("启用数据帧模式", [this]() { return sdk_->enable(); });
        std::this_thread::sleep_for(50ms);

        // 启用传感器数据推送（如果启用）
        if (enable_sensor_push)
        {
            RCLCPP_INFO(this->get_logger(), "正在启用传感器数据推送...");
            sendCommandWithRetry("启用传感器数据推送", [this]() { return sdk_->sensorEnable(); });
            std::this_thread::sleep_for(50ms);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "跳过启用传感器数据推送（已禁用）");
        }

        // 启用映射数据推送（如果启用）
        if (enable_mapping_push)
        {
            RCLCPP_INFO(this->get_logger(), "正在启用映射数据推送...");
            sendCommandWithRetry("启用映射数据推送", [this]() { return sdk_->mappingEnable(); });
            std::this_thread::sleep_for(50ms);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "跳过启用映射数据推送（已禁用）");
        }

        RCLCPP_INFO(this->get_logger(), "✓ 已完成自动启用配置");

        // 设置协议类型
        const char* protocol_names[] = {"L20", "L10", "L21"};
        if (protocol >= 0 && protocol <= 2)
        {
            RCLCPP_INFO(this->get_logger(), "正在设置协议为 %s...", protocol_names[protocol]);
            bool success = sendCommandWithRetry("设置协议", 
                [this, protocol]() { return sdk_->setProtocol(static_cast<uint8_t>(protocol)); });
            if (success)
            {
                RCLCPP_INFO(this->get_logger(), "✓ 协议设置成功: %s", protocol_names[protocol]);
            }
            std::this_thread::sleep_for(100ms);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "无效的协议ID: %d，跳过协议设置", protocol);
        }

        // 启用/禁用CAN总线控制
        if (enable_can)
        {
            RCLCPP_INFO(this->get_logger(), "正在启用CAN总线控制...");
            bool success = sendCommandWithRetry("启用CAN总线控制", 
                [this]() { return sdk_->canEnable(); });
            if (success)
            {
                RCLCPP_INFO(this->get_logger(), "✓ CAN总线控制已启用");
            }
            std::this_thread::sleep_for(100ms);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "正在禁用CAN总线控制...");
            bool success = sendCommandWithRetry("禁用CAN总线控制", 
                [this]() { return sdk_->canDisable(); });
            if (success)
            {
                RCLCPP_INFO(this->get_logger(), "✓ CAN总线控制已禁用");
            }
            std::this_thread::sleep_for(100ms);
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "自动启用失败: %s", e.what());
    }
}

}  // namespace exhand_read

// 主函数
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    std::shared_ptr<exhand_read::ExHandNode> node = nullptr;
    
    try
    {
        node = std::make_shared<exhand_read::ExHandNode>();
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        std::cerr << "节点运行错误: " << e.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "节点运行错误: 未知异常" << std::endl;
    }
    
    // 确保节点被正确销毁
    node.reset();
    
    rclcpp::shutdown();
    return 0;
}

