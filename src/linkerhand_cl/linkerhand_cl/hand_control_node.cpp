/**
 * @file hand_control_node.cpp
 * @brief LinkerHand控制ROS2节点
 * 
 * 订阅EXHand发布的映射数据话题，控制灵巧手
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "linkerhand_cl/hand_control.hpp"
#include "linkerhand_cl/can_sender.hpp"
#include "linkerhand_cl/can_receiver.hpp"
#include <memory>
#include <chrono>
#include <fstream>
#include <string>
#include <map>
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std::chrono_literals;

namespace linkerhand_cl
{

/**
 * @brief LinkerHand控制节点类
 */
class HandControlNode : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数
     */
    HandControlNode()
        : Node("hand_control_node")
        , hand_control_()
        , can_sender_(nullptr)
    {
        // 声明参数
        this->declare_parameter<std::string>("mapping_topic_right", "exhand/mapping_data_right");
        this->declare_parameter<std::string>("mapping_topic_left", "exhand/mapping_data_left");
        this->declare_parameter<int>("right_hand_id", static_cast<int>(HAND_ID_RIGHT));
        this->declare_parameter<int>("left_hand_id", static_cast<int>(HAND_ID_LEFT));
        this->declare_parameter<int>("protocol", static_cast<int>(HandProtocol::HAND_PROTO_L10));
        this->declare_parameter<std::string>("right_hand_model", "");
        this->declare_parameter<std::string>("left_hand_model", "");
        this->declare_parameter<std::string>("task_config_path", "");
        this->declare_parameter<bool>("enable_can", false);
        this->declare_parameter<std::string>("can_interface", "can0");
        this->declare_parameter<int>("can_bitrate", 1000000);  // CAN比特率（bps），默认1Mbps
        this->declare_parameter<bool>("enable_feedback", true);  // 是否启用反馈数据接收和发布
        this->declare_parameter<double>("feedback_publish_rate", 50.0);  // 反馈数据发布频率（Hz）
        
        // 获取参数
        std::string mapping_topic_right = this->get_parameter("mapping_topic_right").as_string();
        std::string mapping_topic_left = this->get_parameter("mapping_topic_left").as_string();
        right_hand_id_ = static_cast<uint32_t>(this->get_parameter("right_hand_id").as_int());
        left_hand_id_ = static_cast<uint32_t>(this->get_parameter("left_hand_id").as_int());
        std::string right_hand_model = this->get_parameter("right_hand_model").as_string();
        std::string left_hand_model = this->get_parameter("left_hand_model").as_string();
        std::string task_config_path = this->get_parameter("task_config_path").as_string();
        bool enable_can = this->get_parameter("enable_can").as_bool();
        std::string can_interface = this->get_parameter("can_interface").as_string();
        uint32_t can_bitrate = static_cast<uint32_t>(this->get_parameter("can_bitrate").as_int());
        
        // 从task_config.json读取配置（如果提供了路径）
        int right_protocol_id = static_cast<int>(HandProtocol::HAND_PROTO_L10);
        int left_protocol_id = static_cast<int>(HandProtocol::HAND_PROTO_L10);
        
        if (!task_config_path.empty()) {
            std::map<std::string, int> hand_model_to_protocol = {
                {"L10", static_cast<int>(HandProtocol::HAND_PROTO_L10)},
                {"L10V7", static_cast<int>(HandProtocol::HAND_PROTO_L10)},
                {"L20", static_cast<int>(HandProtocol::HAND_PROTO_L20)},
                {"L21", static_cast<int>(HandProtocol::HAND_PROTO_L21)},
                {"L7", static_cast<int>(HandProtocol::HAND_PROTO_L7)},
                {"O7", static_cast<int>(HandProtocol::HAND_PROTO_L7)},
                {"L25", static_cast<int>(HandProtocol::HAND_PROTO_L25)},
                {"T25", static_cast<int>(HandProtocol::HAND_PROTO_L25)},
                {"O6", static_cast<int>(HandProtocol::HAND_PROTO_O6)},
                {"L24", static_cast<int>(HandProtocol::HAND_PROTO_L24)}
            };
            
            // 读取task_config.json
            std::ifstream config_file(task_config_path);
            if (config_file.is_open()) {
                std::string json_content((std::istreambuf_iterator<char>(config_file)),
                                        std::istreambuf_iterator<char>());
                config_file.close();
                
                // 简单的JSON解析（查找handModel字段）
                // 注意：这里使用简单的字符串查找，实际项目中建议使用JSON库
                size_t hand_model_pos = json_content.find("\"handModel\"");
                if (hand_model_pos != std::string::npos) {
                    size_t colon_pos = json_content.find(":", hand_model_pos);
                    size_t quote1_pos = json_content.find("\"", colon_pos);
                    size_t quote2_pos = json_content.find("\"", quote1_pos + 1);
                    if (quote1_pos != std::string::npos && quote2_pos != std::string::npos) {
                        std::string hand_model = json_content.substr(quote1_pos + 1, 
                                                                     quote2_pos - quote1_pos - 1);
                        if (hand_model_to_protocol.find(hand_model) != hand_model_to_protocol.end()) {
                            right_protocol_id = hand_model_to_protocol[hand_model];
                            left_protocol_id = hand_model_to_protocol[hand_model];
                            RCLCPP_INFO(this->get_logger(), "从task_config.json读取到handModel: %s, 协议ID: %d", 
                                       hand_model.c_str(), right_protocol_id);
                        }
                    }
                }
                
                // 查找handSide字段（如果存在）
                size_t hand_side_pos = json_content.find("\"handSide\"");
                if (hand_side_pos != std::string::npos) {
                    size_t colon_pos = json_content.find(":", hand_side_pos);
                    size_t quote1_pos = json_content.find("\"", colon_pos);
                    size_t quote2_pos = json_content.find("\"", quote1_pos + 1);
                    if (quote1_pos != std::string::npos && quote2_pos != std::string::npos) {
                        std::string hand_side = json_content.substr(quote1_pos + 1, 
                                                                    quote2_pos - quote1_pos - 1);
                        RCLCPP_INFO(this->get_logger(), "从task_config.json读取到handSide: %s", hand_side.c_str());
                    }
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "无法打开task_config.json文件: %s", task_config_path.c_str());
            }
        }
        
        // 如果通过参数指定了hand_model，优先使用参数
        if (!right_hand_model.empty() || !left_hand_model.empty()) {
            std::map<std::string, int> hand_model_to_protocol = {
                {"L10", static_cast<int>(HandProtocol::HAND_PROTO_L10)},
                {"L10V7", static_cast<int>(HandProtocol::HAND_PROTO_L10)},
                {"L20", static_cast<int>(HandProtocol::HAND_PROTO_L20)},
                {"L21", static_cast<int>(HandProtocol::HAND_PROTO_L21)},
                {"L7", static_cast<int>(HandProtocol::HAND_PROTO_L7)},
                {"O7", static_cast<int>(HandProtocol::HAND_PROTO_L7)},
                {"L25", static_cast<int>(HandProtocol::HAND_PROTO_L25)},
                {"T25", static_cast<int>(HandProtocol::HAND_PROTO_L25)},
                {"O6", static_cast<int>(HandProtocol::HAND_PROTO_O6)},
                {"L24", static_cast<int>(HandProtocol::HAND_PROTO_L24)}
            };
            
            if (!right_hand_model.empty() && hand_model_to_protocol.find(right_hand_model) != hand_model_to_protocol.end()) {
                right_protocol_id = hand_model_to_protocol[right_hand_model];
            }
            if (!left_hand_model.empty() && hand_model_to_protocol.find(left_hand_model) != hand_model_to_protocol.end()) {
                left_protocol_id = hand_model_to_protocol[left_hand_model];
            }
        } else {
            // 使用默认协议ID（从参数获取）
            int protocol_id = this->get_parameter("protocol").as_int();
            right_protocol_id = protocol_id;
            left_protocol_id = protocol_id;
        }
        
        // 设置协议（使用右手协议作为默认，左手可以单独设置）
        if (right_protocol_id >= 0 && right_protocol_id <= 6) {
            hand_control_.setProtocol(static_cast<HandProtocol>(right_protocol_id));
            RCLCPP_INFO(this->get_logger(), "右手协议设置为: %d", right_protocol_id);
        }
        if (left_protocol_id >= 0 && left_protocol_id <= 6 && left_protocol_id != right_protocol_id) {
            RCLCPP_INFO(this->get_logger(), "左手协议设置为: %d (注意：当前实现使用统一协议)", left_protocol_id);
        }
        
        // 初始化设备
        if (hand_control_.initDevice(right_hand_id_) == 0) {
            RCLCPP_INFO(this->get_logger(), "右手设备初始化成功: 0x%02X", right_hand_id_);
        }
        if (hand_control_.initDevice(left_hand_id_) == 0) {
            RCLCPP_INFO(this->get_logger(), "左手设备初始化成功: 0x%02X", left_hand_id_);
        }
        
        // 获取反馈相关参数
        bool enable_feedback = this->get_parameter("enable_feedback").as_bool();
        double feedback_publish_rate = this->get_parameter("feedback_publish_rate").as_double();
        
        // 初始化CAN发送器（如果启用）
        if (enable_can) {
            can_sender_ = std::make_unique<CanSender>(can_interface, can_bitrate);
            if (can_sender_->isOpen()) {
                RCLCPP_INFO(this->get_logger(), "CAN发送已启用，接口: %s，比特率: %u bps", 
                           can_interface.c_str(), can_bitrate);
            } else {
                RCLCPP_ERROR(this->get_logger(), "CAN接口打开失败: %s，比特率: %u bps", 
                            can_interface.c_str(), can_bitrate);
                can_sender_.reset();
            }
            
            // 初始化CAN接收器（如果启用反馈）
            if (enable_feedback) {
                // 为左右手分别创建接收器
                can_receiver_right_ = std::make_unique<CanReceiver>(can_interface, can_bitrate, right_hand_id_);
                can_receiver_left_ = std::make_unique<CanReceiver>(can_interface, can_bitrate, left_hand_id_);
                
                if (can_receiver_right_->isOpen() && can_receiver_left_->isOpen()) {
                    // 启动接收线程
                    can_receiver_right_->start();
                    can_receiver_left_->start();
                    
                    // 创建反馈数据发布者
                    rclcpp::QoS feedback_qos(10);
                    feedback_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
                    
                    state_pub_right_ = this->create_publisher<sensor_msgs::msg::JointState>(
                        "/cb_right_hand_state", feedback_qos);
                    state_pub_left_ = this->create_publisher<sensor_msgs::msg::JointState>(
                        "/cb_left_hand_state", feedback_qos);
                    
                    // 创建定时器定期发布反馈数据
                    double feedback_interval = 1.0 / feedback_publish_rate;
                    feedback_timer_ = this->create_wall_timer(
                        std::chrono::duration<double>(feedback_interval),
                        std::bind(&HandControlNode::publishFeedback, this)
                    );
                    
                    RCLCPP_INFO(this->get_logger(), "CAN反馈接收已启用，发布频率: %.1f Hz", feedback_publish_rate);
                } else {
                    RCLCPP_WARN(this->get_logger(), "CAN接收器打开失败，反馈功能将不可用");
                    can_receiver_right_.reset();
                    can_receiver_left_.reset();
                }
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "CAN发送未启用（仅模拟模式）");
        }
        
        // 创建控制命令发布者（用于数据采集，无论是否启用CAN反馈都需要）
        rclcpp::QoS cmd_qos(10);
        cmd_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        
        cmd_pub_right_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/cb_right_hand_control_cmd", cmd_qos);
        cmd_pub_left_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/cb_left_hand_control_cmd", cmd_qos);
        
        RCLCPP_INFO(this->get_logger(), "控制命令发布者已创建: /cb_right_hand_control_cmd, /cb_left_hand_control_cmd");
        
        // 创建订阅者，使用BEST_EFFORT QoS以匹配发布者
        rclcpp::QoS qos_profile(10);
        qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        
        // 订阅右手映射数据话题
        mapping_sub_right_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            mapping_topic_right,
            qos_profile,
            std::bind(&HandControlNode::rightHandMappingCallback, this, std::placeholders::_1)
        );
        
        // 订阅左手映射数据话题
        mapping_sub_left_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            mapping_topic_left,
            qos_profile,
            std::bind(&HandControlNode::leftHandMappingCallback, this, std::placeholders::_1)
        );
        
        RCLCPP_INFO(this->get_logger(), 
                   "LinkerHand控制节点已启动，订阅话题: %s, %s", 
                   mapping_topic_right.c_str(), mapping_topic_left.c_str());
    }
    
    /**
     * @brief 析构函数
     */
    ~HandControlNode()
    {
    }

private:
    /**
     * @brief 右手映射数据回调函数
     * @param msg 映射数据消息（15个数据）
     */
    void rightHandMappingCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        /**
         * @brief 右手映射数据回调函数
         * 
         * 当收到一帧右手数据时，立即处理并发送右手CAN数据
         */
        
        // 获取映射数据
        const auto& mapping_data = msg->data;
        
        // 检查数据长度
        if (mapping_data.size() != 15) {
            RCLCPP_WARN(this->get_logger(), 
                       "收到异常长度的右手映射数据: %zu，期望15个数据", mapping_data.size());
            return;
        }
        
        // 将映射数据转换为手部控制命令并发送
        // 假设映射数据对应15个关节（每个手指3个关节：Yaw, Pitch, Tip）
        // 拇指: 0, 1, 2
        // 食指: 3, 4, 5
        // 中指: 6, 7, 8
        // 无名指: 9, 10, 11
        // 小指: 12, 13, 14
        
        // 控制右手
        setHandControlFromMapping(right_hand_id_, mapping_data, FingerIndex::FINGER_THUMB);
        sendHandControl(right_hand_id_, mapping_data);
        
        RCLCPP_DEBUG(this->get_logger(), 
                    "收到右手映射数据: [%.3f, %.3f, %.3f]...",
                    mapping_data[0], mapping_data[1], mapping_data[2]);
    }
    
    /**
     * @brief 左手映射数据回调函数
     * @param msg 映射数据消息（15个数据）
     */
    void leftHandMappingCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        /**
         * @brief 左手映射数据回调函数
         * 
         * 当收到一帧左手数据时，立即处理并发送左手CAN数据
         */
        
        // 获取映射数据
        const auto& mapping_data = msg->data;
        
        // 检查数据长度
        if (mapping_data.size() != 15) {
            RCLCPP_WARN(this->get_logger(), 
                       "收到异常长度的左手映射数据: %zu，期望15个数据", mapping_data.size());
            return;
        }
        
        // 将映射数据转换为手部控制命令并发送
        // 假设映射数据对应15个关节（每个手指3个关节：Yaw, Pitch, Tip）
        // 拇指: 0, 1, 2
        // 食指: 3, 4, 5
        // 中指: 6, 7, 8
        // 无名指: 9, 10, 11
        // 小指: 12, 13, 14
        
        // 控制左手
        setHandControlFromMapping(left_hand_id_, mapping_data, FingerIndex::FINGER_THUMB);
        sendHandControl(left_hand_id_, mapping_data);
        
        RCLCPP_DEBUG(this->get_logger(), 
                    "收到左手映射数据: [%.3f, %.3f, %.3f]...",
                    mapping_data[0], mapping_data[1], mapping_data[2]);
    }
    
    /**
     * @brief 中点死区处理函数
     * @details 对于Yaw轴（除了大拇指），实现中点死区处理：
     *   - 当值在0.35-0.65之间时，输出0.5（死区中心）
     *   - 当值在0-0.35之间时，映射到0-0.5
     *   - 当值在0.65-1.0之间时，映射到0.5-1.0
     * @param input_val 输入值（0.0-1.0）
     * @return 处理后的值（0.0-1.0）
     */
    float applyYawDeadzone(float input_val)
    {
        const float deadzone_low = 0.35f;   /**< 死区下限 */
        const float deadzone_high = 0.65f;  /**< 死区上限 */
        const float deadzone_center = 0.5f; /**< 死区中心值 */
        
        // 限制输入值在有效范围内
        if (input_val < 0.0f) input_val = 0.0f;
        if (input_val > 1.0f) input_val = 1.0f;
        
        // 死区处理
        if (input_val >= deadzone_low && input_val <= deadzone_high) {
            // 在死区内，输出中心值
            return deadzone_center;
        } else if (input_val < deadzone_low) {
            // 在死区下方，映射到0-0.5
            return (input_val / deadzone_low) * deadzone_center;
        } else {
            // 在死区上方，映射到0.5-1.0
            float upper_range = 1.0f - deadzone_high;
            float mapped_val = (input_val - deadzone_high) / upper_range;
            return deadzone_center + mapped_val * (1.0f - deadzone_center);
        }
    }
    
    /**
     * @brief 从映射数据设置手部控制
     * @param device_id 设备ID
     * @param mapping_data 映射数据（15个值）
     * @param start_finger 起始手指索引
     */
    void setHandControlFromMapping(uint32_t device_id, 
                                    const std::vector<float>& mapping_data,
                                    FingerIndex /* start_finger */)
    {
        // 映射数据索引到手指和关节
        // 每个手指有3个关节：Yaw(0), Pitch(1), Tip(2)
        for (size_t finger_idx = 0; finger_idx < HAND_NUM_FINGERS; finger_idx++) {
            size_t base_idx = finger_idx * 3;
            if (base_idx + 2 >= mapping_data.size()) {
                break;
            }
            
            float yaw_val = mapping_data[base_idx];
            float pitch = mapping_data[base_idx + 1];
            float tip = mapping_data[base_idx + 2];
            
            // 处理Yaw轴（传感器索引：base_idx）
            // 使用映射数据直接作为yaw_val（原代码中的map_sensor_with_anchor_hand功能）
            FingerIndex finger_index = static_cast<FingerIndex>(finger_idx);
            
            if (finger_index == FingerIndex::FINGER_THUMB) {
                // 拇指Yaw轴：仅右手反向；左手不反向
                float out_yaw = (device_id == HAND_ID_RIGHT) ? (1.0f - yaw_val) : yaw_val;
                float thumb_yaw_scale = 1.0f;
                float thumb_yaw_offset = 0.0f;
                float thumb_roll_scale = 1.0f;
                float thumb_roll_offset = 0.0f;
                hand_control_.setFingerYawEx(device_id, FingerIndex::FINGER_THUMB, 
                                            out_yaw * thumb_yaw_scale + thumb_yaw_offset);
                hand_control_.setFingerRollEx(device_id, FingerIndex::FINGER_THUMB, 
                                            out_yaw * thumb_roll_scale + thumb_roll_offset);
            } else {
                // 其他手指Yaw轴：死区处理；仅右手反向
                yaw_val = applyYawDeadzone(yaw_val);
/*                 if (device_id == HAND_ID_RIGHT) {
                    yaw_val = 1.0f - yaw_val;
                } */
                hand_control_.setFingerYawEx(device_id, finger_index, yaw_val);
            }
            
            // 设置Pitch和Tip（保持不变）
            hand_control_.setFingerPitchEx(device_id, finger_index, pitch);
            hand_control_.setFingerTipEx(device_id, finger_index, tip);
        }
    }
    
    void sendHandControl(uint32_t device_id, const std::vector<float>&)
    {
        /**
         * @brief 发送手部控制数据
         * 
         * 将当前的手部控制数据转换为CAN帧并发送
         * 同时发布到ROS2话题用于数据采集
         */
        if (!can_sender_) {
            return;
        }
        
        // 转换并发送控制数据
        HandControlRequest request{};
        hand_control_.convertUserControlToSendData(device_id, request);
        
        // 发布控制命令到ROS2话题（用于数据采集）
        publishControlCommand(device_id, request);
        
        // 不应用默认值，直接发送实际数据
        // 因为数据已经在setHandControlFromMapping中设置好了
        can_sender_->send(hand_control_.getProtocol(), device_id, request);
    }
    
    /**
     * @brief 发布控制命令到ROS2话题
     * @param device_id 设备ID
     * @param request 控制请求数据
     */
    void publishControlCommand(uint32_t device_id, const HandControlRequest& request)
    {
        /**
         * @brief 发布控制命令到ROS2话题用于数据采集
         * 
         * 将控制命令转换为sensor_msgs/JointState格式发布
         * 数据范围为0-255（与SDK保持一致）
         */
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = this->get_clock()->now();
        
        // 根据协议类型确定关节数量和数据格式
        HandProtocol protocol = hand_control_.getProtocol();
        std::vector<uint8_t> positions;
        std::vector<std::string> joint_names;
        
        // 根据协议提取位置数据
        switch (protocol) {
            case HandProtocol::HAND_PROTO_L10: {
                // L10: 10个关节
                // 第一帧: 拇指Pitch, 拇指Yaw, 食指Pitch, 中指Pitch, 无名指Pitch, 小指Pitch (6个)
                // 第二帧: 食指Yaw, 无名指Yaw, 小指Yaw, 拇指Roll (4个)
                positions.push_back(request.fingers[0].pitch_angle);  // 拇指Pitch
                positions.push_back(request.fingers[0].yaw_angle);      // 拇指Yaw
                positions.push_back(request.fingers[1].pitch_angle);    // 食指Pitch
                positions.push_back(request.fingers[2].pitch_angle);    // 中指Pitch
                positions.push_back(request.fingers[3].pitch_angle);   // 无名指Pitch
                positions.push_back(request.fingers[4].pitch_angle);   // 小指Pitch
                positions.push_back(request.fingers[1].yaw_angle);      // 食指Yaw
                positions.push_back(request.fingers[3].yaw_angle);      // 无名指Yaw
                positions.push_back(request.fingers[4].yaw_angle);      // 小指Yaw
                positions.push_back(request.fingers[0].roll_angle);     // 拇指Roll
                
                for (size_t i = 0; i < 10; i++) {
                    joint_names.push_back("joint" + std::to_string(i + 1));
                }
                break;
            }
            case HandProtocol::HAND_PROTO_L20:
            case HandProtocol::HAND_PROTO_L21: {
                // L20/L21: 20个关节
                // Pitch(5) + Yaw(5) + Roll(5) + Tip(5) = 20
                for (size_t i = 0; i < 5; i++) {
                    positions.push_back(request.fingers[i].pitch_angle);
                }
                for (size_t i = 0; i < 5; i++) {
                    positions.push_back(request.fingers[i].yaw_angle);
                }
                for (size_t i = 0; i < 5; i++) {
                    positions.push_back(request.fingers[i].roll_angle);
                }
                for (size_t i = 0; i < 5; i++) {
                    positions.push_back(request.fingers[i].tip_angle);
                }
                
                for (size_t i = 0; i < 20; i++) {
                    joint_names.push_back("joint" + std::to_string(i + 1));
                }
                break;
            }
            case HandProtocol::HAND_PROTO_L7: {
                // L7: 7个关节
                // 拇指Pitch, 食指Pitch, 中指Pitch, 无名指Pitch, 小指Pitch, 拇指Yaw, 拇指Roll
                positions.push_back(request.fingers[0].pitch_angle);
                positions.push_back(request.fingers[1].pitch_angle);
                positions.push_back(request.fingers[2].pitch_angle);
                positions.push_back(request.fingers[3].pitch_angle);
                positions.push_back(request.fingers[4].pitch_angle);
                positions.push_back(request.fingers[0].yaw_angle);
                positions.push_back(request.fingers[0].roll_angle);
                
                for (size_t i = 0; i < 7; i++) {
                    joint_names.push_back("joint" + std::to_string(i + 1));
                }
                break;
            }
            case HandProtocol::HAND_PROTO_L25: {
                // L25: 25个关节
                // Roll(5) + Yaw(5) + Root1(5) + Root2(5) + Tip(5) = 25
                for (size_t i = 0; i < 5; i++) {
                    positions.push_back(request.fingers[i].roll_angle);
                }
                for (size_t i = 0; i < 5; i++) {
                    positions.push_back(request.fingers[i].yaw_angle);
                }
                for (size_t i = 0; i < 5; i++) {
                    positions.push_back(request.fingers[i].pitch_angle);  // Root1使用pitch
                }
                for (size_t i = 0; i < 5; i++) {
                    positions.push_back(request.fingers[i].pitch_angle);  // Root2使用pitch（简化）
                }
                for (size_t i = 0; i < 5; i++) {
                    positions.push_back(request.fingers[i].tip_angle);
                }
                
                for (size_t i = 0; i < 25; i++) {
                    joint_names.push_back("joint" + std::to_string(i + 1));
                }
                break;
            }
            case HandProtocol::HAND_PROTO_O6: {
                // O6: 6个关节
                // 拇指Pitch, 食指Pitch, 中指Pitch, 无名指Pitch, 小指Pitch, 拇指Yaw
                positions.push_back(request.fingers[0].pitch_angle);
                positions.push_back(request.fingers[1].pitch_angle);
                positions.push_back(request.fingers[2].pitch_angle);
                positions.push_back(request.fingers[3].pitch_angle);
                positions.push_back(request.fingers[4].pitch_angle);
                positions.push_back(request.fingers[0].yaw_angle);
                
                for (size_t i = 0; i < 6; i++) {
                    joint_names.push_back("joint" + std::to_string(i + 1));
                }
                break;
            }
            default:
                // 默认使用L10格式
                for (size_t i = 0; i < 5; i++) {
                    positions.push_back(request.fingers[i].pitch_angle);
                    positions.push_back(request.fingers[i].yaw_angle);
                }
                for (size_t i = 0; i < 10; i++) {
                    joint_names.push_back("joint" + std::to_string(i + 1));
                }
                break;
        }
        
        // 设置消息内容
        msg.name = joint_names;
        msg.position.resize(positions.size());
        for (size_t i = 0; i < positions.size(); i++) {
            msg.position[i] = static_cast<double>(positions[i]);
        }
        msg.velocity.resize(positions.size(), 0.0);
        msg.effort.resize(positions.size(), 0.0);
        
        // 根据设备ID发布到对应话题
        if (device_id == right_hand_id_ && cmd_pub_right_) {
            msg.header.frame_id = "right_hand";
            cmd_pub_right_->publish(msg);
        } else if (device_id == left_hand_id_ && cmd_pub_left_) {
            msg.header.frame_id = "left_hand";
            cmd_pub_left_->publish(msg);
        }
    }
    
    /**
     * @brief 发布反馈数据
     */
    void publishFeedback()
    {
        /**
         * @brief 定期发布灵巧手反馈数据
         */
        HandProtocol current_protocol = hand_control_.getProtocol();
        
        // 发布右手反馈数据
        if (can_receiver_right_ && state_pub_right_) {
            auto positions = can_receiver_right_->getJointPositions(current_protocol);
            auto velocities = can_receiver_right_->getJointVelocities(current_protocol);
            
            if (!positions.empty()) {
                sensor_msgs::msg::JointState msg;
                msg.header.stamp = this->get_clock()->now();
                msg.header.frame_id = "right_hand";
                
                // 设置关节名称
                for (size_t i = 0; i < positions.size(); i++) {
                    msg.name.push_back("right_joint" + std::to_string(i + 1));
                }
                
                // 设置位置数据（转换为弧度）
                msg.position.resize(positions.size());
                for (size_t i = 0; i < positions.size(); i++) {
                    // 将0-255的值转换为弧度（假设范围是0-2π）
                    msg.position[i] = static_cast<double>(positions[i]) * 2.0 * M_PI / 255.0;
                }
                
                // 设置速度数据
                if (!velocities.empty() && velocities.size() == positions.size()) {
                    msg.velocity.resize(velocities.size());
                    for (size_t i = 0; i < velocities.size(); i++) {
                        msg.velocity[i] = static_cast<double>(velocities[i]);
                    }
                } else {
                    msg.velocity.resize(positions.size(), 0.0);
                }
                
                // 设置effort（暂时为0）
                msg.effort.resize(positions.size(), 0.0);
                
                state_pub_right_->publish(msg);
            }
        }
        
        // 发布左手反馈数据
        if (can_receiver_left_ && state_pub_left_) {
            auto positions = can_receiver_left_->getJointPositions(current_protocol);
            auto velocities = can_receiver_left_->getJointVelocities(current_protocol);
            
            if (!positions.empty()) {
                sensor_msgs::msg::JointState msg;
                msg.header.stamp = this->get_clock()->now();
                msg.header.frame_id = "left_hand";
                
                // 设置关节名称
                for (size_t i = 0; i < positions.size(); i++) {
                    msg.name.push_back("left_joint" + std::to_string(i + 1));
                }
                
                // 设置位置数据（转换为弧度）
                msg.position.resize(positions.size());
                for (size_t i = 0; i < positions.size(); i++) {
                    // 将0-255的值转换为弧度（假设范围是0-2π）
                    msg.position[i] = static_cast<double>(positions[i]) * 2.0 * M_PI / 255.0;
                }
                
                // 设置速度数据
                if (!velocities.empty() && velocities.size() == positions.size()) {
                    msg.velocity.resize(velocities.size());
                    for (size_t i = 0; i < velocities.size(); i++) {
                        msg.velocity[i] = static_cast<double>(velocities[i]);
                    }
                } else {
                    msg.velocity.resize(positions.size(), 0.0);
                }
                
                // 设置effort（暂时为0）
                msg.effort.resize(positions.size(), 0.0);
                
                state_pub_left_->publish(msg);
            }
        }
        
        /**
         * @note 带返回的协议不需要请求状态
         * 每次发送控制帧后，设备会自动返回状态数据
         * 接收器会在后台线程中自动接收并处理返回的状态
         */
    }
    
    HandControl hand_control_;                    /* 手部控制对象 */
    std::unique_ptr<CanSender> can_sender_;       /* CAN发送器 */
    std::unique_ptr<CanReceiver> can_receiver_right_;  /* 右手CAN接收器 */
    std::unique_ptr<CanReceiver> can_receiver_left_;   /* 左手CAN接收器 */
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr mapping_sub_right_;  /* 右手映射数据订阅者 */
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr mapping_sub_left_;   /* 左手映射数据订阅者 */
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_right_;  /* 右手状态发布者 */
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_left_;   /* 左手状态发布者 */
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_pub_right_;     /* 右手控制命令发布者 */
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_pub_left_;      /* 左手控制命令发布者 */
    rclcpp::TimerBase::SharedPtr feedback_timer_;  /* 反馈数据发布定时器 */
    uint32_t right_hand_id_;                       /* 右手设备ID */
    uint32_t left_hand_id_;                        /* 左手设备ID */
};

}  // namespace linkerhand_cl

/**
 * @brief 主函数
 */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<linkerhand_cl::HandControlNode>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}

