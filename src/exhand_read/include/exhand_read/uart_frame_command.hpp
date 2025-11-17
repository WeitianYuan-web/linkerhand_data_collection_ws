/**
 * @file uart_frame_command.hpp
 * @brief 手部控制器串口通信 SDK (C++版本)
 * 
 * 提供与手部控制器进行串口通信的接口，支持命令发送和数据接收。
 */

#ifndef EXHAND_READ_UART_FRAME_COMMAND_HPP_
#define EXHAND_READ_UART_FRAME_COMMAND_HPP_

#include <string>
#include <vector>
#include <memory>
#include <cstdint>
#include <optional>
#include <mutex>

namespace exhand_read
{

/**
 * @brief 串口数据帧命令客户端类
 */
class UartFrameCommand
{
public:
    /**
     * @brief 帧格式常量
     */
    static constexpr uint8_t FRAME_HEADER = 0xAA;
    static constexpr uint8_t FRAME_TAIL = 0x55;
    static constexpr uint8_t MIN_FRAME_LEN = 5;

    /**
     * @brief 命令类型
     */
    static constexpr uint8_t CMD_ENABLE = 0x01;
    static constexpr uint8_t CMD_DISABLE = 0x02;
    static constexpr uint8_t CMD_QUICK_START = 0x03;
    static constexpr uint8_t CMD_QUICK_FINISH = 0x04;
    static constexpr uint8_t CMD_ANCHOR_START = 0x05;
    static constexpr uint8_t CMD_RECORD = 0x06;
    static constexpr uint8_t CMD_APPLY = 0x07;
    static constexpr uint8_t CMD_SAVE = 0x08;
    static constexpr uint8_t CMD_LOAD = 0x09;
    static constexpr uint8_t CMD_CLEAR = 0x0A;
    static constexpr uint8_t CMD_STATUS = 0x0B;
    static constexpr uint8_t CMD_RESET = 0x0C;
    static constexpr uint8_t CMD_CAN_ENABLE = 0x0D;
    static constexpr uint8_t CMD_CAN_DISABLE = 0x0E;
    static constexpr uint8_t CMD_SENSOR_ENABLE = 0x0F;
    static constexpr uint8_t CMD_SENSOR_DISABLE = 0x10;
    static constexpr uint8_t CMD_MAPPING_ENABLE = 0x11;
    static constexpr uint8_t CMD_MAPPING_DISABLE = 0x12;
    static constexpr uint8_t CMD_SET_PROTOCOL = 0x13;

    /**
     * @brief 数据通知类型
     */
    static constexpr uint8_t CMD_SENSOR_DATA = 0x20;
    static constexpr uint8_t CMD_MAPPING_DATA = 0x21;

    /**
     * @brief 协议类型
     */
    static constexpr uint8_t PROTOCOL_L20 = 0;
    static constexpr uint8_t PROTOCOL_L10 = 1;
    static constexpr uint8_t PROTOCOL_L21 = 2;

    /**
     * @brief 结果码
     */
    static constexpr uint8_t RESULT_SUCCESS = 0x00;
    static constexpr uint8_t RESULT_FAIL = 0x01;
    static constexpr uint8_t RESULT_UNKNOWN_CMD = 0xFD;
    static constexpr uint8_t RESULT_NOT_ENABLED = 0xFE;
    static constexpr uint8_t RESULT_CHECKSUM_ERROR = 0xFF;

    /**
     * @brief 响应数据结构
     */
    struct Response
    {
        uint8_t cmd_type;
        uint8_t result;
        std::vector<uint8_t> data;
    };

    /**
     * @brief 传感器数据结构
     */
    struct SensorData
    {
        uint8_t hand;  // 0=右手, 1=左手
        std::vector<uint16_t> sensor_data;  // 15个传感器数据
    };

    /**
     * @brief 映射数据结构
     */
    struct MappingData
    {
        uint8_t hand;  // 0=右手, 1=左手
        std::vector<float> mapping_data;  // 15个映射数据
    };

    /**
     * @brief 任意数据帧结构
     */
    struct AnyDataFrame
    {
        enum class FrameType
        {
            SENSOR,
            MAPPING
        };
        FrameType frame_type;
        uint8_t hand;
        std::vector<uint16_t> sensor_data;  // 仅当frame_type为SENSOR时有效
        std::vector<float> mapping_data;     // 仅当frame_type为MAPPING时有效
    };

    /**
     * @brief 状态数据结构
     */
    struct Status
    {
        uint8_t quick_state;
        uint8_t anchor_state;
        uint8_t frame_enabled;
        uint8_t sensor_print;
        uint8_t can_enabled;
        uint8_t sensor_send;
    };

    /**
     * @brief 构造函数
     * @param port 串口名称（如 "/dev/ttyUSB0"）
     * @param baudrate 波特率（默认115200）
     * @param timeout 超时时间（秒）
     */
    UartFrameCommand(const std::string& port, uint32_t baudrate = 115200, double timeout = 1.0);

    /**
     * @brief 析构函数
     */
    ~UartFrameCommand();

    /**
     * @brief 计算校验和（补码累加和）
     */
    uint8_t calcChecksum(uint8_t cmd_type, uint8_t data_len, const std::vector<uint8_t>& data);

    /**
     * @brief 发送命令帧并接收响应
     */
    std::optional<Response> sendCommand(uint8_t cmd_type, 
                                        const std::vector<uint8_t>& data = {},
                                        bool wait_response = true);

    /**
     * @brief 接收响应帧
     */
    std::optional<Response> receiveResponse(double timeout = -1.0);

    /**
     * @brief 接收任意数据帧（传感器数据或映射数据）
     * 这是一个统一的接收函数，不会跳过任何类型的帧
     */
    std::optional<AnyDataFrame> receiveAnyDataFrame(double timeout = -1.0);

    /**
     * @brief 启用数据帧模式
     */
    std::optional<Response> enable();

    /**
     * @brief 禁用数据帧模式
     */
    std::optional<Response> disable();

    /**
     * @brief 启用传感器数据推送
     */
    std::optional<Response> sensorEnable();

    /**
     * @brief 禁用传感器数据推送
     */
    std::optional<Response> sensorDisable();

    /**
     * @brief 启用映射数据推送
     */
    std::optional<Response> mappingEnable();

    /**
     * @brief 禁用映射数据推送
     */
    std::optional<Response> mappingDisable();

    /**
     * @brief 设置协议类型
     */
    std::optional<Response> setProtocol(uint8_t protocol_id);

    /**
     * @brief 启用CAN总线控制
     */
    std::optional<Response> canEnable();

    /**
     * @brief 禁用CAN总线控制
     */
    std::optional<Response> canDisable();

    /**
     * @brief 查询系统状态
     */
    std::optional<Status> status();

    /**
     * @brief 关闭串口连接
     */
    void close();

    /**
     * @brief 检查串口是否打开
     */
    bool isOpen() const;

    /**
     * @brief 获取串口等待读取的字节数
     */
    size_t bytesAvailable() const;

private:
    class SerialPortImpl;  // 前向声明
    std::unique_ptr<SerialPortImpl> serial_port_;
    double timeout_;
    mutable std::mutex serial_mutex_;
};

}  // namespace exhand_read

#endif  // EXHAND_READ_UART_FRAME_COMMAND_HPP_

