/**
 * @file uart_frame_command.cpp
 * @brief 手部控制器串口通信 SDK 实现 (C++版本)
 */

#include "exhand_read/uart_frame_command.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <cstring>
#include <algorithm>
#include <chrono>
#include <thread>
#include <iostream>
#include <cmath>
#include <stdexcept>

namespace exhand_read
{

/**
 * @brief 串口封装类（使用termios）
 */
class UartFrameCommand::SerialPortImpl
{
public:
    SerialPortImpl();
    ~SerialPortImpl();
    
    bool open(const std::string& port, uint32_t baudrate);
    void close();
    bool isOpen() const;
    ssize_t read(uint8_t* buffer, size_t size);
    ssize_t write(const uint8_t* buffer, size_t size);
    size_t bytesAvailable() const;
    bool waitForReadyRead(int timeout_ms);
    void flush();
    int getFd() const;
    void setRTS(bool state);
    void setDTR(bool state);

private:
    int fd_;
};

UartFrameCommand::SerialPortImpl::SerialPortImpl() : fd_(-1)
{
}

UartFrameCommand::SerialPortImpl::~SerialPortImpl()
{
    close();
}

bool UartFrameCommand::SerialPortImpl::open(const std::string& port, uint32_t baudrate)
{
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0)
    {
        return false;
    }

    struct termios tty;
    if (tcgetattr(fd_, &tty) != 0)
    {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    // 设置波特率
    speed_t speed = B115200;
    switch (baudrate)
    {
        case 9600: speed = B9600; break;
        case 19200: speed = B19200; break;
        case 38400: speed = B38400; break;
        case 57600: speed = B57600; break;
        case 115200: speed = B115200; break;
        case 230400: speed = B230400; break;
        case 460800: speed = B460800; break;
        case 500000: speed = B500000; break;
        case 576000: speed = B576000; break;
        case 921600: speed = B921600; break;
        case 1000000: speed = B1000000; break;
        case 1152000: speed = B1152000; break;
        case 1500000: speed = B1500000; break;
        case 2000000: speed = B2000000; break;
        case 2500000: speed = B2500000; break;
        case 3000000: speed = B3000000; break;
        case 3500000: speed = B3500000; break;
        case 4000000: speed = B4000000; break;
        default:
            // 对于不支持的波特率，尝试使用B38400并设置自定义波特率
            speed = B38400;
            break;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    // 8N1配置
    tty.c_cflag &= ~PARENB;  // 无奇偶校验
    tty.c_cflag &= ~CSTOPB;  // 1个停止位
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;      // 8位数据位
    tty.c_cflag &= ~CRTSCTS; // 无硬件流控
    tty.c_cflag |= CREAD | CLOCAL; // 启用接收器，忽略调制解调器控制线

    // 输入模式
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // 关闭软件流控
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    // 输出模式
    tty.c_oflag &= ~OPOST;

    // 本地模式
    tty.c_lflag = 0; // 非规范模式

    // 控制字符
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0)
    {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    // 清空缓冲区
    tcflush(fd_, TCIOFLUSH);

    return true;
}

void UartFrameCommand::SerialPortImpl::close()
{
    if (fd_ >= 0)
    {
        ::close(fd_);
        fd_ = -1;
    }
}

bool UartFrameCommand::SerialPortImpl::isOpen() const
{
    return fd_ >= 0;
}

ssize_t UartFrameCommand::SerialPortImpl::read(uint8_t* buffer, size_t size)
{
    if (fd_ < 0) return -1;
    return ::read(fd_, buffer, size);
}

ssize_t UartFrameCommand::SerialPortImpl::write(const uint8_t* buffer, size_t size)
{
    if (fd_ < 0) return -1;
    return ::write(fd_, buffer, size);
}

size_t UartFrameCommand::SerialPortImpl::bytesAvailable() const
{
    if (fd_ < 0) return 0;
    int bytes = 0;
    if (ioctl(fd_, FIONREAD, &bytes) == 0)
    {
        return static_cast<size_t>(bytes);
    }
    return 0;
}

bool UartFrameCommand::SerialPortImpl::waitForReadyRead(int timeout_ms)
{
    if (fd_ < 0) return false;

    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(fd_, &readfds);

    struct timeval timeout;
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;

    int result = select(fd_ + 1, &readfds, nullptr, nullptr, &timeout);
    return result > 0 && FD_ISSET(fd_, &readfds);
}

void UartFrameCommand::SerialPortImpl::flush()
{
    if (fd_ >= 0)
    {
        tcflush(fd_, TCIOFLUSH);
    }
}

int UartFrameCommand::SerialPortImpl::getFd() const
{
    return fd_;
}

void UartFrameCommand::SerialPortImpl::setRTS(bool state)
{
    if (fd_ < 0) return;
    
    int status;
    if (ioctl(fd_, TIOCMGET, &status) == 0)
    {
        if (state)
        {
            status |= TIOCM_RTS;
        }
        else
        {
            status &= ~TIOCM_RTS;
        }
        ioctl(fd_, TIOCMSET, &status);
    }
}

void UartFrameCommand::SerialPortImpl::setDTR(bool state)
{
    if (fd_ < 0) return;
    
    int status;
    if (ioctl(fd_, TIOCMGET, &status) == 0)
    {
        if (state)
        {
            status |= TIOCM_DTR;
        }
        else
        {
            status &= ~TIOCM_DTR;
        }
        ioctl(fd_, TIOCMSET, &status);
    }
}

UartFrameCommand::UartFrameCommand(const std::string& port, uint32_t baudrate, double timeout)
    : timeout_(timeout)
{
    serial_port_ = std::make_unique<UartFrameCommand::SerialPortImpl>();

    if (!serial_port_->open(port, baudrate))
    {
        throw std::runtime_error("无法打开串口: " + port);
    }

    // 设置 DTR 和 RTS 为 False，可以帮助重置设备状态
    serial_port_->setDTR(false);
    serial_port_->setRTS(false);
    
    // 等待串口稳定
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    
    // 重置 DTR 和 RTS（触发设备复位）
    serial_port_->setDTR(true);
    serial_port_->setRTS(true);
    
    // 等待设备复位完成（复位后设备应该处于初始状态）
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // 清空缓冲区
    serial_port_->flush();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 持续清空缓冲区直到稳定
    auto start = std::chrono::steady_clock::now();
    int stable_count = 0;
    while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(1500))
    {
        size_t waiting = serial_port_->bytesAvailable();
        if (waiting > 0)
        {
            std::vector<uint8_t> buffer(waiting);
            serial_port_->read(buffer.data(), waiting);
            stable_count = 0;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        else
        {
            stable_count++;
            if (stable_count >= 5)
            {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    serial_port_->flush();
}

UartFrameCommand::~UartFrameCommand()
{
    close();
}

uint8_t UartFrameCommand::calcChecksum(uint8_t cmd_type, uint8_t data_len, const std::vector<uint8_t>& data)
{
    int sum = cmd_type + data_len;
    for (uint8_t byte : data)
    {
        sum += byte;
    }
    return static_cast<uint8_t>((~sum + 1) & 0xFF);
}

std::optional<UartFrameCommand::Response> UartFrameCommand::sendCommand(
    uint8_t cmd_type, const std::vector<uint8_t>& data, bool wait_response)
{
    std::lock_guard<std::mutex> lock(serial_mutex_);

    // 清空输入缓冲区，避免读取到旧数据
    serial_port_->flush();

    std::vector<uint8_t> frame;
    frame.push_back(FRAME_HEADER);
    frame.push_back(cmd_type);
    frame.push_back(static_cast<uint8_t>(data.size()));
    frame.insert(frame.end(), data.begin(), data.end());

    uint8_t checksum = calcChecksum(cmd_type, static_cast<uint8_t>(data.size()), data);
    frame.push_back(checksum);
    frame.push_back(FRAME_TAIL);

    if (serial_port_->write(frame.data(), frame.size()) != static_cast<ssize_t>(frame.size()))
    {
        return std::nullopt;
    }

    if (wait_response)
    {
        // 使用较短的超时时间，避免长时间阻塞
        return receiveResponse(0.5);  // 500ms超时
    }
    return std::nullopt;
}

std::optional<UartFrameCommand::Response> UartFrameCommand::receiveResponse(double timeout)
{
    // 注意：这个函数不应该持有锁，因为sendCommand已经持有锁了
    // 如果wait_response为false，sendCommand不会调用这个函数
    // 但是为了线程安全，如果单独调用这个函数，需要持有锁
    // 这里我们假设调用者已经持有锁（从sendCommand调用时）
    
    double actual_timeout = (timeout < 0) ? timeout_ : timeout;
    auto start_time = std::chrono::steady_clock::now();

    std::vector<uint8_t> buffer;

    // 查找帧头，使用更短的检查间隔以便快速响应
    while (true)
    {
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (std::chrono::duration<double>(elapsed).count() > actual_timeout)
        {
            return std::nullopt;
        }

        uint8_t byte;
        ssize_t n = serial_port_->read(&byte, 1);
        if (n == 1)
        {
            if (byte == FRAME_HEADER)
            {
                buffer.push_back(byte);
                break;
            }
        }
        else if (n < 0)
        {
            // 读取错误，立即返回
            return std::nullopt;
        }
        else
        {
            // 没有数据，短暂等待
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    // 读取命令类型和数据长度
    while (buffer.size() < 3)
    {
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (std::chrono::duration<double>(elapsed).count() > actual_timeout)
        {
            return std::nullopt;
        }

        uint8_t byte;
        ssize_t n = serial_port_->read(&byte, 1);
        if (n == 1)
        {
            buffer.push_back(byte);
        }
        else if (n < 0)
        {
            return std::nullopt;
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    uint8_t cmd_type = buffer[1];
    uint8_t data_len = buffer[2];
    size_t expected_len = 5 + data_len;

    // 读取完整帧
    while (buffer.size() < expected_len)
    {
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (std::chrono::duration<double>(elapsed).count() > actual_timeout)
        {
            return std::nullopt;
        }

        uint8_t byte;
        ssize_t n = serial_port_->read(&byte, 1);
        if (n == 1)
        {
            buffer.push_back(byte);
        }
        else if (n < 0)
        {
            return std::nullopt;
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    // 验证帧尾
    if (buffer.back() != FRAME_TAIL)
    {
        return std::nullopt;
    }

    // 验证校验和
    std::vector<uint8_t> frame_data(buffer.begin() + 3, buffer.begin() + 3 + data_len);
    uint8_t checksum = buffer[3 + data_len];
    uint8_t calc_checksum = calcChecksum(cmd_type, data_len, frame_data);
    if (checksum != calc_checksum)
    {
        return std::nullopt;
    }

    Response response;
    response.cmd_type = cmd_type;
    response.result = (frame_data.size() > 0) ? frame_data[0] : 0x00;
    if (frame_data.size() > 1)
    {
        response.data.assign(frame_data.begin() + 1, frame_data.end());
    }
    return response;
}

std::optional<UartFrameCommand::AnyDataFrame> UartFrameCommand::receiveAnyDataFrame(double timeout)
{
    std::lock_guard<std::mutex> lock(serial_mutex_);

    double actual_timeout = (timeout < 0) ? timeout_ : timeout;
    auto start_time = std::chrono::steady_clock::now();

    constexpr int max_attempts = 3;  // 减少尝试次数，因为超时时间很短

    for (int attempt = 0; attempt < max_attempts; ++attempt)
    {
        std::vector<uint8_t> buffer;

        // 查找帧头，使用更短的检查间隔
        while (true)
        {
            auto elapsed = std::chrono::steady_clock::now() - start_time;
            if (std::chrono::duration<double>(elapsed).count() > actual_timeout)
            {
                return std::nullopt;
            }

            uint8_t byte;
            ssize_t n = serial_port_->read(&byte, 1);
            if (n == 1)
            {
                if (byte == FRAME_HEADER)
                {
                    buffer.push_back(byte);
                    break;
                }
            }
            else if (n < 0)
            {
                // 读取错误，立即返回
                return std::nullopt;
            }
            else
            {
                // 没有数据，短暂等待
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }

        // 读取命令类型和数据长度
        uint8_t header_bytes[2];
        size_t header_read = 0;
        while (header_read < 2)
        {
            auto elapsed = std::chrono::steady_clock::now() - start_time;
            if (std::chrono::duration<double>(elapsed).count() > actual_timeout)
            {
                break;
            }

            ssize_t n = serial_port_->read(header_bytes + header_read, 2 - header_read);
            if (n > 0)
            {
                header_read += n;
            }
            else if (n < 0)
            {
                // 读取错误，退出
                break;
            }
            else
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }

        if (header_read < 2)
        {
            continue;
        }

        buffer.push_back(header_bytes[0]);
        buffer.push_back(header_bytes[1]);

        uint8_t cmd_type = buffer[1];
        uint8_t data_len = buffer[2];

        size_t expected_len = 0;
        if (cmd_type == CMD_SENSOR_DATA)
        {
            expected_len = 36;
            if (data_len != 31)
            {
                continue;
            }
        }
        else if (cmd_type == CMD_MAPPING_DATA)
        {
            expected_len = 66;
            if (data_len != 0x3D)
            {
                continue;
            }
        }
        else
        {
            // 跳过未知类型的帧
            size_t skip_bytes = data_len + 2;
            std::vector<uint8_t> skip_buffer(skip_bytes);
            serial_port_->read(skip_buffer.data(), skip_bytes);
            continue;
        }

        // 读取剩余字节
        size_t remaining_len = expected_len - 3;
        std::vector<uint8_t> remaining(remaining_len);
        size_t remaining_read = 0;
        while (remaining_read < remaining_len)
        {
            auto elapsed = std::chrono::steady_clock::now() - start_time;
            if (std::chrono::duration<double>(elapsed).count() > actual_timeout)
            {
                break;
            }

            ssize_t n = serial_port_->read(remaining.data() + remaining_read, remaining_len - remaining_read);
            if (n > 0)
            {
                remaining_read += n;
            }
            else if (n < 0)
            {
                // 读取错误，退出
                break;
            }
            else
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }

        if (remaining_read < remaining_len)
        {
            continue;
        }

        buffer.insert(buffer.end(), remaining.begin(), remaining.end());

        // 验证帧长度
        if (buffer.size() != expected_len)
        {
            continue;
        }

        // 验证帧尾
        if (buffer.back() != FRAME_TAIL)
        {
            continue;
        }

        // 验证校验和
        std::vector<uint8_t> frame_data(buffer.begin() + 3, buffer.begin() + 3 + data_len);
        uint8_t checksum = buffer[3 + data_len];
        uint8_t calc_checksum = calcChecksum(cmd_type, data_len, frame_data);
        if (checksum != calc_checksum)
        {
            continue;
        }

        // 解析数据
        uint8_t hand = buffer[3];
        AnyDataFrame frame;
        frame.hand = hand;

        if (cmd_type == CMD_SENSOR_DATA)
        {
            frame.frame_type = AnyDataFrame::FrameType::SENSOR;
            frame.sensor_data.resize(15);
            for (int i = 0; i < 15; ++i)
            {
                size_t idx = 4 + i * 2;
                if (idx + 1 < buffer.size())
                {
                    frame.sensor_data[i] = buffer[idx] | (buffer[idx + 1] << 8);
                }
            }
            return frame;
        }
        else if (cmd_type == CMD_MAPPING_DATA)
        {
            frame.frame_type = AnyDataFrame::FrameType::MAPPING;
            frame.mapping_data.resize(15);
            for (int i = 0; i < 15; ++i)
            {
                size_t idx = 4 + i * 4;
                if (idx + 4 <= buffer.size())
                {
                    uint32_t float_bits = buffer[idx] | 
                                         (buffer[idx + 1] << 8) |
                                         (buffer[idx + 2] << 16) |
                                         (buffer[idx + 3] << 24);
                    frame.mapping_data[i] = *reinterpret_cast<float*>(&float_bits);
                }
            }
            return frame;
        }
    }

    return std::nullopt;
}

std::optional<UartFrameCommand::Response> UartFrameCommand::enable()
{
    std::lock_guard<std::mutex> lock(serial_mutex_);
    std::string command = "frame_enable\n";
    if (serial_port_->write(reinterpret_cast<const uint8_t*>(command.data()), command.size()) == static_cast<ssize_t>(command.size()))
    {
        Response resp;
        resp.cmd_type = 0;
        resp.result = RESULT_SUCCESS;
        return resp;
    }
    return std::nullopt;
}

std::optional<UartFrameCommand::Response> UartFrameCommand::disable()
{
    return sendCommand(CMD_DISABLE);
}

std::optional<UartFrameCommand::Response> UartFrameCommand::sensorEnable()
{
    return sendCommand(CMD_SENSOR_ENABLE);
}

std::optional<UartFrameCommand::Response> UartFrameCommand::sensorDisable()
{
    return sendCommand(CMD_SENSOR_DISABLE);
}

std::optional<UartFrameCommand::Response> UartFrameCommand::mappingEnable()
{
    return sendCommand(CMD_MAPPING_ENABLE);
}

std::optional<UartFrameCommand::Response> UartFrameCommand::mappingDisable()
{
    return sendCommand(CMD_MAPPING_DISABLE);
}

std::optional<UartFrameCommand::Response> UartFrameCommand::setProtocol(uint8_t protocol_id)
{
    if (protocol_id > PROTOCOL_L21)
    {
        return std::nullopt;
    }
    return sendCommand(CMD_SET_PROTOCOL, {protocol_id});
}

std::optional<UartFrameCommand::Response> UartFrameCommand::canEnable()
{
    return sendCommand(CMD_CAN_ENABLE);
}

std::optional<UartFrameCommand::Response> UartFrameCommand::canDisable()
{
    return sendCommand(CMD_CAN_DISABLE);
}

std::optional<UartFrameCommand::Status> UartFrameCommand::status()
{
    auto resp = sendCommand(CMD_STATUS);
    if (resp && resp->result == RESULT_SUCCESS && resp->data.size() >= 6)
    {
        Status status;
        status.quick_state = resp->data[0];
        status.anchor_state = resp->data[1];
        status.frame_enabled = resp->data[2];
        status.sensor_print = resp->data[3];
        status.can_enabled = resp->data[4];
        status.sensor_send = resp->data[5];
        return status;
    }
    return std::nullopt;
}

void UartFrameCommand::close()
{
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (serial_port_ && serial_port_->isOpen())
    {
        // 尝试禁用所有数据推送
        try
        {
            sendCommand(CMD_MAPPING_DISABLE, {}, false);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            sendCommand(CMD_SENSOR_DISABLE, {}, false);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            sendCommand(CMD_DISABLE, {}, false);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        catch (...)
        {
        }

        serial_port_->flush();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        serial_port_->flush();
        std::this_thread::sleep_for(std::chrono::milliseconds(300));

        serial_port_->close();
    }
}

bool UartFrameCommand::isOpen() const
{
    return serial_port_ && serial_port_->isOpen();
}

size_t UartFrameCommand::bytesAvailable() const
{
    if (serial_port_)
    {
        return serial_port_->bytesAvailable();
    }
    return 0;
}

}  // namespace exhand_read

