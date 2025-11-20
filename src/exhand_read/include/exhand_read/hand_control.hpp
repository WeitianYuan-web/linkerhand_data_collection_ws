/**
 * @file hand_control.hpp
 * @brief 手部控制接口头文件
 * 
 * 将嵌入式项目中的手部控制接口转换为C++类
 */

#ifndef EXHAND_READ_HAND_CONTROL_HPP_
#define EXHAND_READ_HAND_CONTROL_HPP_

#include <cstdint>
#include <array>

namespace exhand_read
{

/* ========================================================================== */
/*                              CONSTANTS                                    */
/* ========================================================================== */

constexpr uint8_t HAND_NUM_FINGERS = 5;
constexpr uint8_t HAND_DEFAULT_VALUE = 250;
constexpr uint8_t HAND_DEFAULT_VALUE_YAW = 127;

/* Hand device IDs */
constexpr uint32_t HAND_ID_LEFT = 0x28;
constexpr uint32_t HAND_ID_RIGHT = 0x27;

/* 最大支持的手部设备数量 */
constexpr uint8_t HAND_MAX_DEVICES = 2;

/* ========================================================================== */
/*                              ENUMS                                        */
/* ========================================================================== */

/**
 * @brief 手指索引枚举
 */
enum class FingerIndex : uint8_t
{
    FINGER_THUMB = 0,
    FINGER_INDEX,
    FINGER_MIDDLE,
    FINGER_RING,
    FINGER_LITTLE
};

/**
 * @brief 协议类型：L20 为现有多帧协议；L10 为单帧7字节协议；L21 为简化多帧协议
 * L7 为7自由度协议；L25 为25自由度协议；O6 为6自由度协议；L24 为24自由度协议
 */
enum class HandProtocol : uint8_t
{
    HAND_PROTO_L20 = 0,
    HAND_PROTO_L10 = 1,
    HAND_PROTO_L21 = 2,
    HAND_PROTO_L7 = 3,
    HAND_PROTO_L25 = 4,
    HAND_PROTO_O6 = 5,
    HAND_PROTO_L24 = 6
};

/* ========================================================================== */
/*                              STRUCTURES                                   */
/* ========================================================================== */

/**
 * @brief 单个手指控制命令
 */
struct HandFingerCmd
{
    uint8_t pitch_angle;    /* 俯仰角 */
    uint8_t yaw_angle;      /* 横摆角 */
    uint8_t roll_angle;      /* 横滚角 */
    uint8_t tip_angle;      /* 指尖角 */
    uint8_t speed_angle;    /* 速度 */
    uint8_t over_current;   /* 过流保护 */
    uint8_t clear_fault;    /* 清除故障 */
    
    HandFingerCmd() : pitch_angle(0), yaw_angle(0), roll_angle(0), 
                      tip_angle(0), speed_angle(0), over_current(0), clear_fault(0) {}
};

/**
 * @brief 整手控制请求
 */
struct HandControlRequest
{
    std::array<HandFingerCmd, HAND_NUM_FINGERS> fingers;  /* 五指控制 */
    HandFingerCmd thumb_aux;                              /* 大拇指辅助 */
    
    HandControlRequest() : fingers{}, thumb_aux{} {}
};

/**
 * @brief 用户控制数据结构 - 用户只能修改这个结构，所有值归一化到0-1
 */
struct HandUserControl
{
    /**
     * @brief 手指控制数据
     */
    struct FingerControl
    {
        float pitch_angle;    /* 俯仰角 (0.0-1.0) */
        float yaw_angle;      /* 横摆角 (0.0-1.0) */
        float roll_angle;     /* 横滚角 (0.0-1.0) */
        float tip_angle;      /* 指尖角 (0.0-1.0) */
        float speed_angle;    /* 速度 (0.0-1.0) */
        float over_current;   /* 过流保护 (0.0-1.0) */
        uint8_t clear_fault;  /* 清除故障 (0/1) */
        
        FingerControl() : pitch_angle(0.5f), yaw_angle(0.5f), roll_angle(0.5f),
                          tip_angle(0.5f), speed_angle(1.0f), over_current(1.0f), clear_fault(0) {}
    };
    
    std::array<FingerControl, HAND_NUM_FINGERS> fingers;  /* 手指控制数据 */
    
    /* 全局控制参数 */
    float global_speed;        /* 全局速度缩放 (0.0-1.0) */
    float global_current;      /* 全局电流限制 (0.0-1.0) */
    uint8_t enable_fault_clear; /* 启用故障清除 (0/1) */
    
    HandUserControl() : fingers{}, global_speed(1.0f), global_current(1.0f), enable_fault_clear(0) {}
};

/* ========================================================================== */
/*                              CLASS                                        */
/* ========================================================================== */

/**
 * @brief 手部控制类
 * 
 * 提供手部控制接口，支持多设备管理和协议转换
 */
class HandControl
{
public:
    /**
     * @brief 构造函数
     */
    HandControl();
    
    /**
     * @brief 析构函数
     */
    ~HandControl();
    
    /* ====================================================================== */
    /*                          设备管理接口                                  */
    /* ====================================================================== */
    
    /**
     * @brief 初始化设备控制数据结构
     * @param device_id 设备ID
     * @return 0:成功, -1:失败（设备已存在或超过最大设备数）
     */
    int initDevice(uint32_t device_id);
    
    /**
     * @brief 设置目标设备ID
     * @param device_id 设备ID
     */
    void setTargetDevice(uint32_t device_id);
    
    /**
     * @brief 获取目标设备ID
     * @return 设备ID
     */
    uint32_t getTargetDevice() const { return target_device_id_; }
    
    /* ====================================================================== */
    /*                          协议控制接口                                  */
    /* ====================================================================== */
    
    /**
     * @brief 设置当前发送协议（L20/L10/L21）
     * @param proto 协议类型
     */
    void setProtocol(HandProtocol proto) { current_protocol_ = proto; }
    
    /**
     * @brief 获取当前发送协议
     * @return 协议类型
     */
    HandProtocol getProtocol() const { return current_protocol_; }
    
    /* ====================================================================== */
    /*                          单设备控制接口（使用默认设备）                 */
    /* ====================================================================== */
    
    /**
     * @brief 设置单个手指的俯仰角
     * @param finger_idx 手指索引
     * @param angle 角度值 (0.0-1.0)
     */
    void setFingerPitch(FingerIndex finger_idx, float angle);
    
    /**
     * @brief 设置单个手指的横摆角
     * @param finger_idx 手指索引
     * @param angle 角度值 (0.0-1.0)
     */
    void setFingerYaw(FingerIndex finger_idx, float angle);
    
    /**
     * @brief 设置单个手指的横滚角
     * @param finger_idx 手指索引
     * @param angle 角度值 (0.0-1.0)
     */
    void setFingerRoll(FingerIndex finger_idx, float angle);
    
    /**
     * @brief 设置单个手指的指尖角
     * @param finger_idx 手指索引
     * @param angle 角度值 (0.0-1.0)
     */
    void setFingerTip(FingerIndex finger_idx, float angle);
    
    /**
     * @brief 设置单个手指的速度
     * @param finger_idx 手指索引
     * @param speed 速度值 (0.0-1.0)
     */
    void setFingerSpeed(FingerIndex finger_idx, float speed);
    
    /**
     * @brief 设置单个手指的电流限制
     * @param finger_idx 手指索引
     * @param current 电流值 (0.0-1.0)
     */
    void setFingerCurrent(FingerIndex finger_idx, float current);
    
    /**
     * @brief 设置单个手指的所有参数
     * @param finger_idx 手指索引
     * @param pitch 俯仰角 (0.0-1.0)
     * @param yaw 横摆角 (0.0-1.0)
     * @param roll 横滚角 (0.0-1.0)
     * @param tip 指尖角 (0.0-1.0)
     * @param speed 速度 (0.0-1.0)
     * @param current 电流 (0.0-1.0)
     */
    void setFingerAll(FingerIndex finger_idx, float pitch, float yaw, 
                      float roll, float tip, float speed, float current);
    
    /**
     * @brief 设置所有手指的俯仰角
     * @param angle 角度值 (0.0-1.0)
     */
    void setAllFingersPitch(float angle);
    
    /**
     * @brief 设置所有手指的横摆角
     * @param angle 角度值 (0.0-1.0)
     */
    void setAllFingersYaw(float angle);
    
    /**
     * @brief 设置所有手指的横滚角
     * @param angle 角度值 (0.0-1.0)
     */
    void setAllFingersRoll(float angle);
    
    /**
     * @brief 设置所有手指的指尖角
     * @param angle 角度值 (0.0-1.0)
     */
    void setAllFingersTip(float angle);
    
    /**
     * @brief 设置所有手指的速度
     * @param speed 速度值 (0.0-1.0)
     */
    void setAllFingersSpeed(float speed);
    
    /**
     * @brief 设置所有手指的电流限制
     * @param current 电流值 (0.0-1.0)
     */
    void setAllFingersCurrent(float current);
    
    /**
     * @brief 设置所有手指为相同参数
     * @param pitch 俯仰角 (0.0-1.0)
     * @param yaw 横摆角 (0.0-1.0)
     * @param roll 横滚角 (0.0-1.0)
     * @param tip 指尖角 (0.0-1.0)
     * @param speed 速度 (0.0-1.0)
     * @param current 电流 (0.0-1.0)
     */
    void setAllFingersAll(float pitch, float yaw, float roll, 
                          float tip, float speed, float current);
    
    /**
     * @brief 设置全局速度缩放
     * @param speed 全局速度缩放 (0.0-1.0)
     */
    void setGlobalSpeed(float speed);
    
    /**
     * @brief 设置全局电流限制
     * @param current 全局电流限制 (0.0-1.0)
     */
    void setGlobalCurrent(float current);
    
    /**
     * @brief 启用/禁用故障清除
     * @param enable 1:启用, 0:禁用
     */
    void setFaultClear(uint8_t enable);
    
    /* ====================================================================== */
    /*                          多设备控制接口（指定设备）                    */
    /* ====================================================================== */
    
    /**
     * @brief 设置指定设备的单个手指俯仰角
     * @param device_id 设备ID
     * @param finger_idx 手指索引
     * @param angle 角度值 (0.0-1.0)
     */
    void setFingerPitchEx(uint32_t device_id, FingerIndex finger_idx, float angle);
    
    /**
     * @brief 设置指定设备的单个手指横摆角
     * @param device_id 设备ID
     * @param finger_idx 手指索引
     * @param angle 角度值 (0.0-1.0)
     */
    void setFingerYawEx(uint32_t device_id, FingerIndex finger_idx, float angle);
    
    /**
     * @brief 设置指定设备的单个手指横滚角
     * @param device_id 设备ID
     * @param finger_idx 手指索引
     * @param angle 角度值 (0.0-1.0)
     */
    void setFingerRollEx(uint32_t device_id, FingerIndex finger_idx, float angle);
    
    /**
     * @brief 设置指定设备的单个手指指尖角
     * @param device_id 设备ID
     * @param finger_idx 手指索引
     * @param angle 角度值 (0.0-1.0)
     */
    void setFingerTipEx(uint32_t device_id, FingerIndex finger_idx, float angle);
    
    /**
     * @brief 设置指定设备的单个手指速度
     * @param device_id 设备ID
     * @param finger_idx 手指索引
     * @param speed 速度值 (0.0-1.0)
     */
    void setFingerSpeedEx(uint32_t device_id, FingerIndex finger_idx, float speed);
    
    /**
     * @brief 设置指定设备的单个手指电流限制
     * @param device_id 设备ID
     * @param finger_idx 手指索引
     * @param current 电流值 (0.0-1.0)
     */
    void setFingerCurrentEx(uint32_t device_id, FingerIndex finger_idx, float current);
    
    /**
     * @brief 设置指定设备的单个手指的所有参数
     * @param device_id 设备ID
     * @param finger_idx 手指索引
     * @param pitch 俯仰角 (0.0-1.0)
     * @param yaw 横摆角 (0.0-1.0)
     * @param roll 横滚角 (0.0-1.0)
     * @param tip 指尖角 (0.0-1.0)
     * @param speed 速度 (0.0-1.0)
     * @param current 电流 (0.0-1.0)
     */
    void setFingerAllEx(uint32_t device_id, FingerIndex finger_idx, float pitch, float yaw,
                        float roll, float tip, float speed, float current);
    
    /**
     * @brief 设置指定设备的所有手指俯仰角
     * @param device_id 设备ID
     * @param angle 角度值 (0.0-1.0)
     */
    void setAllFingersPitchEx(uint32_t device_id, float angle);
    
    /**
     * @brief 设置指定设备的所有手指横摆角
     * @param device_id 设备ID
     * @param angle 角度值 (0.0-1.0)
     */
    void setAllFingersYawEx(uint32_t device_id, float angle);
    
    /**
     * @brief 设置指定设备的所有手指横滚角
     * @param device_id 设备ID
     * @param angle 角度值 (0.0-1.0)
     */
    void setAllFingersRollEx(uint32_t device_id, float angle);
    
    /**
     * @brief 设置指定设备的所有手指指尖角
     * @param device_id 设备ID
     * @param angle 角度值 (0.0-1.0)
     */
    void setAllFingersTipEx(uint32_t device_id, float angle);
    
    /**
     * @brief 设置指定设备的所有手指速度
     * @param device_id 设备ID
     * @param speed 速度值 (0.0-1.0)
     */
    void setAllFingersSpeedEx(uint32_t device_id, float speed);
    
    /**
     * @brief 设置指定设备的所有手指电流限制
     * @param device_id 设备ID
     * @param current 电流值 (0.0-1.0)
     */
    void setAllFingersCurrentEx(uint32_t device_id, float current);
    
    /**
     * @brief 设置指定设备的所有手指为相同参数
     * @param device_id 设备ID
     * @param pitch 俯仰角 (0.0-1.0)
     * @param yaw 横摆角 (0.0-1.0)
     * @param roll 横滚角 (0.0-1.0)
     * @param tip 指尖角 (0.0-1.0)
     * @param speed 速度 (0.0-1.0)
     * @param current 电流 (0.0-1.0)
     */
    void setAllFingersAllEx(uint32_t device_id, float pitch, float yaw, float roll,
                            float tip, float speed, float current);
    
    /**
     * @brief 设置指定设备的全局速度缩放
     * @param device_id 设备ID
     * @param speed 全局速度缩放 (0.0-1.0)
     */
    void setGlobalSpeedEx(uint32_t device_id, float speed);
    
    /**
     * @brief 设置指定设备的全局电流限制
     * @param device_id 设备ID
     * @param current 全局电流限制 (0.0-1.0)
     */
    void setGlobalCurrentEx(uint32_t device_id, float current);
    
    /**
     * @brief 设置指定设备的故障清除
     * @param device_id 设备ID
     * @param enable 1:启用, 0:禁用
     */
    void setFaultClearEx(uint32_t device_id, uint8_t enable);
    
    /* ====================================================================== */
    /*                          数据发送接口                                  */
    /* ====================================================================== */
    
    /**
     * @brief 将用户控制数据转换为发送数据（使用默认设备）
     */
    void convertUserControlToSendData();
    
    /**
     * @brief 将指定设备的用户控制数据转换为发送数据（公有接口）
     * @param device_id 设备ID
     * @param request 输出：发送数据结构
     */
    void convertUserControlToSendData(uint32_t device_id, HandControlRequest& request);
    
    /**
     * @brief 应用默认值到单个手指
     * @param finger 手指控制命令
     */
    void applyDefaultsToFinger(HandFingerCmd& finger);

private:
    /**
     * @brief 设备管理结构
     */
    struct HandDevice
    {
        uint32_t device_id;              /* 设备ID */
        HandUserControl user_control;    /* 用户控制数据 */
        HandControlRequest hand_request; /* 发送数据结构 */
        bool initialized;                /* 是否已初始化 */
        
        HandDevice() : device_id(0), user_control{}, hand_request{}, initialized(false) {}
    };
    
    /**
     * @brief 查找设备索引
     * @param device_id 设备ID
     * @return 设备索引，-1表示未找到
     */
    int findDeviceIndex(uint32_t device_id) const;
    
    /**
     * @brief 获取设备控制数据指针（内部函数）
     * @param device_id 设备ID
     * @return 设备控制数据指针，NULL表示设备不存在
     */
    HandUserControl* getDeviceControl(uint32_t device_id);
    
    /**
     * @brief 将指定设备的用户控制数据转换为发送数据
     * @param device_id 设备ID
     * @param request 输出：发送数据结构
     */
    void convertUserControlToSendDataEx(uint32_t device_id, HandControlRequest& request);
    
    /* 设备管理 */
    std::array<HandDevice, HAND_MAX_DEVICES> devices_;
    uint32_t target_device_id_;          /* 目标设备ID */
    uint32_t default_device_id_;         /* 默认设备ID */
    HandProtocol current_protocol_;      /* 当前协议 */
    uint32_t send_counter_;              /* 发送计数器 */
};

}  // namespace exhand_read

#endif  // EXHAND_READ_HAND_CONTROL_HPP_

