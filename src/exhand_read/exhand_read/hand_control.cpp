/**
 * @file hand_control.cpp
 * @brief 手部控制接口实现
 */

#include "exhand_read/hand_control.hpp"
#include <algorithm>
#include <cstring>

namespace exhand_read
{

HandControl::HandControl()
    : devices_{}
    , target_device_id_(HAND_ID_RIGHT)
    , default_device_id_(HAND_ID_RIGHT)
    , current_protocol_(HandProtocol::HAND_PROTO_L10)
    , send_counter_(0)
{
    // 初始化默认设备
    initDevice(HAND_ID_RIGHT);
    initDevice(HAND_ID_LEFT);
}

HandControl::~HandControl()
{
}

int HandControl::findDeviceIndex(uint32_t device_id) const
{
    for (size_t i = 0; i < HAND_MAX_DEVICES; i++) {
        if (devices_[i].initialized && devices_[i].device_id == device_id) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

HandUserControl* HandControl::getDeviceControl(uint32_t device_id)
{
    int idx = findDeviceIndex(device_id);
    if (idx >= 0) {
        return &devices_[idx].user_control;
    }
    // 如果设备不存在，返回默认设备（向后兼容）
    if (findDeviceIndex(default_device_id_) >= 0) {
        return &devices_[findDeviceIndex(default_device_id_)].user_control;
    }
    return nullptr;
}

void HandControl::applyDefaultsToFinger(HandFingerCmd& finger)
{
    if (finger.pitch_angle == 0) finger.pitch_angle = HAND_DEFAULT_VALUE;
    if (finger.yaw_angle == 0) finger.yaw_angle = HAND_DEFAULT_VALUE_YAW;
    if (finger.roll_angle == 0) finger.roll_angle = HAND_DEFAULT_VALUE;
    if (finger.tip_angle == 0) finger.tip_angle = HAND_DEFAULT_VALUE;
    if (finger.speed_angle == 0) finger.speed_angle = HAND_DEFAULT_VALUE;
    if (finger.over_current == 0) finger.over_current = HAND_DEFAULT_VALUE;
}

void HandControl::convertUserControlToSendDataEx(uint32_t device_id, HandControlRequest& request)
{
    HandUserControl* control = getDeviceControl(device_id);
    if (!control) {
        return;
    }
    
    // 将用户控制数据转换为发送数据
    for (size_t i = 0; i < HAND_NUM_FINGERS; i++) {
        // 限制输入范围到0-1
        float pitch = std::clamp(control->fingers[i].pitch_angle, 0.0f, 1.0f);
        float yaw = std::clamp(control->fingers[i].yaw_angle, 0.0f, 1.0f);
        float roll = std::clamp(control->fingers[i].roll_angle, 0.0f, 1.0f);
        float tip = std::clamp(control->fingers[i].tip_angle, 0.0f, 1.0f);
        float speed = std::clamp(control->fingers[i].speed_angle, 0.0f, 1.0f);
        float current = std::clamp(control->fingers[i].over_current, 0.0f, 1.0f);
        
        // 应用全局速度缩放
        float scaled_speed = speed * control->global_speed;
        if (scaled_speed > 1.0f) scaled_speed = 1.0f;
        
        // 应用全局电流限制
        float scaled_current = current * control->global_current;
        if (scaled_current > 1.0f) scaled_current = 1.0f;
        
        // 转换为0-255范围并限幅到5-250
        uint8_t pitch_val = static_cast<uint8_t>(pitch * 255.0f);
        uint8_t yaw_val = static_cast<uint8_t>(yaw * 255.0f);
        uint8_t roll_val = static_cast<uint8_t>(roll * 255.0f);
        uint8_t tip_val = static_cast<uint8_t>(tip * 255.0f);
        uint8_t speed_val = static_cast<uint8_t>(scaled_speed * 255.0f);
        uint8_t current_val = static_cast<uint8_t>(scaled_current * 255.0f);
        
        // 限幅到5-250范围
        pitch_val = std::clamp(pitch_val, static_cast<uint8_t>(5), static_cast<uint8_t>(250));
        yaw_val = std::clamp(yaw_val, static_cast<uint8_t>(5), static_cast<uint8_t>(250));
        roll_val = std::clamp(roll_val, static_cast<uint8_t>(5), static_cast<uint8_t>(250));
        tip_val = std::clamp(tip_val, static_cast<uint8_t>(5), static_cast<uint8_t>(250));
        speed_val = std::clamp(speed_val, static_cast<uint8_t>(5), static_cast<uint8_t>(255));  // 速度允许到255
        current_val = std::clamp(current_val, static_cast<uint8_t>(5), static_cast<uint8_t>(255));  // 电流允许到255
        
        request.fingers[i].pitch_angle = pitch_val;
        request.fingers[i].yaw_angle = yaw_val;
        request.fingers[i].roll_angle = roll_val;
        request.fingers[i].tip_angle = tip_val;
        request.fingers[i].speed_angle = speed_val;
        request.fingers[i].over_current = current_val;
        request.fingers[i].clear_fault = control->fingers[i].clear_fault;
    }
}

int HandControl::initDevice(uint32_t device_id)
{
    // 检查设备是否已存在
    if (findDeviceIndex(device_id) >= 0) {
        return -1;  // 设备已存在
    }
    
    // 查找空闲槽位
    for (size_t i = 0; i < HAND_MAX_DEVICES; i++) {
        if (!devices_[i].initialized) {
            devices_[i].device_id = device_id;
            devices_[i].initialized = true;
            // 控制数据已通过默认构造函数初始化
            return 0;  // 成功
        }
    }
    
    return -1;  // 没有空闲槽位
}

void HandControl::setTargetDevice(uint32_t device_id)
{
    target_device_id_ = device_id;
    default_device_id_ = device_id;  // 同时更新默认设备ID
}

// 单设备控制接口实现（使用默认设备）
void HandControl::setFingerPitch(FingerIndex finger_idx, float angle)
{
    setFingerPitchEx(default_device_id_, finger_idx, angle);
}

void HandControl::setFingerYaw(FingerIndex finger_idx, float angle)
{
    setFingerYawEx(default_device_id_, finger_idx, angle);
}

void HandControl::setFingerRoll(FingerIndex finger_idx, float angle)
{
    setFingerRollEx(default_device_id_, finger_idx, angle);
}

void HandControl::setFingerTip(FingerIndex finger_idx, float angle)
{
    setFingerTipEx(default_device_id_, finger_idx, angle);
}

void HandControl::setFingerSpeed(FingerIndex finger_idx, float speed)
{
    setFingerSpeedEx(default_device_id_, finger_idx, speed);
}

void HandControl::setFingerCurrent(FingerIndex finger_idx, float current)
{
    setFingerCurrentEx(default_device_id_, finger_idx, current);
}

void HandControl::setFingerAll(FingerIndex finger_idx, float pitch, float yaw,
                                float roll, float tip, float speed, float current)
{
    setFingerAllEx(default_device_id_, finger_idx, pitch, yaw, roll, tip, speed, current);
}

void HandControl::setAllFingersPitch(float angle)
{
    setAllFingersPitchEx(default_device_id_, angle);
}

void HandControl::setAllFingersYaw(float angle)
{
    setAllFingersYawEx(default_device_id_, angle);
}

void HandControl::setAllFingersRoll(float angle)
{
    setAllFingersRollEx(default_device_id_, angle);
}

void HandControl::setAllFingersTip(float angle)
{
    setAllFingersTipEx(default_device_id_, angle);
}

void HandControl::setAllFingersSpeed(float speed)
{
    setAllFingersSpeedEx(default_device_id_, speed);
}

void HandControl::setAllFingersCurrent(float current)
{
    setAllFingersCurrentEx(default_device_id_, current);
}

void HandControl::setAllFingersAll(float pitch, float yaw, float roll,
                                   float tip, float speed, float current)
{
    setAllFingersAllEx(default_device_id_, pitch, yaw, roll, tip, speed, current);
}

void HandControl::setGlobalSpeed(float speed)
{
    setGlobalSpeedEx(default_device_id_, speed);
}

void HandControl::setGlobalCurrent(float current)
{
    setGlobalCurrentEx(default_device_id_, current);
}

void HandControl::setFaultClear(uint8_t enable)
{
    setFaultClearEx(default_device_id_, enable);
}

// 多设备控制接口实现（指定设备）
void HandControl::setFingerPitchEx(uint32_t device_id, FingerIndex finger_idx, float angle)
{
    if (static_cast<uint8_t>(finger_idx) < HAND_NUM_FINGERS) {
        HandUserControl* control = getDeviceControl(device_id);
        if (control) {
            control->fingers[static_cast<size_t>(finger_idx)].pitch_angle = angle;
        }
    }
}

void HandControl::setFingerYawEx(uint32_t device_id, FingerIndex finger_idx, float angle)
{
    if (static_cast<uint8_t>(finger_idx) < HAND_NUM_FINGERS) {
        HandUserControl* control = getDeviceControl(device_id);
        if (control) {
            control->fingers[static_cast<size_t>(finger_idx)].yaw_angle = angle;
        }
    }
}

void HandControl::setFingerRollEx(uint32_t device_id, FingerIndex finger_idx, float angle)
{
    if (static_cast<uint8_t>(finger_idx) < HAND_NUM_FINGERS) {
        HandUserControl* control = getDeviceControl(device_id);
        if (control) {
            control->fingers[static_cast<size_t>(finger_idx)].roll_angle = angle;
        }
    }
}

void HandControl::setFingerTipEx(uint32_t device_id, FingerIndex finger_idx, float angle)
{
    if (static_cast<uint8_t>(finger_idx) < HAND_NUM_FINGERS) {
        HandUserControl* control = getDeviceControl(device_id);
        if (control) {
            control->fingers[static_cast<size_t>(finger_idx)].tip_angle = angle;
        }
    }
}

void HandControl::setFingerSpeedEx(uint32_t device_id, FingerIndex finger_idx, float speed)
{
    if (static_cast<uint8_t>(finger_idx) < HAND_NUM_FINGERS) {
        HandUserControl* control = getDeviceControl(device_id);
        if (control) {
            control->fingers[static_cast<size_t>(finger_idx)].speed_angle = speed;
        }
    }
}

void HandControl::setFingerCurrentEx(uint32_t device_id, FingerIndex finger_idx, float current)
{
    if (static_cast<uint8_t>(finger_idx) < HAND_NUM_FINGERS) {
        HandUserControl* control = getDeviceControl(device_id);
        if (control) {
            control->fingers[static_cast<size_t>(finger_idx)].over_current = current;
        }
    }
}

void HandControl::setFingerAllEx(uint32_t device_id, FingerIndex finger_idx, float pitch, float yaw,
                                  float roll, float tip, float speed, float current)
{
    if (static_cast<uint8_t>(finger_idx) < HAND_NUM_FINGERS) {
        HandUserControl* control = getDeviceControl(device_id);
        if (control) {
            auto& finger = control->fingers[static_cast<size_t>(finger_idx)];
            finger.pitch_angle = pitch;
            finger.yaw_angle = yaw;
            finger.roll_angle = roll;
            finger.tip_angle = tip;
            finger.speed_angle = speed;
            finger.over_current = current;
        }
    }
}

void HandControl::setAllFingersPitchEx(uint32_t device_id, float angle)
{
    HandUserControl* control = getDeviceControl(device_id);
    if (control) {
        for (size_t i = 0; i < HAND_NUM_FINGERS; i++) {
            control->fingers[i].pitch_angle = angle;
        }
    }
}

void HandControl::setAllFingersYawEx(uint32_t device_id, float angle)
{
    HandUserControl* control = getDeviceControl(device_id);
    if (control) {
        for (size_t i = 0; i < HAND_NUM_FINGERS; i++) {
            control->fingers[i].yaw_angle = angle;
        }
    }
}

void HandControl::setAllFingersRollEx(uint32_t device_id, float angle)
{
    HandUserControl* control = getDeviceControl(device_id);
    if (control) {
        for (size_t i = 0; i < HAND_NUM_FINGERS; i++) {
            control->fingers[i].roll_angle = angle;
        }
    }
}

void HandControl::setAllFingersTipEx(uint32_t device_id, float angle)
{
    HandUserControl* control = getDeviceControl(device_id);
    if (control) {
        for (size_t i = 0; i < HAND_NUM_FINGERS; i++) {
            control->fingers[i].tip_angle = angle;
        }
    }
}

void HandControl::setAllFingersSpeedEx(uint32_t device_id, float speed)
{
    HandUserControl* control = getDeviceControl(device_id);
    if (control) {
        for (size_t i = 0; i < HAND_NUM_FINGERS; i++) {
            control->fingers[i].speed_angle = speed;
        }
    }
}

void HandControl::setAllFingersCurrentEx(uint32_t device_id, float current)
{
    HandUserControl* control = getDeviceControl(device_id);
    if (control) {
        for (size_t i = 0; i < HAND_NUM_FINGERS; i++) {
            control->fingers[i].over_current = current;
        }
    }
}

void HandControl::setAllFingersAllEx(uint32_t device_id, float pitch, float yaw, float roll,
                                     float tip, float speed, float current)
{
    HandUserControl* control = getDeviceControl(device_id);
    if (control) {
        for (size_t i = 0; i < HAND_NUM_FINGERS; i++) {
            auto& finger = control->fingers[i];
            finger.pitch_angle = pitch;
            finger.yaw_angle = yaw;
            finger.roll_angle = roll;
            finger.tip_angle = tip;
            finger.speed_angle = speed;
            finger.over_current = current;
        }
    }
}

void HandControl::setGlobalSpeedEx(uint32_t device_id, float speed)
{
    HandUserControl* control = getDeviceControl(device_id);
    if (control) {
        control->global_speed = speed;
    }
}

void HandControl::setGlobalCurrentEx(uint32_t device_id, float current)
{
    HandUserControl* control = getDeviceControl(device_id);
    if (control) {
        control->global_current = current;
    }
}

void HandControl::setFaultClearEx(uint32_t device_id, uint8_t enable)
{
    HandUserControl* control = getDeviceControl(device_id);
    if (control) {
        control->enable_fault_clear = enable;
        for (size_t i = 0; i < HAND_NUM_FINGERS; i++) {
            control->fingers[i].clear_fault = enable;
        }
    }
}

void HandControl::convertUserControlToSendData()
{
    int idx = findDeviceIndex(default_device_id_);
    if (idx >= 0) {
        convertUserControlToSendDataEx(default_device_id_, devices_[idx].hand_request);
    }
}

void HandControl::convertUserControlToSendData(uint32_t device_id, HandControlRequest& request)
{
    convertUserControlToSendDataEx(device_id, request);
}

}  // namespace exhand_read

