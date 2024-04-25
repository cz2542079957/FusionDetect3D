#pragma once
#include "common.h"

class CarMasterNode : public rclcpp::Node
{
public:
    CarMasterNode();
    //设置编码器数据回调
    void setEncoderDataCallback(std::function<void(const message::msg::CarEncoderData::SharedPtr msg)> callback);
    //设置舵机数据回调
    void setServoDataCallback(std::function<void(const message::msg::CarServoData::SharedPtr msg)> callback);
    //设置电压数据回调
    void setVoltageDataCallback(std::function<void(const message::msg::CarVotageData::SharedPtr msg)> callback);

    //发布模式控制数据
    void publishModeControl(int mode);
    //发布运动控制数据
    void publishMotionControl(int state, int speed);

    void run();
private:
    //节点前缀
    std::string nodePrefix = "/carMasterNode";

    // 编码器数据订阅者
    rclcpp::Subscription<message::msg::CarEncoderData>::SharedPtr encoderDataSubscriber;
    // 舵机数据订阅者
    rclcpp::Subscription<message::msg::CarServoData>::SharedPtr servoDataSubscriber;
    // 电压数据订阅者
    rclcpp::Subscription<message::msg::CarVotageData>::SharedPtr voltageDataSubscriber;
    // 模式数据发布者
    rclcpp::Publisher<message::msg::ModeControl>::SharedPtr modeControlPublisher;
    // 运动控制发布者
    rclcpp::Publisher<message::msg::CarMotionControl>::SharedPtr motionControlPublisher;




    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;

};
