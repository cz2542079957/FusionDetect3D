#pragma once
#include "common.h"

class CarMasterNode : public rclcpp::Node
{
public:
    CarMasterNode();
    // void setCallback(std::function<void(const message::msg::ImuData::SharedPtr msg)> call);
    //设置编码器数据回调
    void setEncoderCallback(std::function<void> callback);
    //设置舵机角度值数据回调
    void setServoAngleCallback(std::function<void> callback);

    void run();
private:
    //节点前缀
    std::string nodePrefix = "/carMasterNode";
    //订阅者节点
    // rclcpp::Subscription<message::msg::ImuData>::SharedPtr subscription;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;

};
