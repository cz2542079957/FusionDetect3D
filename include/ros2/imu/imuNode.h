#ifndef IMUNODE_H
#define IMUNODE_H

#include "rclcpp/rclcpp.hpp"
#include "message/msg/imu_data.hpp"
#include "vector"

class ImuNode : public rclcpp::Node
{
public:
    ImuNode();
    void setCallback(std::function<void(const message::msg::ImuData::SharedPtr msg)> call);
    void run();
private:

    //节点前缀
    std::string nodePrefix = "/imuNode";
    //订阅者节点
    rclcpp::Subscription<message::msg::ImuData>::SharedPtr subscription;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;


};

#endif // IMUNODE_H
