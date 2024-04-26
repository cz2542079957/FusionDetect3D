#include "carImuNode.h"

CarImuNode::CarImuNode(): Node("carImuNode")
{

}

void CarImuNode::setCallback(std::function<void (const message::msg::ImuData::SharedPtr msg)> call)
{
    subscription = this->create_subscription<message::msg::ImuData>(nodePrefix + "/imuData", rclcpp::QoS(rclcpp::KeepLast(10)), call);
}

void CarImuNode::run()
{
    executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(this->shared_from_this());
    executor->spin();
}
