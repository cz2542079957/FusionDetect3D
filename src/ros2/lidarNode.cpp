#include "lidarNode.h"

LidarNode::LidarNode(): Node("lidarNode")
{
}

void LidarNode::setCallback(std::function<void(const message::msg::LidarData::SharedPtr msg)> call)
{
    subscription = this->create_subscription<message::msg::LidarData>(nodePrefix + "/lidarScan", rclcpp::QoS(rclcpp::KeepLast(10)), call);

}

void LidarNode::run()
{
    executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(this->shared_from_this());
    executor->spin();
}
