#include "lidar/lidarNode.h"

LidarNode::LidarNode(): Node("lidarNodeSubscriber")
{
}

void LidarNode::setCallback(std::function<void(const sensor_msgs::msg::LaserScan::SharedPtr msg)> call)
{
    subscription = this->create_subscription<sensor_msgs::msg::LaserScan>(nodePrefix + "/lidarScan", rclcpp::QoS(rclcpp::KeepLast(10)), call);

}

void LidarNode::run()
{
    executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(this->shared_from_this());
    executor->spin();
}
