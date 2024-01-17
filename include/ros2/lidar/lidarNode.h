#ifndef LIDARNODE_H
#define LIDARNODE_H
#include <rclcpp/rclcpp.hpp>
#include "message/msg/lidar_data.hpp"

class LidarNode: public rclcpp::Node
{
public:
    LidarNode();
    void setCallback(std::function<void(const message::msg::LidarData::SharedPtr msg)> call);
    void run();

private:
    //节点前缀
    std::string nodePrefix = "/lidarNode";
    //订阅者节点
    rclcpp::Subscription<message::msg::LidarData>::SharedPtr subscription;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;

};

#endif // LIDARNODE_H
