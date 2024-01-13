#ifndef LIDARNODE_H
#define LIDARNODE_H
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LidarNode: public rclcpp::Node
{
public:
    LidarNode();
    void setCallback(std::function<void(const sensor_msgs::msg::LaserScan::SharedPtr msg)>);
    void run();

private:
    //节点前缀
    std::string nodePrefix = "/lidarNode";
    //订阅者节点
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;

};

#endif // LIDARNODE_H
