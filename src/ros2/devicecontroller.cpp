#include "devicecontroller.h"

DeviceController::DeviceController()
{
    rclcpp::init(0, NULL);
    std::thread([this]()
    {
        auto lidarNode = std::make_shared<LidarNode>();
        lidarNode->setCallback([this](const sensor_msgs::msg::LaserScan::SharedPtr msg) -> void
        {
            this->lidarScanCallback(msg);
        } );
        lidarNode->run();
    }).detach();
}

void DeviceController::lidarScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // RCLCPP_INFO_STREAM(rclcpp::get_logger("lidarNodeSubscriber"),  msg->ranges.size());
    emit sendPointsSignals(msg);
    // for (size_t i = 0; i < msg->ranges.size(); ++i)
    // {

    //     RCLCPP_INFO_STREAM( rclcpp::get_logger("lidarNodeSubscriber"),
    //                         "Index: " << i << ", Range: " << msg->ranges[i] << ", Intensity: " << msg->intensities[i]);
    // }
}


