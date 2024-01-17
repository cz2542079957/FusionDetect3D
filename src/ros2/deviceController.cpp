#include "deviceController.h"

DeviceController::DeviceController()
{
    rclcpp::init(0, NULL);
    std::thread([this]()
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("DeviceController"),  "激光雷达节点线程[启动]");
        auto lidarNode = std::make_shared<LidarNode>();
        lidarNode->setCallback([this](const message::msg::LidarData::SharedPtr msg) -> void
        {
            this->lidarScanCallback(msg);
        } );
        lidarNode->run();
        RCLCPP_INFO_STREAM(rclcpp::get_logger("DeviceController"),  "激光雷达节点线程[退出]");
    }).detach();

    std::thread([this]()
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("DeviceController"),  "惯导模块节点线程[启动]");
        auto imuNode = std::make_shared<ImuNode>();
        imuNode->setCallback([this](const message::msg::ImuData::SharedPtr msg) -> void
        {
            this->imuDataCallback(msg);
        });
        imuNode->run();
        RCLCPP_INFO_STREAM(rclcpp::get_logger("DeviceController"),  "惯导模块节点线程[退出]");
    }).detach();
}

void DeviceController::lidarScanCallback(const message::msg::LidarData::SharedPtr msg)
{
    // RCLCPP_INFO_STREAM(rclcpp::get_logger("lidarNodeSubscriber"),  msg->ranges.size());
    emit sendPointsSignals(msg);
    // for (size_t i = 0; i < msg->ranges.size(); ++i)
    // {

    //     RCLCPP_INFO_STREAM( rclcpp::get_logger("lidarNodeSubscriber"),
    //                         "Index: " << i << ", Range: " << msg->ranges[i] << ", Intensity: " << msg->intensities[i]);
    // }
}

void DeviceController::imuDataCallback(const message::msg::ImuData::SharedPtr msg)
{
    // std::vector<message::msg::ImuDataFrame> list = msg->data;
    // for (int i = 0 ; i < list.size(); i ++)
    // {
    //     RCLCPP_INFO_STREAM(rclcpp::get_logger("DeviceController"),  i << " " << list[i].timestemp);
    // }
    emit sendImuDataSignals(msg);
}


