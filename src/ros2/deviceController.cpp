#include "deviceController.h"

DeviceController::DeviceController()
{
    rclcpp::init(0, NULL);
    std::thread([this]()
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("DeviceController"),  "小车控制板节点线程[启动]");
        auto carMasterNode = std::make_shared<CarMasterNode>();
        this->carMaterNode = carMasterNode;
        carMasterNode->setEncoderDataCallback([this](const message::msg::CarEncoderData::SharedPtr msg) -> void
        {
            this->encoderDataCallback(msg);
        });
        carMasterNode->setServoDataCallback([this](const message::msg::CarServoData::SharedPtr msg) -> void
        {
            this->servoDataCallback(msg);
        });
        carMasterNode->setVoltageDataCallback([this](const message::msg::CarVotageData::SharedPtr msg) -> void
        {
            this->voltageDataCallback(msg);
        });
        carMasterNode->run();
        RCLCPP_INFO_STREAM(rclcpp::get_logger("DeviceController"),  "小车控制板节点线程[退出]");
    })
    .detach();

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
    })
    .detach();

    std::thread([this]()
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("DeviceController"),  "惯导模块节点线程[启动]");
        auto imuNode = std::make_shared<LidarImuNode>();
        imuNode->setCallback([this](const message::msg::ImuData::SharedPtr msg) -> void
        {
            this->lidarImuDataCallback(msg);
        });
        imuNode->run();
        RCLCPP_INFO_STREAM(rclcpp::get_logger("DeviceController"),  "惯导模块节点线程[退出]");
    })
    .detach();
}

void DeviceController::lidarScanCallback(const message::msg::LidarData::SharedPtr msg)
{
    // RCLCPP_INFO_STREAM(rclcpp::get_logger("lidarNodeSubscriber"),  msg->ranges.size());
    emit sendPointsSignal(msg);
    // for (size_t i = 0; i < msg->ranges.size(); ++i)
    // {

    //     RCLCPP_INFO_STREAM( rclcpp::get_logger("lidarNodeSubscriber"),
    //                         "Index: " << i << ", Range: " << msg->ranges[i] << ", Intensity: " << msg->intensities[i]);
    // }
}

void DeviceController::servoDataCallback(const message::msg::CarServoData::SharedPtr msg)
{
    // qDebug() << msg->angle1 << " " << msg->angle2;
    // todo 完成激光雷达扫描
    emit sendServoDataSignal(msg);
}

void DeviceController::voltageDataCallback(const message::msg::CarVotageData::SharedPtr msg)
{
    emit sendVoltageDataSignal(msg);
}

void DeviceController::lidarImuDataCallback(const message::msg::ImuData::SharedPtr msg)
{
    // std::vector<message::msg::ImuDataFrame> list = msg->data;
    // for (int i = 0 ; i < list.size(); i ++)
    // {
    //     RCLCPP_INFO_STREAM(rclcpp::get_logger("DeviceController"),  i << " " << list[i].timestamp);
    // }
    emit sendLidarImuDataSignal(msg);
}

void DeviceController::encoderDataCallback(const message::msg::CarEncoderData::SharedPtr msg)
{
    emit sendEncoderDataSignal(msg);
}

void DeviceController::sendControlSlot(int state, int speed)
{
    // qDebug() << state << " " << speed;
    carMaterNode->publishMotionControl(state, speed);
}
