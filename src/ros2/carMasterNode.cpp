#include "carMasterNode.h"

CarMasterNode::CarMasterNode(): Node("carMasterNode")
{
    modeControlPublisher  = this->create_publisher<message::msg::ModeControl>(nodePrefix + "/modeControl", rclcpp::QoS(rclcpp::KeepLast(10)));
    motionControlPublisher = this->create_publisher<message::msg::CarMotionControl>(nodePrefix + "/motionControl", rclcpp::QoS(rclcpp::KeepLast(10)));
}

void CarMasterNode::setEncoderDataCallback(std::function<void (const message::msg::CarEncoderData::SharedPtr)> callback)
{
    encoderDataSubscriber = this->create_subscription<message::msg::CarEncoderData>(nodePrefix + "/encoderData", rclcpp::QoS(rclcpp::KeepLast(10)),
                            callback);
}

void CarMasterNode::setServoDataCallback(std::function<void (const message::msg::CarServoData::SharedPtr)> callback)
{
    servoDataSubscriber = this->create_subscription<message::msg::CarServoData>(nodePrefix + "/servoData", rclcpp::QoS(rclcpp::KeepLast(10)),
                          callback);
}

void CarMasterNode::publishModeControl(int mode)
{
    auto message = std::make_shared<message::msg::ModeControl>();
    message->mode = mode;
    modeControlPublisher->publish(*message);
}

void CarMasterNode::publishMotionControl(int state, int speed)
{
    auto message = std::make_shared<message::msg::CarMotionControl>();
    message->state = state;
    message->speed = speed;
    motionControlPublisher->publish(*message);
}

void CarMasterNode::run()
{
    executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(this->shared_from_this());
    executor->spin();
}
