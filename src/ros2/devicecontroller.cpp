#include "devicecontroller.h"

DeviceController::DeviceController()
{
    rclcpp::init(0, NULL);
    auto node1 = std::make_shared<MinimalPublisher>();

    auto node2 = std::make_shared<MinimalSubscriber>();

    node1->spin_thread();

    node2->spin_thread();

    // rclcpp::shutdown();
}

