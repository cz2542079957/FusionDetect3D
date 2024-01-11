#ifndef DEVICECONTROLLER_H
#define DEVICECONTROLLER_H
#include <QObject>

class DeviceController: public QObject
{
    Q_OBJECT
public:
    DeviceController();

};

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber()
        : Node("minimal_subscriber")
    {
        auto callback = [this](const std_msgs::msg::String::SharedPtr msg) -> void
        {
            printf("456\n");
            RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
        };

        subscription_ = this->create_subscription<std_msgs::msg::String>("my_topic", 10, callback);
    }

    void spin_thread()
    {
        // 创建一个单一执行器并添加订阅者到执行器
        auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor->add_node(this->shared_from_this());

        // 在子线程中运行执行器
        std::thread{[executor]()
        {
            executor->spin();
        }}.detach();
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher()
        : Node("minimal_publisher")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("my_topic", 10);

        printf("create");
        auto timer_callback = [this]() -> void
        {
            auto message = std_msgs::msg::String();
            message.data = "Hello, ROS 2!";
            printf("123");
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            this->publisher_->publish(message);
        };


        timer_ = this->create_wall_timer(
                     std::chrono::milliseconds(100), // 发布频率为每秒一次
                     timer_callback);
    }

    void spin_thread()
    {
        // 创建一个单一执行器并添加当前节点到执行器
        auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor->add_node(this->shared_from_this());
        // 在子线程中运行执行器
        std::thread{[executor]()
        {
            executor->spin();
        }}.detach();
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

};



#endif // DEVICECONTROLLER_H
