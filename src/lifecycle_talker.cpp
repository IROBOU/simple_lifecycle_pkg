#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"

using namespace std::chrono_literals;

// class MinimalPublisher : public rclcpp::Node
class MinimalPublisher : public rclcpp_lifecycle::LifecycleNode
{
public:
    MinimalPublisher()
        // : Node("minimal_publisher"), count_(0)
        : rclcpp_lifecycle::LifecycleNode("minimal_publisher"), count_(0)
    {
        // publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        // timer_ = this->create_wall_timer(
        //     500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &state)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(1s, std::bind(&MinimalPublisher::timer_callback, this));

        RCUTILS_LOG_INFO_NAMED(get_name(), "on_configure() is called from state %s.", state.label().c_str());

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &state)
    {

        publisher_->on_activate();

        RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called from state %s.", state.label().c_str());

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &state)
    {
        publisher_->on_deactivate();

        RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called from state %s.", state.label().c_str());

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &state)
    {

        timer_.reset();
        publisher_.reset();

        RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called from state %s.", state.label().c_str());

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State &state)
    {

        timer_.reset();
        publisher_.reset();

        RCUTILS_LOG_INFO_NAMED(get_name(), "on shutdown is called from state %s.", state.label().c_str());

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Creating a map " + std::to_string(count_++);
        // Print the current state for demo purposes
        if (!publisher_->is_activated())
        {
            RCLCPP_INFO(
                get_logger(), "Lifecycle publisher is currently inactive. Messages are not published.");
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Lifecycle publisher is active. Publishing: [%s]", message.data.c_str());
        }

        // We independently from the current state call publish on the lifecycle
        // publisher.
        // Only if the publisher is in an active state, the message transfer is
        // enabled and the message actua        // subscription_ = this->create_subscription<std_msgs::msg::String>(
        //     "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
        publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> publisher_;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<MinimalPublisher> minimal_publisher = std::make_shared<MinimalPublisher>();
    rclcpp::spin(minimal_publisher->get_node_base_interface());
    // rclcpp::spin(std::make_shared<MinimalPublisher>()); //origianl
    rclcpp::shutdown();
    return 0;
}