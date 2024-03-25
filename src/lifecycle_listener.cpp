#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

#include "rclcpp_lifecycle/lifecycle_node.hpp"

class MinimalSubscriber : public rclcpp_lifecycle::LifecycleNode
{
public:
    MinimalSubscriber()
        : rclcpp_lifecycle::LifecycleNode("minimal_subscriber")
    {
        // subscription_ = this->create_subscription<std_msgs::msg::String>(
        //     "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &state)
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
        m_isActivated = false;
        RCUTILS_LOG_INFO_NAMED(get_name(), "on_configure() is called from state %s.", state.label().c_str());

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &state)
    {
        m_isActivated = true;
        RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called from state %s.", state.label().c_str());

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &state)
    {
        m_isActivated = false;
        RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called from state %s.", state.label().c_str());

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &state)
    {
        subscription_.reset();
        RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called from state %s.", state.label().c_str());

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State &state)
    {
        subscription_.reset();
        RCUTILS_LOG_INFO_NAMED(get_name(), "on shutdown is called from state %s.", state.label().c_str());

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

private:
    void topic_callback(const std_msgs::msg::String &msg) const
    {
        if (m_isActivated)
        {
            RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
        }
        else
        {
            RCLCPP_INFO(
                get_logger(), "Subscriber is currently inactive. Callback function is not working .");
        }
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    bool m_isActivated = false;
    // std::shared_ptr<rclcpp_lifecycle::LifecycleSusbcription<std_msgs::msg::String>> subscription_; -> unfortunately no such thing
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<MinimalSubscriber> minimal_subscriber = std::make_shared<MinimalSubscriber>();
    rclcpp::spin(minimal_subscriber->get_node_base_interface());
    // rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}