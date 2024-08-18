#include <functional>
#include <memory>
#include <string>

#include "rclcpp/type_adapter.hpp"
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"

template<>
struct rclcpp::Type::TypeAdapter<std::string, std_msgs::msg::String>
{
    using is_specialized = std::true_type;
    using custom_type = std::string;
    using ros_message_type = std_msgs::msg::String;

    static void convert_to_ros_message(const custom_type& source, ros_message_type& destination)
    {
        destination.data = source;
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination)
    {
        destination = source.data;
    }
};

class MinimalSubscriber : public rclcpp::Node
{
    using MyAdaptedType = rclcpp::TypeAdapter<std::string, std_msgs::msg::String>;

public:
    MinimalSubscriber() : Node("minimal_subscriber")
    {
        subscription_ = this->create_subscription<MyAdaptedType>(
            "topic", 1, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1);
        );
    }
private:
    void topic_callback(const std::string& msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.c_str());
    }

    rclcpp::Subscription<MyAdaptedType>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}