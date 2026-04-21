#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <chrono>

/**
 * @class JoyNode
 * @brief A ROS 2 node that subscribes to the /joy topic, stores the last received message,
 * and republishes it every 0.2 seconds on the /joy_out topic.
 *
 * This node ensures that the latest joystick message is always available on the /joy_out topic,
 * even if the original /joy topic is not being published at a consistent rate.
 */
class JoyNode : public rclcpp::Node {
private:
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::Joy last_joy_msg_;
    bool is_first_message_;

public:
    JoyNode() : rclcpp::Node("republish_joy_node") {
        // Create publisher
        joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("/joy_out", 10);
        
        // Create subscriber
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&JoyNode::joyCallback, this, std::placeholders::_1)
        );
        
        // Create timer (200ms)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&JoyNode::timerCallback, this)
        );
        
        is_first_message_ = true;
        RCLCPP_INFO(this->get_logger(), "JoyNode initialized");
    }

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        if (is_first_message_ || *msg != last_joy_msg_) {
            last_joy_msg_ = *msg;
            is_first_message_ = false;
        }
    }

    void timerCallback() {
        joy_pub_->publish(last_joy_msg_);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}