// include/motion_primitives/in_place_turn_node.hpp
#pragma once
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace motion_primitives
{

    class InPlaceTurnNode : public rclcpp_lifecycle::LifecycleNode
    {
    public:
        InPlaceTurnNode();

    private:
        using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

        rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        double angular_speed_{0.5}; // [rad/s], パラメータで可変

        CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
        CallbackReturn on_error(const rclcpp_lifecycle::State &) override;
    };

} // namespace motion_primitives
