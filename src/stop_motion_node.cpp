// src/stop_motion_node.cpp
#include "motion_primitives/stop_motion_node.hpp"
#include <rclcpp/rclcpp.hpp>

namespace motion_primitives
{

    StopMotionNode::StopMotionNode()
        : rclcpp_lifecycle::LifecycleNode("stop_motion_node") {}

    auto StopMotionNode::on_configure(const rclcpp_lifecycle::State &) -> CallbackReturn
    {
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::QoS(10));
        RCLCPP_INFO(this->get_logger(), "configured.");
        return CallbackReturn::SUCCESS;
    }

    auto StopMotionNode::on_activate(const rclcpp_lifecycle::State &) -> CallbackReturn
    {
        cmd_pub_->on_activate();
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]()
            {
                geometry_msgs::msg::Twist cmd; // 既定で全ゼロ
                cmd_pub_->publish(cmd);
            });
        RCLCPP_INFO(this->get_logger(), "activated.");
        return CallbackReturn::SUCCESS;
    }

    auto StopMotionNode::on_deactivate(const rclcpp_lifecycle::State &) -> CallbackReturn
    {
        timer_.reset();
        cmd_pub_->on_deactivate();
        RCLCPP_INFO(this->get_logger(), "deactivated.");
        return CallbackReturn::SUCCESS;
    }

    auto StopMotionNode::on_cleanup(const rclcpp_lifecycle::State &) -> CallbackReturn
    {
        timer_.reset();
        cmd_pub_.reset();
        RCLCPP_INFO(this->get_logger(), "cleaned up.");
        return CallbackReturn::SUCCESS;
    }

    auto StopMotionNode::on_shutdown(const rclcpp_lifecycle::State &) -> CallbackReturn
    {
        RCLCPP_INFO(this->get_logger(), "shutdown.");
        return CallbackReturn::SUCCESS;
    }

    auto StopMotionNode::on_error(const rclcpp_lifecycle::State &) -> CallbackReturn
    {
        RCLCPP_ERROR(this->get_logger(), "error.");
        return CallbackReturn::SUCCESS;
    }

} // namespace motion_primitives
