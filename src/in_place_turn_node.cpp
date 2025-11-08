// src/in_place_turn_node.cpp
#include "motion_primitives/in_place_turn_node.hpp"
#include <rclcpp/rclcpp.hpp>

namespace motion_primitives
{

  InPlaceTurnNode::InPlaceTurnNode()
      : rclcpp_lifecycle::LifecycleNode("in_place_turn_node") {}

  auto InPlaceTurnNode::on_configure(const rclcpp_lifecycle::State &) -> CallbackReturn
  {
    this->declare_parameter<double>("angular_speed", angular_speed_);
    this->get_parameter("angular_speed", angular_speed_);
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::QoS(10));
    RCLCPP_INFO(this->get_logger(), "configured. angular_speed=%.3f", angular_speed_);
    return CallbackReturn::SUCCESS;
  }

  auto InPlaceTurnNode::on_activate(const rclcpp_lifecycle::State &) -> CallbackReturn
  {
    cmd_pub_->on_activate();
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]()
        {
          geometry_msgs::msg::Twist cmd;
          cmd.linear.x = 0.0;
          cmd.angular.z = angular_speed_;
          cmd_pub_->publish(cmd);
        });
    RCLCPP_INFO(this->get_logger(), "activated.");
    return CallbackReturn::SUCCESS;
  }

  auto InPlaceTurnNode::on_deactivate(const rclcpp_lifecycle::State &) -> CallbackReturn
  {
    timer_.reset();
    cmd_pub_->on_deactivate();
    RCLCPP_INFO(this->get_logger(), "deactivated.");
    return CallbackReturn::SUCCESS;
  }

  auto InPlaceTurnNode::on_cleanup(const rclcpp_lifecycle::State &) -> CallbackReturn
  {
    timer_.reset();
    cmd_pub_.reset();
    RCLCPP_INFO(this->get_logger(), "cleaned up.");
    return CallbackReturn::SUCCESS;
  }

  auto InPlaceTurnNode::on_shutdown(const rclcpp_lifecycle::State &) -> CallbackReturn
  {
    RCLCPP_INFO(this->get_logger(), "shutdown.");
    return CallbackReturn::SUCCESS;
  }

  auto InPlaceTurnNode::on_error(const rclcpp_lifecycle::State &) -> CallbackReturn
  {
    RCLCPP_ERROR(this->get_logger(), "error.");
    return CallbackReturn::SUCCESS;
  }

} // namespace motion_primitives
