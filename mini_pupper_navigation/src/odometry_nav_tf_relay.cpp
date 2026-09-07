/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2026 Tony Kiegel
 *
 * Licensed under the Apache License, Version 2.0.
 */

#include <chrono>
#include <memory>
#include <mutex>
#include <utility>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

using namespace std::chrono_literals;

class OdometryNavTfRelay : public rclcpp::Node
{
public:
  OdometryNavTfRelay()
  : Node("guarded_navigation_tf_relay")
  {
    const auto input_qos =
      rclcpp::QoS(rclcpp::KeepLast(1))
      .best_effort()
      .durability_volatile();

    const auto output_qos =
      rclcpp::QoS(rclcpp::KeepLast(1))
      .reliable()
      .durability_volatile();

    publisher_ =
      create_publisher<tf2_msgs::msg::TFMessage>(
        "/nav_tf",
        output_qos);

    odom_subscription_ =
      create_subscription<nav_msgs::msg::Odometry>(
        "/odom",
        input_qos,
        [this](
          nav_msgs::msg::Odometry::ConstSharedPtr message)
        {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_odom_ = std::move(message);
        });

    local_odom_subscription_ =
      create_subscription<nav_msgs::msg::Odometry>(
        "/odom/local",
        input_qos,
        [this](
          nav_msgs::msg::Odometry::ConstSharedPtr message)
        {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_local_odom_ = std::move(message);
        });

    timer_ =
      create_wall_timer(
        100ms,
        [this]()
        {
          publish_latest();
        });

    RCLCPP_INFO(
      get_logger(),
      "Guarded navigation odometry TF relay: "
      "/odom + /odom/local -> /nav_tf at 10 Hz");
  }

private:
  static geometry_msgs::msg::TransformStamped convert(
    const nav_msgs::msg::Odometry & message)
  {
    geometry_msgs::msg::TransformStamped transform;

    transform.header = message.header;
    transform.child_frame_id = message.child_frame_id;

    transform.transform.translation.x =
      message.pose.pose.position.x;
    transform.transform.translation.y =
      message.pose.pose.position.y;
    transform.transform.translation.z =
      message.pose.pose.position.z;

    transform.transform.rotation =
      message.pose.pose.orientation;

    return transform;
  }

  void publish_latest()
  {
    nav_msgs::msg::Odometry::ConstSharedPtr odom;
    nav_msgs::msg::Odometry::ConstSharedPtr local_odom;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      odom = latest_odom_;
      local_odom = latest_local_odom_;
    }

    if (!odom || !local_odom) {
      return;
    }

    tf2_msgs::msg::TFMessage message;

    message.transforms.reserve(2);

    message.transforms.push_back(
      convert(*odom));

    message.transforms.push_back(
      convert(*local_odom));

    publisher_->publish(message);
  }

  std::mutex mutex_;

  nav_msgs::msg::Odometry::ConstSharedPtr
    latest_odom_;

  nav_msgs::msg::Odometry::ConstSharedPtr
    latest_local_odom_;

  rclcpp::Subscription<
    nav_msgs::msg::Odometry>::SharedPtr
    odom_subscription_;

  rclcpp::Subscription<
    nav_msgs::msg::Odometry>::SharedPtr
    local_odom_subscription_;

  rclcpp::Publisher<
    tf2_msgs::msg::TFMessage>::SharedPtr
    publisher_;

  rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<OdometryNavTfRelay>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
