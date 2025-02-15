#pragma once
#include <rclcpp/node.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <sensor_msgs/msg/image.hpp>


class ImageSubscriber : public rclcpp::Node
{
public:
  ImageSubscriber(const rclcpp::NodeOptions &options,
                  const std::string &camera_node_name)
      : Node("image_subscriber", options),
        topic("/" + camera_node_name + "/image_raw")
  {
  }

  bool
  wait()
  {
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img =
      this->create_subscription<sensor_msgs::msg::Image>(topic, 1, [](const sensor_msgs::msg::Image &) {});

    sensor_msgs::msg::Image msg;
    return rclcpp::wait_for_message(msg, sub_img, this->get_node_options().context());
  }

private:
  const std::string topic;
};
