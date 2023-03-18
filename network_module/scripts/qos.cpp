#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

// Constants
const int W = 100;  // Initial reliability parameter
int Vp = 0;         // Producer's sequence number
int Vs = 0;         // Subscriber's sequence number

// Update QoS policy with reliability parameter based on equation: w ± ⌈|(Vp − Vs)|/1000⌉
rclcpp::QoS update_qos_policy(int Vp, int Vs, rclcpp::QoS qos_profile) {
  int delta = std::abs(Vp - Vs) / 1000;
  int reliability = std::max(1, W + delta);  // Ensure reliability is at least 1

  qos_profile.history(rclcpp::QoSHistoryPolicy::KeepLast(10))
    .reliability(rclcpp::QoSReliabilityPolicy::Reliable(reliability))
    .durability(rclcpp::QoSDurabilityPolicy::TransientLocal())
    .deadline(rclcpp::Duration(500ms))
    .liveliness(rclcpp::QoSLivelinessPolicy::SystemDefault())
    .keep_last(10);

  return qos_profile;
}

// Subscriber callback function
void subscriber_callback(const std_msgs::msg::String::SharedPtr msg) {
  Vs = msg->data.size();  // Update subscriber's sequence number
  RCLCPP_INFO(rclcpp::get_logger("my_node"), "Received message: '%s', Vs = %d", msg->data.c_str(), Vs);
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  // Create a ROS2 node
  auto node = rclcpp::Node::make_shared("my_node");

  // Create a subscriber with the initial QoS profile
  auto qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile_sensor_data));
  auto sub = node->create_subscription<std_msgs::msg::String>(
    "my_topic",
    update_qos_policy(Vp, Vs, qos_profile),
    subscriber_callback);

  // Publish messages with increasing sequence numbers
  auto pub = node->create_publisher<std_msgs::msg::String>("my_topic", update_qos_policy(Vp, Vs, qos_profile));
  auto timer = node->create_wall_timer(100ms, [&]() {
    auto msg = std::make_unique<std_msgs::msg::String>();
    Vp += 1;
    msg->data = "message_" + std::to_string(Vp);
    pub->publish(std::move(msg));
  });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
