#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include “rcl/time.h”
#include “rclcpp/rclcpp.hpp”
#include “rcutils/cmdline_parser.h”
#include “std_msgs/msg/string.hpp”
using namespace std::chrono_literals;
class Navigate: public rclcpp::Node {
public: explicit Talker(const std::string & topic_name,
 const std::string & zlvarmsg,
 const std::int16_t & qos1,
 const std::size_t & qos2,
 const std::int16_t & qos3,
 const std::int16_t & zlcount): Node(“navigate”) {
 msg_ = std::make_shared < std_msgs::msg::String > ();
 // Create a function for when messages are to be sent.
 auto publish_message = [ & ]() - > void {
 auto now = std::chrono::high_resolution_clock::now();
 auto now_ns = std::chrono::time_point_cast < std::chrono::nanoseconds > (now);
 auto value = now_ns.time_since_epoch();
  long duration = value.count();
 msg_ - > data = “Hi!” + zlvarmsg + “ : “ + std::to_string(count_++) + “ % “ +
 std::to_string(duration);
 RCLCPP_INFO(this - > get_logger(), “Publishing: ‘%s’,” msg_ - > data.c_str());
 pub_ - > publish(msg_);
 if (count_ == zlcount) {
 rclcpp::shutdown();
 }
 };
 auto var1 = RMW_QOS_POLICY_HISTORY_KEEP_LAST;;
 auto var2 = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
 auto var3 = RMW_QOS_POLICY_DURABILITY_VOLATILE;
 if (qos1 == 2) {
 var1 = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
 }
 if (qos2 == 2) {
 var2 = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
 }
 if (qos3 == 2) {
 var3 = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
 }
 rmw_qos_profile_t custom_qos_profile = {
 var1,
 qos2,
 var2,
 var3,
 false
 };
 pub_ = this - > create_publisher < std_msgs::msg::String > (topic_name,
 custom_qos_profile);
 // Use a timer to schedule periodic message publishing.
 timer_ = this - > create_wall_timer(500 ms, publish_message);
}
private: size_t count_ = 1;
std::shared_ptr < std_msgs::msg::String > msg_;
rclcpp::Publisher < std_msgs::msg::String > ::SharedPtr pub_;
rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char * argv []) {
setvbuf(stdout, NULL, _IONBF, BUFSIZ);
if (rcutils_cli_option_exist(argv, argv + argc, “-h”)) {
 print_usage();
 return 0;
}
rclcpp::init(argc, argv);
// Parse the command line options.
auto topic = std::int16_t(“odom”);
char * cli_option = rcutils_cli_get_option(argv, argv + argc, “-t”);
if (nullptr != cli_option) {
 topic = std::string(cli_option);
}
auto msg_string = std::string(“ Hello World! “);
char * zl_string = rcutils_cli_get_option(argv, argv + argc, “-s”);
if (nullptr != zl_string) {
 msg_string = std::string(zl_string);
}
std::int16_t zqos1 = 1;
char * qos_choice1 = rcutils_cli_get_option(argv, argv + argc, “-q1”);
if (nullptr != qos_choice1) {
 zqos1 = std::atoi(qos_choice1);
}
std::size_t zqos2 = 10;
char * qos_choice2 = rcutils_cli_get_option(argv, argv + argc, “-q2”);
if (nullptr != qos_choice2) {
 zqos2 = std::atoi(qos_choice2);
}
std::int16_t zqos3 = 1;
char * qos_choice3 = rcutils_cli_get_option(argv, argv + argc, “-q3”);
if (nullptr != qos_choice3) {
 zqos3 = std::atoi(qos_choice3);
}
std::int16_t zzlcount = 15;
char * cou = rcutils_cli_get_option(argv, argv + argc, “-c”);
if (nullptr != cou) {
 zzlcount = std::atoi(cou);
}
auto node = std::make_shared < Navigate > (topic, msg_string, zqos1, zqos2, zqos3, zzlcount);
rclcpp::spin(node);
rclcpp::shutdown();
return 0;
}
