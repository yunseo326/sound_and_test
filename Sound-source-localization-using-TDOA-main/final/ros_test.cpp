#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MasterOrder : public rclcpp::Node
{
public:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr order_pub_;
  MasterOrder() : Node("master_order")
  {
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    order_pub_ = this->create_publisher<std_msgs::msg::String>(
      "order",
      qos_profile
    );
  }

  void publish_order_msg(std::string order)
  {
    auto msg = std_msgs::msg::String();
    msg.data = order;
    RCLCPP_INFO(this->get_logger(), "Order publish");
    order_pub_->publish(msg);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto master_order = std::make_shared<MasterOrder>();
  std::string order;
  master_order->publish_order_msg(order);
  rclcpp::shutdown();
  return 0;
}