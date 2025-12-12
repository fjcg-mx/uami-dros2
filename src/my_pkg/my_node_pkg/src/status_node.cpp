#include <rclcpp/rclcpp.hpp>
#include <my_custom_msgs/msg/custom_status.hpp>

class StatusNode : public rclcpp::Node
{
public:
  StatusNode() : Node("status_node")
  {
    pub_ = this->create_publisher<my_custom_msgs::msg::CustomStatus>("status", 10);

    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      [this]() {
        my_custom_msgs::msg::CustomStatus msg;
        msg.header.stamp = this->get_clock()->now();
        msg.status = "OK";
        msg.level = 1;

        RCLCPP_INFO(this->get_logger(),
          "Publishing CustomStatus: %s | level=%d", msg.status.c_str(), msg.level);

        pub_->publish(msg);
      }
    );
  }

private:
  rclcpp::Publisher<my_custom_msgs::msg::CustomStatus>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StatusNode>());
  rclcpp::shutdown();
  return 0;
}
