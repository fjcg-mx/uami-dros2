#include <rclcpp/rclcpp.hpp>

#include <my_custom_msgs/msg/custom_status.hpp>
#include <my_custom_msgs/srv/set_mode.hpp>
#include <my_custom_msgs/action/navigate.hpp>

#include <geographic_msgs/msg/geo_pose_stamped.hpp>

#include <rclcpp_action/rclcpp_action.hpp>

class NavigationNode : public rclcpp::Node {
public:
  using Navigate = my_custom_msgs::action::Navigate;
  using GoalHandleNavigate = rclcpp_action::ServerGoalHandle<Navigate>;

  NavigationNode() : Node("navigation_node")
  {
    // Publicador
    status_pub_ = create_publisher<my_custom_msgs::msg::CustomStatus>("status", 10);

    // Servicio
    mode_srv_ = create_service<my_custom_msgs::srv::SetMode>(
      "set_mode",
      std::bind(&NavigationNode::setModeCallback, this,
                std::placeholders::_1, std::placeholders::_2));

    // GeoPose ejemplo
    geographic_msgs::msg::GeoPoseStamped pose;
    pose.position.latitude = 40.0;
    pose.position.longitude = -3.7;
    pose.position.altitude = 650;

    RCLCPP_INFO(get_logger(), "Loaded geographic pose %f, %f",
                pose.position.latitude, pose.position.longitude);

    // Acción
    action_server_ = rclcpp_action::create_server<Navigate>(
      this,
      "navigate",
      std::bind(&NavigationNode::goalCallback, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&NavigationNode::cancelCallback, this, std::placeholders::_1),
      std::bind(&NavigationNode::acceptCallback, this, std::placeholders::_1)
    );

    timer_ = create_wall_timer(1s, [&]() {
      my_custom_msgs::msg::CustomStatus msg;
      msg.header.stamp = now();
      msg.status = "OK";
      msg.level = 1;
      status_pub_->publish(msg);
    });
  }

private:
  // SERVICE
  void setModeCallback(
    const std::shared_ptr<my_custom_msgs::srv::SetMode::Request> req,
    std::shared_ptr<my_custom_msgs::srv::SetMode::Response> res)
  {
    RCLCPP_INFO(get_logger(), "Setting mode: %s", req->mode.c_str());
    res->success = true;
    res->message = "Mode updated";
  }

  // ACTION
  rclcpp_action::GoalResponse goalCallback(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Navigate::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Received goal.");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse cancelCallback(
    const std::shared_ptr<GoalHandleNavigate> goal_handle)
  {
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void acceptCallback(
    const std::shared_ptr<GoalHandleNavigate> goal_handle)
  {
    auto feedback = std::make_shared<Navigate::Feedback>();
    auto result = std::make_shared<Navigate::Result>();

    feedback->current_pose.pose.position.x = 0.0;
    feedback->progress = 0.5;

    goal_handle->publish_feedback(feedback);

    result->accepted = true;
    goal_handle->succeed(result);
  }

  rclcpp::Publisher<my_custom_msgs::msg::CustomStatus>::SharedPtr status_pub_;
  rclcpp::Service<my_custom_msgs::srv::SetMode>::SharedPtr mode_srv_;
  rclcpp_action::Server<Navigate>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr timer_;
};
