#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

int main(int argc, char ** argv)
{
    // Inicializa ROS 2
    rclcpp::init(argc, argv);

    // Crea un nodo ROS 2
    auto node = rclcpp::Node::make_shared("test_rclcpp_tf2");

    // Crea un objeto Clock
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);

    // Crea un objeto tf2::Buffer con los parámetros necesarios
    tf2_ros::Buffer tf2_buffer(clock);

    // Crea el listener de transformaciones
    tf2_ros::TransformListener tf2_listener(tf2_buffer);

    // Log para verificar que todo está funcionando
    RCLCPP_INFO(node->get_logger(), "TF2 listener created");

    rclcpp::spin(node); // Mantén el nodo en ejecución

    rclcpp::shutdown();
    return 0;
}
