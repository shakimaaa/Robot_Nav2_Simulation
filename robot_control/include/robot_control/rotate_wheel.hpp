#ifndef ROBOT_CONTROL__ROTATE_WHEEL_HPP_
#define ROBOT_CONTROL__ROTATE_WHEEL_HPP_

// ROS
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"

// STD
#include <thread>
#include <chrono>
#include <vector>



namespace robot_control
{

class RotateWheelNode : public rclcpp::Node
{

public:
  // 构造函数，初始化节点
  RotateWheelNode(const rclcpp::NodeOptions & options);


private:
  // 初始化关节状态
  void init_joint_states();
  // 更新速度
  void update_speed(const std::vector<double> & speeds);
  // 发布关节状态的线程函数
  void thread_pub();
  // 参数事件回调函数
  void on_parameter_event_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event);
  
  // 发布者对象
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_publisher_;
  
  // 参数事件订阅者
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_subscriber_;

  // 关节状态消息
  sensor_msgs::msg::JointState joint_states_;
  // 存储左右轮子的速度
  std::vector<double> joint_speeds_;
  // 发布频率控制器
  std::shared_ptr<rclcpp::Rate> pub_rate_;
  // 发布线程
  std::thread thread_;

};


}  // namespace robot_control



#endif  // ROBOT_CONTROL__ROTATE_WHEEL_HPP_