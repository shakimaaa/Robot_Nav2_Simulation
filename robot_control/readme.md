include/robot_control/rotate_wheel.hpp
这个头文件声明了 RotateWheelNode 类，它是一个 ROS 2 节点，负责控制机器人的轮子速度。


头文件的结构
#ifndef ROBOT_CONTROL__ROTATE_WHEEL_HPP_
#define ROBOT_CONTROL__ROTATE_WHEEL_HPP_

// 包含必要的 ROS 头文件
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"

// 包含标准库头文件
#include <thread>
#include <chrono>
#include <vector>

namespace robot_control
{

class RotateWheelNode : public rclcpp::Node
{
public:
  // 构造函数，初始化节点
  RotateWheelNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

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
  
  // 关节状态消息
  sensor_msgs::msg::JointState joint_states_;
  
  // 存储左右轮子的速度
  std::vector<double> joint_speeds_;
  
  // 发布频率控制器
  std::shared_ptr<rclcpp::Rate> pub_rate_;
  
  // 发布线程
  std::thread thread_;
  
  // 参数事件订阅者
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_subscriber_;
};

}  // namespace robot_control

#endif  // ROBOT_CONTROL__ROTATE_WHEEL_HPP_
主要功能解释
类声明和成员变量：
RotateWheelNode 继承自 rclcpp::Node，表示这是一个 ROS 2 节点。
joint_states_publisher_ 是一个发布者，用于发布 sensor_msgs::msg::JointState 消息。
joint_states_ 保存关节状态，包括位置、速度等信息。
joint_speeds_ 存储左右轮的速度。
pub_rate_ 用于控制消息的发布频率。
thread_ 是一个线程，用于定时发布关节状态消息。
parameter_event_subscriber_ 是一个订阅者，用于接收参数事件的通知。
src/rotate_wheel.cpp
这个源文件实现了 RotateWheelNode 类的具体功能。



构造函数
RotateWheelNode::RotateWheelNode(const rclcpp::NodeOptions &options)
    : Node("rotate_fishbot_wheel", options), joint_speeds_{0.0, 0.0}
{
    // 打印节点初始化信息
    RCLCPP_INFO(this->get_logger(), "节点 rotate_fishbot_wheel 初始化...");

    // 声明参数并设置默认值
    this->declare_parameter("left_wheel_speed", 0.0);
    this->declare_parameter("right_wheel_speed", 0.0);

    // 初始化参数
    auto left_wheel_speed = this->get_parameter("left_wheel_speed").as_double();
    auto right_wheel_speed = this->get_parameter("right_wheel_speed").as_double();
    joint_speeds_ = {left_wheel_speed, right_wheel_speed};

    // 创建并初始化关节状态发布者
    joint_states_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    // 初始化关节状态数据
    init_joint_states();

    // 设置发布频率为每秒30次
    pub_rate_ = std::make_shared<rclcpp::Rate>(30);

    // 启动发布线程
    thread_ = std::thread(&RotateWheelNode::thread_pub, this);

    // 订阅参数事件
    parameter_event_subscriber_ = this->create_subscription<rcl_interfaces::msg::ParameterEvent>(
        "/parameter_events", 10,
        std::bind(&RotateWheelNode::on_parameter_event_callback, this, std::placeholders::_1)
    );
}
主要功能解释

声明和初始化参数：
declare_parameter 方法用于声明节点的参数，并设置默认值。
使用 get_parameter 方法获取参数的初始值，并存储在 joint_speeds_ 向量中。

创建发布者：
joint_states_publisher_ 发布 sensor_msgs::msg::JointState 消息，用于发布轮子的速度和位置状态。

初始化关节状态：
init_joint_states 方法初始化关节状态消息的各个字段。

启动发布线程：
使用 std::thread 启动一个独立的线程，用于定时发布关节状态消息。

订阅参数事件：
parameter_event_subscriber_ 订阅 /parameter_events 话题，监听参数变化事件，并调用 on_parameter_event_callback 方法处理参数变化。



初始化关节状态
void RotateWheelNode::init_joint_states()
{
    // 初始化关节状态消息的各个字段
    joint_states_.header.frame_id = "";
    joint_states_.name = {"left_wheel_joint", "right_wheel_joint"};
    joint_states_.position = {0.0, 0.0};
    joint_states_.velocity = joint_speeds_;
    joint_states_.effort = {};
}
主要功能解释
该方法设置 joint_states_ 消息的初始状态，包括关节名称、位置、速度等信息。


发布关节状态的线程函数
void RotateWheelNode::thread_pub()
{
    auto last_update_time = std::chrono::steady_clock::now();
    while (rclcpp::ok())
    {
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> delta_time = now - last_update_time;
        last_update_time = now;

        // 更新关节位置
        joint_states_.position[0] += delta_time.count() * joint_states_.velocity[0];
        joint_states_.position[1] += delta_time.count() * joint_states_.velocity[1];

        // 更新关节速度
        joint_states_.velocity = joint_speeds_;

        // 更新消息的时间戳
        joint_states_.header.stamp = this->get_clock()->now();

        // 发布关节状态消息
        joint_states_publisher_->publish(joint_states_);

        // 控制发布频率
        pub_rate_->sleep();
    }
}
主要功能解释
该方法是一个独立的线程函数，不断更新关节位置和速度，并定时发布关节状态消息。


参数事件回调函数
void RotateWheelNode::on_parameter_event_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
    // 处理新参数
    for (const auto &new_parameter : event->new_parameters) {
        if (new_parameter.name == "left_wheel_speed") {
            joint_speeds_[0] = new_parameter.value.double_value;
        } else if (new_parameter.name == "right_wheel_speed") {
            joint_speeds_[1] = new_parameter.value.double_value;
        }
    }

    // 处理更改的参数
    for (const auto &changed_parameter : event->changed_parameters) {
        if (changed_parameter.name == "left_wheel_speed") {
            joint_speeds_[0] = changed_parameter.value.double_value;
        } else if (changed_parameter.name == "right_wheel_speed") {
            joint_speeds_[1] = changed_parameter.value.double_value;
        }
    }

    // 处理删除的参数
    for (const auto &deleted_parameter : event->deleted_parameters) {
        if (deleted_parameter.name == "left_wheel_speed") {
            joint_speeds_[0] = 0.0;
        } else if (deleted_parameter.name == "right_wheel_speed") {
            joint_speeds_[1] = 0.0;
        }
    }
}
主要功能解释
该方法处理参数事件，更新 joint_speeds_ 中的轮子速度。当参数被添加、修改或删除时，回调函数会相应地更新左右轮子的速度。
