# RotateWheelNode: 控制机器人轮子速度的 ROS 2 节点

本仓库包含 `RotateWheelNode` 类的头文件和源文件，该类是一个 ROS 2 节点，用于控制机器人的轮子速度。

## 头文件：`rotate_wheel.hpp`

### 概述

`rotate_wheel.hpp` 文件声明了 `RotateWheelNode` 类。该类通过发布关节状态消息，以指定的频率控制机器人的轮子速度。

### 文件结构

```cpp
#ifndef ROBOT_CONTROL__ROTATE_WHEEL_HPP_
#define ROBOT_CONTROL__ROTATE_WHEEL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"

#include <thread>
#include <chrono>
#include <vector>

namespace robot_control {

class RotateWheelNode : public rclcpp::Node {
public:
  RotateWheelNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void init_joint_states();
  void update_speed(const std::vector<double> & speeds);
  void thread_pub();
  void on_parameter_event_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event);

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_publisher_;
  sensor_msgs::msg::JointState joint_states_;
  std::vector<double> joint_speeds_;
  std::shared_ptr<rclcpp::Rate> pub_rate_;
  std::thread thread_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_subscriber_;
};

}  // namespace robot_control

#endif  // ROBOT_CONTROL__ROTATE_WHEEL_HPP_
```

## 主要组件

- **类声明和成员变量:**
  - `RotateWheelNode` 继承自 `rclcpp::Node`，表示这是一个 ROS 2 节点。
  - `joint_states_publisher_` 是一个发布者，用于发布 `sensor_msgs::msg::JointState` 消息。
  - `joint_states_` 保存关节状态数据，包括位置和速度。
  - `joint_speeds_` 存储左右轮的速度。
  - `pub_rate_` 控制消息的发布频率。
  - `thread_` 是一个线程，用于定时发布关节状态消息。
  - `parameter_event_subscriber_` 订阅参数事件。

## 源文件：`rotate_wheel.cpp`

### 概述

`rotate_wheel.cpp` 文件实现了 `RotateWheelNode` 类的具体功能。

### 构造函数

```cpp
RotateWheelNode::RotateWheelNode(const rclcpp::NodeOptions &options)
    : Node("rotate_fishbot_wheel", options), joint_speeds_{0.0, 0.0} {
    RCLCPP_INFO(this->get_logger(), "节点 rotate_fishbot_wheel 初始化...");

    this->declare_parameter("left_wheel_speed", 0.0);
    this->declare_parameter("right_wheel_speed", 0.0);

    auto left_wheel_speed = this->get_parameter("left_wheel_speed").as_double();
    auto right_wheel_speed = this->get_parameter("right_wheel_speed").as_double();
    joint_speeds_ = {left_wheel_speed, right_wheel_speed};

    joint_states_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    init_joint_states();

    pub_rate_ = std::make_shared<rclcpp::Rate>(30);

    thread_ = std::thread(&RotateWheelNode::thread_pub, this);

    parameter_event_subscriber_ = this->create_subscription<rcl_interfaces::msg::ParameterEvent>(
        "/parameter_events", 10,
        std::bind(&RotateWheelNode::on_parameter_event_callback, this, std::placeholders::_1)
    );
}
```
## 主要功能

#### 参数声明和初始化

- 使用 `declare_parameter` 方法声明节点参数并设置默认值。
- 使用 `get_parameter` 方法获取参数初始值，并存储在 `joint_speeds_` 中。

#### 创建发布者

- 创建一个发布者，用于发布 `sensor_msgs::msg::JointState` 消息。

#### 初始化关节状态

- 使用 `init_joint_states` 方法初始化关节状态数据字段。

#### 启动发布线程

- 使用 `std::thread` 启动一个独立的线程，定时发布关节状态消息。

#### 订阅参数事件

- 订阅 `/parameter_events`，监听参数变化，并使用 `on_parameter_event_callback` 方法处理这些变化。




