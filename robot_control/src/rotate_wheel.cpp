# include "robot_control/rotate_wheel.hpp"
#include <sensor_msgs/msg/detail/joint_state__builder.hpp>
#include <sensor_msgs/msg/detail/joint_state__struct.hpp>
#include <vector>




namespace robot_control
{
RotateWheelNode::RotateWheelNode(const rclcpp::NodeOptions & options)
: Node("rotate_wheel", options)
{
  RCLCPP_INFO(this->get_logger(), "节点 rotate_wheel 初始化...");

  // 声明参数并设置默认值
  this->declare_parameter("left_front_wheel_speed", 0.0);
  this->declare_parameter("left_back_wheel_speed", 0.0);
  this->declare_parameter("right_front_wheel_speed", 0.0);
  this->declare_parameter("right_back_wheel_speed", 0.0);

  this->declare_parameter("left_front_wheel_angle", 0.0);
  this->declare_parameter("left_back_wheel_angle", 0.0);
  this->declare_parameter("right_front_wheel_angle", 0.0);
  this->declare_parameter("right_back_wheel_angle", 0.0);


  // 初始化参数
  auto left_front_wheel_speed = this->get_parameter("left_front_wheel_speed").as_double();
  auto left_back_wheel_speed = this->get_parameter("left_back_wheel_speed").as_double();
  auto right_front_wheel_speed = this->get_parameter("right_front_wheel_speed").as_double();
  auto right_back_wheel_speed = this->get_parameter("right_back_wheel_speed").as_double();
  joint_speeds_ = {left_front_wheel_speed, left_back_wheel_speed, right_front_wheel_speed, right_back_wheel_speed};
  

  auto left_front_wheel_angle = this->get_parameter("left_front_wheel_angle").as_double();
  auto left_back_wheel_angle = this->get_parameter("left_back_wheel_angle").as_double();
  auto right_front_wheel_angle = this->get_parameter("right_front_wheel_angle").as_double();
  auto right_back_wheel_angle = this->get_parameter("right_back_wheel_angle").as_double();
  joint_angles_ = {left_front_wheel_angle, left_back_wheel_angle, right_front_wheel_angle, right_back_wheel_angle};

  // 创建并初始化关节状态发布者
  joint_states_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

  // 初始化关节参数
  init_joint_states();

  // 设置发布频率为每秒30次
  pub_rate_ = std::make_shared<rclcpp::Rate>(30);

  // 启动发布线程
  thread_ = std::thread(&RotateWheelNode::thread_pub, this);

  // 订阅参数事件
  parameter_event_subscriber_ = this->create_subscription<rcl_interfaces::msg::ParameterEvent>(
        "/parameter_events", 10,
        [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) {
            this->on_parameter_event_callback(event);
        }
  );



  
}


/**
 * @brief 设置 joint_states_ 消息的初始状态，包括关节名称、位置、速度等信息
 * 
 * 函数的详细描述。
 *
 * @param joint_states_.header.frame_id  这个字段通常用于指明消息所关联的坐标框架（frame）。在这里，设置为空字符串意味着不指定任何特定的坐标框架。
 * 在某些应用中，你可能会指定一个坐标框架名称，例如 "base_link"，以便与机器人其他部分的消息一致。
 *   
 * @param joint_states_.name  这是一个字符串数组，用于指定各个关节的名称,这些名称应与机器人模型（如 URDF 文件）中定义的关节名称一致，以确保正确匹配关节数据。
 * 
 * @param joint_states_.position  用于存储各个关节的当前位置,在运行过程中，这些值会更新为每个关节的实际位置，例如轮子的旋转角度
 * 
 * @param joint_states_.velocity  用于存储各个关节的速度,这些值会更新为每个关节的实际速度，例如轮子的旋转速度。
 * 
 * @param joint_states_.effort  用于存储各个关节的作用力或力矩,如果需要监控或控制每个关节的力或力矩，可以在运行过程中更新这些值。
 * 
 */
void RotateWheelNode::init_joint_states()
{
  // 初始化关节状态消息的各个字段
  joint_states_.header.frame_id = ""; 
  joint_states_.name = {"left_front_wheel_joint", "left_back_wheel_joint", 
                        "right_front_wheel_joint", "right_back_wheel_joint",
                        "left_front_wheel_mount_joint", "left_back_wheel_mount_joint",
                        "right_front_wheel_mount_joint", "right_back_wheel_mount_joint"};
  joint_states_.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  joint_states_.velocity = joint_speeds_;
  joint_states_.effort = {};
}

void RotateWheelNode::update_param(const std::vector<double> & speeds,  const std::vector<double> & angles)
{
  if (speeds.size() == 4 && angles.size() == 4) {
    joint_speeds_ = speeds;
    joint_angles_ = angles;
  }
}

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
    joint_states_.position[2] += delta_time.count() * joint_states_.velocity[2];
    joint_states_.position[3] += delta_time.count() * joint_states_.velocity[3];
    joint_states_.position[4] = joint_angles_[0];
    joint_states_.position[5] = joint_angles_[1];
    joint_states_.position[6] = joint_angles_[2];
    joint_states_.position[7] = joint_angles_[3];
    // 更新关节速度
    joint_states_.velocity = joint_speeds_;  
    // 更新时间戳
    joint_states_.header.stamp = this->get_clock()->now(); 

     // 发布关节状态消息
    joint_states_publisher_->publish(joint_states_);

     // 控制发布频率
    pub_rate_->sleep();

  }
}

void RotateWheelNode::on_parameter_event_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
  // 处理新参数
  for (const auto &new_parameter : event->new_parameters) {
    if (new_parameter.name == "left_front_wheel_speed") {
      joint_speeds_[0] = new_parameter.value.double_value;
    } else if (new_parameter.name == "left_back_wheel_speed") {
      joint_speeds_[1] = new_parameter.value.double_value;
    } else if (new_parameter.name == "right_front_wheel_speed") {
      joint_speeds_[2] = new_parameter.value.double_value;
    } else if (new_parameter.name == "right_back_wheel_speed") {
      joint_speeds_[3] = new_parameter.value.double_value;
    } else if (new_parameter.name == "left_back_wheel_angle") {
      joint_angles_[1] = new_parameter.value.double_value;
    } else if (new_parameter.name == "right_front_wheel_angle") {
      joint_angles_[2] = new_parameter.value.double_value;
    } else if (new_parameter.name == "right_back_wheel_angle") {
      joint_angles_[3] = new_parameter.value.double_value;
    }

  }

    // 处理更改的参数
  for (const auto &changed_parameter : event->changed_parameters) {
    if (changed_parameter.name == "left_front_wheel_speed") {
      joint_speeds_[0] = changed_parameter.value.double_value;
    } else if (changed_parameter.name == "left_back_wheel_speed") {
      joint_speeds_[1] = changed_parameter.value.double_value;
    } else if (changed_parameter.name == "right_front_wheel_speed") {
      joint_speeds_[2] = changed_parameter.value.double_value;
    } else if (changed_parameter.name == "right_back_wheel_speed") {
      joint_speeds_[3] = changed_parameter.value.double_value;
    } else if (changed_parameter.name == "left_front_wheel_angle") {
      joint_angles_[0] = changed_parameter.value.double_value;
    } else if (changed_parameter.name == "left_back_wheel_angle") {
      joint_angles_[1] = changed_parameter.value.double_value;
    } else if (changed_parameter.name == "right_front_wheel_angle") {
      joint_angles_[2] = changed_parameter.value.double_value;
    } else if (changed_parameter.name == "right_back_wheel_angle") {
      joint_angles_[3] = changed_parameter.value.double_value;
    }
    
  }
  
  for (const auto &deleted_parameter : event->deleted_parameters) {
    if (deleted_parameter.name == "left_front_wheel_speed") {
      joint_speeds_[0] = 0.0;
    } else if (deleted_parameter.name == "left_back_wheel_speed") {
      joint_speeds_[1] = 0.0;
    } else if (deleted_parameter.name == "right_front_wheel_speed") {
      joint_speeds_[2] = 0.0;
    } else if (deleted_parameter.name == "right_back_wheel_speed") {
      joint_speeds_[3] = 0.0;
    } else if (deleted_parameter.name == "left_front_wheel_angle") {
      joint_angles_[0] = 0.0;
    } else if (deleted_parameter.name == "left_back_wheel_angle") {
      joint_angles_[1] = 0.0;
    } else if (deleted_parameter.name == "right_front_wheel_angle") {
      joint_angles_[2] = 0.0;
    } else if (deleted_parameter.name == "right_back_wheel_angle") {
      joint_angles_[3] = 0.0;
    }

  }
  
}



}  // namespace robot_control

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(robot_control::RotateWheelNode)