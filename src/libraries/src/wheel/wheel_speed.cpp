#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class RotateWheelNode : public rclcpp::Node {
public:
  RotateWheelNode(const std::string &name) : Node(name) {
    RCLCPP_INFO(this->get_logger(), "Node %s initialized", name.c_str());

    // 创建并初始化发布者
    joint_states_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    // 初始化数据
    initJointStates();

    // 创建定时器
    pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&RotateWheelNode::publishJointStates, this));
  }

  void updateSpeed(const std::vector<double> &speeds) {
    joint_speeds_ = speeds;
  }

private:
  void initJointStates() {
    // 初始左右轮子的速度
    joint_speeds_ = {0.0, 0.0};

    joint_states_.header.stamp = this->now();
    joint_states_.header.frame_id = "";

    // 关节名称
    joint_states_.name = {"left_wheel_joint", "right_wheel_joint"};

    // 关节的位置
    joint_states_.position = {0.0, 0.0};

    // 关节速度
    joint_states_.velocity = joint_speeds_;

    // 力
    joint_states_.effort = {};
  }

  void publishJointStates() {
    // 更新时间差
    auto current_time = this->now();
    auto delta_time = (current_time - last_update_time_).seconds();
    last_update_time_ = current_time;

    // 更新位置
    joint_states_.position[0] += delta_time * joint_states_.velocity[0];
    joint_states_.position[1] += delta_time * joint_states_.velocity[1];

    // 更新速度
    joint_states_.velocity = joint_speeds_;

    // 更新时间戳
    joint_states_.header.stamp = current_time;

    // 发布关节数据
    joint_states_publisher_->publish(joint_states_);
  }

  std::vector<double> joint_speeds_;
  sensor_msgs::msg::JointState joint_states_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_publisher_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
  rclcpp::Time last_update_time_ = this->now();
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<RotateWheelNode>("rotate_wheel_node");
  node->updateSpeed({0.1, 10.0});  // 示例：更新速度
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}