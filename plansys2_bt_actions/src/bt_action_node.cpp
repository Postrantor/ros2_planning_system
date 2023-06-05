// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <string>

#include "plansys2_bt_actions/BTAction.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

/**
 * @brief ROS2规划系统中执行BTAction的主函数
 * @param argc 命令行参数个数
 * @param argv 命令行参数列表
 * @details
 * 该函数初始化ROS2节点，创建BTAction对象并设置执行周期，触发BTAction的配置过渡，运行ROS2节点，最后关闭ROS2节点并返回程序退出状态码。
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);  // 初始化ROS2节点

  auto action_node = std::make_shared<plansys2::BTAction>(
      "default",  // BTAction的名称
      200ms       // 执行BTAction的周期，单位为毫秒
  );

  action_node->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);  // 触发BTAction的配置过渡
  rclcpp::spin(action_node->get_node_base_interface());        // 运行ROS2节点
  rclcpp::shutdown();                                          // 关闭ROS2节点

  return 0;                                                    // 返回程序退出状态码
}