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

#include "plansys2_executor/ComputeBT.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @brief ros2_planning_system 组件的主函数
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @details 初始化 ROS2 节点，创建 ComputeBT
 * 对象并触发状态转换，使其进入激活状态，然后开始节点的循环运行，直到收到退出信号。
 */
int main(int argc, char** argv) {
  // 初始化 ROS2 节点
  rclcpp::init(argc, argv);
  // 创建 ComputeBT 对象
  auto node = std::make_shared<plansys2::ComputeBT>();

  // 触发状态转换，使 ComputeBT 进入配置状态
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  // 触发状态转换，使 ComputeBT 进入激活状态
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // 开始节点的循环运行，直到收到退出信号
  rclcpp::spin(node->get_node_base_interface());
  // 关闭 ROS2 节点
  rclcpp::shutdown();

  return 0;
}
