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

#include "plansys2_executor/ExecutorNode.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @brief ros2_planning_system 组件的主函数
 * @param argc 命令行参数个数
 * @param argv 命令行参数列表
 * @details
 * 1. 初始化 ROS 节点
 * 2. 创建 plansys2::ExecutorNode 类型的智能指针 node，并将其初始化为 ExecutorNode 的实例
 * 3. 运行 ROS 节点，直到节点被关闭
 * 4. 关闭 ROS 节点
 * 5. 返回 0
 */
int main(int argc, char** argv) {
  // 初始化 ROS 节点
  rclcpp::init(argc, argv);
  // 创建 plansys2::ExecutorNode 类型的智能指针 node，并将其初始化为 ExecutorNode 的实例
  auto node = std::make_shared<plansys2::ExecutorNode>();
  // 运行 ROS 节点，直到节点被关闭
  rclcpp::spin(node->get_node_base_interface());
  // 关闭 ROS 节点
  rclcpp::shutdown();
  // 返回 0
  return 0;
}
