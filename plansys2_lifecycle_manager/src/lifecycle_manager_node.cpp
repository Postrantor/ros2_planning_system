// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <map>
#include <memory>
#include <string>

#include "plansys2_lifecycle_manager/lifecycle_manager.hpp"
#include "rclcpp/rclcpp.hpp"

// 这个文件是一个main函数的角色，是功能的入口？
// 要这么用吗？
// 这里给出的manager路径下的文件中确实是包含了具体的功能上的函数 start_function()

/**
 * @brief 主函数
 * @param argc 命令行参数个数
 * @param argv 命令行参数列表
 * @return 程序退出状态码
 */
int main(int argc, char** argv) {
  // 设置标准输出缓冲区，避免输出混乱
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // 初始化 ROS2 节点
  rclcpp::init(argc, argv);

  // 创建生命周期管理器节点的 map 容器
  // 这里为什么会有多个？为啥是一个manager对一个被管理的节点，这不浪费吗？
  std::map<std::string, std::shared_ptr<plansys2::LifecycleServiceClient>> manager_nodes;

  // 向 map 容器中添加领域专家、问题专家、规划器和执行器生命周期管理器节点
  manager_nodes["domain_expert"] =
      std::make_shared<plansys2::LifecycleServiceClient>("domain_expert_lc_mngr", "domain_expert");
  manager_nodes["problem_expert"] =  //
      std::make_shared<plansys2::LifecycleServiceClient>(
          "problem_expert_lc_mngr", "problem_expert");
  manager_nodes["planner"] =
      std::make_shared<plansys2::LifecycleServiceClient>("planner_lc_mngr", "planner");
  manager_nodes["executor"] =
      std::make_shared<plansys2::LifecycleServiceClient>("executor_lc_mngr", "executor");

  // 创建单线程执行器
  rclcpp::executors::SingleThreadedExecutor exe;

  // 遍历生命周期管理器节点 map 容器
  for (auto& manager_node : manager_nodes) {
    // 初始化生命周期管理器节点
    manager_node.second->init();
    // 将生命周期管理器节点添加到单线程执行器中
    exe.add_node(manager_node.second);
  }

  // 异步启动 plansys2，等待启动完成
  // startup_function() 专门就是为了其这个 std::map<> 的
  // 这样的话，在实现 lifecycle_manager 的程序时候，是否将这部分内容进行抽象，实现一个更通用的接口
  // 感觉对于用户的逻辑还是很有用的
  std::shared_future<bool> startup_future = std::async(
      std::launch::async,
      std::bind(plansys2::startup_function, manager_nodes, std::chrono::seconds(5)));
  exe.spin_until_future_complete(startup_future);

  // 如果 plansys2 启动失败，则输出错误信息并退出程序
  if (!startup_future.get()) {
    RCLCPP_ERROR(rclcpp::get_logger("plansys2_lifecycle_manager"), "Failed to start plansys2!");
    rclcpp::shutdown();
    return -1;
  }

  // 关闭 ROS2 节点
  rclcpp::shutdown();

  // 返回程序退出状态码
  return 0;
}
