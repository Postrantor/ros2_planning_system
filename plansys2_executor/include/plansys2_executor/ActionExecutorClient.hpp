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

#ifndef PLANSYS2_EXECUTOR__ACTIONEXECUTORCLIENT_HPP_
#define PLANSYS2_EXECUTOR__ACTIONEXECUTORCLIENT_HPP_

#include <memory>
#include <string>
#include <vector>

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_msgs/msg/action_execution.hpp"
#include "plansys2_msgs/msg/action_performer_status.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

namespace plansys2 {

/**
 * @brief
 * ActionExecutorClient是一个继承自CascadeLifecycleNode的类，用于执行plansys2_msgs::msg::ActionExecution类型的消息。
 * @details
 * 该类包含了执行动作所需的各种方法和变量，例如获取参数、发送反馈、完成动作等。同时还实现了CascadeLifecycleNode的生命周期函数。
 *
 * 参数列表：
 * const std::string& node_name: 节点名称
 * const std::chrono::nanoseconds& rate: 执行频率
 */

class ActionExecutorClient : public rclcpp_cascade_lifecycle::CascadeLifecycleNode {
public:
  using Ptr = std::shared_ptr<ActionExecutorClient>;
  static Ptr make_shared(const std::string& node_name, const std::chrono::nanoseconds& rate) {
    return std::make_shared<ActionExecutorClient>(node_name, rate);
  }

  ActionExecutorClient(const std::string& node_name, const std::chrono::nanoseconds& rate);

  /**
   * @brief 获取内部状态
   * @return plansys2_msgs::msg::ActionPerformerStatus类型的状态信息
   */
  plansys2_msgs::msg::ActionPerformerStatus get_internal_status() const { return status_; }

protected:
  /**
   * @brief 执行具体的工作
   */
  virtual void do_work() {}

  /**
   * @brief 获取当前参数列表
   * @return 当前参数列表
   */
  const std::vector<std::string>& get_arguments() const { return current_arguments_; }

  /**
   * @brief 获取当前动作名称
   * @return 当前动作名称
   */
  const std::string get_action_name() const { return action_managed_; }

  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /**
   * @brief 生命周期函数，当节点被配置时调用
   * @param state 当前状态
   * @return 回调函数返回值
   */
  virtual CallbackReturnT on_configure(const rclcpp_lifecycle::State& state);

  /**
   * @brief 生命周期函数，当节点被激活时调用
   * @param state 当前状态
   * @return 回调函数返回值
   */
  virtual CallbackReturnT on_activate(const rclcpp_lifecycle::State& state);

  /**
   * @brief 生命周期函数，当节点被停用时调用
   * @param state 当前状态
   * @return 回调函数返回值
   */
  virtual CallbackReturnT on_deactivate(const rclcpp_lifecycle::State& state);

  /**
   * @brief 动作执行回调函数
   * @param msg plansys2_msgs::msg::ActionExecution类型的消息
   */
  void action_hub_callback(const plansys2_msgs::msg::ActionExecution::SharedPtr msg);

  /**
   * @brief 判断是否应该执行动作
   * @param action 动作名称
   * @param args 参数列表
   * @return 是否应该执行动作
   */
  bool should_execute(const std::string& action, const std::vector<std::string>& args);

  /**
   * @brief 发送反馈信息
   * @param completion 完成度
   * @param status 状态信息
   */
  void send_feedback(float completion, const std::string& status = "");

  /**
   * @brief 完成动作
   * @param success 是否成功
   * @param completion 完成度
   * @param status 状态信息
   */
  void finish(bool success, float completion, const std::string& status = "");

  std::chrono::nanoseconds rate_;                   // 执行频率
  std::string action_managed_;                      // 当前执行的动作名称
  bool commited_;                                   // 是否已提交

  std::vector<std::string> current_arguments_;      // 当前参数列表
  std::vector<std::string> specialized_arguments_;  // 特定参数列表

  rclcpp_lifecycle::LifecyclePublisher<plansys2_msgs::msg::ActionExecution>::SharedPtr
      action_hub_pub_;                  // 动作发布者
  rclcpp::Subscription<plansys2_msgs::msg::ActionExecution>::SharedPtr
      action_hub_sub_;                  // 动作订阅者
  rclcpp::TimerBase::SharedPtr timer_;  // 定时器

  rclcpp_lifecycle::LifecyclePublisher<plansys2_msgs::msg::ActionPerformerStatus>::SharedPtr
      status_pub_;                                    // 状态发布者
  rclcpp::TimerBase::SharedPtr hearbeat_pub_;         // 心跳定时器
  plansys2_msgs::msg::ActionPerformerStatus status_;  // 状态信息
};

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__ACTIONEXECUTORCLIENT_HPP_
