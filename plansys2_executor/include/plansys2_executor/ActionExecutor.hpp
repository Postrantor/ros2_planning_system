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

#ifndef PLANSYS2_EXECUTOR__ACTIONEXECUTOR_HPP_
#define PLANSYS2_EXECUTOR__ACTIONEXECUTOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "plansys2_msgs/msg/action_execution.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/durative_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace plansys2 {

/**
 * @brief ActionExecutor类用于执行动作，包括获取动作状态、取消动作等功能。
 * @param action 动作名称
 * @param node ros2生命周期节点
 * @details 包含以下功能：
 * 1. 使用枚举类型定义状态；
 * 2. 提供make_shared方法创建ActionExecutor对象；
 * 3.
 * 提供获取状态、取消动作、获取内部状态、获取动作名称、获取动作参数、获取开始时间、获取状态时间、获取反馈、获取完成度等方法；
 * 4. 提供回调函数、请求执行者、确认执行者、拒绝执行者、获取动作名称和获取动作参数等保护方法。
 */
class ActionExecutor {
public:
  enum Status { IDLE, DEALING, RUNNING, SUCCESS, FAILURE, CANCELLED };

  using Ptr = std::shared_ptr<ActionExecutor>;
  static Ptr make_shared(
      const std::string& action, rclcpp_lifecycle::LifecycleNode::SharedPtr node) {
    return std::make_shared<ActionExecutor>(action, node);
  }

  explicit ActionExecutor(
      const std::string& action, rclcpp_lifecycle::LifecycleNode::SharedPtr node);

  /**
   * @brief 执行动作，返回执行状态。
   * @param now 当前时间
   * @return BT::NodeStatus 枚举类型的执行状态
   */
  BT::NodeStatus tick(const rclcpp::Time& now);

  /**
   * @brief 取消当前正在执行的动作。
   */
  void cancel();

  /**
   * @brief 获取当前动作的执行状态。
   * @return BT::NodeStatus 枚举类型的执行状态
   */
  BT::NodeStatus get_status();

  /**
   * @brief 判断当前动作是否已经完成。
   * @return 如果已完成返回true，否则返回false
   */
  bool is_finished();

  // Methods for debug

  /**
   * @brief 获取内部状态。
   * @return Status 枚举类型的内部状态
   */
  Status get_internal_status() const { return state_; }

  /**
   * @brief 设置内部状态。
   * @param state Status枚举类型的内部状态
   */
  void set_internal_status(Status state) { state_ = state; }

  /**
   * @brief 获取动作名称。
   * @return 动作名称
   */
  std::string get_action_name() const { return action_name_; }

  /**
   * @brief 获取动作参数。
   * @return 动作参数数组
   */
  std::vector<std::string> get_action_params() const { return action_params_; }

  plansys2_msgs::msg::ActionExecution last_msg;

  /**
   * @brief 获取动作开始执行的时间。
   * @return 开始执行的时间
   */
  rclcpp::Time get_start_time() const { return start_execution_; }

  /**
   * @brief 获取动作状态更新的时间。
   * @return 状态更新的时间
   */
  rclcpp::Time get_status_time() const { return state_time_; }

  /**
   * @brief 获取动作反馈信息。
   * @return 反馈信息
   */
  std::string get_feedback() const { return feedback_; }

  /**
   * @brief 获取动作完成度。
   * @return 完成度
   */
  float get_completion() const { return completion_; }

protected:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

  Status state_;
  rclcpp::Time state_time_;
  rclcpp::Time start_execution_;

  std::string action_;
  std::string action_name_;
  std::string current_performer_id_;
  std::vector<std::string> action_params_;

  std::string feedback_;
  float completion_;

  rclcpp_lifecycle::LifecyclePublisher<plansys2_msgs::msg::ActionExecution>::SharedPtr
      action_hub_pub_;
  rclcpp::Subscription<plansys2_msgs::msg::ActionExecution>::SharedPtr action_hub_sub_;

  /**
   * @brief 动作执行状态更新的回调函数。
   * @param msg 动作执行状态消息
   */
  void action_hub_callback(const plansys2_msgs::msg::ActionExecution::SharedPtr msg);

  /**
   * @brief 请求执行者。
   */
  void request_for_performers();

  /**
   * @brief 确认执行者。
   * @param node_id 执行者节点ID
   */
  void confirm_performer(const std::string& node_id);

  /**
   * @brief 拒绝执行者。
   * @param node_id 执行者节点ID
   */
  void reject_performer(const std::string& node_id);

  /**
   * @brief 获取动作名称。
   * @param action_expr 动作表达式
   * @return 动作名称
   */
  std::string get_name(const std::string& action_expr);

  /**
   * @brief 获取动作参数。
   * @param action_expr 动作表达式
   * @return 动作参数数组
   */
  std::vector<std::string> get_params(const std::string& action_expr);

  /**
   * @brief 等待超时。
   */
  void wait_timeout();

  rclcpp::TimerBase::SharedPtr waiting_timer_;
};

/**
 * @brief ActionExecutionInfo结构体，用于存储动作执行的相关信息
 * @param action_executor 动作执行器的智能指针
 * @param at_start_effects_applied 是否已应用起始效果
 * @param at_end_effects_applied 是否已应用结束效果
 * @param durative_action_info 持续性动作信息的智能指针
 * @param execution_error_info 执行错误信息
 * @param duration 动作执行时间
 * @param duration_overrun_percentage 超时百分比
 *
 * @details
 * 存储导航规划器中动作执行的相关信息，包括动作执行器、是否应用起始/结束效果、持续性动作信息、执行错误信息、动作执行时间和超时百分比等。
 */
struct ActionExecutionInfo {
  std::shared_ptr<ActionExecutor> action_executor = {nullptr};
  bool at_start_effects_applied = {false};
  bool at_end_effects_applied = {false};
  std::shared_ptr<plansys2_msgs::msg::DurativeAction> durative_action_info = {nullptr};
  std::string execution_error_info;
  double duration;
  double duration_overrun_percentage = -1.0;
};

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__ACTIONEXECUTOR_HPP_
