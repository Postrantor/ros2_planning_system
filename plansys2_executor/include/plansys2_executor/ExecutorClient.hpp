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

#ifndef PLANSYS2_EXECUTOR__EXECUTORCLIENT_HPP_
#define PLANSYS2_EXECUTOR__EXECUTORCLIENT_HPP_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "plansys2_msgs/action/execute_plan.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_msgs/msg/tree.hpp"
#include "plansys2_msgs/srv/get_ordered_sub_goals.hpp"
#include "plansys2_msgs/srv/get_plan.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace plansys2 {

/**
 * @brief ExecutorClient类用于执行计划并获取反馈和结果
 * @param node_name 节点名称
 * @details 该类提供了以下功能：
 *    1. 启动计划的执行
 *    2. 执行并检查计划
 *    3. 取消计划的执行
 *    4. 获取有序子目标列表
 *    5. 获取计划
 *    6. 获取反馈
 *    7. 获取结果
 */
class ExecutorClient {
public:
  using ExecutePlan = plansys2_msgs::action::ExecutePlan;
  using GoalHandleExecutePlan = rclcpp_action::ClientGoalHandle<ExecutePlan>;

  ExecutorClient();
  explicit ExecutorClient(const std::string& node_name);

  bool start_plan_execution(const plansys2_msgs::msg::Plan& plan);  // 启动计划的执行
  bool execute_and_check_plan();                                    // 执行并检查计划
  void cancel_plan_execution();                                     // 取消计划的执行
  std::vector<plansys2_msgs::msg::Tree> getOrderedSubGoals();       // 获取有序子目标列表
  std::optional<plansys2_msgs::msg::Plan> getPlan();                // 获取计划

  ExecutePlan::Feedback getFeedBack() { return feedback_; }         // 获取反馈
  std::optional<ExecutePlan::Result> getResult();                   // 获取结果

private:
  rclcpp::Node::SharedPtr node_;

  rclcpp_action::Client<ExecutePlan>::SharedPtr action_client_;
  rclcpp::Client<plansys2_msgs::srv::GetOrderedSubGoals>::SharedPtr get_ordered_sub_goals_client_;
  rclcpp::Client<plansys2_msgs::srv::GetPlan>::SharedPtr get_plan_client_;

  ExecutePlan::Feedback feedback_;  // 反馈
  rclcpp_action::ClientGoalHandle<ExecutePlan>::SharedPtr goal_handle_;
  rclcpp_action::ClientGoalHandle<ExecutePlan>::WrappedResult result_;

  bool goal_result_available_{false};

  bool executing_plan_{false};

  /**
   * @brief 处理执行计划的结果回调函数
   * @param result 执行计划的结果
   */
  void result_callback(const GoalHandleExecutePlan::WrappedResult& result);

  /**
   * @brief 处理执行计划的反馈回调函数
   * @param goal_handle 目标句柄
   * @param feedback 反馈信息
   */
  void feedback_callback(
      GoalHandleExecutePlan::SharedPtr goal_handle,
      const std::shared_ptr<const ExecutePlan::Feedback> feedback);

  /**
   * @brief 处理新目标的接收
   * @param plan 计划
   * @return bool 是否成功处理新目标
   */
  bool on_new_goal_received(const plansys2_msgs::msg::Plan& plan);

  /**
   * @brief 判断是否应该取消目标
   * @return bool 是否应该取消目标
   */
  bool should_cancel_goal();

  /**
   * @brief 创建动作客户端
   */
  void createActionClient();
};

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__EXECUTORCLIENT_HPP_
