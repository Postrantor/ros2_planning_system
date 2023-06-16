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

#include "plansys2_executor/ExecutorClient.hpp"

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "plansys2_msgs/msg/action_execution_info.hpp"

namespace plansys2 {

using namespace std::chrono_literals;
using namespace std::placeholders;

using ExecutePlan = plansys2_msgs::action::ExecutePlan;

/*
  ExecutorClient类的构造函数初始化ROS节点、创建动作客户端和服务客户端；
  createActionClient函数创建动作客户端，并等待动作服务器的启动；
  start_plan_execution函数开始执行计划，如果当前没有正在执行的计划，则创建动作客户端并处理新的目标。
*/

/**
 * @brief ExecutorClient类的构造函数，初始化ROS节点、创建动作客户端和服务客户端。
 * @details
 */
ExecutorClient::ExecutorClient() {
  node_ = rclcpp::Node::make_shared("executor_client");  // 创建ROS节点

  createActionClient();                                  // 创建动作客户端

  // 创建获取有序子目标的服务客户端
  get_ordered_sub_goals_client_ = node_->create_client<plansys2_msgs::srv::GetOrderedSubGoals>(
      "executor/get_ordered_sub_goals");
  // 创建获取计划的服务客户端
  get_plan_client_ = node_->create_client<plansys2_msgs::srv::GetPlan>("executor/get_plan");
}

/**
 * @brief ExecutorClient类的构造函数，初始化ROS节点、创建动作客户端和服务客户端，并指定节点名称。
 */
ExecutorClient::ExecutorClient(const std::string& node_name) {
  node_ = rclcpp::Node::make_shared(node_name);  // 创建ROS节点

  createActionClient();                          // 创建动作客户端

  // 创建获取有序子目标的服务客户端
  get_ordered_sub_goals_client_ = node_->create_client<plansys2_msgs::srv::GetOrderedSubGoals>(
      "executor/get_ordered_sub_goals");
  // 创建获取计划的服务客户端
  get_plan_client_ = node_->create_client<plansys2_msgs::srv::GetPlan>("executor/get_plan");
}

/**
 * @brief 创建动作客户端。
 */
void ExecutorClient::createActionClient() {
  // 创建动作客户端
  action_client_ = rclcpp_action::create_client<ExecutePlan>(node_, "execute_plan");

  // 等待动作服务器
  if (!this->action_client_->wait_for_action_server(3s)) {
    RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
  }
}

/**
 * @brief 开始执行计划。
 * @param plan: 待执行的计划。
 * @return bool: 表示是否成功开始执行计划。
 */
bool ExecutorClient::start_plan_execution(const plansys2_msgs::msg::Plan& plan) {
  // 如果当前没有正在执行的计划
  if (!executing_plan_) {
    createActionClient();                       // 创建动作客户端
    auto success = on_new_goal_received(plan);  // 处理新的目标

    // 如果处理成功，标记为正在执行计划
    if (success) {
      executing_plan_ = true;
      return true;
    }
  } else {
    RCLCPP_INFO(node_->get_logger(), "Already executing a plan");
  }

  return false;
}

/**
 * @brief 执行并检查计划是否完成
 * @param 无
 * @details
 *    1. 如果ROS2节点处于运行状态且目标结果不可用，则调用spin_some()函数；
 *    2. 如果目标结果仍然不可用，则返回true，表示计划未完成；
 *    3. 如果目标结果可用，则根据执行结果进行相应的处理。
 *    4. 如果执行成功，则输出“Plan Succeeded”；
 *    5. 如果执行失败，则输出“Plan Failed”，并输出每个动作的执行状态信息；
 *    6. 如果计划被中止，则输出“Plan Aborted”；
 *    7. 如果计划被取消，则输出“Plan Cancelled”；
 *    8. 最后将执行计划的标志位和目标结果可用的标志位均设为false，并返回false，表示计划已完成。
 */
bool ExecutorClient::execute_and_check_plan() {
  // 如果ROS2节点处于运行状态且目标结果不可用，则调用spin_some()函数；
  if (rclcpp::ok() && !goal_result_available_) {
    rclcpp::spin_some(node_);

    // 如果目标结果仍然不可用，则返回true，表示计划未完成；
    if (!goal_result_available_) {
      return true;  // Plan not finished
    }
  }

  switch (result_.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      // 如果执行成功，则输出“Plan Succeeded”；
      if (result_.result == nullptr) {
        RCLCPP_WARN(node_->get_logger(), "Plan failed due to a nullptr in the result");
      } else if (result_.result->success) {
        RCLCPP_INFO(node_->get_logger(), "Plan Succeeded");
      } else {
        // 如果执行失败，则输出“Plan Failed”，并输出每个动作的执行状态信息；
        RCLCPP_ERROR(node_->get_logger(), "Plan Failed");
        for (auto msg : result_.result->action_execution_status) {
          switch (msg.status) {
            case plansys2_msgs::msg::ActionExecutionInfo::SUCCEEDED:
              RCLCPP_WARN_STREAM(
                  node_->get_logger(),
                  "Action: " << msg.action_full_name
                             << " succeeded with message_status: " << msg.message_status);
              break;
            case plansys2_msgs::msg::ActionExecutionInfo::FAILED:
              RCLCPP_ERROR_STREAM(
                  node_->get_logger(),
                  "Action: " << msg.action_full_name
                             << " failed with message_status: " << msg.message_status);
              break;
            case plansys2_msgs::msg::ActionExecutionInfo::NOT_EXECUTED:
              RCLCPP_WARN_STREAM(
                  node_->get_logger(), "Action: " << msg.action_full_name << " was not executed");
              break;
            case plansys2_msgs::msg::ActionExecutionInfo::CANCELLED:
              RCLCPP_WARN_STREAM(
                  node_->get_logger(), "Action: " << msg.action_full_name << " was cancelled");
              break;
            case plansys2_msgs::msg::ActionExecutionInfo::EXECUTING:
              RCLCPP_WARN_STREAM(
                  node_->get_logger(), "Action: " << msg.action_full_name << " was executing");
          }
        }
      }
      break;

    case rclcpp_action::ResultCode::ABORTED:
      // 如果计划被中止，则输出“Plan Aborted”；
      RCLCPP_WARN(node_->get_logger(), "Plan Aborted");
      break;

    case rclcpp_action::ResultCode::CANCELED:
      // 如果计划被取消，则输出“Plan Cancelled”；
      RCLCPP_INFO(node_->get_logger(), "Plan Cancelled");
      break;

    default:
      throw std::logic_error("ExecutorClient::executePlan: invalid status value");
  }

  // 最后将执行计划的标志位和目标结果可用的标志位均设为false，并返回false，表示计划已完成。
  executing_plan_ = false;
  goal_result_available_ = false;

  return false;  // Plan finished
}

/**
 * @brief 接收新的计划并执行
 * @param plan 计划
 * @return bool 执行是否成功
 * @details
 *    1. 将接收到的计划存储在goal中
 *    2. 设置发送目标选项，包括反馈回调和结果回调
 *    3. 异步发送目标，并等待3秒钟以获取结果
 *    4. 检查目标句柄是否为空，如果为空则返回false
 *    5. 返回true
 */
bool ExecutorClient::on_new_goal_received(const plansys2_msgs::msg::Plan& plan) {
  auto goal = ExecutePlan::Goal();
  goal.plan = plan;

  auto send_goal_options = rclcpp_action::Client<ExecutePlan>::SendGoalOptions();
  send_goal_options.feedback_callback = std::bind(&ExecutorClient::feedback_callback, this, _1, _2);
  send_goal_options.result_callback = std::bind(&ExecutorClient::result_callback, this, _1);
  auto future_goal_handle = action_client_->async_send_goal(goal, send_goal_options);

  if (rclcpp::spin_until_future_complete(
          node_->get_node_base_interface(), future_goal_handle, 3s) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "send_goal failed");
    return false;
  }

  goal_handle_ = future_goal_handle.get();
  if (!goal_handle_) {
    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by the action server");
    return false;
  }

  return true;
}

/**
 * @brief 判断是否应该取消当前计划的执行
 * @return bool 是否应该取消
 * @details
 *    1. 如果当前没有正在执行的计划，则返回false
 *    2. 调用spin_some以处理所有待处理的回调
 *    3. 获取目标句柄的状态
 *    4. 如果状态为STATUS_ACCEPTED或STATUS_EXECUTING，则返回true，否则返回false
 */
bool ExecutorClient::should_cancel_goal() {
  if (!executing_plan_) {
    return false;
  }

  rclcpp::spin_some(node_);
  auto status = goal_handle_->get_status();

  return status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
         status == action_msgs::msg::GoalStatus::STATUS_EXECUTING;
}

/**
 * @brief 取消计划的执行
 * @details
 * 1. 如果应该取消当前计划，则异步取消目标，并等待3秒钟以获取结果
 * 2. 将executing_plan_和goal_result_available_设置为false
 */
void ExecutorClient::cancel_plan_execution() {
  if (should_cancel_goal()) {
    auto future_cancel = action_client_->async_cancel_goal(goal_handle_);
    if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(), future_cancel, 3s) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to cancel action server for execute_plan");
    }
  }

  executing_plan_ = false;
  goal_result_available_ = false;
}

/**
 * @brief 获取有序子目标的函数
 * @param 无参数
 * @details
 *    1. 创建一个 plansys2_msgs::msg::Tree 类型的 vector 容器 ret。
 *    2. 当 get_ordered_sub_goals_client_
 *    没有等待到服务时，每隔5秒尝试一次，如果超过5秒还没有等到服务，则返回容器ret。
 *    3. 如果 rclcpp::ok() 返回 false，则返回容器ret。
 *    4. 如果成功等到了服务，则创建一个 plansys2_msgs::srv::GetOrderedSubGoals::Request
 * 类型的智能指针 request。
 *    5. 使用 get_ordered_sub_goals_client_ 的 async_send_request 函数异步发送请求，并将返回值存储在
 *    future_result 中。
 *    6. 如果在1秒内没有得到返回值，则返回容器ret。
 *    7. 将 future_result 解引用并存储在 result 中。
 *    8. 如果请求成功，则将 sub_goals 存储在 ret 中。
 *    9. 如果请求失败，则在日志中输出错误信息。
 *    10. 返回容器ret。
 */
std::vector<plansys2_msgs::msg::Tree> ExecutorClient::getOrderedSubGoals() {
  std::vector<plansys2_msgs::msg::Tree> ret;

  while (!get_ordered_sub_goals_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return ret;
    }
    RCLCPP_ERROR_STREAM(
        node_->get_logger(), get_ordered_sub_goals_client_->get_service_name()
                                 << " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetOrderedSubGoals::Request>();
  auto future_result = get_ordered_sub_goals_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    return ret;
  }

  auto result = *future_result.get();

  if (result.success) {
    ret = result.sub_goals;
  } else {
    RCLCPP_INFO_STREAM(
        node_->get_logger(), get_ordered_sub_goals_client_->get_service_name()
                                 << ": " << result.error_info);
  }

  return ret;
}

/**
 * @brief 获取计划
 * @details
 *    1. 等待获取计划的服务出现，如果超时则返回空。
 *    2. 创建一个请求对象，并异步发送给获取计划的服务。
 *    3. 等待获取计划的服务返回结果，如果超时则返回空。
 *    4. 如果获取计划成功，则返回计划；否则返回空。
 * @return std::optional<plansys2_msgs::msg::Plan> 计划或空
 */
std::optional<plansys2_msgs::msg::Plan> ExecutorClient::getPlan() {
  while (!get_plan_client_->wait_for_service(
      std::chrono::seconds(5))) {  // 等待获取计划的服务出现，如果超时则返回空
    if (!rclcpp::ok()) {
      return {};
    }
    RCLCPP_ERROR_STREAM(
        node_->get_logger(), get_plan_client_->get_service_name()
                                 << " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetPlan::Request>();  // 创建一个请求对象
  auto future_result = get_plan_client_->async_send_request(request);       // 异步发送请求

  // 等待获取计划的服务返回结果，如果超时则返回空
  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    return {};
  }

  auto result = *future_result.get();  // 获取异步发送请求后的结果
  if (result.success) {                // 如果获取计划成功，则返回计划
    return result.plan;
  } else {                             // 否则返回空
    RCLCPP_ERROR_STREAM(
        node_->get_logger(), get_plan_client_->get_service_name() << ": " << result.error_info);
    return {};
  }
}

/**
 * @brief 执行计划的反馈回调函数
 * @param goal_handle 目标句柄
 * @param feedback 反馈信息
 * @details 将反馈信息存储到成员变量 feedback_ 中。
 */
void ExecutorClient::feedback_callback(
    GoalHandleExecutePlan::SharedPtr goal_handle,
    const std::shared_ptr<const ExecutePlan::Feedback> feedback) {
  feedback_ = *feedback;  // 将反馈信息存储到成员变量 feedback_ 中
}

/**
 * @brief 执行计划的结果回调函数
 * @param result 结果
 * @details
 * 1. 将结果存储到成员变量 result_ 中。
 * 2. 将目标结果可用标志位设置为 true。
 * 3. 将成员变量 feedback_ 清空。
 */
void ExecutorClient::result_callback(const GoalHandleExecutePlan::WrappedResult& result) {
  goal_result_available_ = true;        // 将目标结果可用标志位设置为 true
  result_ = result;                     // 将结果存储到成员变量 result_ 中
  feedback_ = ExecutePlan::Feedback();  // 将成员变量 feedback_ 清空
}

/**
 * @brief 获取执行计划的结果
 * @param 无
 * @details 如果结果存在，则返回结果；否则返回空。
 * @return std::optional<ExecutePlan::Result> 执行计划的结果或空
 */
std::optional<ExecutePlan::Result> ExecutorClient::getResult() {
  if (result_.result != nullptr) {  // 如果结果存在，则返回结果
    return *result_.result;
  } else {                          // 否则返回空
    return {};
  }
}

}  // namespace plansys2
