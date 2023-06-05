// Copyright (c) 2018 Intel Corporation
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

#ifndef PLANSYS2_BT_ACTIONS__BTACTIONNODE_HPP_
#define PLANSYS2_BT_ACTIONS__BTACTIONNODE_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace plansys2 {

using namespace std::chrono_literals;  // NOLINT

/**
 * @brief BT动作节点类，继承自BT::ActionNodeBase
 * @tparam ActionT 动作类型
 * @details 该类用于实现BT动作节点的功能，继承自BT::ActionNodeBase，提供了动作节点的基本操作。
 */
template <class ActionT>
class BtActionNode : public BT::ActionNodeBase {
public:
  /**
   * @brief 构造函数
   * @param xml_tag_name 节点在XML中的标签名
   * @param action_name 动作名称
   * @param conf 节点配置
   * @details 通过给定的参数构造BtActionNode对象，并初始化成员变量。
   */
  BtActionNode(
      const std::string& xml_tag_name,
      const std::string& action_name,
      const BT::NodeConfiguration& conf)
      : BT::ActionNodeBase(xml_tag_name, conf), action_name_(action_name) {
    node_ = rclcpp::Node::make_shared(action_name_ + "bta");

    // Get the required items from the blackboard
    server_timeout_ = 5s;

    // Initialize the input and output messages
    goal_ = typename ActionT::Goal();
    result_ = typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult();

    std::string remapped_action_name;
    if (getInput("server_name", remapped_action_name)) {
      action_name_ = remapped_action_name;
    }

    // Give the derive class a chance to do any initialization
    RCLCPP_INFO(node_->get_logger(), "\"%s\" BtActionNode initialized", xml_tag_name.c_str());
  }

  BtActionNode() = delete;

  virtual ~BtActionNode() {}

  /**
   * @brief 创建一个动作服务器实例
   * @param action_name 动作名称
   * @return bool 是否成功创建动作客户端
   * @details 使用ROS节点创建BT动作的客户端，等待服务器响应并返回是否成功等待
   * 参数列表：
   * - action_name：动作名称
   */
  bool createActionClient(const std::string& action_name) {
    // Now that we have the ROS node to use, create the action client for this BT action
    // 创建BT动作客户端
    action_client_ = rclcpp_action::create_client<ActionT>(node_, action_name);

    // Make sure the server is actually there before continuing
    // 等待服务器响应
    RCLCPP_INFO(node_->get_logger(), "Waiting for \"%s\" action server", action_name.c_str());
    bool success_waiting = action_client_->wait_for_action_server(server_timeout_);

    if (!success_waiting) {
      // 如果等待超时则记录错误日志
      RCLCPP_ERROR(
          node_->get_logger(), "Timeout (%ld secs) waiting for \"%s\" action server",
          server_timeout_.count() * 1000, action_name.c_str());
    }

    return success_waiting;
  }

  /**
   * @brief 提供基本端口信息
   * @param addition 需要添加的端口信息
   * @return BT::PortsList 基本端口信息
   * @details
   * 任何接受参数的BtActionNode子类都必须提供providedPorts方法，并在其中调用providedBasicPorts。
   * providedBasicPorts函数提供了基本的输入端口信息，包括服务器名称和等待服务器响应的时间。
   * 参数列表：
   * - addition：需要添加的端口信息
   */
  static BT::PortsList providedBasicPorts(BT::PortsList addition) {
    BT::PortsList basic = {
        BT::InputPort<std::string>("server_name", "Action server name"),
        BT::InputPort<double>(
            "server_timeout", 5.0,
            "The amount of time to wait for a response from the action server, in seconds")};
    basic.insert(addition.begin(), addition.end());

    return basic;
  }

  /**
   * @brief 提供端口信息
   * @return BT::PortsList 端口信息
   * @details providedPorts函数提供了所有的输入端口信息，包括providedBasicPorts提供的基本端口信息。
   */
  static BT::PortsList providedPorts() { return providedBasicPorts({}); }

  /**
   * @brief 处理动作
   * @return BT::NodeStatus 节点状态
   * @details on_tick函数可以进行动态检查，例如获取黑板上的值更新变量goal_updated_以请求新目标。
   */
  virtual BT::NodeStatus on_tick() { return BT::NodeStatus::RUNNING; }

  /**
   * @brief 处理反馈信息
   * @param feedback 动作反馈信息
   * @details on_feedback函数提供了派生类记录反馈、更新目标或取消目标的机会。
   * 参数列表：
   * - feedback：动作反馈信息
   */
  virtual void on_feedback(const std::shared_ptr<const typename ActionT::Feedback> feedback) {
    (void)feedback;
  }

  /**
   * @brief 当行为成功完成时调用。派生类可以重写此方法，在黑板上放置一个值。
   * @param None
   * @details 返回BT::NodeStatus::SUCCESS表示节点状态为成功。
   */
  virtual BT::NodeStatus on_success() { return BT::NodeStatus::SUCCESS; }

  /**
   * @brief 当行为被中止时调用。默认情况下，节点将返回FAILURE。
   * @param None
   * @details 用户可以重写它以返回另一个值。
   */
  virtual BT::NodeStatus on_aborted() { return BT::NodeStatus::FAILURE; }

  /**
   * @brief 当行为被取消时调用。默认情况下，节点将返回SUCCESS。
   * @param None
   * @details 用户可以重写它以返回另一个值。
   */
  virtual BT::NodeStatus on_cancelled() { return BT::NodeStatus::SUCCESS; }

  /**
   * @brief BT行为树中一个动作节点的执行函数
   * @param None
   * @details
   * 该函数是BT行为树中一个动作节点的主要执行函数，包括了节点的初始化、循环执行以及结果判断等步骤。
   */
  BT::NodeStatus tick() override {
    // 如果当前状态为IDLE，则进行初始化操作
    if (status() == BT::NodeStatus::IDLE) {
      double server_timeout = 5.0;
      // 获取输入参数server_timeout，若未设置则使用默认值5s
      if (!getInput("server_timeout", server_timeout)) {
        RCLCPP_INFO(
            node_->get_logger(),
            "Missing input port [server_timeout], "
            "using default value of 5s");
      }
      server_timeout_ = std::chrono::milliseconds(static_cast<int>(server_timeout * 1000.0));

      // 创建Action客户端
      if (!createActionClient(action_name_)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create action client");
        return BT::NodeStatus::FAILURE;
      }

      // 用户自定义tick函数
      auto user_status = on_tick();
      if (user_status != BT::NodeStatus::RUNNING) {
        return user_status;
      }

      // 接收新的目标点
      if (!on_new_goal_received()) {
        return BT::NodeStatus::FAILURE;
      }

      return BT::NodeStatus::RUNNING;
    }

    // 进入RUNNING状态后的循环执行
    if (rclcpp::ok() && !goal_result_available_) {
      auto goal_status = goal_handle_->get_status();
      // 若接收到新的目标点，则调用on_new_goal_received()函数
      if (goal_updated_ && (goal_status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
                            goal_status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED)) {
        goal_updated_ = false;
        if (!on_new_goal_received()) {
          cancel_goal();
          return BT::NodeStatus::FAILURE;
        }
      }

      rclcpp::spin_some(node_);

      // 用户自定义tick函数
      auto user_status = on_tick();
      if (user_status != BT::NodeStatus::RUNNING) {
        cancel_goal();
        return user_status;
      }

      // 检查是否已经接收到结果
      if (!goal_result_available_) {
        // 返回RUNNING状态
        return BT::NodeStatus::RUNNING;
      }
    }

    // 根据结果进行相应处理
    switch (result_.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        return on_success();

      case rclcpp_action::ResultCode::ABORTED:
        return on_aborted();

      case rclcpp_action::ResultCode::CANCELED:
        return on_cancelled();

      default:
        throw std::logic_error("BtActionNode::Tick: invalid status value");
    }
  }

  /**
   * @brief BT 行为树中的另一种可选重载。在这种情况下，我们确保如果 ROS2 action 仍在运行，则取消它。
   * @param None
   * @details 如果应该取消目标，则取消目标，并将节点状态设置为 IDLE。
   */
  void halt() override {
    if (should_cancel_goal()) {  // 判断是否需要取消目标
      cancel_goal();             // 取消目标
    }

    setStatus(BT::NodeStatus::IDLE);  // 将节点状态设置为 IDLE
  }

protected:
  /**
   * @brief 取消当前的任务目标
   * @param 无
   * @details 使用async_cancel_goal函数异步取消当前的任务目标，如果取消失败则输出错误信息。
   */
  void cancel_goal() {
    auto future_cancel = action_client_->async_cancel_goal(goal_handle_);
    if (rclcpp::spin_until_future_complete(node_, future_cancel, server_timeout_) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(
          node_->get_logger(), "Failed to cancel action server for %s", action_name_.c_str());
    }
  }

  /**
   * @brief 判断是否需要取消当前的任务目标
   * @param 无
   * @details
   * 如果当前节点不处于运行状态，则返回false；否则检查当前任务目标的执行状态，如果仍在执行中，则返回true，否则返回false。
   * @return bool 是否需要取消当前的任务目标
   */
  bool should_cancel_goal() {
    // Shut the node down if it is currently running
    if (status() != BT::NodeStatus::RUNNING) {
      return false;
    }

    rclcpp::spin_some(node_);
    auto status = goal_handle_->get_status();

    // Check if the goal is still executing
    return status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
           status == action_msgs::msg::GoalStatus::STATUS_EXECUTING;
  }

  /**
   * @brief 当新的目标被接收时，发送目标给 action server
   * @param 无
   * @details
   * 1. 初始化 goal_result_available_ 为 false
   * 2. 创建 send_goal_options 对象，设置 result_callback 和 feedback_callback
   * 3. 调用 action_client_->async_send_goal 发送目标，并返回 future_goal_handle
   * 4. 使用 rclcpp::spin_until_future_complete 等待 future_goal_handle 完成
   * 5. 检查 goal_handle_ 是否为空，如果为空则说明目标被拒绝，返回 false，否则返回 true
   */
  bool on_new_goal_received() {
    goal_result_available_ = false;
    auto send_goal_options = typename rclcpp_action::Client<ActionT>::SendGoalOptions();
    send_goal_options.result_callback =
        [this](const typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult& result) {
          // TODO(#1652): a work around until rcl_action interface is updated
          // if goal ids are not matched, the older goal call this callback so ignore the result
          // if matched, it must be processed (including aborted)
          if (this->goal_handle_->get_goal_id() == result.goal_id) {
            goal_result_available_ = true;
            result_ = result;
          }
        };
    send_goal_options.feedback_callback =
        [this](
            typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr,
            const std::shared_ptr<const typename ActionT::Feedback> feedback) {
          on_feedback(feedback);
        };

    RCLCPP_INFO(node_->get_logger(), "Sending goal to action server %s", action_name_.c_str());
    auto future_goal_handle = action_client_->async_send_goal(goal_, send_goal_options);

    if (rclcpp::spin_until_future_complete(node_, future_goal_handle, server_timeout_) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(
          node_->get_logger(), "Failed to send goal to action server %s", action_name_.c_str());
      return false;
    }

    goal_handle_ = future_goal_handle.get();
    if (!goal_handle_) {
      RCLCPP_ERROR(
          node_->get_logger(), "Goal was rejected by action server %s", action_name_.c_str());
      return false;
    }

    return true;
  }

  /**
   * @brief 增加恢复次数
   * @param 无
   * @details
   * 1. 获取当前恢复次数 recovery_count
   * 2. 将 recovery_count 加 1
   * 3. 更新 blackboard 中的 number_recoveries
   */
  void increment_recovery_count() {
    int recovery_count = 0;
    config().blackboard->get<int>("number_recoveries", recovery_count);  // NOLINT
    recovery_count += 1;
    config().blackboard->set<int>("number_recoveries", recovery_count);  // NOLINT
  }

  /**
   * @brief 用于ros2_planning_system组件中的客户端，用于发送和接收action请求和响应
   * @param action_name_ 表示当前客户端要连接的action服务器的名称
   * @param action_client_ 表示当前客户端与action服务器之间的连接
   * @param goal_ 表示当前客户端要发送的goal
   * @param goal_updated_ 表示当前goal是否已经更新
   * @param goal_result_available_ 表示当前goal的结果是否可用
   * @param goal_handle_ 表示当前goal的句柄
   * @param result_ 表示当前goal的结果
   * @param node_ 表示当前客户端所在的ROS节点
   * @param server_timeout_ 表示等待服务器响应的超时时间
   * @details
   * 该代码段定义了一个用于ros2_planning_system组件中的客户端类，用于发送和接收action请求和响应。其中包括了客户端需要的各种属性和方法。
   */
  std::string action_name_;
  typename std::shared_ptr<rclcpp_action::Client<ActionT>> action_client_;

  // 所有ROS2 actions都有一个goal和一个result
  typename ActionT::Goal goal_;
  bool goal_updated_{false};
  bool goal_result_available_{false};
  typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr goal_handle_;
  typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult result_;

  // 用于进行任何ROS操作的节点
  rclcpp::Node::SharedPtr node_;

  // 发送或取消新action goal时等待服务器响应的超时值
  std::chrono::milliseconds server_timeout_;
};

}  // namespace plansys2

#endif  // PLANSYS2_BT_ACTIONS__BTACTIONNODE_HPP_
