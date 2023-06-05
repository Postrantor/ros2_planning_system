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

#include "plansys2_executor/ActionExecutor.hpp"

#include <memory>
#include <string>
#include <vector>

#include "plansys2_pddl_parser/Utils.h"

namespace plansys2 {

using std::placeholders::_1;
using namespace std::chrono_literals;

/**
 * @brief ActionExecutor类的构造函数，用于执行导航相关的动作
 * @param action 导航相关的动作名称
 * @param node 生命周期节点的指针
 * @details 创建发布器和订阅器，用于与其他组件通信；初始化状态、完成度等参数。
 */
ActionExecutor::ActionExecutor(
    const std::string& action, rclcpp_lifecycle::LifecycleNode::SharedPtr node)
    : node_(node), state_(IDLE), completion_(0.0) {
  // 创建发布器，用于向actions_hub话题发布plansys2_msgs::msg::ActionExecution消息
  action_hub_pub_ = node_->create_publisher<plansys2_msgs::msg::ActionExecution>(
      "actions_hub", rclcpp::QoS(100).reliable());
  // 创建订阅器，用于接收plansys2_msgs::msg::ActionExecution消息
  action_hub_sub_ = node_->create_subscription<plansys2_msgs::msg::ActionExecution>(
      "actions_hub", rclcpp::QoS(100).reliable(),
      std::bind(&ActionExecutor::action_hub_callback, this, _1));

  // 初始化状态时间
  state_time_ = node_->now();

  // 获取动作名称、参数以及开始执行的时间
  action_ = action;
  action_name_ = get_name(action);
  action_params_ = get_params(action);
  start_execution_ = node_->now();
  state_time_ = start_execution_;
}

// clang-format off
/**
 * @brief ActionExecutor类的action_hub_callback函数，处理ActionExecution消息
 * @param msg plansys2_msgs::msg::ActionExecution类型的指针，表示接收到的消息
 * @details 根据接收到的消息类型进行不同的处理：
 *          1. 对于REQUEST、CONFIRM、REJECT、CANCEL这四种类型的消息，无意义，直接break；
 *          2. 对于RESPONSE类型的消息，如果其arguments和action与当前执行的相同，则根据state_的状态进行不同的处理：
 *             a. 如果state_为DEALING，则确认执行者，并更新current_performer_id_、state_、waiting_timer_、start_execution_、state_time_等变量；
 *             b. 如果state_不为DEALING，则拒绝执行者；
 *          3. 对于FEEDBACK类型的消息，如果其arguments、action、node_id与当前执行的相同，则更新feedback_、completion_、state_time_等变量；
 *          4. 对于FINISH类型的消息，如果其arguments、action、node_id与当前执行的相同，则根据success的值更新state_的状态，同时更新feedback_、completion_、state_time_等变量，并将action_hub_pub_、action_hub_sub_设为nullptr。
 */
// clang-format on
void ActionExecutor::action_hub_callback(const plansys2_msgs::msg::ActionExecution::SharedPtr msg) {
  last_msg = *msg;

  switch (msg->type) {
    // 1. 对于REQUEST、CONFIRM、REJECT、CANCEL这四种类型的消息，无意义，直接break；
    case plansys2_msgs::msg::ActionExecution::REQUEST:
    case plansys2_msgs::msg::ActionExecution::CONFIRM:
    case plansys2_msgs::msg::ActionExecution::REJECT:
    case plansys2_msgs::msg::ActionExecution::CANCEL:
      // These cases have no meaning requester
      break;
    case plansys2_msgs::msg::ActionExecution::RESPONSE:
      if (msg->arguments == action_params_ && msg->action == action_name_) {
        if (state_ == DEALING) {
          confirm_performer(msg->node_id);
          current_performer_id_ = msg->node_id;
          state_ = RUNNING;
          waiting_timer_ = nullptr;
          start_execution_ = node_->now();
          state_time_ = node_->now();
        } else {
          reject_performer(msg->node_id);
        }
      }
      break;
    case plansys2_msgs::msg::ActionExecution::FEEDBACK:
      if (state_ != RUNNING || msg->arguments != action_params_ || msg->action != action_name_ ||
          msg->node_id != current_performer_id_) {
        return;
      }
      feedback_ = msg->status;
      completion_ = msg->completion;
      state_time_ = node_->now();

      break;
    case plansys2_msgs::msg::ActionExecution::FINISH:
      if (msg->arguments == action_params_ && msg->action == action_name_ &&
          msg->node_id == current_performer_id_) {
        if (msg->success) {
          state_ = SUCCESS;
        } else {
          state_ = FAILURE;
        }

        feedback_ = msg->status;
        completion_ = msg->completion;

        state_time_ = node_->now();

        action_hub_pub_->on_deactivate();
        action_hub_pub_ = nullptr;
        action_hub_sub_ = nullptr;
      }
      break;
    default:
      RCLCPP_ERROR(
          node_->get_logger(), "Msg %d type not recognized in %s executor requester", msg->type,
          action_.c_str());
      break;
  }
}

/**
 * @brief 发送确认执行的消息到 action_hub_pub_ 话题中
 * @param node_id 节点ID
 * @details 根据不同的状态，发送不同类型的消息到 action_hub_pub_ 话题中
 */
void ActionExecutor::confirm_performer(const std::string& node_id) {
  plansys2_msgs::msg::ActionExecution msg;
  msg.type = plansys2_msgs::msg::ActionExecution::CONFIRM;  // 确认执行
  msg.node_id = node_id;
  msg.action = action_name_;
  msg.arguments = action_params_;

  action_hub_pub_->publish(msg);
}

/**
 * @brief 发送拒绝执行的消息到 action_hub_pub_ 话题中
 * @param node_id 节点ID
 * @details 根据不同的状态，发送不同类型的消息到 action_hub_pub_ 话题中
 */
void ActionExecutor::reject_performer(const std::string& node_id) {
  plansys2_msgs::msg::ActionExecution msg;
  msg.type = plansys2_msgs::msg::ActionExecution::REJECT;  // 拒绝执行
  msg.node_id = node_id;
  msg.action = action_name_;
  msg.arguments = action_params_;

  action_hub_pub_->publish(msg);
}

/**
 * @brief 发送请求执行的消息到 action_hub_pub_ 话题中
 * @details 根据不同的状态，发送不同类型的消息到 action_hub_pub_ 话题中
 */
void ActionExecutor::request_for_performers() {
  plansys2_msgs::msg::ActionExecution msg;
  msg.type = plansys2_msgs::msg::ActionExecution::REQUEST;  // 请求执行
  msg.node_id = node_->get_name();
  msg.action = action_name_;
  msg.arguments = action_params_;

  action_hub_pub_->publish(msg);
}

/**
 * @brief 获取节点状态
 * @return BT::NodeStatus 节点状态
 * @details 根据不同的状态，返回不同的节点状态
 */
BT::NodeStatus ActionExecutor::get_status() {
  switch (state_) {
    case IDLE:
      return BT::NodeStatus::IDLE;
      break;
    case DEALING:
    case RUNNING:
      return BT::NodeStatus::RUNNING;
      break;
    case SUCCESS:
      return BT::NodeStatus::SUCCESS;
      break;
    case FAILURE:
      return BT::NodeStatus::FAILURE;
      break;
    default:
      return BT::NodeStatus::IDLE;
      break;
  }
}

/**
 * @brief 判断节点是否完成
 * @return bool 是否完成
 * @details 如果节点处于成功或失败状态，则认为已完成
 */
bool ActionExecutor::is_finished() { return state_ == SUCCESS || state_ == FAILURE; }

/**
 * @brief ActionExecutor组件的tick函数，用于执行某个动作。
 * @param now 当前时间
 * @details 根据当前状态执行相应的操作：
 * 1. IDLE状态：进入DEALING状态，记录状态时间，激活action_hub_pub_，初始化completion_和feedback_，
 *    请求可执行者列表，创建等待计时器waiting_timer_
 * 2. DEALING状态：如果请求超时（30s），则进入FAILURE状态
 * 3. RUNNING状态：无操作
 * 4. SUCCESS/FAILURE/CANCELLED状态：无操作
 * @return BT::NodeStatus 表示节点执行状态的枚举类型
 */
BT::NodeStatus ActionExecutor::tick(const rclcpp::Time& now) {
  switch (state_) {                    // 根据当前状态执行相应的操作
    case IDLE:                         // 如果处于IDLE状态
      state_ = DEALING;                // 进入DEALING状态
      state_time_ = node_->now();      // 记录状态时间

      action_hub_pub_->on_activate();  // 激活action_hub_pub_

      completion_ = 0.0;               // 初始化completion_
      feedback_ = "";                  // 初始化feedback_

      request_for_performers();        // 请求可执行者列表
      waiting_timer_ = node_->create_wall_timer(
          1s, std::bind(&ActionExecutor::wait_timeout, this));  // 创建等待计时器waiting_timer_
      break;
    case DEALING: {                                             // 如果处于DEALING状态
      auto time_since_dealing =
          (node_->now() - state_time_).seconds();  // 计算自进入DEALING状态以来的时间
      if (time_since_dealing > 30.0) {             // 如果请求超时（30s）
        RCLCPP_ERROR(
            node_->get_logger(), "Aborting %s. Timeout after requesting for 30 seconds",
            action_.c_str());  // 输出错误信息
        state_ = FAILURE;      // 进入FAILURE状态
      }
    } break;

    case RUNNING:    // 如果处于RUNNING状态
      break;         // 无操作
    case SUCCESS:    // 如果处于SUCCESS状态
    case FAILURE:    // 如果处于FAILURE状态
    case CANCELLED:  // 如果处于CANCELLED状态
      break;         // 无操作
    default:
      break;
  }

  return get_status();  // 返回节点执行状态的枚举类型
}

/**
 * @brief ActionExecutor 类中的 cancel 函数，用于取消当前正在执行的动作。
 * @details 将状态设置为 CANCELLED，并发布一个 plansys2_msgs::msg::ActionExecution 类型的消息，其中
 * type 为 CANCEL， node_id 为 current_performer_id_，action 为 action_name_，arguments 为
 * action_params_。
 * @param 无参数
 */
void ActionExecutor::cancel() {
  state_ = CANCELLED;
  plansys2_msgs::msg::ActionExecution msg;
  msg.type = plansys2_msgs::msg::ActionExecution::CANCEL;
  msg.node_id = current_performer_id_;
  msg.action = action_name_;
  msg.arguments = action_params_;

  action_hub_pub_->publish(msg);
}

/**
 * @brief ActionExecutor 类中的 get_name 函数，用于获取给定动作表达式的名称。
 * @details
 * 对给定的动作表达式进行处理，去除首尾括号后，提取出第一个空格之前的字符串作为动作名称返回。
 * @param action_expr 一个 std::string 类型的参数，表示待处理的动作表达式。
 * @return 返回一个 std::string 类型的值，表示提取出来的动作名称。
 */
std::string ActionExecutor::get_name(const std::string& action_expr) {
  std::string working_action_expr = parser::pddl::getReducedString(action_expr);
  working_action_expr.erase(0, 1);               // 去除首部的 (
  working_action_expr.pop_back();                // 去除尾部的 )

  size_t delim = working_action_expr.find(" ");  // 找到第一个空格的位置

  return working_action_expr.substr(0, delim);   // 返回第一个空格之前的字符串
}

/**
 * @brief 获取动作表达式中的参数列表
 * @param action_expr 动作表达式
 * @return 参数列表
 * @details
 * 1. 对传入的动作表达式进行处理，去除首尾括号
 * 2. 找到第一个空格，将其前面的内容删除
 * 3. 循环遍历字符串，找到每个参数，并将其添加到参数列表中
 * 4. 返回参数列表
 */
std::vector<std::string> ActionExecutor::get_params(const std::string& action_expr) {
  std::vector<std::string> ret;

  //  * 1. 对传入的动作表达式进行处理，去除首尾括号
  std::string working_action_expr = parser::pddl::getReducedString(action_expr);
  working_action_expr.erase(0, 1);  // remove initial (
  working_action_expr.pop_back();   // remove last )

  //  * 2. 找到第一个空格，将其前面的内容删除
  size_t delim = working_action_expr.find(" ");

  working_action_expr = working_action_expr.substr(delim + 1);

  //  * 3. 循环遍历字符串，找到每个参数，并将其添加到参数列表中
  size_t start = 0, end = 0;
  while (end != std::string::npos) {
    end = working_action_expr.find(" ", start);
    auto param = working_action_expr.substr(
        start, (end == std::string::npos) ? std::string::npos : end - start);
    ret.push_back(param);
    start = ((end > (std::string::npos - 1)) ? std::string::npos : end + 1);
  }

  return ret;
}

/**
 * @brief 等待超时
 * @details 如果没有执行者来执行该动作，则等待并重试
 */
void ActionExecutor::wait_timeout() {
  RCLCPP_WARN(node_->get_logger(), "No action performer for %s. retrying", action_.c_str());
  request_for_performers();
}

}  // namespace plansys2
