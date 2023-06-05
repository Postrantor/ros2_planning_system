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

#include "plansys2_executor/ActionExecutorClient.hpp"

#include <memory>
#include <string>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"

namespace plansys2 {

using namespace std::chrono_literals;

/**
 * @brief ActionExecutorClient类的构造函数，继承CascadeLifecycleNode类
 * @param node_name 节点名称
 * @param rate 执行频率
 * @details
 * 1. 声明并初始化成员变量rate_和commited_
 * 2. 声明参数action_name和specialized_arguments，并设置默认值为空字符串和空vector
 * 3. 根据rate_计算出默认频率default_rate，并声明参数rate，并设置默认值为default_rate
 * 4. 初始化成员变量status_，将状态设置为NOT_READY，并记录时间戳和节点名称
 */
ActionExecutorClient::ActionExecutorClient(
    const std::string& node_name, const std::chrono::nanoseconds& rate)
    : CascadeLifecycleNode(node_name), rate_(rate), commited_(false) {
  declare_parameter<std::string>("action_name", "");
  declare_parameter<std::vector<std::string>>(
      "specialized_arguments", std::vector<std::string>({}));

  double default_rate = 1.0 / std::chrono::duration<double>(rate_).count();
  declare_parameter<double>("rate", default_rate);
  status_.state = plansys2_msgs::msg::ActionPerformerStatus::NOT_READY;
  status_.status_stamp = now();
  status_.node_name = get_name();
}

/**
 * @brief ActionExecutorClient组件的on_configure回调函数
 * @param state 生命周期状态
 *
 * on_configure回调函数：
 * 1. 创建发布者status_pub_，发布plansys2_msgs::msg::ActionPerformerStatus类型的消息；
 * 2. 激活status_pub_；
 * 3. 创建周期性定时器hearbeat_pub_，每秒触发一次，发布status_；
 * 4. 获取action_name参数，如果获取失败则设置status_为FAILURE；
 * 5. 获取specialized_arguments参数；
 * 6. 获取rate参数，计算出rate_；
 * 7. 创建发布者action_hub_pub_，发布plansys2_msgs::msg::ActionExecution类型的消息；
 * 8.
 * 创建订阅者action_hub_sub_，接收plansys2_msgs::msg::ActionExecution类型的消息，并绑定到回调函数action_hub_callback；
 * 9. 激活action_hub_pub_；
 * 10. 设置status_的各个字段，返回CallbackReturnT::SUCCESS。
 */
using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using std::placeholders::_1;

CallbackReturnT ActionExecutorClient::on_configure(const rclcpp_lifecycle::State& state) {
  // 创建发布者status_pub_
  status_pub_ = create_publisher<plansys2_msgs::msg::ActionPerformerStatus>(
      "performers_status", rclcpp::QoS(100).reliable());
  // 激活status_pub_
  status_pub_->on_activate();

  // 创建周期性定时器hearbeat_pub_，每秒触发一次，发布status_
  hearbeat_pub_ = create_wall_timer(1s, [this]() {
    status_.status_stamp = now();
    status_pub_->publish(status_);
  });

  // 获取action_name参数，如果获取失败则设置status_为FAILURE
  if (!get_parameter("action_name", action_managed_)) {
    RCLCPP_ERROR(get_logger(), "action_name parameter not set");
    status_.state = plansys2_msgs::msg::ActionPerformerStatus::FAILURE;
    status_.status_stamp = now();
  }
  // 获取specialized_arguments参数
  get_parameter_or<std::vector<std::string>>(
      "specialized_arguments", specialized_arguments_, std::vector<std::string>({}));

  // 获取rate参数，计算出rate_
  double rate;
  get_parameter("rate", rate);
  rate_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / rate));

  // 创建发布者action_hub_pub_，发布plansys2_msgs::msg::ActionExecution类型的消息
  action_hub_pub_ = create_publisher<plansys2_msgs::msg::ActionExecution>(
      "actions_hub", rclcpp::QoS(100).reliable());
  // 创建订阅者action_hub_sub_，接收plansys2_msgs::msg::ActionExecution类型的消息，并绑定到回调函数action_hub_callback
  action_hub_sub_ = create_subscription<plansys2_msgs::msg::ActionExecution>(
      "actions_hub", rclcpp::QoS(100).reliable(),
      std::bind(&ActionExecutorClient::action_hub_callback, this, _1));
  // 激活action_hub_pub_
  action_hub_pub_->on_activate();

  // 设置status_的各个字段
  status_.state = plansys2_msgs::msg::ActionPerformerStatus::READY;
  status_.status_stamp = now();
  status_.action = action_managed_;
  status_.specialized_arguments = specialized_arguments_;

  return CallbackReturnT::SUCCESS;
}

/**
 * @brief ActionExecutorClient组件的on_activate回调函数
 * @param state 生命周期状态
 *
 * on_activate回调函数：
 * 1. 设置status_的state为RUNNING；
 * 2. 创建周期性定时器timer_，每隔rate_时间触发一次do_work函数；
 * 3. 返回CallbackReturnT::SUCCESS。
 */
CallbackReturnT ActionExecutorClient::on_activate(const rclcpp_lifecycle::State& state) {
  // 设置status_的state为RUNNING
  status_.state = plansys2_msgs::msg::ActionPerformerStatus::RUNNING;
  status_.status_stamp = now();
  // 创建周期性定时器timer_，每隔rate_时间触发一次do_work函数
  timer_ = create_wall_timer(rate_, std::bind(&ActionExecutorClient::do_work, this));

  return CallbackReturnT::SUCCESS;
}

/**
 * @brief ActionExecutorClient类的on_deactivate回调函数，用于执行动作停止时的操作
 * @param state 生命周期状态
 * @return CallbackReturnT 回调函数返回值类型
 * @details 将当前动作执行状态设置为READY，并更新状态时间戳，最后将计时器置为空指针并返回SUCCESS
 */
CallbackReturnT ActionExecutorClient::on_deactivate(const rclcpp_lifecycle::State& state) {
  status_.state = plansys2_msgs::msg::ActionPerformerStatus::READY;
  status_.status_stamp = now();
  timer_ = nullptr;

  return CallbackReturnT::SUCCESS;
}

/**
 * @brief ActionExecutorClient类的action_hub_callback回调函数，用于处理动作执行相关的消息
 * @param msg plansys2_msgs::msg::ActionExecution类型的ROS2消息指针
 * @details 根据不同的消息类型进行不同的处理：
 *          1. REQUEST：如果当前状态为PRIMARY_STATE_INACTIVE且未提交且应该执行，则提交并发送响应
 *          2.
 * CONFIRM：如果当前状态为PRIMARY_STATE_INACTIVE且已提交且节点ID与本节点相同，则触发激活转换并重置提交状态
 *          3. REJECT：如果节点ID与本节点相同，则重置提交状态
 *          4. CANCEL：如果当前状态为PRIMARY_STATE_ACTIVE且节点ID与本节点相同，则触发停止转换
 *          5. RESPONSE、FEEDBACK、FINISH：不做任何处理
 *          如果消息类型不在上述范围内，则输出错误信息
 */
void ActionExecutorClient::action_hub_callback(
    const plansys2_msgs::msg::ActionExecution::SharedPtr msg) {
  switch (msg->type) {
    case plansys2_msgs::msg::ActionExecution::REQUEST:
      if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE &&
          !commited_ && should_execute(msg->action, msg->arguments)) {
        commited_ = true;
        send_response(msg);
      }
      break;
    case plansys2_msgs::msg::ActionExecution::CONFIRM:
      if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE &&
          commited_ && msg->node_id == get_name()) {
        current_arguments_ = msg->arguments;
        trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
        commited_ = false;
      }
      break;
    case plansys2_msgs::msg::ActionExecution::REJECT:
      if (msg->node_id == get_name()) {
        commited_ = false;
      }
      break;
    case plansys2_msgs::msg::ActionExecution::CANCEL:
      if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE &&
          msg->node_id == get_name()) {
        trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
      }
      break;
    case plansys2_msgs::msg::ActionExecution::RESPONSE:
    case plansys2_msgs::msg::ActionExecution::FEEDBACK:
    case plansys2_msgs::msg::ActionExecution::FINISH:
      break;
    default:
      RCLCPP_ERROR(
          get_logger(), "Msg %d type not recognized in %s executor performer", msg->type,
          get_name());
      break;
  }
}

/**
 * @brief 判断是否应该执行当前动作
 * @param action 当前动作的名称
 * @param args 当前动作的参数列表
 * @return bool 是否应该执行当前动作
 * @details
 * 1. 如果当前动作不是管理的动作，则返回false。
 * 2.
 * 如果有专门的参数，则检查当前参数长度和专门参数长度是否匹配，如果不匹配则记录警告信息并返回false。
 * 3. 如果有专门的参数，则逐个比较当前参数和专门参数是否相同，如果不相同则返回false。
 * 4. 如果以上条件都满足，则返回true。
 */
bool ActionExecutorClient::should_execute(
    const std::string& action, const std::vector<std::string>& args) {
  if (action != action_managed_) {  // 如果当前动作不是管理的动作，则返回false
    return false;
  }

  if (!specialized_arguments_.empty()) {  // 如果有专门的参数
    if (specialized_arguments_.size() != args.size()) {  // 检查当前参数长度和专门参数长度是否匹配
      RCLCPP_WARN(
          get_logger(), "current and specialized arguments length doesn't match %zu %zu",
          args.size(), specialized_arguments_.size());  // 记录警告信息
      return false;
    }

    for (size_t i = 0; i < specialized_arguments_.size() && i < args.size();
         i++) {  // 逐个比较当前参数和专门参数是否相同
      if (specialized_arguments_[i] != "" && args[i] != "" &&
          specialized_arguments_[i] != args[i]) {
        return false;
      }
    }
  }

  return true;  // 如果以上条件都满足，则返回true
}

/**
 * @brief 发送动作执行的响应消息
 * @param msg 动作执行的消息
 * @details 将动作执行的消息类型设置为RESPONSE，并将其发送到动作中心。
 */
void ActionExecutorClient::send_response(const plansys2_msgs::msg::ActionExecution::SharedPtr msg) {
  plansys2_msgs::msg::ActionExecution msg_resp(*msg);
  msg_resp.type =
      plansys2_msgs::msg::ActionExecution::RESPONSE;  // 将动作执行的消息类型设置为RESPONSE
  msg_resp.node_id = get_name();

  action_hub_pub_->publish(msg_resp);  // 将消息发送到动作中心
}

/**
 * @brief 发送动作执行的反馈消息
 * @param completion 动作执行的完成度
 * @param status 动作执行的状态
 * @details 将动作执行的消息类型设置为FEEDBACK，并将其发送到动作中心。
 */
void ActionExecutorClient::send_feedback(float completion, const std::string& status) {
  plansys2_msgs::msg::ActionExecution msg_resp;
  msg_resp.type =
      plansys2_msgs::msg::ActionExecution::FEEDBACK;  // 将动作执行的消息类型设置为FEEDBACK
  msg_resp.node_id = get_name();
  msg_resp.action = action_managed_;
  msg_resp.arguments = current_arguments_;
  msg_resp.completion = completion;
  msg_resp.status = status;

  action_hub_pub_->publish(msg_resp);  // 将消息发送到动作中心
}

/**
 * @brief 完成动作执行的客户端
 * @param success 动作是否成功完成
 * @param completion 动作完成的进度
 * @param status 动作完成后的状态
 * @details
 * 1. 如果当前状态为 ACTIVE，则触发 TRANSITION_DEACTIVATE 状态转换。
 * 2. 创建 plansys2_msgs::msg::ActionExecution 消息，填充相关信息。
 * 3. 发布消息到 action_hub_pub_。
 */
void ActionExecutorClient::finish(bool success, float completion, const std::string& status) {
  if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
  }

  plansys2_msgs::msg::ActionExecution msg_resp;
  msg_resp.type = plansys2_msgs::msg::ActionExecution::FINISH;
  msg_resp.node_id = get_name();
  msg_resp.action = action_managed_;
  msg_resp.arguments = current_arguments_;
  msg_resp.completion = completion;
  msg_resp.status = status;
  msg_resp.success = success;

  action_hub_pub_->publish(msg_resp);
}

}  // namespace plansys2
