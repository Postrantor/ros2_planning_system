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

#include "plansys2_lifecycle_manager/lifecycle_manager.hpp"

#include <chrono>
#include <map>
#include <memory>
#include <string>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"

namespace plansys2 {

/**
 * @brief LifecycleServiceClient类的构造函数，用于创建一个LifecycleServiceClient对象。
 *
 * @param node_name 节点名称。
 * @param managed_node 管理节点名称。
 */
LifecycleServiceClient::LifecycleServiceClient(
    const std::string& node_name, const std::string& managed_node)
    : Node(node_name), managed_node_(managed_node) {}

/**
 * @brief 初始化LifecycleServiceClient对象。
 */
void LifecycleServiceClient::init() {
  // 获取管理节点的状态服务和更改状态服务的名称。
  std::string get_state_service_name = managed_node_ + "/get_state";
  std::string change_state_service_name = managed_node_ + "/change_state";
  // 输出日志信息，表示正在为获取状态服务和更改状态服务创建客户端。
  RCLCPP_INFO(get_logger(), "Creating client for service [%s]", get_state_service_name.c_str());
  RCLCPP_INFO(get_logger(), "Creating client for service [%s]", change_state_service_name.c_str());
  // 创建获取状态服务和更改状态服务的客户端。
  client_get_state_ = this->create_client<lifecycle_msgs::srv::GetState>(get_state_service_name);
  client_change_state_ =
      this->create_client<lifecycle_msgs::srv::ChangeState>(change_state_service_name);
}

/**
 * @brief 获取管理节点的当前状态。
 *
 * @param time_out 请求超时时间。
 * @return unsigned int 返回管理节点的当前状态。
 */
unsigned int LifecycleServiceClient::get_state(std::chrono::seconds time_out) {
  // 创建一个获取状态请求。
  auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
  // 如果获取状态服务不可用，则返回未知状态。
  if (!client_get_state_->wait_for_service(time_out)) {
    RCLCPP_ERROR(
        get_logger(), "Service %s is not available.", client_get_state_->get_service_name());
    return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
  }
  // 发送获取状态请求，并等待获取到节点的响应。
  auto future_result = client_get_state_->async_send_request(request);
  auto future_status = wait_for_result(future_result, time_out);
  auto state = future_result.get();
  // 如果请求超时，则返回未知状态。
  if (future_status != std::future_status::ready) {
    RCLCPP_ERROR(
        get_logger(), "Server time out while getting current state for node %s",
        managed_node_.c_str());
    return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
  }
  // 如果成功获取到节点的响应，则输出当前状态信息，并返回当前状态。
  if (state != nullptr) {
    RCLCPP_INFO(
        get_logger(), "Node %s has current state %s.", get_name(),
        state->current_state.label.c_str());
    return state->current_state.id;
  } else {
    // 如果未能成功获取到节点的响应，则返回未知状态。
    RCLCPP_ERROR(get_logger(), "Failed to get current state for node %s", managed_node_.c_str());
    return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
  }
}

/**
 * @brief 对 LifecycleServiceClient 类中的 change_state 函数进行说明
 *
 * @param transition 一个 uint8_t 类型的参数，表示要执行的状态转换
 * @param time_out 一个 std::chrono::seconds 类型的参数，表示等待服务的超时时间
 * @return bool 返回一个布尔值，表示状态转换是否成功
 */
bool LifecycleServiceClient::change_state(std::uint8_t transition, std::chrono::seconds time_out) {
  // 创建一个 ChangeState 请求
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  // 设置请求中的状态转换
  request->transition.id = transition;
  // 等待服务可用
  if (!client_change_state_->wait_for_service(time_out)) {
    // 如果服务不可用，输出错误信息并返回 false
    RCLCPP_ERROR(
        get_logger(), "Service %s is not available.", client_change_state_->get_service_name());
    return false;
  }
  // 发送带有所需状态转换的请求
  auto future_result = client_change_state_->async_send_request(request);
  // 等待节点的响应
  auto future_status = wait_for_result(future_result, time_out);
  // 如果请求超时，返回 false
  if (future_status != std::future_status::ready) {
    RCLCPP_ERROR(
        get_logger(), "Server time out while getting current state for node %s",
        managed_node_.c_str());
    return false;
  }
  // 如果请求成功，输出成功信息并返回 true
  if (future_result.get()->success) {
    RCLCPP_INFO(
        get_logger(), "Transition %d successfully triggered.", static_cast<int>(transition));
    return true;
  } else {
    // 如果请求失败，输出警告信息并返回 false
    RCLCPP_WARN(
        get_logger(), "Failed to trigger transition %u", static_cast<unsigned int>(transition));
    return false;
  }
}

/**
 * @brief 启动函数，用于启动 lifecycle 组件
 *
 * @param manager_nodes 存储了各个组件的 LifecycleServiceClient 对象的 map
 * @param timeout 等待超时时间
 * @return true 启动成功
 * @return false 启动失败
 */
bool startup_function(
    std::map<std::string, std::shared_ptr<LifecycleServiceClient>>& manager_nodes,
    std::chrono::seconds timeout) {
  // configure domain_expert
  {
    // 如果 domain_expert 组件无法进入 CONFIGURED 状态，则返回 false
    if (!manager_nodes["domain_expert"]->change_state(
            lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, timeout)) {
      return false;
    }

    // 等待 domain_expert 组件进入 PRIMARY_STATE_INACTIVE 状态
    while (manager_nodes["domain_expert"]->get_state() !=
           lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      std::cerr << "Waiting for inactive state for domain_expert" << std::endl;
    }
  }

  // configure problem_expert
  {
    // 如果 problem_expert 组件无法进入 CONFIGURED 状态，则返回 false
    if (!manager_nodes["problem_expert"]->change_state(
            lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, timeout)) {
      return false;
    }

    // 等待 problem_expert 组件进入 PRIMARY_STATE_INACTIVE 状态
    while (manager_nodes["problem_expert"]->get_state() !=
           lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      std::cerr << "Waiting for inactive state for problem_expert" << std::endl;
    }
  }

  // configure planner
  {
    // 如果 planner 组件无法进入 CONFIGURED 状态，则返回 false
    if (!manager_nodes["planner"]->change_state(
            lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, timeout)) {
      return false;
    }

    // 等待 planner 组件进入 PRIMARY_STATE_INACTIVE 状态
    while (manager_nodes["planner"]->get_state() !=
           lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      std::cerr << "Waiting for inactive state for planner" << std::endl;
    }
  }

  // configure executor
  {
    // 如果 executor 组件无法进入 CONFIGURED 状态，则返回 false
    if (!manager_nodes["executor"]->change_state(
            lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, timeout)) {
      return false;
    }

    // 等待 executor 组件进入 PRIMARY_STATE_INACTIVE 状态
    while (manager_nodes["executor"]->get_state() !=
           lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      std::cerr << "Waiting for inactive state for planner" << std::endl;
    }
  }

  // activate
  {
    // 如果 rclcpp 不可用，则返回 false
    if (!rclcpp::ok()) {
      return false;
    }

    // 如果 domain_expert 组件无法进入 ACTIVE 状态，则返回 false
    if (!manager_nodes["domain_expert"]->change_state(
            lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, timeout)) {
      return false;
    }

    // 如果 problem_expert 组件无法进入 ACTIVE 状态，则返回 false
    if (!manager_nodes["problem_expert"]->change_state(
            lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, timeout)) {
      return false;
    }

    // 如果 planner 组件无法进入 ACTIVE 状态，则返回 false
    if (!manager_nodes["planner"]->change_state(
            lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, timeout)) {
      return false;
    }

    // 如果 executor 组件无法进入 ACTIVE 状态，则返回 false
    if (!manager_nodes["executor"]->change_state(
            lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, timeout)) {
      return false;
    }

    // 如果 domain_expert 组件不在 ACTIVE 状态，则返回 false
    if (!manager_nodes["domain_expert"]->get_state()) {
      return false;
    }

    // 如果 problem_expert 组件不在 ACTIVE 状态，则返回 false
    if (!manager_nodes["problem_expert"]->get_state()) {
      return false;
    }

    // 如果 planner 组件不在 ACTIVE 状态，则返回 false
    if (!manager_nodes["planner"]->get_state()) {
      return false;
    }

    // 如果 executor 组件不在 ACTIVE 状态，则返回 false
    if (!manager_nodes["executor"]->get_state()) {
      return false;
    }
  }

  return true;
}

}  // namespace plansys2
