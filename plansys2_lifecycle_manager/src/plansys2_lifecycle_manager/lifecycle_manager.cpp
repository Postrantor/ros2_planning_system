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

/*
  其中LifecycleServiceClient类用于初始化节点和管理节点名称，并提供了获取管理节点当前状态的方法get_state。
  在init函数中，通过管理节点名称获取管理节点的状态服务和更改状态服务的名称，并创建了状态服务和更改状态服务的客户端。
  在get_state函数中，首先创建获取状态请求，然后等待状态服务可用。
  如果超时，则返回未知状态；如果成功获取到状态，则输出当前状态信息并返回当前状态；否则返回未知状态。
*/

/**
 * @brief LifecycleServiceClient类的构造函数，用于初始化节点和管理节点名称
 * @param node_name 节点名称
 * @param managed_node 管理节点名称
 */
LifecycleServiceClient::LifecycleServiceClient(
    const std::string& node_name,  //
    const std::string& managed_node)
    : Node(node_name),             //
      managed_node_(managed_node) {}

/**
 * @brief 初始化生命周期服务客户端
 */
void LifecycleServiceClient::init() {
  // clang-format off
  // 获取管理节点的状态服务和更改状态服务的名称，并输出日志信息
  std::string get_state_service_name = managed_node_ + "/get_state";
  std::string change_state_service_name = managed_node_ + "/change_state";
  RCLCPP_INFO(get_logger(), "Creating client for service [%s]", get_state_service_name.c_str());
  RCLCPP_INFO(get_logger(), "Creating client for service [%s]", change_state_service_name.c_str());
  // 创建状态服务和更改状态服务的客户端
  // 这里使用的是两个接口，按照design中的设计，rclcpp_lifecycle 中是提供了5个默认的接口
  client_get_state_ = this->create_client<lifecycle_msgs::srv::GetState>(get_state_service_name);
  client_change_state_ = this->create_client<lifecycle_msgs::srv::ChangeState>(change_state_service_name);
  // clang-format on
}

// clang-format off
/*
  [](E:\Download\Documents\ChatGPT\dialogue\ros\std_future_20230616.md)
  [](E:\Download\Documents\ChatGPT\dialogue\ros\async_20230613.md)

  1. 创建获取状态请求：首先创建一个 `lifecycle_msgs::srv::GetState::Request` 类型的请求消息 `request`，用于获取管理节点的当前状态。
  2. 等待状态服务可用：使用 `client_get_state_->wait_for_service(time_out)` 方法等待管理节点的获取状态服务可用，如果超时时间 `time_out` 到达而服务不可用，则返回 UNKNOWN 状态。
  3. 发送获取状态请求：使用 `client_get_state_->async_send_request(request)` 方法向管理节点发送状态请求，返回类型 `auto` 会自动推断为 `std::shared_future<std::shared_ptr<lifecycle_msgs::srv::GetState::Response>>`。
  4. 等待获取状态请求的结果：使用 `wait_for_result(future_result, time_out)` 方法等待请求结果，如果超时时间到达则返回 UNKNOWN 状态，并输出错误信息。如果请求结果已经准备好，则会调用 `future_result.get()` 方法返回状态信息指针 `state`。
  5. 处理状态信息：如果成功获取到状态，则输出当前状态信息并返回当前状态。如果获取状态失败，则返回 UNKNOWN 状态，并输出错误信息。

  总体来说，这段代码主要涉及到目标节点的状态获取服务 `lifecycle_msgs::srv::GetState` 的异步调用操作，其中的 `std::future` 和 `std::shared_future` 类型是 C++11 STL 中提供的用于异步编程的线程库，它们可以让线程在异步执行任务的同时返回一个未来的对象，根据错误码或者异常处理未来对象的结果。而异步获取节点状态的方式可以使得节点状态的获取不影响主线程的运行。

  `std::future_status::ready` 是 C++11 STL 中的一个枚举类型之一，用于表示一个异步操作是否已经完成。具体来说，当一个 `std::future` 或者 `std::shared_future` 类型的对象关联的异步操作已经完成时，对应的状态是 `std::future_status::ready`，此时可以通过 `future_result.get()` 方法来获取异步操作的结果；当异步操作仍在进行中时，对应的状态是 `std::future_status::timeout` 或者 `std::future_status::deferred`。

  在这段代码中，`future_status` 用于判断异步获取状态请求是否已经完成。如果 `future_status` 不等于 `std::future_status::ready`，说明异步请求超时或者出现错误，需要输出错误信息并返回 UNKNOWN 状态。
  反之，则代表成功地获取到了节点的当前状态，需要输出当前状态信息并返回当前状态。

  其中，注释中提到的 ChangeState 是一个 ROS2 中定义的服务类型，包含一个 uint8 类型的 id 字段，表示要执行的状态转换。
    - client_change_state_ 是一个 rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr 类型的成员变量，表示 LifecycleServiceClient 类中用于请求节点状态转换服务的客户端。
    - wait_for_service 和 async_send_request 是 rclcpp::Client 类中定义的方法，分别用于等待服务可用和异步发送请求。
    - wait_for_result 是一个辅助函数，用于等待 future 对象的结果。
    - RCLCPP_ERROR、RCLCPP_INFO 和 RCLCPP_WARN 是 ROS2 中定义的日志打印宏，分别用于打印错误、信息和警告日志。
*/
// clang-format on
/**
 * @brief 获取管理节点的当前状态
 * @param time_out 请求超时时间
 * @return 管理节点的当前状态
 */
unsigned int LifecycleServiceClient::get_state(std::chrono::seconds time_out) {
  // 创建获取状态请求
  auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
  // 等待状态服务可用
  if (!client_get_state_->wait_for_service(time_out)) {
    RCLCPP_ERROR(
        get_logger(), "Service %s is not available.", client_get_state_->get_service_name());
    return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
  }

  // 发送获取状态请求
  auto future_result = client_get_state_->async_send_request(request);
  // 等待获取状态请求的结果，如果超时则返回未知状态
  auto future_status = wait_for_result(future_result, time_out);
  // 判断异步操作是否已经完成
  if (future_status != std::future_status::ready) {
    RCLCPP_ERROR(
        get_logger(), "Server time out while getting current state for node %s",
        managed_node_.c_str());
    return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
  }

  // @zhiqi.jia 移动到 if() 之后
  // 如果成功获取到状态，则输出当前状态信息并返回当前状态
  auto state = future_result.get();
  if (state != nullptr) {
    RCLCPP_INFO(
        get_logger(), "Node %s has current state %s.", get_name(),
        state->current_state.label.c_str());
    return state->current_state.id;
  } else {
    // 获取状态失败，返回未知状态
    RCLCPP_ERROR(get_logger(), "Failed to get current state for node %s", managed_node_.c_str());
    return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
  }
}

/**
 * @brief LifecycleServiceClient 类的 change_state 函数，用于请求节点状态转换服务
 * @param transition 要执行的状态转换
 * @param time_out 等待服务可用和等待响应的超时时间
 * @return bool 返回是否成功触发状态转换
 * @details
 */
bool LifecycleServiceClient::change_state(std::uint8_t transition, std::chrono::seconds time_out) {
  // 1. 创建一个 ChangeState 请求对象 request，并将要执行的状态转换赋值给 request 的 transition
  // 字段。
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition;
  // 2. 如果等待服务可用的时间超过了 time_out，则打印错误日志并返回 false。
  if (!client_change_state_->wait_for_service(time_out)) {
    RCLCPP_ERROR(
        get_logger(), "Service %s is not available.", client_change_state_->get_service_name());
    return false;
  }
  // 3. 使用 async_send_request 方法异步发送请求，并返回一个 future_result 对象。
  // We send the request with the transition we want to invoke.
  auto future_result = client_change_state_->async_send_request(request);
  // 4. 使用 wait_for_result 方法等待 future_result 对象的结果，如果等待时间超过了
  // time_out，则打印错误日志并返回 false。 Let's wait until we have the answer from the node. If
  // the request times out, we return an unknown state.
  auto future_status = wait_for_result(future_result, time_out);
  if (future_status != std::future_status::ready) {
    RCLCPP_ERROR(
        get_logger(), "Server time out while getting current state for node %s",
        managed_node_.c_str());
    return false;
  }
  // 5. 如果请求成功，打印信息日志并返回 true；否则打印警告日志并返回 false。
  // We have an answer, let's print our success.
  if (future_result.get()->success) {
    RCLCPP_INFO(
        get_logger(), "Transition %d successfully triggered.", static_cast<int>(transition));
    return true;
  } else {
    RCLCPP_WARN(
        get_logger(), "Failed to trigger transition %u", static_cast<unsigned int>(transition));
    return false;
  }
}

// 这个是和具体的功能相关的了，为啥要放在这里？
// 注意，这里给的就是std::map<>，就是为了在lifecycle_manager_node 中通过std::bind进行绑定用的
// clang-format off
/*
  该函数接受两个参数：生命周期管理器节点的映射表和超时时间。该函数对四个生命周期管理器节点进行配置和激活操作，并等待其进入非活跃状态。如果任意一个操作失败或超时，则返回 false。

  具体来说，该函数首先对领域专家、问题专家、规划器和执行器四个生命周期管理器节点进行配置操作，即调用 change_state 函数并传入 TRANSITION_CONFIGURE 和超时时间。然后，该函数进入一个循环，等待每个节点进入非活跃状态。

  接着，该函数对四个生命周期管理器节点进行激活操作，即调用 change_state 函数并传入 TRANSITION_ACTIVATE 和超时时间。如果任意一个操作失败或超时，则返回 false。最后，该函数确认每个节点已经激活，并返回 true 表示启动成功。
*/
// clang-format on
/**
 * @brief 启动函数
 * @param manager_nodes 生命周期管理器节点的映射表
 * @param timeout 超时时间
 * @return bool 启动成功返回 true，否则返回 false
 * @details 对四个生命周期管理器节点进行配置和激活操作，并等待其进入非活跃状态。
 *         如果任意一个操作失败或超时，则返回 false。
 */
bool startup_function(
    std::map<std::string, std::shared_ptr<LifecycleServiceClient>>& manager_nodes,
    std::chrono::seconds timeout) {
  // configure domain_expert
  {
    if (!manager_nodes["domain_expert"]->change_state(
            lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, timeout)) {
      return false;
    }
    while (manager_nodes["domain_expert"]->get_state() !=
           lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      std::cerr << "Waiting for inactive state for domain_expert" << std::endl;
    }
  }

  // configure problem_expert
  {
    if (!manager_nodes["problem_expert"]->change_state(
            lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, timeout)) {
      return false;
    }
    while (manager_nodes["problem_expert"]->get_state() !=
           lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      std::cerr << "Waiting for inactive state for problem_expert" << std::endl;
    }
  }

  // configure planner
  {
    if (!manager_nodes["planner"]->change_state(
            lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, timeout)) {
      return false;
    }
    while (manager_nodes["planner"]->get_state() !=
           lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      std::cerr << "Waiting for inactive state for planner" << std::endl;
    }
  }

  // configure executor
  {
    if (!manager_nodes["executor"]->change_state(
            lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, timeout)) {
      return false;
    }
    while (manager_nodes["executor"]->get_state() !=
           lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      std::cerr << "Waiting for inactive state for planner" << std::endl;
    }
  }

  // =========
  // activate
  {
    // 如果 ROS2 节点已经关闭，则返回 false
    if (!rclcpp::ok()) {
      return false;
    }

    // 激活领域专家节点
    if (!manager_nodes["domain_expert"]->change_state(
            lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, timeout)) {
      return false;
    }
    // 激活问题专家节点
    if (!manager_nodes["problem_expert"]->change_state(
            lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, timeout)) {
      return false;
    }
    // 激活规划器节点
    if (!manager_nodes["planner"]->change_state(
            lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, timeout)) {
      return false;
    }
    // 激活执行器节点
    if (!manager_nodes["executor"]->change_state(
            lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, timeout)) {
      return false;
    }

    // =========
    // 确认领域专家节点已经激活
    if (!manager_nodes["domain_expert"]->get_state()) {
      return false;
    }
    // 确认问题专家节点已经激活
    if (!manager_nodes["problem_expert"]->get_state()) {
      return false;
    }
    // 确认规划器节点已经激活
    if (!manager_nodes["planner"]->get_state()) {
      return false;
    }
    // 确认执行器节点已经激活
    if (!manager_nodes["executor"]->get_state()) {
      return false;
    }
  }

  // 启动成功，返回 true
  return true;
}

}  // namespace plansys2
