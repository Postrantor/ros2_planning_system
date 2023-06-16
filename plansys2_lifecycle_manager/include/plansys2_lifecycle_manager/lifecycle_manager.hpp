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

#ifndef PLANSYS2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_
#define PLANSYS2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_

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
  这段代码又是和 demos/lifecycle 中的代码一样...
  感觉这个就是manager
*/

/**
 * @brief 等待一个 future 对象的结果，直到超时或者得到结果。
 *
 * @tparam FutureT 未来对象类型模板参数
 * @tparam WaitTimeT 等待时间类型模板参数
 * @param future 未来对象
 * @param time_to_wait 等待时间
 * @return std::future_status 返回 future 对象的状态
 */
template <typename FutureT, typename WaitTimeT>
std::future_status wait_for_result(FutureT& future, WaitTimeT time_to_wait) {
  auto end = std::chrono::steady_clock::now() + time_to_wait;  // 获取等待结束时间点
  std::chrono::milliseconds wait_period(100);                  // 定义等待周期为 100 毫秒
  std::future_status status = std::future_status::timeout;     // 初始化状态为超时
  do {
    auto now = std::chrono::steady_clock::now();               // 获取当前时间点
    auto time_left = end - now;                                // 计算剩余等待时间
    // 如果剩余等待时间小于等于 0 秒，则跳出循环
    if (time_left <= std::chrono::seconds(0)) {
      break;
    }
    // 等待 future 对象的结果，等待时间为剩余等待时间和等待周期中较小的那个
    status = future.wait_for((time_left < wait_period) ? time_left : wait_period);
    // 当 ROS2 节点正常运行且未得到结果时继续循环
  } while (rclcpp::ok() && status != std::future_status::ready);

  // 返回 future 对象的状态
  return status;
}

// [](D:\Document\Hirain\Project\rolling\ros-planning\navigation2\nav2_util\src\lifecycle_service_client.cpp)
// 这个LifecycleServerClient就相当于这个 "nav2_util\src\lifecycle_service_client.cpp" 角色
// 反过来，这个cpp实际上就是manager的角色，在 demos/lifecycle 中的例子也是一样的
/**
 * @brief 生命周期服务客户端类，继承自 rclcpp::Node 类。
 */
class LifecycleServiceClient : public rclcpp::Node {
public:
  /**
   * @brief 构造函数，初始化节点名称和被管理节点名称。
   * @param node_name 节点名称
   * @param managed_node 被管理节点名称(在 nav2 中是 parent_node)
   */
  explicit LifecycleServiceClient(
      const std::string& node_name,  //
      const std::string& managed_node);

  /**
   * @brief 初始化生命周期服务客户端。
   */
  void init();

  /**
   * @brief 获取被管理节点的状态。
   * @param time_out 等待超时时间，默认为 3 秒
   * @return unsigned int 返回被管理节点的状态码
   */
  unsigned int get_state(std::chrono::seconds time_out = std::chrono::seconds(3));

  /**
   * @brief 改变被管理节点的状态。
   * @param transition 状态转换码
   * @param time_out 等待超时时间，默认为 3 秒
   * @return true 改变状态成功
   * @return false 改变状态失败
   */
  bool change_state(
      std::uint8_t transition,  //
      std::chrono::seconds time_out = std::chrono::seconds(3));

private:
  // 生命周期服务获取状态客户端
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> client_get_state_;
  // 生命周期服务改变状态客户端
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state_;
  // 被管理节点名称
  std::string managed_node_;
};

/**
 * @brief 启动函数，初始化生命周期服务客户端。
 * @param manager_nodes 管理节点的映射表
 * @param timeout 等待超时时间，默认为 3 秒
 * @return true 初始化成功
 * @return false 初始化失败
 */
bool startup_function(
    std::map<std::string, std::shared_ptr<LifecycleServiceClient>>& manager_nodes,
    std::chrono::seconds timeout);

}  // namespace plansys2

#endif  // PLANSYS2_LIFECYCLE_MANAGER__LIFECYCLE_MANAGER_HPP_
