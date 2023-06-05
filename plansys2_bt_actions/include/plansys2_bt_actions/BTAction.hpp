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

#ifndef PLANSYS2_BT_ACTIONS__BTACTION_HPP_
#define PLANSYS2_BT_ACTIONS__BTACTION_HPP_

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_minitrace_logger.h"
#include "behaviortree_cpp_v3/xml_parsing.h"

#ifdef ZMQ_FOUND
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#endif

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"

namespace plansys2 {

/**
 * @brief BTAction类是plansys2::ActionExecutorClient的子类，用于执行行为树相关操作
 * @param action 行为名称
 * @param rate 执行频率
 * @details
 * 该类定义了BTAction的各种成员函数和变量，包括获取行为名称、获取行为树文件、配置、清理、激活、去激活等操作。
 */
class BTAction : public plansys2::ActionExecutorClient {
public:
  /**
   * @brief 构造函数
   * @param action 行为名称
   * @param rate 执行频率
   */
  explicit BTAction(const std::string& action, const std::chrono::nanoseconds& rate);

  /**
   * @brief 获取行为名称
   * @return 返回行为名称
   */
  const std::string& getActionName() const { return action_; }

  /**
   * @brief 获取行为树文件
   * @return 返回行为树文件
   */
  const std::string& getBTFile() const { return bt_xml_file_; }

protected:
  /**
   * @brief 配置函数
   * @param previous_state 前一个状态
   * @return 返回状态
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state);

  /**
   * @brief 清理函数
   * @param previous_state 前一个状态
   * @return 返回状态
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State& previous_state);

  /**
   * @brief 激活函数
   * @param previous_state 前一个状态
   * @return 返回状态
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state);

  /**
   * @brief 去激活函数
   * @param previous_state 前一个状态
   * @return 返回状态
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state);

  /**
   * @brief 执行函数
   */
  void do_work();

  BT::BehaviorTreeFactory factory_;

private:
  BT::Tree tree_;
  BT::Blackboard::Ptr blackboard_;
  std::string action_;       // 行为名称
  std::string bt_xml_file_;  // 行为树文件
  std::vector<std::string> plugin_list_;
  bool finished_;
  std::unique_ptr<BT::PublisherZMQ> publisher_zmq_;
  std::unique_ptr<BT::FileLogger> bt_file_logger_;
  std::unique_ptr<BT::MinitraceLogger> bt_minitrace_logger_;
};

}  // namespace plansys2

#endif  // PLANSYS2_BT_ACTIONS__BTACTION_HPP_
