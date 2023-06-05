// Copyright 2020 Intelligent Robotics Lab
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

#ifndef PLANSYS2_EXECUTOR__BEHAVIOR_TREE__EXECUTE_ACTION_NODE_HPP_
#define PLANSYS2_EXECUTOR__BEHAVIOR_TREE__EXECUTE_ACTION_NODE_HPP_

#include <map>
#include <memory>
#include <random>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "plansys2_executor/ActionExecutor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace plansys2 {

/**
 * @brief ExecuteAction类是继承自BT::ActionNodeBase的一个动作节点，用于执行指定的操作。
 * @param xml_tag_name 节点名称
 * @param conf 节点配置
 * @details 1. 构造函数，初始化节点；2. halt函数，停止节点；3. tick函数，执行节点；4.
 * providedPorts函数，提供输入端口。
 */
class ExecuteAction : public BT::ActionNodeBase {
public:
  /**
   * @brief Construct a new Execute Action object
   *
   * @param xml_tag_name 节点名称
   * @param conf 节点配置
   */
  ExecuteAction(const std::string& xml_tag_name, const BT::NodeConfiguration& conf);

  /**
   * @brief 停止节点
   *
   */
  void halt();

  /**
   * @brief 执行节点
   *
   * @return BT::NodeStatus 节点状态
   */
  BT::NodeStatus tick() override;

  /**
   * @brief 提供输入端口
   *
   * @return BT::PortsList 输入端口列表
   */
  static BT::PortsList providedPorts() {
    return BT::PortsList({
        BT::InputPort<std::string>("action", "Action to be executed"),
    });
  }

private:
  std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map_;  // 存储操作信息的指针
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;  // 生命周期节点的智能指针
};

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__BEHAVIOR_TREE__EXECUTE_ACTION_NODE_HPP_
