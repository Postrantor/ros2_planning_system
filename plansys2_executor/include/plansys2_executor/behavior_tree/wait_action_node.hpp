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

#ifndef PLANSYS2_EXECUTOR__BEHAVIOR_TREE__WAIT_ACTION_NODE_HPP_
#define PLANSYS2_EXECUTOR__BEHAVIOR_TREE__WAIT_ACTION_NODE_HPP_

#include <map>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "plansys2_executor/ActionExecutor.hpp"
#include "plansys2_executor/behavior_tree/execute_action_node.hpp"

namespace plansys2 {

/**
 * @brief WaitAction类是BT::ActionNodeBase的派生类，用于等待执行某个动作。
 * @details WaitAction类提供了构造函数、halt()和tick()方法，以及providedPorts()静态方法。
 *
 * 参数列表：
 * - xml_tag_name: 表示XML标签名称的字符串常量引用。
 * - conf: BT::NodeConfiguration类型的节点配置对象。
 */
class WaitAction : public BT::ActionNodeBase {
public:
  WaitAction(const std::string& xml_tag_name, const BT::NodeConfiguration& conf);

  // 停止节点的执行
  void halt() {}

  // 执行节点的逻辑
  BT::NodeStatus tick() override;

  // 静态方法，返回提供的端口列表
  static BT::PortsList providedPorts() {
    return BT::PortsList({
        BT::InputPort<std::string>("action", "Action to be executed"),
    });
  }

private:
  // 存储动作执行信息的map指针
  std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map_;
};

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__BEHAVIOR_TREE__WAIT_ACTION_NODE_HPP_
