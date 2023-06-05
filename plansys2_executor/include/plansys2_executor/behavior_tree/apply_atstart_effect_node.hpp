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

#ifndef PLANSYS2_EXECUTOR__BEHAVIOR_TREE__APPLY_ATSTART_EFFECT_NODE_HPP_
#define PLANSYS2_EXECUTOR__BEHAVIOR_TREE__APPLY_ATSTART_EFFECT_NODE_HPP_

#include <map>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "plansys2_executor/ActionExecutor.hpp"
#include "plansys2_executor/behavior_tree/execute_action_node.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_problem_expert/Utils.hpp"

namespace plansys2 {

/**
 * @brief ApplyAtStartEffect类是继承自BT::ActionNodeBase的一个动作节点，用于在行为树执行前应用效果。
 * @param xml_tag_name 该节点的XML标签名称
 * @param conf 节点配置
 * @details 1. halt函数为空实现，因为该节点不需要停止。
 *          2. tick函数是该节点的核心功能，用于在行为树执行前应用效果。
 *          3. providedPorts函数提供了该节点所需的输入端口信息。
 *          4. action_map_是一个指向存储ActionExecutionInfo的map的智能指针。
 *          5. problem_client_是一个plansys2::ProblemExpertClient类型的智能指针。
 */
class ApplyAtStartEffect : public BT::ActionNodeBase {
public:
  ApplyAtStartEffect(const std::string& xml_tag_name, const BT::NodeConfiguration& conf);

  void halt() {}

  /**
   * @brief tick函数是该节点的核心功能，用于在行为树执行前应用效果。
   * @return BT::NodeStatus 表示节点状态的枚举值之一
   */
  BT::NodeStatus tick() override;

  /**
   * @brief providedPorts函数提供了该节点所需的输入端口信息。
   * @return BT::PortsList 输入端口列表
   */
  static BT::PortsList providedPorts() {
    return BT::PortsList({
        BT::InputPort<std::string>("action", "Action whose at end reqs must stop"),
    });
  }

private:
  std::shared_ptr<std::map<std::string, ActionExecutionInfo>>
      action_map_;      // 存储ActionExecutionInfo的map的智能指针
  std::shared_ptr<plansys2::ProblemExpertClient>
      problem_client_;  // plansys2::ProblemExpertClient类型的智能指针
};

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__BEHAVIOR_TREE__APPLY_ATSTART_EFFECT_NODE_HPP_
