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

#ifndef PLANSYS2_EXECUTOR__BEHAVIOR_TREE__WAIT_ATSTART_REQ_NODE_HPP_
#define PLANSYS2_EXECUTOR__BEHAVIOR_TREE__WAIT_ATSTART_REQ_NODE_HPP_

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
 * @brief WaitAtStartReq类是一个继承自BT::ActionNodeBase的动作节点，用于等待满足启动要求的行为。
 * @param xml_tag_name 节点的名称
 * @param conf 节点的配置
 * @details 1. 继承了BT::ActionNodeBase基类，实现了tick()和halt()函数。
 *          2. providedPorts()函数提供了输入端口列表，其中包括一个字符串类型的action端口。
 *          3. action_map_是一个指向存储ActionExecutionInfo的map的智能指针。
 *          4. problem_client_是一个plansys2::ProblemExpertClient类型的智能指针。
 */
class WaitAtStartReq : public BT::ActionNodeBase {
public:
  WaitAtStartReq(const std::string& xml_tag_name, const BT::NodeConfiguration& conf);

  void halt() {}

  /**
   * @brief tick函数是WaitAtStartReq类的主要逻辑，用于检查是否满足启动要求。
   * @return BT::NodeStatus 枚举类型，表示节点的状态
   */
  BT::NodeStatus tick() override;

  /**
   * @brief providedPorts函数提供了WaitAtStartReq节点所需的输入端口列表。
   * @return BT::PortsList 输入端口列表
   */
  static BT::PortsList providedPorts() {
    return BT::PortsList({
        BT::InputPort<std::string>("action", "Action whose at start reqs must stop"),
    });
  }

private:
  std::shared_ptr<std::map<std::string, ActionExecutionInfo>>
      action_map_;      // 存储ActionExecutionInfo的map
  std::shared_ptr<plansys2::ProblemExpertClient>
      problem_client_;  // plansys2::ProblemExpertClient类型的智能指针
};

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__BEHAVIOR_TREE__WAIT_ATSTART_REQ_NODE_HPP_
