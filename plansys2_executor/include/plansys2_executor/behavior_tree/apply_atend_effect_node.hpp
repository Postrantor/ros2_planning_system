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

#ifndef PLANSYS2_EXECUTOR__BEHAVIOR_TREE__APPLY_ATEND_EFFECT_NODE_HPP_
#define PLANSYS2_EXECUTOR__BEHAVIOR_TREE__APPLY_ATEND_EFFECT_NODE_HPP_

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
 * @brief ApplyAtEndEffect类是继承自BT::ActionNodeBase的一个动作节点，用于在行为树中执行某些操作。
 * @param xml_tag_name 该节点在XML文件中的标签名
 * @param conf 行为树节点的配置
 * @details 1. 构造函数，初始化ApplyAtEndEffect对象；
 *          2. halt()函数，空实现，用于停止节点的执行；
 *          3. tick()函数，重写BT::ActionNodeBase的虚函数，用于执行节点的操作；
 *          4. providedPorts()函数，返回BT::PortsList类型的端口列表，用于与其他节点进行数据交互；
 *          5. action_map_成员变量，存储ActionExecutionInfo对象的共享指针；
 *          6. problem_client_成员变量，plansys2::ProblemExpertClient类型的共享指针。
 */
class ApplyAtEndEffect : public BT::ActionNodeBase {
public:
  /**
   * @brief 构造函数
   * @param xml_tag_name 该节点在XML文件中的标签名
   * @param conf 行为树节点的配置
   */
  ApplyAtEndEffect(const std::string& xml_tag_name, const BT::NodeConfiguration& conf);

  /**
   * @brief 停止节点的执行
   */
  void halt() {}

  /**
   * @brief 执行节点的操作
   * @return BT::NodeStatus 节点的状态
   */
  BT::NodeStatus tick() override;

  /**
   * @brief 返回端口列表
   * @return BT::PortsList 端口列表
   */
  static BT::PortsList providedPorts() {
    return BT::PortsList({
        BT::InputPort<std::string>("action", "Action whose at end reqs must stop"),
    });
  }

private:
  std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map_; // 存储ActionExecutionInfo对象的共享指针
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client_; // plansys2::ProblemExpertClient类型的共享指针
};

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__BEHAVIOR_TREE__APPLY_ATEND_EFFECT_NODE_HPP_
