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

#ifndef PLANSYS2_EXECUTOR__BEHAVIOR_TREE__CHECK_TIMEOUT_NODE_HPP_
#define PLANSYS2_EXECUTOR__BEHAVIOR_TREE__CHECK_TIMEOUT_NODE_HPP_

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
 * @brief CheckTimeout类是一个BT::ActionNodeBase的子类，用于检查某个动作是否超时。
 * @param xml_tag_name 该节点在行为树XML文件中的标签名
 * @param conf BT节点配置对象
 * @details 提供了tick()和halt()两个函数，以及providedPorts()静态函数。其中：
 * - tick()函数会被行为树调用，用于执行检查动作是否超时的操作；
 * - halt()函数为空实现，因为该节点没有需要停止的操作；
 * - providedPorts()函数提供了一个输入端口，用于接收需要检查的动作名称。
 *
 * 类成员变量：
 * -
 * start_：一个std::chrono::high_resolution_clock::time_point类型的变量，表示动作开始执行的时间点；
 * - action_map_：一个指向std::map<std::string,
 * ActionExecutionInfo>类型的shared_ptr，存储所有动作的执行信息；
 * - problem_client_：一个指向plansys2::ProblemExpertClient类型的shared_ptr，用于查询当前问题状态。
 */
class CheckTimeout : public BT::ActionNodeBase {
public:
  CheckTimeout(const std::string& xml_tag_name, const BT::NodeConfiguration& conf);

  void halt() {}

  /**
   * @brief 执行检查动作是否超时的操作
   * @return BT::NodeStatus 行为树节点状态
   */
  BT::NodeStatus tick() override;

  /**
   * @brief 静态函数，提供了一个输入端口，用于接收需要检查的动作名称
   * @return BT::PortsList 输入端口列表
   */
  static BT::PortsList providedPorts() {
    return BT::PortsList({
        BT::InputPort<std::string>("action", "Action whose over all reqs must stop"),
    });
  }

private:
  std::chrono::high_resolution_clock::time_point start_;  // 动作开始执行的时间点
  std::shared_ptr<std::map<std::string, ActionExecutionInfo>>
      action_map_;                                        // 存储所有动作的执行信息
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client_;  // 查询当前问题状态的客户端
};

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__BEHAVIOR_TREE__CHECK_TIMEOUT_NODE_HPP_
