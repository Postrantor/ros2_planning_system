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

#ifndef PLANSYS2_EXECUTOR__BEHAVIOR_TREE__CHECK_ATEND_REQ_NODE_HPP_
#define PLANSYS2_EXECUTOR__BEHAVIOR_TREE__CHECK_ATEND_REQ_NODE_HPP_

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
 * @brief
 * CheckAtEndReq类是继承自BT::ActionNodeBase的一个动作节点，用于检查某个action是否满足结束条件
 * @param xml_tag_name 该节点在XML文件中的标签名
 * @param conf 节点配置信息
 * @details
 * 1. 构造函数CheckAtEndReq初始化了xml_tag_name和conf两个参数；
 * 2. halt()函数为空实现，不做任何操作；
 * 3. tick()函数为重载函数，用于执行节点逻辑；
 * 4.
 * providedPorts()函数返回一个BT::PortsList对象，其中包含一个输入端口"action"，用于传入需要检查的action名称；
 * 5. 类成员变量包括一个指向std::map<std::string,
 * ActionExecutionInfo>类型的智能指针action_map_，以及一个指向plansys2::ProblemExpertClient类型的智能指针problem_client_
 */
class CheckAtEndReq : public BT::ActionNodeBase {
public:
  CheckAtEndReq(const std::string& xml_tag_name, const BT::NodeConfiguration& conf);

  void halt() {}

  /**
   * @brief 执行节点逻辑
   * @return BT::NodeStatus 表示节点状态的枚举值之一
   * @details 1. 获取输入端口"action"的值；2.
   * 根据action名称从action_map_中获取对应的ActionExecutionInfo对象；3.
   * 调用problem_client_的getActionResult函数获取action的执行结果；
   * 4. 如果action执行成功，则返回BT::NodeStatus::SUCCESS；否则返回BT::NodeStatus::FAILURE。
   */
  BT::NodeStatus tick() override;

  /**
   * @brief 返回提供的端口列表
   * @return BT::PortsList 包含所有提供的端口的BT::PortsList对象
   * @details
   * providedPorts()函数返回一个BT::PortsList对象，其中包含一个输入端口"action"，用于传入需要检查的action名称
   */
  static BT::PortsList providedPorts() {
    return BT::PortsList({
        BT::InputPort<std::string>("action", "Action whose at end reqs must stop"),
    });
  }

private:
  std::shared_ptr<std::map<std::string, ActionExecutionInfo>>
      action_map_;      // 指向std::map<std::string, ActionExecutionInfo>类型的智能指针
  std::shared_ptr<plansys2::ProblemExpertClient>
      problem_client_;  // 指向plansys2::ProblemExpertClient类型的智能指针
};

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__BEHAVIOR_TREE__CHECK_ATEND_REQ_NODE_HPP_
