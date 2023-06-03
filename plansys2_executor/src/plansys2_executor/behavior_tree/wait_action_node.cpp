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

#include "plansys2_executor/behavior_tree/wait_action_node.hpp"

#include <map>
#include <memory>
#include <string>

namespace plansys2 {

/**
 * @brief WaitAction 类的构造函数
 *
 * @param xml_tag_name 一个字符串，表示 XML 标签名称
 * @param conf BT::NodeConfiguration 类型的引用，表示节点配置信息
 */
WaitAction::WaitAction(const std::string& xml_tag_name, const BT::NodeConfiguration& conf)
    : ActionNodeBase(xml_tag_name, conf) {
  // 获取 action_map_ 对象指针
  action_map_ =
      config().blackboard->get<std::shared_ptr<std::map<std::string, ActionExecutionInfo>>>(
          "action_map");
}

/**
 * @brief WaitAction 类的执行函数
 *
 * @return BT::NodeStatus 枚举类型，表示节点状态
 */
BT::NodeStatus WaitAction::tick() {
  std::string action;
  getInput("action", action);                                 // 获取输入参数 action

  if ((*action_map_).find(action) == (*action_map_).end()) {  // 如果 action_map_ 中没有找到 action
    return BT::NodeStatus::RUNNING;  // 返回 RUNNING 状态，表示未开始
  }

  if ((*action_map_)[action].action_executor != nullptr &&      // 如果 action_executor 不为空
      (*action_map_)[action].action_executor->is_finished() &&  // 如果 action 执行完成
      (*action_map_)[action].at_start_effects_applied &&  // 如果 at_start_effects 已经应用
      (*action_map_)[action].at_end_effects_applied) {    // 如果 at_end_effects 已经应用
    return BT::NodeStatus::SUCCESS;  // 返回 SUCCESS 状态，表示执行成功
  } else {
    return BT::NodeStatus::RUNNING;  // 返回 RUNNING 状态，表示正在执行
  }
}

}  // namespace plansys2
