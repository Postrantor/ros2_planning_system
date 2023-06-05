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

#include "plansys2_executor/behavior_tree/check_action_node.hpp"

#include <map>
#include <memory>
#include <string>

namespace plansys2 {

/**
 * @brief CheckAction类的构造函数，继承自ActionNodeBase类
 * @param xml_tag_name 该节点在XML文件中的标签名
 * @param conf 行为树节点配置信息
 * @details 初始化CheckAction对象，并获取"action_map"参数的值
 */
CheckAction::CheckAction(const std::string& xml_tag_name, const BT::NodeConfiguration& conf)
    : ActionNodeBase(xml_tag_name, conf) {
  action_map_ =
      config().blackboard->get<std::shared_ptr<std::map<std::string, ActionExecutionInfo>>>(
          "action_map");
}

/**
 * @brief CheckAction类的tick函数，重载自BT::coro::AsyncActionNode类
 * @details 获取输入参数"action"的值，判断是否存在于action_map_中，
 * 如果存在且执行器已经完成任务并且开始和结束效果都已应用，则返回SUCCESS；
 * 否则返回RUNNING。
 */
BT::NodeStatus CheckAction::tick() {
  std::string action;
  getInput("action", action);

  if ((*action_map_).find(action) == (*action_map_).end()) {
    return BT::NodeStatus::RUNNING;  // Not started yet
  }

  if ((*action_map_)[action].action_executor != nullptr &&
      (*action_map_)[action].action_executor->is_finished() &&
      (*action_map_)[action].at_start_effects_applied &&
      (*action_map_)[action].at_end_effects_applied) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::RUNNING;
  }
}

}  // namespace plansys2
