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

#include "plansys2_executor/behavior_tree/apply_atstart_effect_node.hpp"

#include <map>
#include <memory>
#include <string>

namespace plansys2 {

/**
 * @brief ApplyAtStartEffect 类的构造函数，用于初始化 ActionNodeBase 的 xml_tag_name 和 conf
 * 参数，并将 action_map_ 和 problem_client_ 初始化为指向 blackboard 中对应变量的 shared_ptr。
 * @param xml_tag_name 节点名称
 * @param conf 行为树节点配置
 * @details ApplyAtStartEffect 继承自 ActionNodeBase，用于在行为树中执行某个动作前，先执行该动作的
 * at_start_effects。at_start_effects 是 plansys2
 * 中一个概念，表示某个动作开始时需要执行的效果。action_map_ 保存了所有动作的信息，problem_client_
 * 用于与 plansys2 的问题专家服务进行通信。
 */
ApplyAtStartEffect::ApplyAtStartEffect(
    const std::string& xml_tag_name, const BT::NodeConfiguration& conf)
    : ActionNodeBase(xml_tag_name, conf) {
  action_map_ =
      config().blackboard->get<std::shared_ptr<std::map<std::string, ActionExecutionInfo>>>(
          "action_map");

  problem_client_ =
      config().blackboard->get<std::shared_ptr<plansys2::ProblemExpertClient>>("problem_client");
}

/**
 * @brief ApplyAtStartEffect 类的 tick 函数，用于执行 at_start_effects。
 * @return BT::NodeStatus::SUCCESS 执行成功
 * @details 如果当前动作的 at_start_effects 还没有被执行过，则执行该动作的
 * at_start_effects，并将该动作的 at_start_effects_applied 标记设为 true。
 */
BT::NodeStatus ApplyAtStartEffect::tick() {
  std::string action;
  getInput("action", action);

  auto effect = (*action_map_)[action].durative_action_info->at_start_effects;

  if (!(*action_map_)[action].at_start_effects_applied) {
    (*action_map_)[action].at_start_effects_applied = true;
    apply(effect, problem_client_, 0);
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace plansys2
