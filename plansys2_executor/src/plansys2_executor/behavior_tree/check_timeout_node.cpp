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

#include "plansys2_executor/behavior_tree/check_timeout_node.hpp"

#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <tuple>

namespace plansys2 {

/**
 * @brief CheckTimeout 类的构造函数
 * @param xml_tag_name 节点的 XML 标签名
 * @param conf 节点的配置信息
 */
CheckTimeout::CheckTimeout(const std::string& xml_tag_name, const BT::NodeConfiguration& conf)
    : ActionNodeBase(xml_tag_name, conf) {
  // 获取 action_map 指针
  action_map_ =
      config().blackboard->get<std::shared_ptr<std::map<std::string, ActionExecutionInfo>>>(
          "action_map");

  // 获取 problem_client 指针
  problem_client_ =
      config().blackboard->get<std::shared_ptr<plansys2::ProblemExpertClient>>("problem_client");
}

/**
 * @brief CheckTimeout 类的 tick 函数
 * @return BT::NodeStatus 返回节点状态
 */
BT::NodeStatus CheckTimeout::tick() {
  std::string action;
  getInput("action", action);

  // 如果当前节点状态为 IDLE，则记录开始时间
  if (status() == BT::NodeStatus::IDLE) {
    start_ = std::chrono::high_resolution_clock::now();
  }
  setStatus(BT::NodeStatus::RUNNING);

  // 如果 action_map 中存在该动作，则进行超时检查
  if ((*action_map_)[action].action_executor != nullptr) {
    double duration = (*action_map_)[action].duration;
    double duration_overrun_percentage = (*action_map_)[action].duration_overrun_percentage;
    // 如果 duration_overrun_percentage 大于等于 0，则计算最大持续时间
    if (duration_overrun_percentage >= 0) {
      double max_duration = (1.0 + duration_overrun_percentage / 100.0) * duration;
      auto current_time = std::chrono::high_resolution_clock::now();
      auto elapsed_time =
          std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_);
      // 如果实际持续时间超过最大持续时间，则返回 FAILURE
      if (elapsed_time > std::chrono::duration<double>(max_duration)) {
        std::cerr << "Actual duration of " << action << " exceeds max duration (" << std::fixed
                  << std::setprecision(2) << max_duration << " secs)." << std::endl;
        return BT::NodeStatus::FAILURE;
      }
    }
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace plansys2
