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

#include "plansys2_executor/behavior_tree/check_atend_req_node.hpp"

#include <map>
#include <memory>
#include <string>
#include <tuple>

namespace plansys2 {

/**
 * @brief CheckAtEndReq类的构造函数，用于初始化成员变量action_map_和problem_client_
 * @param xml_tag_name 节点名称
 * @param conf BT::NodeConfiguration类型的配置信息
 * @details 1. 获取blackboard中的action_map指针并赋值给action_map_；
 *          2. 获取blackboard中的problem_client指针并赋值给problem_client_。
 */
CheckAtEndReq::CheckAtEndReq(const std::string& xml_tag_name, const BT::NodeConfiguration& conf)
    : ActionNodeBase(xml_tag_name, conf) {
  action_map_ =
      config().blackboard->get<std::shared_ptr<std::map<std::string, ActionExecutionInfo>>>(
          "action_map");

  problem_client_ =
      config().blackboard->get<std::shared_ptr<plansys2::ProblemExpertClient>>("problem_client");
}

/**
 * @brief CheckAtEndReq类的tick函数，用于检查当前动作是否满足结束要求
 * @return BT::NodeStatus 枚举类型，返回SUCCESS表示满足结束要求，返回FAILURE表示不满足结束要求
 * @details 1. 获取输入参数action；
 *          2. 获取blackboard中的node指针；
 *          3. 获取当前动作的结束要求reqs；
 *          4. 调用check函数检查当前动作是否满足结束要求：
 *              a. 如果不满足，则将执行错误信息赋值给execution_error_info，并返回FAILURE；
 *              b. 如果满足，则返回SUCCESS。
 */
BT::NodeStatus CheckAtEndReq::tick() {
  std::string action;
  getInput("action", action);

  auto node = config().blackboard->get<rclcpp_lifecycle::LifecycleNode::SharedPtr>("node");

  auto reqs = (*action_map_)[action].durative_action_info->at_end_requirements;

  if (!check(reqs, problem_client_)) {
    (*action_map_)[action].execution_error_info = "Error checking at end requirements";

    RCLCPP_ERROR_STREAM(
        node->get_logger(),
        "[" << action << "]" << (*action_map_)[action].execution_error_info << ": "
            << parser::pddl::toString(
                   (*action_map_)[action].durative_action_info->at_end_requirements));

    return BT::NodeStatus::FAILURE;
  } else {
    return BT::NodeStatus::SUCCESS;
  }
}

}  // namespace plansys2
