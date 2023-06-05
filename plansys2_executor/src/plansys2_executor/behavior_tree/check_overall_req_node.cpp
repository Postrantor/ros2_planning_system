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

#include "plansys2_executor/behavior_tree/check_overall_req_node.hpp"

#include <map>
#include <memory>
#include <string>
#include <tuple>

namespace plansys2 {

/**
 * @brief CheckOverAllReq类的构造函数，初始化action_map_和problem_client_
 * @param xml_tag_name 节点名称
 * @param conf BT节点配置信息
 * @details 1. 从blackboard中获取action_map_和problem_client_指针；2. 初始化ActionNodeBase基类
 */
CheckOverAllReq::CheckOverAllReq(const std::string& xml_tag_name, const BT::NodeConfiguration& conf)
    : ActionNodeBase(xml_tag_name, conf) {  // 调用基类构造函数初始化节点
  action_map_ =                             // 获取action_map_指针
      config().blackboard->get<std::shared_ptr<std::map<std::string, ActionExecutionInfo>>>(
          "action_map");

  problem_client_ =  // 获取problem_client_指针
      config().blackboard->get<std::shared_ptr<plansys2::ProblemExpertClient>>("problem_client");
}

/**
 * @brief CheckOverAllReq类的tick函数，检查over_all_requirements是否满足
 * @return BT::NodeStatus::SUCCESS 满足要求；BT::NodeStatus::FAILURE 不满足要求
 * @details 1. 获取输入参数action；2. 获取节点指针node；3. 获取当前动作的over_all_requirements；4.
 * 调用check函数检查over_all_requirements是否满足；5.
 * 如果不满足，更新execution_error_info并返回BT::NodeStatus::FAILURE；6.
 * 如果满足，返回BT::NodeStatus::SUCCESS。
 */
BT::NodeStatus CheckOverAllReq::tick() {
  std::string action;          // 定义输入参数action
  getInput("action", action);  // 获取输入参数action

  auto node = config().blackboard->get<rclcpp_lifecycle::LifecycleNode::SharedPtr>(
      "node");  // 获取节点指针node

  auto reqs =
      (*action_map_)[action]
          .durative_action_info->over_all_requirements;  // 获取当前动作的over_all_requirements

  if (!check(reqs, problem_client_)) {  // 调用check函数检查over_all_requirements是否满足
    (*action_map_)[action].execution_error_info =
        "Error checking over all requirements";  // 更新execution_error_info

    RCLCPP_ERROR_STREAM(                         // 输出错误信息
        node->get_logger(),
        "[" << action << "]" << (*action_map_)[action].execution_error_info << ": "
            << parser::pddl::toString(
                   (*action_map_)[action].durative_action_info->over_all_requirements));

    return BT::NodeStatus::FAILURE;  // 返回BT::NodeStatus::FAILURE
  } else {
    return BT::NodeStatus::SUCCESS;  // 返回BT::NodeStatus::SUCCESS
  }
}

}  // namespace plansys2
