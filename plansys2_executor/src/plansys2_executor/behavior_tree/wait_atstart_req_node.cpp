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

#include "plansys2_executor/behavior_tree/wait_atstart_req_node.hpp"

#include <map>
#include <memory>
#include <string>
#include <tuple>

namespace plansys2 {

/**
 * @brief WaitAtStartReq类的构造函数，继承自ActionNodeBase类
 * @param xml_tag_name 节点名称
 * @param conf BT::NodeConfiguration类型的配置信息
 * @details 从黑板中获取动作映射表和问题专家客户端
 */
WaitAtStartReq::WaitAtStartReq(const std::string& xml_tag_name, const BT::NodeConfiguration& conf)
    : ActionNodeBase(xml_tag_name, conf) {
  // 获取动作映射表
  action_map_ =
      config().blackboard->get<std::shared_ptr<std::map<std::string, ActionExecutionInfo>>>(
          "action_map");
  // 获取问题专家客户端
  problem_client_ =
      config().blackboard->get<std::shared_ptr<plansys2::ProblemExpertClient>>("problem_client");
}

/**
 * @brief WaitAtStartReq类的执行函数
 * @details
 * 获取动作名称，节点，开始需求和总体需求，检查开始需求和总体需求是否满足，如果不满足则返回运行状态RUNNING，否则返回运行状态SUCCESS
 */
BT::NodeStatus WaitAtStartReq::tick() {
  std::string action;
  // 获取输入参数action
  getInput("action", action);

  auto node = config().blackboard->get<rclcpp_lifecycle::LifecycleNode::SharedPtr>("node");

  // 获取动作的开始需求
  auto reqs_as = (*action_map_)[action].durative_action_info->at_start_requirements;
  // 获取动作的总体需求
  auto reqs_oa = (*action_map_)[action].durative_action_info->over_all_requirements;

  // 检查开始需求是否满足
  bool check_as = check(reqs_as, problem_client_);
  if (!check_as) {
    (*action_map_)[action].execution_error_info = "Error checking at start reqs";
    // 输出错误信息
    RCLCPP_ERROR_STREAM(
        node->get_logger(),
        "[" << action << "]" << (*action_map_)[action].execution_error_info << ": "
            << parser::pddl::toString(
                   (*action_map_)[action].durative_action_info->at_start_requirements));
    // 返回运行状态RUNNING
    return BT::NodeStatus::RUNNING;
  }

  // 检查总体需求是否满足
  bool check_oa = check(reqs_oa, problem_client_);
  if (!check_oa) {
    (*action_map_)[action].execution_error_info = "Error checking over all reqs";
    // 输出错误信息
    RCLCPP_ERROR_STREAM(
        node->get_logger(),
        "[" << action << "]" << (*action_map_)[action].execution_error_info << ": "
            << parser::pddl::toString(
                   (*action_map_)[action].durative_action_info->over_all_requirements));
    // 返回运行状态RUNNING
    return BT::NodeStatus::RUNNING;
  }

  // 返回运行状态SUCCESS
  return BT::NodeStatus::SUCCESS;
}

}  // namespace plansys2
