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

#include "plansys2_executor/behavior_tree/execute_action_node.hpp"

#include <map>
#include <memory>
#include <string>

namespace plansys2 {

/**
 * @brief ExecuteAction 类的构造函数，用于初始化节点配置和动作映射表。
 *
 * @param xml_tag_name 节点 XML 标签名
 * @param conf 节点配置
 */
ExecuteAction::ExecuteAction(const std::string& xml_tag_name, const BT::NodeConfiguration& conf)
    : ActionNodeBase(xml_tag_name, conf) {
  // 获取动作映射表
  action_map_ =
      config().blackboard->get<std::shared_ptr<std::map<std::string, ActionExecutionInfo>>>(
          "action_map");
  // 获取生命周期节点
  node_ = config().blackboard->get<rclcpp_lifecycle::LifecycleNode::SharedPtr>("node");
}

/**
 * @brief 停止当前正在执行的动作。
 */
void ExecuteAction::halt() {
  std::string action;
  getInput("action", action);

  // 查找冒号位置，分离出动作表达式
  size_t delim = action.find(":");
  auto action_expr = action.substr(0, delim);

  // 如果动作正在运行，则取消它
  if ((*action_map_)[action].action_executor->get_status() == BT::NodeStatus::RUNNING) {
    (*action_map_)[action].action_executor->cancel();
  }
}

/**
 * @brief 执行指定的动作并返回执行结果。
 *
 * @return BT::NodeStatus 动作执行结果
 */
BT::NodeStatus ExecuteAction::tick() {
  std::string action;
  getInput("action", action);

  // 查找冒号位置，分离出动作表达式
  size_t delim = action.find(":");
  auto action_expr = action.substr(0, delim);

  // 如果动作执行器为空，则创建它
  if ((*action_map_)[action].action_executor == nullptr) {
    (*action_map_)[action].action_executor = ActionExecutor::make_shared(action_expr, node_);
  }

  // 执行动作并返回执行结果
  auto retval = (*action_map_)[action].action_executor->tick(node_->now());

  // 如果执行失败，则记录错误信息
  if (retval == BT::NodeStatus::FAILURE) {
    (*action_map_)[action].execution_error_info = "Error executing the action";

    RCLCPP_ERROR_STREAM(
        node_->get_logger(), "[" << action << "]" << (*action_map_)[action].execution_error_info);
  }

  return retval;
}

}  // namespace plansys2
