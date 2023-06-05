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

#ifndef PLANSYS2_EXECUTOR__BEHAVIOR_TREE__CHECK_ACTION_NODE_HPP_
#define PLANSYS2_EXECUTOR__BEHAVIOR_TREE__CHECK_ACTION_NODE_HPP_

#include <map>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "plansys2_executor/ActionExecutor.hpp"
#include "plansys2_executor/behavior_tree/execute_action_node.hpp"

namespace plansys2 {

/**
 * @brief CheckAction类是BT::ActionNodeBase的子类，用于检查要执行的操作是否存在
 * @param xml_tag_name 表示XML标记名称
 * @param conf 表示节点配置
 * @details CheckAction类有一个构造函数和两个虚函数，提供了providedPorts()方法以返回输入端口列表。
 *          该类还包含一个指向动作映射的共享指针。
 */
class CheckAction : public BT::ActionNodeBase {
public:
  /**
   * @brief 构造函数
   * @param xml_tag_name 表示XML标记名称
   * @param conf 表示节点配置
   */
  CheckAction(const std::string& xml_tag_name, const BT::NodeConfiguration& conf);

  /**
   * @brief 停止执行
   */
  void halt() {}

  /**
   * @brief 执行检查操作
   * @return 返回节点状态
   */
  BT::NodeStatus tick() override;

  /**
   * @brief 提供输入端口列表
   * @return 返回输入端口列表
   */
  static BT::PortsList providedPorts() {
    return BT::PortsList({
        BT::InputPort<std::string>("action", "Action to be executed"),
    });
  }

private:
  /**
   * @brief 指向动作映射的共享指针
   */
  std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map_;
};

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__BEHAVIOR_TREE__CHECK_ACTION_NODE_HPP_
