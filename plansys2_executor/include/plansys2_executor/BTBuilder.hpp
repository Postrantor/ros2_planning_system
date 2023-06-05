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

#ifndef PLANSYS2_EXECUTOR__BTBUILDER_HPP_
#define PLANSYS2_EXECUTOR__BTBUILDER_HPP_

#include <map>
#include <memory>
#include <string>

#include "plansys2_executor/ActionExecutor.hpp"
#include "plansys2_msgs/msg/plan.hpp"

namespace plansys2 {

/**
 * @brief 枚举类型，表示动作的不同状态
 */
enum struct ActionType {
  UNKNOWN,   // 未知状态
  INIT,      // 初始化状态
  DURATIVE,  // 持续状态
  START,     // 开始状态
  OVERALL,   // 总体状态
  END,       // 结束状态
  GOAL       // 目标状态
};

/**
 * @brief 表示一个带时间戳的动作结构体
 */
struct ActionStamped {
  float time;                                                  // 时间戳
  std::string expression;                                      // 动作表达式
  float duration;                                              // 持续时间
  ActionType type;                                             // 动作类型
  std::shared_ptr<plansys2_msgs::msg::DurativeAction> action;  // 持续动作

  /**
   * @brief 默认构造函数
   */
  ActionStamped() : time(0.0), duration(0.0) {}
};

/**
 * @brief BTBuilder类，用于构建行为树
 */
class BTBuilder {
public:
  using Ptr = std::shared_ptr<plansys2::BTBuilder>;

  /**
   * @brief 初始化函数
   * @param bt_action_1 第一个行为树动作
   * @param bt_action_2 第二个行为树动作
   * @param precision 精度
   */
  virtual void initialize(
      const std::string& bt_action_1 = "",
      const std::string& bt_action_2 = "",
      int precision = 3) = 0;

  /**
   * @brief 获取行为树
   * @param current_plan 当前计划
   * @return 行为树字符串
   */
  virtual std::string get_tree(const plansys2_msgs::msg::Plan& current_plan) = 0;

  /**
   * @brief 获取行为树的dot图形式
   * @param action_map 动作映射表
   * @param enable_legend 是否启用图例
   * @param enable_print_graph 是否打印图形
   * @return dot图形字符串
   */
  virtual std::string get_dotgraph(
      std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map,
      bool enable_legend = false,
      bool enable_print_graph = false) = 0;

  /**
   * @brief 将浮点型时间转换为整型时间
   * @param time 浮点型时间
   * @param power 幂次
   * @return 整型时间
   */
  static int to_int_time(float time, int power) {
    float scale = pow(10.0, static_cast<float>(power));
    return static_cast<int>(time * scale);
  }

  /**
   * @brief 将动作类型转换为字符串
   * @param action_type 动作类型
   * @return 动作类型字符串
   */
  static std::string to_string(const ActionType& action_type) {
    switch (action_type) {
      case ActionType::INIT:
        return "INIT";
      case ActionType::DURATIVE:
        return "DURATIVE";
      case ActionType::START:
        return "START";
      case ActionType::OVERALL:
        return "OVERALL";
      case ActionType::END:
        return "END";
      case ActionType::GOAL:
        return "GOAL";
      default:
        return "UNKNOWN";
    }
  }

  /**
   * @brief 将计划项转换为动作ID
   * @param item 计划项
   * @param precision 精度
   * @return 动作ID字符串
   */
  static std::string to_action_id(const plansys2_msgs::msg::PlanItem& item, int precision) {
    return item.action + ":" + std::to_string(to_int_time(item.time, precision));
  }

  /**
   * @brief 将带时间戳的动作转换为动作ID
   * @param action 带时间戳的动作
   * @param precision 精度
   * @return 动作ID字符串
   */
  static std::string to_action_id(const ActionStamped& action, int precision) {
    return action.expression + ":" + std::to_string(to_int_time(action.time, precision));
  }
};

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__BTBUILDER_HPP_
