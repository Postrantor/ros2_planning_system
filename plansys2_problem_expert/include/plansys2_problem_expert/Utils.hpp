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

#ifndef PLANSYS2_PROBLEM_EXPERT__UTILS_HPP_
#define PLANSYS2_PROBLEM_EXPERT__UTILS_HPP_

#include <map>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_msgs/msg/tree.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

/**
  该代码段包含了一些函数，这些函数用于处理 PDDL 表达式。其中，
  - evaluate 函数用于评估表示为树的 PDDL 表达式，
  - check 函数用于检查表示为树的 PDDL 表达式，
  - apply 函数用于应用表示为树的 PDDL 表达式。
  - parse_action 函数用于从输入字符串中解析出动作表达式和时间（可选），

  - get_action_expression 函数用于从输入字符串中解析出动作表达式，
  - get_action_time 函数用于从输入字符串中解析出动作时间，
  - get_action_name 函数用于从输入字符串中解析出动作名称，
  - get_action_params 函数用于从输入字符串中解析出动作参数名称。
 */
namespace plansys2 {

/**
 * @brief 评估表示为树的 PDDL 表达式。
 * @param[in] node PDDL 表达式的根节点。
 * @param[in] problem_client ProblemExpertClient 对象指针。
 * @param[in] predicates 当前谓词状态。
 * @param[in] functions 当前函数状态。
 * @param[in] apply 是否将结果应用到 ProblemExpert 或状态中。
 * @param[in] use_state 是否使用状态表示或 ProblemExpert。
 * @param[in] node_id 节点 ID。
 * @param[in] negate 反转真值。
 * @return result <- tuple(bool, bool, double)
 *         result(0) 如果成功，则为 true。
 *         result(1) 布尔表达式的真值。
 *         result(2) 数值表达式的值。
 */
std::tuple<bool, bool, double> evaluate(
    const plansys2_msgs::msg::Tree& tree,
    std::shared_ptr<plansys2::ProblemExpertClient> problem_client,
    std::vector<plansys2::Predicate>& predicates,
    std::vector<plansys2::Function>& functions,
    bool apply = false,
    bool use_state = false,
    uint8_t node_id = 0,
    bool negate = false);

/**
 * @brief 评估表示为树的 PDDL 表达式。
 * @param[in] node PDDL 表达式的根节点。
 * @param[in] problem_client ProblemExpertClient 对象指针。
 * @param[in] apply 是否将结果应用到 ProblemExpert 或状态中。
 * @param[in] node_id 节点 ID。
 * @return result <- tuple(bool, bool, double)
 *         result(0) 如果成功，则为 true。
 *         result(1) 布尔表达式的真值。
 *         result(2) 数值表达式的值。
 */
std::tuple<bool, bool, double> evaluate(
    const plansys2_msgs::msg::Tree& tree,
    std::shared_ptr<plansys2::ProblemExpertClient> problem_client,
    bool apply = false,
    uint32_t node_id = 0);

/**
 * @brief 评估表示为树的 PDDL 表达式。
 * @param[in] node PDDL 表达式的根节点。
 * @param[in] predicates 当前谓词状态。
 * @param[in] functions 当前函数状态。
 * @param[in] apply 是否将结果应用到 ProblemExpert 或状态中。
 * @param[in] node_id 节点 ID。
 * @return result <- tuple(bool, bool, double)
 *         result(0) 如果成功，则为 true。
 *         result(1) 布尔表达式的真值。
 *         result(2) 数值表达式的值。
 */
std::tuple<bool, bool, double> evaluate(
    const plansys2_msgs::msg::Tree& tree,
    std::vector<plansys2::Predicate>& predicates,
    std::vector<plansys2::Function>& functions,
    bool apply = false,
    uint32_t node_id = 0);

/**
 * @brief 检查表示为树的 PDDL 表达式。
 * @param[in] node PDDL 表达式的根节点。
 * @param[in] problem_client ProblemExpertClient 对象指针。
 * @param[in] node_id 节点 ID。
 * @return ret PDDL 表达式的真值。
 *
 * 此函数调用 evaluate 函数。
 */
bool check(
    const plansys2_msgs::msg::Tree& tree,
    std::shared_ptr<plansys2::ProblemExpertClient> problem_client,
    uint32_t node_id = 0);

/**
 * @brief 检查表示为树的 PDDL 表达式。
 * @param[in] node PDDL 表达式的根节点。
 * @param[in] predicates 当前谓词状态。
 * @param[in] functions 当前函数状态。
 * @param[in] node_id 节点 ID。
 * @return ret PDDL 表达式的真值。
 */
bool check(
    const plansys2_msgs::msg::Tree& tree,
    std::vector<plansys2::Predicate>& predicates,
    std::vector<plansys2::Function>& functions,
    uint32_t node_id = 0);

/**
 * @brief 应用表示为树的 PDDL 表达式。
 * @param[in] node PDDL 表达式的根节点。
 * @param[in] problem_client ProblemExpertClient 对象指针。
 * @param[in] node_id 节点 ID。
 * @return success 执行是否成功。
 *
 * 此函数调用 evaluate 函数。
 */
bool apply(
    const plansys2_msgs::msg::Tree& tree,
    std::shared_ptr<plansys2::ProblemExpertClient> problem_client,
    uint32_t node_id = 0);

/**
 * @brief 应用表示为树的 PDDL 表达式。
 * @param[in] node PDDL 表达式的根节点。
 * @param[in] predicates 当前谓词状态。
 * @param[in] functions 当前函数状态。
 * @param[in] node_id 节点 ID。
 * @return success 执行是否成功。
 */
bool apply(
    const plansys2_msgs::msg::Tree& tree,
    std::vector<plansys2::Predicate>& predicates,
    std::vector<plansys2::Function>& functions,
    uint32_t node_id = 0);

/**
 * @brief 从输入字符串中解析出动作表达式和时间（可选）。
 * @param[in] input 输入字符串。
 * @return result <- pair(string, int)
 *         result(0) 动作表达式。
 *         result(1) 动作开始时间。
 *
 * 输入字符串可以是以下任一格式。
 *   "(<name> <param_1> ... <param_n>)"
 *   "(<name> <param_1> ... <param_n>):<time>"
 * 输出的动作表达式将具有以下格式。
 *   "<name> <param_1> ... <param_n>"
 */
std::pair<std::string, int> parse_action(const std::string& input);

/**
 * @brief 从输入字符串中解析出动作表达式。
 * @param[in] input 输入字符串。
 * @return 动作表达式。
 *
 * 输入字符串可以是以下任一格式。
 *   "(<name> <param_1> ... <param_n>)"
 *   "(<name> <param_1> ... <param_n>):<time>"
 */
std::string get_action_expression(const std::string& input);

/**
 * @brief 从输入字符串中解析出动作时间。
 * @param[in] input 输入字符串。
 * @return 动作开始时间。
 *
 * 输入字符串可以是以下任一格式。
 *   "(<name> <param_1> ... <param_n>)"
 *   "(<name> <param_1> ... <param_n>):<time>"
 */
int get_action_time(const std::string& input);

/**
 * @brief 从输入字符串中解析出动作名称。
 * @param[in] input 输入字符串。
 * @return 动作名称。
 *
 * 输入字符串可以是以下任一格式。
 *   "(<name> <param_1> ... <param_n>)"
 *   "(<name> <param_1> ... <param_n>):<time>"
 */
std::string get_action_name(const std::string& input);

/**
 * @brief 从输入字符串中解析出动作参数名称。
 * @param[in] action_expr 动作表达式。
 * @return 动作参数名称列表。
 *
 * 输入字符串可以是以下任一格式。
 *   "(<name> <param_1> ... <param_n>)"
 *   "(<name> <param_1> ... <param_n>):<time>"
 */
std::vector<std::string> get_action_params(const std::string& action_expr);

}  // namespace plansys2

#endif  // PLANSYS2_PROBLEM_EXPERT__UTILS_HPP_
