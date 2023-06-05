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

#ifndef PLANSYS2_EXECUTOR__BT_BUILDER_PLUGINS__SIMPLE_BT_BUILDER_HPP_
#define PLANSYS2_EXECUTOR__BT_BUILDER_PLUGINS__SIMPLE_BT_BUILDER_HPP_

#include <list>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "plansys2_core/Types.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ActionExecutor.hpp"
#include "plansys2_executor/BTBuilder.hpp"
#include "plansys2_msgs/msg/durative_action.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/empty.hpp"

namespace plansys2 {

/**
 * @brief GraphNode结构体
 * @details 用于存储规划器中的节点信息，包括动作、节点编号、层级编号、谓词和函数等信息。
 */
struct GraphNode {
  using Ptr = std::shared_ptr<GraphNode>;
  static Ptr make_shared() { return std::make_shared<GraphNode>(); }

  ActionStamped action;                         // 动作
  int node_num;                                 // 节点编号
  int level_num;                                // 层级编号

  std::vector<plansys2::Predicate> predicates;  // 谓词
  std::vector<plansys2::Function> functions;    // 函数

  std::list<GraphNode::Ptr> in_arcs;            // 入边
  std::list<GraphNode::Ptr> out_arcs;           // 出边
};

/**
 * @brief Graph结构体
 * @details 用于存储规划器中的图信息，包括根节点、不同层级的节点等信息。
 */
struct Graph {
  using Ptr = std::shared_ptr<Graph>;
  static Ptr make_shared() { return std::make_shared<Graph>(); }

  std::list<GraphNode::Ptr> roots;                    // 根节点
  std::map<float, std::list<GraphNode::Ptr>> levels;  // 不同层级的节点
};

/**
 * @brief SimpleBTBuilder是一个继承自BTBuilder的类，用于生成行为树。
 */
class SimpleBTBuilder : public BTBuilder {
public:
  /**
   * @brief 构造函数。
   */
  SimpleBTBuilder();

  /**
   * @brief 初始化函数，用于设置行为树中的动作和精度等参数。
   * @param bt_action_1 行为树中的第一个动作。
   * @param bt_action_2 行为树中的第二个动作。
   * @param precision 精度。
   */
  void initialize(
      const std::string& bt_action_1 = "", const std::string& bt_action_2 = "", int precision = 3);

  /**
   * @brief 根据当前计划获取行为树。
   * @param current_plan 当前计划。
   * @return 返回生成的行为树。
   */
  std::string get_tree(const plansys2_msgs::msg::Plan& current_plan);

  /**
   * @brief 根据行为图生成dot格式的图形化表示。
   * @param action_map 动作映射表。
   * @param enable_legend 是否启用图例。
   * @param enable_print_graph 是否打印图形化表示。
   * @return 返回dot格式的图形化表示。
   */
  std::string get_dotgraph(
      std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map,
      bool enable_legend = false,
      bool enable_print_graph = false);

protected:
  std::shared_ptr<plansys2::DomainExpertClient> domain_client_;    // 领域专家客户端
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client_;  // 问题专家客户端

  Graph::Ptr graph_;                                               // 行为图指针
  std::string bt_;                                                 // 行为树
  std::string bt_action_;                                          // 行为树中的动作

  /**
   * @brief 根据当前计划获取行为图。
   * @param current_plan 当前计划。
   * @return 返回生成的行为图。
   */
  Graph::Ptr get_graph(const plansys2_msgs::msg::Plan& current_plan);

  /**
   * @brief 获取计划中的所有动作。
   * @param plan 计划。
   * @return 返回计划中的所有动作。
   */
  std::vector<ActionStamped> get_plan_actions(const plansys2_msgs::msg::Plan& plan);

  /**
   * @brief 向后修剪行为图。
   * @param new_node 新节点。
   * @param node_satisfy 满足节点。
   */
  void prune_backwards(GraphNode::Ptr new_node, GraphNode::Ptr node_satisfy);

  /**
   * @brief 向前修剪行为图。
   * @param current 当前节点。
   * @param used_nodes 已使用的节点列表。
   */
  void prune_forward(GraphNode::Ptr current, std::list<GraphNode::Ptr>& used_nodes);

  /**
   * @brief 获取节点状态。
   * @param node 节点。
   * @param used_nodes 已使用的节点列表。
   * @param predicates 谓词列表。
   * @param functions 函数列表。
   */
  void get_state(
      const GraphNode::Ptr& node,
      std::list<GraphNode::Ptr>& used_nodes,
      std::vector<plansys2::Predicate>& predicates,
      std::vector<plansys2::Function>& functions) const;

  /**
   * @brief 判断动作是否可执行。
   * @param action 动作。
   * @param predicates 谓词列表。
   * @param functions 函数列表。
   * @return 如果动作可执行返回true，否则返回false。
   */
  bool is_action_executable(
      const ActionStamped& action,
      std::vector<plansys2::Predicate>& predicates,
      std::vector<plansys2::Function>& functions) const;

  /**
   * @brief 获取根节点。
   * @param action_sequence 动作序列。
   * @param predicates 谓词列表。
   * @param functions 函数列表。
   * @param node_counter 节点计数器。
   * @return 返回根节点列表。
   */
  std::list<GraphNode::Ptr> get_roots(
      std::vector<plansys2::ActionStamped>& action_sequence,
      std::vector<plansys2::Predicate>& predicates,
      std::vector<plansys2::Function>& functions,
      int& node_counter);

  /**
   * @brief 获取满足节点。
   * @param requirement 需求。
   * @param graph 行为图。
   * @param current 当前节点。
   * @return 返回满足节点。
   */
  GraphNode::Ptr get_node_satisfy(
      const plansys2_msgs::msg::Tree& requirement,
      const Graph::Ptr& graph,
      const GraphNode::Ptr& current);

  /**
   * @brief 获取满足节点。
   * @param requirement 需求。
   * @param node 节点。
   * @param current 当前节点。
   * @return 返回满足节点。
   */
  GraphNode::Ptr get_node_satisfy(
      const plansys2_msgs::msg::Tree& requirement,
      const GraphNode::Ptr& node,
      const GraphNode::Ptr& current);

  /**
   * @brief 获取矛盾节点列表。
   * @param graph 行为图。
   * @param current 当前节点。
   * @return 返回矛盾节点列表。
   */
  std::list<GraphNode::Ptr> get_node_contradict(
      const Graph::Ptr& graph, const GraphNode::Ptr& current);

  /**
   * @brief 获取矛盾节点列表。
   * @param node 节点。
   * @param current 当前节点。
   * @param parents 父节点列表。
   */
  void get_node_contradict(
      const GraphNode::Ptr& node,
      const GraphNode::Ptr& current,
      std::list<GraphNode::Ptr>& parents);

  /**
   * @brief 移除已存在的需求。
   * @param requirements 需求列表。
   * @param predicates 谓词列表。
   * @param functions 函数列表。
   */
  void remove_existing_requirements(
      std::vector<plansys2_msgs::msg::Tree>& requirements,
      std::vector<plansys2::Predicate>& predicates,
      std::vector<plansys2::Function>& functions) const;

  /**
   * @brief 判断动作是否可并行执行。
   * @param action 动作。
   * @param predicates 谓词列表。
   * @param functions 函数列表。
   * @param ret 返回的节点列表。
   * @return 如果动作可并行执行返回true，否则返回false。
   */
  bool is_parallelizable(
      const plansys2::ActionStamped& action,
      const std::vector<plansys2::Predicate>& predicates,
      const std::vector<plansys2::Function>& functions,
      const std::list<GraphNode::Ptr>& ret) const;

  /**
   * @brief 获取流程树。
   * @param node 节点。
   * @param used_nodes 已使用的节点列表。
   * @param level 层数。
   * @return 返回流程树。
   */
  std::string get_flow_tree(GraphNode::Ptr node, std::list<std::string>& used_nodes, int level = 0);

  /**
   * @brief 获取dot格式的流程图。
   * @param node 节点。
   * @param edges 边缘列表。
   */
  void get_flow_dotgraph(GraphNode::Ptr node, std::set<std::string>& edges);

  /**
   * @brief 获取节点的dot格式表示。
   * @param node 节点。
   * @param action_map 动作映射表。
   * @param level 层数。
   * @return 返回节点的dot格式表示。
   */
  std::string get_node_dotgraph(
      GraphNode::Ptr node,
      std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map,
      int level = 0);

  /**
   * @brief 获取动作状态。
   * @param action 动作。
   * @param action_map 动作映射表。
   * @return 返回动作状态。
   */
  ActionExecutor::Status get_action_status(
      ActionStamped action, std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map);

  /**
   * @brief 添加dot格式的图例。
   * @param ss 字符串流。
   * @param tab_level 制表符层数。
   * @param level_counter 层计数器。
   * @param node_counter 节点计数器。
   */
  void addDotGraphLegend(std::stringstream& ss, int tab_level, int level_counter, int node_counter);

  /**
   * @brief 获取制表符。
   * @param level 层数。
   * @return 返回制表符。
   */
  std::string t(int level);

  /**
   * @brief 执行块，返回节点的字符串表示形式
   * @param node 节点指针
   * @param l 节点的层数
   * @return 节点的字符串表示形式
   */
  std::string execution_block(const GraphNode::Ptr& node, int l);

  /**
   * @brief 打印节点的信息，包括节点名称、层数和子节点等信息
   * @param node 节点指针
   * @param level 节点的层数
   * @param used_nodes 已经使用过的节点集合
   */
  void print_node(
      const GraphNode::Ptr& node, int level, std::set<GraphNode::Ptr>& used_nodes) const;

  /**
   * @brief 打印图的信息，包括所有节点的信息
   * @param graph 图指针
   */
  void print_graph(const plansys2::Graph::Ptr& graph) const;

  /**
   * @brief 以 CSV 格式打印节点的信息，包括节点名称、层数和子节点等信息
   * @param node 节点指针
   * @param root_num 根节点编号
   */
  void print_node_csv(const GraphNode::Ptr& node, uint32_t root_num) const;

  /**
   * @brief 以 CSV 格式打印图的信息，包括所有节点的信息
   * @param graph 图指针
   */
  void print_graph_csv(const plansys2::Graph::Ptr& graph) const;

  /**
   * @brief 获取节点的表格形式，用于在 GUI 中显示
   * @param node 节点指针
   * @param root_num 根节点编号
   * @param graph 节点的表格形式，包括节点编号、层数、父节点编号和节点名称
   */
  void get_node_tabular(
      const plansys2::GraphNode::Ptr& node,
      uint32_t root_num,
      std::vector<std::tuple<uint32_t, uint32_t, uint32_t, std::string>>& graph) const;

  /**
   * @brief 获取图的表格形式，用于在 GUI 中显示
   * @param graph 图指针
   * @return 图的表格形式，包括节点编号、层数、父节点编号和节点名称
   */
  std::vector<std::tuple<uint32_t, uint32_t, uint32_t, std::string>> get_graph_tabular(
      const plansys2::Graph::Ptr& graph) const;
};

}  // namespace plansys2

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(plansys2::SimpleBTBuilder, plansys2::BTBuilder)

#endif  // PLANSYS2_EXECUTOR__BT_BUILDER_PLUGINS__SIMPLE_BT_BUILDER_HPP_
