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

#ifndef PLANSYS2_EXECUTOR__BT_BUILDER_PLUGINS__STN_BT_BUILDER_HPP_
#define PLANSYS2_EXECUTOR__BT_BUILDER_PLUGINS__STN_BT_BUILDER_HPP_

#include <list>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ActionExecutor.hpp"
#include "plansys2_executor/BTBuilder.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

namespace plansys2 {

/**
 * @brief 定义了三个结构体：StateVec、GraphNode和Graph，用于规划器的图搜索。
 * @details
 * 1. StateVec 结构体包含两个向量，分别存储谓词和函数。
 * 2. GraphNode 结构体表示图中的一个节点，包含节点编号、动作、输入弧和输出弧。
 * 3. Graph 结构体表示一个图，包含多个节点。
 *
 * 参数列表：
 * - StateVec: 无需传入参数，只是定义了一个结构体。
 * - GraphNode: 需要传入节点编号，以创建一个 GraphNode 对象。
 * - Graph: 无需传入参数，只是定义了一个结构体。
 */
struct StateVec {
  std::vector<plansys2::Predicate> predicates;  // 存储谓词的向量
  std::vector<plansys2::Function> functions;    // 存储函数的向量
};

struct GraphNode {
  // 创建并返回一个指向 GraphNode 的智能指针
  using Ptr = std::shared_ptr<GraphNode>;
  static Ptr make_shared(int id) { return std::make_shared<GraphNode>(id); }

  int node_num;                                                      // 节点编号
  ActionStamped action;                                              // 动作

  std::set<std::tuple<GraphNode::Ptr, double, double>> input_arcs;   // 输入弧
  std::set<std::tuple<GraphNode::Ptr, double, double>> output_arcs;  // 输出弧

  explicit GraphNode(int id) : node_num(id) {}  // 构造函数，初始化节点编号
};

struct Graph {
  // 创建并返回一个指向 Graph 的智能指针
  using Ptr = std::shared_ptr<Graph>;
  static Ptr make_shared() { return std::make_shared<Graph>(); }

  std::list<GraphNode::Ptr> nodes;  // 节点列表
};

/**
 * @brief STNBTBuilder类是一个继承自BTBuilder的类，用于构建行为树。
 * @details STNBTBuilder类包含了构建STN（Simple Temporal Network）和BT（Behavior
 * Tree）所需的所有函数。 它还提供了获取行为树和dot图的方法，以及一些辅助函数。
 *          这个类主要用于nav2_planner组件中。
 *
 * 参数列表：
 * - bt_action_1: 行为树的起始动作名称，默认为空字符串。
 * - bt_action_2: 行为树的结束动作名称，默认为空字符串。
 * - precision: 时间精度，默认为3。
 */
class STNBTBuilder : public BTBuilder {
public:
  /**
   * @brief 构造函数，初始化DomainExpertClient和ProblemExpertClient。
   */
  STNBTBuilder();

  /**
   * @brief 初始化函数，设置行为树的起始动作、结束动作和时间精度。
   * @param bt_action_1 行为树的起始动作名称，默认为空字符串。
   * @param bt_action_2 行为树的结束动作名称，默认为空字符串。
   * @param precision 时间精度，默认为3。
   */
  void initialize(
      const std::string& bt_action_1 = "", const std::string& bt_action_2 = "", int precision = 3);

  /**
   * @brief 获取当前计划的行为树。
   * @param current_plan 当前计划。
   * @return 返回当前计划的行为树。
   */
  std::string get_tree(const plansys2_msgs::msg::Plan& current_plan);

  /**
   * @brief 获取当前计划的dot图。
   * @param action_map 动作映射表。
   * @param enable_legend 是否启用图例，默认为false。
   * @param enable_print_graph 是否打印图形，默认为false。
   * @return 返回当前计划的dot图。
   */
  std::string get_dotgraph(
      std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map,
      bool enable_legend = false,
      bool enable_print_graph = false);

protected:
  /**
   * @brief 构建STN（Simple Temporal Network）。
   * @param plan 计划。
   * @return 返回构建好的STN。
   */
  Graph::Ptr build_stn(const plansys2_msgs::msg::Plan& plan) const;

  /**
   * @brief 构建BT（Behavior Tree）。
   * @param stn STN。
   * @return 返回构建好的BT。
   */
  std::string build_bt(const Graph::Ptr stn) const;

  /**
   * @brief 初始化图。
   * @param plan 计划。
   * @return 返回初始化好的图。
   */
  Graph::Ptr init_graph(const plansys2_msgs::msg::Plan& plan) const;

  /**
   * @brief 获取计划中的所有动作。
   * @param plan 计划。
   * @return 返回计划中的所有动作。
   */
  std::vector<ActionStamped> get_plan_actions(const plansys2_msgs::msg::Plan& plan) const;

  /**
   * @brief 获取计划中发生的事件。
   * @param plan 计划。
   * @return 返回计划中发生的事件。
   */
  std::set<int> get_happenings(const plansys2_msgs::msg::Plan& plan) const;

  /**
   * @brief 获取指定时间的事件迭代器。
   * @param time 时间。
   * @param happenings 发生的事件。
   * @return 返回指定时间的事件迭代器。
   */
  std::set<int>::iterator get_happening(int time, const std::set<int>& happenings) const;

  /**
   * @brief 获取前一个事件迭代器。
   * @param time 时间。
   * @param happenings 发生的事件。
   * @return 返回前一个事件迭代器。
   */
  std::set<int>::iterator get_previous(int time, const std::set<int>& happenings) const;

  /**
   * @brief 获取简单计划。
   * @param plan 计划。
   * @return 返回简单计划。
   */
  std::multimap<int, ActionStamped> get_simple_plan(const plansys2_msgs::msg::Plan& plan) const;

  /**
   * @brief 获取状态。
   * @param happenings 发生的事件。
   * @param plan 简单计划。
   * @return 返回状态。
   */
  std::map<int, StateVec> get_states(
      const std::set<int>& happenings, const std::multimap<int, ActionStamped>& plan) const;

  /**
   * @brief 将状态转换为树。
   * @param preds 谓词。
   * @param funcs 函数。
   * @return 返回转换后的树。
   */
  plansys2_msgs::msg::Tree from_state(
      const std::vector<plansys2::Predicate>& preds,
      const std::vector<plansys2::Function>& funcs) const;

  /**
   * @brief 获取节点。
   * @param action 动作。
   * @param graph 图。
   * @return 返回节点列表。
   */
  std::vector<GraphNode::Ptr> get_nodes(const ActionStamped& action, const Graph::Ptr graph) const;

  /**
   * @brief 判断节点是否匹配动作。
   * @param node 节点。
   * @param action 动作。
   * @return 如果匹配，则返回true，否则返回false。
   */
  bool is_match(const GraphNode::Ptr node, const ActionStamped& action) const;

  /**
   * @brief 获取父节点。
   * @param action 动作。
   * @param plan 简单计划。
   * @param happenings 发生的事件。
   * @param states 状态。
   * @return 返回父节点列表。
   */
  std::vector<std::pair<int, ActionStamped>> get_parents(
      const std::pair<int, ActionStamped>& action,
      const std::multimap<int, ActionStamped>& plan,
      const std::set<int>& happenings,
      const std::map<int, StateVec>& states) const;

  /**
   * @brief 获取满足条件的动作。
   * @param action 动作。
   * @param plan 简单计划。
   * @param happenings 发生的事件。
   * @param states 状态。
   * @return 返回满足条件的动作列表。
   */
  std::vector<std::pair<int, ActionStamped>> get_satisfy(
      const std::pair<int, ActionStamped>& action,
      const std::multimap<int, ActionStamped>& plan,
      const std::set<int>& happenings,
      const std::map<int, StateVec>& states) const;

  /**
   * @brief 获取threat的动作。
   * @param action 动作。
   * @param plan 简单计划。
   * @param happenings 发生的事件。
   * @param states 状态。
   * @return 返回威胁的动作列表。
   */
  std::vector<std::pair<int, ActionStamped>> get_threat(
      const std::pair<int, ActionStamped>& action,
      const std::multimap<int, ActionStamped>& plan,
      const std::set<int>& happenings,
      const std::map<int, StateVec>& states) const;

  /**
   * @brief 判断是否可以应用动作。
   * @param action 动作。
   * @param plan 简单计划。
   * @param time 时间。
   * @param state 状态。
   * @return 如果可以应用，则返回true，否则返回false。
   */
  bool can_apply(
      const std::pair<int, ActionStamped>& action,
      const std::multimap<int, ActionStamped>& plan,
      const int& time,
      StateVec& state) const;

  /**
   * @brief 获取状态差异。
   * @param X_1 状态1。
   * @param X_2 状态2。
   * @return 返回状态差异。
   */
  StateVec get_diff(const StateVec& X_1, const StateVec& X_2) const;

  /**
   * @brief 获取状态交集。
   * @param X_1 状态1。
   * @param X_2 状态2。
   * @return 返回状态交集。
   */
  StateVec get_intersection(const StateVec& X_1, const StateVec& X_2) const;

  /**
   * @brief 获取条件树。
   * @param action 动作。
   * @return 返回条件树。
   */
  plansys2_msgs::msg::Tree get_conditions(const ActionStamped& action) const;

  /**
   * @brief 获取影响树。
   * @param action 动作。
   * @return 返回影响树。
   */
  plansys2_msgs::msg::Tree get_effects(const ActionStamped& action) const;

  /**
   * @brief 从当前节点到上一个节点的路径是否需要剪枝
   * @param current 当前节点
   * @param previous 上一个节点
   * @return bool 返回是否需要剪枝
   */
  void prune_paths(GraphNode::Ptr current, GraphNode::Ptr previous) const;

  /**
   * @brief 检查从当前节点到上一个节点的路径是否合法
   * @param current 当前节点
   * @param previous 上一个节点
   * @return bool 返回路径是否合法
   */
  bool check_paths(GraphNode::Ptr current, GraphNode::Ptr previous) const;

  /**
   * @brief 获取从当前节点到根节点的执行流程
   * @param node 当前节点
   * @param parent 父节点
   * @param used 已经使用过的节点
   * @param level 当前节点所在的层数
   * @return std::string 返回执行流程字符串
   */
  std::string get_flow(
      const GraphNode::Ptr node,
      const GraphNode::Ptr parent,
      std::set<GraphNode::Ptr>& used,
      const int& level) const;

  /**
   * @brief 生成开始执行块的语句
   * @param node 当前节点
   * @param parent 父节点
   * @param l 当前节点所在的层数
   * @return std::string 返回开始执行块的语句
   */
  std::string start_execution_block(
      const GraphNode::Ptr node, const GraphNode::Ptr parent, const int& l) const;

  /**
   * @brief 生成结束执行块的语句
   * @param node 当前节点
   * @param parent 父节点
   * @param l 当前节点所在的层数
   * @return std::string 返回结束执行块的语句
   */
  std::string end_execution_block(
      const GraphNode::Ptr node, const GraphNode::Ptr parent, const int& l) const;

  /**
   * @brief 获取从当前节点到根节点的dot图中的边
   * @param node 当前节点
   * @param edges 存储边的set
   */
  void get_flow_dotgraph(GraphNode::Ptr node, std::set<std::string>& edges);

  /**
   * @brief 获取当前节点在dot图中的字符串表示
   * @param node 当前节点
   * @param action_map 动作映射表
   * @return std::string 返回当前节点在dot图中的字符串表示
   */
  std::string get_node_dotgraph(
      GraphNode::Ptr node, std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map);

  /**
   * @brief 获取动作的状态
   * @param action 动作
   * @param action_map 动作映射表
   * @return ActionExecutor::Status 返回动作的状态
   */
  ActionExecutor::Status get_action_status(
      ActionStamped action, std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map);

  /**
   * @brief 添加dot图的图例
   * @param level_counter 层数计数器
   * @param node_counter 节点计数器
   * @return std::string 返回dot图的图例
   */
  std::string add_dot_graph_legend(int level_counter, int node_counter);

  /**
   * @brief 打印规划图
   * @param graph 规划图指针
   */
  void print_graph(const plansys2::Graph::Ptr graph) const;

  /**
   * @brief 打印节点
   * @param node 节点指针
   * @param level 节点所在的层数
   */
  void print_node(const GraphNode::Ptr node, int level) const;

  /**
   * @brief 替换字符串中的子串
   * @param str 待替换的字符串
   * @param from 被替换的子串
   * @param to 替换成的子串
   */
  void replace(std::string& str, const std::string& from, const std::string& to) const;

  /**
   * @brief 判断当前边是否是结束动作
   * @param edge 边
   * @param action 动作
   * @return bool 返回当前边是否是结束动作
   */
  bool is_end(
      const std::tuple<GraphNode::Ptr, double, double>& edge, const ActionStamped& action) const;

  /**
   * @brief 获取时间戳字符串
   * @param level 当前节点所在的层数
   * @return std::string 返回时间戳字符串
   */
  std::string t(const int& level) const;

  /**
   * @brief 领域专家客户端指针
   */
  std::shared_ptr<plansys2::DomainExpertClient> domain_client_;

  /**
   * @brief 问题专家客户端指针
   */
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client_;

  /**
   * @brief 规划图指针
   */
  Graph::Ptr stn_;

  /**
   * @brief 开始动作名称
   */
  std::string bt_start_action_;

  /**
   * @brief 结束动作名称
   */
  std::string bt_end_action_;

  /**
   * @brief 动作时间精度
   */
  int action_time_precision_;
};

}  // namespace plansys2

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(plansys2::STNBTBuilder, plansys2::BTBuilder)

#endif  // PLANSYS2_EXECUTOR__BT_BUILDER_PLUGINS__STN_BT_BUILDER_HPP_
