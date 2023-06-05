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

#include "plansys2_executor/bt_builder_plugins/simple_bt_builder.hpp"

#include <algorithm>
#include <list>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "plansys2_pddl_parser/Utils.h"
#include "plansys2_problem_expert/Utils.hpp"
#include "rclcpp/rclcpp.hpp"

namespace plansys2 {

/**
 * @brief SimpleBTBuilder类的构造函数，初始化domain_client_和problem_client_成员变量
 */
SimpleBTBuilder::SimpleBTBuilder() {
  domain_client_ = std::make_shared<plansys2::DomainExpertClient>();
  problem_client_ = std::make_shared<plansys2::ProblemExpertClient>();
}

/**
 * @brief 初始化SimpleBTBuilder对象的bt_action_成员变量
 * @param bt_action_1 第一个备选的行为树模板字符串
 * @param bt_action_2 第二个备选的行为树模板字符串
 * @param precision 精度参数
 * @details 如果bt_action_1不为空，则将其赋值给bt_action_；否则，使用默认的行为树模板字符串。
 */
void SimpleBTBuilder::initialize(
    const std::string& bt_action_1, const std::string& bt_action_2, int precision) {
  if (bt_action_1 != "") {
    bt_action_ = bt_action_1;
  } else {
    bt_action_ =
        R""""(<Sequence name="ACTION_ID">
WAIT_PREV_ACTIONS
  <ApplyAtStartEffect action="ACTION_ID"/>
  <ReactiveSequence name="ACTION_ID">
    <CheckOverAllReq action="ACTION_ID"/>
    <ExecuteAction action="ACTION_ID"/>
  </ReactiveSequence>
  <CheckAtEndReq action="ACTION_ID"/>
  <ApplyAtEndEffect action="ACTION_ID"/>
</Sequence>
)"""";
  }
}

/**
 * @brief 判断一个动作是否可执行
 * @param action 待判断的动作
 * @param predicates 所有谓词的向量
 * @param functions 所有函数的向量
 * @return 如果该动作的所有前置条件都被满足，则返回true；否则，返回false。
 */
bool SimpleBTBuilder::is_action_executable(
    const ActionStamped& action,
    std::vector<plansys2::Predicate>& predicates,
    std::vector<plansys2::Function>& functions) const {
  return check(action.action->at_start_requirements, predicates, functions) &&
         check(action.action->at_end_requirements, predicates, functions) &&
         check(action.action->over_all_requirements, predicates, functions);
}

/**
 * @brief 获取满足需求的节点
 * @param requirement 需求树
 * @param node 节点
 * @param current 当前节点
 * @details 1. 判断当前节点是否为目标节点，如果是则返回空指针。
 *          2. 获取应用效果之前的状态。
 *          3. 判断是否在应用效果之前就已经满足了需求。
 *          4. 应用效果。
 *          5. 判断是否在应用效果之后满足了需求。
 *          6. 如果在应用效果之后满足了需求并且在应用效果之前没有满足，则返回该节点。
 *          7. 遍历其它节点。
 *          8. 返回满足需求的节点。
 */
GraphNode::Ptr SimpleBTBuilder::get_node_satisfy(
    const plansys2_msgs::msg::Tree& requirement,
    const GraphNode::Ptr& node,
    const GraphNode::Ptr& current) {
  if (node == current) {  // 如果当前节点为目标节点，则返回空指针
    return nullptr;
  }

  GraphNode::Ptr ret = nullptr;

  // Get the state prior to applying the effects
  auto predicates = node->predicates;  // 获取应用效果之前的谓词
  auto functions = node->functions;    // 获取应用效果之前的函数

  // Is the requirement satisfied before applying the effects?
  bool satisfied_before =
      check(requirement, predicates, functions);  // 判断是否在应用效果之前就已经满足了需求

  // Apply the effects
  apply(node->action.action->at_start_effects, predicates, functions);  // 应用开始效果
  apply(node->action.action->at_end_effects, predicates, functions);    // 应用结束效果

  // Is the requirement satisfied after applying the effects?
  bool satisfied_after =
      check(requirement, predicates, functions);  // 判断是否在应用效果之后满足了需求

  if (satisfied_after &&
      !satisfied_before) {  // 如果在应用效果之后满足了需求并且在应用效果之前没有满足，则返回该节点
    ret = node;
  }

  // Traverse the rest of the graph.
  for (const auto& arc : node->out_arcs) {  // 遍历其它节点
    auto node_ret = get_node_satisfy(requirement, arc, current);

    if (node_ret != nullptr) {
      ret = node_ret;
    }
  }

  return ret;  // 返回满足需求的节点
}

/**
 * @brief 获取与当前节点存在矛盾的节点列表
 * @param node 要检查的节点
 * @param current 当前节点
 * @param contradictions 与当前节点存在矛盾的节点列表
 * @details 对给定的节点进行递归遍历，获取与当前节点存在矛盾的节点列表。
 * 遍历过程中，会对节点的谓词和函数进行处理，并检查是否满足执行当前动作的要求。
 * 如果满足要求，则应用当前动作的效果，并检查是否存在矛盾。
 * 如果存在矛盾，则将该节点添加到矛盾列表中。
 */
void SimpleBTBuilder::get_node_contradict(
    const GraphNode::Ptr& node,
    const GraphNode::Ptr& current,
    std::list<GraphNode::Ptr>& contradictions) {
  if (node == current) {
    return;
  }

  // 获取应用效果之前的状态
  auto predicates = node->predicates;
  auto functions = node->functions;

  // 检查是否满足执行当前动作的要求
  if (is_action_executable(current->action, predicates, functions)) {
    // 应用当前动作的效果
    apply(current->action.action->at_start_effects, predicates, functions);

    // 检查是否存在矛盾
    if (!is_action_executable(node->action, predicates, functions)) {
      contradictions.push_back(node);
    }
  }

  // 遍历图的其余部分
  for (const auto& arc : node->out_arcs) {
    get_node_contradict(arc, current, contradictions);
  }
}

/**
 * @brief 判断给定动作是否可以与一组节点并行执行
 * @param action 要检查的动作
 * @param predicates 动作执行前的谓词列表
 * @param functions 动作执行前的函数列表
 * @param nodes 给定的节点列表
 * @return 如果可以并行执行，则返回 true，否则返回 false
 * @details 对给定的动作和节点进行处理，判断是否可以并行执行。
 * 首先，应用新动作的“at start”效果。然后，检查输入集合中所有动作的要求是否满足。
 * 接着，逐个应用输入集合中的动作效果，并检查新动作的要求是否满足。
 * 如果都满足，则返回 true，否则返回 false。
 */
bool SimpleBTBuilder::is_parallelizable(
    const plansys2::ActionStamped& action,
    const std::vector<plansys2::Predicate>& predicates,
    const std::vector<plansys2::Function>& functions,
    const std::list<GraphNode::Ptr>& nodes) const {
  // 应用新动作的“at start”效果
  auto preds = predicates;
  auto funcs = functions;
  apply(action.action->at_start_effects, preds, funcs);

  // 检查输入集合中所有动作的要求是否满足
  for (const auto& other : nodes) {
    if (!is_action_executable(other->action, preds, funcs)) {
      return false;
    }
  }

  // 逐个应用输入集合中的动作效果，并检查新动作的要求是否满足
  for (const auto& other : nodes) {
    // 应用动作的“at start”效果
    preds = predicates;
    funcs = functions;
    apply(other->action.action->at_start_effects, preds, funcs);

    // 检查新动作的要求是否满足
    if (!is_action_executable(action, preds, funcs)) {
      return false;
    }
  }

  return true;
}

/**
 * @brief 获取满足需求的节点
 * @param requirement 需求树
 * @param graph 图
 * @param current 当前节点
 * @return 满足需求的节点
 * @details 在图中查找满足需求的节点，如果有多个则返回最后一个。
 */
GraphNode::Ptr SimpleBTBuilder::get_node_satisfy(
    const plansys2_msgs::msg::Tree& requirement,
    const Graph::Ptr& graph,
    const GraphNode::Ptr& current) {
  GraphNode::Ptr ret;
  for (const auto& root : graph->roots) {
    auto node_satisfy = get_node_satisfy(requirement, root, current);
    if (node_satisfy != nullptr) {
      ret = node_satisfy;
    }
  }

  return ret;
}

/**
 * @brief 获取与当前节点矛盾的节点列表
 * @param graph 图
 * @param current 当前节点
 * @return 矛盾节点列表
 * @details 在图中查找与当前节点矛盾的节点列表。
 */
std::list<GraphNode::Ptr> SimpleBTBuilder::get_node_contradict(
    const Graph::Ptr& graph, const GraphNode::Ptr& current) {
  std::list<GraphNode::Ptr> ret;

  for (const auto& root : graph->roots) {
    get_node_contradict(root, current, ret);
  }

  return ret;
}

/**
 * @brief 获取根节点列表
 * @param action_sequence 动作序列
 * @param predicates 谓词列表
 * @param functions 函数列表
 * @param node_counter 节点计数器
 * @return 根节点列表
 * @details 在动作序列中查找可执行且可并行的动作，将其构造为根节点。
 */
std::list<GraphNode::Ptr> SimpleBTBuilder::get_roots(
    std::vector<plansys2::ActionStamped>& action_sequence,
    std::vector<plansys2::Predicate>& predicates,
    std::vector<plansys2::Function>& functions,
    int& node_counter) {
  std::list<GraphNode::Ptr> ret;

  auto it = action_sequence.begin();
  while (it != action_sequence.end()) {
    const auto& action = *it;
    if (is_action_executable(action, predicates, functions) &&
        is_parallelizable(action, predicates, functions, ret)) {
      auto new_root = GraphNode::make_shared();
      new_root->action = action;
      new_root->node_num = node_counter++;
      new_root->level_num = 0;
      new_root->predicates = predicates;
      new_root->functions = functions;

      ret.push_back(new_root);
      it = action_sequence.erase(it);
    } else {
      break;
    }
  }

  return ret;
}

/**
 * @brief 移除已存在的要求
 * @param requirements 要求列表
 * @param predicates 断言列表
 * @param functions 函数列表
 * @details 检查要求列表中的每个要求是否都在断言列表和函数列表中，如果是，则从要求列表中删除该要求。
 */
void SimpleBTBuilder::remove_existing_requirements(
    std::vector<plansys2_msgs::msg::Tree>& requirements,
    std::vector<plansys2::Predicate>& predicates,
    std::vector<plansys2::Function>& functions) const {
  auto it = requirements.begin();
  while (it != requirements.end()) {
    if (check(*it, predicates, functions)) {
      it = requirements.erase(it);
    } else {
      ++it;
    }
  }
}

/**
 * @brief 向后修剪
 * @param new_node 新节点
 * @param node_satisfy 满足条件的节点
 * @details 从满足条件的节点开始向根节点进行修剪，将新节点与满足条件的节点之间的边删除。
 */
void SimpleBTBuilder::prune_backwards(GraphNode::Ptr new_node, GraphNode::Ptr node_satisfy) {
  // Repeat prune to the roots
  for (auto& in : node_satisfy->in_arcs) {
    prune_backwards(new_node, in);
  }

  auto it = node_satisfy->out_arcs.begin();
  while (it != node_satisfy->out_arcs.end()) {
    if (*it == new_node) {
      new_node->in_arcs.remove(node_satisfy);
      it = node_satisfy->out_arcs.erase(it);
    } else {
      ++it;
    }
  }
}

/**
 * @brief 向前修剪
 * @param current 当前节点
 * @param used_nodes 已使用的节点列表
 * @details 从当前节点开始向后遍历，将已经使用过的节点之间的边删除。
 */
void SimpleBTBuilder::prune_forward(GraphNode::Ptr current, std::list<GraphNode::Ptr>& used_nodes) {
  auto it = current->out_arcs.begin();
  while (it != current->out_arcs.end()) {
    if (std::find(used_nodes.begin(), used_nodes.end(), *it) != used_nodes.end()) {
      it = current->out_arcs.erase(it);
    } else {
      prune_forward(*it, used_nodes);
      used_nodes.push_back(*it);

      ++it;
    }
  }
}

/**
 * @brief 获取给定节点的状态信息
 * @param node 给定的节点
 * @param used_nodes 已经使用过的节点列表
 * @param predicates 谓词列表
 * @param functions 函数列表
 * @details
 * 该函数通过遍历图形结构，获取给定节点到根节点的所有节点，并将这些节点的状态信息存储在谓词列表和函数列表中。其中，谓词表示状态信息，函数表示状态变量。
 */
void SimpleBTBuilder::get_state(
    const GraphNode::Ptr& node,
    std::list<GraphNode::Ptr>& used_nodes,
    std::vector<plansys2::Predicate>& predicates,
    std::vector<plansys2::Function>& functions) const {
  // 遍历图形结构到根节点
  for (auto& in : node->in_arcs) {
    if (std::find(used_nodes.begin(), used_nodes.end(), in) == used_nodes.end()) {
      // 如果当前节点没有被使用过，则递归调用 get_state 函数
      get_state(in, used_nodes, predicates, functions);
      // 将当前节点的起始效果和结束效果应用到谓词列表和函数列表中
      apply(in->action.action->at_start_effects, predicates, functions);
      apply(in->action.action->at_end_effects, predicates, functions);
      used_nodes.push_back(in);
    }
  }
}

/**
 * @brief 获取规划图
 * @param current_plan 当前规划
 * @return 规划图
 * @details
 * 1. 初始化节点计数器和层级计数器
 * 2. 创建一个空的规划图
 * 3. 获取当前规划中的所有行动序列、谓词和函数
 * 4. 获取可以并行运行的根行动
 * 5. 构建剩余的规划图
 * 6. 对于每个新节点，找到满足其要求的节点，并将其连接到该节点
 * 7. 查找矛盾的并行行动，并将其连接到该节点
 * 8. 计算到新节点的状态，不应用新节点的效果
 * 9. 检查未满足的要求是否由初始状态满足
 * 10. 返回规划图
 */
Graph::Ptr SimpleBTBuilder::get_graph(const plansys2_msgs::msg::Plan& current_plan) {
  int node_counter = 0;
  int level_counter = 0;
  auto graph = Graph::make_shared();

  auto action_sequence = get_plan_actions(current_plan);
  auto predicates = problem_client_->getPredicates();
  auto functions = problem_client_->getFunctions();

  // Get root actions that can be run in parallel
  graph->roots = get_roots(action_sequence, predicates, functions, node_counter);

  // Build the rest of the graph
  while (!action_sequence.empty()) {
    auto new_node = GraphNode::make_shared();
    new_node->action = *action_sequence.begin();
    new_node->node_num = node_counter++;
    float time = new_node->action.time;

    auto level = graph->levels.find(time);
    if (level == graph->levels.end()) {
      level_counter++;
      std::list<GraphNode::Ptr> new_level;
      new_level.push_back(new_node);
      graph->levels.insert({time, new_level});
    } else {
      level->second.push_back(new_node);
    }
    new_node->level_num = level_counter;

    // 获取新节点的所有要求
    std::vector<plansys2_msgs::msg::Tree> at_start_requirements =
        parser::pddl::getSubtrees(new_node->action.action->at_start_requirements);
    std::vector<plansys2_msgs::msg::Tree> over_all_requirements =
        parser::pddl::getSubtrees(new_node->action.action->over_all_requirements);
    std::vector<plansys2_msgs::msg::Tree> at_end_requirements =
        parser::pddl::getSubtrees(new_node->action.action->at_end_requirements);

    std::vector<plansys2_msgs::msg::Tree> requirements;
    requirements.insert(
        std::end(requirements), std::begin(at_start_requirements), std::end(at_start_requirements));
    requirements.insert(
        std::end(requirements), std::begin(over_all_requirements), std::end(over_all_requirements));
    requirements.insert(
        std::end(requirements), std::begin(at_end_requirements), std::end(at_end_requirements));

    // Look for satisfying nodes
    // A satisfying node is a node with an effect that satisfies a requirement of the new node
    auto it = requirements.begin();
    while (it != requirements.end()) {
      auto parent = get_node_satisfy(*it, graph, new_node);
      if (parent != nullptr) {
        prune_backwards(new_node, parent);

        // Create the connections to the parent node
        if (std::find(new_node->in_arcs.begin(), new_node->in_arcs.end(), parent) ==
            new_node->in_arcs.end()) {
          new_node->in_arcs.push_back(parent);
        }
        if (std::find(parent->out_arcs.begin(), parent->out_arcs.end(), new_node) ==
            parent->out_arcs.end()) {
          parent->out_arcs.push_back(new_node);
        }

        it = requirements.erase(it);
      } else {
        ++it;
      }
    }

    // Look for contradicting parallel actions
    // A1 and A2 cannot run in parallel if the effects of A1 contradict the requirements of A2
    auto contradictions = get_node_contradict(graph, new_node);
    for (const auto parent : contradictions) {
      prune_backwards(new_node, parent);

      // Create the connections to the parent node
      if (std::find(new_node->in_arcs.begin(), new_node->in_arcs.end(), parent) ==
          new_node->in_arcs.end()) {
        new_node->in_arcs.push_back(parent);
      }
      if (std::find(parent->out_arcs.begin(), parent->out_arcs.end(), new_node) ==
          parent->out_arcs.end()) {
        parent->out_arcs.push_back(new_node);
      }
    }

    // Compute the state up to the new node
    // The effects of the new node are not applied
    std::list<GraphNode::Ptr> used_nodes;
    predicates = problem_client_->getPredicates();
    functions = problem_client_->getFunctions();
    get_state(new_node, used_nodes, predicates, functions);
    new_node->predicates = predicates;
    new_node->functions = functions;

    // Check any requirements that do not have satisfying nodes.
    // These should be satisfied by the initial state.
    remove_existing_requirements(requirements, predicates, functions);
    for (const auto& req : requirements) {
      std::cerr << "[ERROR] requirement not met: [" << parser::pddl::toString(req) << "]"
                << std::endl;
    }
    assert(requirements.empty());

    action_sequence.erase(action_sequence.begin());
  }

  return graph;
}

/**
 * @brief 获取行动树
 * @param current_plan 当前计划
 * @details 根据当前计划获取行动图，然后通过剪枝算法对图进行处理，最后生成行动树并返回。
 */
std::string SimpleBTBuilder::get_tree(const plansys2_msgs::msg::Plan& current_plan) {
  // 获取行动图
  graph_ = get_graph(current_plan);

  std::list<GraphNode::Ptr> used_actions;
  // 对每一个根节点进行向前剪枝
  for (auto& root : graph_->roots) {
    prune_forward(root, used_actions);
  }

  std::list<std::string> used_nodes;

  if (graph_->roots.size() > 1) {
    // 生成并行行动树
    bt_ = std::string("<root main_tree_to_execute=\"MainTree\">\n") + t(1) +
          "<BehaviorTree ID=\"MainTree\">\n" + t(2) + "<Parallel success_threshold=\"" +
          std::to_string(graph_->roots.size()) + "\" failure_threshold=\"1\">\n";

    for (const auto& node : graph_->roots) {
      // 获取流程树
      bt_ = bt_ + get_flow_tree(node, used_nodes, 3);
    }

    bt_ = bt_ + t(2) + "</Parallel>\n" + t(1) + "</BehaviorTree>\n</root>\n";
  } else {
    // 生成顺序行动树
    bt_ = std::string("<root main_tree_to_execute=\"MainTree\">\n") + t(1) +
          "<BehaviorTree ID=\"MainTree\">\n";

    bt_ = bt_ + get_flow_tree(*graph_->roots.begin(), used_nodes, 2);

    bt_ = bt_ + t(1) + "</BehaviorTree>\n</root>\n";
  }

  return bt_;
}

/**
 * @brief 获取用于可视化的dot图
 * @param action_map 存储动作执行信息的map指针
 * @param enable_legend 是否添加图例
 * @param enable_print_graph 是否打印图形
 * @details
 * 1. 如果enable_print_graph为true，则打印graph_。
 * 2. 创建xdot图。
 * 3. 定义所有级别和节点。
 * 4. 定义边缘。
 * 5. 如果enable_legend为true，则添加图例。
 * 6. 返回xdot图字符串。
 */
std::string SimpleBTBuilder::get_dotgraph(
    std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map,
    bool enable_legend,
    bool enable_print_graph) {
  if (enable_print_graph) {
    print_graph(graph_);
  }

  // 创建xdot图
  std::stringstream ss;
  ss << "digraph plan {\n";

  int tab_level = 1;
  // dotgraph格式选项
  ss << t(tab_level);
  ss << "node[shape=box];\n";
  ss << t(tab_level);
  ss << "rankdir=TB;\n";

  // 定义所有级别和节点
  ss << t(tab_level);
  ss << "subgraph cluster_0 {\n";

  tab_level = 2;
  ss << t(tab_level);
  ss << "label = \"Time: 0.0\";\n";
  ss << t(tab_level);
  ss << "style = rounded;\n";
  ss << t(tab_level);
  ss << "color = yellow3;\n";
  ss << t(tab_level);
  ss << "bgcolor = lemonchiffon;\n";
  ss << t(tab_level);
  ss << "labeljust = l;\n";

  tab_level = 3;
  for (auto& node : graph_->roots) {
    ss << get_node_dotgraph(node, action_map, tab_level);
  }
  tab_level = 2;

  ss << t(tab_level);
  ss << "}\n";

  int max_level = 0;
  int max_node = 0;
  for (auto& level : graph_->levels) {
    if (!level.second.empty()) {
      ss << t(tab_level);
      ss << "subgraph cluster_" << level.second.front()->level_num << " {\n";
      max_level = std::max(max_level, level.second.front()->level_num);

      tab_level = 2;
      ss << t(tab_level);
      ss << "label = \"Time: " << level.second.front()->action.time << "\";\n";
      ss << t(tab_level);
      ss << "style = rounded;\n";
      ss << t(tab_level);
      ss << "color = yellow3;\n";
      ss << t(tab_level);
      ss << "bgcolor = lemonchiffon;\n";
      ss << t(tab_level);
      ss << "labeljust = l;\n";

      tab_level = 3;
      for (auto& node : level.second) {
        max_node = std::max(max_node, node->node_num);
        ss << get_node_dotgraph(node, action_map, tab_level);
      }
      tab_level = 2;

      ss << t(tab_level);
      ss << "}\n";
    }
  }

  // 定义边缘
  std::set<std::string> edges;
  for (const auto& graph_root : graph_->roots) {
    get_flow_dotgraph(graph_root, edges);
  }

  tab_level = 1;
  for (const auto& edge : edges) {
    ss << t(tab_level) << edge;
  }

  if (enable_legend) {
    max_level++;
    max_node++;
    addDotGraphLegend(ss, tab_level, max_level, max_node);
  }

  ss << "}";

  return ss.str();
}

/**
 * @brief 获取行为树的流程树
 * @param node 行为树中的节点
 * @param used_nodes 已使用的节点列表
 * @param level 当前节点的层级
 * @return 返回流程树字符串
 * @details 根据输入的行为树节点，递归生成对应的流程树字符串。
 *          如果该节点已经被使用，则返回 WaitAction 字符串；
 *          否则将该节点加入已使用列表，并根据子节点数量生成 Sequence 或 Parallel 节点。
 *          对于 Sequence 节点，依次执行 execution_block 和子节点的 get_flow_tree 函数；
 *          对于 Parallel 节点，先执行 execution_block，然后并行执行子节点的 get_flow_tree 函数。
 */
std::string SimpleBTBuilder::get_flow_tree(
    GraphNode::Ptr node, std::list<std::string>& used_nodes, int level) {
  std::string ret;
  int l = level;

  // 构造 action_id 字符串
  const std::string action_id = "(" + parser::pddl::nameActionsToString(node->action.action) +
                                "):" + std::to_string(static_cast<int>(node->action.time * 1000));

  // 如果该节点已经被使用，则返回 WaitAction 字符串
  if (std::find(used_nodes.begin(), used_nodes.end(), action_id) != used_nodes.end()) {
    return t(l) + "<WaitAction action=\"" + action_id + "\"/>\n";
  }

  // 将该节点加入已使用列表
  used_nodes.push_back(action_id);

  // 根据子节点数量生成 Sequence 或 Parallel 节点
  if (node->out_arcs.size() == 0) {
    ret = ret + execution_block(node, l);
  } else if (node->out_arcs.size() == 1) {
    ret = ret + t(l) + "<Sequence name=\"" + action_id + "\">\n";
    ret = ret + execution_block(node, l + 1);

    for (const auto& child_node : node->out_arcs) {
      ret = ret + get_flow_tree(child_node, used_nodes, l + 1);
    }

    ret = ret + t(l) + "</Sequence>\n";
  } else {
    ret = ret + t(l) + "<Sequence name=\"" + action_id + "\">\n";
    ret = ret + execution_block(node, l + 1);

    ret = ret + t(l + 1) + "<Parallel success_threshold=\"" +
          std::to_string(node->out_arcs.size()) + "\" failure_threshold=\"1\">\n";

    for (const auto& child_node : node->out_arcs) {
      ret = ret + get_flow_tree(child_node, used_nodes, l + 2);
    }

    ret = ret + t(l + 1) + "</Parallel>\n";
    ret = ret + t(l) + "</Sequence>\n";
  }

  return ret;
}

/**
 * @brief 遍历节点的出边，获取流程图中的边
 * @param node 节点
 * @param edges 边集合
 * @details
 * 对于节点的每一条出边，将其转化为字符串形式，并加入到边集合中。然后递归地对该出边指向的节点进行同样的操作。
 */
void SimpleBTBuilder::get_flow_dotgraph(GraphNode::Ptr node, std::set<std::string>& edges) {
  for (const auto& arc : node->out_arcs) {
    std::string edge =
        std::to_string(node->node_num) + "->" + std::to_string(arc->node_num) + ";\n";
    edges.insert(edge);
    get_flow_dotgraph(arc, edges);
  }
}

/**
 * @brief 获取节点在流程图中的表示形式
 * @param node 节点
 * @param action_map 动作映射表
 * @param level 节点所处的层数
 * @return 节点在流程图中的表示形式
 * @details
 * 根据节点的信息和动作的状态，生成节点在流程图中的表示形式。其中包括节点的编号、标签、填充颜色等信息。
 */
std::string SimpleBTBuilder::get_node_dotgraph(
    GraphNode::Ptr node,
    std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map,
    int level) {
  std::stringstream ss;
  ss << t(level);
  ss << node->node_num << " [label=\"" << parser::pddl::nameActionsToString(node->action.action)
     << "\"";
  ss << "labeljust=c,style=filled";

  auto status = get_action_status(node->action, action_map);
  switch (status) {
    case ActionExecutor::RUNNING:
      ss << ",color=blue,fillcolor=skyblue";
      break;
    case ActionExecutor::SUCCESS:
      ss << ",color=green4,fillcolor=seagreen2";
      break;
    case ActionExecutor::FAILURE:
      ss << ",color=red,fillcolor=pink";
      break;
    case ActionExecutor::CANCELLED:
      ss << ",color=red,fillcolor=pink";
      break;
    case ActionExecutor::IDLE:
    case ActionExecutor::DEALING:
    default:
      ss << ",color=yellow3,fillcolor=lightgoldenrod1";
      break;
  }
  ss << "];\n";
  return ss.str();
}

/**
 * @brief 获取动作的状态
 * @param action_stamped 动作及其时间戳
 * @param action_map 动作映射表
 * @return 动作的状态
 * @details
 * 根据动作在动作映射表中的索引，获取该动作的执行器并返回其内部状态。如果该动作没有对应的执行器，则返回
 * IDLE 状态。
 */
ActionExecutor::Status SimpleBTBuilder::get_action_status(
    ActionStamped action_stamped,
    std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map) {
  auto index = "(" + parser::pddl::nameActionsToString(action_stamped.action) +
               "):" + std::to_string(static_cast<int>(action_stamped.time * 1000));
  if ((*action_map)[index].action_executor) {
    return (*action_map)[index].action_executor->get_internal_status();
  } else {
    return ActionExecutor::IDLE;
  }
}

/**
 * @brief 添加图例到dot格式的图形中
 * @param ss 字符串流，用于构建dot格式的图形
 * @param tab_level 缩进级别
 * @param level_counter 当前层级计数器
 * @param node_counter 当前节点计数器
 * @details
 * 1. 根据传入的参数构建图例子图，包括完成、失败、当前和未来四种动作状态；
 * 2. 将构建好的图例子图添加到dot格式的图形字符串流中。
 */
void SimpleBTBuilder::addDotGraphLegend(
    std::stringstream& ss, int tab_level, int level_counter, int node_counter) {
  // 初始化图例计数器
  int legend_counter = level_counter;
  int legend_node_counter = node_counter;

  // 添加图例子图的头部信息
  ss << t(tab_level);
  ss << "subgraph cluster_" << legend_counter++ << " {\n";
  tab_level++;
  ss << t(tab_level);
  ss << "label = \"Legend\";\n";

  // 添加Plan Timestep信息
  ss << t(tab_level);
  ss << "subgraph cluster_" << legend_counter++ << " {\n";
  tab_level++;
  ss << t(tab_level);
  ss << "label = \"Plan Timestep (sec): X.X\";\n";
  ss << t(tab_level);
  ss << "style = rounded;\n";
  ss << t(tab_level);
  ss << "color = yellow3;\n";
  ss << t(tab_level);
  ss << "bgcolor = lemonchiffon;\n";

  // 添加完成、失败、当前和未来四种动作状态节点
  ss << t(tab_level);
  ss << "labeljust = l;\n";
  ss << t(tab_level);
  ss << legend_node_counter++
     << " [label=\n\"Finished "
        "action\n\",labeljust=c,style=filled,color=green4,fillcolor=seagreen2];\n";
  ss << t(tab_level);
  ss << legend_node_counter++
     << " [label=\n\"Failed action\n\",labeljust=c,style=filled,color=red,fillcolor=pink];\n";
  ss << t(tab_level);
  ss << legend_node_counter++
     << " [label=\n\"Current action\n\",labeljust=c,style=filled,color=blue,fillcolor=skyblue];\n";
  ss << t(tab_level);
  ss << legend_node_counter++ << " [label=\n\"Future action\n\",labeljust=c,style=filled,"
     << "color=yellow3,fillcolor=lightgoldenrod1];\n";

  // 添加图例子图的尾部信息
  tab_level--;
  ss << t(tab_level);
  ss << "}\n";

  // 添加节点之间的连线
  ss << t(tab_level);
  for (int i = node_counter; i < legend_node_counter; i++) {
    if (i > node_counter) {
      ss << "->";
    }
    ss << i;
  }
  ss << " [style=invis];\n";

  // 添加图例子图的尾部信息
  tab_level--;
  ss << t(tab_level);
  ss << "}\n";
}

/**
 * @brief 生成行为树的执行块
 * @param node GraphNode类型，表示当前节点
 * @param l int类型，表示当前节点的层数
 * @return std::string类型，表示生成的执行块
 * @details 根据传入的GraphNode类型的节点信息，生成对应的行为树执行块。
 *          执行块中包含了该节点的动作信息、等待前置动作信息等。
 */
std::string SimpleBTBuilder::execution_block(const GraphNode::Ptr& node, int l) {
  const auto& action = node->action;  // 获取当前节点的动作信息
  std::string ret;
  std::string ret_aux = bt_action_;   // 获取行为树执行块的模板
  const std::string action_id =
      "(" + parser::pddl::nameActionsToString(action.action) +
      "):" + std::to_string(static_cast<int>(action.time * 1000));  // 获取当前节点的动作ID

  std::string wait_actions;
  for (const auto& previous_node : node->in_arcs) {  // 遍历当前节点的所有前置节点
    const std::string parent_action_id =
        "(" + parser::pddl::nameActionsToString(previous_node->action.action) + "):" +
        std::to_string(
            static_cast<int>(previous_node->action.time * 1000));  // 获取前置节点的动作ID
    wait_actions = wait_actions + t(1) + "<WaitAction action=\"" + parent_action_id +
                   "\"/>";  // 生成等待前置动作的XML标签

    if (previous_node !=
        *node->in_arcs.rbegin()) {  // 如果当前前置节点不是最后一个，则在标签末尾添加换行符
      wait_actions = wait_actions + "\n";
    }
  }

  replace(ret_aux, "ACTION_ID", action_id);  // 将模板中的占位符替换为实际的动作ID
  replace(
      ret_aux, "WAIT_PREV_ACTIONS", wait_actions);  // 将模板中的占位符替换为实际的等待前置动作信息

  std::istringstream f(ret_aux);
  std::string line;
  while (std::getline(f, line)) {  // 将生成的XML标签按行添加到执行块中
    if (line != "") {
      ret = ret + t(l) + line + "\n";
    }
  }
  return ret;  // 返回生成的执行块
}

/**
 * @brief 缩进函数，用于生成XML标签时进行缩进
 * @param level int类型，表示需要缩进的层数
 * @return std::string类型，表示生成的缩进字符串
 */
std::string SimpleBTBuilder::t(int level) {
  std::string ret;
  for (int i = 0; i < level; i++) {  // 根据传入的层数生成对应的缩进字符串
    ret = ret + "  ";
  }
  return ret;  // 返回生成的缩进字符串
}

/**
 * @brief 字符串替换函数
 * @param str std::string类型，表示需要进行替换的字符串
 * @param from const std::string& 类型，表示需要被替换的子字符串
 * @param to const std::string& 类型，表示替换后的子字符串
 * @details 将字符串str中所有的from子字符串都替换为to子字符串
 */
void replace(std::string& str, const std::string& from, const std::string& to) {
  size_t start_pos = std::string::npos;
  while ((start_pos = str.find(from)) != std::string::npos) {  // 循环查找需要被替换的子字符串
    str.replace(start_pos, from.length(), to);                 // 进行替换操作
  }
}

/**
 * @brief 获取规划计划中的所有动作
 * @param plan 规划计划
 * @return 包含所有动作的 ActionStamped 类型向量
 * @details 从给定的规划计划中获取所有动作，将其打包成 ActionStamped 向量并返回。
 */
std::vector<ActionStamped> SimpleBTBuilder::get_plan_actions(const plansys2_msgs::msg::Plan& plan) {
  std::vector<ActionStamped> ret;

  for (auto& item : plan.items) {
    ActionStamped action_stamped;

    // 将规划计划中的动作信息打包成 ActionStamped 类型
    action_stamped.time = item.time;
    action_stamped.duration = item.duration;
    action_stamped.action = domain_client_->getDurativeAction(
        get_action_name(item.action), get_action_params(item.action));

    // 将打包好的 ActionStamped 加入到返回向量中
    ret.push_back(action_stamped);
  }

  return ret;
}

/**
 * @brief 打印规划图中的节点信息
 * @param node 要打印的节点
 * @param level 当前节点所在的层数
 * @param used_nodes 已经被打印过的节点集合
 * @details
 * 递归地打印规划图中的节点信息。对于每个节点，输出其时间戳、动作名称、参数列表、入边和出边的数量。
 */
void SimpleBTBuilder::print_node(
    const plansys2::GraphNode::Ptr& node,
    int level,
    std::set<plansys2::GraphNode::Ptr>& used_nodes) const {
  std::cerr << std::string(level, '\t') << "[" << node->action.time << "] ";
  std::cerr << node->action.action->name << " ";
  for (const auto& param : node->action.action->parameters) {
    std::cerr << param.name << " ";
  }
  std::cerr << " in arcs " << node->in_arcs.size() << "  ";
  std::cerr << " out arcs " << node->out_arcs.size() << std::endl;

  // 对于每个出边，递归地打印其指向的节点
  for (const auto& out : node->out_arcs) {
    print_node(out, level + 1, used_nodes);
  }
}

/**
 * @brief 打印规划图中所有根节点的信息
 * @param graph 要打印的规划图
 * @details 对于规划图中的每个根节点，调用 print_node() 函数打印其信息。
 */
void SimpleBTBuilder::print_graph(const plansys2::Graph::Ptr& graph) const {
  std::set<plansys2::GraphNode::Ptr> used_nodes;
  for (const auto& root : graph->roots) {
    print_node(root, 0, used_nodes);
  }
}

/**
 * @brief 打印节点的csv格式信息
 * @param node 节点指针
 * @param root_num 根节点编号
 * @details 遍历节点的出边，将节点信息转换为csv格式输出
 */
void SimpleBTBuilder::print_node_csv(
    const plansys2::GraphNode::Ptr& node, uint32_t root_num) const {
  std::string out_str = std::to_string(root_num) + ", " + std::to_string(node->node_num) + ", " +
                        std::to_string(node->level_num) + ", " +
                        parser::pddl::nameActionsToString(node->action.action);
  for (const auto& arc : node->out_arcs) {
    out_str = out_str + ", " + parser::pddl::nameActionsToString(arc->action.action);
  }
  std::cerr << out_str << std::endl;
  for (const auto& out : node->out_arcs) {
    print_node_csv(out, root_num);
  }
}

/**
 * @brief 打印图的csv格式信息
 * @param graph 图指针
 * @details 遍历图的根节点，调用print_node_csv函数打印每个节点的csv格式信息
 */
void SimpleBTBuilder::print_graph_csv(const plansys2::Graph::Ptr& graph) const {
  uint32_t root_num = 0;
  for (const auto& root : graph->roots) {
    print_node_csv(root, root_num);
    root_num++;
  }
}

/**
 * @brief 获取节点的表格格式信息
 * @param node 节点指针
 * @param root_num 根节点编号
 * @param graph 表格信息
 * @details 遍历节点的出边，将节点信息转换为表格格式存入graph中
 */
void SimpleBTBuilder::get_node_tabular(
    const plansys2::GraphNode::Ptr& node,
    uint32_t root_num,
    std::vector<std::tuple<uint32_t, uint32_t, uint32_t, std::string>>& graph) const {
  graph.push_back(std::make_tuple(
      root_num, node->node_num, node->level_num,
      parser::pddl::nameActionsToString(node->action.action)));
  for (const auto& out : node->out_arcs) {
    get_node_tabular(out, root_num, graph);
  }
}

/**
 * @brief 获取图的表格格式信息
 * @param graph 图指针
 * @return 表格信息
 * @details 遍历图的根节点，调用get_node_tabular函数获取每个节点的表格格式信息，并存入vector中返回
 */
std::vector<std::tuple<uint32_t, uint32_t, uint32_t, std::string>>
SimpleBTBuilder::get_graph_tabular(const plansys2::Graph::Ptr& graph) const {
  std::vector<std::tuple<uint32_t, uint32_t, uint32_t, std::string>> graph_tabular;
  uint32_t root_num = 0;
  for (const auto& root : graph->roots) {
    get_node_tabular(root, root_num, graph_tabular);
    root_num++;
  }
  return graph_tabular;
}

}  // namespace plansys2
