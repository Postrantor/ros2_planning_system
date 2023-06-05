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

#include "plansys2_executor/bt_builder_plugins/stn_bt_builder.hpp"

#include <algorithm>
#include <limits>
#include <map>
#include <memory>
#include <queue>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "plansys2_problem_expert/Utils.hpp"

namespace plansys2 {

/**
 * @brief STNBTBuilder类的构造函数，用于初始化DomainExpertClient和ProblemExpertClient。
 */
STNBTBuilder::STNBTBuilder() {
  domain_client_ = std::make_shared<plansys2::DomainExpertClient>();
  problem_client_ = std::make_shared<plansys2::ProblemExpertClient>();
}

/**
 * @brief 初始化STNBTBuilder对象的参数。
 * @param bt_action_1 BT树的起始动作，如果为空，则使用默认值。
 * @param bt_action_2 BT树的结束动作，如果为空，则使用默认值。
 * @param precision 动作执行时间的精度。
 */
void STNBTBuilder::initialize(
    const std::string& bt_action_1, const std::string& bt_action_2, int precision) {
  if (bt_action_1 != "") {
    bt_start_action_ = bt_action_1;
  } else {
    bt_start_action_ =
        R""""(<Sequence name="ACTION_ID">
WAIT_PREV_ACTIONS
  <WaitAtStartReq action="ACTION_ID"/>
  <ApplyAtStartEffect action="ACTION_ID"/>
</Sequence>
)"""";  // 默认起始动作的BT树结构
  }

  if (bt_action_2 != "") {
    bt_end_action_ = bt_action_2;
  } else {
    bt_end_action_ =
        R""""(<Sequence name="ACTION_ID">
  <ReactiveSequence name="ACTION_ID">
  <CheckOverAllReq action="ACTION_ID"/>
    <ExecuteAction action="ACTION_ID"/>
  </ReactiveSequence>
CHECK_PREV_ACTIONS
  <CheckAtEndReq action="ACTION_ID"/>
  <ApplyAtEndEffect action="ACTION_ID"/>
</Sequence>
)"""";  // 默认结束动作的BT树结构
  }

  action_time_precision_ = precision;
}

/**
 * @brief 根据传入的计划生成STN，并返回生成的BT树。
 * @param plan 计划。
 * @return BT树的字符串表示。
 */
std::string STNBTBuilder::get_tree(const plansys2_msgs::msg::Plan& plan) {
  stn_ = build_stn(plan);
  auto bt = build_bt(stn_);
  return bt;
}

/**
 * @brief 获取STNBTBuilder对象的dotgraph字符串
 * @param action_map 存储动作执行信息的map指针
 * @param enable_legend 是否启用图例
 * @param enable_print_graph 是否打印图形
 * @return std::string 返回生成的dotgraph字符串
 * @details
 * 1. 如果enable_print_graph为true，则调用print_graph函数打印stn_图形
 * 2. 创建xdot graph
 * 3. 设置dotgraph格式选项
 * 4. 遍历stn_中的每个节点，创建一个子图cluster_
 * 5. 在子图cluster_中设置节点的标签、样式和颜色，并获取节点的dotgraph信息
 * 6. 定义边缘
 * 7. 如果enable_legend为true，则添加图例
 * 8. 返回生成的dotgraph字符串
 */
std::string STNBTBuilder::get_dotgraph(
    std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map,
    bool enable_legend,
    bool enable_print_graph) {
  if (enable_print_graph) {
    print_graph(stn_);
  }

  // create xdot graph
  std::stringstream ss;
  ss << "digraph plan {\n";

  // dotgraph formatting options
  ss << t(1) << "node[shape=box];\n";
  ss << t(1) << "rankdir=TB;\n";

  int node_count = 0;
  for (const auto node : stn_->nodes) {
    ss << t(1) << "subgraph cluster_" << node_count << " {\n";
    auto start_time = node->action.time;
    if (node->action.type == ActionType::END) {
      start_time += node->action.duration;
    }
    ss << t(2) << "label = \"Time: " << start_time << "\";\n";
    ss << t(2) << "style = rounded;\n";
    ss << t(2) << "color = yellow3;\n";
    ss << t(2) << "bgcolor = lemonchiffon;\n";
    ss << t(2) << "labeljust = l;\n";
    ss << get_node_dotgraph(node, action_map);
    ss << t(1) << "}\n";
    node_count++;
  }

  // define the edges
  std::set<std::string> edges;
  for (const auto& graph_root : stn_->nodes) {
    get_flow_dotgraph(graph_root, edges);
  }

  for (const auto& edge : edges) {
    ss << t(1) << edge;
  }

  if (enable_legend) {
    ss << add_dot_graph_legend(node_count, node_count);
  }

  ss << "}";

  return ss.str();
}

/**
 * @brief 获取节点的dotgraph信息
 * @param node 节点指针
 * @param action_map 存储动作执行信息的map指针
 * @return std::string 返回生成的节点dotgraph字符串
 */
std::string STNBTBuilder::get_node_dotgraph(
    const BT::TreeNode* node,
    std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map) {
  std::stringstream ss;

  // get the action name and status
  std::string action_name = "";
  std::string action_status = "";
  if (node->action.type != ActionType::EMPTY) {
    action_name = node->action.name;
    auto it = action_map->find(action_name);
    if (it != action_map->end()) {
      action_status = it->second.status.toString();
    }
  }

  // create the node label
  ss << t(2) << "\"" << node->name << "\"";
  if (!action_name.empty()) {
    ss << " [label=<<table border=\"0\" cellborder=\"0\" cellspacing=\"0\">\n"
       << t(3) << "<tr><td>" << node->name << "</td></tr>\n"
       << t(3) << "<tr><td>" << action_name << "</td></tr>\n"
       << t(3) << "<tr><td>" << action_status << "</td></tr>\n"
       << t(2) << "</table>>]\n";
  } else {
    ss << ";\n";
  }

  return ss.str();
}

/**
 * @brief 获取流程图的dotgraph信息
 * @param node 节点指针
 * @param edges 存储边缘信息的set指针
 */
void STNBTBuilder::get_flow_dotgraph(const BT::TreeNode* node, std::set<std::string>& edges) {
  for (const auto& child : node->children) {
    std::stringstream ss;
    ss << "\"" << node->name << "\" -> \"" << child->name << "\"";
    edges.insert(ss.str());
    get_flow_dotgraph(child, edges);
  }
}

/**
 * @brief 构建 STN（时序网络）图
 * @param plan 规划的计划
 * @return 返回构建好的 STN 图
 * @details 根据传入的规划计划，初始化 STN
 * 图，获取发生事件、简单计划和状态等信息。然后对于每个简单计划中的动作，
 *          获取当前动作对应的图节点，获取当前动作的父动作，遍历所有父动作，将当前动作与父动作进行比较，如果两者相同则跳过；
 *          否则，修剪路径并检查路径是否可行，若不可行，则在两个节点之间添加输入/输出弧。
 */
Graph::Ptr STNBTBuilder::build_stn(const plansys2_msgs::msg::Plan& plan) const {
  auto stn = init_graph(plan);
  auto happenings = get_happenings(plan);
  auto simple_plan = get_simple_plan(plan);
  auto states = get_states(happenings, simple_plan);

  for (const auto& item : simple_plan) {
    // Skip the first action corresponding to the initial state
    if (item.first < 0) {
      continue;
    }

    // Get the graph nodes corresponding to the current action
    auto current = get_nodes(item.second, stn);

    // Get the parent actions of the current action
    auto parents = get_parents(item, simple_plan, happenings, states);

    for (const auto& parent : parents) {
      auto previous = get_nodes(parent.second, stn);
      for (auto& n : current) {
        for (auto& h : previous) {
          if (h->action.time == n->action.time) {
            if (h->action.expression == n->action.expression) {
              // No self-referencing edges are allowed in an STN.
              continue;
            }
          }
          prune_paths(n, h);
          if (!check_paths(n, h)) {
            h->output_arcs.insert(std::make_tuple(n, 0, std::numeric_limits<float>::infinity()));
            n->input_arcs.insert(std::make_tuple(h, 0, std::numeric_limits<float>::infinity()));
          }
        }
      }
    }
  }

  return stn;
}

/**
 * @brief 构建行为树
 * @param stn 输入的 STN 图
 * @return 返回构建好的行为树
 * @details 根据传入的 STN 图，获取根节点，然后递归遍历所有节点，生成对应的行为树。
 */
std::string STNBTBuilder::build_bt(const Graph::Ptr stn) const {
  std::set<GraphNode::Ptr> used;
  const auto& root = stn->nodes.front();

  auto bt = std::string("<root main_tree_to_execute=\"MainTree\">\n") + t(1) +
            "<BehaviorTree ID=\"MainTree\">\n";
  bt = bt + get_flow(root, root, used, 1);
  bt = bt + t(1) + "</BehaviorTree>\n</root>\n";

  return bt;
}

/**
 * @brief 初始化图形结构，用于规划路径
 * @param plan 规划器生成的计划
 * @return 返回初始化后的图形结构
 *
 * @details
 * 1. 获取计划中的动作序列
 * 2. 添加一个节点来表示初始状态
 * 3. 添加起始和结束快照动作的节点
 * 4. 添加一个节点来表示目标状态
 */
Graph::Ptr STNBTBuilder::init_graph(const plansys2_msgs::msg::Plan& plan) const {
  // 创建一个空的图形结构
  auto graph = Graph::make_shared();
  // 获取计划中的动作序列
  auto action_sequence = get_plan_actions(plan);

  // 添加一个节点来表示初始状态
  auto predicates = problem_client_->getPredicates();
  auto functions = problem_client_->getFunctions();

  int node_cnt = 0;
  auto init_node = GraphNode::make_shared(node_cnt++);
  init_node->action.action = std::make_shared<plansys2_msgs::msg::DurativeAction>();
  init_node->action.action->at_end_effects = from_state(predicates, functions);
  init_node->action.type = ActionType::INIT;
  graph->nodes.push_back(init_node);

  // 添加起始和结束快照动作的节点
  for (const auto& action : action_sequence) {
    auto start_node = GraphNode::make_shared(node_cnt++);
    start_node->action = action;
    start_node->action.type = ActionType::START;

    auto end_node = GraphNode::make_shared(node_cnt++);
    end_node->action = action;
    end_node->action.type = ActionType::END;

    start_node->output_arcs.insert(std::make_tuple(end_node, action.duration, action.duration));
    end_node->input_arcs.insert(std::make_tuple(start_node, action.duration, action.duration));

    graph->nodes.push_back(start_node);
    graph->nodes.push_back(end_node);
  }

  // 添加一个节点来表示目标状态
  auto goal = problem_client_->getGoal();
  plansys2_msgs::msg::Tree* goal_tree = &goal;

  auto goal_node = GraphNode::make_shared(node_cnt++);
  goal_node->action.action = std::make_shared<plansys2_msgs::msg::DurativeAction>();
  goal_node->action.action->at_start_requirements = *goal_tree;
  goal_node->action.type = ActionType::GOAL;
  graph->nodes.push_back(goal_node);

  return graph;
}

/**
代码功能梳理：
-
get_plan_actions函数：根据规划结果获取所有动作序列，并将其转化为ActionStamped类型，添加到返回的动作序列中。
- get_happenings函数：根据规划结果获取所有事件的开始和结束时间点，将其添加到返回的时间点集合中。
- get_happening函数：根据时间点获取发生事件的时间点，返回指向该时间点的迭代器。
*/

/**
 * @brief 获取规划的动作序列
 * @param plan 规划结果
 * @return std::vector<ActionStamped> 动作序列
 * @details 遍历规划结果，将每个动作转化为ActionStamped类型，并添加到返回的动作序列中。
 */
std::vector<ActionStamped> STNBTBuilder::get_plan_actions(
    const plansys2_msgs::msg::Plan& plan) const {
  std::vector<ActionStamped> ret;

  for (auto& item : plan.items) {
    ActionStamped action_stamped;
    action_stamped.time = item.time;
    action_stamped.expression = item.action;
    action_stamped.duration = item.duration;
    action_stamped.type = ActionType::DURATIVE;
    action_stamped.action = domain_client_->getDurativeAction(
        get_action_name(item.action), get_action_params(item.action));

    ret.push_back(action_stamped);
  }

  return ret;
}

/**
 * @brief 获取发生事件的时间点集合
 * @param plan 规划结果
 * @return std::set<int> 发生事件的时间点集合
 * @details 根据规划结果获取所有事件的开始和结束时间点，将其添加到返回的时间点集合中。
 */
std::set<int> STNBTBuilder::get_happenings(const plansys2_msgs::msg::Plan& plan) const {
  std::set<int> happenings;
  happenings.insert(-1);
  auto action_sequence = get_plan_actions(plan);
  for (const auto& action : action_sequence) {
    auto time = to_int_time(action.time, action_time_precision_ + 1);
    auto duration = to_int_time(action.duration, action_time_precision_ + 1);
    happenings.insert(time);
    happenings.insert(time + duration);
  }
  return happenings;
}

/**
 * @brief 获取一个迭代器，该迭代器指向 std::set<int> happenings 中与 time 相等或最后一个小于 time
 * 的元素。
 * @param time 时间戳
 * @param happenings 事件集合
 * @return std::set<int>::iterator 迭代器
 * @details
 * 1. 如果找到了与 time 相等的元素，则返回指向该元素的迭代器。
 * 2. 如果没有找到与 time 相等的元素，则返回指向最后一个小于 time 的元素的迭代器。
 * 3. 如果不存在小于 time 的元素，则返回 end() 迭代器。
 */
std::set<int>::iterator STNBTBuilder::get_happening(
    int time, const std::set<int>& happenings) const {
  // lower_bound 返回一个迭代器，该迭代器指向第一个不小于（即大于或等于）key的元素。
  auto it = happenings.lower_bound(time);
  if (it != happenings.end()) {
    if (*it != time) {
      if (it != happenings.begin()) {
        it--;
      } else {
        it = happenings.end();
      }
    }
  }

  return it;
}

/**
 * @brief 获取一个迭代器，该迭代器指向 std::set<int> happenings 中小于 time 的最后一个元素。
 * @param time 时间戳
 * @param happenings 事件集合
 * @return std::set<int>::iterator 迭代器
 * @details
 * 1. 如果存在小于 time 的元素，则返回指向最后一个小于 time 的元素的迭代器。
 * 2. 如果不存在小于 time 的元素，则返回 end() 迭代器。
 */
std::set<int>::iterator STNBTBuilder::get_previous(
    int time, const std::set<int>& happenings) const {
  auto it = get_happening(time, happenings);

  if (it != happenings.end()) {
    if (it != happenings.begin()) {
      return std::prev(it);
    }
  }

  return happenings.end();
}

/**
 * @brief 获取简单计划的函数
 * @param plan 规划消息
 * @return 简单计划，即去除并行动作后的计划
 * @details
 * 1. 获取规划中的所有动作序列。
 * 2. 添加一个动作来表示初始状态。
 * 3. 添加快照动作。
 * 4. 添加一个动作来表示目标状态。
 * 5. 创建整体动作。
 * 6. 添加整体动作。
 */
std::multimap<int, ActionStamped> STNBTBuilder::get_simple_plan(
    const plansys2_msgs::msg::Plan& plan) const {
  std::multimap<int, ActionStamped> simple_plan;
  auto action_sequence = get_plan_actions(plan);

  // Add an action to represent the initial state
  auto predicates = problem_client_->getPredicates();
  auto functions = problem_client_->getFunctions();

  // 添加一个动作来表示初始状态
  ActionStamped init_action;
  init_action.action = std::make_shared<plansys2_msgs::msg::DurativeAction>();
  init_action.action->at_end_effects = from_state(predicates, functions);
  init_action.type = ActionType::INIT;
  simple_plan.insert(std::make_pair(-1, init_action));

  // 添加快照动作
  int max_time = -1;
  for (auto action : action_sequence) {
    auto time = to_int_time(action.time, action_time_precision_ + 1);
    auto duration = to_int_time(action.duration, action_time_precision_ + 1);
    action.type = ActionType::START;
    simple_plan.insert(std::make_pair(time, action));
    action.type = ActionType::END;
    simple_plan.insert(std::make_pair(time + duration, action));
    max_time = std::max(max_time, time + duration);
  }

  // 添加一个动作来表示目标状态
  auto goal = problem_client_->getGoal();
  plansys2_msgs::msg::Tree* goal_tree = &goal;

  ActionStamped goal_action;
  goal_action.action = std::make_shared<plansys2_msgs::msg::DurativeAction>();
  goal_action.action->at_start_requirements = *goal_tree;
  goal_action.type = ActionType::GOAL;
  simple_plan.insert(std::make_pair(max_time, goal_action));

  // 创建整体动作
  std::vector<std::pair<int, ActionStamped>> overall_actions;
  for (const auto& action : action_sequence) {
    auto time = to_int_time(action.time, action_time_precision_ + 1);
    auto duration = to_int_time(action.duration, action_time_precision_ + 1);

    // 找到开始动作
    auto it = simple_plan.equal_range(time);
    auto start_action = std::find_if(it.first, it.second, [&](std::pair<int, ActionStamped> a) {
      return (a.second.expression == action.expression) && (a.second.type == ActionType::START);
    });

    // 找到结束动作
    it = simple_plan.equal_range(time + duration);
    auto end_action = std::find_if(it.first, it.second, [&](std::pair<int, ActionStamped> a) {
      return (a.second.expression == action.expression) && (a.second.type == ActionType::END);
    });

    // 计算整体动作
    int prev = time;
    for (auto iter = start_action; iter != simple_plan.end(); ++iter) {
      if (iter->first != prev) {
        int time = prev + (iter->first - prev) / 2;
        auto overall_action = std::make_pair(time, start_action->second);
        overall_action.second.type = ActionType::OVERALL;
        overall_actions.push_back(overall_action);
        prev = iter->first;
      }
      if (iter == end_action) {
        break;
      }
    }
  }

  // 添加整体动作
  for (const auto& overall_action : overall_actions) {
    simple_plan.insert(overall_action);
  }

  return simple_plan;
}

/**
 * @brief 获取规划器的状态信息
 * @param happenings 规划器中发生的事件
 * @param plan 规划器中的计划
 * @return 返回一个 map，其中 key 为时间，value 为对应时间的状态信息
 * @details 1. 初始化一个空的状态信息容器 states 和一个状态向量 state_vec。
 *          2. 将问题客户端中的谓词和函数分别存入 state_vec.predicates 和 state_vec.functions
 * 中，并将其插入到 states 的 -1 时间点处。
 *          3. 遍历 happenings 中的每个时间点 time，查找 plan 中该时间点对应的动作 iter。
 *          4. 如果该动作是 START 类型，则将其 at_start_effects 应用于 state_vec 中的 predicates 和
 * functions。
 *          5. 如果该动作是 END 类型，则将其 at_end_effects 应用于 state_vec 中的 predicates 和
 * functions。
 *          6. 将当前时间点 time 和更新后的状态信息 state_vec 插入到 states 容器中。
 */
std::map<int, StateVec> STNBTBuilder::get_states(
    const std::set<int>& happenings, const std::multimap<int, ActionStamped>& plan) const {
  // 初始化一个空的状态信息容器 states
  std::map<int, StateVec> states;

  // 初始化一个状态向量 state_vec，将问题客户端中的谓词和函数分别存入 state_vec.predicates 和
  // state_vec.functions 中，并将其插入到 states 的 -1 时间点处。
  StateVec state_vec;
  state_vec.predicates = problem_client_->getPredicates();
  state_vec.functions = problem_client_->getFunctions();
  states.insert(std::make_pair(-1, state_vec));

  // 遍历 happenings 中的每个时间点 time，查找 plan 中该时间点对应的动作 iter。
  for (const auto& time : happenings) {
    auto it = plan.equal_range(time);
    for (auto iter = it.first; iter != it.second; ++iter) {
      // 如果该动作是 START 类型，则将其 at_start_effects 应用于 state_vec 中的 predicates 和
      // functions。
      if (iter->second.type == ActionType::START) {
        apply(iter->second.action->at_start_effects, state_vec.predicates, state_vec.functions);
      }
      // 如果该动作是 END 类型，则将其 at_end_effects 应用于 state_vec 中的 predicates 和
      // functions。
      else if (iter->second.type == ActionType::END) {
        apply(iter->second.action->at_end_effects, state_vec.predicates, state_vec.functions);
      }
    }
    // 将当前时间点 time 和更新后的状态信息 state_vec 插入到 states 容器中。
    states.insert(std::make_pair(time, state_vec));
  }

  return states;
}

/**
 * @brief 将规划器的状态信息转换为 plansys2_msgs::msg::Tree 类型
 * @param preds 规划器中的谓词
 * @param funcs 规划器中的函数
 * @return 返回一个 plansys2_msgs::msg::Tree 类型的树形结构，其中包含了所有的谓词和函数信息
 * @details 1. 初始化一个空的 plansys2_msgs::msg::Tree 类型的树形结构 tree 和一个
 * plansys2_msgs::msg::Node 类型的节点 node。
 *          2. 将节点类型设置为 AND，并将其加入到 tree 中。
 *          3. 遍历 preds 中的每个谓词 pred，将其转换为 plansys2_msgs::msg::Node 类型的节点
 * child，并将其加入到 tree 中。
 *          4. 将 child 的 node_id 设置为当前 tree 中节点的数量减一，并将其加入到 tree.nodes[0] 的
 * children 中。
 *          5. 遍历 funcs 中的每个函数 func，将其转换为 plansys2_msgs::msg::Node 类型的节点
 * child，并将其加入到 tree 中。
 *          6. 将 child 的 node_id 设置为当前 tree 中节点的数量减一，并将其加入到 tree.nodes[0] 的
 * children 中。
 *          7. 返回转换后的树形结构 tree。
 */
plansys2_msgs::msg::Tree STNBTBuilder::from_state(
    const std::vector<plansys2::Predicate>& preds,
    const std::vector<plansys2::Function>& funcs) const {
  // 初始化一个空的 plansys2_msgs::msg::Tree 类型的树形结构 tree 和一个 plansys2_msgs::msg::Node
  // 类型的节点 node。
  plansys2_msgs::msg::Tree tree;
  plansys2_msgs::msg::Node node;
  // 将节点类型设置为 AND，并将其加入到 tree 中。
  node.node_type = plansys2_msgs::msg::Node::AND;
  node.node_id = 0;
  node.negate = false;
  tree.nodes.push_back(node);

  // 遍历 preds 中的每个谓词 pred，将其转换为 plansys2_msgs::msg::Node 类型的节点
  // child，并将其加入到 tree 中。
  for (const auto& pred : preds) {
    const plansys2_msgs::msg::Node* child = &pred;
    tree.nodes.push_back(*child);
    // 将 child 的 node_id 设置为当前 tree 中节点的数量减一
    tree.nodes.back().node_id = tree.nodes.size() - 1;
    // 将 child 加入到 tree.nodes[0] 的 children 中
    tree.nodes[0].children.push_back(tree.nodes.size() - 1);
  }

  // 遍历 funcs 中的每个函数 func，将其转换为 plansys2_msgs::msg::Node 类型的节点
  // child，并将其加入到 tree 中。
  for (const auto& func : funcs) {
    const plansys2_msgs::msg::Node* child = &func;
    tree.nodes.push_back(*child);
    // 将 child 的 node_id 设置为当前 tree 中节点的数量减一
    tree.nodes.back().node_id = tree.nodes.size() - 1;
    // 将 child 加入到 tree.nodes[0] 的 children 中
    tree.nodes[0].children.push_back(tree.nodes.size() - 1);
  }

  // 返回转换后的树形结构 tree。
  return tree;
}

/**
 * @brief 获取与给定动作匹配的图节点
 * @param action 给定的动作
 * @param graph 图对象指针
 * @return 匹配的图节点指针向量
 * @details
 * 1. 如果动作类型为INIT，则返回与该类型匹配的节点。
 * 2. 如果动作类型为GOAL，则返回与该类型匹配的节点。
 * 3. 如果动作类型为START或OVERALL，则返回与该类型匹配的节点。
 * 4. 如果动作类型为END或OVERALL，则返回与该类型匹配的节点。
 * 5. 如果找不到匹配的节点，则输出错误信息并返回空向量。
 */
std::vector<GraphNode::Ptr> STNBTBuilder::get_nodes(
    const ActionStamped& action, const Graph::Ptr graph) const {
  std::vector<GraphNode::Ptr> ret;

  if (action.type == ActionType::INIT) {  // 如果动作类型为INIT，则返回与该类型匹配的节点。
    auto it = std::find_if(graph->nodes.begin(), graph->nodes.end(), [&](GraphNode::Ptr node) {
      return node->action.type == ActionType::INIT;
    });
    if (it != graph->nodes.end()) {
      ret.push_back(*it);
    } else {
      std::cerr << "get_nodes: Could not find initial state node" << std::endl;  // 输出错误信息
    }
    return ret;
  }

  if (action.type == ActionType::GOAL) {  // 如果动作类型为GOAL，则返回与该类型匹配的节点。
    auto it = std::find_if(graph->nodes.begin(), graph->nodes.end(), [&](GraphNode::Ptr node) {
      return node->action.type == ActionType::GOAL;
    });
    if (it != graph->nodes.end()) {
      ret.push_back(*it);
    } else {
      std::cerr << "get_nodes: Could not find goal node" << std::endl;  // 输出错误信息
    }
    return ret;
  }

  std::vector<GraphNode::Ptr> matches;
  std::copy_if(
      graph->nodes.begin(), graph->nodes.end(), std::back_inserter(matches),
      std::bind(&STNBTBuilder::is_match, this, std::placeholders::_1, action));

  if (action.type == ActionType::START ||
      action.type ==
          ActionType::OVERALL) {  // 如果动作类型为START或OVERALL，则返回与该类型匹配的节点。
    auto it = std::find_if(matches.begin(), matches.end(), [&](GraphNode::Ptr node) {
      return node->action.type == ActionType::START;
    });
    if (it != matches.end()) {
      ret.push_back(*it);
    } else {
      std::cerr << "get_nodes: Could not find start node" << std::endl;  // 输出错误信息
    }
  }

  if (action.type == ActionType::END ||
      action.type ==
          ActionType::OVERALL) {  // 如果动作类型为END或OVERALL，则返回与该类型匹配的节点。
    auto it = std::find_if(matches.begin(), matches.end(), [&](GraphNode::Ptr node) {
      return node->action.type == ActionType::END;
    });
    if (it != matches.end()) {
      ret.push_back(*it);
    } else {
      std::cerr << "get_nodes: Could not find end node" << std::endl;  // 输出错误信息
    }
  }

  if (ret.empty()) {
    std::cerr << "get_nodes: Could not find graph node" << std::endl;  // 输出错误信息
  }

  return ret;
}

/**
 * @brief 判断给定的GraphNode节点是否与给定的ActionStamped动作匹配
 * @param node 给定的GraphNode节点
 * @param action 给定的ActionStamped动作
 * @return 如果匹配则返回true，否则返回false
 * @details
 * 1. 将node节点的时间戳转换为整型t_1
 * 2. 将action的时间戳转换为整型t_2
 * 3. 如果t_1等于t_2并且node节点的表达式与action的表达式相同，则返回true，否则返回false
 */
bool STNBTBuilder::is_match(const GraphNode::Ptr node, const ActionStamped& action) const {
  auto t_1 = to_int_time(node->action.time, action_time_precision_ + 1);
  auto t_2 = to_int_time(action.time, action_time_precision_ + 1);
  return (t_1 == t_2) && (node->action.expression == action.expression);
}

/**
 * @brief 获取给定动作的父节点
 * @param action 给定的动作
 * @param plan 动作序列
 * @param happenings 发生事件集合
 * @param states 状态映射表
 * @return 返回一个pair<int, ActionStamped>类型的vector，表示给定动作的所有父节点
 * @details
 * 1. 调用get_satisfy函数获取满足条件的父节点parents
 * 2. 调用get_threat函数获取威胁动作的父节点threats
 * 3. 将threats插入到parents的末尾
 * 4. 返回parents
 */
std::vector<std::pair<int, ActionStamped>> STNBTBuilder::get_parents(
    const std::pair<int, ActionStamped>& action,
    const std::multimap<int, ActionStamped>& plan,
    const std::set<int>& happenings,
    const std::map<int, StateVec>& states) const {
  auto parents = get_satisfy(action, plan, happenings, states);
  auto threats = get_threat(action, plan, happenings, states);
  parents.insert(std::end(parents), std::begin(threats), std::end(threats));

  return parents;
}

/**
 * @brief 获取满足条件的动作对
 * @param action 动作对
 * @param plan 规划
 * @param happenings 发生事件
 * @param states 状态
 * @return 满足条件的动作对向量
 * @details
 * 1. 初始化一个空的动作对向量 ret。
 * 2. 根据动作对 action 的时间戳，从 happenings 中获取发生事件的迭代器 H_it。
 * 3. 如果 H_it 为 end()，则输出错误信息并返回 ret。
 * 4. 获取 R_a，即动作对 action 的条件。
 * 5. 当 t_2 大于等于 0 时，执行以下循环：
 *    a. 从 happenings 中获取上一个发生事件的迭代器 H_it。
 *    b. 如果 H_it 为 end()，则输出错误信息并跳出循环。
 *    c. 获取上一个状态 X_1。
 *    d. 遍历 R_a 中的每个条件 r，如果 X_1 不满足 r，则执行以下操作：
 *       i. 在 plan 中查找时间戳为 t_2 的动作对。
 *       ii. 遍历查找到的动作对，如果动作对的时间戳与 action 相同且表达式相同，则跳过该动作对。
 *       iii. 否则，获取动作对 k 的效果 E_k，并将其应用到状态 X_hat 上。
 *       iv. 如果动作 k 满足条件 r，则将动作对 k 加入 ret 中。
 *    e. 将 t_2 更新为上一个发生事件的时间戳。
 * 6. 获取问题客户端中的谓词和函数。
 * 7. 遍历 R_a 中的每个条件 r，如果 X_1 满足 r，则将 plan 的第一个动作对加入 ret 中。
 * 8. 返回满足条件的动作对向量 ret。
 */
std::vector<std::pair<int, ActionStamped>> STNBTBuilder::get_satisfy(
    const std::pair<int, ActionStamped>& action,
    const std::multimap<int, ActionStamped>& plan,
    const std::set<int>& happenings,
    const std::map<int, StateVec>& states) const {
  std::vector<std::pair<int, ActionStamped>> ret;

  auto H_it = get_happening(action.first, happenings);
  if (H_it == happenings.end()) {
    std::cerr << "get_satisfy: Happening time not found" << std::endl;
    return ret;
  }
  auto t_2 = *H_it;

  auto R_a = parser::pddl::getSubtrees(get_conditions(action.second));

  while (t_2 >= 0) {
    H_it = get_previous(t_2, happenings);
    if (H_it == happenings.end()) {
      std::cerr << "get_satisfy: Previous happening time not found" << std::endl;
      break;
    }
    auto t_1 = *H_it;

    auto X_it = states.find(t_1);
    if (X_it == states.end()) {
      std::cerr << "get_satisfy: Previous state not found" << std::endl;
      break;
    }
    auto X_1 = X_it->second;

    for (const auto& r : R_a) {
      if (!check(r, X_1.predicates, X_1.functions)) {
        auto it = plan.equal_range(t_2);
        for (auto iter = it.first; iter != it.second; ++iter) {
          if (iter->first == action.first) {
            if (iter->second.expression == action.second.expression) {
              continue;
            }
          }

          auto E_k = get_effects(iter->second);

          auto X_hat = X_1;
          apply(E_k, X_hat.predicates, X_hat.functions);

          // Check if action k satisfies action i
          if (check(r, X_hat.predicates, X_hat.functions)) {
            ret.push_back(*iter);
          }
        }
      }
    }
    t_2 = t_1;
  }

  auto predicates = problem_client_->getPredicates();
  auto functions = problem_client_->getFunctions();

  for (const auto& r : R_a) {
    if (check(r, predicates, functions)) {
      ret.push_back(*plan.begin());
    }
  }

  return ret;
}

/**
 * @brief 获取威胁列表
 * @param action 当前动作及其时间戳
 * @param plan 动作序列及其时间戳的多重映射
 * @param happenings 发生时间的集合
 * @param states 状态及其时间戳的映射
 * @return 威胁列表，包含威胁当前动作的动作及其时间戳
 * @details 在给定的计划中查找威胁当前动作的所有动作。对于每个可能的威胁动作，
 * 检查它是否与当前动作在同一发生时间内，如果是，则忽略该威胁动作。
 * 否则，检查该威胁动作是否威胁当前动作，并将其添加到威胁列表中。
 */
std::vector<std::pair<int, ActionStamped>> STNBTBuilder::get_threat(
    const std::pair<int, ActionStamped>& action,
    const std::multimap<int, ActionStamped>& plan,
    const std::set<int>& happenings,
    const std::map<int, StateVec>& states) const {
  std::vector<std::pair<int, ActionStamped>> ret;

  auto H_it = get_happening(action.first, happenings);
  if (H_it == happenings.end()) {
    std::cerr << "get_threat: Happening time not found" << std::endl;
    return ret;
  }
  auto t_in = *H_it;
  auto t_2 = *H_it;

  auto R_a = get_conditions(action.second);
  auto E_a = get_effects(action.second);

  while (t_2 >= 0) {
    H_it = get_previous(t_2, happenings);
    if (H_it == happenings.end()) {
      std::cerr << "get_threat: Previous happening time not found" << std::endl;
      break;
    }
    auto t_1 = *H_it;

    auto X_it = states.find(t_1);
    if (X_it == states.end()) {
      std::cerr << "get_threat: Previous state not found" << std::endl;
      break;
    }
    auto X_1 = X_it->second;

    auto X_1_a = X_1;
    if (can_apply(action, plan, t_2, X_1_a)) {
      auto it = plan.end();
      H_it = happenings.find(t_2);
      if (std::next(H_it) != happenings.end()) {
        it = plan.lower_bound(*std::next(H_it));
      }

      for (auto iter = plan.lower_bound(t_2); iter != it; ++iter) {
        if (iter->first == action.first) {
          if (iter->second.expression == action.second.expression) {
            continue;
          }
        }

        auto X_1_k = X_1;
        if (!can_apply(*iter, plan, t_2, X_1_k)) {
          std::cerr << "get_threat: Suitable intermediate state not found. ";
          std::cerr << "However, one should exist." << std::endl;
          continue;
        }

        auto R_k = get_conditions(iter->second);
        auto E_k = get_effects(iter->second);

        auto X_hat = X_1_k;
        apply(E_a, X_hat.predicates, X_hat.functions);

        // 检查输入动作是否威胁动作k
        if (action.second.type != ActionType::OVERALL &&
            !check(R_k, X_hat.predicates, X_hat.functions)) {
          if (t_2 != t_in) {
            ret.push_back(*iter);
          } else {
            std::cerr << "get_threat: An action should not be threatened ";
            std::cerr << "by another action at the same happening time. ";
            std::cerr << "Check the plan validity." << std::endl;
          }
          continue;
        }

        auto X_bar = X_1_a;
        apply(E_k, X_bar.predicates, X_bar.functions);

        // 检查动作k是否威胁输入动作
        if (iter->second.type != ActionType::OVERALL &&
            !check(R_a, X_bar.predicates, X_bar.functions)) {
          if (t_2 != t_in) {
            ret.push_back(*iter);
          } else {
            std::cerr << "get_threat: An action should not be threatened ";
            std::cerr << "by another action at the same happening time. ";
            std::cerr << "Check the plan validity." << std::endl;
          }
          continue;
        }

        // 检查输入动作和动作k是否修改了相同的效果
        if (action.second.type != ActionType::OVERALL && iter->second.type != ActionType::OVERALL) {
          auto DX_hat = get_diff(X_1_k, X_hat);
          auto DX_bar = get_diff(X_1_a, X_bar);
          auto intersection = get_intersection(DX_hat, DX_bar);
          if (intersection.predicates.size() > 0 || intersection.functions.size() > 0) {
            if (t_2 != t_in) {
              ret.push_back(*iter);
            } else {
              std::cerr << "get_threat: An action should not be threatened ";
              std::cerr << "by another action at the same happening time. ";
              std::cerr << "Check the plan validity." << std::endl;
            }
          }
        }
      }
    }
    t_2 = t_1;
  }

  return ret;
}

/**
 * @brief 判断当前状态是否满足给定的条件，以及在计划中是否存在与给定动作冲突的动作序列
 * @param action 给定的动作
 * @param plan 计划中的动作序列
 * @param time 执行动作的时间
 * @param state 当前状态
 * @return 如果当前状态满足给定条件并且不存在与给定动作冲突的动作序列，则返回 true；否则返回 false
 * @details 该函数用于判断当前状态是否满足给定的条件，以及在计划中是否存在与给定动作冲突的动作序列。
 * 首先将当前状态保存到变量 X 中，并获取给定动作的前置条件和效果。
 * 然后检查当前状态是否满足给定动作的前置条件，如果满足则直接返回 true。
 * 否则，遍历计划中所有执行时间为 time 的动作，依次选取其中 m
 * 个动作进行组合，并对每种组合进行如下操作：
 * 对于被选中的动作，如果其与给定动作相同，则跳过；否则，获取其效果并应用到当前状态上。
 * 最后检查当前状态是否满足给定动作的前置条件，如果满足则返回 true。
 * 如果所有组合都被尝试过仍未找到符合条件的状态，则返回 false。
 */
bool STNBTBuilder::can_apply(
    const std::pair<int, ActionStamped>& action,    // 给定的动作及其时间戳
    const std::multimap<int, ActionStamped>& plan,  // 计划中的动作序列
    const int& time,                                // 执行动作的时间
    StateVec& state) const {                        // 当前状态
  auto X = state;                                   // 将当前状态保存到变量 X 中
  auto R = get_conditions(action.second);           // 获取给定动作的前置条件

  // 检查当前状态是否满足给定动作的前置条件，如果满足则直接返回 true
  if (check(R, X.predicates, X.functions)) {
    return true;
  }

  int n = plan.count(time);          // 获取计划中执行时间为 time 的动作数量
  auto it = plan.lower_bound(time);  // 获取第一个执行时间为 time 的动作的迭代器
  for (int m = 1; m <= n; ++m) {     // 对每种动作组合进行尝试
    std::vector<bool> v(n);          // 定义长度为 n 的 bool 向量 v
    std::fill(v.begin(), v.begin() + m, true);  // 将前 m 个元素设为 true，表示选中

    state = X;                                  // 将当前状态恢复到 X
    do {                                        // 对每种动作组合进行如下操作
      for (int i = 0; i < n; ++i) {           // 遍历计划中所有执行时间为 time 的动作
        if (v[i]) {                           // 如果该动作被选中
          auto iter = std::next(it, i);       // 获取该动作的迭代器
          if (iter->first == action.first) {  // 如果该动作与给定动作时间戳相同
            if (iter->second.expression == action.second.expression) {  // 如果该动作与给定动作相同
              continue;                                                 // 跳过该动作
            }
          }
          auto E = get_effects(iter->second);           // 获取该动作的效果
          apply(E, state.predicates, state.functions);  // 将该动作的效果应用到当前状态上
          if (check(
                  R, state.predicates,
                  state.functions)) {  // 检查当前状态是否满足给定动作的前置条件
            return true;               // 如果满足，则返回 true
          }
        }
      }
    } while (std::prev_permutation(v.begin(), v.end()));  // 生成下一种动作组合
  }

  return false;  // 如果所有组合都被尝试过仍未找到符合条件的状态，则返回 false
}

/**
 * @brief 获取两个状态之间的差异
 * @param X_1 第一个状态向量
 * @param X_2 第二个状态向量
 * @return 返回两个状态之间的差异
 * @details
 * 1. 定义一个空的状态向量 ret
 * 2. 遍历第一个状态向量 X_1 中的谓词，查找在第二个状态向量 X_2 中不存在的谓词，并将其添加到 ret 中
 * 3. 遍历第二个状态向量 X_2 中的谓词，查找在第一个状态向量 X_1 中不存在的谓词，并将其添加到 ret 中
 * 4. 查找函数值的变化，如果变化超过阈值，则将其添加到 ret 中
 */
StateVec STNBTBuilder::get_diff(const StateVec& X_1, const StateVec& X_2) const {
  StateVec ret;

  // Look for predicates in X_1 that are not in X_2
  for (const auto& p_1 : X_1.predicates) {
    auto it = std::find_if(
        X_2.predicates.begin(), X_2.predicates.end(),
        [&](plansys2::Predicate p_2) { return parser::pddl::checkNodeEquality(p_1, p_2); });
    if (it == X_2.predicates.end()) {
      ret.predicates.push_back(p_1);
    }
  }

  // Look for predicates in X_2 that are not in X_1
  for (const auto& p_2 : X_2.predicates) {
    auto it = std::find_if(
        X_1.predicates.begin(), X_1.predicates.end(),
        [&](plansys2::Predicate p_1) { return parser::pddl::checkNodeEquality(p_1, p_2); });
    if (it == X_1.predicates.end()) {
      ret.predicates.push_back(p_2);
    }
  }

  // Look for function changes
  for (const auto& f_1 : X_1.functions) {
    auto it = std::find_if(X_2.functions.begin(), X_2.functions.end(), [&](plansys2::Function f_2) {
      return parser::pddl::checkNodeEquality(f_1, f_2);
    });
    if (it != X_2.functions.end()) {
      if (std::abs(f_1.value - it->value) >
          1e-5 * std::max(std::abs(f_1.value), std::abs(it->value))) {
        ret.functions.push_back(f_1);
      }
    }
  }

  return ret;
}

/**
 * @brief 获取两个状态的交集
 * @param X_1 第一个状态向量
 * @param X_2 第二个状态向量
 * @return 两个状态向量的交集
 * @details
 * 1. 遍历第一个状态向量中的谓词，查找是否存在于第二个状态向量中，将其加入返回结果中。
 * 2. 遍历第一个状态向量中的函数，查找是否存在于第二个状态向量中，将其加入返回结果中。
 */
StateVec STNBTBuilder::get_intersection(const StateVec& X_1, const StateVec& X_2) const {
  StateVec ret;

  // Look for predicates in X_1 that are also in X_2
  for (const auto& p_1 : X_1.predicates) {
    auto it = std::find_if(
        X_2.predicates.begin(), X_2.predicates.end(),
        [&](plansys2::Predicate p_2) { return parser::pddl::checkNodeEquality(p_1, p_2); });
    if (it != X_2.predicates.end()) {
      ret.predicates.push_back(p_1);
    }
  }

  // Look for functions in X_1 that are also in X_2
  for (const auto& f_1 : X_1.functions) {
    auto it = std::find_if(X_2.functions.begin(), X_2.functions.end(), [&](plansys2::Function f_2) {
      return parser::pddl::checkNodeEquality(f_1, f_2);
    });
    if (it != X_2.functions.end()) {
      ret.functions.push_back(f_1);
    }
  }

  return ret;
}

/**
 * @brief 获取动作的条件
 * @param action 动作
 * @return 动作的条件
 * @details
 * 根据动作的类型，返回对应的条件。
 */
plansys2_msgs::msg::Tree STNBTBuilder::get_conditions(const ActionStamped& action) const {
  if (action.type == ActionType::START || action.type == ActionType::GOAL) {
    return action.action->at_start_requirements;
  } else if (action.type == ActionType::OVERALL) {
    return action.action->over_all_requirements;
  } else if (action.type == ActionType::END) {
    return action.action->at_end_requirements;
  }

  return plansys2_msgs::msg::Tree();
}

/**
 * @brief 获取动作的效果
 * @param action 动作
 * @return 动作的效果
 * @details
 * 根据动作的类型，返回对应的效果。
 */
plansys2_msgs::msg::Tree STNBTBuilder::get_effects(const ActionStamped& action) const {
  if (action.type == ActionType::START) {
    return action.action->at_start_effects;
  } else if (action.type == ActionType::END || action.type == ActionType::INIT) {
    return action.action->at_end_effects;
  }

  return plansys2_msgs::msg::Tree();
}

/**
 * @brief 从当前节点到根节点遍历图，删除不必要的路径
 * @param current 当前节点
 * @param previous 前一个节点
 * @details 遍历图，从前一个节点到根节点，删除与当前节点无关的路径。
 *          如果前一个节点和当前节点是起点和终点，则保留这两个节点之间的路径。
 *          如果前一个节点和当前节点不是起点和终点，则删除它们之间的路径。
 */
void STNBTBuilder::prune_paths(GraphNode::Ptr current, GraphNode::Ptr previous) const {
  // 1. 从前一个节点到根节点遍历图
  for (auto& in : previous->input_arcs) {
    prune_paths(current, std::get<0>(in));
  }

  // 2. 如果前一个节点和当前节点是起点和终点，则保留这两个节点之间的路径
  if (previous->action.time == current->action.time) {
    if (previous->action.expression == current->action.expression) {
      if (previous->action.type != ActionType::START) {
        std::cerr << "prune_paths: Expected previous action to be of type START" << std::endl;
      }
      if (current->action.type != ActionType::END) {
        std::cerr << "prune_paths: Expected current action to be of type END" << std::endl;
      }
      return;
    }
  }

  // 3. 删除前一个节点和当前节点之间的路径
  auto it = previous->output_arcs.begin();
  while (it != previous->output_arcs.end()) {
    // 检查是否有输出链接到当前节点
    if (std::get<0>(*it) == current) {
      // 找到相应的输入链接
      auto in = std::find_if(
          current->input_arcs.begin(), current->input_arcs.end(),
          [&](std::tuple<GraphNode::Ptr, double, double> arc) {
            return std::get<0>(arc) == previous;
          });
      // 删除输出和输入链接
      if (in != current->input_arcs.end()) {
        current->input_arcs.erase(in);
        it = previous->output_arcs.erase(it);
      } else {
        std::cerr << "prune_backards: Input arc could not be found" << std::endl;
      }
    } else {
      ++it;
    }
  }
}

/**
 * @brief 检查从当前节点到根节点的路径上是否存在给定的前一个节点
 * @param current 当前节点
 * @param previous 给定的前一个节点
 * @return 如果存在给定的前一个节点，返回true；否则返回false
 * @details 遍历从当前节点到根节点的路径，检查路径上是否存在给定的前一个节点
 */
bool STNBTBuilder::check_paths(GraphNode::Ptr current, GraphNode::Ptr previous) const {
  // 从当前节点遍历图形到根节点
  for (auto& in : current->input_arcs) {
    if (check_paths(std::get<0>(in), previous)) {  // 递归调用check_paths函数
      return true;
    }
  }

  // 检查当前节点是否等于前一个节点
  if (current == previous) {
    return true;
  }

  return false;
}

/**
 * @brief 获取节点的执行流程
 * @param node 节点指针
 * @param parent 父节点指针
 * @param used 已使用的节点集合
 * @param level 当前节点深度
 * @return 执行流程字符串
 * @details 根据传入的节点信息，获取其执行流程，并返回对应的字符串。
 * 1. 判断当前节点是否已经被使用过，如果是，则返回等待动作的执行流程。
 * 2. 将当前节点加入已使用的节点集合中。
 * 3. 如果当前节点没有输出边，则判断其动作类型，如果是 END
 * 类型，则返回结束执行块；否则输出错误信息并返回空字符串。
 * 4.
 * 如果当前节点有输出边，则根据其动作类型生成对应的执行块，并遍历其所有输出边，递归获取每个子节点的执行流程，并将其拼接为完整的执行流程。
 * 5. 返回完整的执行流程字符串。
 */
std::string STNBTBuilder::get_flow(
    const GraphNode::Ptr node,       // 节点指针
    const GraphNode::Ptr parent,     // 父节点指针
    std::set<GraphNode::Ptr>& used,  // 已使用的节点集合
    const int& level) const {        // 当前节点深度
  int l = level;                     // 当前节点深度
  const auto action_id = to_action_id(node->action, action_time_precision_);  // 获取节点的动作 ID

  // 判断当前节点是否已经被使用过，如果是，则返回等待动作的执行流程
  if (used.find(node) != used.end()) {
    return t(l) + "<WaitAction action=\"" + action_id + "\"/>\n";
  }

  used.insert(node);  // 将当前节点加入已使用的节点集合中

  // 如果当前节点没有输出边，则判断其动作类型，如果是 END
  // 类型，则返回结束执行块；否则输出错误信息并返回空字符串。
  if (node->output_arcs.size() == 0) {
    if (node->action.type == ActionType::END) {
      return end_execution_block(node, parent, l);
    }
    std::cerr << "get_flow: Unexpected action type" << std::endl;
    return {};
  }

  std::string flow;  // 执行流程字符串

  // 如果当前节点不是 INIT 类型，则生成 Sequence 块
  if (node->action.type != ActionType::INIT) {
    flow = flow + t(l) + "<Sequence name=\"" + action_id + "\">\n";
  }

  // 根据节点的动作类型生成对应的执行块
  if (node->action.type == ActionType::START) {
    flow = flow + start_execution_block(node, parent, l + 1);
  } else if (node->action.type == ActionType::END) {
    flow = flow + end_execution_block(node, parent, l + 1);
  }

  int n = 0;                           // 并行执行阈值
  if (node->output_arcs.size() > 1) {  // 如果当前节点有多个输出边，则生成 Parallel 块
    flow = flow + t(l + 1) + "<Parallel success_threshold=\"" +
           std::to_string(node->output_arcs.size()) + "\" failure_threshold=\"1\">\n";
    n = 1;
  }

  // 先访问 END 类型的子节点
  if (node->action.type == ActionType::START) {
    auto end_action = std::find_if(
        node->output_arcs.begin(), node->output_arcs.end(),
        std::bind(&STNBTBuilder::is_end, this, std::placeholders::_1, node->action));
    if (end_action != node->output_arcs.end()) {
      const auto& next = std::get<0>(*end_action);
      flow = flow + get_flow(next, node, used, l + n + 1);  // 递归获取子节点的执行流程
    }
  }

  // 访问其余输出边的子节点
  for (const auto& child : node->output_arcs) {
    if (!is_end(child, node->action)) {
      const auto& next = std::get<0>(child);
      flow = flow + get_flow(next, node, used, l + n + 1);  // 递归获取子节点的执行流程
    }
  }

  if (node->output_arcs.size() > 1) {  // 如果当前节点有多个输出边，则结束 Parallel 块
    flow = flow + t(l + 1) + "</Parallel>\n";
  }

  if (node->action.type != ActionType::INIT) {  // 如果当前节点不是 INIT 类型，则结束 Sequence 块
    flow = flow + t(l) + "</Sequence>\n";
  }

  return flow;  // 返回完整的执行流程字符串
}

/**
 * @brief 开始执行块的函数
 * @param node 当前节点
 * @param parent 父节点
 * @param l 缩进量
 * @details 根据当前节点和父节点生成开始执行块的字符串，其中包括等待之前动作完成的 WaitAction
 * 和当前动作的 Action。
 */
std::string STNBTBuilder::start_execution_block(
    const GraphNode::Ptr node, const GraphNode::Ptr parent, const int& l) const {
  std::string ret;
  std::string ret_aux = bt_start_action_;  // 获取开始执行块的模板字符串
  const std::string action_id =
      to_action_id(node->action, action_time_precision_);  // 将当前节点的动作转换成字符串

  std::string wait_actions;
  for (const auto& prev : node->input_arcs) {  // 遍历当前节点的所有输入边
    const auto& prev_node = std::get<0>(prev);
    if (prev_node != parent) {  // 如果输入边的起点不是父节点，则需要等待该动作完成
      wait_actions = wait_actions + t(1) + "<WaitAction action=\"" +
                     to_action_id(prev_node->action, action_time_precision_) + "\"/>";
    }

    if (prev !=
        *node->input_arcs.rbegin()) {  // 如果不是最后一条输入边，则需要在字符串末尾添加换行符
      wait_actions = wait_actions + "\n";
    }
  }

  replace(ret_aux, "ACTION_ID", action_id);  // 将模板字符串中的 ACTION_ID 替换为当前节点的动作
  replace(
      ret_aux, "WAIT_PREV_ACTIONS",
      wait_actions);  // 将模板字符串中的 WAIT_PREV_ACTIONS 替换为等待之前动作完成的字符串

  std::istringstream f(ret_aux);
  std::string line;
  while (std::getline(f, line)) {  // 将生成的字符串按行缩进
    if (line != "") {
      ret = ret + t(l) + line + "\n";
    }
  }
  return ret;  // 返回生成的字符串
}

/**
 * @brief 结束执行块的函数
 * @param node 当前节点
 * @param parent 父节点
 * @param l 缩进量
 * @details 根据当前节点和父节点生成结束执行块的字符串，其中包括检查之前动作是否完成的 CheckAction
 * 和当前动作的 Action。
 */
std::string STNBTBuilder::end_execution_block(
    const GraphNode::Ptr node, const GraphNode::Ptr parent, const int& l) const {
  std::string ret;
  std::string ret_aux = bt_end_action_;  // 获取结束执行块的模板字符串
  const std::string action_id =
      to_action_id(node->action, action_time_precision_);  // 将当前节点的动作转换成字符串

  std::string check_actions;
  for (const auto& prev : node->input_arcs) {  // 遍历当前节点的所有输入边
    const auto& prev_node = std::get<0>(prev);
    if (prev_node != parent) {  // 如果输入边的起点不是父节点，则需要检查该动作是否完成
      check_actions = check_actions + t(1) + "<CheckAction action=\"" +
                      to_action_id(prev_node->action, action_time_precision_) + "\"/>";
    }

    if (prev !=
        *node->input_arcs.rbegin()) {  // 如果不是最后一条输入边，则需要在字符串末尾添加换行符
      check_actions = check_actions + "\n";
    }
  }

  replace(ret_aux, "ACTION_ID", action_id);  // 将模板字符串中的 ACTION_ID 替换为当前节点的动作
  replace(
      ret_aux, "CHECK_PREV_ACTIONS",
      check_actions);  // 将模板字符串中的 CHECK_PREV_ACTIONS 替换为检查之前动作是否完成的字符串

  std::istringstream f(ret_aux);
  std::string line;
  while (std::getline(f, line)) {  // 将生成的字符串按行缩进
    if (line != "") {
      ret = ret + t(l) + line + "\n";
    }
  }
  return ret;  // 返回生成的字符串
}

/**
 * @brief 获取节点的输出边和子节点信息，并将其转化为dot格式
 * @param node 节点指针
 * @param edges 存储所有边的set容器
 * @details 递归遍历所有子节点，获取每个子节点的编号并与父节点连接，最终生成dot格式的字符串
 */
void STNBTBuilder::get_flow_dotgraph(GraphNode::Ptr node, std::set<std::string>& edges) {
  for (const auto& arc : node->output_arcs) {  // 遍历当前节点的所有输出边
    auto child = std::get<0>(arc);             // 获取子节点指针
    std::string edge = std::to_string(node->node_num) + "->" + std::to_string(child->node_num) +
                       ";\n";         // 将父节点编号和子节点编号拼接成一条边
    edges.insert(edge);               // 将边存入set容器中
    get_flow_dotgraph(child, edges);  // 递归遍历子节点
  }
}

/**
 * @brief 获取节点的dot格式字符串
 * @param node 节点指针
 * @param action_map 存储动作执行信息的map指针
 * @return 返回生成的dot格式字符串
 * @details 根据节点的动作类型、状态等信息，生成对应的dot格式字符串
 */
std::string STNBTBuilder::get_node_dotgraph(
    GraphNode::Ptr node, std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map) {
  std::stringstream ss;
  ss << t(2) << node->node_num << " [label=\"";  // 将节点编号和标签拼接成一行
  ss << parser::pddl::nameActionsToString(node->action.action);  // 获取动作名称并添加到标签中
  ss << " " << to_string(node->action.type) << "\"";  // 将动作类型添加到标签中
  ss << "labeljust=c,style=filled";                   // 设置标签居中对齐，填充颜色

  auto status = get_action_status(node->action, action_map);  // 获取动作执行状态
  switch (status) {  // 根据不同的状态设置不同的颜色
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
  return ss.str();  // 返回生成的字符串
}

/**
 * @brief 获取动作的执行状态
 * @param action_stamped 动作信息
 * @param action_map 存储动作执行信息的map指针
 * @return 返回动作的执行状态
 * @details 根据动作信息在map中查找对应的执行信息，如果存在则返回该动作的状态，否则返回IDLE状态
 */
ActionExecutor::Status STNBTBuilder::get_action_status(
    ActionStamped action_stamped,
    std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map) {
  auto index = "(" + parser::pddl::nameActionsToString(action_stamped.action) + "):" +
               std::to_string(
                   static_cast<int>(action_stamped.time * 1000));  // 将动作名称和时间拼接成一个索引
  if (action_map->find(index) != action_map->end()) {  // 在map中查找该索引
    if ((*action_map)[index].action_executor) {  // 如果存在对应的执行器，则返回其内部状态
      return (*action_map)[index].action_executor->get_internal_status();
    } else {
      return ActionExecutor::IDLE;  // 否则返回IDLE状态
    }
  } else {
    return ActionExecutor::IDLE;  // 如果map中不存在该索引，则返回IDLE状态
  }
}

/**
 * @brief 添加图例
 * @param level_counter 当前层数
 * @param node_counter 当前节点数
 * @return 返回一个字符串，表示添加完图例后的dot格式的字符串
 * @details
 * 1. 定义一个stringstream类型的变量ss
 * 2.
 * 定义两个int类型的变量legend_counter和legend_node_counter，并将它们初始化为level_counter和node_counter
 * 3. 在ss中添加一行t(1)的缩进和"subgraph cluster_" + legend_counter++ + " {\n"的字符串
 * 4. 在ss中添加一行t(2)的缩进和"label = \"Legend\";\n"的字符串
 * 5. 在ss中添加一行t(2)的缩进和"subgraph cluster_" + legend_counter++ + " {\n"的字符串
 * 6. 在ss中添加一行t(3)的缩进和"label = \"Plan Timestep (sec): X.X\";\n"的字符串
 * 7. 在ss中添加一行t(3)的缩进和"style = rounded;\n"的字符串
 * 8. 在ss中添加一行t(3)的缩进和"color = yellow3;\n"的字符串
 * 9. 在ss中添加一行t(3)的缩进和"bgcolor = lemonchiffon;\n"的字符串
 * 10. 在ss中添加一行t(3)的缩进和"labeljust = l;\n"的字符串
 * 11. 在ss中添加一行legend_node_counter++和"[label=\n\"Finished
 * "action\n\",labeljust=c,style=filled,color=green4,fillcolor=seagreen2];\n"的字符串
 * 12. 在ss中添加一行legend_node_counter++和"[label=\n\"Failed
 * action\n\",labeljust=c,style=filled,color=red,fillcolor=pink];\n"的字符串
 * 13. 在ss中添加一行legend_node_counter++和"[label=\n\"Current
 * action\n\",labeljust=c,style=filled,color=blue,fillcolor=skyblue];\n"的字符串
 * 14. 在ss中添加一行legend_node_counter++和"[label=\n\"Future
 * action\n\",labeljust=c,style=filled,color=yellow3,fillcolor=lightgoldenrod1];\n"的字符串
 * 15. 在ss中添加一行t(2)的缩进和"}\n"的字符串
 * 16.
 * 在ss中添加一行t(2)的缩进和一个for循环，循环变量为i，从node_counter到legend_node_counter-1，每次循环添加一个节点的编号和箭头符号"->"，如果i>node_counter，则在前面添加箭头符号
 * 17. 在ss中添加一行t(1)的缩进和"}\n"的字符串
 * 18. 返回ss.str()表示的字符串
 */
std::string STNBTBuilder::add_dot_graph_legend(int level_counter, int node_counter) {
  std::stringstream ss;
  int legend_counter = level_counter;
  int legend_node_counter = node_counter;

  ss << t(1);
  ss << "subgraph cluster_" << legend_counter++ << " {\n";
  ss << t(2);
  ss << "label = \"Legend\";\n";

  ss << t(2);
  ss << "subgraph cluster_" << legend_counter++ << " {\n";
  ss << t(3);
  ss << "label = \"Plan Timestep (sec): X.X\";\n";
  ss << t(3);
  ss << "style = rounded;\n";
  ss << t(3);
  ss << "color = yellow3;\n";
  ss << t(3);
  ss << "bgcolor = lemonchiffon;\n";
  ss << t(3);
  ss << "labeljust = l;\n";
  ss << t(3);
  ss << legend_node_counter++
     << " [label=\n\"Finished "
        "action\n\",labeljust=c,style=filled,color=green4,fillcolor=seagreen2];\n";
  ss << t(3);
  ss << legend_node_counter++
     << " [label=\n\"Failed action\n\",labeljust=c,style=filled,color=red,fillcolor=pink];\n";
  ss << t(3);
  ss << legend_node_counter++
     << " [label=\n\"Current action\n\",labeljust=c,style=filled,color=blue,fillcolor=skyblue];\n";
  ss << t(3);
  ss << legend_node_counter++ << " [label=\n\"Future action\n\",labeljust=c,style=filled,"
     << "color=yellow3,fillcolor=lightgoldenrod1];\n";
  ss << t(2);
  ss << "}\n";

  ss << t(2);
  for (int i = node_counter; i < legend_node_counter; i++) {
    if (i > node_counter) {
      ss << "->";
    }
    ss << i;
  }
  ss << " [style=invis];\n";

  ss << t(1);
  ss << "}\n";

  return ss.str();
}

/**
 * @brief 打印图
 * @param graph 待打印的图
 * @details
 * 1. 调用print_node函数，打印graph的第一个节点和0作为缩进层数
 */
void STNBTBuilder::print_graph(const plansys2::Graph::Ptr graph) const {
  print_node(graph->nodes.front(), 0);
}

/**
 * @brief 打印节点信息
 * @param node 要打印的节点
 * @param level 当前节点的层数
 * @details 输出节点的编号、时间、动作名称、参数、类型、持续时间和输出弧的边界等信息
 */
void STNBTBuilder::print_node(const plansys2::GraphNode::Ptr node, int level) const {
  std::cerr << t(level) << "(" << node->node_num << ") ";
  if (node->action.type == ActionType::START) {
    std::cerr << node->action.time;
  } else {
    std::cerr << node->action.time + node->action.duration;
  }
  std::cerr << ": (" << node->action.action->name;
  for (const auto& param : node->action.action->parameters) {
    std::cerr << " " << param.name;
  }
  std::cerr << ")_" << to_string(node->action.type);
  std::cerr << "  [" << node->action.duration << "]";
  for (const auto& arc : node->output_arcs) {
    auto lower = std::get<1>(arc);
    auto upper = std::get<2>(arc);
    std::cerr << " [" << lower << ", " << upper << "]";
  }
  std::cerr << std::endl;

  for (const auto& arc : node->output_arcs) {
    auto child = std::get<0>(arc);
    print_node(child, level + 1);
  }
}

/**
 * @brief 将字符串中的某个子串替换为另一个子串
 * @param str 待处理的字符串
 * @param from 要被替换的子串
 * @param to 替换后的子串
 */
void STNBTBuilder::replace(std::string& str, const std::string& from, const std::string& to) const {
  size_t start_pos = std::string::npos;
  while ((start_pos = str.find(from)) != std::string::npos) {
    str.replace(start_pos, from.length(), to);
  }
}

/**
 * @brief 判断某个边是否为结束动作
 * @param edge 待判断的边
 * @param action 当前动作
 * @return 如果该边是结束动作，则返回 true，否则返回 false
 * @details 判断条件：1. 该边连接的节点是一个结束动作；2.
 * 该边连接的节点的时间与当前动作的时间相同；3. 该边连接的节点的表达式与当前动作的表达式相同。
 */
bool STNBTBuilder::is_end(
    const std::tuple<GraphNode::Ptr, double, double>& edge, const ActionStamped& action) const {
  const auto& node = std::get<0>(edge);
  auto t_1 = to_int_time(node->action.time, action_time_precision_ + 1);
  auto t_2 = to_int_time(action.time, action_time_precision_ + 1);
  return action.type == ActionType::START && node->action.type == ActionType::END && (t_1 == t_2) &&
         (node->action.expression == action.expression);
}

/**
 * @brief 根据层数生成缩进字符串
 * @param level 当前层数
 * @return 缩进字符串
 */
std::string STNBTBuilder::t(const int& level) const {
  std::string ret;
  for (int i = 0; i < level; i++) {
    ret = ret + "  ";
  }
  return ret;
}

}  // namespace plansys2
