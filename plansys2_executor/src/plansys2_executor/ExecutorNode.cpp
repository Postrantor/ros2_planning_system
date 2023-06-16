// Copyright 2019 Intelligent Robotics Lab
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

#include "plansys2_executor/ExecutorNode.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/blackboard.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "plansys2_executor/ActionExecutor.hpp"
#include "plansys2_executor/BTBuilder.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_pddl_parser/Utils.h"
#include "plansys2_problem_expert/Utils.hpp"

#ifdef ZMQ_FOUND
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#endif

#include "plansys2_executor/behavior_tree/apply_atend_effect_node.hpp"
#include "plansys2_executor/behavior_tree/apply_atstart_effect_node.hpp"
#include "plansys2_executor/behavior_tree/check_action_node.hpp"
#include "plansys2_executor/behavior_tree/check_atend_req_node.hpp"
#include "plansys2_executor/behavior_tree/check_overall_req_node.hpp"
#include "plansys2_executor/behavior_tree/check_timeout_node.hpp"
#include "plansys2_executor/behavior_tree/execute_action_node.hpp"
#include "plansys2_executor/behavior_tree/wait_action_node.hpp"
#include "plansys2_executor/behavior_tree/wait_atstart_req_node.hpp"

namespace plansys2 {

using ExecutePlan = plansys2_msgs::action::ExecutePlan;
using namespace std::chrono_literals;

/**
 * @brief ExecutorNode类的构造函数，继承自rclcpp_lifecycle::LifecycleNode
 * @param 无
 * @details
 * 1.
 * 声明了一系列参数，包括默认行为、开始行为、结束行为的BT文件名，BTBuilder插件，动作时间精度，是否启用dotgraph_legend，是否打印图形等；
 * 2. 创建了ExecutePlan action server，并绑定了handle_goal、handle_cancel和handle_accepted回调函数；
 * 3. 创建了get_ordered_sub_goals_service_和get_plan_service_两个服务。
 */
ExecutorNode::ExecutorNode()
    : rclcpp_lifecycle::LifecycleNode("executor"),
      bt_builder_loader_("plansys2_executor", "plansys2::BTBuilder") {
  using namespace std::placeholders;

  this->declare_parameter<std::string>("default_action_bt_xml_filename", "");
  this->declare_parameter<std::string>("default_start_action_bt_xml_filename", "");
  this->declare_parameter<std::string>("default_end_action_bt_xml_filename", "");
  this->declare_parameter<std::string>("bt_builder_plugin", "");
  this->declare_parameter<int>("action_time_precision", 3);
  this->declare_parameter<bool>("enable_dotgraph_legend", true);
  this->declare_parameter<bool>("print_graph", false);
  this->declare_parameter("action_timeouts.actions", std::vector<std::string>{});
  // Declaring individual action parameters so they can be queried on the command line
  auto action_timeouts_actions = this->get_parameter("action_timeouts.actions").as_string_array();
  for (auto action : action_timeouts_actions) {
    this->declare_parameter<double>(
        "action_timeouts." + action + ".duration_overrun_percentage", 0.0);
  }

#ifdef ZMQ_FOUND
  this->declare_parameter<bool>("enable_groot_monitoring", true);
  this->declare_parameter<int>("publisher_port", 2666);
  this->declare_parameter<int>("server_port", 2667);
  this->declare_parameter<int>("max_msgs_per_second", 25);
#endif

  execute_plan_action_server_ = rclcpp_action::create_server<ExecutePlan>(
      this->get_node_base_interface(), this->get_node_clock_interface(),
      this->get_node_logging_interface(), this->get_node_waitables_interface(), "execute_plan",
      std::bind(&ExecutorNode::handle_goal, this, _1, _2),
      std::bind(&ExecutorNode::handle_cancel, this, _1),
      std::bind(&ExecutorNode::handle_accepted, this, _1));

  get_ordered_sub_goals_service_ = create_service<plansys2_msgs::srv::GetOrderedSubGoals>(
      "executor/get_ordered_sub_goals",
      std::bind(
          &ExecutorNode::get_ordered_sub_goals_service_callback, this, std::placeholders::_1,
          std::placeholders::_2, std::placeholders::_3));

  get_plan_service_ = create_service<plansys2_msgs::srv::GetPlan>(
      "executor/get_plan",
      std::bind(
          &ExecutorNode::get_plan_service_callback, this, std::placeholders::_1,
          std::placeholders::_2, std::placeholders::_3));
}

/**
 * @brief ExecutorNode::on_configure函数，当节点配置时调用的回调函数
 * @param state 节点状态
 * @return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
 * 回调函数返回值类型
 * @details
 *    1. 获取并设置默认行为树文件路径
 *    2. 加载行为树文件
 *    3. 创建发布器和客户端对象
 *    4. 返回回调函数执行结果
 */
CallbackReturnT ExecutorNode::on_configure(const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(get_logger(), "[%s] Configuring...", get_name());

  // 获取并设置默认行为树文件路径
  auto default_action_bt_xml_filename =
      this->get_parameter("default_action_bt_xml_filename").as_string();
  if (default_action_bt_xml_filename.empty()) {
    default_action_bt_xml_filename =
        ament_index_cpp::get_package_share_directory("plansys2_executor") +
        "/behavior_trees/plansys2_action_bt.xml";
  }

  // 加载行为树文件
  std::ifstream action_bt_ifs(default_action_bt_xml_filename);
  if (!action_bt_ifs) {
    RCLCPP_ERROR_STREAM(get_logger(), "Error openning [" << default_action_bt_xml_filename << "]");
    return CallbackReturnT::FAILURE;
  }
  action_bt_xml_.assign(
      std::istreambuf_iterator<char>(action_bt_ifs), std::istreambuf_iterator<char>());

  // 获取并设置默认开始行为树文件路径
  auto default_start_action_bt_xml_filename =
      this->get_parameter("default_start_action_bt_xml_filename").as_string();
  if (default_start_action_bt_xml_filename.empty()) {
    default_start_action_bt_xml_filename =
        ament_index_cpp::get_package_share_directory("plansys2_executor") +
        "/behavior_trees/plansys2_start_action_bt.xml";
  }

  // 加载开始行为树文件
  std::ifstream start_action_bt_ifs(default_start_action_bt_xml_filename);
  if (!start_action_bt_ifs) {
    RCLCPP_ERROR_STREAM(
        get_logger(), "Error openning [" << default_start_action_bt_xml_filename << "]");
    return CallbackReturnT::FAILURE;
  }
  start_action_bt_xml_.assign(
      std::istreambuf_iterator<char>(start_action_bt_ifs), std::istreambuf_iterator<char>());

  // 获取并设置默认结束行为树文件路径
  auto default_end_action_bt_xml_filename =
      this->get_parameter("default_end_action_bt_xml_filename").as_string();
  if (default_end_action_bt_xml_filename.empty()) {
    default_end_action_bt_xml_filename =
        ament_index_cpp::get_package_share_directory("plansys2_executor") +
        "/behavior_trees/plansys2_end_action_bt.xml";
  }

  // 加载结束行为树文件
  std::ifstream end_action_bt_ifs(default_end_action_bt_xml_filename);
  if (!end_action_bt_ifs) {
    RCLCPP_ERROR_STREAM(
        get_logger(), "Error openning [" << default_end_action_bt_xml_filename << "]");
    return CallbackReturnT::FAILURE;
  }
  end_action_bt_xml_.assign(
      std::istreambuf_iterator<char>(end_action_bt_ifs), std::istreambuf_iterator<char>());

  // 创建发布器和客户端对象
  dotgraph_pub_ = this->create_publisher<std_msgs::msg::String>("dot_graph", 1);
  execution_info_pub_ =
      create_publisher<plansys2_msgs::msg::ActionExecutionInfo>("action_execution_info", 100);
  executing_plan_pub_ = create_publisher<plansys2_msgs::msg::Plan>(
      "executing_plan", rclcpp::QoS(100).transient_local());

  domain_client_ = std::make_shared<plansys2::DomainExpertClient>();
  problem_client_ = std::make_shared<plansys2::ProblemExpertClient>();
  planner_client_ = std::make_shared<plansys2::PlannerClient>();

  RCLCPP_INFO(get_logger(), "[%s] Configured", get_name());
  return CallbackReturnT::SUCCESS;
}

/**
 * @brief ExecutorNode的on_activate函数，用于激活节点
 * @param state 节点状态
 * @details
 * 1. 输出节点正在激活的信息；
 * 2. 调用dotgraph_pub_、execution_info_pub_和executing_plan_pub_的on_activate函数；
 * 3. 输出节点已经激活的信息。
 * @return CallbackReturnT::SUCCESS
 */
CallbackReturnT ExecutorNode::on_activate(const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(get_logger(), "[%s] Activating...", get_name());
  dotgraph_pub_->on_activate();
  execution_info_pub_->on_activate();
  executing_plan_pub_->on_activate();
  RCLCPP_INFO(get_logger(), "[%s] Activated", get_name());

  return CallbackReturnT::SUCCESS;
}

/**
 * @brief ExecutorNode的on_deactivate函数，用于停用节点
 * @param state 节点状态
 * @details 1. 输出节点正在停用的信息；2.
 * 调用dotgraph_pub_和executing_plan_pub_的on_deactivate函数；3. 输出节点已经停用的信息。
 * @return CallbackReturnT::SUCCESS
 */
CallbackReturnT ExecutorNode::on_deactivate(const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(get_logger(), "[%s] Deactivating...", get_name());
  dotgraph_pub_->on_deactivate();
  executing_plan_pub_->on_deactivate();
  RCLCPP_INFO(get_logger(), "[%s] Deactivated", get_name());

  return CallbackReturnT::SUCCESS;
}

/**
 * @brief ExecutorNode的on_cleanup函数，用于清理节点
 * @param state 节点状态
 * @details
 *    1. 输出节点正在清理的信息；
 *    2. 重置dotgraph_pub_和executing_plan_pub_；
 *    3. 输出节点已经清理的信息。
 * @return CallbackReturnT::SUCCESS
 */
CallbackReturnT ExecutorNode::on_cleanup(const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(get_logger(), "[%s] Cleaning up...", get_name());
  dotgraph_pub_.reset();
  executing_plan_pub_.reset();
  RCLCPP_INFO(get_logger(), "[%s] Cleaned up", get_name());

  return CallbackReturnT::SUCCESS;
}

/**
 * @brief ExecutorNode的on_shutdown函数，用于关闭节点
 * @param state 节点状态
 * @details
 *    1. 输出节点正在关闭的信息；
 *    2. 重置dotgraph_pub_和executing_plan_pub_；
 *    3. 输出节点已经关闭的信息。
 * @return CallbackReturnT::SUCCESS
 */
CallbackReturnT ExecutorNode::on_shutdown(const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(get_logger(), "[%s] Shutting down...", get_name());
  dotgraph_pub_.reset();
  executing_plan_pub_.reset();
  RCLCPP_INFO(get_logger(), "[%s] Shutted down", get_name());

  return CallbackReturnT::SUCCESS;
}

/**
 * @brief ExecutorNode的on_error函数，用于处理节点错误
 * @param state 节点状态
 * @details 输出节点错误转换的信息。
 * @return CallbackReturnT::SUCCESS
 */
CallbackReturnT ExecutorNode::on_error(const rclcpp_lifecycle::State& state) {
  RCLCPP_ERROR(get_logger(), "[%s] Error transition", get_name());

  return CallbackReturnT::SUCCESS;
}

/**
 * @brief ExecutorNode类的get_ordered_sub_goals_service_callback函数，用于获取有序子目标
 * @param request_header 请求头部信息
 * @param request 请求信息
 * @param response 响应信息
 * @details
 *    如果当前存在有序子目标，则将其存储在响应信息中并返回true，否则返回false并提示“无当前计划”。
 */
void ExecutorNode::get_ordered_sub_goals_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetOrderedSubGoals::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetOrderedSubGoals::Response> response) {
  // 如果当前存在有序子目标
  if (ordered_sub_goals_.has_value()) {
    response->sub_goals = ordered_sub_goals_.value();  // 将有序子目标存储在响应信息中
    response->success = true;                          // 返回true
  } else {
    response->success = false;                         // 返回false
    response->error_info = "No current plan.";         // 提示“无当前计划”
  }
}

/**
 * @brief ExecutorNode类的getOrderedSubGoals函数，用于获取有序子目标
 * @return 返回一个optional类型的vector，包含有序的子目标
 * @details 如果当前不存在计划，则返回空向量；否则，根据计划和问题信息获取有序的子目标，并返回。
 */
std::optional<std::vector<plansys2_msgs::msg::Tree>>  //
ExecutorNode::getOrderedSubGoals() {
  // 如果当前不存在计划
  if (!current_plan_.has_value()) {
    return {};  // 返回空向量
  }

  auto goal = problem_client_->getGoal();                    // 获取问题的目标
  auto local_predicates = problem_client_->getPredicates();  // 获取问题的谓词
  auto local_functions = problem_client_->getFunctions();    // 获取问题的函数

  std::vector<plansys2_msgs::msg::Tree> ordered_goals;       // 定义有序子目标向量
  std::vector<uint32_t> unordered_subgoals =
      parser::pddl::getSubtreeIds(goal);  // 获取目标树中所有子目标的ID

  // just in case some goals are already satisfied
  // 遍历所有子目标
  for (auto it = unordered_subgoals.begin(); it != unordered_subgoals.end();) {
    // 如果该子目标已经被满足
    if (check(goal, local_predicates, local_functions, *it)) {
      plansys2_msgs::msg::Tree new_goal;  // 创建新的目标树
      parser::pddl::fromString(
          new_goal,
          "(and " + parser::pddl::toString(goal, (*it)) + ")");  // 将该子目标添加到新的目标树中
      ordered_goals.push_back(new_goal);  // 将新的目标树添加到有序子目标向量中
      it = unordered_subgoals.erase(it);  // 删除该子目标
    } else {
      ++it;
    }
  }

  // 遍历当前计划中的所有项
  for (const auto& plan_item : current_plan_.value().items) {
    std::shared_ptr<plansys2_msgs::msg::DurativeAction> action =
        domain_client_->getDurativeAction(  // 获取该项对应的动作
            get_action_name(plan_item.action), get_action_params(plan_item.action));
    apply(action->at_start_effects, local_predicates, local_functions);  // 应用动作的起始效果
    apply(action->at_end_effects, local_predicates, local_functions);  // 应用动作的结束效果

    // 遍历未满足的子目标
    for (auto it = unordered_subgoals.begin(); it != unordered_subgoals.end();) {
      // 如果该子目标已经被满足
      if (check(goal, local_predicates, local_functions, *it)) {
        plansys2_msgs::msg::Tree new_goal;  // 创建新的目标树
        parser::pddl::fromString(
            new_goal,
            "(and " + parser::pddl::toString(goal, (*it)) + ")");  // 将该子目标添加到新的目标树中
        ordered_goals.push_back(new_goal);  // 将新的目标树添加到有序子目标向量中
        it = unordered_subgoals.erase(it);  // 删除该子目标
      } else {
        ++it;
      }
    }
  }

  return ordered_goals;  // 返回有序子目标向量
}

/**
 * @brief 获取规划服务的回调函数
 * @param request_header 请求头
 * @param request 请求参数
 * @param response 响应参数
 * @details 如果当前有可用的规划，返回规划结果；否则返回错误信息。
 */
void ExecutorNode::get_plan_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetPlan::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetPlan::Response> response) {
  if (current_plan_) {
    response->success = true;
    response->plan = current_plan_.value();
  } else {
    response->success = false;
    response->error_info = "Plan not available";
  }
}

/**
 * @brief 处理目标请求
 * @param uuid 目标UUID
 * @param goal 目标请求
 * @return rclcpp_action::GoalResponse 响应结果
 * @details 处理目标请求，清空当前规划和子目标，并返回ACCEPT_AND_EXECUTE响应。
 */
rclcpp_action::GoalResponse ExecutorNode::handle_goal(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const ExecutePlan::Goal> goal) {
  RCLCPP_DEBUG(this->get_logger(), "Received goal request with order");

  current_plan_ = {};
  ordered_sub_goals_ = {};

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

/**
 * @brief 处理取消请求
 * @param goal_handle 目标句柄
 * @return rclcpp_action::CancelResponse 取消响应结果
 * @details 处理取消请求，设置取消规划的标志，并返回ACCEPT响应。
 */
rclcpp_action::CancelResponse ExecutorNode::handle_cancel(
    const std::shared_ptr<GoalHandleExecutePlan> goal_handle) {
  RCLCPP_DEBUG(this->get_logger(), "Received request to cancel goal");

  cancel_plan_requested_ = true;

  return rclcpp_action::CancelResponse::ACCEPT;
}

/**
 * @brief 执行计划的执行器节点
 * @param goal_handle 目标句柄
 * @details
 *    1. 创建反馈和结果指针；
 *    2. 将 cancel_plan_requested_ 置为 false；
 *    3. 获取当前计划；
 *    4. 如果当前计划不存在，则发布空计划，设置执行失败，并返回；
 *    5. 发布当前计划；
 *    6. 创建动作映射和超时动作映射；
 *    7. 遍历当前计划中的每个计划项，将其添加到动作映射中；
 *    8. 获取有序子目标；
 *    9. 根据 bt_builder_plugin 参数创建行为树构建器实例；
 *    10. 初始化行为树构建器；
 *    11. 创建黑板；
 *    12. 注册行为树节点类型；
 *    13. 获取行为树 xml 树；
 *    14. 发布 dotgraph 消息；
 *    15. 将行为树 xml 写入文件；
 *    16. 从文本创建行为树并返回。
 */
void ExecutorNode::execute(const std::shared_ptr<GoalHandleExecutePlan> goal_handle) {
  auto feedback = std::make_shared<ExecutePlan::Feedback>();
  auto result = std::make_shared<ExecutePlan::Result>();

  cancel_plan_requested_ = false;
  current_plan_ = goal_handle->get_goal()->plan;

  if (!current_plan_.has_value()) {
    RCLCPP_ERROR(get_logger(), "No plan found");
    result->success = false;
    goal_handle->succeed(result);

    // Publish void plan
    executing_plan_pub_->publish(plansys2_msgs::msg::Plan());
    return;
  }

  executing_plan_pub_->publish(current_plan_.value());

  auto action_map = std::make_shared<std::map<std::string, ActionExecutionInfo>>();  // 动作映射
  auto action_timeout_actions =
      this->get_parameter("action_timeouts.actions").as_string_array();  // 超时动作映射

  // 遍历计划项
  for (const auto& plan_item : current_plan_.value().items) {
    auto index = BTBuilder::to_action_id(plan_item, 3);  // 获取动作 id

    (*action_map)[index] = ActionExecutionInfo();        // 添加到动作映射中
    (*action_map)[index].action_executor =
        ActionExecutor::make_shared(plan_item.action, shared_from_this());  // 创建动作执行器
    (*action_map)[index].durative_action_info = domain_client_->getDurativeAction(
        get_action_name(plan_item.action),
        get_action_params(plan_item.action));            // 获取持续动作信息

    (*action_map)[index].duration = plan_item.duration;  // 设置动作执行时间
    std::string action_name = (*action_map)[index].durative_action_info->name;
    if (std::find(action_timeout_actions.begin(), action_timeout_actions.end(), action_name) !=
            action_timeout_actions.end() &&
        this->has_parameter("action_timeouts." + action_name + ".duration_overrun_percentage")) {
      (*action_map)[index].duration_overrun_percentage =
          this->get_parameter("action_timeouts." + action_name + ".duration_overrun_percentage")
              .as_double();
    }
    RCLCPP_INFO(
        get_logger(), "Action %s timeout percentage %f", action_name.c_str(),
        (*action_map)[index].duration_overrun_percentage);
  }

  ordered_sub_goals_ = getOrderedSubGoals();  // 获取有序子目标

  auto bt_builder_plugin =
      this->get_parameter("bt_builder_plugin").as_string();  // 获取行为树构建器插件
  if (bt_builder_plugin.empty()) {
    bt_builder_plugin = "SimpleBTBuilder";
  }

  std::shared_ptr<plansys2::BTBuilder> bt_builder;
  try {
    bt_builder = bt_builder_loader_.createSharedInstance(
        "plansys2::" + bt_builder_plugin);  // 创建行为树构建器实例
  } catch (pluginlib::PluginlibException& ex) {
    RCLCPP_ERROR(get_logger(), "pluginlib error: %s", ex.what());
  }

  // 初始化行为树构建器
  if (bt_builder_plugin == "SimpleBTBuilder") {
    bt_builder->initialize(action_bt_xml_);
  } else if (bt_builder_plugin == "STNBTBuilder") {
    auto precision = this->get_parameter("action_time_precision").as_int();
    bt_builder->initialize(start_action_bt_xml_, end_action_bt_xml_, precision);
  }

  // 创建黑板
  auto blackboard = BT::Blackboard::create();
  blackboard->set("action_map", action_map);
  blackboard->set("node", shared_from_this());
  blackboard->set("domain_client", domain_client_);
  blackboard->set("problem_client", problem_client_);

  // 注册行为树节点类型
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<ExecuteAction>("ExecuteAction");
  factory.registerNodeType<WaitAction>("WaitAction");
  factory.registerNodeType<CheckAction>("CheckAction");
  factory.registerNodeType<CheckOverAllReq>("CheckOverAllReq");
  factory.registerNodeType<WaitAtStartReq>("WaitAtStartReq");
  factory.registerNodeType<CheckAtEndReq>("CheckAtEndReq");
  factory.registerNodeType<ApplyAtStartEffect>("ApplyAtStartEffect");
  factory.registerNodeType<ApplyAtEndEffect>("ApplyAtEndEffect");
  factory.registerNodeType<CheckTimeout>("CheckTimeout");

  // 获取行为树 xml 树
  auto bt_xml_tree = bt_builder->get_tree(current_plan_.value());
  std_msgs::msg::String dotgraph_msg;
  dotgraph_msg.data = bt_builder->get_dotgraph(
      action_map, this->get_parameter("enable_dotgraph_legend").as_bool(),
      this->get_parameter("print_graph").as_bool());
  dotgraph_pub_->publish(dotgraph_msg);  // 发布 dotgraph 消息

  std::filesystem::path tp = std::filesystem::temp_directory_path();
  std::ofstream out(std::string("/tmp/") + get_namespace() + "/bt.xml");
  out << bt_xml_tree;  // 将行为树 xml 写入文件
  out.close();

  // 从文本创建行为树并返回
  auto tree = factory.createTreeFromText(bt_xml_tree, blackboard);

#ifdef ZMQ_FOUND
  unsigned int publisher_port = this->get_parameter("publisher_port").as_int();
  unsigned int server_port = this->get_parameter("server_port").as_int();
  unsigned int max_msgs_per_second = this->get_parameter("max_msgs_per_second").as_int();

  std::unique_ptr<BT::PublisherZMQ> publisher_zmq;
  if (this->get_parameter("enable_groot_monitoring").as_bool()) {
    RCLCPP_DEBUG(
        get_logger(),
        "[%s] Groot monitoring: Publisher port: %d, Server port: %d, Max msgs per second: %d",
        get_name(), publisher_port, server_port, max_msgs_per_second);
    try {
      publisher_zmq.reset(
          new BT::PublisherZMQ(tree, max_msgs_per_second, publisher_port, server_port));
    } catch (const BT::LogicError& exc) {
      RCLCPP_ERROR(get_logger(), "ZMQ error: %s", exc.what());
    }
  }
#endif

  /**
   * @brief 创建定时器并发布执行信息
   * @param action_map 行动映射
   * @details
   * 1. 使用 create_wall_timer() 函数创建一个定时器，每隔一秒钟触发一次回调函数。
   * 2. 回调函数中获取反馈信息并发布到 execution_info_pub_。
   */
  auto info_pub = create_wall_timer(1s, [this, &action_map]() {
    auto msgs = get_feedback_info(action_map);
    for (const auto& msg : msgs) {
      execution_info_pub_->publish(msg);
    }
  });

  rclcpp::Rate rate(10);                  // 设置循环频率为 10Hz
  auto status = BT::NodeStatus::RUNNING;  // 初始化行为树的状态为 RUNNING

  while (status == BT::NodeStatus::RUNNING && !cancel_plan_requested_) {
    try {
      status = tree.tickRoot();  // 执行行为树的根节点
    } catch (std::exception& e) {
      std::cerr << e.what() << std::endl;
      status == BT::NodeStatus::FAILURE;  // 如果执行出现异常，则将状态置为 FAILURE
    }

    feedback->action_execution_status = get_feedback_info(action_map);  // 获取反馈信息
    goal_handle->publish_feedback(feedback);                            // 发布反馈信息

    // 获取行为树的可视化图形，并发布
    dotgraph_msg.data = bt_builder->get_dotgraph(
        action_map, this->get_parameter("enable_dotgraph_legend").as_bool());
    dotgraph_pub_->publish(dotgraph_msg);

    rate.sleep();  // 控制循环频率
  }

  // 如果取消了计划，则停止行为树的执行
  if (cancel_plan_requested_) {
    tree.haltTree();
  }

  // 如果行为树的状态为 FAILURE，则停止行为树的执行
  if (status == BT::NodeStatus::FAILURE) {
    tree.haltTree();
    RCLCPP_ERROR(get_logger(), "Executor BT finished with FAILURE state");
  }

  // 获取行为树的可视化图形
  dotgraph_msg.data =
      bt_builder->get_dotgraph(action_map, this->get_parameter("enable_dotgraph_legend").as_bool());
  dotgraph_pub_->publish(dotgraph_msg);

  result->success = status == BT::NodeStatus::SUCCESS;  // 判断行为树的执行是否成功
  result->action_execution_status = get_feedback_info(action_map);  // 获取反馈信息

  // 遍历所有行动的执行状态
  size_t i = 0;
  while (i < result->action_execution_status.size() && result->success) {
    // 如果有一个行动的执行状态不是 SUCCEEDED，则认为执行失败
    if (result->action_execution_status[i].status !=
        plansys2_msgs::msg::ActionExecutionInfo::SUCCEEDED) {
      result->success = false;
    }
    i++;
  }

  // 如果 ROS2 节点正常运行，发送执行结果
  if (rclcpp::ok()) {
    goal_handle->succeed(result);
    if (result->success) {
      RCLCPP_INFO(this->get_logger(), "Plan Succeeded");
    } else {
      RCLCPP_INFO(this->get_logger(), "Plan Failed");
    }
  }
}

/**
 * @brief 处理接受到的执行计划目标
 * @param goal_handle 执行计划目标句柄
 * @details 通过 std::thread 创建一个线程，调用 execute 函数执行计划目标
 */
void ExecutorNode::handle_accepted(const std::shared_ptr<GoalHandleExecutePlan> goal_handle) {
  using namespace std::placeholders;
  std::thread{std::bind(&ExecutorNode::execute, this, _1), goal_handle}.detach();
}

/**
 * @brief 获取反馈信息
 * @param action_map 动作映射表
 * @return 返回 plansys2_msgs::msg::ActionExecutionInfo 类型的向量
 * @details 遍历动作映射表中的每一个动作，获取其执行状态和相关信息，并将其添加到返回的向量中
 */
std::vector<plansys2_msgs::msg::ActionExecutionInfo>  //
ExecutorNode::get_feedback_info(
    std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map) {
  std::vector<plansys2_msgs::msg::ActionExecutionInfo> ret;

  if (!action_map) {
    return ret;
  }

  for (const auto& action : *action_map) {
    if (!action.second.action_executor) {
      RCLCPP_WARN(
          get_logger(), "Action executor does not exist for %s. Skipping", action.first.c_str());
      continue;
    }

    plansys2_msgs::msg::ActionExecutionInfo info;
    switch (action.second.action_executor->get_internal_status()) {
      case ActionExecutor::IDLE:
      case ActionExecutor::DEALING:
        info.status = plansys2_msgs::msg::ActionExecutionInfo::NOT_EXECUTED;
        break;
      case ActionExecutor::RUNNING:
        info.status = plansys2_msgs::msg::ActionExecutionInfo::EXECUTING;
        break;
      case ActionExecutor::SUCCESS:
        info.status = plansys2_msgs::msg::ActionExecutionInfo::SUCCEEDED;
        break;
      case ActionExecutor::FAILURE:
        info.status = plansys2_msgs::msg::ActionExecutionInfo::FAILED;
        break;
      case ActionExecutor::CANCELLED:
        info.status = plansys2_msgs::msg::ActionExecutionInfo::CANCELLED;
        break;
    }

    info.action_full_name = action.first;

    info.start_stamp = action.second.action_executor->get_start_time();
    info.status_stamp = action.second.action_executor->get_status_time();
    info.action = action.second.action_executor->get_action_name();

    info.arguments = action.second.action_executor->get_action_params();
    info.duration = rclcpp::Duration::from_seconds(action.second.duration);
    info.completion = action.second.action_executor->get_completion();
    info.message_status = action.second.action_executor->get_feedback();

    ret.push_back(info);
  }

  return ret;
}

/**
 * @brief 打印执行信息
 * @param exec_info 一个指向存储 ActionExecutionInfo 的 map 的 shared_ptr
 * @details
 *    1. 遍历 exec_info 中的每个 action_info，输出其状态和相关信息
 *    2. 输出每个 action_info 的名称和状态
 *    3. 如果该 action_info 没有持续时间信息，则输出 "With no duration info"
 *    4. 如果该 action_info 的开始效果已应用，则输出 "At start effects applied"，否则输出 "At start
 *    effects NOT applied"
 *    5. 如果该 action_info 的结束效果已应用，则输出 "At end effects applied"，否则输出 "At end
 * effects NOT applied"
 */
void ExecutorNode::print_execution_info(
    std::shared_ptr<std::map<std::string, ActionExecutionInfo>> exec_info) {
  fprintf(stderr, "Execution info =====================\n");

  for (const auto& action_info : *exec_info) {
    fprintf(stderr, "[%s]", action_info.first.c_str());
    switch (action_info.second.action_executor->get_internal_status()) {
      case ActionExecutor::IDLE:
        fprintf(stderr, "\tIDLE\n");
        break;
      case ActionExecutor::DEALING:
        fprintf(stderr, "\tDEALING\n");
        break;
      case ActionExecutor::RUNNING:
        fprintf(stderr, "\tRUNNING\n");
        break;
      case ActionExecutor::SUCCESS:
        fprintf(stderr, "\tSUCCESS\n");
        break;
      case ActionExecutor::FAILURE:
        fprintf(stderr, "\tFAILURE\n");
        break;
    }
    if (action_info.second.durative_action_info == nullptr) {
      fprintf(stderr, "\tWith no duration info\n");
    }

    if (action_info.second.at_start_effects_applied) {
      fprintf(stderr, "\tAt start effects applied\n");
    } else {
      fprintf(stderr, "\tAt start effects NOT applied\n");
    }

    if (action_info.second.at_end_effects_applied) {
      fprintf(stderr, "\tAt end effects applied\n");
    } else {
      fprintf(stderr, "\tAt end effects NOT applied\n");
    }
  }
}

}  // namespace plansys2
