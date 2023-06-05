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

#ifndef PLANSYS2_EXECUTOR__EXECUTORNODE_HPP_
#define PLANSYS2_EXECUTOR__EXECUTORNODE_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ActionExecutor.hpp"
#include "plansys2_executor/BTBuilder.hpp"
#include "plansys2_msgs/action/execute_plan.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_msgs/srv/get_ordered_sub_goals.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/string.hpp"

namespace plansys2 {

/**
 * @brief ExecutorNode是nav2_planner组件中的一个节点类，继承自rclcpp_lifecycle::LifecycleNode。
 *        该节点类实现了一系列回调函数和服务/动作回调函数，用于执行规划结果的动作序列，并发布执行信息。
 * @details
 * 参数列表：
 * - node_name：节点名称
 * - domain_path：领域文件路径
 * - problem_path：问题文件路径
 * - action_bt_xml：行为树xml字符串
 * - start_action_bt_xml：起始行为树xml字符串
 * - end_action_bt_xml：结束行为树xml字符串
 *
 * 公共成员函数：
 * - on_configure：配置回调函数
 * - on_activate：激活回调函数
 * - on_deactivate：去激活回调函数
 * - on_cleanup：清理回调函数
 * - on_shutdown：关闭回调函数
 * - on_error：错误回调函数
 * - get_ordered_sub_goals_service_callback：获取有序子目标服务回调函数
 * - get_plan_service_callback：获取计划服务回调函数
 *
 * 受保护成员变量：
 * - node_：节点句柄
 * - cancel_plan_requested_：是否请求取消计划
 * - current_plan_：当前计划
 * - ordered_sub_goals_：有序子目标
 * - action_bt_xml_：行为树xml字符串
 * - start_action_bt_xml_：起始行为树xml字符串
 * - end_action_bt_xml_：结束行为树xml字符串
 * - bt_builder_loader_：行为树构建器插件加载器
 * - domain_client_：领域专家客户端
 * - problem_client_：问题专家客户端
 * - planner_client_：规划器客户端
 * - execution_info_pub_：执行信息发布者
 * - executing_plan_pub_：正在执行的计划发布者
 * - execute_plan_action_server_：执行计划动作服务器
 * - get_ordered_sub_goals_service_：获取有序子目标服务
 * - dotgraph_pub_：dot图发布者
 *
 * 受保护成员函数：
 * - handle_goal：处理动作执行目标
 * - handle_cancel：处理动作执行取消请求
 * - execute：执行动作序列
 * - handle_accepted：处理动作执行目标接受
 * - get_feedback_info：获取反馈信息
 * - print_execution_info：打印执行信息
 */

/**
 * @brief ExecutorNode类，继承自rclcpp_lifecycle::LifecycleNode
 * @param node_name 节点名称
 * @param domain_path 领域文件路径
 * @param problem_path 问题文件路径
 * @param action_bt_xml 行为树xml文件路径
 * @param start_action_bt_xml 开始行为树xml文件路径
 * @param end_action_bt_xml 结束行为树xml文件路径
 *
 * @details nav2_planner组件中的ExecutorNode类，用于执行规划后的任务。
 *          包含了一系列回调函数和服务函数，以及与其他组件交互的接口。
 */
class ExecutorNode : public rclcpp_lifecycle::LifecycleNode {
public:
  using ExecutePlan = plansys2_msgs::action::ExecutePlan;
  using GoalHandleExecutePlan = rclcpp_action::ServerGoalHandle<ExecutePlan>;
  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /**
   * @brief 构造函数
   * @param node_name 节点名称
   * @param domain_path 领域文件路径
   * @param problem_path 问题文件路径
   * @param action_bt_xml 行为树xml文件路径
   * @param start_action_bt_xml 开始行为树xml文件路径
   * @param end_action_bt_xml 结束行为树xml文件路径
   */
  ExecutorNode(
      const std::string& node_name,
      const std::string& domain_path,
      const std::string& problem_path,
      const std::string& action_bt_xml,
      const std::string& start_action_bt_xml,
      const std::string& end_action_bt_xml);

  /**
   * @brief 生命周期回调函数，当节点被配置时调用
   * @param state 节点状态
   * @return 回调函数执行结果
   */
  CallbackReturnT on_configure(const rclcpp_lifecycle::State& state) override;

  /**
   * @brief 生命周期回调函数，当节点被激活时调用
   * @param state 节点状态
   * @return 回调函数执行结果
   */
  CallbackReturnT on_activate(const rclcpp_lifecycle::State& state) override;

  /**
   * @brief 生命周期回调函数，当节点被停用时调用
   * @param state 节点状态
   * @return 回调函数执行结果
   */
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State& state) override;

  /**
   * @brief 生命周期回调函数，当节点被清理时调用
   * @param state 节点状态
   * @return 回调函数执行结果
   */
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State& state) override;

  /**
   * @brief 生命周期回调函数，当节点被关闭时调用
   * @param state 节点状态
   * @return 回调函数执行结果
   */
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State& state) override;

  /**
   * @brief 生命周期回调函数，当节点出错时调用
   * @param state 节点状态
   * @return 回调函数执行结果
   */
  CallbackReturnT on_error(const rclcpp_lifecycle::State& state) override;

  /**
   * @brief 获取有序子目标的服务回调函数
   * @param request_header 请求头
   * @param request 请求参数
   * @param response 响应参数
   */
  void get_ordered_sub_goals_service_callback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<plansys2_msgs::srv::GetOrderedSubGoals::Request> request,
      std::shared_ptr<plansys2_msgs::srv::GetOrderedSubGoals::Response> response);

  /**
   * @brief 获取计划的服务回调函数
   * @param request_header 请求头
   * @param request 请求参数
   * @param response 响应参数
   */
  void get_plan_service_callback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<plansys2_msgs::srv::GetPlan::Request> request,
      std::shared_ptr<plansys2_msgs::srv::GetPlan::Response> response);

protected:
  rclcpp::Node::SharedPtr node_;

  bool cancel_plan_requested_;
  std::optional<plansys2_msgs::msg::Plan> current_plan_;
  std::optional<std::vector<plansys2_msgs::msg::Tree>> ordered_sub_goals_;

  std::string action_bt_xml_;
  std::string start_action_bt_xml_;
  std::string end_action_bt_xml_;
  pluginlib::ClassLoader<plansys2::BTBuilder> bt_builder_loader_;

  std::shared_ptr<plansys2::DomainExpertClient> domain_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;

  rclcpp_lifecycle::LifecyclePublisher<plansys2_msgs::msg::ActionExecutionInfo>::SharedPtr
      execution_info_pub_;
  rclcpp_lifecycle::LifecyclePublisher<plansys2_msgs::msg::Plan>::SharedPtr executing_plan_pub_;

  rclcpp_action::Server<ExecutePlan>::SharedPtr execute_plan_action_server_;
  rclcpp::Service<plansys2_msgs::srv::GetOrderedSubGoals>::SharedPtr get_ordered_sub_goals_service_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr dotgraph_pub_;

  /**
   * @brief 获取有序子目标
   * @return 有序子目标列表
   */
  std::optional<std::vector<plansys2_msgs::msg::Tree>> getOrderedSubGoals();

  rclcpp::Service<plansys2_msgs::srv::GetPlan>::SharedPtr get_plan_service_;

  /**
   * @brief 处理goal请求
   * @param uuid 请求的uuid
   * @param goal 请求的goal
   * @return goal响应结果
   */
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const ExecutePlan::Goal> goal);

  /**
   * @brief 处理cancel请求
   * @param goal_handle 请求的goal_handle
   * @return cancel响应结果
   */
  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleExecutePlan> goal_handle);

  /**
   * @brief 执行任务
   * @param goal_handle 请求的goal_handle
   */
  void execute(const std::shared_ptr<GoalHandleExecutePlan> goal_handle);

  /**
   * @brief 处理已接受的goal_handle
   * @param goal_handle 请求的goal_handle
   */
  void handle_accepted(const std::shared_ptr<GoalHandleExecutePlan> goal_handle);

  /**
   * @brief 获取反馈信息
   * @param action_map 行为映射表
   * @return 反馈信息列表
   */
  std::vector<plansys2_msgs::msg::ActionExecutionInfo> get_feedback_info(
      std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map);

  /**
   * @brief 打印执行信息
   * @param exec_info 执行信息
   */
  void print_execution_info(std::shared_ptr<std::map<std::string, ActionExecutionInfo>> exec_info);
};

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__EXECUTORNODE_HPP_
