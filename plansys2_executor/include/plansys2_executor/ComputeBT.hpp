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

#ifndef PLANSYS2_EXECUTOR__COMPUTEBT_HPP_
#define PLANSYS2_EXECUTOR__COMPUTEBT_HPP_

#include <memory>
#include <string>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_executor/BTBuilder.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_planner/PlannerNode.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertNode.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace plansys2 {

/**
 * @brief ComputeBT类，继承自rclcpp_lifecycle::LifecycleNode
 * @details 该类用于计算并生成行为树（Behavior Tree）
 *
 * 参数列表：
 * - action_bt_xml_: 行为树的XML文件路径
 * - start_action_bt_xml_: 起始行为树的XML文件路径
 * - end_action_bt_xml_: 终止行为树的XML文件路径
 * - bt_builder_loader_: 行为树构建器的插件库加载器
 * - domain_node_: 领域专家节点
 * - planner_node_: 规划器节点
 * - problem_node_: 问题专家节点
 * - domain_client_: 领域专家客户端
 * - problem_client_: 问题专家客户端
 * - planner_client_: 规划器客户端
 * - compute_bt_srv_: 计算行为树的服务
 * - dotgraph_pub_: 发布行为树的dot图形式
 */
class ComputeBT : public rclcpp_lifecycle::LifecycleNode {
public:
  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  ComputeBT();

  /**
   * @brief 生命周期配置回调函数
   * @param state 生命周期状态
   * @return CallbackReturnT 生命周期回调返回值
   */
  CallbackReturnT on_configure(const rclcpp_lifecycle::State& state);

  /**
   * @brief 生命周期激活回调函数
   * @param state 生命周期状态
   * @return CallbackReturnT 生命周期回调返回值
   */
  CallbackReturnT on_activate(const rclcpp_lifecycle::State& state);

  /**
   * @brief 生命周期停用回调函数
   * @param state 生命周期状态
   * @return CallbackReturnT 生命周期回调返回值
   */
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State& state);

  /**
   * @brief 生命周期清理回调函数
   * @param state 生命周期状态
   * @return CallbackReturnT 生命周期回调返回值
   */
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State& state);

  /**
   * @brief 生命周期关闭回调函数
   * @param state 生命周期状态
   * @return CallbackReturnT 生命周期回调返回值
   */
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State& state);

  /**
   * @brief 生命周期错误回调函数
   * @param state 生命周期状态
   * @return CallbackReturnT 生命周期回调返回值
   */
  CallbackReturnT on_error(const rclcpp_lifecycle::State& state);

private:
  std::string action_bt_xml_;        // 行为树的XML文件路径
  std::string start_action_bt_xml_;  // 起始行为树的XML文件路径
  std::string end_action_bt_xml_;    // 终止行为树的XML文件路径
  pluginlib::ClassLoader<plansys2::BTBuilder> bt_builder_loader_;  // 行为树构建器的插件库加载器

  std::shared_ptr<plansys2::DomainExpertNode> domain_node_;            // 领域专家节点
  std::shared_ptr<plansys2::PlannerNode> planner_node_;                // 规划器节点
  std::shared_ptr<plansys2::ProblemExpertNode> problem_node_;          // 问题专家节点

  std::shared_ptr<plansys2::DomainExpertClient> domain_client_;        // 领域专家客户端
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client_;      // 问题专家客户端
  std::shared_ptr<plansys2::PlannerClient> planner_client_;            // 规划器客户端

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr compute_bt_srv_;  // 计算行为树的服务
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr
      dotgraph_pub_;  // 发布行为树的dot图形式

  /**
   * @brief 计算行为树的回调函数
   * @param request_header 请求头部
   * @param request 请求
   * @param response 响应
   */
  void computeBTCallback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      const std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /**
   * @brief 获取问题描述文件
   * @param filename 文件名
   * @return std::string 问题描述字符串
   */
  std::string getProblem(const std::string& filename) const;

  /**
   * @brief 保存计划
   * @param plan 计划消息
   * @param filename 文件名
   */
  void savePlan(const plansys2_msgs::msg::Plan& plan, const std::string& filename) const;

  /**
   * @brief 保存行为树
   * @param bt_xml 行为树的XML字符串
   * @param filename 文件名
   */
  void saveBT(const std::string& bt_xml, const std::string& filename) const;

  /**
   * @brief 保存行为树的dot图形式
   * @param dotgraph dot图形式字符串
   * @param filename 文件名
   */
  void saveDotGraph(const std::string& dotgraph, const std::string& filename) const;
};

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__COMPUTEBT_HPP_
