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

#ifndef PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERTNODE_HPP_
#define PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERTNODE_HPP_

#include <memory>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_msgs/msg/knowledge.hpp"
#include "plansys2_msgs/srv/add_problem.hpp"
#include "plansys2_msgs/srv/add_problem_goal.hpp"
#include "plansys2_msgs/srv/affect_node.hpp"
#include "plansys2_msgs/srv/affect_param.hpp"
#include "plansys2_msgs/srv/clear_problem_knowledge.hpp"
#include "plansys2_msgs/srv/exist_node.hpp"
#include "plansys2_msgs/srv/get_node_details.hpp"
#include "plansys2_msgs/srv/get_problem.hpp"
#include "plansys2_msgs/srv/get_problem_goal.hpp"
#include "plansys2_msgs/srv/get_problem_instance_details.hpp"
#include "plansys2_msgs/srv/get_problem_instances.hpp"
#include "plansys2_msgs/srv/get_states.hpp"
#include "plansys2_msgs/srv/is_problem_goal_satisfied.hpp"
#include "plansys2_msgs/srv/remove_problem_goal.hpp"
#include "plansys2_problem_expert/ProblemExpert.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"

namespace plansys2 {

/**
 * @brief ProblemExpertNode类，继承自LifecycleNode，用于处理与问题相关的服务请求和响应
 * @details 包含多个回调函数，用于处理添加、删除、获取问题实例、谓词和函数等操作的服务请求和响应
 * @param 无
 */
class ProblemExpertNode : public rclcpp_lifecycle::LifecycleNode {
public:
  /**
   * @brief 构造函数
   * @param 无
   */
  ProblemExpertNode();

  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /**
   * @brief 生命周期回调函数，在节点配置时被调用
   * @param state 节点状态
   * @return 回调返回值
   */
  CallbackReturnT on_configure(const rclcpp_lifecycle::State& state);

  /**
   * @brief 生命周期回调函数，在节点激活时被调用
   * @param state 节点状态
   * @return 回调返回值
   */
  CallbackReturnT on_activate(const rclcpp_lifecycle::State& state);

  /**
   * @brief 生命周期回调函数，在节点失活时被调用
   * @param state 节点状态
   * @return 回调返回值
   */
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State& state);

  /**
   * @brief 生命周期回调函数，在节点清理时被调用
   * @param state 节点状态
   * @return 回调返回值
   */
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State& state);

  /**
   * @brief 生命周期回调函数，在节点关闭时被调用
   * @param state 节点状态
   * @return 回调返回值
   */
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State& state);

  /**
   * @brief 生命周期回调函数，在节点出错时被调用
   * @param state 节点状态
   * @return 回调返回值
   */
  CallbackReturnT on_error(const rclcpp_lifecycle::State& state);

  /**
   * @brief 将当前知识库的内容转换为plansys2_msgs::msg::Knowledge类型的消息并返回
   * @param 无
   * @return plansys2_msgs::msg::Knowledge类型的指针
   */
  plansys2_msgs::msg::Knowledge::SharedPtr get_knowledge_as_msg() const;

  /**
   * @brief 添加问题服务的回调函数
   * @param request_header 请求头部
   * @param request 请求
   * @param response 响应
   * @return 无
   */
  void add_problem_service_callback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<plansys2_msgs::srv::AddProblem::Request> request,
      const std::shared_ptr<plansys2_msgs::srv::AddProblem::Response> response);

  /**
   * @brief 添加问题目标服务的回调函数
   * @param request_header 请求头部
   * @param request 请求
   * @param response 响应
   * @return 无
   */
  void add_problem_goal_service_callback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<plansys2_msgs::srv::AddProblemGoal::Request> request,
      const std::shared_ptr<plansys2_msgs::srv::AddProblemGoal::Response> response);

  /**
   * @brief 添加问题实例服务的回调函数
   * @param request_header 请求头部
   * @param request 请求
   * @param response 响应
   * @return 无
   */
  void add_problem_instance_service_callback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<plansys2_msgs::srv::AffectParam::Request> request,
      const std::shared_ptr<plansys2_msgs::srv::AffectParam::Response> response);

  /**
   * @brief 添加问题谓词服务的回调函数
   * @param request_header 请求头部
   * @param request 请求
   * @param response 响应
   * @return 无
   */
  void add_problem_predicate_service_callback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<plansys2_msgs::srv::AffectNode::Request> request,
      const std::shared_ptr<plansys2_msgs::srv::AffectNode::Response> response);

  /**
   * @brief 添加问题函数服务的回调函数
   * @param request_header 请求头部
   * @param request 请求
   * @param response 响应
   * @return 无
   */
  void add_problem_function_service_callback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<plansys2_msgs::srv::AffectNode::Request> request,
      const std::shared_ptr<plansys2_msgs::srv::AffectNode::Response> response);

  /**
   * @brief 获取问题目标服务的回调函数
   * @param request_header 请求头部
   * @param request 请求
   * @param response 响应
   * @return 无
   */
  void get_problem_goal_service_callback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<plansys2_msgs::srv::GetProblemGoal::Request> request,
      const std::shared_ptr<plansys2_msgs::srv::GetProblemGoal::Response> response);

  /**
   * @brief 获取问题实例详情服务的回调函数
   * @param request_header 请求头部
   * @param request 请求
   * @param response 响应
   * @return 无
   */
  void get_problem_instance_details_service_callback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<plansys2_msgs::srv::GetProblemInstanceDetails::Request> request,
      const std::shared_ptr<plansys2_msgs::srv::GetProblemInstanceDetails::Response> response);

  /**
   * @brief 获取问题实例列表服务的回调函数
   * @param request_header 请求头部
   * @param request 请求
   * @param response 响应
   * @return 无
   */
  void get_problem_instances_service_callback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<plansys2_msgs::srv::GetProblemInstances::Request> request,
      const std::shared_ptr<plansys2_msgs::srv::GetProblemInstances::Response> response);

  /**
   * @brief 获取问题谓词详情服务的回调函数
   * @param request_header 请求头部
   * @param request 请求
   * @param response 响应
   * @return 无
   */
  void get_problem_predicate_details_service_callback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<plansys2_msgs::srv::GetNodeDetails::Request> request,
      const std::shared_ptr<plansys2_msgs::srv::GetNodeDetails::Response> response);

  /**
   * @brief 获取问题谓词列表服务的回调函数
   * @param request_header 请求头部
   * @param request 请求
   * @param response 响应
   * @return 无
   */
  void get_problem_predicates_service_callback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<plansys2_msgs::srv::GetStates::Request> request,
      const std::shared_ptr<plansys2_msgs::srv::GetStates::Response> response);

  /**
   * @brief 获取问题函数详情服务的回调函数
   * @param request_header 请求头部
   * @param request 请求
   * @param response 响应
   * @return 无
   */
  void get_problem_function_details_service_callback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<plansys2_msgs::srv::GetNodeDetails::Request> request,
      const std::shared_ptr<plansys2_msgs::srv::GetNodeDetails::Response> response);

  /**
   * @brief 获取问题函数列表服务的回调函数
   * @param request_header 请求头部
   * @param request 请求
   * @param response 响应
   * @return 无
   */
  void get_problem_functions_service_callback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<plansys2_msgs::srv::GetStates::Request> request,
      const std::shared_ptr<plansys2_msgs::srv::GetStates::Response> response);

  /**
   * @brief 获取问题服务的回调函数
   * @param request_header 请求头部
   * @param request 请求
   * @param response 响应
   * @return 无
   */
  void get_problem_service_callback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<plansys2_msgs::srv::GetProblem::Request> request,
      const std::shared_ptr<plansys2_msgs::srv::GetProblem::Response> response);

  /**
   * @brief 用于检查问题是否已经得到解决的服务回调函数
   * @param request_header ROS2请求标识符
   * @param request 请求对象，包含要检查的问题
   * @param response 响应对象，包含检查结果
   */
  void is_problem_goal_satisfied_service_callback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<plansys2_msgs::srv::IsProblemGoalSatisfied::Request> request,
      const std::shared_ptr<plansys2_msgs::srv::IsProblemGoalSatisfied::Response> response);

  /**
   * @brief 用于删除问题目标的服务回调函数
   * @param request_header ROS2请求标识符
   * @param request 请求对象，包含要删除的目标
   * @param response 响应对象，包含删除结果
   */
  void remove_problem_goal_service_callback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<plansys2_msgs::srv::RemoveProblemGoal::Request> request,
      const std::shared_ptr<plansys2_msgs::srv::RemoveProblemGoal::Response> response);

  /**
   * @brief 用于清除问题知识的服务回调函数
   * @param request_header ROS2请求标识符
   * @param request 请求对象，不包含任何参数
   * @param response 响应对象，包含清除结果
   */
  void clear_problem_knowledge_service_callback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<plansys2_msgs::srv::ClearProblemKnowledge::Request> request,
      const std::shared_ptr<plansys2_msgs::srv::ClearProblemKnowledge::Response> response);

  /**
   * @brief 用于删除问题实例的服务回调函数
   * @param request_header ROS2请求标识符
   * @param request 请求对象，包含要删除的实例
   * @param response 响应对象，包含删除结果
   */
  void remove_problem_instance_service_callback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<plansys2_msgs::srv::AffectParam::Request> request,
      const std::shared_ptr<plansys2_msgs::srv::AffectParam::Response> response);

  /**
   * @brief 用于删除问题谓词的服务回调函数
   * @param request_header ROS2请求标识符
   * @param request 请求对象，包含要删除的谓词
   * @param response 响应对象，包含删除结果
   */
  void remove_problem_predicate_service_callback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<plansys2_msgs::srv::AffectNode::Request> request,
      const std::shared_ptr<plansys2_msgs::srv::AffectNode::Response> response);

  /**
   * @brief 用于删除问题函数的服务回调函数
   * @param request_header ROS2请求标识符
   * @param request 请求对象，包含要删除的函数
   * @param response 响应对象，包含删除结果
   */
  void remove_problem_function_service_callback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<plansys2_msgs::srv::AffectNode::Request> request,
      const std::shared_ptr<plansys2_msgs::srv::AffectNode::Response> response);

  /**
   * @brief 用于检查问题谓词是否存在的服务回调函数
   * @param request_header ROS2请求标识符
   * @param request 请求对象，包含要检查的谓词
   * @param response 响应对象，包含检查结果
   */
  void exist_problem_predicate_service_callback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<plansys2_msgs::srv::ExistNode::Request> request,
      const std::shared_ptr<plansys2_msgs::srv::ExistNode::Response> response);

  /**
   * @brief 用于检查问题函数是否存在的服务回调函数
   * @param request_header ROS2请求标识符
   * @param request 请求对象，包含要检查的函数
   * @param response 响应对象，包含检查结果
   */
  void exist_problem_function_service_callback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<plansys2_msgs::srv::ExistNode::Request> request,
      const std::shared_ptr<plansys2_msgs::srv::ExistNode::Response> response);

  /**
   * @brief 用于更新问题函数的服务回调函数
   * @param request_header ROS2请求标识符
   * @param request 请求对象，包含要更新的函数
   * @param response 响应对象，包含更新结果
   */
  void update_problem_function_service_callback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<plansys2_msgs::srv::AffectNode::Request> request,
      const std::shared_ptr<plansys2_msgs::srv::AffectNode::Response> response);

private:
  std::shared_ptr<ProblemExpert> problem_expert_;

  rclcpp::Service<plansys2_msgs::srv::AddProblem>::SharedPtr add_problem_service_;
  rclcpp::Service<plansys2_msgs::srv::AddProblemGoal>::SharedPtr add_problem_goal_service_;
  rclcpp::Service<plansys2_msgs::srv::AffectParam>::SharedPtr add_problem_instance_service_;
  rclcpp::Service<plansys2_msgs::srv::AffectNode>::SharedPtr add_problem_predicate_service_;
  rclcpp::Service<plansys2_msgs::srv::AffectNode>::SharedPtr add_problem_function_service_;
  rclcpp::Service<plansys2_msgs::srv::GetProblemGoal>::SharedPtr get_problem_goal_service_;
  rclcpp::Service<plansys2_msgs::srv::GetProblemInstanceDetails>::SharedPtr
      get_problem_instance_details_service_;
  rclcpp::Service<plansys2_msgs::srv::GetProblemInstances>::SharedPtr
      get_problem_instances_service_;
  rclcpp::Service<plansys2_msgs::srv::GetNodeDetails>::SharedPtr
      get_problem_predicate_details_service_;
  rclcpp::Service<plansys2_msgs::srv::GetStates>::SharedPtr get_problem_predicates_service_;
  rclcpp::Service<plansys2_msgs::srv::GetNodeDetails>::SharedPtr
      get_problem_function_details_service_;
  rclcpp::Service<plansys2_msgs::srv::GetStates>::SharedPtr get_problem_functions_service_;
  rclcpp::Service<plansys2_msgs::srv::GetProblem>::SharedPtr get_problem_service_;
  rclcpp::Service<plansys2_msgs::srv::IsProblemGoalSatisfied>::SharedPtr
      is_problem_goal_satisfied_service_;
  rclcpp::Service<plansys2_msgs::srv::RemoveProblemGoal>::SharedPtr remove_problem_goal_service_;
  rclcpp::Service<plansys2_msgs::srv::ClearProblemKnowledge>::SharedPtr
      clear_problem_knowledge_service_;
  rclcpp::Service<plansys2_msgs::srv::AffectParam>::SharedPtr remove_problem_instance_service_;
  rclcpp::Service<plansys2_msgs::srv::AffectNode>::SharedPtr remove_problem_predicate_service_;
  rclcpp::Service<plansys2_msgs::srv::AffectNode>::SharedPtr remove_problem_function_service_;
  rclcpp::Service<plansys2_msgs::srv::ExistNode>::SharedPtr exist_problem_predicate_service_;
  rclcpp::Service<plansys2_msgs::srv::ExistNode>::SharedPtr exist_problem_function_service_;
  rclcpp::Service<plansys2_msgs::srv::AffectNode>::SharedPtr update_problem_function_service_;

  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Empty>::SharedPtr update_pub_;
  rclcpp_lifecycle::LifecyclePublisher<plansys2_msgs::msg::Knowledge>::SharedPtr knowledge_pub_;
};

}  // namespace plansys2

#endif  // PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERTNODE_HPP_
