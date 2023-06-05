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

#ifndef PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERTCLIENT_HPP_
#define PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERTCLIENT_HPP_

#include <optional>
#include <string>
#include <vector>

#include "plansys2_core/Types.hpp"
#include "plansys2_msgs/msg/node.hpp"
#include "plansys2_msgs/msg/param.hpp"
#include "plansys2_msgs/msg/tree.hpp"
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
#include "plansys2_problem_expert/ProblemExpertInterface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace plansys2 {

/**
 * @brief ProblemExpertClient类，继承自ProblemExpertInterface接口
 * @details 该类用于与规划系统中的问题专家进行通信，实现对实例、谓词、函数、目标等的增删改查操作
 * @param node_ 一个ROS2节点的指针，用于创建客户端
 * @param update_time_ 记录最后一次更新时间
 */
class ProblemExpertClient : public ProblemExpertInterface {
public:
  ProblemExpertClient();

  /**
   * @brief 获取所有实例
   * @return 返回一个plansys2::Instance类型的vector，包含了所有实例
   */
  std::vector<plansys2::Instance> getInstances();

  /**
   * @brief 添加一个实例
   * @param instance 要添加的实例
   * @return 如果添加成功返回true，否则返回false
   */
  bool addInstance(const plansys2::Instance& instance);

  /**
   * @brief 移除一个实例
   * @param instance 要移除的实例
   * @return 如果移除成功返回true，否则返回false
   */
  bool removeInstance(const plansys2::Instance& instance);

  /**
   * @brief 根据名称获取一个实例
   * @param name 实例名称
   * @return 如果找到了该实例，则返回它，否则返回std::nullopt
   */
  std::optional<plansys2::Instance> getInstance(const std::string& name);

  /**
   * @brief 获取所有谓词
   * @return 返回一个plansys2::Predicate类型的vector，包含了所有谓词
   */
  std::vector<plansys2::Predicate> getPredicates();

  /**
   * @brief 添加一个谓词
   * @param predicate 要添加的谓词
   * @return 如果添加成功返回true，否则返回false
   */
  bool addPredicate(const plansys2::Predicate& predicate);

  /**
   * @brief 移除一个谓词
   * @param predicate 要移除的谓词
   * @return 如果移除成功返回true，否则返回false
   */
  bool removePredicate(const plansys2::Predicate& predicate);

  /**
   * @brief 判断是否存在某个谓词
   * @param predicate 要判断的谓词
   * @return 如果存在该谓词，则返回true，否则返回false
   */
  bool existPredicate(const plansys2::Predicate& predicate);

  /**
   * @brief 根据名称获取一个谓词
   * @param predicate 谓词名称
   * @return 如果找到了该谓词，则返回它，否则返回std::nullopt
   */
  std::optional<plansys2::Predicate> getPredicate(const std::string& predicate);

  /**
   * @brief 获取所有函数
   * @return 返回一个plansys2::Function类型的vector，包含了所有函数
   */
  std::vector<plansys2::Function> getFunctions();

  /**
   * @brief 添加一个函数
   * @param function 要添加的函数
   * @return 如果添加成功返回true，否则返回false
   */
  bool addFunction(const plansys2::Function& function);

  /**
   * @brief 移除一个函数
   * @param function 要移除的函数
   * @return 如果移除成功返回true，否则返回false
   */
  bool removeFunction(const plansys2::Function& function);

  /**
   * @brief 判断是否存在某个函数
   * @param function 要判断的函数
   * @return 如果存在该函数，则返回true，否则返回false
   */
  bool existFunction(const plansys2::Function& function);

  /**
   * @brief 更新一个函数
   * @param function 要更新的函数
   * @return 如果更新成功返回true，否则返回false
   */
  bool updateFunction(const plansys2::Function& function);

  /**
   * @brief 根据名称获取一个函数
   * @param function 函数名称
   * @return 如果找到了该函数，则返回它，否则返回std::nullopt
   */
  std::optional<plansys2::Function> getFunction(const std::string& function);

  /**
   * @brief 获取当前目标
   * @return 返回当前的plansys2::Goal类型的目标
   */
  plansys2::Goal getGoal();

  /**
   * @brief 设置新的目标
   * @param goal 新的目标
   * @return 如果设置成功返回true，否则返回false
   */
  bool setGoal(const plansys2::Goal& goal);

  /**
   * @brief 判断是否满足某个目标
   * @param goal 要判断的目标
   * @return 如果满足该目标，则返回true，否则返回false
   */
  bool isGoalSatisfied(const plansys2::Goal& goal);

  /**
   * @brief 清除当前目标
   * @return 如果清除成功返回true，否则返回false
   */
  bool clearGoal();

  /**
   * @brief 清除所有知识（实例、谓词、函数）
   * @return 如果清除成功返回true，否则返回false
   */
  bool clearKnowledge();

  /**
   * @brief 获取问题描述
   * @return 返回一个string类型的问题描述
   */
  std::string getProblem();

  /**
   * @brief 添加一个问题描述
   * @param problem_str 问题描述字符串
   * @return 如果添加成功返回true，否则返回false
   */
  bool addProblem(const std::string& problem_str);

  /**
   * @brief 获取最后一次更新时间
   * @return 返回一个rclcpp::Time类型的时间戳
   */
  rclcpp::Time getUpdateTime() const { return update_time_; }

private:
  rclcpp::Client<plansys2_msgs::srv::AddProblem>::SharedPtr add_problem_client_;
  rclcpp::Client<plansys2_msgs::srv::AddProblemGoal>::SharedPtr add_problem_goal_client_;
  rclcpp::Client<plansys2_msgs::srv::AffectParam>::SharedPtr add_problem_instance_client_;
  rclcpp::Client<plansys2_msgs::srv::AffectNode>::SharedPtr add_problem_predicate_client_;
  rclcpp::Client<plansys2_msgs::srv::AffectNode>::SharedPtr add_problem_function_client_;
  rclcpp::Client<plansys2_msgs::srv::GetProblemGoal>::SharedPtr get_problem_goal_client_;
  rclcpp::Client<plansys2_msgs::srv::GetProblemInstanceDetails>::SharedPtr
      get_problem_instance_details_client_;
  rclcpp::Client<plansys2_msgs::srv::GetProblemInstances>::SharedPtr get_problem_instances_client_;
  rclcpp::Client<plansys2_msgs::srv::GetNodeDetails>::SharedPtr
      get_problem_predicate_details_client_;
  rclcpp::Client<plansys2_msgs::srv::GetStates>::SharedPtr get_problem_predicates_client_;
  rclcpp::Client<plansys2_msgs::srv::GetNodeDetails>::SharedPtr
      get_problem_function_details_client_;
  rclcpp::Client<plansys2_msgs::srv::GetStates>::SharedPtr get_problem_functions_client_;
  rclcpp::Client<plansys2_msgs::srv::GetProblem>::SharedPtr get_problem_client_;
  rclcpp::Client<plansys2_msgs::srv::RemoveProblemGoal>::SharedPtr remove_problem_goal_client_;
  rclcpp::Client<plansys2_msgs::srv::ClearProblemKnowledge>::SharedPtr
      clear_problem_knowledge_client_;
  rclcpp::Client<plansys2_msgs::srv::AffectParam>::SharedPtr remove_problem_instance_client_;
  rclcpp::Client<plansys2_msgs::srv::AffectNode>::SharedPtr remove_problem_predicate_client_;
  rclcpp::Client<plansys2_msgs::srv::AffectNode>::SharedPtr remove_problem_function_client_;
  rclcpp::Client<plansys2_msgs::srv::ExistNode>::SharedPtr exist_problem_predicate_client_;
  rclcpp::Client<plansys2_msgs::srv::ExistNode>::SharedPtr exist_problem_function_client_;
  rclcpp::Client<plansys2_msgs::srv::AffectNode>::SharedPtr update_problem_function_client_;
  rclcpp::Client<plansys2_msgs::srv::IsProblemGoalSatisfied>::SharedPtr
      is_problem_goal_satisfied_client_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Time update_time_;
};

}  // namespace plansys2

#endif  // PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERTCLIENT_HPP_
