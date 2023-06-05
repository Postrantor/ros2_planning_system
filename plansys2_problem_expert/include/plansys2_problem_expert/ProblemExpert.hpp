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

#ifndef PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERT_HPP_
#define PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERT_HPP_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "plansys2_domain_expert/DomainExpert.hpp"
#include "plansys2_msgs/msg/node.hpp"
#include "plansys2_msgs/msg/param.hpp"
#include "plansys2_msgs/msg/tree.hpp"
#include "plansys2_pddl_parser/Utils.h"
#include "plansys2_problem_expert/ProblemExpertInterface.hpp"

namespace plansys2 {

/**
 * @brief
 * ProblemExpert类，实现ProblemExpertInterface接口，用于管理规划问题的实例、谓词、函数和目标。
 * @param domain_expert 存储领域信息的智能指针
 * @details
 * 包含了对规划问题实例、谓词、函数和目标的增删改查等操作。同时还包含了一些辅助函数用于检查谓词树类型、删除无效谓词、函数和目标等。
 */
class ProblemExpert : public ProblemExpertInterface {
public:
  /**
   * @brief 构造函数，初始化domain_expert_和goal_
   * @param domain_expert 存储领域信息的智能指针
   */
  explicit ProblemExpert(std::shared_ptr<DomainExpert>& domain_expert);

  /**
   * @brief 获取所有实例
   * @return 所有实例的vector
   */
  std::vector<plansys2::Instance> getInstances();

  /**
   * @brief 添加一个实例
   * @param instance 要添加的实例
   * @return 添加成功返回true，否则返回false
   */
  bool addInstance(const plansys2::Instance& instance);

  /**
   * @brief 删除一个实例
   * @param instance 要删除的实例
   * @return 删除成功返回true，否则返回false
   */
  bool removeInstance(const plansys2::Instance& instance);

  /**
   * @brief 根据名称获取一个实例
   * @param name 实例名称
   * @return 如果找到则返回该实例，否则返回空
   */
  std::optional<plansys2::Instance> getInstance(const std::string& name);

  /**
   * @brief 获取所有谓词
   * @return 所有谓词的vector
   */
  std::vector<plansys2::Predicate> getPredicates();

  /**
   * @brief 添加一个谓词
   * @param predicate 要添加的谓词
   * @return 添加成功返回true，否则返回false
   */
  bool addPredicate(const plansys2::Predicate& predicate);

  /**
   * @brief 删除一个谓词
   * @param predicate 要删除的谓词
   * @return 删除成功返回true，否则返回false
   */
  bool removePredicate(const plansys2::Predicate& predicate);

  /**
   * @brief 判断一个谓词是否存在
   * @param predicate 要判断的谓词
   * @return 存在返回true，否则返回false
   */
  bool existPredicate(const plansys2::Predicate& predicate);

  /**
   * @brief 根据表达式获取一个谓词
   * @param expr 表达式
   * @return 如果找到则返回该谓词，否则返回空
   */
  std::optional<plansys2::Predicate> getPredicate(const std::string& expr);

  /**
   * @brief 获取所有函数
   * @return 所有函数的vector
   */
  std::vector<plansys2::Function> getFunctions();

  /**
   * @brief 添加一个函数
   * @param function 要添加的函数
   * @return 添加成功返回true，否则返回false
   */
  bool addFunction(const plansys2::Function& function);

  /**
   * @brief 删除一个函数
   * @param function 要删除的函数
   * @return 删除成功返回true，否则返回false
   */
  bool removeFunction(const plansys2::Function& function);

  /**
   * @brief 判断一个函数是否存在
   * @param function 要判断的函数
   * @return 存在返回true，否则返回false
   */
  bool existFunction(const plansys2::Function& function);

  /**
   * @brief 更新一个函数
   * @param function 要更新的函数
   * @return 更新成功返回true，否则返回false
   */
  bool updateFunction(const plansys2::Function& function);

  /**
   * @brief 根据表达式获取一个函数
   * @param expr 表达式
   * @return 如果找到则返回该函数，否则返回空
   */
  std::optional<plansys2::Function> getFunction(const std::string& expr);

  /**
   * @brief 获取目标
   * @return 目标
   */
  plansys2::Goal getGoal();

  /**
   * @brief 设置目标
   * @param goal 目标
   * @return 设置成功返回true，否则返回false
   */
  bool setGoal(const plansys2::Goal& goal);

  /**
   * @brief 判断目标是否已经满足
   * @param goal 目标
   * @return 已经满足返回true，否则返回false
   */
  bool isGoalSatisfied(const plansys2::Goal& goal);

  /**
   * @brief 清除目标
   * @return 清除成功返回true，否则返回false
   */
  bool clearGoal();

  /**
   * @brief 清除所有知识（实例、谓词、函数和目标）
   * @return 清除成功返回true，否则返回false
   */
  bool clearKnowledge();

  /**
   * @brief 获取问题描述字符串
   * @return 问题描述字符串
   */
  std::string getProblem();

  /**
   * @brief 添加一个问题描述字符串
   * @param problem_str 问题描述字符串
   * @return 添加成功返回true，否则返回false
   */
  bool addProblem(const std::string& problem_str);

  /**
   * @brief 判断一个实例是否存在
   * @param name 实例名称
   * @return 存在返回true，否则返回false
   */
  bool existInstance(const std::string& name);

  /**
   * @brief 判断一个类型是否合法
   * @param type 类型名称
   * @return 合法返回true，否则返回false
   */
  bool isValidType(const std::string& type);

  /**
   * @brief 判断一个谓词是否合法
   * @param predicate 要判断的谓词
   * @return 合法返回true，否则返回false
   */
  bool isValidPredicate(const plansys2::Predicate& predicate);

  /**
   * @brief 判断一个函数是否合法
   * @param function 要判断的函数
   * @return 合法返回true，否则返回false
   */
  bool isValidFunction(const plansys2::Function& function);

  /**
   * @brief 判断一个目标是否合法
   * @param goal 要判断的目标
   * @return 合法返回true，否则返回false
   */
  bool isValidGoal(const plansys2::Goal& goal);

private:
  /**
   * @brief 检查谓词树类型是否正确
   * @param tree 谓词树
   * @param domain_expert 存储领域信息的智能指针
   * @param node_id 节点id
   * @return 类型正确返回true，否则返回false
   */
  bool checkPredicateTreeTypes(
      const plansys2_msgs::msg::Tree& tree,
      std::shared_ptr<DomainExpert>& domain_expert_,
      uint8_t node_id = 0);

  /**
   * @brief 删除无效谓词
   * @param predicates 谓词vector
   * @param instance 实例
   */
  void removeInvalidPredicates(
      std::vector<plansys2::Predicate>& predicates, const plansys2::Instance& instance);

  /**
   * @brief 删除无效函数
   * @param functions 函数vector
   * @param instance 实例
   */
  void removeInvalidFunctions(
      std::vector<plansys2::Function>& functions, const plansys2::Instance& instance);

  /**
   * @brief 删除无效目标
   * @param instance 实例
   */
  void removeInvalidGoals(const plansys2::Instance& instance);

  std::vector<plansys2::Instance> instances_;    // 存储实例的vector
  std::vector<plansys2::Predicate> predicates_;  // 存储谓词的vector
  std::vector<plansys2::Function> functions_;    // 存储函数的vector
  plansys2::Goal goal_;                          // 目标

  std::shared_ptr<DomainExpert> domain_expert_;  // 存储领域信息的智能指针
};

}  // namespace plansys2

#endif  // PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERT_HPP_
