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

#ifndef PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERTINTERFACE_HPP_
#define PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERTINTERFACE_HPP_

#include <string>
#include <vector>

#include "plansys2_core/Types.hpp"
#include "plansys2_msgs/msg/node.hpp"
#include "plansys2_msgs/msg/param.hpp"
#include "plansys2_msgs/msg/tree.hpp"

namespace plansys2 {

/**
 * @brief ProblemExpertInterface 类是一个抽象类，定义了规划问题的专家接口。
 * @details
 * 该类中定义了一系列纯虚函数，用于获取、添加、删除实例、谓词、函数等，以及设置和判断目标状态是否满足等操作。
 * @param 无
 */
class ProblemExpertInterface {
public:
  /**
   * @brief 获取所有实例
   * @param 无
   * @return 返回一个 vector，包含所有实例
   */
  virtual std::vector<plansys2::Instance> getInstances() = 0;

  /**
   * @brief 添加一个实例
   * @param instance 要添加的实例
   * @return 添加成功返回 true，否则返回 false
   */
  virtual bool addInstance(const plansys2::Instance& instance) = 0;

  /**
   * @brief 删除一个实例
   * @param instance 要删除的实例
   * @return 删除成功返回 true，否则返回 false
   */
  virtual bool removeInstance(const plansys2::Instance& instance) = 0;

  /**
   * @brief 根据名称获取一个实例
   * @param name 实例名称
   * @return 如果找到了该实例，则返回该实例，否则返回空
   */
  virtual std::optional<plansys2::Instance> getInstance(const std::string& name) = 0;

  /**
   * @brief 获取所有谓词
   * @param 无
   * @return 返回一个 vector，包含所有谓词
   */
  virtual std::vector<plansys2::Predicate> getPredicates() = 0;

  /**
   * @brief 添加一个谓词
   * @param predicate 要添加的谓词
   * @return 添加成功返回 true，否则返回 false
   */
  virtual bool addPredicate(const plansys2::Predicate& predicate) = 0;

  /**
   * @brief 删除一个谓词
   * @param predicate 要删除的谓词
   * @return 删除成功返回 true，否则返回 false
   */
  virtual bool removePredicate(const plansys2::Predicate& predicate) = 0;

  /**
   * @brief 判断是否存在某个谓词
   * @param predicate 要判断的谓词
   * @return 存在返回 true，否则返回 false
   */
  virtual bool existPredicate(const plansys2::Predicate& predicate) = 0;

  /**
   * @brief 根据表达式获取一个谓词
   * @param expr 谓词表达式
   * @return 如果找到了该谓词，则返回该谓词，否则返回空
   */
  virtual std::optional<plansys2::Predicate> getPredicate(const std::string& expr) = 0;

  /**
   * @brief 获取所有函数
   * @param 无
   * @return 返回一个 vector，包含所有函数
   */
  virtual std::vector<plansys2::Function> getFunctions() = 0;

  /**
   * @brief 添加一个函数
   * @param function 要添加的函数
   * @return 添加成功返回 true，否则返回 false
   */
  virtual bool addFunction(const plansys2::Function& function) = 0;

  /**
   * @brief 删除一个函数
   * @param function 要删除的函数
   * @return 删除成功返回 true，否则返回 false
   */
  virtual bool removeFunction(const plansys2::Function& function) = 0;

  /**
   * @brief 判断是否存在某个函数
   * @param function 要判断的函数
   * @return 存在返回 true，否则返回 false
   */
  virtual bool existFunction(const plansys2::Function& function) = 0;

  /**
   * @brief 更新一个函数
   * @param function 要更新的函数
   * @return 更新成功返回 true，否则返回 false
   */
  virtual bool updateFunction(const plansys2::Function& function) = 0;

  /**
   * @brief 根据表达式获取一个函数
   * @param expr 函数表达式
   * @return 如果找到了该函数，则返回该函数，否则返回空
   */
  virtual std::optional<plansys2::Function> getFunction(const std::string& expr) = 0;

  /**
   * @brief 获取目标状态
   * @param 无
   * @return 返回当前的目标状态
   */
  virtual plansys2::Goal getGoal() = 0;

  /**
   * @brief 设置目标状态
   * @param goal 要设置的目标状态
   * @return 设置成功返回 true，否则返回 false
   */
  virtual bool setGoal(const plansys2::Goal& goal) = 0;

  /**
   * @brief 判断当前状态是否满足目标状态
   * @param goal 目标状态
   * @return 如果当前状态满足目标状态，则返回 true，否则返回 false
   */
  virtual bool isGoalSatisfied(const plansys2::Goal& goal) = 0;

  /**
   * @brief 清空目标状态
   * @param 无
   * @return 清空成功返回 true，否则返回 false
   */
  virtual bool clearGoal() = 0;

  /**
   * @brief 清空所有知识
   * @param 无
   * @return 清空成功返回 true，否则返回 false
   */
  virtual bool clearKnowledge() = 0;

  /**
   * @brief 获取问题描述
   * @param 无
   * @return 返回问题描述字符串
   */
  virtual std::string getProblem() = 0;

  /**
   * @brief 添加一个问题描述
   * @param problem_str 要添加的问题描述字符串
   * @return 添加成功返回 true，否则返回 false
   */
  virtual bool addProblem(const std::string& problem_str) = 0;
};

}  // namespace plansys2

#endif  // PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERTINTERFACE_HPP_
