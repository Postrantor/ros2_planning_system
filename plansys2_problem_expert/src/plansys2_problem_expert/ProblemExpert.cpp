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

#include "plansys2_problem_expert/ProblemExpert.hpp"

#include <algorithm>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "plansys2_core/Types.hpp"
#include "plansys2_core/Utils.hpp"
#include "plansys2_pddl_parser/Domain.h"
#include "plansys2_pddl_parser/Instance.h"
#include "plansys2_problem_expert/Utils.hpp"

namespace plansys2 {

/*
ProblemExpert类是一个用于处理问题的专家，其中包含了添加、获取和移除实例的方法。\
其中，addInstance方法用于向问题中添加实例，如果实例类型无效或者实例已存在且类型不同，则添加失败；
getInstances方法用于获取问题中所有实例；removeInstance方法用于从问题中移除实例，并且会调用removeInvalidPredicates、removeInvalidFunctions和removeInvalidGoals函数，将与该实例相关的谓词、函数和目标从问题中删除。
*/

/**
 * @brief ProblemExpert类的构造函数，用于创建ProblemExpert对象
 * @param domain_expert 指向DomainExpert对象的智能指针
 */
ProblemExpert::ProblemExpert(std::shared_ptr<DomainExpert>& domain_expert)
    : domain_expert_(domain_expert) {}

/**
 * @brief 向问题中添加实例
 * @param instance 待添加的实例
 * @return 添加成功返回true，否则返回false
 * @details
 * 如果实例类型无效，则添加失败；如果实例已存在且类型不同，则添加失败；如果实例不存在，则将其添加到instances_中并返回true。
 */
bool ProblemExpert::addInstance(const plansys2::Instance& instance) {
  if (!isValidType(instance.type)) {
    return false;
  }

  std::optional<plansys2::Instance> existing_instance = getInstance(instance.name);
  bool exist_instance = existing_instance.has_value();

  if (exist_instance && existing_instance.value().type != instance.type) {
    return false;
  }

  if (!exist_instance) {
    instances_.push_back(instance);
  }

  return true;
}

/**
 * @brief 获取问题中所有实例
 * @return 返回一个vector，包含问题中的所有实例
 */
std::vector<plansys2::Instance> ProblemExpert::getInstances() { return instances_; }

/**
 * @brief 从问题中移除实例
 * @param instance 待移除的实例
 * @return 成功移除返回true，否则返回false
 * @details
 * 首先在instances_中查找该实例，如果找到则删除；然后调用removeInvalidPredicates、removeInvalidFunctions和removeInvalidGoals函数，将与该实例相关的谓词、函数和目标从问题中删除。
 */
bool ProblemExpert::removeInstance(const plansys2::Instance& instance) {
  bool found = false;
  int i = 0;

  while (!found && i < instances_.size()) {
    if (instances_[i].name == instance.name) {
      found = true;
      instances_.erase(instances_.begin() + i);
    }
    i++;
  }

  removeInvalidPredicates(predicates_, instance);
  removeInvalidFunctions(functions_, instance);
  removeInvalidGoals(instance);

  return found;
}

/**
 * @brief 获取指定名称的实例对象
 * @param instance_name 实例名称
 * @return 如果找到了该名称的实例，则返回该实例对象，否则返回空值
 * @details 遍历实例列表，查找是否存在指定名称的实例，如果找到则返回该实例对象，否则返回空值。
 */
std::optional<plansys2::Instance> ProblemExpert::getInstance(const std::string& instance_name) {
  plansys2::Instance ret;

  bool found = false;
  int i = 0;
  while (i < instances_.size() && !found) {
    if (instances_[i].name == instance_name) {
      found = true;
      ret = instances_[i];
    }
    i++;
  }

  if (found) {
    return ret;
  } else {
    return {};
  }
}

/**
 * @brief 获取谓词列表
 * @return 谓词列表
 * @details 返回存储在 ProblemExpert 中的谓词列表
 */
std::vector<plansys2::Predicate> ProblemExpert::getPredicates() { return predicates_; }

/**
 * @brief 添加谓词
 * @param predicate 待添加的谓词
 * @return 如果谓词已经存在，则返回 true；如果谓词不合法，则返回 false；否则添加谓词并返回 true。
 * @details
 * 检查待添加的谓词是否已经存在，如果不存在则检查其是否合法，如果合法则添加到谓词列表中并返回
 * true，否则返回 false。
 */
bool ProblemExpert::addPredicate(const plansys2::Predicate& predicate) {
  if (!existPredicate(predicate)) {
    if (isValidPredicate(predicate)) {
      predicates_.push_back(predicate);
      return true;
    } else {
      return false;
    }
  } else {
    return true;
  }
}

/**
 * @brief 从问题中删除一个谓词
 * @param predicate 要删除的谓词
 * @return bool 返回是否成功删除谓词
 * @details 遍历谓词列表，查找要删除的谓词，如果找到则删除并返回true，否则返回false
 */
bool ProblemExpert::removePredicate(const plansys2::Predicate& predicate) {
  bool found = false;
  int i = 0;

  if (!isValidPredicate(predicate)) {  // 如果谓词无效，则返回错误
    return false;
  }
  while (!found && i < predicates_.size()) {
    if (parser::pddl::checkNodeEquality(
            predicates_[i], predicate)) {          // 检查当前谓词与目标谓词是否相等
      found = true;
      predicates_.erase(predicates_.begin() + i);  // 删除谓词
    }
    i++;
  }

  return true;
}

/**
 * @brief 根据表达式获取一个谓词
 * @param expr 谓词表达式
 * @return std::optional<plansys2::Predicate> 返回找到的谓词，如果没有找到则返回空
 * @details 遍历谓词列表，查找与表达式匹配的谓词，如果找到则返回该谓词，否则返回空
 */
std::optional<plansys2::Predicate> ProblemExpert::getPredicate(const std::string& expr) {
  plansys2::Predicate ret;
  plansys2::Predicate pred = parser::pddl::fromStringPredicate(expr);

  bool found = false;
  size_t i = 0;
  while (i < predicates_.size() && !found) {
    if (parser::pddl::checkNodeEquality(predicates_[i], pred)) {  // 检查当前谓词与目标谓词是否相等
      found = true;
      ret = predicates_[i];
    }
    i++;
  }

  if (found) {
    return ret;  // 返回找到的谓词
  } else {
    return {};   // 返回空
  }
}

/**
 * @brief 获取所有函数列表
 * @return std::vector<plansys2::Function> 返回函数列表
 */
std::vector<plansys2::Function> ProblemExpert::getFunctions() { return functions_; }

/**
 * @brief 向问题中添加一个函数
 * @param function 要添加的函数
 * @return bool 返回是否成功添加函数
 * @details 如果函数已经存在，则更新该函数，否则将其添加到函数列表中
 */
bool ProblemExpert::addFunction(const plansys2::Function& function) {
  if (!existFunction(function)) {
    if (isValidFunction(function)) {   // 检查函数是否有效
      functions_.push_back(function);  // 添加函数
      return true;
    } else {
      return false;
    }
  } else {
    return updateFunction(function);  // 更新函数
  }
}

/**
 * @brief 从问题中移除一个函数
 * @param function 要移除的函数
 * @return bool 移除是否成功
 * @details 遍历函数列表，找到要移除的函数并删除
 */
bool ProblemExpert::removeFunction(const plansys2::Function& function) {
  bool found = false;
  int i = 0;

  if (!isValidFunction(function)) {  // 如果函数无效，返回错误
    return false;
  }
  while (!found && i < functions_.size()) {
    if (parser::pddl::checkNodeEquality(
            functions_[i], function)) {  // 检查当前函数是否与要移除的函数相等
      found = true;
      functions_.erase(functions_.begin() + i);  // 删除该函数
    }
    i++;
  }

  return true;
}

/**
 * @brief 更新问题中的一个函数
 * @param function 要更新的函数
 * @return bool 更新是否成功
 * @details 如果函数存在且有效，则先移除原有函数再添加新函数
 */
bool ProblemExpert::updateFunction(const plansys2::Function& function) {
  if (existFunction(function)) {       // 如果函数已存在
    if (isValidFunction(function)) {   // 如果函数有效
      removeFunction(function);        // 先移除原有函数
      functions_.push_back(function);  // 再添加新函数
      return true;
    } else {
      return false;  // 函数无效，更新失败
    }
  } else {
    return false;  // 函数不存在，更新失败
  }
}

/**
 * @brief 获取问题中符合表达式的函数
 * @param expr 函数表达式
 * @return std::optional<plansys2::Function> 符合表达式的函数，如果不存在则返回空
 * @details 遍历函数列表，找到符合表达式的函数并返回
 */
std::optional<plansys2::Function> ProblemExpert::getFunction(const std::string& expr) {
  plansys2::Function ret;
  plansys2::Function func = parser::pddl::fromStringFunction(expr);

  bool found = false;
  size_t i = 0;
  while (i < functions_.size() && !found) {
    if (parser::pddl::checkNodeEquality(functions_[i], func)) {  // 检查当前函数是否与表达式相等
      found = true;
      ret = functions_[i];
    }
    i++;
  }

  if (found) {
    return ret;  // 返回符合表达式的函数
  } else {
    return {};   // 返回空
  }
}

/**
 * @brief 从谓词向量中移除与实例名称匹配的无效谓词
 * @param predicates 谓词向量
 * @param instance 实例
 * @details 遍历谓词向量，查找参数列表中是否有与实例名称匹配的参数，如果有则将该谓词从向量中移除。
 */
void ProblemExpert::removeInvalidPredicates(
    std::vector<plansys2::Predicate>& predicates, const plansys2::Instance& instance) {
  for (auto rit = predicates.rbegin(); rit != predicates.rend(); ++rit) {
    if (std::find_if(
            rit->parameters.begin(), rit->parameters.end(),
            [&](const plansys2_msgs::msg::Param& param) { return param.name == instance.name; }) !=
        rit->parameters.end()) {
      predicates.erase(std::next(rit).base());
    }
  }
}

/**
 * @brief 从函数向量中移除与实例名称匹配的无效函数
 * @param functions 函数向量
 * @param instance 实例
 * @details 遍历函数向量，查找参数列表中是否有与实例名称匹配的参数，如果有则将该函数从向量中移除。
 */
void ProblemExpert::removeInvalidFunctions(
    std::vector<plansys2::Function>& functions, const plansys2::Instance& instance) {
  for (auto rit = functions.rbegin(); rit != functions.rend(); ++rit) {
    if (std::find_if(
            rit->parameters.begin(), rit->parameters.end(),
            [&](const plansys2_msgs::msg::Param& param) { return param.name == instance.name; }) !=
        rit->parameters.end()) {
      functions.erase(std::next(rit).base());
    }
  }
}

/**
 * @brief 从规划目标中移除无效的子目标
 * @param instance 要移除的实例
 * @details
 * 1. 获取规划目标中的所有子目标
 * 2. 检查是否存在子目标，如果不存在则直接返回
 * 3. 遍历所有子目标，获取子目标中的谓词和函数
 * 4. 检查谓词和函数中是否包含要移除的实例，如果包含则将该子目标移除
 * 5. 根据剩余的子目标创建一个新的规划目标
 */
void ProblemExpert::removeInvalidGoals(const plansys2::Instance& instance) {
  // Get subgoals.
  auto subgoals = parser::pddl::getSubtrees(goal_);

  // Check for subgoals before continuing.
  if (subgoals.empty()) {
    return;
  }

  // Remove invalid subgoals.
  for (auto rit = subgoals.rbegin(); rit != subgoals.rend(); ++rit) {
    // Get predicates.
    std::vector<plansys2_msgs::msg::Node> predicates;
    parser::pddl::getPredicates(predicates, *rit);

    // Check predicates for removed instance.
    bool params_valid = true;
    for (const auto& predicate : predicates) {
      if (std::find_if(
              predicate.parameters.begin(), predicate.parameters.end(),
              [&](const plansys2_msgs::msg::Param& param) {
                return param.name == instance.name;
              }) != predicate.parameters.end()) {
        params_valid = false;
        break;
      }
    }

    // Remove invalid subgoal.
    if (!params_valid) {
      subgoals.erase(std::next(rit).base());
      continue;
    }

    // Get functions.
    std::vector<plansys2_msgs::msg::Node> functions;
    parser::pddl::getFunctions(functions, *rit);

    // Check functions for removed instance.
    params_valid = true;
    for (const auto& function : functions) {
      if (std::find_if(
              function.parameters.begin(), function.parameters.end(),
              [&](const plansys2_msgs::msg::Param& param) {
                return param.name == instance.name;
              }) != function.parameters.end()) {
        params_valid = false;
        break;
      }
    }

    // Remove invalid subgoal.
    if (!params_valid) {
      subgoals.erase(std::next(rit).base());
    }
  }

  // Create a new goal from the remaining subgoals.
  auto tree = parser::pddl::fromSubtrees(subgoals, goal_.nodes[0].node_type);
  if (tree) {
    goal_ = plansys2::Goal(*tree);
  } else {
    goal_.nodes.clear();
  }
}

/**
 * @brief 获取规划系统中的目标
 * @param 无参数
 * @details 返回规划系统中的目标
 */
plansys2::Goal ProblemExpert::getGoal() { return goal_; }

/**
 * @brief 设置规划系统中的目标
 * @param goal 规划系统中的目标
 * @details 如果目标合法，则将其设置为规划系统的目标并返回true，否则返回false
 */
bool ProblemExpert::setGoal(const plansys2::Goal& goal) {
  if (isValidGoal(goal)) {
    goal_ = goal;
    return true;
  } else {
    return false;
  }
}

/**
 * @brief 检查给定的目标是否被满足
 * @param goal 给定的目标
 * @details 检查给定的目标是否被满足，如果满足则返回true，否则返回false
 */
bool ProblemExpert::isGoalSatisfied(const plansys2::Goal& goal) {
  return check(goal, predicates_, functions_);
}

/**
 * @brief 清空规划系统中的目标
 * @param 无参数
 * @details 将规划系统中的目标清空，并返回true
 */
bool ProblemExpert::clearGoal() {
  goal_.nodes.clear();
  return true;
}

/**
 * @brief 清空规划系统中的知识库
 * @param 无参数
 * @details 将规划系统中的实例、谓词和函数清空，并清空规划系统中的目标，最后返回true
 */
bool ProblemExpert::clearKnowledge() {
  instances_.clear();
  predicates_.clear();
  functions_.clear();
  clearGoal();

  return true;
}

/**
 * @brief 检查给定的类型是否合法
 * @param type 给定的类型
 * @details
 * 获取规划系统中的所有合法类型，检查给定的类型是否在其中，如果在其中则返回true，否则返回false
 */
bool ProblemExpert::isValidType(const std::string& type) {
  auto valid_types = domain_expert_->getTypes();
  auto it = std::find(valid_types.begin(), valid_types.end(), type);

  return it != valid_types.end();
}

bool ProblemExpert::existInstance(const std::string& name) {
  bool found = false;
  int i = 0;

  while (!found && i < instances_.size()) {
    if (instances_[i].name == name) {
      found = true;
    }
    i++;
  }

  return found;
}

bool ProblemExpert::existPredicate(const plansys2::Predicate& predicate) {
  bool found = false;
  int i = 0;

  while (!found && i < predicates_.size()) {
    if (parser::pddl::checkNodeEquality(predicates_[i], predicate)) {
      found = true;
    }
    i++;
  }

  return found;
}

bool ProblemExpert::existFunction(const plansys2::Function& function) {
  bool found = false;
  int i = 0;

  while (!found && i < functions_.size()) {
    if (parser::pddl::checkNodeEquality(functions_[i], function)) {
      found = true;
    }
    i++;
  }

  return found;
}

bool ProblemExpert::isValidPredicate(const plansys2::Predicate& predicate) {
  bool valid = false;

  const std::optional<plansys2::Predicate>& model_predicate =
      domain_expert_->getPredicate(predicate.name);
  if (model_predicate) {
    if (model_predicate.value().parameters.size() == predicate.parameters.size()) {
      bool same_types = true;
      int i = 0;
      while (same_types && i < predicate.parameters.size()) {
        auto arg_type = getInstance(predicate.parameters[i].name);

        if (!arg_type.has_value()) {
          same_types = false;
        } else if (arg_type.value().type != model_predicate.value().parameters[i].type) {
          bool isSubtype = false;
          for (std::string subType : model_predicate.value().parameters[i].sub_types) {
            if (arg_type.value().type == subType) {
              isSubtype = true;
              break;
            }
          }
          if (!isSubtype) {
            same_types = false;
          }
        }
        i++;
      }
      valid = same_types;
    }
  }

  return valid;
}

bool ProblemExpert::isValidFunction(const plansys2::Function& function) {
  bool valid = false;

  const std::optional<plansys2::Function>& model_function =
      domain_expert_->getFunction(function.name);
  if (model_function) {
    if (model_function.value().parameters.size() == function.parameters.size()) {
      bool same_types = true;
      int i = 0;
      while (same_types && i < function.parameters.size()) {
        auto arg_type = getInstance(function.parameters[i].name);

        if (!arg_type.has_value()) {
          same_types = false;
        } else if (arg_type.value().type != model_function.value().parameters[i].type) {
          bool isSubtype = false;
          for (std::string subType : model_function.value().parameters[i].sub_types) {
            if (arg_type.value().type == subType) {
              isSubtype = true;
              break;
            }
          }
          if (!isSubtype) {
            same_types = false;
          }
        }
        i++;
      }
      valid = same_types;
    }
  }

  return valid;
}

bool ProblemExpert::isValidGoal(const plansys2::Goal& goal) {
  return checkPredicateTreeTypes(goal, domain_expert_);
}

bool ProblemExpert::checkPredicateTreeTypes(
    const plansys2_msgs::msg::Tree& tree,
    std::shared_ptr<DomainExpert>& domain_expert,
    uint8_t node_id) {
  if (node_id >= tree.nodes.size()) {
    return false;
  }

  switch (tree.nodes[node_id].node_type) {
    case plansys2_msgs::msg::Node::AND: {
      bool ret = true;

      for (auto& child_id : tree.nodes[node_id].children) {
        ret = ret && checkPredicateTreeTypes(tree, domain_expert, child_id);
      }
      return ret;
    }

    case plansys2_msgs::msg::Node::OR: {
      bool ret = true;

      for (auto& child_id : tree.nodes[node_id].children) {
        ret = ret && checkPredicateTreeTypes(tree, domain_expert, child_id);
      }
      return ret;
    }

    case plansys2_msgs::msg::Node::NOT: {
      return checkPredicateTreeTypes(tree, domain_expert, tree.nodes[node_id].children[0]);
    }

    case plansys2_msgs::msg::Node::PREDICATE: {
      return isValidPredicate(tree.nodes[node_id]);
    }

    case plansys2_msgs::msg::Node::FUNCTION: {
      return isValidFunction(tree.nodes[node_id]);
    }

    case plansys2_msgs::msg::Node::EXPRESSION: {
      bool ret = true;

      for (auto& child_id : tree.nodes[node_id].children) {
        ret = ret && checkPredicateTreeTypes(tree, domain_expert, child_id);
      }
      return ret;
    }

    case plansys2_msgs::msg::Node::FUNCTION_MODIFIER: {
      bool ret = true;

      for (auto& child_id : tree.nodes[node_id].children) {
        ret = ret && checkPredicateTreeTypes(tree, domain_expert, child_id);
      }
      return ret;
    }

    case plansys2_msgs::msg::Node::NUMBER: {
      return true;
    }

    default:
      // LCOV_EXCL_START
      std::cerr << "checkPredicateTreeTypes: Error parsing expresion ["
                << parser::pddl::toString(tree, node_id) << "]" << std::endl;
      // LCOV_EXCL_STOP
  }

  return false;
}

std::string ProblemExpert::getProblem() {
  parser::pddl::Domain domain(domain_expert_->getDomain());
  parser::pddl::Instance problem(domain);

  problem.name = "problem_1";

  for (const auto& instance : instances_) {
    bool is_constant = domain.getType(instance.type)->parseConstant(instance.name).first;
    if (is_constant) {
      std::cout << "Skipping adding constant to problem :object: " << instance.name << " "
                << instance.type << std::endl;
    } else {
      problem.addObject(instance.name, instance.type);
    }
  }

  for (plansys2_msgs::msg::Node predicate : predicates_) {
    StringVec v;

    for (size_t i = 0; i < predicate.parameters.size(); i++) {
      v.push_back(predicate.parameters[i].name);
    }

    std::transform(predicate.name.begin(), predicate.name.end(), predicate.name.begin(), ::tolower);

    problem.addInit(predicate.name, v);
  }

  for (plansys2_msgs::msg::Node function : functions_) {
    StringVec v;

    for (size_t i = 0; i < function.parameters.size(); i++) {
      v.push_back(function.parameters[i].name);
    }

    std::transform(function.name.begin(), function.name.end(), function.name.begin(), ::tolower);

    problem.addInit(function.name, function.value, v);
  }

  const std::string gs = parser::pddl::toString(goal_);
  problem.addGoal(gs);

  std::ostringstream stream;
  stream << problem;
  return stream.str();
}

bool ProblemExpert::addProblem(const std::string& problem_str) {
  if (problem_str.empty()) {
    std::cerr << "Empty problem." << std::endl;
    return false;
  }
  parser::pddl::Domain domain(domain_expert_->getDomain());

  std::string lc_problem = problem_str;
  std::transform(problem_str.begin(), problem_str.end(), lc_problem.begin(), [](unsigned char c) {
    return std::tolower(c);
  });

  lc_problem = remove_comments(lc_problem);

  std::cout << "Domain:\n" << domain << std::endl;
  std::cout << "Problem:\n" << lc_problem << std::endl;

  parser::pddl::Instance problem(domain);

  std::string domain_name = problem.getDomainName(lc_problem);
  if (domain_name.empty()) {
    std::cerr << "Domain name is empty" << std::endl;
    return false;
  } else if (!domain_expert_->existDomain(domain_name)) {
    std::cerr << "Domain name does not exist: " << domain_name << std::endl;
    return false;
  }

  domain.name = domain_name;
  try {
    problem.parse(lc_problem);
  } catch (std::runtime_error ex) {
    // all errors thrown by the Stringreader object extend std::runtime_error
    std::cerr << ex.what() << std::endl;
    return false;
  }

  std::cout << "Parsed problem: " << problem << std::endl;

  for (unsigned i = 0; i < domain.types.size(); ++i) {
    if (domain.types[i]->constants.size()) {
      for (unsigned j = 0; j < domain.types[i]->constants.size(); ++j) {
        plansys2::Instance instance;
        instance.name = domain.types[i]->constants[j];
        instance.type = domain.types[i]->name;
        std::cout << "Adding constant: " << instance.name << " " << instance.type << std::endl;
        addInstance(instance);
      }
    }
  }

  for (unsigned i = 0; i < domain.types.size(); ++i) {
    if (domain.types[i]->objects.size()) {
      for (unsigned j = 0; j < domain.types[i]->objects.size(); ++j) {
        plansys2::Instance instance;
        instance.name = domain.types[i]->objects[j];
        instance.type = domain.types[i]->name;
        std::cout << "Adding instance: " << instance.name << " " << instance.type << std::endl;
        addInstance(instance);
      }
    }
  }

  plansys2_msgs::msg::Tree tree;
  for (auto ground : problem.init) {
    auto tree_node = ground->getTree(tree, domain);
    switch (tree_node->node_type) {
      case plansys2_msgs::msg::Node::PREDICATE: {
        plansys2::Predicate pred_node(*tree_node);
        std::cout << "Adding predicate: " << parser::pddl::toString(tree, tree_node->node_id)
                  << std::endl;
        if (!addPredicate(pred_node)) {
          std::cerr << "Failed to add predicate: "
                    << parser::pddl::toString(tree, tree_node->node_id) << std::endl;
        }
      } break;
      case plansys2_msgs::msg::Node::FUNCTION: {
        plansys2::Function func_node(*tree_node);
        std::cout << "Adding function: " << parser::pddl::toString(tree, tree_node->node_id)
                  << std::endl;
        if (!addFunction(func_node)) {
          std::cerr << "Failed to add function: "
                    << parser::pddl::toString(tree, tree_node->node_id) << std::endl;
        }
      } break;
      default:
        break;
    }
  }

  plansys2_msgs::msg::Tree goal;
  auto node = problem.goal->getTree(goal, domain);
  std::cout << "Adding Goal: " << parser::pddl::toString(goal) << std::endl;
  if (setGoal(goal)) {
    std::cout << "Goal insertion ok" << std::endl;
  } else {
    std::cout << "Goal insertion failed" << std::endl;
  }

  return true;
}

}  // namespace plansys2
