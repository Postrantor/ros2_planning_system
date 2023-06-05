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

#include "plansys2_executor/ComputeBT.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/blackboard.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"

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

/**
 * @brief ComputeBT类的构造函数，继承自rclcpp_lifecycle::LifecycleNode类
 * @param 无
 * @details
 * 1. 声明并初始化了bt_builder_loader_对象，用于加载plansys2_executor中的BTBuilder插件；
 * 2. 声明并初始化了一系列参数，包括：
 *    a. action_bt_xml_filename: 行动行为树的xml文件名；
 *    b. start_action_bt_xml_filename: 开始行动行为树的xml文件名；
 *    c. end_action_bt_xml_filename: 结束行动行为树的xml文件名；
 *    d. bt_builder_plugin: BTBuilder插件的名称；
 *    e. domain: 领域描述文件的路径；
 *    f. problem: 问题描述文件的路径；
 *    g. action_time_precision: 行动时间精度，默认为3；
 *    h. enable_dotgraph_legend: 是否启用dot图例，默认为true；
 *    i. print_graph: 是否打印图形，默认为true；
 *    j. action_timeouts.actions: 行动超时列表，初始为空；
 * 3. 根据action_timeouts.actions声明每个行动的超时时间，初始值为0。
 * 4. 如果编译时开启了ZMQ_FOUND宏，则声明并初始化以下参数：
 *    a. enable_groot_monitoring: 是否启用groot监视器，默认为true；
 *    b. publisher_port: 发布者端口号，默认为2666；
 *    c. server_port: 服务器端口号，默认为2667；
 *    d. max_msgs_per_second: 每秒最大消息数，默认为25。
 * 5. 创建compute_bt服务，用于计算行动行为树，并绑定回调函数computeBTCallback。
 */
ComputeBT::ComputeBT()
    : rclcpp_lifecycle::LifecycleNode("compute_bt"),
      bt_builder_loader_("plansys2_executor", "plansys2::BTBuilder") {
  using namespace std::placeholders;

  // 2. 声明并初始化了一系列参数，包括：
  //    a. action_bt_xml_filename: 行动行为树的xml文件名；
  //    b. start_action_bt_xml_filename: 开始行动行为树的xml文件名；
  //    c. end_action_bt_xml_filename: 结束行动行为树的xml文件名；
  //    d. bt_builder_plugin: BTBuilder插件的名称；
  //    e. domain: 领域描述文件的路径；
  //    f. problem: 问题描述文件的路径；
  //    g. action_time_precision: 行动时间精度，默认为3；
  //    h. enable_dotgraph_legend: 是否启用dot图例，默认为true；
  //    i. print_graph: 是否打印图形，默认为true；
  //    j. action_timeouts.actions: 行动超时列表，初始为空；
  this->declare_parameter<std::string>("action_bt_xml_filename", "");
  this->declare_parameter<std::string>("start_action_bt_xml_filename", "");
  this->declare_parameter<std::string>("end_action_bt_xml_filename", "");
  this->declare_parameter<std::string>("bt_builder_plugin", "");
  this->declare_parameter<std::string>("domain", "");
  this->declare_parameter<std::string>("problem", "");
  this->declare_parameter<int>("action_time_precision", 3);
  this->declare_parameter<bool>("enable_dotgraph_legend", true);
  this->declare_parameter<bool>("print_graph", true);
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

  // 5. 创建compute_bt服务，用于计算行动行为树，并绑定回调函数computeBTCallback。
  compute_bt_srv_ = create_service<std_srvs::srv::Trigger>(
      "compute_bt",                       //
      std::bind(
          &ComputeBT::computeBTCallback,  //
          this,                           //
          std::placeholders::_1,          //
          std::placeholders::_2,          //
          std::placeholders::_3));
}

/**
 * @brief ComputeBT组件的on_configure回调函数
 * @param state 节点状态
 * @details
 * 1. 获取action_bt_xml_filename参数，如果为空则使用默认路径
 * 2. 打开并读取action_bt_xml文件
 * 3. 获取start_action_bt_xml_filename参数，如果为空则使用默认路径
 * 4. 打开并读取start_action_bt_xml文件
 * 5. 获取end_action_bt_xml_filename参数，如果为空则使用默认路径
 * 6. 打开并读取end_action_bt_xml文件
 * 7. 创建plan_dotgraph话题发布器
 * 8. 创建DomainExpertNode、PlannerNode、ProblemExpertNode实例
 * 9. 创建DomainExpertClient、PlannerClient、ProblemExpertClient实例
 * 10. 返回CallbackReturnT::SUCCESS
 */
using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT ComputeBT::on_configure(const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(get_logger(), "[%s] Configuring...", get_name());

  // 1. 获取action_bt_xml_filename参数，如果为空则使用默认路径
  auto action_bt_xml_filename = this->get_parameter("action_bt_xml_filename").as_string();
  if (action_bt_xml_filename.empty()) {
    action_bt_xml_filename = ament_index_cpp::get_package_share_directory("plansys2_executor") +
                             "/behavior_trees/plansys2_action_bt.xml";
  }

  // 2. 打开并读取action_bt_xml文件
  std::ifstream action_bt_ifs(action_bt_xml_filename);
  if (!action_bt_ifs) {
    RCLCPP_ERROR_STREAM(get_logger(), "Error openning [" << action_bt_xml_filename << "]");
    return CallbackReturnT::FAILURE;
  }

  action_bt_xml_.assign(
      std::istreambuf_iterator<char>(action_bt_ifs), std::istreambuf_iterator<char>());

  // 3. 获取start_action_bt_xml_filename参数，如果为空则使用默认路径
  auto start_action_bt_xml_filename =
      this->get_parameter("start_action_bt_xml_filename").as_string();
  if (start_action_bt_xml_filename.empty()) {
    start_action_bt_xml_filename =
        ament_index_cpp::get_package_share_directory("plansys2_executor") +
        "/behavior_trees/plansys2_start_action_bt.xml";
  }

  // 4. 打开并读取start_action_bt_xml文件
  std::ifstream start_action_bt_ifs(start_action_bt_xml_filename);
  if (!start_action_bt_ifs) {
    RCLCPP_ERROR_STREAM(get_logger(), "Error openning [" << start_action_bt_xml_filename << "]");
    return CallbackReturnT::FAILURE;
  }

  start_action_bt_xml_.assign(
      std::istreambuf_iterator<char>(start_action_bt_ifs), std::istreambuf_iterator<char>());

  // 5. 获取end_action_bt_xml_filename参数，如果为空则使用默认路径
  auto end_action_bt_xml_filename = this->get_parameter("end_action_bt_xml_filename").as_string();
  if (end_action_bt_xml_filename.empty()) {
    end_action_bt_xml_filename = ament_index_cpp::get_package_share_directory("plansys2_executor") +
                                 "/behavior_trees/plansys2_end_action_bt.xml";
  }

  // 6. 打开并读取end_action_bt_xml文件
  std::ifstream end_action_bt_ifs(end_action_bt_xml_filename);
  if (!end_action_bt_ifs) {
    RCLCPP_ERROR_STREAM(get_logger(), "Error openning [" << end_action_bt_xml_filename << "]");
    return CallbackReturnT::FAILURE;
  }

  end_action_bt_xml_.assign(
      std::istreambuf_iterator<char>(end_action_bt_ifs), std::istreambuf_iterator<char>());

  // 7. 创建plan_dotgraph话题发布器
  dotgraph_pub_ = this->create_publisher<std_msgs::msg::String>("plan_dotgraph", 1);

  // 8. 创建DomainExpertNode、PlannerNode、ProblemExpertNode实例
  domain_node_ = std::make_shared<plansys2::DomainExpertNode>();
  planner_node_ = std::make_shared<plansys2::PlannerNode>();
  problem_node_ = std::make_shared<plansys2::ProblemExpertNode>();

  // 9. 创建DomainExpertClient、PlannerClient、ProblemExpertClient实例
  domain_client_ = std::make_shared<plansys2::DomainExpertClient>();
  planner_client_ = std::make_shared<plansys2::PlannerClient>();
  problem_client_ = std::make_shared<plansys2::ProblemExpertClient>();

  RCLCPP_INFO(get_logger(), "[%s] Configured", get_name());
  return CallbackReturnT::SUCCESS;
}

/**
ComputeBT组件的激活回调函数：激活ComputeBT组件，并在日志中输出激活信息，同时激活dotgraph_pub_发布器。
ComputeBT组件的去激活回调函数：去激活ComputeBT组件，并在日志中输出去激活信息，同时去激活dotgraph_pub_发布器。
ComputeBT组件的清理回调函数：清理ComputeBT组件，并在日志中输出清理信息，同时重置dotgraph_pub_发布器。
ComputeBT组件的关闭回调函数：关闭ComputeBT组件，并在日志中输出关闭信息，同时重置dotgraph_pub_发布器。
ComputeBT组件的错误回调函数：在日志中输出错误信息。
*/

/**
 * @brief ComputeBT组件的激活回调函数
 * @param state 生命周期状态
 * @details 激活ComputeBT组件，并在日志中输出激活信息，同时激活dotgraph_pub_发布器
 * @return 回调返回值为CallbackReturnT::SUCCESS
 */
CallbackReturnT ComputeBT::on_activate(const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(get_logger(), "[%s] Activating...", get_name());
  dotgraph_pub_->on_activate();
  RCLCPP_INFO(get_logger(), "[%s] Activated", get_name());
  return CallbackReturnT::SUCCESS;
}

/**
 * @brief ComputeBT组件的去激活回调函数
 * @param state 生命周期状态
 * @details 去激活ComputeBT组件，并在日志中输出去激活信息，同时去激活dotgraph_pub_发布器
 * @return 回调返回值为CallbackReturnT::SUCCESS
 */
CallbackReturnT ComputeBT::on_deactivate(const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(get_logger(), "[%s] Deactivating...", get_name());
  dotgraph_pub_->on_deactivate();
  RCLCPP_INFO(get_logger(), "[%s] Deactivated", get_name());
  return CallbackReturnT::SUCCESS;
}

/**
 * @brief ComputeBT组件的清理回调函数
 * @param state 生命周期状态
 * @details 清理ComputeBT组件，并在日志中输出清理信息，同时重置dotgraph_pub_发布器
 * @return 回调返回值为CallbackReturnT::SUCCESS
 */
CallbackReturnT ComputeBT::on_cleanup(const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(get_logger(), "[%s] Cleaning up...", get_name());
  dotgraph_pub_.reset();
  RCLCPP_INFO(get_logger(), "[%s] Cleaned up", get_name());
  return CallbackReturnT::SUCCESS;
}

/**
 * @brief ComputeBT组件的关闭回调函数
 * @param state 生命周期状态
 * @details 关闭ComputeBT组件，并在日志中输出关闭信息，同时重置dotgraph_pub_发布器
 * @return 回调返回值为CallbackReturnT::SUCCESS
 */
CallbackReturnT ComputeBT::on_shutdown(const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(get_logger(), "[%s] Shutting down...", get_name());
  dotgraph_pub_.reset();
  RCLCPP_INFO(get_logger(), "[%s] Shutted down", get_name());
  return CallbackReturnT::SUCCESS;
}

/**
 * @brief ComputeBT组件的错误回调函数
 * @param state 生命周期状态
 * @details 在日志中输出错误信息
 * @return 回调返回值为CallbackReturnT::SUCCESS
 */
CallbackReturnT ComputeBT::on_error(const rclcpp_lifecycle::State& state) {
  RCLCPP_ERROR(get_logger(), "[%s] Error transition", get_name());
  return CallbackReturnT::SUCCESS;
}

/**
该函数是ComputeBT组件的回调函数，用于计算行动树（Behavior Tree）。主要包括以下几个步骤：

获取领域文件和问题文件名，并检查是否存在
配置并激活领域节点、问题节点和规划器节点
添加问题到问题客户端
获取领域、问题和计划，并保存计划、行动映射和行
*/
/**
 * @brief ComputeBT组件的回调函数，用于计算行动树（Behavior Tree）
 * @param request_header 请求头
 * @param request 请求体
 * @param response 响应体
 * @details
 * 1. 获取领域文件和问题文件名，并检查是否存在
 * 2. 配置并激活领域节点、问题节点和规划器节点
 * 3. 添加问题到问题客户端
 * 4. 获取领域、问题和计划，并保存计划、行动映射和行动树
 * 5. 创建黑板并设置相关参数
 * 6. 注册行为树节点类型
 * 7. 从XML文本创建行为树
 * 8. 发布行动树的dotgraph表示
 * 9. 如果启用了Groot监控，则使用ZMQ发布行动树
 */
void ComputeBT::computeBTCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  // 1. 获取领域文件和问题文件名，并检查是否存在
  auto domain_filename = this->get_parameter("domain").as_string();
  const std::filesystem::path domain_path{domain_filename};
  if (!std::filesystem::exists(domain_path)) {
    RCLCPP_ERROR(get_logger(), "%s not found!", domain_filename.c_str());
    return;
  }

  auto problem_filename = this->get_parameter("problem").as_string();
  const std::filesystem::path problem_path{problem_filename};
  if (!std::filesystem::exists(problem_path)) {
    RCLCPP_ERROR(get_logger(), "%s not found!", problem_filename.c_str());
    return;
  }

  auto problem_string = getProblem(problem_filename);

  // 2. 配置并激活领域节点、问题节点和规划器节点
  /**
   * @brief 设置参数
   * @param {"model_file", domain_filename} 参数名和参数值
   * @details
   * 此代码段设置了两个节点(domain_node_和problem_node_)的模型文件(model_file)参数为domain_filename。
   */
  domain_node_->set_parameter({"model_file", domain_filename});
  problem_node_->set_parameter({"model_file", domain_filename});

  /**
   * @brief 创建多线程执行器
   * @param rclcpp::ExecutorOptions() 执行器选项
   * @param 8 线程数量
   * @details 此代码段创建了一个多线程执行器(exe)，并将线程数量设置为8。
   */
  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::ExecutorOptions(), 8);

  /**
   * @brief 添加节点到执行器中
   * @param domain_node_->get_node_base_interface() 要添加的节点
   * @param problem_node_->get_node_base_interface() 要添加的节点
   * @param planner_node_->get_node_base_interface() 要添加的节点
   * @details 此代码段将三个节点(domain_node_, problem_node_, planner_node_)添加到执行器(exe)中。
   */
  exe.add_node(domain_node_->get_node_base_interface());
  exe.add_node(problem_node_->get_node_base_interface());
  exe.add_node(planner_node_->get_node_base_interface());

  /**
   * @brief 多线程执行器的轮询
   * @details
   * 此代码段使用了一个while循环，在循环中调用了exe.spin_some()函数，来处理所有已经添加到执行器中的节点。
   */
  /*
    这段代码定义了一个布尔类型的变量 `finish`，并将其初始化为 `false`。接下来创建了一个新的线程
    `t`，其中使用了一个 lambda 表达式作为线程函数，该表达式中包含一个 while 循环，只要 `finish`
    的值为 `false`，就会一直调用 `exe.spin_some()`
    函数。也就是说，这个线程会不断地处理已经添加到执行器中的节点，直到 `finish` 的值被设置为
    `true`。

    lambda 表达式是一种 C++11 引入的匿名函数，可以在需要函数对象的地方使用。它的语法形式为：

    ```
    [capture list] (parameter list) -> return type { function body }
    ```

    其中，`capture list` 用于捕获外部变量，可以为空；`parameter list` 是参数列表，可以为空；`return
    type` 是返回值类型，可以省略；`function body` 是函数体。

    在上面的代码中，使用了一个 lambda 表达式作为线程函数，其语法形式为：

    ```
    [&]() {
      while (!finish) {
        exe.spin_some();
      }
    }
    ```

    其中，`&` 表示以引用方式捕获所有外部变量，`() {}` 中是函数体，这里只有一个 while
    循环。由于没有参数和返回值，因此省略了参数列表和返回值类型。这个 lambda 表达式的作用是不断地调用
    `exe.spin_some()` 函数，直到 `finish` 的值被设置为 `true`。
  */
  bool finish = false;
  std::thread t([&]() {
    while (!finish) {
      exe.spin_some();
    }
  });

  domain_node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  problem_node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  planner_node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  domain_node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  problem_node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  planner_node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // 3. 添加问题到问题客户端
  problem_client_->addProblem(problem_string);

  // 4.1. 获取领域、问题和计划，并保存计划、行动映射和行动树
  auto domain = domain_client_->getDomain();                                         // 获取领域
  auto problem = problem_client_->getProblem();                                      // 获取问题
  auto plan = planner_client_->getPlan(domain, problem);                             // 获取计划

  savePlan(plan.value(), problem_path.stem().u8string());                            // 保存计划

  auto action_map = std::make_shared<std::map<std::string, ActionExecutionInfo>>();  // 创建行动映射

  auto action_timeout_actions =
      this->get_parameter("action_timeouts.actions").as_string_array();  // 获取行动超时时间

  for (const auto& plan_item : plan.value().items) {     // 遍历计划中的每个行动项
    auto index = BTBuilder::to_action_id(plan_item, 3);  // 将行动项转换为行动ID

    (*action_map)[index] = ActionExecutionInfo();  // 在行动映射中创建新的行动信息
    (*action_map)[index].action_executor =         // 设置行动执行器
        ActionExecutor::make_shared(plan_item.action, shared_from_this());
    (*action_map)[index].durative_action_info =
        domain_client_->getDurativeAction(  // 获取持续性行动信息
            get_action_name(plan_item.action), get_action_params(plan_item.action));

    (*action_map)[index].duration = plan_item.duration;  // 设置行动时长
    std::string action_name = (*action_map)[index].durative_action_info->name;
    if (std::find(
            action_timeout_actions.begin(), action_timeout_actions.end(),
            action_name) !=  // 如果行动在超时时间列表中
            action_timeout_actions.end() &&
        this->has_parameter("action_timeouts." + action_name + ".duration_overrun_percentage")) {
      (*action_map)[index].duration_overrun_percentage =  // 设置持续时间超出百分比
          this->get_parameter("action_timeouts." + action_name + ".duration_overrun_percentage")
              .as_double();
    }
    RCLCPP_INFO(
        get_logger(), "Action %s timeout percentage %f", action_name.c_str(),
        (*action_map)[index].duration_overrun_percentage);
  }

  // 4.2. 初始化行为树生成器
  auto bt_builder_plugin =
      this->get_parameter("bt_builder_plugin").as_string();  // 获取行为树生成器插件名称
  if (bt_builder_plugin.empty()) {  // 如果未设置插件名称，则使用默认插件
    bt_builder_plugin = "SimpleBTBuilder";
  }
  RCLCPP_INFO(get_logger(), "bt_builder_plugin: %s", bt_builder_plugin.c_str());

  std::shared_ptr<plansys2::BTBuilder> bt_builder;  // 创建行为树生成器指针
  try {
    bt_builder = bt_builder_loader_.createSharedInstance(
        "plansys2::" + bt_builder_plugin);                         // 创建行为树生成器实例
  } catch (pluginlib::PluginlibException& ex) {
    RCLCPP_ERROR(get_logger(), "pluginlib error: %s", ex.what());  // 如果创建失败，输出错误信息
  }

  if (bt_builder_plugin == "SimpleBTBuilder") {      // 如果是简单行为树生成器
    bt_builder->initialize(action_bt_xml_);          // 初始化行为树生成器
  } else if (bt_builder_plugin == "STNBTBuilder") {  // 如果是STN行为树生成器
    auto precision = this->get_parameter("action_time_precision").as_int();  // 获取时间精度
    bt_builder->initialize(
        start_action_bt_xml_, end_action_bt_xml_, precision);  // 初始化行为树生成器
  }

  // 4.3. 生成行为树并保存
  auto bt_xml = bt_builder->get_tree(plan.value());  // 生成行为树
  saveBT(bt_xml, problem_path.stem().u8string());    // 保存行为树

  // 5. 创建黑板并设置相关参数
  auto blackboard = BT::Blackboard::create();
  blackboard->set("action_map", action_map);
  blackboard->set("node", shared_from_this());
  blackboard->set("domain_client", domain_client_);
  blackboard->set("problem_client", problem_client_);

  // 6. 注册行为树节点类型
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

  // 7. 从XML文本创建行为树
  auto tree = factory.createTreeFromText(bt_xml, blackboard);

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

  finish = true;
  t.join();

  // 8. 发布行动树的dotgraph表示
  std_msgs::msg::String dotgraph_msg;
  dotgraph_msg.data = bt_builder->get_dotgraph(
      action_map, this->get_parameter("enable_dotgraph_legend").as_bool(),
      this->get_parameter("print_graph").as_bool());
  dotgraph_pub_->publish(dotgraph_msg);
  saveDotGraph(dotgraph_msg.data, problem_path.stem().u8string());

  // 9. 如果启用了Groot监控，则使用ZMQ发布行动树
  response->success = true;
}

/**
 * @brief 从指定的文件中读取字符串内容
 * @param filename 文件名
 * @return 返回读取到的字符串内容
 * @details 1. 打开指定的文件并读取其中的内容；2. 将读取到的内容存储在 std::string 类型的变量 ret
 * 中；3. 返回 ret。
 */
std::string ComputeBT::getProblem(const std::string& filename) const {
  std::string ret;
  std::ifstream file(filename);
  if (file) {
    std::ostringstream ss;
    ss << file.rdbuf();
    ret = ss.str();
  }

  return ret;
}

/**
 * @brief 将规划结果保存到文件中
 * @param plan 规划结果
 * @param filename 文件名
 * @details 1. 打开指定的文件并将规划结果写入文件；
 * 2. 写入的内容包括每个动作的时间戳、动作名称和持续时间；
 * 3. 如果文件无法打开，则输出错误信息。
 */
void ComputeBT::savePlan(const plansys2_msgs::msg::Plan& plan, const std::string& filename) const {
  std::ofstream file(filename + "_plan.pddl");
  if (file.is_open()) {
    for (const auto& item : plan.items) {
      file << item.time << ": " << item.action << "  [" << item.duration << "]\n";
    }
    file.close();
  } else {
    std::cerr << "Unable to open " << filename << "_plan.pddl" << std::endl;
  }
}

/**
 * @brief 将行为树的 XML 格式保存到文件中
 * @param bt_xml 行为树的 XML 格式
 * @param filename 文件名
 * @details 1. 打开指定的文件并将行为树的 XML 格式写入文件；2. 如果文件无法打开，则输出错误信息。
 */
void ComputeBT::saveBT(const std::string& bt_xml, const std::string& filename) const {
  std::ofstream file(filename + "_bt.xml");
  if (file.is_open()) {
    file << bt_xml;
    file.close();
  } else {
    std::cerr << "Unable to open " << filename << "_bt.xml" << std::endl;
  }
}

/**
 * @brief 将 Graphviz DOT 格式的图形保存到文件中
 * @param dotgraph Graphviz DOT 格式的图形
 * @param filename 文件名
 * @details
 * 1. 打开指定的文件并将 Graphviz DOT 格式的图形写入文件；
 * 2. 如果文件无法打开，则输出错误信息。
 */
void ComputeBT::saveDotGraph(const std::string& dotgraph, const std::string& filename) const {
  std::ofstream file(filename + "_graph.dot");
  if (file.is_open()) {
    file << dotgraph << "\n";
    file.close();
  } else {
    std::cerr << "Unable to open " << filename << "_graph.dot" << std::endl;
  }
}

}  // namespace plansys2
