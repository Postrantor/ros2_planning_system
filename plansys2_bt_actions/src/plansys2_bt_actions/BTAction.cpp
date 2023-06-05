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

#include "plansys2_bt_actions/BTAction.hpp"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/utils/shared_library.h"

namespace plansys2 {

/**
 * @brief BTAction类的构造函数，继承自ActionExecutorClient类
 * @param action 行为树的名称
 * @param rate 执行频率
 * @details 构造函数中声明了一些参数，并初始化了一些变量
 */
BTAction::BTAction(const std::string& action, const std::chrono::nanoseconds& rate)
    : ActionExecutorClient(action, rate) {
  // 声明参数
  declare_parameter<std::string>("bt_xml_file", "");
  declare_parameter<std::vector<std::string>>("plugins", std::vector<std::string>({}));
  declare_parameter<bool>("bt_file_logging", false);
  declare_parameter<bool>("bt_minitrace_logging", false);
#ifdef ZMQ_FOUND
  declare_parameter<bool>("enable_groot_monitoring", true);
  declare_parameter<int>("publisher_port", -1);
  declare_parameter<int>("server_port", -1);
  declare_parameter<int>("max_msgs_per_second", 25);
#endif
}

/**
 * @brief BTAction类的on_configure回调函数
 * @param previous_state 上一个状态
 * @return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 回调返回值
 * @details 获取参数并进行相应处理
 */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn BTAction::on_configure(
    const rclcpp_lifecycle::State& previous_state) {
  get_parameter("action_name", action_);
  get_parameter("bt_xml_file", bt_xml_file_);

  RCLCPP_INFO_STREAM(get_logger(), "action_name: [" << action_ << "]");
  RCLCPP_INFO_STREAM(get_logger(), "bt_xml_file: [" << bt_xml_file_ << "]");

  auto plugin_lib_names = get_parameter("plugins").as_string_array();
  for (auto plugin : plugin_lib_names) {
    RCLCPP_INFO_STREAM(get_logger(), "plugin: [" << plugin << "]");
  }

  BT::SharedLibrary loader;

  for (auto plugin : plugin_lib_names) {
    factory_.registerFromPlugin(loader.getOSName(plugin));
  }

  blackboard_ = BT::Blackboard::create();
  blackboard_->set("node", shared_from_this());

  return ActionExecutorClient::on_configure(previous_state);
}

/**
 * @brief BTAction类的on_cleanup回调函数
 * @param previous_state 上一个状态
 * @return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 回调返回值
 * @details 对publisher_zmq_进行重置，并调用父类的on_cleanup函数
 */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn BTAction::on_cleanup(
    const rclcpp_lifecycle::State& previous_state) {
  publisher_zmq_.reset();
  return ActionExecutorClient::on_cleanup(previous_state);
}

/**
 * @brief BTAction 组件的 on_activate
 * 回调函数，实现在激活状态下创建行为树、设置黑板参数、记录日志和启用 Groot 监控等功能。
 * @param previous_state 生命周期节点的前一个状态。
 * @return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
 * 指示回调函数执行结果的枚举类型。
 * @details 该函数首先从 bt_xml_file_ 文件中创建行为树 tree_，如果创建失败则返回 FAILURE。接着将
 * get_arguments() 中的参数设置到 blackboard_ 中。 如果 bt_file_logging 或 bt_minitrace_logging
 * 参数为 true，则在 /tmp/{node_name}/ 目录下创建以时间戳命名的日志文件，并分别创建 BT::FileLogger
 * 和 BT::MinitraceLogger 对象，将日志输出到对应的文件中。 如果 enable_groot_monitoring 参数为
 * true，则获取 publisher_port、server_port 和 max_msgs_per_second 参数，创建 BT::PublisherZMQ
 * 对象并启动 Groot 监控。 最后将 finished_ 设置为 false 并调用
 * ActionExecutorClient::on_activate(previous_state) 函数。
 */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn BTAction::on_activate(
    const rclcpp_lifecycle::State& previous_state) {
  try {
    // 从 bt_xml_file_ 文件中创建行为树 tree_
    tree_ = factory_.createTreeFromFile(bt_xml_file_, blackboard_);
  } catch (const std::exception& ex) {
    // 创建失败则返回 FAILURE
    RCLCPP_ERROR_STREAM(get_logger(), "Failed to create BT with exception: " << ex.what());
    RCLCPP_ERROR(get_logger(), "Transition to activate failed");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  // 将 get_arguments() 中的参数设置到 blackboard_ 中
  for (int i = 0; i < get_arguments().size(); i++) {
    std::string argname = "arg" + std::to_string(i);
    blackboard_->set(argname, get_arguments()[i]);
  }

  // 如果 bt_file_logging 或 bt_minitrace_logging 参数为 true，则在 /tmp/{node_name}/
  // 目录下创建以时间戳命名的日志文件，并分别创建 BT::FileLogger 和 BT::MinitraceLogger
  // 对象，将日志输出到对应的文件中。
  if (get_parameter("bt_file_logging").as_bool() ||
      get_parameter("bt_minitrace_logging").as_bool()) {
    auto temp_path = std::filesystem::temp_directory_path();
    std::filesystem::path node_name_path = get_name();
    std::filesystem::create_directories(temp_path / node_name_path);

    auto now_time_t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::stringstream filename;
    filename << "/tmp/" << get_name() << "/bt_trace_";
    filename << std::put_time(std::localtime(&now_time_t), "%Y_%m_%d__%H_%M_%S");

    if (get_parameter("bt_file_logging").as_bool()) {
      std::string filename_extension = filename.str() + ".fbl";
      RCLCPP_INFO_STREAM(get_logger(), "Logging to file: " << filename_extension);
      bt_file_logger_ = std::make_unique<BT::FileLogger>(tree_, filename_extension.c_str());
    }

    if (get_parameter("bt_minitrace_logging").as_bool()) {
      std::string filename_extension = filename.str() + ".json";
      RCLCPP_INFO_STREAM(get_logger(), "Logging to file: " << filename_extension);
      bt_minitrace_logger_ =
          std::make_unique<BT::MinitraceLogger>(tree_, filename_extension.c_str());
    }
  }

#ifdef ZMQ_FOUND
  // 如果 enable_groot_monitoring 参数为 true，则获取 publisher_port、server_port 和
  // max_msgs_per_second 参数，创建 BT::PublisherZMQ 对象并启动 Groot 监控。
  int publisher_port = get_parameter("publisher_port").as_int();
  int server_port = get_parameter("server_port").as_int();
  unsigned int max_msgs_per_second = get_parameter("max_msgs_per_second").as_int();

  if (publisher_port <= 0 || server_port <= 0) {
    RCLCPP_WARN(
        get_logger(),
        "[%s] Groot monitoring ports not provided, disabling Groot monitoring."
        " publisher port: %d, server port: %d",
        get_name(), publisher_port, server_port);
  } else if (get_parameter("enable_groot_monitoring").as_bool()) {
    RCLCPP_DEBUG(
        get_logger(),
        "[%s] Groot monitoring: Publisher port: %d, Server port: %d, Max msgs per second: %d",
        get_name(), publisher_port, server_port, max_msgs_per_second);
    try {
      publisher_zmq_.reset(
          new BT::PublisherZMQ(tree_, max_msgs_per_second, publisher_port, server_port));
    } catch (const BT::LogicError& exc) {
      RCLCPP_ERROR(get_logger(), "ZMQ error: %s", exc.what());
    }
  }
#endif

  // 将 finished_ 设置为 false 并调用 ActionExecutorClient::on_activate(previous_state) 函数。
  finished_ = false;
  return ActionExecutorClient::on_activate(previous_state);
}

/**
 * 该代码段是在ROS2项目中的ros2_planning_system组件相关的代码。其中，BTAction类是一个行为树执行器，on_deactivate()函数用于重置一些变量并停止行为树的执行，do_work()函数用于执行行为树。
 *
 * on_deactivate()函数中，publisher_zmq_、bt_minitrace_logger_、bt_file_logger_三个变量被重置，tree_.haltTree()函数用于停止行为树的执行。该函数返回rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn类型的值，表示节点状态的转换结果。
 *
 * do_work()函数中，如果finished_为false，则执行行为树的executeTick()函数，并根据返回值进行相应的处理。如果executeTick()函数抛出异常，则会调用finish()函数结束行为树的执行。根据executeTick()函数的返回值，使用switch语句进行判断，分别调用finish()和send_feedback()函数，完成对行为树执行结果的处理。
 */

/**
 * @brief BTAction类的on_deactivate函数，当节点从激活状态转换到非激活状态时调用
 * @param previous_state 节点先前的状态
 * @return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
 * @details 重置publisher_zmq_、bt_minitrace_logger_、bt_file_logger_，并停止行为树的执行
 */
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn BTAction::on_deactivate(
    const rclcpp_lifecycle::State& previous_state) {
  publisher_zmq_.reset();
  bt_minitrace_logger_.reset();
  bt_file_logger_.reset();
  tree_.haltTree();

  return ActionExecutorClient::on_deactivate(previous_state);
}

/**
 * @brief BTAction类的do_work函数，执行行为树
 * @details 如果finished_为false，则执行行为树的executeTick()函数，并根据返回值进行相应的处理
 */
void BTAction::do_work() {
  if (!finished_) {
    BT::NodeStatus result;
    try {
      result = tree_.rootNode()->executeTick();
    } catch (BT::LogicError e) {
      RCLCPP_ERROR_STREAM(get_logger(), e.what());
      finish(false, 0.0, "BTAction behavior tree threw a BT::LogicError");
    } catch (BT::RuntimeError e) {
      RCLCPP_ERROR_STREAM(get_logger(), e.what());
      finish(false, 0.0, "BTAction behavior tree threw a BT::RuntimeError");
    } catch (std::exception e) {
      finish(false, 0.0, "BTAction behavior tree threw an unknown exception");
    }

    switch (result) {
      case BT::NodeStatus::SUCCESS:
        finish(true, 1.0, "BTAction behavior tree returned SUCCESS");
        finished_ = true;
        break;
      case BT::NodeStatus::RUNNING:
        send_feedback(0.0, "BTAction behavior tree returned RUNNING");
        break;
      case BT::NodeStatus::FAILURE:
        finish(false, 1.0, "BTAction behavior tree returned FAILURE");
        finished_ = true;
        break;
    }
  }
}

}  // namespace plansys2
