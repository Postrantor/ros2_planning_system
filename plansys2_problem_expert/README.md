---
tip: translate by baidu@2023-06-05 14:02:34
...

# Problem Expert

The Problem Expert module is responsible for maintaining the instances, predicates and goals of the PDDL problem.

> Problem Expert 模块负责维护 PDDL 问题的实例、谓词和目标。

The main class is [`plansys2::ProblemExpertNode`](include/plansys2_problem_expert/ProblemExpertNode.hpp), which is instantiated from [`ProblemExpertNode.cpp`](src/ProblemExpertNode.cpp). `plansys2::ProblemExpertNode` is a `rclcpp_lifecycle::LifecycleNode`, but currently the functionality is in the active phase.

> 主类是[`plansys2::ProbemExpertNode`]，它是从[`ProbemExpert Node.cpp`]实例化的。`plansys2::ProblemExpert Node`是`rclcpp_lifecycle::LifecycleNode`，但当前该功能处于活动阶段。

The class responsible for maintaining this problem instance is [`plansys2::ProblemExpert`](include/plansys2_problem_expert/ProblemExpert.hpp), which is independent of ROS2.

> 负责维护此问题实例的类是[`plansys2::ProbemExpert`]，它独立于 ROS2。

The Problem Expert is dynamic and volatile, accessing its functionality through ROS2 services. To facilitate the task of the application developer, an [`plansys2::ProblemExpertClient`](include/plansys2_problem_expert/ProblemExpertClient.hpp) class has been implemented that hides the complexity of handling ROS2 messages and services. Its API is similar to that of [`plansys2::ProblemExpert`](include/plansys2_problem_expert/ProblemExpert.hpp), since both have to implement the [`plansys2::ProblemExpertInterface`](include/include/plansys2_problem_expert/ProblemExpertInterface.hpp) interface.

> **Problem Expert 是动态的和不稳定的，通过 ROS2 服务访问其功能。为了方便应用程序开发人员的任务**，已经实现了一个[`plansys2::ProbemExpertClient`]类，该类**隐藏了处理 ROS2 消息和服务的复杂性**。其 API 类似于[`plansys2::ProblemExpert`]的 API，因为两者都必须实现[`plansys2::problemExportInterface`]”接口。

The Problem Expert instantiates a [`plansys2::DomainExpertClient`](include/include/plansys2_domain_expert/DomainExpertClient.hpp), and every update query is verified against domain to check if it is valid.

> 问题专家实例化一个[`plansys2::DomainExpertClient`]，**每个更新查询都会根据域进行验证，以检查其是否有效**。

Every update in the Problem, is notified publishing a `std_msgs::msg::Empty` in `/problem_expert/update_notify`. It helps other modules and applications to be aware of updates, being not necessary to do polling to check it.

> Problem 中的每个更新都会被通知在`/Problem_expert/update_notify`中发布`std_msgs::msg::Empty`。它**有助于其他模块和应用程序了解更新，而无需进行轮询检查**。

## Services

- `/problem_expert/add_problem_function` [[`plansys2_msgs::srv::AffectNode`](../plansys2_msgs/srv/AffectNode.srv)]
- `/problem_expert/add_problem_goal` [[`plansys2_msgs::srv::AddProblemGoal`](../plansys2_msgs/srv/AddProblemGoal.srv)]
- `/problem_expert/add_problem_instance` [[`plansys2_msgs::srv::AffectParam`](../plansys2_msgs/srv/AffectParam.srv)]
- `/problem_expert/add_problem_predicate` [[`plansys2_msgs::srv::AffectNode`](../plansys2_msgs/srv/AffectNode.srv)]
- `/problem_expert/clear_problem_knowledge` [[`plansys2_msgs::srv::ClearProblemKnowledge`](../plansys2_msgs/srv/ClearProblemKnowledge.srv)]
- `/problem_expert/exist_problem_function` [[`plansys2_msgs::srv::ExistNode`](../plansys2_msgs/srv/ExistNode.srv)]
- `/problem_expert/exist_problem_predicate` [[`plansys2_msgs::srv::ExistNode`](../plansys2_msgs/srv/ExistNode.srv)]
- `/problem_expert/get_problem` [[`plansys2_msgs::srv::GetProblem`](../plansys2_msgs/srv/GetProblem.srv)]
- `/problem_expert/get_problem_function` [[`plansys2_msgs::srv::GetNodeDetails`](../plansys2_msgs/srv/GetNodeDetails.srv)]
- `/problem_expert/get_problem_functions` [[`plansys2_msgs::srv::GetStates`](../plansys2_msgs/srv/GetStates.srv)]
- `/problem_expert/get_problem_goal` [[`plansys2_msgs::srv::GetProblemGoal`](../plansys2_msgs/srv/GetProblemGoal.srv)]
- `/problem_expert/get_problem_instance` [[`plansys2_msgs::srv::GetProblemInstanceDetails`](../plansys2_msgs/srv/GetProblemInstanceDetails.srv)]
- `/problem_expert/get_problem_instances` [[`plansys2_msgs::srv::GetProblemInstances`](../plansys2_msgs/srv/GetProblemInstances.srv)]
- `/problem_expert/get_problem_predicate` [[`plansys2_msgs::srv::GetNodeDetails`](../plansys2_msgs/srv/GetNodeDetails.srv)]
- `/problem_expert/get_problem_predicates` [[`plansys2_msgs::srv::GetStates`](../plansys2_msgs/srv/GetStates.srv)]
- `/problem_expert/is_problem_goal_satisfied` [[`plansys2_msgs::srv::IsProblemGoalSatisfied`](../plansys2_msgs/srv/IsProblemGoalSatisfied.srv)]
- `/problem_expert/remove_problem_function` [[`plansys2_msgs::srv::AffectNode`](../plansys2_msgs/srv/AffectNode.srv)]
- `/problem_expert/remove_problem_goal` [[`plansys2_msgs::srv::RemoveProblemGoal`](../plansys2_msgs/srv/RemoveProblemGoal.srv)]
- `/problem_expert/remove_problem_instance` [[`plansys2_msgs::srv::AffectParam`](../plansys2_msgs/srv/AffectParam.srv)]
- `/problem_expert/remove_problem_predicate` [[`plansys2_msgs::srv::AffectNode`](../plansys2_msgs/srv/AffectNode.srv)]
- `/problem_expert/update_problem_function` [[`plansys2_msgs::srv::AffectNode`](../plansys2_msgs/srv/AffectNode.srv)]

## Published topics

- `/problem_expert/update_notify` [`std_msgs::msg::Empty`]
