---
tip: translate by baidu@2023-06-05 11:21:09
...

# Planner

The Planner module is responsible for creating plans. By default, it uses [popf](https://github.com/fmrico/popf), that is a PDDL solver.

> 计划器模块负责创建计划。默认情况下，它使用[popf](https://github.com/fmrico/popf)，即 PDDL 解算器。

This module is very simple, as its only task is calling the popf binary and parsing the result.

> 这个模块非常简单，因为它唯一的任务就是调用 popf 二进制文件并解析结果。

The main class is [`plansys2::PlannerNode`](include/include/plansys2_planner/PlannerNode.hpp), which is instantiated from [`planner_node.cpp`](src/planner_node.cpp). `plansys2::PlannerNode` is a also `rclcpp_lifecycle::LifecycleNode`, but currently the functionality is in the active phase.

> 主类是[`plansys2:：PlannerNode`]（include/include/plansys2_planner/PlannerNode.hpp），它是从[`planner_node.cpp`]（src/planner_node.cpp）实例化的。`plansys2::PlannerNode`也是`rclcpp_lifecycle::LifecycleNode`，但当前该功能处于活动阶段。

Plan solvers are specified in the `plan_solver_plugins` parameter. In case of more than one specified, the first one will be used. If this parameter is not specified, POPF will be used by default.

> 计划求解器在“Plan_solver_plugins”参数中指定。如果指定了多个，则将使用第一个。如果未指定此参数，则默认情况下将使用 POPF。

## Services

- `/planner/get_plan` [[`plansys2_msgs::srv::GetPlan`](../plansys2_msgs/srv/GetPlan.srv)]
