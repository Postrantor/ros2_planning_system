---
tip: translate by openai@2023-06-03 00:51:39
...

# Executor

The Executor module is responsible for requesting a plan to the Planner, and carry it out, calling to the nodes in the client application that implements the actions. While executing each action, it checks the requisites (_At Start, At End and Over all_, in case of durative Actions). If the requirements are not meet, it cancels the plan execution. It also is responsible for applying the effects of the actions, requesting updates to the Problem Expert.

> **执行模块**负责**向计划者请求计划，并执行它**，**调用客户端**应用程序中实现的操作。在执行每个操作时，它会**检查必要条件**(开始时，结束时和持续操作的整体)。如果不满足要求，它将取消计划执行。它还负责应用操作的效果，**要求问题专家更新**。

The main class of Executor is [`plansys2::ExecutorNode`](include/include/plansys2_executor/ExcutorNode.hpp), which is instantiated from [`executor_node.cpp`](src/executor_node.cpp).

> 主要的执行器类是`plansys2::ExecutorNode`(参见 include/include/plansys2_executor/ExcutorNode.hpp)，它是从`executor_node.cpp`(参见 src/executor_node.cpp)实例化的。

The executions of plans are carried out using ROS2 actions, in particular, [`plansys2_msgs::action::ExecutePlan`](../plansys2_msgs/action/ExecutePlan.action). Take note that the goal must be already in the Domain Expert.

> 规划的执行是使用 ROS2 动作完成的，特别是`plansys2_msgs::action::ExecutePlan`。注意，目标必须已经在**域专家**中。

ExecutorNode ask for the **domain and problem**, and ask for a plan to the Planner. For each action in the plan, ExecuterNode creates a [`plansys2::ActionExecutor`](include/include/plansys2_executor/ActionExecutor.hpp). The lifetime of this object is only one action. This object calls the actions implemented in the client appliciation using the ROS2 actions [`plansys2_msgs::action::ExecuteAction`](../plansys2_msgs/action/ExecuteAction.action). Each client action implementation can use the class [`plansys2::ActionExecutorClient`](include/include/plansys2_executor/ActionExecutorClient.hpp) to avoid the complexity of managing ROS2 actions.

> ExecutorNode **要求域和问题**，并向 Planner 要求一个计划。对于计划中的每个动作，ExecuterNode 都创建一个`plansys2::ActionExecutor`。这个对象的生命周期只有一个动作。这个对象使用 ROS2 动作`plansys2_msgs::action::ExecuteAction`调用客户端应用程序中实现的动作。每个客户端动作实现都可以使用类`plansys2::ActionExecutorClient`来避免管理 ROS2 动作的复杂性。

Using the feedback information from `plansys2_msgs::action::ExecuteAction`, feedback for `plansys2::ExecutorNode` is composed and returned to `plansys2::ExecutorClient`. It contains the current action in the plan and the progress in the currently executing action.

> 使用来自`plansys2_msgs::action::ExecuteAction`的反馈信息，给`plansys2::ExecutorNode`组成并返回给`plansys2::ExecutorClient`。它包含了计划中的当前动作和当前正在执行动作的进度。

Next graph shows an example of the execution flow:

> 下一张图显示了一个执行流程的示例：

![Executor Flow](../plansys2_docs/Executor_graph.png)

## Actions:

- `/execute_plan` [[`plansys2_msgs::action::ExecutePlan`](../plansys2_msgs/action/ExecutePlan.action)]

(in ActionExecutorClient)

- `/${ACTION_NAME`}[[`plansys2_msgs::action::ExecuteAction`](../plansys2_msgs/action/ExecuteAction.action)]

## Services:

- `/executor/get_ordered_sub_goals` [[`plansys2_msgs::srv::GetOrderedSubGoals`](../plansys2_msgs/srv/GetOrderedSubGoals.srv)]

## Subscribed topics:

(in ActionExecutor)

- `/problem_expert/update_notify` [`std_msgs::msg::Empty`]

## Parameters:

(in ExecutorNode)

- `~/action_timeouts/actions` [`list of strings`]

  - List of actions which have duration overrun percentages specified.

- `~/action_timeouts/[ACTION_NAME]/duration_overrun_percentage` [`double`]

  - Defines the allowable time overrun of an action based on a percentage of the predicted plan duration.

    > 定义根据预测计划持续时间的百分比允许的行动超时。

    For example, if the plan predicts that an action should take 1000 secs and a duration overrun percentage of 20% is specified, then the action should be halted if the actual duration exceeds 1200 secs.

    > 例如，如果该计划预测一项措施应花费 1000 秒，并且指定了 20％的持续时间超额百分比，则如果实际持续时间超过 1200 秒，则应停止操作。

    ```yaml
    executor:
      ros__parameters:
        action_timeouts:
          actions: ["move"]
          move:
            duration_overrun_percentage: 20.0
    ```
