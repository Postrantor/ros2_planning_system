---
tip: translate by baidu@2023-06-05 08:23:36
...

# Patrolling example

[This package](https://github.com/IntelligentRoboticsLabs/ros2_planning_system_examples/tree/master/plansys2_patrol_navigation_example) contains a more complex example that uses ROS2 Navigation to make a robot patrol.

> [此包](https://github.com/IntelligentRoboticsLabs/ros2_planning_system_examples/tree/master/plansys2_patrol_navigation_example)包含一个使用 ROS2 导航进行机器人巡逻的更复杂的示例。

In one terminal run:

> 在一次终端运行中：

```shell
ros2 launch patrol_navigation_example patrol_example_launch.py
```

This command launchs plansys2 and the nodes that implements the actions (move and patrol). The navigation param `autostart` is set to `true`, and the amcl is set to the starting position of the robot. This position is `wp_control`. We will use the next waypoints (x, y, yaw):

> 此命令启动 plansys2 和执行操作（移动和巡逻）的节点。导航参数“autostart”设置为“true”，amcl 设置为机器人的起始位置。此位置为“wp_control”。我们将使用下一个航点（x、y、偏航）：

- wp_control (-2.0, -4.0, 0.0) - This is the starting point
- wp1 (0.0, -2.0, 0.0)
- wp2 (1.8, 0.0, 0.0)
- wp3 (0.0, 2.0, 0.0)
- wp4 (-0.5, 0.5, 0.0)

The setup from symbol to metric coordinates are made in the contructior of [MoveAction](https://github.com/IntelligentRoboticsLabs/ros2_planning_system_examples/blob/master/plansys2_patrol_navigation_example/src/move_action_node.cpp). They are stored in a std::map to get the metric coordinate of the commanded waypoint when the action is activated. The waypoint is an argument of the action.

> 从符号到公制坐标的设置是在[MoveAction]的结构中进行的(https://github.com/IntelligentRoboticsLabs/ros2_planning_system_examples/blob/master/plansys2_patrol_navigation_example/src/move_action_node.cpp)。它们存储在标准：：地图中，以在激活动作时获得命令航路点的公制坐标。航路点是行动的论据。

The [MoveAction](https://github.com/IntelligentRoboticsLabs/ros2_planning_system_examples/blob/master/plansys2_patrol_navigation_example/src/move_action_node.cpp) action calls to the navigation 2 stack to make the robot move.

> [MoveAction](https://github.com/IntelligentRoboticsLabs/ros2_planning_system_examples/blob/master/plansys2_patrol_navigation_example/src/move_action_node.cpp)动作调用导航 2 堆栈以使机器人移动。

The [Patrol](https://github.com/IntelligentRoboticsLabs/ros2_planning_system_examples/blob/master/plansys2_patrol_navigation_example/src/patrol_action_node.cpp) action only makes the robot turn few seconds, by sending `geometry_msgs::msg::Twist` to `/cmd_vel`.

> [巡逻队](https://github.com/IntelligentRoboticsLabs/ros2_planning_system_examples/blob/master/plansys2_patrol_navigation_example/src/patrol_action_node.cpp)通过向“/cmd_vel'”发送“geometry_msgs:：msg:：Twist”，该动作只会使机器人转动几秒钟。

Next, we will run the node of the application, [patrolling controller node](https://github.com/IntelligentRoboticsLabs/ros2_planning_system_examples/blob/master/plansys2_patrol_navigation_example/src/patrolling_controller_node.cpp). This controls the phase of the behavior of the robot. It is implemented with a Finite State Machine (FSM). In each state, it sets a goal (`(and(patrolled wp1))`, for example), and calls to executor to generate a plan and execute it. The `init_knowledge()` method sets the connections among waypoints (all the navigations from a waypoint to another has to visit `wp_control`):

> 接下来，我们将运行应用程序的节点[巡逻控制器节点](https://github.com/IntelligentRoboticsLabs/ros2_planning_system_examples/blob/master/plansys2_patrol_navigation_example/src/patrolling_controller_node.cpp)。这控制了机器人行为的阶段。它是用有限状态机（FSM）实现的。在每种状态下，它都会设置一个目标（例如“（和（巡逻的 wp1））”），并调用 executor 生成一个计划并执行它。“init_knowledge（）”方法设置航路点之间的连接（从一个航路点到另一个航点的所有导航都必须访问“wp_control”）：

```c++
    problem_expert_->addInstance(plansys2::Instance{"r2d2", "robot"});
    problem_expert_->addInstance(plansys2::Instance{"wp_control", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp1", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp2", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp3", "waypoint"});
    problem_expert_->addInstance(plansys2::Instance{"wp4", "waypoint"});

    problem_expert_->addPredicate(plansys2::Predicate("(robot_at r2d2 wp_control)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_control wp1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp1 wp_control)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_control wp2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp2 wp_control)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_control wp3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp3 wp_control)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp_control wp4)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected wp4 wp_control)"));
```

To run this node, only type in another terminal:

> 若要运行此节点，请仅键入另一个终端：

```shell
ros2 run patrol_navigation_example patrolling_controller_node
```

When the robot visit wp_4, it starts again the patrolling.

> 当机器人访问 wp_4 时，它再次开始巡逻。

You have to see something like this:

> 你必须看到这样的东西：

[![Patrolling example](https://img.youtube.com/vi/fAEGySqefwo/0.jpg)](https://www.youtube.com/watch?v=fAEGySqefwo)

> [！[巡逻示例](https://img.youtube.com/vi/fAEGySqefwo/0.jpg)](https://www.youtube.com/watch?v=fAEGySqefwo)
