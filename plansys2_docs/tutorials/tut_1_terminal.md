---
tip: translate by baidu@2023-06-05 08:23:25
...

# Working with Plansys2 and Terminal

Check out this [PDDL domain example](https://github.com/IntelligentRoboticsLabs/ros2_planning_system_examples/tree/master/plansys2_simple_example/pddl/simple_example.pddl). This is a small example of a tiny domain. It defines the types, predicates, and three actions for making a robot moving taking into account the battery level. It is a very basic example, but it is useful to illustrate this example.

> 查看此[PDL 域示例](https://github.com/IntelligentRoboticsLabs/ros2_planning_system_examples/tree/master/plansys2_simple_example/pddl/simple_example.pddl)。这是一个微小领域的小例子。它定义了使机器人在考虑电池电量的情况下移动的类型、谓词和三个动作。这是一个非常基本的例子，但说明这个例子是有用的。

Open a terminal and launch plansys2. We will use a launcher that includes the main planning system launcher, the specific action nodes for this example, and selects the domain:

> 打开一个终端并启动 plansys2。我们将使用一个启动器，该启动器包括主计划系统启动器、本例的特定操作节点，并选择域：

```shell
ros2 launch plansys2_simple_example plansys2_simple_example_launch.py
```

or

```shell
ros2 launch plansys2_simple_example plansys2_simple_example_launch.py namespace:=my_namespace
```

if you want to launch it in `my_namespace` namespace.

> 如果您想在“mynamespace”命名空间中启动它。

Open other terminal and launch the plansys2 terminal:

> 打开其他终端并启动 plansys2 终端：

```shell
ros2 run plansys2_terminal plansys2_terminal
```

or, if you used a namespace

```shell
ros2 run plansys2_terminal plansys2_terminal --ros-args -r __ns:=/my_namespace
```

The plansys2 terminal lets us operate directly against the planning system. It is a useful tool, useful to monitorize and developing your application. Usually, many of the next operations should be done inside your nodes. Plansys2 terminal is functional, but there is still too much to improve.

> plansys2 终端允许我们直接针对计划系统进行操作。它是一个有用的工具，对监控和开发应用程序非常有用。通常，接下来的许多操作都应该在节点内完成。Plansys2 终端是功能性的，但仍有太多需要改进。

Inside the plansys2 terminal, you can check the domain:

> 在 plansys2 终端内，您可以检查域：

```plansys2_terminal
> get domain
```

You also can check the current instances in the plan, that now it's void:

> 您还可以检查计划中的当前实例，现在它是无效的：

```plansys2_terminal
> get problem instances
```

To get the current predicates:

> 要获取当前谓词：

```plansys2_terminal
> get problem predicates
```

Now, the problem is void. Let's add some content and recheck it after.

> 现在，问题是空洞的。让我们添加一些内容，然后重新检查。

```plansys2_terminal
> set instance leia robot
> set instance entrance room
> set instance kitchen room
> set instance bedroom room
> set instance dinning room
> set instance bathroom room
> set instance chargingroom room
> set predicate (connected entrance dinning)
> set predicate (connected dinning entrance)
> set predicate (connected dinning kitchen)
> set predicate (connected kitchen dinning)
> set predicate (connected dinning bedroom)
> set predicate (connected bedroom dinning)
> set predicate (connected bathroom bedroom)
> set predicate (connected bedroom bathroom)
> set predicate (connected chargingroom kitchen)
> set predicate (connected kitchen chargingroom)
> set predicate (charging_point_at chargingroom)
> set predicate (battery_low leia)
> set predicate (robot_at leia entrance)
```

Once added content to the problem, verify the content of the problem again:

> 向问题添加内容后，请再次验证问题的内容：

```plansys2_terminal
> get problem instances
...
> get problem predicates
```

Lets planify. Add a goal to be achieved:

> 让我们计划一下。添加要实现的目标：

```plansys2_terminal
> set goal (and(robot_at leia bathroom))
```

And get the plan. This command will not execute the plan. Only will calculate it:

> 然后制定计划。此命令不会执行计划。只会计算它：

```plansys2_terminal
> get plan
plan:
0 (askcharge leia entrance chargingroom) 5
0.001 (charge leia chargingroom) 5
5.002 (move leia chargingroom kitchen) 5
10.003 (move leia kitchen dinning) 5
15.004 (move leia dinning bedroom) 5
20.005 (move leia bedroom bathroom) 5
```

To run the plan, type:

> 要运行计划，请键入：

```plansys2_terminal
run
```

You will see how the actions (calling to the nodes that implement the actions) are executed.

> 您将看到操作（调用实现操作的节点）是如何执行的。

press Ctrl-D to exit.

> 按 Ctrl-D 键退出。

The Plansys2 terminal has many functionalities: Adding/removing instances and predicates, asking for model details (predicates, actions, and types), run actions, getting plans, and running plans.

> Plansys2 终端具有许多功能：添加/删除实例和谓词，询问模型详细信息（谓词、操作和类型），运行操作，获取计划和运行计划。
