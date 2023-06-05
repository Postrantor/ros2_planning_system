---
tip: translate by openai@2023-06-03 00:49:41
...

# BT Actions

The purpose of this package is to provide built-in support for `plansys2` actions which use [Behavior Trees](https://github.com/BehaviorTree/BehaviorTree.CPP) (BTs) for their implementation. A drop-in replacement for a vanilla `plansys2::ActionExecutorClient` is provided in the form of a ROS2 node, which will execute an arbitrary BT passed in as a ROS parameter. A BehaviorTree.CPP node, `plansys2::BtActionNode`, is also included to provide a convenient wrapper around a ROS2 action client (a common application for BT nodes).

> 本软件包的目的是为使用[行为树] (BTs)实现的`plansys2`动作提供内置支持。以 ROS2 节点的形式提供了一个可替换的香草`plansys2::ActionExecutorClient`，它将执行作为 ROS 参数传入的任意 BT。还包括一个 BehaviorTree.CPP 节点`plansys2::BtActionNode`，为 ROS2 动作客户端(BT 节点的常见应用)提供了方便的封装。

### ROS2 node for implementing `plansys2` actions

The `bt_action_node` ROS node takes in several parameters to set up its execution.

> `bt_action_node` ROS 节点接受多个参数来设置其执行。

These parameters are

1. `action_name`: The name of the `plansys2` action to implement. Note that this name must match what is in your pddl domain file.
2. `bt_xml_file`: An absolute path to the BT `.xml` file to execute.
3. `plugins`: a list of BehaviorTree.CPP shared libraries to load. Any BT node which is in the `.xml` but is not provided by the BehaviorTree.CPP library itself must be in one of the libraries specified
4. `enable_groot_monitoring`: a boolean which specifies if ZMQ publisher should be created, for use with [Groot](https://github.com/BehaviorTree/Groot) (default is `false`)
5. `publisher_port`: the ZMQ publisher port to use (if `enable_groot_monitoring` is enabled)
6. `server_port`: the ZMQ server port to use (if `enable_groot_monitoring` is enabled)
7. `max_msgs_per_second`: max ZMQ messages per second (if `enable_groot_monitoring` is enabled)
8. `bt_file_logging`: a boolean which [enables logging of BT state changes in `.fbl` files](https://www.behaviortree.dev/tutorial_05_subtrees/), useful for playing back behavior tree execution using `Groot` (default is `false`)
9. `bt_minitrace_logging`: a boolean which enables logging of `.json` files for recording the execution time of each node (default is `false`)

> 1. `action_name`：要实施的`plansys2`动作的名称。 请注意，此名称必须与 pddl 域文件中的内容相匹配。
> 1. `bt_xml_file`：执行 BT `.xml`文件的绝对路径。
> 1. `plugins`：一个加载共享库的行为树.CPP 列表。任何在`.xml`中但不是由行为树.CPP 库本身提供的 BT 节点都必须在指定的库中之一。
> 1. `enable_groot_monitoring`：一个布尔值，用于指定是否应该创建 ZMQ 发布者，用于[Groot](https://github.com/BehaviorTree/Groot)(默认为`false`)
> 1. `publisher_port`：如果启用`enable_groot_monitoring`，要使用的 ZMQ 发布者端口
> 1. `server_port`：使用的 ZMQ 服务器端口(如果已启用`enable_groot_monitoring`)
> 1. `max_msgs_per_second`: 最大每秒 ZMQ 消息数(如果启用了`enable_groot_monitoring`)
> 1. `bt_file_logging`：一个布尔值，用于启用在`.fbl`文件中记录 BT 状态更改的功能，可以**使用`Groot`回放行为树执行**(默认为`false`)
> 1. `bt_minitrace_logging`：一个布尔值，用于启用**记录每个节点执行时间**的`.json`文件的日志(默认为`false`)

Files created by the `.fbl` and minitrace loggers are stored in `/tmp/<node_name>/`, with names containing a timestamp.

> 文件由`.fbl`和 minitrace 记录器创建，存储在`/tmp/<node_name>/`中，文件名包含时间戳。

### BT node for calling ROS2 action servers

The `BtActionNode` template class provides a convenient means of calling ROS2 action servers from within a BT. It takes care of the details of setting up and handling a ROS action client, reducing code duplication and providing a simple API.

> `BtActionNode`模板类提供了一种从 BT 中**方便地调用 ROS2 动作服务器**的方法。它负责设置和处理 ROS 动作客户端的细节，**减少代码重复**，提供简单的 API。

The template parameter for the class is the type of ROS action (e.g. `action_tutorials_interfaces::action::Fibonacci`) to be used. The node's constructor takes in three arguments: the XML tag name, the ROS topic for the action server (e.g. `/namepace/server_name`), and a `BT::NodeConfiguration`. Note that the XML name and `NodeConfiguration` are the same as any other BT.CPP node.

> 模板参数为类型的 ROS 动作(例如`action_tutorials_interfaces::action::Fibonacci`)。节点的构造函数需要三个参数：
>
> - XML 标记名称、
> - ROS 动作服务器的主题(例如`/namepace/server_name`)和
> - `BT::NodeConfiguration`。
>
> 请注意，XML 名称和`NodeConfiguration`与任何其他 BT.CPP 节点相同。

There are several functions which are provided for the end user to use/implement (some of which are optional).

> 有几个功能提供给最终用户使用/实现(其中一些是可选的)。

1. `static BT::PortsList providedPorts()`: every BT node which uses ports must define this member function. A default implementation is provided, but you are free to override it if additional ports are desired. By default, the function returns two input ports: `server_name` (string) and `server_timeout` (double). These ports can be preserved when overriding using `providedBasicPorts`

> `static BT::PortsList providedPorts()`：每个使用端口的 BT 节点都必须定义这个成员函数。提供了默认实现，但如果需要添加其他端口，您可以自由覆盖它。默认情况下，该函数会返回两个输入端口：`server_name`(字符串)和`server_timeout`(双精度)。使用`providedBasicPorts`覆盖时，可以保留这些端口。

- `server_name`: an (optional) means of overriding the action server topic provided in the constructor
- `server_timeout`: how long to wait for an action server before failing, in units of seconds (default is 5s)

> - `服务器名称`：可选的覆盖构造函数中提供的动作服务器主题的方法
> - `server_timeout`: 在失败之前等待操作服务器的时间(以秒为单位，默认为 5 秒)

2. `BT::PortsList providedBasicPorts(BT::PortsList addition)`: a convenience function for preserving the default input ports when overriding the `providedPorts()` function. An example use is shown below

> 2. `BT::PortsList providedBasicPorts(BT::PortsList addition)`：一个方便的函数，用于在覆盖`providedPorts()`函数时保留默认输入端口。以下是一个示例用法：`Wait`

    ```cpp
    static BT::PortsList providedPorts() override
    {
      return providedBasicPorts({ BT::InputPort<std::string>("my_additional_port") });
    }
    ```

3. `virtual BT::NodeStatus on_tick()`: This function is called every time the node is ticked (in the BT sense). The user is expected to set the `goal_` variable somewhere in this function, which is of the same type as the template parameter. Note that the node will likely be ticked multiple times before the action completes, so you may want to only set the goal on the first tick (i.e. when the node is IDLE). If you want to preempt the current goal and send a new one after the first tick, set the `goal_updated_` member variable to true, along with any changes to the `goal_` variable. An example is shown below.

> 这个函数每次节点被 tick(在 BT 中)时调用。用户预期在这个函数中设置`goal_`变量，它与模板参数的类型相同。请注意，在动作完成之前，节点可能会被 tick 多次，因此您可能只想在第一次 tick(即节点 IDLE 时)设置目标。如果您想在第一次 tick 之后抢先当前目标并发送新的目标，请将`goal_updated_`成员变量设置为 true，以及对`goal_`变量的任何更改。下面是一个示例。

    ```cpp
    BT::NodeStatus on_tick() override
    {
      if (status() == BT::NodeStatus::IDLE) { // this block will only execute on the first tick
        goal_.string = "foo";
      } else if (<some condition>) {  // this block may execute anytime afterward
        goal_.string = "bar";
        goal_updated_ = true;
      }
    }
    ```

4. `virtual void on_feedback(const std::shared_ptr<ActionT::Feedback> feedback)`: this function will be called whenever feedback is received from the action server **asynchronous of BT execution** (and is optional)

> `virtual void on_feedback(const std::shared_ptr<ActionT::Feedback> feedback)`：此函数将在接收到从动作服务器发送的反馈(异步 BT 执行)时调用(可选)。

5. `virtual BT::NodeStatus on_success()`: this function is called if the action server returned successfully and will return `BT::NodeStatus::SUCCESS` by default. You may wish to override it if the node's success depends on the result, which is placed in the `result_` member variable.

> `virtual BT::NodeStatus on_success()`：如果动作服务器成功返回，将调用此函数，默认情况下将返回 `BT::NodeStatus::SUCCESS`。如果节点的成功取决于结果(存放在`result*`成员变量中)，您可能希望覆盖它。

6. `virtual BT::NodeStatus on_aborted()`: this function is called if the action server aborted the action, returning `BT::NodeStatus::FAILURE` by default.

> `virtual BT::NodeStatus on_aborted()`：如果动作服务器中止了动作，那么将默认调用此函数，返回`BT::NodeStatus::FAILURE`。

7. `virtual BT::NodeStatus on_cancelled()`: this function is called if the action was cancelled, returning `BT::NodeStatus::SUCCESS` by default.

> `virtual BT::NodeStatus on_cancelled()`：如果操作被取消，则会调用此函数，默认情况下会返回`BT::NodeStatus::SUCCESS`。
