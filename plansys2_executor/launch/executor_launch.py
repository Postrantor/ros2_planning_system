# Copyright 2019 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# /**
#  * @brief 生成启动描述
#  * @param namespace 命名空间
#  * @param params_file 参数文件路径
#  * @param action_bt_file 行为树文件路径
#  * @param start_action_bt_file 开始行为树文件路径
#  * @param end_action_bt_file 结束行为树文件路径
#  * @param bt_builder_plugin 行为树构建插件
#  * @details 该函数用于生成启动描述，主要包括声明命名空间、行为树文件、行为树构建插件等参数，并通过Node节点来执行plansys2_executor中的executor_node。
#  */
def generate_launch_description():
    # 声明各个参数
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')
    action_bt_file = LaunchConfiguration('action_bt_file')
    start_action_bt_file = LaunchConfiguration('start_action_bt_file')
    end_action_bt_file = LaunchConfiguration('end_action_bt_file')
    bt_builder_plugin = LaunchConfiguration('bt_builder_plugin')

    # 声明命名空间参数
    declare_namespace_cmd = DeclareLaunchArgument('namespace',
                                                  default_value='',
                                                  description='Namespace')

    # 声明行为树文件参数
    declare_action_bt_file_cmd = DeclareLaunchArgument(
        'action_bt_file',
        default_value=os.path.join(
            get_package_share_directory('plansys2_executor'), 'behavior_trees',
            'plansys2_action_bt.xml'),
        description='BT representing a PDDL action')

    # 声明开始行为树文件参数
    declare_start_action_bt_file_cmd = DeclareLaunchArgument(
        'start_action_bt_file',
        default_value=os.path.join(
            get_package_share_directory('plansys2_executor'), 'behavior_trees',
            'plansys2_start_action_bt.xml'),
        description='BT representing a PDDL start action')

    # 声明结束行为树文件参数
    declare_end_action_bt_file_cmd = DeclareLaunchArgument(
        'end_action_bt_file',
        default_value=os.path.join(
            get_package_share_directory('plansys2_executor'), 'behavior_trees',
            'plansys2_end_action_bt.xml'),
        description='BT representing a PDDL end action')

    # 声明行为树构建插件参数
    declare_bt_builder_plugin_cmd = DeclareLaunchArgument(
        'bt_builder_plugin',
        default_value='SimpleBTBuilder',
        description='Behavior tree builder plugin.',
    )

    # 执行plansys2_executor中的executor_node节点
    executor_cmd = Node(package='plansys2_executor',
                        executable='executor_node',
                        name='executor',
                        namespace=namespace,
                        output='screen',
                        parameters=[{
                            'default_action_bt_xml_filename':
                            action_bt_file
                        }, {
                            'default_start_action_bt_xml_filename':
                            start_action_bt_file
                        }, {
                            'default_end_action_bt_xml_filename':
                            end_action_bt_file
                        }, {
                            'bt_builder_plugin': bt_builder_plugin
                        }, params_file])

    # 创建启动描述并添加各个参数和节点
    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_action_bt_file_cmd)
    ld.add_action(declare_start_action_bt_file_cmd)
    ld.add_action(declare_end_action_bt_file_cmd)
    ld.add_action(declare_bt_builder_plugin_cmd)

    ld.add_action(executor_cmd)

    return ld
