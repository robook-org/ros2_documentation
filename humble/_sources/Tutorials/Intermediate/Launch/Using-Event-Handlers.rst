.. redirect-from::

    Tutorials/Launch-Files/Using-Event-Handlers
    Tutorials/Launch/Using-Event-Handlers

使用事件处理器(event handlers)
===============================

**目标:** 学习 ROS 2 启动文件中事件处理器的使用。

**教程等级:** 中级

**预计时长:** 15 分钟

.. contents:: Table of Contents
   :depth: 2
   :local:

背景
----------

启动系统(Launch)在 ROS 2 中是一个执行和管理用户定义进程的系统。
它负责监视它启动的进程的状态，并且报告和响应这些进程状态的变化。
我们把这些状态的变化称为事件(event)，可以通过在启动系统中注册事件处理器(event handler)来处理这些事件。
事件处理器中可以注册特定事件的处理程序，用于监视进程的状态。
此外，它们可以用于定义一组复杂的规则，这些规则可以用于动态修改启动文件。

本教程展示了在 ROS 2 启动文件中使用事件处理器的用法。

前提条件
-------------

本教程用到了 :doc:`turtlesim <../../Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim>` 包。
本教程还假设你已经 :doc:`创建了一个新的包 <../../Beginner-Client-Libraries/Creating-Your-First-ROS2-Package>`，构建类型为 ``ament_python``，名为 ``launch_tutorial``。

本教程扩展了 :doc:`在启动文件中使用替换 <./Using-Substitutions>` 教程中展示的代码。

使用事件处理器
--------------------

1 带有事件处理器的示例启动文件
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

创建一个名为 ``example_event_handlers.launch.py`` 的新文件，放在 ``launch_tutorial`` 包的 ``launch`` 文件夹中。

.. code-block:: python

    from launch_ros.actions import Node

    from launch import LaunchDescription
    from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                                LogInfo, RegisterEventHandler, TimerAction)
    from launch.conditions import IfCondition
    from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                    OnProcessIO, OnProcessStart, OnShutdown)
    from launch.events import Shutdown
    from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                    LaunchConfiguration, LocalSubstitution,
                                    PythonExpression)


    def generate_launch_description():
        turtlesim_ns = LaunchConfiguration('turtlesim_ns')
        use_provided_red = LaunchConfiguration('use_provided_red')
        new_background_r = LaunchConfiguration('new_background_r')

        turtlesim_ns_launch_arg = DeclareLaunchArgument(
            'turtlesim_ns',
            default_value='turtlesim1'
        )
        use_provided_red_launch_arg = DeclareLaunchArgument(
            'use_provided_red',
            default_value='False'
        )
        new_background_r_launch_arg = DeclareLaunchArgument(
            'new_background_r',
            default_value='200'
        )

        turtlesim_node = Node(
            package='turtlesim',
            namespace=turtlesim_ns,
            executable='turtlesim_node',
            name='sim'
        )
        spawn_turtle = ExecuteProcess(
            cmd=[[
                FindExecutable(name='ros2'),
                ' service call ',
                turtlesim_ns,
                '/spawn ',
                'turtlesim/srv/Spawn ',
                '"{x: 2, y: 2, theta: 0.2}"'
            ]],
            shell=True
        )
        change_background_r = ExecuteProcess(
            cmd=[[
                FindExecutable(name='ros2'),
                ' param set ',
                turtlesim_ns,
                '/sim background_r ',
                '120'
            ]],
            shell=True
        )
        change_background_r_conditioned = ExecuteProcess(
            condition=IfCondition(
                PythonExpression([
                    new_background_r,
                    ' == 200',
                    ' and ',
                    use_provided_red
                ])
            ),
            cmd=[[
                FindExecutable(name='ros2'),
                ' param set ',
                turtlesim_ns,
                '/sim background_r ',
                new_background_r
            ]],
            shell=True
        )

        return LaunchDescription([
            turtlesim_ns_launch_arg,
            use_provided_red_launch_arg,
            new_background_r_launch_arg,
            turtlesim_node,
            RegisterEventHandler(
                OnProcessStart(
                    target_action=turtlesim_node,
                    on_start=[
                        LogInfo(msg='Turtlesim started, spawning turtle'),
                        spawn_turtle
                    ]
                )
            ),
            RegisterEventHandler(
                OnProcessIO(
                    target_action=spawn_turtle,
                    on_stdout=lambda event: LogInfo(
                        msg='Spawn request says "{}"'.format(
                            event.text.decode().strip())
                    )
                )
            ),
            RegisterEventHandler(
                OnExecutionComplete(
                    target_action=spawn_turtle,
                    on_completion=[
                        LogInfo(msg='Spawn finished'),
                        change_background_r,
                        TimerAction(
                            period=2.0,
                            actions=[change_background_r_conditioned],
                        )
                    ]
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=turtlesim_node,
                    on_exit=[
                        LogInfo(msg=(EnvironmentVariable(name='USER'),
                                ' closed the turtlesim window')),
                        EmitEvent(event=Shutdown(
                            reason='Window closed'))
                    ]
                )
            ),
            RegisterEventHandler(
                OnShutdown(
                    on_shutdown=[LogInfo(
                        msg=['Launch was asked to shutdown: ',
                            LocalSubstitution('event.reason')]
                    )]
                )
            ),
        ])

``OnProcessStart``, ``OnProcessIO``, ``OnExecutionComplete``, ``OnProcessExit`` 和 ``OnShutdown`` 事件的 ``RegisterEventHandler`` 操作在启动描述中定义好了。

``OnProcessStart`` 用于注册一个在turtlesim 节点启动时执行的回调函数。
它在 turtlesim 节点启动时记录一条消息到控制台，并在turtlesim节点启动时运行 ``spawn_turtle`` action。

.. code-block:: python

    RegisterEventHandler(
        OnProcessStart(
            target_action=turtlesim_node,
            on_start=[
                LogInfo(msg='Turtlesim started, spawning turtle'),
                spawn_turtle
            ]
        )
    ),

``OnProcessIO`` 用于注册一个在 ``spawn_turtle`` action 向其标准输出写数据时执行的回调函数。
它记录了生成请求的结果。

.. code-block:: python

    RegisterEventHandler(
        OnProcessIO(
            target_action=spawn_turtle,
            on_stdout=lambda event: LogInfo(
                msg='Spawn request says "{}"'.format(
                    event.text.decode().strip())
            )
        )
    ),

``OnExecutionComplete`` 用于注册一个在 ``spawn_turtle`` action 完成时的回调函数。
它记录了一条消息到控制台，并在 spawn action 完成时执行 ``change_background_r`` 和 ``change_background_r_conditioned`` actions。

.. code-block:: python

    RegisterEventHandler(
        OnExecutionComplete(
            target_action=spawn_turtle,
            on_completion=[
                LogInfo(msg='Spawn finished'),
                change_background_r,
                TimerAction(
                    period=2.0,
                    actions=[change_background_r_conditioned],
                )
            ]
        )
    ),

``OnProcessExit`` 用于注册一个在 turtlesim 节点退出时执行的回调函数。
它记录了一条消息到控制台，并在 turtlesim 节点退出时执行 ``EmitEvent`` action 来发出一个 ``Shutdown`` 事件。
这意味着当 turtlesim 窗口关闭时， launch 进程将关闭。

.. code-block:: python

    RegisterEventHandler(
        OnProcessExit(
            target_action=turtlesim_node,
            on_exit=[
                LogInfo(msg=(EnvironmentVariable(name='USER'),
                        ' closed the turtlesim window')),
                EmitEvent(event=Shutdown(
                    reason='Window closed'))
            ]
        )
    ),

最后，``OnShutdown`` 事件处理器用于注册一个回调函数，在启动文件关闭时执行。
它记录了一条消息到控制台，说明关闭启动文件的原因。
比如，当 turtlesim 窗口关闭或用户通过 :kbd:`ctrl-c` 信号关闭时，它记录了带有关闭原因的消息。

.. code-block:: python

    RegisterEventHandler(
        OnShutdown(
            on_shutdown=[LogInfo(
                msg=['Launch was asked to shutdown: ',
                    LocalSubstitution('event.reason')]
            )]
        )
    ),

构建包
-----------------

进入工作空间的根目录，构建包:

.. code-block:: console

  colcon build

记得在构建后 source 代码。

运行示例
-----------------

现在可以使用 ``ros2 launch`` 命令启动 ``example_event_handlers.launch.py`` 。

.. code-block:: console

    ros2 launch launch_tutorial example_event_handlers.launch.py turtlesim_ns:='turtlesim3' use_provided_red:='True' new_background_r:=200

这将执行以下操作:

#. 使用蓝色背景启动 turtlesim 节点
#. 生成第二只乌龟
#. 将背景颜色更改为紫色
#. 如果提供的 ``background_r`` 参数为 ``200`` 并且 ``use_provided_red`` 参数为 ``True``，则在两秒后将颜色更改为粉色
#. 当 turtlesim 窗口关闭时关闭启动文件

此外，在以下情况出现时会在控制台打印 log :

#. turtlesim 节点启动时
#. spawn action 运行时
#. ``change_background_r`` action 运行时
#. ``change_background_r_conditioned`` action 运行时
#. turtlesim 节点退出时
#. 启动文件被关闭时

文档
-------------

`The launch documentation <https://github.com/ros2/launch/blob/{REPOS_FILE_BRANCH}/launch/doc/source/architecture.rst>`_ 提供了关于事件处理器的详细信息。

总结
-------

在本教程中，你学习了如何在启动文件中使用事件处理器。
你学习了它们的语法和用法示例，用于定义一组复杂的规则，这些规则可以用于动态修改启动文件。
