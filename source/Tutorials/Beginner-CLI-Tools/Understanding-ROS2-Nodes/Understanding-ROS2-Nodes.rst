.. redirect-from::

    Tutorials/Understanding-ROS2-Nodes

.. _ROS2Nodes:

理解节点
===================

**目标:** 学习 ROS 2 中节点的功能，以及与之交互的工具。

**教程等级:** 初级

**预计时长:** 10 分钟

.. contents:: Contents
   :depth: 2
   :local:

背景
----------

1 The ROS 2 graph
^^^^^^^^^^^^^^^^^

接下来的几节教程中，你将学习到一系列 ROS 2 核心概念，这些概念构成了的 "ROS (2) graph"。
ROS graph 是一个同时处理数据的 ROS 2 组件构成的网络。
它包含了所有可执行文件以及它们之间的连接，如果你把它们全部映射出来并可视化，那么它就是一个 ROS 2 图。

2 ROS 2 中的节点
^^^^^^^^^^^^^^^^^^^^

ROS 中的每个节点应该只为一个单一、独立和模块化的目的负责，例如控制轮子的电机或者发布激光测距仪的传感器数据。
每个节点都可以通过 topic 、服务、action 或 parameters 与其他节点进行数据交换。

.. image:: images/Nodes-TopicandService.gif

一个完整的机器人系统由许多协同工作的节点组成。
在 ROS 2 中，一个可执行文件（C++ 程序、Python 程序等）可以包含一个或多个节点。

前提条件
-------------

:doc:`之前的教程中 <../Introducing-Turtlesim/Introducing-Turtlesim>` 中描述了将要使用的 ``turtlesim`` 包的安装方法。

如往常一样，不要忘记在 :doc:`每次打开新终端 <../Configuring-ROS2-Environment>` 时都要 source ROS 2。

任务
-----

1 ros2 run
^^^^^^^^^^

``ros2 run`` 命令从一个包中启动一个可执行文件。

.. code-block:: console

    ros2 run <package_name> <executable_name>

为了运行 turtlesim，打开一个新终端，输入以下命令：

.. code-block:: console

    ros2 run turtlesim turtlesim_node

turtlesim 窗口会打开，就像你在 :doc:`之前的教程 <../Introducing-Turtlesim/Introducing-Turtlesim>` 中看到的那样。

在上面指令中，包名是 ``turtlesim``，可执行文件名是 ``turtlesim_node``。

不过我们还是不知道节点的名字
你可以使用 ``ros2 node list`` 来查看节点的名字

2 ros2 node list
^^^^^^^^^^^^^^^^

``ros2 node list`` 命令会显示所有正在运行的节点的名字。
在你想和一个节点交互时，或者需要追踪运行着许多节点的系统时，这个指令特别有用。

在 turtlesim 仍在运行的终端中打开一个新终端，输入以下命令：

.. code-block:: console

    ros2 node list

终端会显示节点的名字：

.. code-block:: console

  /turtlesim

打开一个新终端，用如下指令运行小乌龟的控制节点

.. code-block:: console

    ros2 run turtlesim turtle_teleop_key

这次我们又从 ``turtlesim`` 包里面启动代码，只不过启动的可执行文件换成了 ``turtle_teleop_key``.

现在回到刚刚运行 ``ros2 node list`` 的终端再运行一次。
你能看到有两个节点名字显示出来：

.. code-block:: console

  /turtlesim
  /teleop_turtle

2.1 重映射(Remapping)
~~~~~~~~~~~~~~~~~~~~~~~~~

`重映射 <https://design.ros2.org/articles/ros_command_line_arguments.html#name-remapping-rules>`__ 使你能够重新分配默认节点属性，比如节点名、topic 名、服务名等等。
在上一个教程中，你使用重映射在 ``turtle_teleop_key`` 上更改了 ``cmd_vel`` topic 使它用来控制 **turtle2**。

现在，让我们重新分配 ``/turtlesim`` 节点的名字。
在一个新终端中，运行以下指令：

.. code-block:: console

  ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle

再次运行 ``ros2 run`` 时会打开一个新的 turtlesim 窗口。
不过，现在如果你回到运行 ``ros2 node list`` 的终端再运行一次，你会看到三个节点名字：

.. code-block:: console

    /my_turtle
    /turtlesim
    /teleop_turtle

3 ros2 node info
^^^^^^^^^^^^^^^^

现在你知道了节点的名字，你可以使用以下指令查看更多关于它们的信息：

.. code-block:: console

    ros2 node info <node_name>

为了查看你最新的 ``my_turtle`` 节点，运行以下指令：

.. code-block:: console

    ros2 node info /my_turtle

``ros2 node info`` 会返回一个订阅者、发布者、服务和 action 的列表，即在 ROS graph 中 与该节点有交互的连接。
输出应该是这样的：

.. code-block:: console

  /my_turtle
    Subscribers:
      /parameter_events: rcl_interfaces/msg/ParameterEvent
      /turtle1/cmd_vel: geometry_msgs/msg/Twist
    Publishers:
      /parameter_events: rcl_interfaces/msg/ParameterEvent
      /rosout: rcl_interfaces/msg/Log
      /turtle1/color_sensor: turtlesim/msg/Color
      /turtle1/pose: turtlesim/msg/Pose
    Service Servers:
      /clear: std_srvs/srv/Empty
      /kill: turtlesim/srv/Kill
      /my_turtle/describe_parameters: rcl_interfaces/srv/DescribeParameters
      /my_turtle/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
      /my_turtle/get_parameters: rcl_interfaces/srv/GetParameters
      /my_turtle/list_parameters: rcl_interfaces/srv/ListParameters
      /my_turtle/set_parameters: rcl_interfaces/srv/SetParameters
      /my_turtle/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
      /reset: std_srvs/srv/Empty
      /spawn: turtlesim/srv/Spawn
      /turtle1/set_pen: turtlesim/srv/SetPen
      /turtle1/teleport_absolute: turtlesim/srv/TeleportAbsolute
      /turtle1/teleport_relative: turtlesim/srv/TeleportRelative
    Service Clients:

    Action Servers:
      /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
    Action Clients:

现在尝试对 ``/teleop_turtle`` 节点运行同样的指令，看看它的连接与 ``my_turtle`` 有什么不同。

在接下来的教程中，你会学到更多关于 ROS graph 中不同连接概念，包括消息类型。

总结
-------

节点是 ROS 2 中的基本元素，它们在机器人系统中服务于单一、模块化的目的。

在这个教程中，你通过运行 ``turtlesim`` 包中的 ``turtlesim_node`` 和 ``turtle_teleop_key`` 可执行文件来使用节点。

你学会了如何使用 ``ros2 node list`` 来发现正在运行的节点名字，以及用 ``ros2 node info`` 来检查与单个节点有关的信息。
这些工具对于理解复杂的真实机器人系统中的数据流是至关重要的。

下一步
----------

现在你了解了 ROS 2 中的节点，你可以继续学习 :doc:`topics 教程 <../Understanding-ROS2-Topics/Understanding-ROS2-Topics>`。
Topics 是节点间通信的方式之一。

相关内容
---------------

:doc:`../../../Concepts` 页面对节点的概念介绍了更多细节。
