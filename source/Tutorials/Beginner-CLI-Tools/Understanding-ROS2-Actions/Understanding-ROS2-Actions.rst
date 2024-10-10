.. redirect-from::

    Tutorials/Understanding-ROS2-Actions

.. _ROS2Actions:

理解 actions
=====================

**目标:** 学习 ROS 2 中的 action.

**教程等级:** 初级

**预计时长:** 15 分钟

.. contents:: Contents
   :depth: 2
   :local:

背景
----------

Actions 是 ROS 2 中的通信类型之一，用于长时间运行的任务。
它们由三部分组成: 目标(goal)、反馈(feedback)和结果(result)。

Actions 建立在 topics 和 services 的基础上。
它们的功能类似于 services，但 actions 可以被取消。
它们还提供稳定的反馈，而不像 services 那样只返回一个响应。

Actions 使用客户端-服务器模型，类似于发布-订阅模型（在 :doc:`topics 教程 <../Understanding-ROS2-Topics/Understanding-ROS2-Topics>` 中描述）。
一个 "action client" 节点发送一个目标到一个 "action server" 节点，后者确认目标并返回一系列反馈和结果。

.. image:: images/Action-SingleActionClient.gif

前提条件
-------------

本教程建立在之前教程中介绍的概念之上，比如 :doc:`nodes <../Understanding-ROS2-Nodes/Understanding-ROS2-Nodes>` 和 :doc:`topics <../Understanding-ROS2-Topics/Understanding-ROS2-Topics>`。

本教程使用 :doc:`turtlesim package <../Introducing-Turtlesim/Introducing-Turtlesim>`。

如往常一样，不要忘记在 :doc:`每次打开新终端时 <../Configuring-ROS2-Environment>` source ROS 2。

任务
-----

1 Setup
^^^^^^^

启动两个 turtlesim 节点， ``/turtlesim`` 和 ``/teleop_turtle``。

打开一个新终端并运行:

.. code-block:: console

    ros2 run turtlesim turtlesim_node

打开另一个终端并运行:

.. code-block:: console

    ros2 run turtlesim turtle_teleop_key


2 Use actions
^^^^^^^^^^^^^

当你启动 ``/teleop_turtle`` 节点时，你会在终端中看到以下消息:

.. code-block:: console

    Use arrow keys to move the turtle.
    Use G|B|V|C|D|E|R|T keys to rotate to absolute orientations. 'F' to cancel a rotation.

让我们关注第二行，它对应一个 action 。
(第一个指令对应于之前在 :doc:`topics 教程 <../Understanding-ROS2-Topics/Understanding-ROS2-Topics>` 中讨论的 "cmd_vel" topic。)

你应该已经注意到，键盘上的字母键 ``G|B|V|C|D|E|R|T`` 组成了一个围绕键盘上 ``F`` 键的 "方块"（但是如果你没有使用 QWERTY 键盘，请参考 `这个链接 <https://upload.wikimedia.org/wikipedia/commons/d/da/KB_United_States.svg>`__）。
每个键在 ``F`` 周围的位置对应于 turtlesim 中的方向。
例如，``E`` 会让乌龟转向左上角。

按下这些键时，注意 ``/turtlesim`` 节点所在的终端。
每次按下这些键之一时，你都会向 ``/turtlesim`` 节点的一个 action 服务器发送一个目标。
目标是让乌龟转向特定方向。
过一会儿小乌龟到达目标之后就会显示如下结果:

.. code-block:: console

    [INFO] [turtlesim]: Rotation goal completed successfully

``F`` 键会在目标执行过程中取消任务。

尝试按下 ``C`` 键，然后在小乌龟完成旋转之前按下 ``F`` 键。
在 ``/turtlesim`` 节点运行的终端中，你会看到消息:

.. code-block:: console

  [INFO] [turtlesim]: Rotation goal canceled

当然，不止能在客户端（你在 teleop 中的输入）停止一个目标，服务器端（ ``/turtlesim`` 节点）也可以。
当服务器端选择停止处理一个目标时，它被称为 "终止(abort)" 该目标。

尝试在第一个旋转完成之前按下 ``D`` 键，然后在第一个旋转完成之前按下 ``G`` 键。
在 ``/turtlesim`` 节点运行的终端中，你会看到消息:

.. code-block:: console

  [WARN] [turtlesim]: Rotation goal received before a previous goal finished. Aborting previous goal

这个 action 服务器选择终止第一个目标，因为它收到了一个新目标。
它也可以选择其他操作，比如拒绝新目标或在第一个目标完成后执行第二个目标。（仅仅是这个样例中实现了这样的逻辑。）
不要假设每个 action 服务器都会在收到新目标时选择终止当前目标。

3 ros2 node info
^^^^^^^^^^^^^^^^

要查看节点提供的 action 列表，例如 ``/turtlesim``，打开一个新终端并运行以下命令:

.. code-block:: console

    ros2 node info /turtlesim

这将返回一个 ``/turtlesim`` 的订阅者、发布者、服务、action 服务器和 action 客户端的列表:

.. code-block:: console

  /turtlesim
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
      /reset: std_srvs/srv/Empty
      /spawn: turtlesim/srv/Spawn
      /turtle1/set_pen: turtlesim/srv/SetPen
      /turtle1/teleport_absolute: turtlesim/srv/TeleportAbsolute
      /turtle1/teleport_relative: turtlesim/srv/TeleportRelative
      /turtlesim/describe_parameters: rcl_interfaces/srv/DescribeParameters
      /turtlesim/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
      /turtlesim/get_parameters: rcl_interfaces/srv/GetParameters
      /turtlesim/list_parameters: rcl_interfaces/srv/ListParameters
      /turtlesim/set_parameters: rcl_interfaces/srv/SetParameters
      /turtlesim/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
    Service Clients:

    Action Servers:
      /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
    Action Clients:

注意 ``/turtlesim`` 的 ``/turtle1/rotate_absolute`` action 在 ``Action Servers`` 下。
这意味着 ``/turtlesim`` 响应并提供反馈给 ``/turtle1/rotate_absolute`` action。

而``/teleop_turtle`` 节点在 ``Action Clients`` 下有名为 ``/turtle1/rotate_absolute`` 的 action，这意味着它能向有这个名称的 action 发送目标。

要查看 ``/teleop_turtle`` 节点的信息，运行以下命令:

.. code-block:: console

    ros2 node info /teleop_turtle

这将返回:

.. code-block:: console

  /teleop_turtle
    Subscribers:
      /parameter_events: rcl_interfaces/msg/ParameterEvent
    Publishers:
      /parameter_events: rcl_interfaces/msg/ParameterEvent
      /rosout: rcl_interfaces/msg/Log
      /turtle1/cmd_vel: geometry_msgs/msg/Twist
    Service Servers:
      /teleop_turtle/describe_parameters: rcl_interfaces/srv/DescribeParameters
      /teleop_turtle/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
      /teleop_turtle/get_parameters: rcl_interfaces/srv/GetParameters
      /teleop_turtle/list_parameters: rcl_interfaces/srv/ListParameters
      /teleop_turtle/set_parameters: rcl_interfaces/srv/SetParameters
      /teleop_turtle/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
    Service Clients:

    Action Servers:

    Action Clients:
      /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute

4 ros2 action list
^^^^^^^^^^^^^^^^^^

要查找 ROS 图中的所有 action，运行以下命令:

.. code-block:: console

    ros2 action list

这将返回:

.. code-block:: console

    /turtle1/rotate_absolute

这是目前 ROS 图中唯一的 action。
正如你之前看到的，它能控制乌龟的旋转。
而且通过使用 ``ros2 node info <node_name>`` 命令，你也已经知道这个 action 有一个 action 客户端（属于 ``/teleop_turtle``）和一个 action 服务器（属于 ``/turtlesim``）。

4.1 ros2 action list -t
~~~~~~~~~~~~~~~~~~~~~~~

action 有类型，类似于 topic 和 service。
要找到 ``/turtle1/rotate_absolute`` 的类型，运行以下命令:

.. code-block:: console

    ros2 action list -t

这将返回:

.. code-block:: console

    /turtle1/rotate_absolute [turtlesim/action/RotateAbsolute]

在每个 action 名称右边的括号中（目前只有 ``/turtle1/rotate_absolute``）标注着 action 类型， ``turtlesim/action/RotateAbsolute``。
你在从命令行或代码中请求 action 时会需要这个类型。

5 ros2 action info
^^^^^^^^^^^^^^^^^^

你可以进一步检查 ``/turtle1/rotate_absolute`` action，使用以下命令:

.. code-block:: console

    ros2 action info /turtle1/rotate_absolute

这将返回:

.. code-block:: console

  Action: /turtle1/rotate_absolute
  Action clients: 1
      /teleop_turtle
  Action servers: 1
      /turtlesim

像我们我们之前用 ``ros2 node info`` 一样，这条指令也能告诉我们对应 action 的有关信息:
这个 action 有一个客户端在 ``/teleop_turtle`` 节点上运行，有一个服务器在 ``/turtlesim`` 上。

6 ros2 interface show
^^^^^^^^^^^^^^^^^^^^^

在发送或执行 action 目标之前，你还需要了解 action 类型的结构。

回想一下，当你运行 ``ros2 action list -t`` 命令时，你识别出了 ``/turtle1/rotate_absolute`` 的类型。
要查看这个 action 类型的结构，使用以下命令:

.. code-block:: console

  ros2 interface show turtlesim/action/RotateAbsolute

这会返回:

.. code-block:: console

  # The desired heading in radians
  float32 theta
  ---
  # The angular displacement in radians to the starting position
  float32 delta
  ---
  # The remaining rotation in radians
  float32 remaining

这条消息的第一个 ``---`` 之上的部分是目标请求的结构（数据类型和名称）。
接下来的部分是结果的结构。
最后一部分是反馈的结构。

7 ros2 action send_goal
^^^^^^^^^^^^^^^^^^^^^^^

现在让我们使用以下语法从命令行发送一个 action 目标:

.. code-block:: console

    ros2 action send_goal <action_name> <action_type> <values>

``<values>`` 需要使用 YAML 格式。

输入以下指令，并且注意观察 turtlesim 窗口:

.. code-block:: console

    ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}"

你应该看到小乌龟旋转，以及在终端中看到以下消息:

.. code-block:: console

  Waiting for an action server to become available...
  Sending goal:
     theta: 1.57

  Goal accepted with ID: f8db8f44410849eaa93d3feb747dd444

  Result:
    delta: -1.568000316619873

  Goal finished with status: SUCCEEDED

所有目标都有一个唯一的 ID，显示在返回消息中。
你可以看到结果，一个名为 ``delta`` 的字段，它是起始位置与当前位置之间的位移。

如果要查看这个目标的反馈，添加 ``--feedback`` 到 ``ros2 action send_goal`` 命令:

.. code-block:: console

    ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: -1.57}" --feedback

你的终端会返回消息:

.. code-block:: console

  Sending goal:
     theta: -1.57

  Goal accepted with ID: e6092c831f994afda92f0086f220da27

  Feedback:
    remaining: -3.1268222332000732

  Feedback:
    remaining: -3.1108222007751465

  …

  Result:
    delta: 3.1200008392333984

  Goal finished with status: SUCCEEDED

这个反馈字段 ``remaining`` 会告诉你乌龟还需要多少弧度才能完成旋转。

总结
-------

Actions 就像服务，允许你执行长时间运行的任务，提供定期反馈，并且可以被取消。

一个机器人系统可能会使用 actions 来导航。
一个 action 目标可以告诉机器人去某个位置。
当机器人导航到这个位置时，它可以发送更新（即反馈），然后在到达目的地后返回一个最终结果。

action 客户端可以向 turtlesim 的 action 服务器发送目标，以控制乌龟的旋转。
在这个教程中，你对这个 action ``/turtle1/rotate_absolute`` 做了更细致的检查，以更好地了解 action 是什么以及它是如何工作的。

下一步
----------

现在你已经掌握了所有 ROS 2 核心概念。
你可以继续学习更多关于 ROS 2 的内容，比如 :doc:`../Using-Rqt-Console/Using-Rqt-Console`。

相关内容
---------------

你可以在 `这里 <https://design.ros2.org/articles/actions.html>`__ 了解更多关于 ROS 2 actions 的设计的信息。
