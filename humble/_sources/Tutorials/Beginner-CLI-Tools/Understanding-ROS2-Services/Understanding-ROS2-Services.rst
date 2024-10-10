.. redirect-from::

    Tutorials/Services/Understanding-ROS2-Services

.. _ROS2Services:

理解服务
======================

**目标:** 使用命令行工具学习 ROS 2 中的服务

**教程等级:** 初级

**预计时长:** 10 分钟

.. contents:: Contents
   :depth: 2
   :local:

背景
----------

服务是 ROS 图中节点之间的另一种通信方式。
服务基于请求-响应模型，而不是话题的发布-订阅模型。
话题允许节点订阅数据流并持续获取更新，而服务只在特定时刻被调用时提供数据。

.. image:: images/Service-SingleServiceClient.gif

.. image:: images/Service-MultipleServiceClient.gif

前提条件
-------------

本教程中提到的一些概念，比如 :doc:`节点 <../Understanding-ROS2-Nodes/Understanding-ROS2-Nodes>` 和 :doc:`topic <../Understanding-ROS2-Topics/Understanding-ROS2-Topics>`，在之前的教程中有介绍。

你需要安装 :doc:`turtlesim package <../Introducing-Turtlesim/Introducing-Turtlesim>`。

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

2 ros2 service list
^^^^^^^^^^^^^^^^^^^

在新终端中运行 ``ros2 service list`` 命令，返回当前系统中所有运行中服务的列表:

.. code-block:: console

  /clear
  /kill
  /reset
  /spawn
  /teleop_turtle/describe_parameters
  /teleop_turtle/get_parameter_types
  /teleop_turtle/get_parameters
  /teleop_turtle/list_parameters
  /teleop_turtle/set_parameters
  /teleop_turtle/set_parameters_atomically
  /turtle1/set_pen
  /turtle1/teleport_absolute
  /turtle1/teleport_relative
  /turtlesim/describe_parameters
  /turtlesim/get_parameter_types
  /turtlesim/get_parameters
  /turtlesim/list_parameters
  /turtlesim/set_parameters
  /turtlesim/set_parameters_atomically

你会发现两个节点都有相同的六个服务，服务名称中都包含 ``parameters``。
这是因为 ROS 2 中的几乎每个节点都有这些基础服务，参数是基于这些服务构建的。
在下一个教程中会有更多关于参数的内容。
在本教程中，将忽略有关参数服务的讨论。

现在，让我们专注于 turtlesim 特定的服务， ``/clear`` 、 ``/kill`` 、 ``/reset`` 、 ``/spawn`` 、 ``/turtle1/set_pen`` 、 ``/turtle1/teleport_absolute`` 和 ``/turtle1/teleport_relative``。
你可能还记得在 :doc:`Use turtlesim, ros2, and rqt <../Introducing-Turtlesim/Introducing-Turtlesim>` 教程中使用 rqt 与这些服务交互。


3 ros2 service type
^^^^^^^^^^^^^^^^^^^

服务用类型来描述服务的请求和响应数据的结构。
服务类型的定义方式与 topic 类型类似，只是服务类型有两部分：一个用于请求的消息，另一个用于响应的消息。

要查找服务的类型，使用命令:

.. code-block:: console

  ros2 service type <service_name>

让我们看看 turtlesim 的 ``/clear`` 服务。
在新终端中输入命令:

.. code-block:: console

  ros2 service type /clear

这会返回:

.. code-block:: console

  std_srvs/srv/Empty

``Empty`` 类型意味着服务调用在请求时不发送数据，在接收响应时也不接收数据。

3.1 ros2 service list -t
~~~~~~~~~~~~~~~~~~~~~~~~

要同时查看所有活跃（active）服务的类型，可以在 ``list`` 命令后添加 ``--show-types`` 选项，简写为 ``-t``:

.. code-block:: console

  ros2 service list -t

这会返回:

.. code-block:: console

  /clear [std_srvs/srv/Empty]
  /kill [turtlesim/srv/Kill]
  /reset [std_srvs/srv/Empty]
  /spawn [turtlesim/srv/Spawn]
  ...
  /turtle1/set_pen [turtlesim/srv/SetPen]
  /turtle1/teleport_absolute [turtlesim/srv/TeleportAbsolute]
  /turtle1/teleport_relative [turtlesim/srv/TeleportRelative]
  ...

4 ros2 service find
^^^^^^^^^^^^^^^^^^^

如果你想找到所有特定类型的服务，可以使用命令:

.. code-block:: console

  ros2 service find <type_name>

例如，你可以这样找到所有 ``Empty`` 类型的服务:

.. code-block:: console

  ros2 service find std_srvs/srv/Empty

这会返回:

.. code-block:: console

  /clear
  /reset

5 ros2 interface show
^^^^^^^^^^^^^^^^^^^^^

你可以从命令行请求服务，但首先需要了解输入参数的结构。

.. code-block:: console

  ros2 interface show <type_name>

尝试对 ``/clear`` 服务的类型 ``Empty`` 使用 ``interface show`` 命令:

.. code-block:: console

  ros2 interface show std_srvs/srv/Empty

这会返回:

.. code-block:: console

  ---

``---`` 分隔了请求结构（上面）和响应结构（下面）。
但是，正如你之前了解到的，``Empty`` 类型不发送或接收任何数据。
所以，这里它的结构是空的。

让我们看一个会交换数据的服务类型，比如 ``/spawn``。
从 ``ros2 service list -t`` 的结果中，我们知道 ``/spawn`` 的类型是 ``turtlesim/srv/Spawn``。

要查看 ``/spawn`` 服务的请求和响应参数，运行命令:

.. code-block:: console

  ros2 interface show turtlesim/srv/Spawn

这会返回:

.. code-block:: console

  float32 x
  float32 y
  float32 theta
  string name # Optional.  A unique name will be created and returned if this is empty
  ---
  string name

``---`` 行上面的信息告诉我们调用 ``/spawn`` 需要的参数。
其中 ``x``、 ``y`` 和 ``theta`` 确定了生成的小乌龟的 2D 姿态，``name`` 显然是可选的（optional）。

``---`` 行下面的信息不是你在这种情况下需要知道的，但它可以帮助你了解调用返回的响应的数据类型。

6 ros2 service call
^^^^^^^^^^^^^^^^^^^

现在你知道了服务类型是什么，如何找到服务的类型，以及如何找到该类型的参数结构，你可以使用以下命令请求服务:

.. code-block:: console

  ros2 service call <service_name> <service_type> <arguments>

``<arguments>`` 部分是可选的。
例如，你知道 ``Empty`` 类型的服务没有任何参数:

.. code-block:: console

  ros2 service call /clear std_srvs/srv/Empty

这会清除小乌龟窗口中的所有已经画出来的线条。

.. image:: images/clear.png

现在我们调用 ``/spawn`` 服务生成一个新的小乌龟。
在命令行调用中， ``<arguments>`` 部分需要使用 YAML 语法。

输入命令:

.. code-block:: console

  ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: ''}"

你会得到这种 method-style 的输出，显示请求了什么和响应了什么:

.. code-block:: console

  requester: making request: turtlesim.srv.Spawn_Request(x=2.0, y=2.0, theta=0.2, name='')

  response:
  turtlesim.srv.Spawn_Response(name='turtle2')

你的 turtlesim 窗口会立即更新，显示新生成的小乌龟:

.. image:: images/spawn.png

总结
-------

ROS 2 中的节点可以使用服务进行通信。
与 topic 不同，服务是一种请求-响应模式，其中客户端向提供服务的节点发出请求，服务处理请求并生成响应。

通常不建议使用服务进行连续调用，topic 或者 action 更适合。

在本教程中，你使用命令行工具识别、检查（introspect）和调用服务。

下一步
----------

在下一个教程中， :doc:`../Understanding-ROS2-Parameters/Understanding-ROS2-Parameters`，你将学习如何配置节点设置。

相关内容
---------------

`这个教程 <https://discourse.ubuntu.com/t/call-services-in-ros-2/15261>`_ 是一个用 ROS 2 服务控制 Robotis 机械臂的的优秀实际应用。
