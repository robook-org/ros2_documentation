.. redirect-from::

    About-ROS-2-Parameters
    Concepts/About-ROS-2-Parameters

参数(Parameters)
=================

.. contents:: Table of Contents
   :local:

概览
--------

ROS 2 中的参数与单个节点相关联。
参数用于在启动时（以及运行时）配置节点，而无需更改代码。
参数的生命周期与节点的生命周期绑定（当然节点可以用某种持久化的方式以在重新启动后重新加载保存的参数）。

参数由节点名称、节点命名空间、参数名称和参数命名空间来标识。
其中参数命名空间是可选的。

每个参数由一个键(key)、一个值(value)和一个描述符(descriptor)组成。
键是一个字符串，值是以下类型之一：``bool``、``int64``、``float64``、``string``、``byte[]``、``bool[]``、``int64[]``、``float64[]`` 或 ``string[]``。
默认情况下，所有描述符都是空的，但可以包含参数描述、值范围、类型信息和其他约束。

有关 ROS 2 参数的更多信息，请参见 :doc:`../../Concepts/Basic/About-Parameters`。

参数相关背景
---------------------

声明参数
^^^^^^^^^^^^^^^^^^^^

默认情况下，节点需要声明其在生命周期中可以接受的所有参数。
这样，参数的类型和名称在节点启动时就被定义好了，降低了后续配置错误的可能性。
请参见 :doc:`../../Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-CPP` 或 :doc:`../../Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python` 了解如何声明和使用节点中的参数。

对于某些类型的节点，不是所有参数都能提前知道。
在这些情况下，可以实例化节点时将 ``allow_undeclared_parameters`` 设置为 ``true``，这样即使参数没有被声明，也可以在节点上进行 get 和 set 操作。

参数类型
^^^^^^^^^^^^^^^

如概述中所述，ROS 2 节点上的每个参数都有一个预定义的参数类型。
默认情况下，无法在运行时更改已声明参数的类型。
这可以防止一些常见的错误，例如将布尔值放入整数参数中。

如果参数需要是多种不同类型，并且使用参数的代码可以处理这样的情况，也可以更改参数类型的默认表现（即可以改成运行时也能传递不同类型的参数）。
在声明参数时，应使用 ``ParameterDescriptor`` 声明参数，其中 ``dynamic_typing`` 成员变量设置为 ``true``。

参数回调(callbacks)
^^^^^^^^^^^^^^^^^^^^^

ROS 2 节点可以注册两种不同类型的回调，以便在参数发生更改时得到通知。
这两种回调都是可选的。

第一种称为 "set parameter" 回调，可以通过从节点 API 调用 ``add_on_set_parameters_callback`` 来设置。
回调函数接收一个不可变的 ``Parameter`` 对象列表，并返回一个 ``rcl_interfaces/msg/SetParametersResult``。
这个回调的主要目的是让用户能够检查参数的即将发生的更改，并可以显式地拒绝更改。

.. note::
    "set parameter" 回调不应该有副作用。
    由于可以链接多个 "set parameter" 回调，因此没有办法让单个回调知道后续的回调是否会拒绝更新。
    如果某个回调对其所在的类进行更改，可能会导致程序认为的与更改后的实际参数不同步。
    为了在参数成功 *更改后* 获得回调，请参见下面的下一种回调类型。

第二种回调称为 "on parameter event" 回调，可以通过从参数客户端 API 调用 ``on_parameter_event`` 来设置。
回调函数接收一个 ``rcl_interfaces/msg/ParameterEvent`` 对象，并不返回任何内容。
此回调将在输入事件中的所有参数已声明、更改或删除后调用。
此回调的主要目的是让用户能够对成功接受的参数更改做出反应。

与参数交互
---------------------------

ROS 2 节点可以通过节点 API操作参数，如 :doc:`../../Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-CPP` 或 :doc:`../../Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python` 中所述
外部程序可以通过节点实例化时默认创建的参数服务操作参数。
默认创建的服务有：

* ``/node_name/describe_parameters``: service 类型 ``rcl_interfaces/srv/DescribeParameters``.
  给定参数名称列表，返回与参数关联的描述符列表。
* ``/node_name/get_parameter_types``: service 类型 ``rcl_interfaces/srv/GetParameterTypes``.
  给定参数名称列表，返回与参数关联的参数类型列表。
* ``/node_name/get_parameters``: service 类型 ``rcl_interfaces/srv/GetParameters``.
  给定参数名称列表，返回与参数关联的参数值列表。
* ``/node_name/list_parameters``: service 类型 ``rcl_interfaces/srv/ListParameters``.
  给定可选的参数前缀列表，返回具有该前缀的可用参数列表。如果前缀为空，则返回所有参数。
* ``/node_name/set_parameters``: service 类型 ``rcl_interfaces/srv/SetParameters``.
  给定参数名称和值列表，尝试在节点上设置参数。返回尝试设置每个参数的结果列表；有些可能成功，有些可能失败。
* ``/node_name/set_parameters_atomically``: service 类型 ``rcl_interfaces/srv/SetParametersAtomically``.
  给定参数名称和值列表，尝试在节点上 atomically 设置参数（也就是一次性设置所有给定的参数）。返回尝试设置所有参数的结果，如果任何一个设置失败，返回值即为失败。

在运行节点时设置初始参数值
----------------------------------------------------

在运行节点时，可以通过单独的命令行参数或 YAML 文件设置初始参数值。
请参见 :ref:`NodeArgsParameters` 了解如何设置初始参数值的示例。

在启动节点时设置初始参数值
-----------------------------------------------------

还可以在在 ROS 2 launch (译者注：这是一种启动节点的方式，可以在教程中查看 launch 有关的内容)节点时设置初始参数值。
请参见 :doc:`../../Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects` 了解如何在 launch 时指定参数。

在运行时修改参数值
----------------------------------------

``ros2 param`` 命令是与已经运行的节点交互的通用方式。
``ros2 param`` 使用参数服务 API 来执行各种操作。
请参见 :doc:`../../How-To-Guides/Using-ros2-param` 了解如何使用 ``ros2 param``。

从 ROS 1 迁移
--------------------

:doc:`Launch 文件迁移指南 <../../How-To-Guides/Migrating-from-ROS1/Migrating-Launch-Files>` 解释了如何从 ROS 1 迁移 ``param`` 和 ``rosparam`` launch 标签到 ROS 2。

:doc:`YAML 参数文件迁移指南 <../../How-To-Guides/Migrating-from-ROS1/Migrating-Parameters>` 解释了如何从 ROS 1 迁移参数文件到 ROS 2。

在 ROS 1 中，``roscore`` 就像一个全局参数黑板(global parameter blackboard)，所有节点都可以从中获取和设置参数。
由于 ROS 2 中没有中心化的 ``roscore``，这种功能不再存在。
ROS 2 中推荐的方法是节点只使用与之紧密相关的节点参数。
如果仍然需要全局黑板，可以为此目的创建一个专用节点。
ROS 2 中的 ``ros-{DISTRO}-demo-nodes-cpp`` 包中附带一个名为 ``parameter_blackboard`` 的节点；可以通过以下命令运行：

.. code-block:: console

   ros2 run demo_nodes_cpp parameter_blackboard

``parameter_blackboard`` 的代码在 `这里 <https://github.com/ros2/demos/blob/{REPOS_FILE_BRANCH}/demo_nodes_cpp/src/parameters/parameter_blackboard.cpp>`__.
