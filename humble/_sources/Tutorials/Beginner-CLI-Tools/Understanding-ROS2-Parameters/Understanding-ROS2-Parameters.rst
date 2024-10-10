.. redirect-from::

    Tutorials/Parameters/Understanding-ROS2-Parameters

.. _ROS2Params:

理解参数
========================

**目标:** 学习如何在 ROS 2 中获取、配置、保存或重新加载参数.

**教程等级:** 初级

**预计时长:** 5 分钟

.. contents:: Contents
   :depth: 2
   :local:

背景
----------

参数是节点的配置值的一部份。
你可以将参数看作对节点的设置。
一个节点可以将参数存储为整数、浮点数、布尔值、字符串和列表。
在 ROS 2 中，每个节点都维护自己的参数。
有关参数的更多背景信息，请参阅 :doc:`参数概念文档 <../../../Concepts/Basic/About-Parameters>`。

前提条件
-------------

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


2 ros2 param list
^^^^^^^^^^^^^^^^^

要查看属于你的节点的参数，请打开一个新终端并输入以下命令:

.. code-block:: console

    ros2 param list

你将看到节点命名空间， ``/teleop_turtle`` 和 ``/turtlesim``，后面是每个节点的参数:

.. code-block:: console

  /teleop_turtle:
    qos_overrides./parameter_events.publisher.depth
    qos_overrides./parameter_events.publisher.durability
    qos_overrides./parameter_events.publisher.history
    qos_overrides./parameter_events.publisher.reliability
    scale_angular
    scale_linear
    use_sim_time
  /turtlesim:
    background_b
    background_g
    background_r
    qos_overrides./parameter_events.publisher.depth
    qos_overrides./parameter_events.publisher.durability
    qos_overrides./parameter_events.publisher.history
    qos_overrides./parameter_events.publisher.reliability
    use_sim_time

每个节点都有参数 ``use_sim_time``; 这不是 turtlesim 的特有参数。

根据命名，可以猜到 ``/turtlesim`` 的参数使用 RGB 颜色值确定 turtlesim 窗口的背景颜色。

要确定参数的类型，可以使用 ``ros2 param get``。


3 ros2 param get
^^^^^^^^^^^^^^^^

要显示参数的类型和当前值，请使用以下命令:

.. code-block:: console

    ros2 param get <node_name> <parameter_name>

让我们找出 ``/turtlesim`` 的参数 ``background_g`` 的当前值:

.. code-block:: console

    ros2 param get /turtlesim background_g

这将返回:

.. code-block:: console

    Integer value is: 86

现在你知道 ``background_g`` 保存了一个整数值。

如果你在 ``background_r`` 和 ``background_b`` 上运行相同的命令，你将得到值 ``69`` 和 ``255``。

4 ros2 param set
^^^^^^^^^^^^^^^^

要在运行时更改参数的值，请使用以下命令:

.. code-block:: console

    ros2 param set <node_name> <parameter_name> <value>

让我们更改 ``/turtlesim`` 的背景颜色:

.. code-block:: console

    ros2 param set /turtlesim background_r 150

你的终端应该返回消息:

.. code-block:: console

  Set parameter successful

并且你的 turtlesim 窗口的背景颜色应该改变:

.. image:: images/set.png

使用 ``set`` 命令设置参数只会在当前会话中更改参数，而不是永久更改。
但是，你可以保存你的设置并在下次启动节点时重新加载它们。

5 ros2 param dump
^^^^^^^^^^^^^^^^^

使用以下命令可以查看节点的所有当前参数值:

.. code-block:: console

  ros2 param dump <node_name>

默认情况下，该命令将打印到标准输出（stdout），但你也可以将参数值重定向到文件中以便稍后保存。
要将当前 ``/turtlesim`` 的参数配置保存到文件 ``turtlesim.yaml`` 中，请输入以下命令:

.. code-block:: console

  ros2 param dump /turtlesim > turtlesim.yaml

运行完之后，你会在当前工作目录中找到这个新文件。
打开这个文件能看到如下内容:

.. code-block:: YAML

  /turtlesim:
    ros__parameters:
      background_b: 255
      background_g: 86
      background_r: 150
      qos_overrides:
        /parameter_events:
          publisher:
            depth: 1000
            durability: volatile
            history: keep_last
            reliability: reliable
      use_sim_time: false

在将来重新加载节点时，参数转储会很有用。

6 ros2 param load
^^^^^^^^^^^^^^^^^

使用以下命令可以从文件加载参数到当前运行的节点:

.. code-block:: console

  ros2 param load <node_name> <parameter_file>

要将使用 ``ros2 param dump`` 生成的 ``turtlesim.yaml`` 文件加载到 ``/turtlesim`` 节点的参数中，请输入以下命令:

.. code-block:: console

  ros2 param load /turtlesim turtlesim.yaml

你的终端将返回消息:

.. code-block:: console

  Set parameter background_b successful
  Set parameter background_g successful
  Set parameter background_r successful
  Set parameter qos_overrides./parameter_events.publisher.depth failed: parameter 'qos_overrides./parameter_events.publisher.depth' cannot be set because it is read-only
  Set parameter qos_overrides./parameter_events.publisher.durability failed: parameter 'qos_overrides./parameter_events.publisher.durability' cannot be set because it is read-only
  Set parameter qos_overrides./parameter_events.publisher.history failed: parameter 'qos_overrides./parameter_events.publisher.history' cannot be set because it is read-only
  Set parameter qos_overrides./parameter_events.publisher.reliability failed: parameter 'qos_overrides./parameter_events.publisher.reliability' cannot be set because it is read-only
  Set parameter use_sim_time successful

.. note::

  只读参数只能在启动时修改，而不能在之后修改，这就是为什么 "qos_overrides" 参数会有一些警告。

7 在节点启动时加载参数文件
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

要使用保存的参数值重新启动相同的节点，请使用:

.. code-block:: console

  ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>

用你启动 turtlesim 的命令，加上 ``--ros-args`` 和 ``--params-file`` 标志，后面跟着你想要加载的文件。

现在停止你正在运行的 turtlesim 节点，然后尝试使用保存的参数重新加载它:

.. code-block:: console

  ros2 run turtlesim turtlesim_node --ros-args --params-file turtlesim.yaml

turtlesim 窗口应该像往常一样出现，但是背景颜色应该是你之前设置的紫色。

.. note::

  当在节点启动时使用参数文件时，所有参数，包括只读参数，都会被更新。

总结
-------

节点使用参数来定义它们的默认配置值。
你可以从命令行中 ``get`` 和 ``set`` 参数值。
你还可以将参数设置保存到文件中以之后后重新加载。

下一步
----------

回到 ROS 2 通信方法，下一个教程将介绍 :doc:`actions <../Understanding-ROS2-Actions/Understanding-ROS2-Actions>`。
