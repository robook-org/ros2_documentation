.. redirect-from::

    Tutorials/Rqt-Console/Using-Rqt-Console

.. _rqt_console:

使用 ``rqt_console`` 查看日志
==================================

**目标:** 了解与日志交互的工具 ``rqt_console``.

**教程等级:** 初级

**预计时长:** 5 分钟

.. contents:: Contents
   :depth: 2
   :local:

背景
----------

``rqt_console`` 是一个 GUI 工具，用于检查 ROS 2 中的日志消息。
一般情况下，日志消息会显示在终端中。
不过，使用 ``rqt_console``，你可以收集这些随时间产生的消息，更仔细地查看它们，以更有组织的方式查看、过滤、保存它们，甚至重新加载保存的文件以在不同时间内查看。
节点使用日志以各种方式输出有关各种事件和状态的消息。
日志的内容一般包含有很多信息性的输出，以便为用户提供足够的信息。

前提条件
-------------

你需要已经安装 :doc:`rqt_console 和 turtlesim <../Introducing-Turtlesim/Introducing-Turtlesim>`。

如往常一样，不要忘记在 :doc:`每次打开新终端时 <../Configuring-ROS2-Environment>` source ROS 2。


任务
-----

1 Setup
^^^^^^^

在新终端中输入以下命令启动 ``rqt_console``:

.. code-block:: console

    ros2 run rqt_console rqt_console

``rqt_console`` 窗口会打开:

.. image:: images/console.png

控制台的第一部分是系统中的日志消息显示区域。

中间部份可以通过设置严重级别来过滤消息。
也可以用右侧的加号按钮添加更多排除过滤条件。

底部部分是用于高亮包含你输入的字符串的消息。
当然也可以添加更多过滤条件。

现在在新终端中输入以下命令启动 ``turtlesim``:

.. code-block:: console

    ros2 run turtlesim turtlesim_node

2 rqt_console 中的消息
^^^^^^^^^^^^^^^^^^^^^^^^^

让乌龟撞到墙上的时候会在 ``rqt_console`` 显示日志消息，。
在新终端中输入以下命令（在 :doc:`topics 教程 <../Understanding-ROS2-Topics/Understanding-ROS2-Topics>` 中有详细讨论）:

.. code-block:: console

    ros2 topic pub -r 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}"

上面的命令以恒定的速率发布 topic，所以乌龟会不停地撞到墙上。
在 ``rqt_console`` 中你会看到标有 ``Warn`` 的消息一遍又一遍地显示，如下图所示:

.. image:: images/warn.png

在你的终端中按 ``Ctrl+C`` 停止 ``ros2 topic pub`` 命令，让乌龟不再撞墙。

3 Logger 等级
^^^^^^^^^^^^^^^

ROS 2 的日志等级按严重性排序:

.. code-block:: console

    Fatal
    Error
    Warn
    Info
    Debug

虽然每个等级表示的含义没有严格的标准，但是一般这样假设是比较安全合理的:

* ``Fatal`` 消息表示系统将终止以保护自身免受损害。
* ``Error`` 消息表示重要问题，不一定会损坏系统，但是会阻止系统正常运行。
* ``Warn`` 消息表示意外活动或非理想结果，可能代表更深层次的问题，但不会直接损害功能。
* ``Info`` 消息表示事件和状态更新，作为系统正常运行的可视信息验证。
* ``Debug`` 消息详细描述系统执行的每个步骤。

默认等级是 ``Info``。
这意味着你只会看到默认等级和更严重的消息。

一般情况下，只有 ``Debug`` 消息被隐藏，因为它们是唯一比 ``Info`` 更不严重的等级。
例如，如果你将默认等级设置为 ``Warn``，你只会看到 ``Warn``、 ``Error`` 和 ``Fatal`` 级的消息。

3.1 设置 logger 的默认等级
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

你可以在第一次运行 ``/turtlesim`` 节点时使用重映射设置默认的 logger 等级。
在终端中输入以下命令:

.. code-block:: console

    ros2 run turtlesim turtlesim_node --ros-args --log-level WARN

现在你再次在 ``rqt_console`` 中看到的消息将只包含 ``Warn``、 ``Error`` 和 ``Fatal`` 级别的消息。
这是因为 ``Info`` 消息的优先级低于新的默认日志等级 ``Warn``。

总结
-------

`rqt_console` 可以帮助你仔细了解系统中的日志消息。
通常你会因为各种原因检查日志消息，通常是为了找出哪里出了问题以及导致问题的一系列有序事件。

下一步
----------

下一个教程将教你如何使用 :doc:`ROS 2 Launch <../Launching-Multiple-Nodes/Launching-Multiple-Nodes>` 一次启动多个节点。
