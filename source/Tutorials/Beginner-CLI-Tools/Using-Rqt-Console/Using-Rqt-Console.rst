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

In the middle you have the option to filter messages by excluding severity levels.
You can also add more exclusion filters using the plus-sign button to the right.

The bottom section is for highlighting messages that include a string you input.
You can add more filters to this section as well.

Now start ``turtlesim`` in a new terminal with the following command:

.. code-block:: console

    ros2 run turtlesim turtlesim_node

2 Messages on rqt_console
^^^^^^^^^^^^^^^^^^^^^^^^^

To produce log messages for ``rqt_console`` to display, let's have the turtle run into the wall.
In a new terminal, enter the ``ros2 topic pub`` command (discussed in detail in the :doc:`topics tutorial <../Understanding-ROS2-Topics/Understanding-ROS2-Topics>`) below:

.. code-block:: console

    ros2 topic pub -r 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}"

Since the above command is publishing the topic at a steady rate, the turtle is continuously running into the wall.
In ``rqt_console`` you will see the same message with the ``Warn`` severity level displayed over and over, like so:

.. image:: images/warn.png

Press ``Ctrl+C`` in the terminal where you ran the ``ros2 topic pub`` command to stop your turtle from running into the wall.

3 Logger levels
^^^^^^^^^^^^^^^

ROS 2's logger levels are ordered by severity:

.. code-block:: console

    Fatal
    Error
    Warn
    Info
    Debug

There is no exact standard for what each level indicates, but it's safe to assume that:

* ``Fatal`` messages indicate the system is going to terminate to try to protect itself from detriment.
* ``Error`` messages indicate significant issues that won't necessarily damage the system, but are preventing it from functioning properly.
* ``Warn`` messages indicate unexpected activity or non-ideal results that might represent a deeper issue, but don't harm functionality outright.
* ``Info`` messages indicate event and status updates that serve as a visual verification that the system is running as expected.
* ``Debug`` messages detail the entire step-by-step process of the system execution.

The default level is ``Info``.
You will only see messages of the default severity level and more-severe levels.

Normally, only ``Debug`` messages are hidden because they're the only level less severe than ``Info``.
For example, if you set the default level to ``Warn``, you would only see messages of severity ``Warn``, ``Error``, and ``Fatal``.

3.1 Set the default logger level
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

You can set the default logger level when you first run the ``/turtlesim`` node using remapping.
Enter the following command in your terminal:

.. code-block:: console

    ros2 run turtlesim turtlesim_node --ros-args --log-level WARN

Now you won't see the initial ``Info`` level messages that came up in the console last time you started ``turtlesim``.
That's because ``Info`` messages are lower priority than the new default severity, ``Warn``.

总结
-------

``rqt_console`` can be very helpful if you need to closely examine the log messages from your system.
You might want to examine log messages for any number of reasons, usually to find out where something went wrong and the series of events leading up to that.

下一步
----------

The next tutorial will teach you about starting multiple nodes at once with :doc:`ROS 2 Launch <../Launching-Multiple-Nodes/Launching-Multiple-Nodes>`.
