.. redirect-from::

    Introspection-with-command-line-tools
    Tutorials/Introspection-with-command-line-tools
    Concepts/About-Command-Line-Tools

使用命令行工具解析数据(Introspection)
===========================================

.. contents:: Table of Contents
   :local:

ROS 2 包含一套用于与 ROS 2 系统交互的命令行工具。

译者注：此处“与 ROS 2 系统交互”翻译自“introspecting a ROS 2 system”。尽管“introspecting” 一词在某些领域中被翻译为“内省”，但是译者认为已有的翻译仅仅是使用中文汉字替代原词，并未传达出这个词汇的实际含义。至少对于译者而言，“自省某个系统”是一种完全不知所云的说法。
考虑到本页面的内容和情景，译者认为“解析数据”、“与系统交互”更能表达这个词汇在中文中实际对应的操作。后续你将进一步理解“introspect”具体是在做什么。

使用
-----

工具的主要入口是 ``ros2`` 命令，它本身有各种子命令，用于解析和处理节点、topic、service等。

运行如下命令，可以看到所有可用的子命令：

.. code-block:: bash

   ros2 --help

一些可用的子命令包括：

* ``action``: 与 ROS actions 交互
* ``bag``: 录制或播放 rosbag
* ``component``: 管理 component containers
* ``daemon``: 与 ROS 2 daemon 交互
* ``doctor``: 检查 ROS 配置中潜在的问题
* ``interface``: 列出 ROS interfaces 和相关信息
* ``launch``: 运行/检查 launch file
* ``lifecycle``: 通过 lifecycles 管理 nodes
* ``multicast``: Multicast debugging commands
* ``node``: 与 ROS nodes 交互
* ``param``: 配置 node 的参数
* ``pkg``: 与 ROS packages 交互
* ``run``: 运行 ROS nodes
* ``security``: 配置 security 有关的参数
* ``service``: 与 ROS services 交互
* ``test``: 运行 ROS launch test
* ``topic``: 与 ROS topics 交互
* ``trace``: 在 ROS nodes 运行期间追踪运行情况的工具(tracing tool) (仅在 Linux 上可用)
* ``wtf``: ``doctor`` 的别名

样例
-------

使用 ``topic`` 子命令可以用于在一个 topic 上发布和显示消息，这样可以生成典型的 talker-listener 示例的 talker端。

在一个终端中发布消息：

.. code-block:: bash

   $ ros2 topic pub /chatter std_msgs/msg/String "data: Hello world"
   publisher: beginning loop
   publishing #1: std_msgs.msg.String(data='Hello world')

   publishing #2: std_msgs.msg.String(data='Hello world')

在另一个终端中显示消息，用来充当 listener 端：

.. code-block:: bash

   $ ros2 topic echo /chatter
   data: Hello world

   data: Hello world

样例之下的细节
-----------------

ROS 2 使用分布式发现的方式(distributed discovery process)来使节点相互连接。
这个过程特意不使用中心化的发现机制，因此 ROS 节点发现 ROS graph 中的其他参与者可能需要一些时间。
因此，后台有一个长时间运行的守护进程(daemon)，用于存储有关 ROS graph 的信息，以提供更快的查询响应，例如节点名称列表。

当第一次使用相关命令行工具时，守护进程会自动启动。
你可以运行 ``ros2 daemon --help`` 以获取与守护进程交互的更多选项。

实现(Implementation)
------------------------

``ros2`` 命令的源码位于 https://github.com/ros2/ros2cli.

``ros2`` 命令已经实现为一个可以通过插件扩展的框架。
例如，如果安装了 `sros2  <https://github.com/ros2/sros2>`__ 包，那么 ``ros2`` 工具会自动检测到 ``sros2`` 包提供的 ``security`` 子命令。
