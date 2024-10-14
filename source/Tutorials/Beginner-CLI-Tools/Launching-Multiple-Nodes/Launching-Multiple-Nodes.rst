.. redirect-from::

    Tutorials/Launch/CLI-Intro

.. _ROS2Launch:

启动多个节点
===============

**目标:** 使用命令行工具一次启动多个节点.

**教程等级:** 初级

**预计时长:** 5 分钟

.. contents:: Contents
   :depth: 2
   :local:

背景
----------

在大多数入门教程中，你一直在为每个新节点打开一个新终端。
当你创建更复杂的系统，有越来越多的节点同时运行时，打开终端并重新输入配置的过程会变得更繁琐。

ROS 2 的 launch files 允许你同时启动和配置多个包含 ROS 2 节点的可执行文件。

使用 ``ros2 launch`` 命令运行单个 launch 文件将一次启动整个系统 - 包括所有节点及其配置。

前提条件
-------------

在开始这些教程之前，请按照 ROS 2 :doc:`../../../Installation/` 页面上的说明安装 ROS 2。

本教程中使用的命令假定你按照操作系统的二进制包安装指南（Linux 的 deb 包）进行了安装。
如果你是从源代码构建的，仍然可以跟着做，但是你的设置文件路径可能会有所不同。
如果你从源代码构建，也无法使用 ``sudo apt install ros-<distro>-<package>`` 命令（在初级教程中经常使用）。

如果你使用的是 Linux 但是还不熟悉 shell，`这个教程 <https://www.linux.com/training-tutorials/bash-101-working-cli/>`__ 会有所帮助。

任务
-----

运行一个 Launch File
^^^^^^^^^^^^^^^^^^^^^

在新终端中运行如下指令：

.. code-block:: console

   ros2 launch turtlesim multisim.launch.py

这个指令会运行下面的 launch file：

.. code-block:: python

   # turtlesim/launch/multisim.launch.py

   from launch import LaunchDescription
   import launch_ros.actions

   def generate_launch_description():
       return LaunchDescription([
           launch_ros.actions.Node(
               namespace= "turtlesim1", package='turtlesim', executable='turtlesim_node', output='screen'),
           launch_ros.actions.Node(
               namespace= "turtlesim2", package='turtlesim', executable='turtlesim_node', output='screen'),
       ])

.. note::

   上面的 launch file 是用 Python 写的，但你也可以使用 XML 和 YAML 来创建 launch file。
   你可以在 :doc:`../../../How-To-Guides/Launch-file-different-formats` 中看到这些不同的 ROS 2 launch 格式的比较。

这会运行两个 turtlesim 节点：

.. image:: images/turtlesim_multisim.png

现在，不用担心看不懂这个 launch file 的内容。
你可以在 :doc:`ROS 2 launch 教程 <../../Intermediate/Launch/Launch-Main>` 中找到更多关于 ROS 2 launch 的信息。

（可选）控制 Turtlesim 节点
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

现在这些节点正在运行，你可以像控制其他 ROS 2 节点一样控制它们。
例如，你可以打开两个额外的终端，运行以下命令让两只乌龟朝相反的方向行驶：

在第二个终端中：

.. code-block:: console

   ros2 topic pub  /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"

在第三个终端中：

.. code-block:: console

   ros2 topic pub  /turtlesim2/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}"

运行这两个命令后，应该看到类似下面的画面：

.. image:: images/turtlesim_multisim_spin.png

总结
-------

现在你做到了一件很重要的事情，用一个命令运行了两个 turtlesim 节点。
一旦你学会写自己的 launch file，你就可以用 ``ros2 launch`` 命令以类似的方式运行多个节点 - 并设置它们的配置。

更多有关 ROS 2 launch file 的教程，请查看 :doc:`../../Intermediate/Launch/Launch-Main`。

下一步
----------

在下一个教程中， :doc:`../Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data`，你将学习另一个有用的工具，``ros2 bag``。
