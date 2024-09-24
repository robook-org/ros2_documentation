.. redirect-from::

    Tutorials/Turtlesim/Introducing-Turtlesim

.. _Turtlesim:

使用 ``turtlesim``, ``ros2``, ``rqt``
==========================================

**目标:** 安装和使用 ros2 命令行工具、turtlesim 包、 rqt 工具包，为后续教程作准备.

**教程等级:** 初级

**预计时长:** 15 分钟

.. contents:: Contents
   :depth: 2
   :local:

背景
----------

Turtlesim 用来学习 ROS 2 的轻量仿真器.
它从很基础的视角展示了 ROS 2 是如何工作的，让你对后续在真实机器人或仿真器上的操作有一个基本的了解.

ros2 工具是用户管理、查看以及与 ROS 系统交互的工具.
它包含多种针对不同组件及其操作的命令.
比如，你可以用它来启动一个节点、设置一个参数、监听一个 topic 等等.
ros2 工具已经 ROS 2 core 安装中包含了.


rqt 是一个有图形界面 (GUI) 的 ROS 2 工具.
rqt 可以做到的事情，使用命令行工具也可以做到，只是 rqt 提供了一个用户友好的使用界面。

本教程会涉及到一些 ROS 2 的核心概念， 例如 nodes, topics, and services.
这些概念将会在后续的教程中展开阐述，目前你只需要在跟随教程配置工具的时候尝试感受它们即可。

前提条件
-------------

上一个教程 :doc:`../Configuring-ROS2-Environment` 会展示如何配置环境.

任务
------

1 安装 turtlesim
^^^^^^^^^^^^^^^^^^^

和之前提到的一样，需要先在新终端中 source 配置脚本， :doc:`previous tutorial <../Configuring-ROS2-Environment>`.

安装你的 ROS 2 版本对应的 turtlesim 包:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        sudo apt update

        sudo apt install ros-{DISTRO}-turtlesim

   .. group-tab:: macOS

      As long as the archive you installed ROS 2 from contains the ``ros_tutorials`` repository, you should already have turtlesim installed.

   .. group-tab:: Windows

      As long as the archive you installed ROS 2 from contains the ``ros_tutorials`` repository, you should already have turtlesim installed.

检查安装已经成功:

.. code-block:: console

  ros2 pkg executables turtlesim

上述指令会返回一系列可用的 turtlesim 命令：

.. code-block:: console

  turtlesim draw_square
  turtlesim mimic
  turtlesim turtle_teleop_key
  turtlesim turtlesim_node

2 启动 turtlesim
^^^^^^^^^^^^^^^^^

在终端中输入以下指令启动:

.. code-block:: console

  ros2 run turtlesim turtlesim_node

仿真器窗口会弹出，中间会有一只随机皮肤的小乌龟.

.. image:: images/turtlesim.png

在终端中，你会看到节点发出的消息:

.. code-block:: console

  [INFO] [turtlesim]: Starting turtlesim with node name /turtlesim
  [INFO] [turtlesim]: Spawning turtle [turtle1] at x=[5.544445], y=[5.544445], theta=[0.000000]

消息包含默认小乌龟的名字和它生成的坐标.

3 使用 turtlesim
^^^^^^^^^^^^^^^^^

打开一个新终端并再次 source ROS 2 配置脚本.

现在你需要运行一个新节点来控制第一个节点中的小乌龟:

.. code-block:: console

  ros2 run turtlesim turtle_teleop_key

此时你应该有三个窗口打开：一个运行 ``turtlesim_node`` 的终端，一个运行 ``turtle_teleop_key`` 的终端，和一个显示小乌龟的窗口.
拖动这些窗口，让你可以看到小乌龟窗口，同时也能看到 ``turtle_teleop_key`` 终端，（译者注：并且在 ``turtle_teleop_key`` 终端里点击一下，使得焦点在这个窗口，）这样你就可以控制小乌龟了.

使用键盘上的方向键来控制小乌龟.
它会在屏幕上移动，并且(它身下会"绑定"一根笔)画出它走过的路径.

.. note::

  按下一个方向键只会让小乌龟移动一小段距离然后停下.
  毕竟你应该不希望一个机器人在操作员失去与机器人的连接时继续执行指令.（译者注：这个表现实际上是在模拟真实对机器人的控制，毕竟在真机上丢失控制指令就停止移动也是一个显而易见的简单保护策略。）

你可以使用 ``ros2 node list``, ``ros2 topic list``, ``ros2 service list``, ``ros2 action list`` 来查看节点和相关的 topics 、服务、actions.
（译者注： 你可以这样理解这些指令， ros2 是主指令， node topic service action 这些是组件，而主指令下某个组件的 list 是子指令。在过去或未来的学习中，你应该用过或者可以用这样的思路猜测、记忆、使用或者设计各种各样的命令行工具。）

.. code-block:: console

  ros2 node list
  ros2 topic list
  ros2 service list
  ros2 action list

在接下来的教程中，你会了解更多有关这些概念的知识.
本篇教程仅限于让你对 turtlesim 有一个大致的了解，让你能够使用 rqt 来调用一些 turtlesim 的服务，和 ``turtlesim_node`` 进行交互.

4 安装 rqt
^^^^^^^^^^^^^

打开一个新终端来安装 ``rqt`` 和相关插件:

.. tabs::

  .. group-tab:: Ubuntu Linux

    .. code-block:: console

      sudo apt update

      sudo apt install '~nros-{DISTRO}-rqt*'

  .. group-tab:: macOS

    The standard archive for installing ROS 2 on macOS contains ``rqt`` and its plugins, so you should already have ``rqt`` installed.

  .. group-tab:: Windows

    The standard archive for installing ROS 2 on Windows contains ``rqt`` and its plugins, so you should already have ``rqt`` installed.

使用如下指令运行 rqt:

.. code-block:: console

  rqt

5 使用 rqt
^^^^^^^^^^^

第一次运行 rqt 的时候，弹出的窗口是空白的.
不过没关系，你可以在顶部菜单栏选择 **Plugins** > **Services** > **Service Caller**.

.. note::

  rqt 可能需要一些时间来加载所有的插件.
  如果你点击 **Plugins** 但是没有看到 **Services** 或者其他选项，那么你应该关闭 rqt 并在终端中输入 ``rqt --force-discover``.

.. image:: images/rqt.png

使用 **Service** 下拉菜单左边的刷新按钮来确保你的 turtlesim 节点的所有服务都可用.

点击 **Service** 下拉菜单查看 turtlesim 的服务，选择 ``/spawn`` 服务.

5.1 尝试 spawn 服务
~~~~~~~~~~~~~~~~~~~~~~~~~~~

让我们使用 rqt 来调用 ``/spawn`` 服务.
你可以从服务的名字猜到， ``/spawn`` 会在 turtlesim 窗口中生成一个新的小乌龟.（译者注： spawn 是英文中“生成”的意思）

双击 **Expression** 列中的空白单引号中间，给新生成的小乌龟取一个独特的名字，比如 ``turtle2``.
你可以看到这个表达式对应的是 **name** 的值，是一个 **string** 类型的值.

接下来输入一些有效的坐标来生成新的小乌龟，比如 ``x = 1.0`` 和 ``y = 1.0``.

.. image:: images/spawn.png

.. note::

  如果你尝试生成一个已经存在的小乌龟，比如默认的 ``turtle1``，你会在运行 ``turtlesim_node`` 的终端中看到错误信息:

  .. code-block:: console

    [ERROR] [turtlesim]: A turtle named [turtle1] already exists

下一步点击 rqt 窗口右上角的 **Call** 按键来调用服务，这样就能生成 ``turtle2`` 了.

如果服务调用成功，你会在你输入的( **x** , **y** )坐标处看到一个新的小乌龟（也是随机皮肤）.

这时你刷新 rqt 中的服务列表的话，能看到除了 ``/turtle1/...`` 之外，还有新生成的与 ``/turtle2/...`` 相关的服务.

5.2 尝试使用 ``set_pen`` 服务
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

现在让我们使用 ``/set_pen`` 服务给 ``turtle1`` 设置一个不一样的笔（这样就能留下不一样的轨迹）:

.. image:: images/set_pen.png

``set_pen`` 服务有四个参数: **r**, **g**, **b**, **width**. 这些参数分别对应笔颜色的rgb分量和宽度.其中 **r**, **g**, **b** 的取值范围在 0 到 255 之间。

让我们尝试给 ``turtle1`` 设置一个红色的笔，使它留下红色的轨迹，把 **r** 的值设为 255， **width** 的值设为 5.
修改完参数之后别忘了点击 **Call** 按键来调用服务.

如果你返回到 ``turtle_teleop_key`` 运行的终端，按下方向键，你会看到 ``turtle1`` 的笔已经改变了.

.. image:: images/new_pen.png

你可能已经注意到 ``turtle2`` 是无法移动的.
这是因为 ``turtle2`` 没有对应的 ``turtle_teleop_key`` 遥控节点.

6 重映射
^^^^^^^^^^^

为了控制 ``turtle2``，你需要一个新的遥控节点.
不过如果你尝试运行之前的指令，你会发现这个遥控节点也会控制 ``turtle1``.这是因为 ``turtle_teleop_key`` 默认会发布到 ``cmd_vel`` topic.
所以如果想控制 ``turtle2``，你需要重映射 ``cmd_vel`` topic给 ``turtle2``.

在一个新的终端中 source ROS 2(译者注：后续我们可能不再详细描述 source 什么文件或者 source 是什么意思，你需要自己理解对应要 source 的文件，如果不懂可以先理解到此之前的教程)，然后运行:

.. code-block:: console

  ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel

现在，当鼠标的焦点在这个新终端的时候，你可以控制 ``turtle2``，当鼠标的焦点在之前的 ``turtle_teleop_key`` 终端的时候，你可以控制 ``turtle1``.

.. image:: images/remap.png

7 关闭 turtlesim
^^^^^^^^^^^^^^^^^

如果要停止仿真，你可以在 ``turtlesim_node`` 终端中输入 ``Ctrl + C``，在 ``turtle_teleop_key`` 终端中输入 ``q``.(译者注： ``Ctrl + C`` 是 Linux 下终止正在运行的程序的快捷键。而一般程序的设计者如果设计了退出的触发按键，那就很有可能是 ``q``，表示 quit。)

总结
-------

使用 turtlesim 和 rqt 是学习 ROS 2 核心概念的好方法.

下一步
----------

现在你已经安装了 turtlesim 和 rqt，并且对它们的工作原理有了一定的了解，那么接下来就可以开始学习 ROS 2 的第一个核心概念了， :doc:`../Understanding-ROS2-Nodes/Understanding-ROS2-Nodes`.

相关内容
---------------

turtlesim 包可以在 `ros_tutorials <https://github.com/ros/ros_tutorials/tree/{REPOS_FILE_BRANCH}/turtlesim>`_ 这个教程仓库里找到.

`这些社区维护的视频 <https://youtu.be/xwT7XWflMdc>`_ 也演示了许多有关这个教程里的东西.
