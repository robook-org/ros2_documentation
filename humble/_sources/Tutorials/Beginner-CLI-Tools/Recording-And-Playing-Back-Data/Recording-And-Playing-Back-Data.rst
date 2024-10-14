.. redirect-from::

    Tutorials/Ros2bag/Recording-And-Playing-Back-Data

.. _ROS2Bag:

记录和回放数据
===============================

**目标:** 记录发布在某个 topic 上的数据，以便后续回放数据或者检查数据。

**教程等级:** 初级

**预计时长:** 10 分钟

.. contents:: Contents
   :depth: 2
   :local:

背景
----------

``ros2 bag`` 是一个命令行工具，用于记录系统中发布在 topic 上的数据。
它会记录发布在任意 topic 上的数据，并将其保存在数据库中。
之后，你可以回放这些数据，以重现你的测试和实验结果。
记录 topic 也是分享你的工作和让其他人复现它的好方法。


前提条件
-------------

你应该已经安装 ``ros2 bag``，它是 ROS 2 的一部分。

如果你需要安装 ROS 2，请参阅 :doc:`安装指南 <../../../Installation>`。

本教程涉及到之前教程中的一些概念，比如 :doc:`nodes <../Understanding-ROS2-Nodes/Understanding-ROS2-Nodes>` 和 :doc:`topics <../Understanding-ROS2-Topics/Understanding-ROS2-Topics>`。
它还使用了 :doc:`turtlesim package <../Introducing-Turtlesim/Introducing-Turtlesim>`。

如往常一样，不要忘记在 :doc:`每次打开新终端时 <../Configuring-ROS2-Environment>` source ROS 2。


任务
-----

1 Setup
^^^^^^^

你将在 ``turtlesim`` 系统中记录键盘输入，以便稍后保存和回放，所以首先启动 ``/turtlesim`` 和 ``/teleop_turtle`` 节点。

打开一个新终端并运行:

.. code-block:: console

    ros2 run turtlesim turtlesim_node

打开另一个终端并运行:

.. code-block:: console

    ros2 run turtlesim turtle_teleop_key

让我们也创建一个新目录来存储我们保存的记录，这是一个好习惯:

.. tabs::

    .. group-tab:: Linux

        .. code-block:: console

            mkdir bag_files
            cd bag_files

    .. group-tab:: macOS

        .. code-block:: console

            mkdir bag_files
            cd bag_files

    .. group-tab:: Windows

        .. code-block:: console

            md bag_files
            cd bag_files


2 选择一个 topic
^^^^^^^^^^^^^^^^

``ros2 bag`` 只能记录发布在 topic 上的数据。
要查看系统中的 topic 列表，请打开一个新终端并运行以下命令:

.. code-block:: console

  ros2 topic list

这会返回：

.. code-block:: console

  /parameter_events
  /rosout
  /turtle1/cmd_vel
  /turtle1/color_sensor
  /turtle1/pose

在 topic 的教程中，你已经学了 ``/turtle_teleop`` 节点在 ``/turtle1/cmd_vel`` topic 上发布命令，以使乌龟在 turtlesim 窗口中移动。

要查看 ``/turtle1/cmd_vel`` 发布的数据，运行以下命令:

.. code-block:: console

  ros2 topic echo /turtle1/cmd_vel

一开始不会有任何显示，因为 teleop 没有发布数据。
返回到运行 teleop 的终端并选中它，使其处于激活状态。
使用箭头键移动乌龟，你会看到数据发布在运行 ``ros2 topic echo`` 的终端上。

.. code-block:: console

  linear:
    x: 2.0
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.0
    ---


3 ros2 bag record
^^^^^^^^^^^^^^^^^

3.1 记录某个单独的 topic
~~~~~~~~~~~~~~~~~~~~~~~~~

要记录发布在 topic 上的数据，使用以下命令:

.. code-block:: console

    ros2 bag record <topic_name>

在你选择的 topic 上运行这个命令之前，打开一个新终端并切换到你之前创建的 ``bag_files`` 目录，因为 rosbag 文件会保存在你运行命令的目录中。

运行以下命令:

.. code-block:: console

    ros2 bag record /turtle1/cmd_vel

你会在终端中看到以下消息（日期和时间不一样）:

.. code-block:: console

    [INFO] [rosbag2_storage]: Opened database 'rosbag2_2019_10_11-05_18_45'.
    [INFO] [rosbag2_transport]: Listening for topics...
    [INFO] [rosbag2_transport]: Subscribed to topic '/turtle1/cmd_vel'
    [INFO] [rosbag2_transport]: All requested topics are subscribed. Stopping discovery...

现在 ``ros2 bag`` 正在记录发布在 ``/turtle1/cmd_vel`` topic 上的数据。
返回到 teleop 终端并再次移动乌龟。
移动的方式不重要，但尽量是一种可识别的方式，以便稍后回放数据时能看出来。

.. image:: images/record.png

按 ``Ctrl+C`` 停止记录。

数据会保存在一个新的 bag 目录中，命名方式符合 ``rosbag2_year_month_day-hour_minute_second`` 。
这个目录会包含一个 ``metadata.yaml`` 文件和一个以记录格式保存的 bag 文件。

3.2 记录多个 topics
~~~~~~~~~~~~~~~~~~~~~~~~~~

你也可以记录多个 topics，以及更改 ``ros2 bag`` 保存的文件名。

运行以下命令:

.. code-block:: console

  ros2 bag record -o subset /turtle1/cmd_vel /turtle1/pose

``-o`` 选项允许你为你的 bag 文件选择一个唯一的名字。
在这个例子中，``subset`` 是文件名。

要记录多个 topic，只需用空格分隔每个 topic。

你会看到以下消息，确认两个 topic 都在被记录:

.. code-block:: console

  [INFO] [rosbag2_storage]: Opened database 'subset'.
  [INFO] [rosbag2_transport]: Listening for topics...
  [INFO] [rosbag2_transport]: Subscribed to topic '/turtle1/cmd_vel'
  [INFO] [rosbag2_transport]: Subscribed to topic '/turtle1/pose'
  [INFO] [rosbag2_transport]: All requested topics are subscribed. Stopping discovery...

你可以随意移动乌龟，觉得可以结束了就按 ``Ctrl+C``。

.. note::

    也可以在命令中添加另一个选项，``-a``，它会记录系统上的所有 topics。

4 ros2 bag info
^^^^^^^^^^^^^^^

你可以通过运行以下命令查看关于你的记录数据的详细信息:

.. code-block:: console

    ros2 bag info <bag_file_name>

在 ``subset`` bag 文件上运行这个命令会返回文件的信息列表:

.. code-block:: console

    ros2 bag info subset

.. code-block:: console

  Files:             subset.db3
  Bag size:          228.5 KiB
  Storage id:        sqlite3
  Duration:          48.47s
  Start:             Oct 11 2019 06:09:09.12 (1570799349.12)
  End                Oct 11 2019 06:09:57.60 (1570799397.60)
  Messages:          3013
  Topic information: Topic: /turtle1/cmd_vel | Type: geometry_msgs/msg/Twist | Count: 9 | Serialization Format: cdr
                   Topic: /turtle1/pose | Type: turtlesim/msg/Pose | Count: 3004 | Serialization Format: cdr

5 ros2 bag play
^^^^^^^^^^^^^^^

在回放 bag 文件之前，输入 ``Ctrl+C`` 停止 teleop 的运行。
然后确保你的 turtlesim 窗口是可见的，这样你就可以看到 bag 文件的运行。

要回放记录的数据，运行以下命令:

.. code-block:: console

    ros2 bag play subset

终端会返回消息:

.. code-block:: console

    [INFO] [rosbag2_storage]: Opened database 'subset'.

现在你的乌龟会按照你录制时的路径移动（虽然不是 100% 准确；turtlesim 对系统时间的小变化很敏感）。

.. image:: images/playback.png

因为 ``subset`` 文件记录的是 ``/turtle1/pose`` 这个 topic，所以只要你不退出 turtlesim， ``ros2 bag play`` 命令就会一直运行。

这是因为只要 ``/turtlesim`` 节点在运行，它就会在固定的时间间隔内（也就是以固定频率）发布数据到 ``/turtle1/pose`` topic 上。
你应该已经注意到，前面 ``ros2 bag info`` 返回的 ``/turtle1/pose`` topic 的 ``Count`` 值只有9.这是我们录制时按下方向键的次数。

不过注意， ``/turtle1/pose`` 的 ``Count`` 值是3000多；这意思是，在我们录制时，数据在这个 topic 上发布了3000多次。

用下面的指令可以查看位置数据发布的频率:

.. code-block:: console

    ros2 topic hz /turtle1/pose

总结
-------

你可以用 ``ros2 bag`` 命令记录 ROS 2 系统中发布在 topic 上的数据。
这是一个很好的工具，无论是与他人分享你的工作，还是自己回顾实验结果。

下一步
----------

你已经完成了 "初级: CLI 工具" 教程！
接下来是 "初级: 客户端库" 教程，从 :doc:`../../Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace` 开始。

相关内容
---------------

更多关于 ``ros2 bag`` 的详细解释可以在 `这个 README <https://github.com/ros2/rosbag2>`__ 找到.
有关 QoS 兼容性和 ``ros2 bag`` 的更多信息，请参阅 :doc:`../../../How-To-Guides/Overriding-QoS-Policies-For-Recording-And-Playback`。
