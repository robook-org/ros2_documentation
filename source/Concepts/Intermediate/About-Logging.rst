.. redirect-from::

    Logging
    Concepts/About-Logging

日志系统配置
================================

.. contents:: Table of Contents
   :local:

概览
--------

ROS 2 中的日志系统旨在将日志消息传递到各种目标，包括：

* 控制台（如果连接了控制台）
* 磁盘上的日志文件（如果本地存储可用）
* ROS 2 网络上的 ``/rosout`` 话题

默认情况下，ROS 2 节点中的日志消息将发送到控制台（stderr）、磁盘上的日志文件以及 ROS 2 网络上的 ``/rosout`` 话题。
所有这些目标都可以在每个节点上单独启用或禁用。

本文档的其余部分将介绍日志系统背后的一些思想。

日志等级
--------------------------

日志消息与其严重程度(severity level)相关联： ``DEBUG``、 ``INFO``、 ``WARN``、 ``ERROR`` 或 ``FATAL``，按升序排列。

记录器(logger)只会处理严重程度高于或等于为其选择的特定级别的日志消息。

每个节点都有一个与之关联的 logger，该 logger 会自动包含节点的名称和命名空间。
如果节点的名称在外部重新映射为源代码中定义的名称之外的其他名称，这个变化也会影响并体现在 logger 名称中。
Logger 也可以使用特定名称在不使用节点的情况下创建。

Logger 的名称会表明一种层次结构。
例如，如果有一个名为 ``abc.def`` 的 logger，那么它的父 logger 就是 ``abc``。如果 ``abc`` 的日志等级未设置，那么它将使用到其父级 ``abc`` 的日志等级，如果这个也没有设置，则将使用默认的 logger 等级。
当更改 logger ``abc`` 的级别时，所有子 logger（例如 ``abc.def``、 ``abc.ghi.jkl``）的日志等级都会受到影响，除非它们的级别已经被显式设置。

APIs
----

这些是 ROS 2 日志系统的终端用户应使用的 API。（按客户端库分组）

.. tabs::

  .. group-tab:: C++

    * ``RCLCPP_{DEBUG,INFO,WARN,ERROR,FATAL}`` - 每次运行到此行时输出给定的 printf 风格消息
    * ``RCLCPP_{DEBUG,INFO,WARN,ERROR,FATAL}_ONCE`` - 仅在第一次运行到此行时输出给定的消息
    * ``RCLCPP_{DEBUG,INFO,WARN,ERROR,FATAL}_EXPRESSION`` - 仅在给定表达式为 true 时输出给定的 printf 风格消息
    * ``RCLCPP_{DEBUG,INFO,WARN,ERROR,FATAL}_FUNCTION`` - 仅在给定函数返回 true 时输出给定的 printf 风格消息
    * ``RCLCPP_{DEBUG,INFO,WARN,ERROR,FATAL}_SKIPFIRST`` - 每次运行到此行时输出给定的 printf 风格消息，除了第一次
    * ``RCLCPP_{DEBUG,INFO,WARN,ERROR,FATAL}_THROTTLE`` - 每次运行到此行时输出给定的 printf 风格消息，但不超过给定的频率，使用毫秒为周期单位
    * ``RCLCPP_{DEBUG,INFO,WARN,ERROR,FATAL}_SKIPFIRST_THROTTLE`` - 每次运行到此行时输出给定的 printf 风格消息，但不超过给定的频率，使用毫秒为周期单位，但跳过第一次
    * ``RCLCPP_{DEBUG,INFO,WARN,ERROR,FATAL}_STREAM`` - 每次运行到此行时输出给定的 C++ stream-style 消息
    * ``RCLCPP_{DEBUG,INFO,WARN,ERROR,FATAL}_STREAM_ONCE`` - 仅在第一次运行到此行时输出给定的 C++ stream-style 消息
    * ``RCLCPP_{DEBUG,INFO,WARN,ERROR,FATAL}_STREAM_EXPRESSION`` - 仅在给定表达式为 true 时输出给定的 C++ stream-style 消息
    * ``RCLCPP_{DEBUG,INFO,WARN,ERROR,FATAL}_STREAM_FUNCTION`` - 仅在给定函数返回 true 时输出给定的 C++ stream-style 消息
    * ``RCLCPP_{DEBUG,INFO,WARN,ERROR,FATAL}_STREAM_SKIPFIRST`` - 每次运行到此行时输出给定的 C++ stream-style 消息，除了第一次
    * ``RCLCPP_{DEBUG,INFO,WARN,ERROR,FATAL}_STREAM_THROTTLE`` - 每次运行到此行时输出给定的 C++ stream-style 消息，但不超过给定的频率，使用毫秒为周期单位
    * ``RCLCPP_{DEBUG,INFO,WARN,ERROR,FATAL}_STREAM_SKIPFIRST_THROTTLE`` - 每次运行到此行时输出给定的 C++ stream-style 消息，但不超过给定的频率，使用毫秒为周期单位，但跳过第一次

    上述每个 API 的第一个参数都是 ``rclcpp::Logger`` 对象。
    这可以通过调用 ``node->get_logger()``（推荐）从节点 API 中提取，也可以通过构造一个独立的 ``rclcpp::Logger`` 对象来获取。

    * ``rcutils_logging_set_logger_level`` - 将指定 logger 的名称的日志等级设置为给定的等级
    * ``rcutils_logging_get_logger_effective_level`` - 给定一个 logger 名称，返回 logger 等级（可能未设置）

  .. group-tab:: Python

    * ``logger.{debug,info,warning,error,fatal}`` - 输出给定的 Python 字符串到日志系统。这些调用接受以下关键字参数来控制行为：

      * ``throttle_duration_sec`` - 如果不是 None，则频率上限对应的周期为此值（以浮点数的秒为单位）
      * ``skip_first`` - 如果为 True，则输出消息除了第一次之外的所有时间
      * ``once`` - 如果为 True，则仅在第一次运行到此行时输出消息

    * ``rclpy.logging.set_logger_level`` - 将指定 logger 名称的日志等级设置为给定的等级
    * ``rclpy.logging.get_logger_effective_level`` - 给定一个 logger 名称，返回 logger 等级（可能未设置）

配置
-------------

由于 ``rclcpp`` 和 ``rclpy`` 使用相同的日志底层，因此配置选项是相同的。

环境变量
^^^^^^^^^^^^^^^^^^^^^

以下环境变量控制 ROS 2 日志 logger 的表现。
对于每个环境设置，请注意这是一个进程范围的设置，因此它适用于该进程中的所有节点。

* ``ROS_LOG_DIR`` - 控制用于将日志消息写入磁盘的日志目录（如果启用）。 如果非空，则使用此变量中指定的确切目录。 如果为空，则使用 ``ROS_HOME`` 环境变量的内容构造一个路径，形式为 ``$ROS_HOME/.log``。 在任何情况下，``~`` 字符会都会扩展为用户的 HOME 目录。
* ``ROS_HOME`` - 控制用于各种 ROS 文件的主目录，包括日志和配置文件。 在日志记录的上下文中，此变量用于构造日志文件目录的路径。 如果非空，则使用此变量的内容作为 ROS_HOME 路径。 在所有情况下，``~`` 字符会都会扩展为用户的 HOME 目录。
* ``RCUTILS_LOGGING_USE_STDOUT`` - 控制输出消息的流去向。 如果未设置或为 0，则使用 stderr。 如果为 1，则使用 stdout。
* ``RCUTILS_LOGGING_BUFFERED_STREAM`` - 控制日志流（如在 ``RCUTILS_LOGGING_USE_STDOUT`` 中配置的）是否应该是行缓冲还是无缓冲。 如果未设置，则使用流的默认值（通常为 stdout 为行缓冲，stderr 为无缓冲）。 如果为 0，则强制流为无缓冲。 如果为 1，则强制流为行缓冲。
* ``RCUTILS_COLORIZED_OUTPUT`` - 控制输出消息时是否使用颜色。 如果未设置，则根据平台和控制台是否为 TTY 自动确定。 如果为 0，则强制禁用输出颜色。 如果为 1，则强制启用输出颜色。
* ``RCUTILS_CONSOLE_OUTPUT_FORMAT`` - 控制输出每个日志消息的字段。 可用字段有：

  * ``{severity}`` - 严重程度。
  * ``{name}`` - 记录器的名称（可能为空）。
  * ``{message}`` - 日志消息（可能为空）。
  * ``{function_name}`` - 调用此函数的函数名（可能为空）。
  * ``{file_name}`` - 调用此函数的文件名（可能为空）。
  * ``{time}`` - unix time,以秒为单位。
  * ``{time_as_nanoseconds}`` - unix time,以纳秒为单位。
  * ``{line_number}`` - 调用此函数的行号（可能为空）。

  如果未给出格式，则使用默认格式 ``[{severity}] [{time}] [{name}]: {message}``。


创建节点时的配置
^^^^^^^^^^^^^^^^^^^^^

在初始化 ROS 2 节点时，可以通过节点选项控制某些行为。
由于这些是每个节点的选项，即使将节点组合到单个进程中，也可以为不同的节点设置不同的选项。

* ``log_level`` - 用于在此特定节点中使用的组件的日志级别。 可以使用以下方式设置： ``ros2 run demo_nodes_cpp talker --ros-args --log-level talker:=DEBUG``
* ``external_log_config_file`` - 用于配置后端 logger 的外部文件。 如果为 NULL，则将使用默认配置。 请注意，此文件的格式是特定于后端的（对于 spdlog 的默认后端记录器目前未实现）。 可以使用以下方式设置： ``ros2 run demo_nodes_cpp talker --ros-args --log-config-file log-config.txt``
* ``log_stdout_disabled`` - 是否禁用将日志消息写入控制台。 可以使用以下方式设置： ``ros2 run demo_nodes_cpp talker --ros-args --disable-stdout-logs``
* ``log_rosout_disabled`` - 是否禁用将日志消息写入 ``/rosout``。 这可以显著节省网络带宽，但外部观察者将无法监视日志。 可以使用以下方式设置： ``ros2 run demo_nodes_cpp talker --ros-args --disable-rosout-logs``
* ``log_ext_lib_disabled`` - 是否完全禁用外部记录器的使用。 在某些情况下可能更快，但意味着日志不会写入磁盘。 可以使用以下方式设置： ``ros2 run demo_nodes_cpp talker --ros-args --disable-external-lib-logs``

日志系统设计
------------------------

下图显示了日志系统的五个主要部分以及它们之间的交互。

.. figure:: ../images/ros2_logging_architecture.png
   :alt: ROS 2 logging architecture
   :width: 550px
   :align: center

rcutils
^^^^^^^

``rcutils`` 具有一个日志实现，可以根据特定格式（请参见上面的 ``Configuration``）格式化日志消息，并将这些日志消息输出到控制台。
``rcutils`` 实现了一个完整的日志解决方案，但允许更高级别的组件以依赖注入模型插入到日志系统中。
这将在下面讨论 ``rcl`` 层时变得更加明显。

请注意，这是一个*每进程*的日志实现，因此在此级别配置的任何内容都将影响整个进程，而不仅仅是单个节点。

rcl_logging_spdlog
^^^^^^^^^^^^^^^^^^

``rcl_logging_spdlog`` 实现了 ``rcl_logging_interface`` API，因此为 ``rcl`` 层提供外部日志服务。
特别是，``rcl_logging_spdlog`` 实现接受格式化的日志消息，并使用 ``spdlog`` 库将其写入磁盘上的日志文件，通常在 ``~/.ros/log`` 中（尽管这是可配置的；请参见上面的 ``Configuration``）。

rcl
^^^

``rcl`` 中的日志子系统使用 ``rcutils`` 和 ``rcl_logging_spdlog`` 提供大部分 ROS 2 日志服务。
当日志消息到达时，``rcl`` 决定将其发送到哪里。
有三个主要地方可以传递日志消息；一个单独的节点可能启用其中任何组合：

* 通过 ``rcutils`` 层到控制台
* 通过 ``rcl_logging_spdlog`` 层到磁盘
* 通过 RMW 层到 ROS 2 网络上的 ``/rosout`` 话题

rclcpp
^^^^^^

这是主要的 ROS 2 C++ API，位于 ``rcl`` API 之上。
在 logging 的 context 中，``rclcpp`` 提供 ``RCLCPP_`` 日志宏；请参见上面的 ``APIs`` 以获取完整列表。
当 ``RCLCPP_`` 宏之一运行时，它会检查节点的当前日志等级与使用的宏的等级之间的关系。
如果宏的等级高于或等于节点的日志等级，则消息将被格式化并输出到当前配置的所有地方。
不过要注意，``rclcpp`` 使用全局互斥锁来处理日志调用，因此同一进程中的所有日志调用最终都是单线程的。

rclpy
^^^^^

这是主要的 ROS 2 Python API，位于 ``rcl`` API 之上。
在 logging 的 context 中，``rclpy`` 提供 ``logger.debug``-style 函数；请参见上面的 ``APIs`` 以获取完整列表。
当 ``logger.debug`` 函数之一运行时，它会检查节点的当前日志等级与使用的宏的等级之间的关系。
如果宏的等级高于或等于节点的日志等级，则消息将被格式化并输出到当前配置的所有地方。

使用 Logging
-------------

.. tabs::

  .. group-tab:: C++

    * See the `rclcpp logging demo <https://github.com/ros2/demos/tree/{REPOS_FILE_BRANCH}/logging_demo>`_ for some simple examples.
    * See the :doc:`logging demo <../../Tutorials/Demos/Logging-and-logger-configuration>` for example usage.
    * See the `rclcpp documentation <https://docs.ros2.org/latest/api/rclcpp/logging_8hpp.html>`__ for an extensive list of functionality.

  .. group-tab:: Python

    * See the `rclpy examples <https://github.com/ros2/examples/blob/{REPOS_FILE_BRANCH}/rclpy/services/minimal_client/examples_rclpy_minimal_client/client.py>`__ for example usage of a node's logger.
    * See the `rclpy tests <https://github.com/ros2/rclpy/blob/{REPOS_FILE_BRANCH}/rclpy/test/test_logging.py>`__ for example usage of keyword arguments (e.g. ``skip_first``, ``once``).
