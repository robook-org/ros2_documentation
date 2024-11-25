.. redirect-from::

    Tutorials/Actions/Writing-a-Cpp-Action-Server-Client

.. _ActionsCpp:

实现 action 服务器和客户端(C++)
=========================================

**目标:** 在 C++ 中实现 action 服务器和客户端。

**教程等级:** 中级

**预计时长:** 15 分钟

.. contents:: Contents
   :depth: 2
   :local:

背景
----------

Actions 是 ROS 2 中的一种异步通信形式。
*Action 客户端* 发送目标请求到 *Action 服务器*。
*Action 服务器* 发送目标反馈和结果到 *Action 客户端*。

前提条件
-------------

你需要有 ``action_tutorials_interfaces`` 包和在之前的教程 :doc:`../Creating-an-Action` 中定义的 ``Fibonacci.action`` 接口。

任务
-----

1 创建 action_tutorials_cpp 包
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

正如我们在 :doc:`../../Beginner-Client-Libraries/Creating-Your-First-ROS2-Package` 教程中看到的，我们需要创建一个新的包来保存 C++ 和相关代码。

1.1 创建 action_tutorials_cpp 包
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

进入你在 :doc:`上一个教程 <../Creating-an-Action>` 中创建的 action 工作空间(记得 source 这个工作空间)，并为 C++ action 服务器创建一个新的包：


.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

      cd ~/ros2_ws/src
      ros2 pkg create --dependencies action_tutorials_interfaces rclcpp rclcpp_action rclcpp_components -- action_tutorials_cpp

  .. group-tab:: macOS

    .. code-block:: bash

      cd ~/ros2_ws/src
      ros2 pkg create --dependencies action_tutorials_interfaces rclcpp rclcpp_action rclcpp_components -- action_tutorials_cpp

  .. group-tab:: Windows

    .. code-block:: bash

      cd \dev\ros2_ws\src
      ros2 pkg create --dependencies action_tutorials_interfaces rclcpp rclcpp_action rclcpp_components -- action_tutorials_cpp

1.2 添加可见性控制
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

为了使包能够在 Windows 上编译和工作，我们需要添加一些 "可见性控制"。
有关更多详细信息，请参见 :ref:`Windows Tips and Tricks 文档中的 Windows Symbol Visibility <Windows_Symbol_Visibility>`。

打开 ``action_tutorials_cpp/include/action_tutorials_cpp/visibility_control.h``，并添加以下代码：

.. code-block:: c++

  #ifndef ACTION_TUTORIALS_CPP__VISIBILITY_CONTROL_H_
  #define ACTION_TUTORIALS_CPP__VISIBILITY_CONTROL_H_

  #ifdef __cplusplus
  extern "C"
  {
  #endif

  // This logic was borrowed (then namespaced) from the examples on the gcc wiki:
  //     https://gcc.gnu.org/wiki/Visibility

  #if defined _WIN32 || defined __CYGWIN__
    #ifdef __GNUC__
      #define ACTION_TUTORIALS_CPP_EXPORT __attribute__ ((dllexport))
      #define ACTION_TUTORIALS_CPP_IMPORT __attribute__ ((dllimport))
    #else
      #define ACTION_TUTORIALS_CPP_EXPORT __declspec(dllexport)
      #define ACTION_TUTORIALS_CPP_IMPORT __declspec(dllimport)
    #endif
    #ifdef ACTION_TUTORIALS_CPP_BUILDING_DLL
      #define ACTION_TUTORIALS_CPP_PUBLIC ACTION_TUTORIALS_CPP_EXPORT
    #else
      #define ACTION_TUTORIALS_CPP_PUBLIC ACTION_TUTORIALS_CPP_IMPORT
    #endif
    #define ACTION_TUTORIALS_CPP_PUBLIC_TYPE ACTION_TUTORIALS_CPP_PUBLIC
    #define ACTION_TUTORIALS_CPP_LOCAL
  #else
    #define ACTION_TUTORIALS_CPP_EXPORT __attribute__ ((visibility("default")))
    #define ACTION_TUTORIALS_CPP_IMPORT
    #if __GNUC__ >= 4
      #define ACTION_TUTORIALS_CPP_PUBLIC __attribute__ ((visibility("default")))
      #define ACTION_TUTORIALS_CPP_LOCAL  __attribute__ ((visibility("hidden")))
    #else
      #define ACTION_TUTORIALS_CPP_PUBLIC
      #define ACTION_TUTORIALS_CPP_LOCAL
    #endif
    #define ACTION_TUTORIALS_CPP_PUBLIC_TYPE
  #endif

  #ifdef __cplusplus
  }
  #endif

  #endif  // ACTION_TUTORIALS_CPP__VISIBILITY_CONTROL_H_

1.3 编写 action 服务器
^^^^^^^^^^^^^^^^^^^^^^^^^^

让我们专注实现一个计算斐波那契数列的 action 服务器，会用到在 :doc:`../Creating-an-Action` 教程中创建的 action 。

2.1 编写 action 服务器代码
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

打开 ``action_tutorials_cpp/src/fibonacci_action_server.cpp``，并添加以下代码：

.. literalinclude:: scripts/server.cpp
    :language: c++

前几行包含了我们编译所需的所有头文件。

接下来创建一个 ``rclcpp::Node`` 的子类：

.. literalinclude:: scripts/server.cpp
    :language: c++
    :lines: 14

``FibonacciActionServer`` 构造函数将节点名称初始化为 ``fibonacci_action_server``：

.. literalinclude:: scripts/server.cpp
    :language: c++
    :lines: 21-22

构造函数还实例化了一个新的 action 服务器：

.. literalinclude:: scripts/server.cpp
    :language: c++
    :lines: 26-31

一个 action 服务器需要 6 个信息：

1. action 类型名称： ``Fibonacci``。
2. 一个 action 所属的 ROS 2 节点： ``this``。
3. action 名称： ``'fibonacci'``。
4. 用于处理目标的回调函数： ``handle_goal``。
5. 用于处理取消的回调函数： ``handle_cancel``。
6. 用于处理目标接受的回调函数： ``handle_accept``。

接下来是文件中的几个回调的实现。
注意，所有回调都需要快速返回，否则可能会导致 executor 闲置。

我们从处理新目标的回调开始：

.. literalinclude:: scripts/server.cpp
    :language: c++
    :lines: 37-44

目前这个实现直接接受所有目标。

接下来是处理取消的回调：

.. literalinclude:: scripts/server.cpp
    :language: c++
    :lines: 46-52

目前这个实现只告诉客户端它接受了取消请求。

最后一个回调用于接受一个新目标并开始处理：

.. literalinclude:: scripts/server.cpp
    :language: c++
    :lines: 54-59

由于执行是一个长时间运行的操作，我们创建一个线程来执行实际的工作，并快速从 ``handle_accepted`` 返回。

所有进一步的处理和更新都在新线程的 ``execute`` 方法中完成：

.. literalinclude:: scripts/server.cpp
    :language: c++
    :lines: 61-95

这个工作线程每秒处理一个斐波那契数列的序列号，为每一步发布一个反馈更新。
当处理完成时，它将 ``goal_handle`` 标记为成功，并退出。

现在我们有了功能完整的 action 服务器。让我们构建并运行它。

2.2 构建 action 服务器
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

在上一节中，我们准备好了 action 服务器的代码。
我们还需要做一些额外的事情，才能构建和运行它。

首先，我们需要设置 CMakeLists.txt，来确保能够构建 action 服务器。
打开 ``action_tutorials_cpp/CMakeLists.txt``，并在 ``find_package`` 之后添加以下内容：

.. code-block:: cmake

  add_library(action_server SHARED
    src/fibonacci_action_server.cpp)
  target_include_directories(action_server PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_definitions(action_server
    PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
  ament_target_dependencies(action_server
    "action_tutorials_interfaces"
    "rclcpp"
    "rclcpp_action"
    "rclcpp_components")
  rclcpp_components_register_node(action_server PLUGIN "action_tutorials_cpp::FibonacciActionServer" EXECUTABLE fibonacci_action_server)
  install(TARGETS
    action_server
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

现在我们可以构建包了。进入 ``ros2_ws`` 的顶层目录，并运行：

.. code-block:: bash

  colcon build

这将构建整个工作空间，包括 ``action_tutorials_cpp`` 包中的 ``fibonacci_action_server``。

2.3 运行 action 服务器
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

现在已经有了编译好的 action 服务器，可以运行它了。
Source 刚刚构建的工作空间（``ros2_ws``），并运行 action 服务器：

.. code-block:: bash

  ros2 run action_tutorials_cpp fibonacci_action_server

3 编写 action 客户端
^^^^^^^^^^^^^^^^^^^^^^^^^^

3.1 编写 action 客户端代码
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

打开 ``action_tutorials_cpp/src/fibonacci_action_client.cpp``，并添加以下代码：

.. literalinclude:: scripts/client.cpp
    :language: c++

前几行包含了我们编译所需的所有头文件。

接下来创建一个 ``rclcpp::Node`` 的子类：

.. literalinclude:: scripts/client.cpp
    :language: c++
    :lines: 15

``FibonacciActionClient`` 构造函数将节点名称初始化为 ``fibonacci_action_client``：

.. literalinclude:: scripts/client.cpp
    :language: c++
    :lines: 20-22

构造函数还实例化了一个新的 action 客户端：

.. literalinclude:: scripts/client.cpp
    :language: c++
    :lines: 24-26

一个 action 客户端需要 3 个信息：

1. action 类型名称： ``Fibonacci``。
2. 一个 action 所属的 ROS 2 节点： ``this``。
3. action 名称： ``'fibonacci'``。

我们还实例化了一个 ROS 定时器，它将周期性地调用 ``send_goal`` ：

.. literalinclude:: scripts/client.cpp
    :language: c++
    :lines: 27-30

每当定时器到期触发，它都会调用 ``send_goal`` ：

.. literalinclude:: scripts/client.cpp
    :language: c++
    :lines: 32-57

这个函数做了以下事情：

1. 取消定时器（这样它只会被调用一次）。
2. 等待 action 服务器启动。
3. 实例化一个新的 ``Fibonacci::Goal``。
4. 设置响应、反馈以及结果的回调。
5. 发送目标到服务器。

当服务器接收并接受目标时，它会向客户端返回一个响应。
这个响应由 ``goal_response_callback`` 处理：

.. literalinclude:: scripts/client.cpp
    :language: c++
    :lines: 62-71

假设服务器接受了目标，它就会开始处理。
所有回到客户端的反馈都由 ``feedback_callback`` 处理：

.. literalinclude:: scripts/client.cpp
    :language: c++
    :lines: 72-83

服务器处理完成后，会向客户端返回一个结果。
这个结果由 ``result_callback`` 处理：

.. literalinclude:: scripts/client.cpp
    :language: c++
    :lines: 84-107

现在我们有了一个功能完全的 action 客户端。让我们构建并运行它。

3.2 构建 action 客户端
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

在上一节中，我们准备好了 action 客户端的代码。

我们还需要再做一些额外的事情，才能构建和运行它。

首先，我们需要设置 CMakeLists.txt，以便编译 action 客户端。
打开 ``action_tutorials_cpp/CMakeLists.txt``，并在 ``find_package`` 之后添加以下内容：

.. code-block:: cmake

  add_library(action_client SHARED
    src/fibonacci_action_client.cpp)
  target_include_directories(action_client PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_definitions(action_client
    PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
  ament_target_dependencies(action_client
    "action_tutorials_interfaces"
    "rclcpp"
    "rclcpp_action"
    "rclcpp_components")
  rclcpp_components_register_node(action_client PLUGIN "action_tutorials_cpp::FibonacciActionClient" EXECUTABLE fibonacci_action_client)
  install(TARGETS
    action_client
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

现在我们可以构建包了。进入 ``ros2_ws`` 的顶层目录，并运行：

.. code-block:: bash

  colcon build

这会构建整个工作空间，包括 ``action_tutorials_cpp`` 包中的 ``fibonacci_action_client``。

3.3 运行 action 客户端
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

现在我们已经构建了 action 客户端，可以运行它了。
首先确保在另一个终端中运行着一个 action 服务器。
现在 source 我们刚刚构建的工作空间（``ros2_ws``），并尝试运行 action 客户端：

.. code-block:: bash

  ros2 run action_tutorials_cpp fibonacci_action_client

应该能在终端中看到 action 客户端的输出，包括被接受的情况、反馈和结果。

总结
-------

在本教程中，你逐行编写了一个 C++ action 服务器和 action 客户端，并配置它们传递目标、反馈和结果。

相关内容
---------------

* 有多种方法可以在 C++ 中编写 action 服务器和客户端；请查看 `ros2/examples <https://github.com/ros2/examples/tree/{REPOS_FILE_BRANCH}/rclcpp>`_ 中的 ``minimal_action_server`` and ``minimal_action_client``.

* 有关 ROS actions 的更详细信息，请参阅 `design article <http://design.ros2.org/articles/actions.html>`__。
