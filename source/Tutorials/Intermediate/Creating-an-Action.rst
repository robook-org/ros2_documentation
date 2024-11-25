.. redirect-from::

    Tutorials/Actions/Creating-an-Action

.. _ActionCreate:

创建 action
==================

**目标:** 在 ROS 2 包中定义 action .

**教程等级:** 中级

**预计时长:** 5 分钟

.. contents:: Contents
   :depth: 2
   :local:

背景
----------

你已经在之前的 :doc:`../Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions` 教程中了解了 action。
与其他通信类型及其各自的接口（topics/msg 和 services/srv）一样，
你也可以在你的包中自定义 action。
本教程将向你展示如何定义和构建一个 action，
以便在下一个教程中编写的 action 服务器和 action 客户端中使用。

前提条件
-------------

你需要已经安装了 :doc:`ROS 2 <../../Installation>` 和 `colcon <https://colcon.readthedocs.org>`__。

配置一个 :doc:`工作空间 <../Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace>` 并创建一个名为 ``action_tutorials_interfaces`` 的包：

(记得先 :doc:`source ROS 2 安装环境 <../Beginner-CLI-Tools/Configuring-ROS2-Environment>`。)

.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

      mkdir -p ros2_ws/src #you can reuse existing workspace with this naming convention
      cd ros2_ws/src
      ros2 pkg create action_tutorials_interfaces

  .. group-tab:: macOS

    .. code-block:: bash

      mkdir -p ros2_ws/src
      cd ros2_ws/src
      ros2 pkg create action_tutorials_interfaces

  .. group-tab:: Windows

    .. code-block:: bash

      md ros2_ws\src
      cd ros2_ws\src
      ros2 pkg create action_tutorials_interfaces

任务
-----

1 定义 action
^^^^^^^^^^^^^^^^^^^^

Action 是在 ``.action`` 文件中以这种形式定义的：

.. code-block:: bash

    # Request 请求
    ---
    # Result 结果
    ---
    # Feedback 反馈

一个 action 定义由三部份消息定义组成，用 ``---`` 分隔。


- 从 action 客户端发送到 action 服务器的 *请求* 消息，初始化一个新的目标。
- 当目标完成时，从 action 服务器发送到 action 客户端的 *结果* 消息。
- *反馈* 消息是周期性地从 action 服务器发送到 action 客户端的消息，用于更新目标。

一个 action 的实例通常被称为 *目标* (goal)。

假设我们想要为计算 `斐波那契数列 <https://en.wikipedia.org/wiki/Fibonacci_number>`__ 定义一个新的 action "Fibonacci"。

在 ROS 2 包 ``action_tutorials_interfaces`` 中创建一个 ``action`` 目录：

.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

      cd action_tutorials_interfaces
      mkdir action

  .. group-tab:: macOS

    .. code-block:: bash

      cd action_tutorials_interfaces
      mkdir action

  .. group-tab:: Windows

    .. code-block:: bash

      cd action_tutorials_interfaces
      md action

在 ``action`` 目录中创建一个名为 ``Fibonacci.action`` 的文件，内容如下：

.. code-block:: console

  int32 order
  ---
  int32[] sequence
  ---
  int32[] partial_sequence

目标请求是我们要计算的斐波那契数列的项数 ``order``，结果是最终算出来的数列 ``sequence``，反馈是到目前为止计算出来的数列 ``partial_sequence``。

2 构建 action
^^^^^^^^^^^^^^^^^^^^

在我们的代码中使用新的 Fibonacci action 类型之前，我们必须将定义传递给 rosidl 代码生成的流程。

这可以通过在我们的 ``CMakeLists.txt`` 文件中添加以下行来完成，这些行应该在 ``action_tutorials_interfaces`` 中的 ``ament_package()`` 行之前:

.. code-block:: cmake

    find_package(rosidl_default_generators REQUIRED)

    rosidl_generate_interfaces(${PROJECT_NAME}
      "action/Fibonacci.action"
    )

还需要在 ``package.xml`` 中添加所需的依赖项：

.. code-block:: xml

    <buildtool_depend>rosidl_default_generators</buildtool_depend>

    <depend>action_msgs</depend>

    <member_of_group>rosidl_interface_packages</member_of_group>

请注意，我们需要依赖于 ``action_msgs``，因为 action 定义依赖了一些额外的元数据（例如目标 ID）。

现在可以构建包含 ``Fibonacci`` action 定义的包了：

.. code-block:: bash

    # Change to the root of the workspace
    cd ~/ros2_ws
    # Build
    colcon build

很好！我们完成了！

按照惯例，action 类型会以其包名和 ``action`` 作为前缀。
因此，当我们想要引用我们的新 action 时，它需要有完整的名称 ``action_tutorials_interfaces/action/Fibonacci``。

我们可以使用命令行工具检查 action 是否构建成功：


.. code-block:: bash

   # Source our workspace
   # On Windows: call install/setup.bat
   . install/setup.bash
   # Check that our action definition exists
   ros2 interface show action_tutorials_interfaces/action/Fibonacci


应该能看到 Fibonacci action 的定义打印到屏幕上。

总结
-------

在本教程中，你学习了 action 定义的结构。
你还学习了如何使用 ``CMakeLists.txt`` 和 ``package.xml`` 正确构建新的 action 接口，
以及如何验证构建是否成功。

下一步
----------

接下来，让我们使用你新定义的 action 接口创建一个 action 服务和客户端（在 :doc:`Python <Writing-an-Action-Server-Client/Py>` 或 :doc:`C++ <Writing-an-Action-Server-Client/Cpp>` 中）。

相关内容
---------------

更多关于 ROS action 的详细信息，请参考 `设计文档 <http://design.ros2.org/articles/actions.html>`__。
