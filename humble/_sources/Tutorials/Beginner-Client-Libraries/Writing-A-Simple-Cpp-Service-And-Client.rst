.. redirect-from::

    Tutorials/Writing-A-Simple-Cpp-Service-And-Client

.. _CppSrvCli:

服务与客户端(Service and Client)(C++ 实现)
===============================================

**目标:** 使用 C++ 创建并运行服务和客户端节点.

**教程等级:** 初级

**预计时长:** 20 分钟

.. contents:: Contents
   :depth: 2
   :local:

背景
----------

在 :doc:`节点 <../Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes>` 之间使用 :doc:`服务 <../Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services>` 通信时，发送数据请求的节点称为客户端节点，而响应请求的节点称为服务节点。
请求和响应(request and response)的结构由 ``.srv`` 文件决定。

本教程将展示如何创建一个简单的整数加法系统；一个节点请求两个整数的和，另一个节点返回结果。

前提条件
-------------

从之前的教程中学习了如何 :doc:`创建工作空间 <./Creating-A-Workspace/Creating-A-Workspace>` 和 :doc:`创建包 <./Creating-Your-First-ROS2-Package>`。

任务
-----

1 创建包
^^^^^^^^^^^^^^^^^^

打开一个新终端并 :doc:`source ROS 2 安装 <../Beginner-CLI-Tools/Configuring-ROS2-Environment>`，这样 ``ros2`` 命令就能用了。

进入之前创建的 ``ros2_ws`` 目录。

之前学过，包应该在 ``src`` 目录中创建，而不是工作空间的根目录。
进入 ``ros2_ws/src`` 并创建一个新包：

.. code-block:: console

  ros2 pkg create --build-type ament_cmake --license Apache-2.0 cpp_srvcli --dependencies rclcpp example_interfaces

终端会返回一个消息，确认包 ``cpp_srvcli`` 及其所有必要文件和文件夹都已创建。

``--dependencies`` 参数会自动将必要的依赖行添加到 ``package.xml`` 和 ``CMakeLists.txt`` 中。
``example_interfaces`` 中包含 `.srv 文件 <https://github.com/ros2/example_interfaces/blob/{REPOS_FILE_BRANCH}/srv/AddTwoInts.srv>`__ ，在这个文件中定义请求和响应的结构。:

.. code-block:: console

    int64 a
    int64 b
    ---
    int64 sum

前两行定义请求，横线下面定义响应。

1.1 更新 ``package.xml``
~~~~~~~~~~~~~~~~~~~~~~~~~~

因为在包创建过程中使用了 ``--dependencies`` 选项，所以不需要手动将依赖项添加到 ``package.xml`` 或 ``CMakeLists.txt``。

和之前一样，记得向 ``package.xml`` 添加描述、维护者邮箱和姓名，以及许可信息。

.. code-block:: xml

  <description>C++ client server tutorial</description>
  <maintainer email="you@email.com">Your Name</maintainer>
  <license>Apache License 2.0</license>


2 编写服务节点
^^^^^^^^^^^^^^^^^^^^^^^^

在 ``ros2_ws/src/cpp_srvcli/src`` 目录中创建一个名为 ``add_two_ints_server.cpp`` 的新文件，并粘贴以下代码：

.. code-block:: C++

      #include "rclcpp/rclcpp.hpp"
      #include "example_interfaces/srv/add_two_ints.hpp"

      #include <memory>

      void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
                std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>      response)
      {
        response->sum = request->a + request->b;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
                      request->a, request->b);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
      }

      int main(int argc, char **argv)
      {
        rclcpp::init(argc, argv);

        std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_server");

        rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service =
          node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", &add);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");

        rclcpp::spin(node);
        rclcpp::shutdown();
      }

2.1 检查代码
~~~~~~~~~~~~~~~~~~~~

前两个 ``#include`` 语句声明包的依赖项。

``add`` 函数从请求中添加两个整数并将和给响应，同时使用日志通知控制台其状态。

.. code-block:: C++

    void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
             std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>      response)
    {
        response->sum = request->a + request->b;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
            request->a, request->b);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
    }

``main`` 函数逐行完成以下操作：

* 初始化 ROS 2 C++ 客户端库：

  .. code-block:: C++

    rclcpp::init(argc, argv);

* 创建名为 ``add_two_ints_server`` 的节点：

  .. code-block:: C++

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_server");

* 为该节点创建一个名为 ``add_two_ints`` 的服务，自动将其广播到网络中，为服务添加 ``&add`` 作为回调函数：

  .. code-block:: C++

    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service =
    node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", &add);

* 当准备好时打印日志消息：

  .. code-block:: C++

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");

运行(spin)节点，使服务可用。

  .. code-block:: C++

    rclcpp::spin(node);

2.2 添加可执行文件
~~~~~~~~~~~~~~~~~~~~~

``add_executable`` 宏生成一个可以使用 ``ros2 run`` 运行的可执行文件。

将以下代码块添加到 ``CMakeLists.txt`` 中，创建一个名为 ``server`` 的可执行文件：

.. code-block:: console

    add_executable(server src/add_two_ints_server.cpp)
    ament_target_dependencies(server rclcpp example_interfaces)

这样 ``ros2 run`` 就能找到可执行文件，将以下行添加到文件末尾，就在 ``ament_package()`` 之前：

.. code-block:: console

    install(TARGETS
        server
      DESTINATION lib/${PROJECT_NAME})

服务端节点已经准备好了，接下来我们创建客户端节点。

3 编写客户端节点
^^^^^^^^^^^^^^^^^^^^^^^

在 ``ros2_ws/src/cpp_srvcli/src`` 目录中创建一个名为 ``add_two_ints_client.cpp`` 的新文件，并粘贴以下代码：

.. code-block:: C++

  #include "rclcpp/rclcpp.hpp"
  #include "example_interfaces/srv/add_two_ints.hpp"

  #include <chrono>
  #include <cstdlib>
  #include <memory>

  using namespace std::chrono_literals;

  int main(int argc, char **argv)
  {
    rclcpp::init(argc, argv);

    if (argc != 3) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_two_ints_client X Y");
        return 1;
    }

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_client");
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client =
      node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

    auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    request->a = atoll(argv[1]);
    request->b = atoll(argv[2]);

    while (!client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return 0;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
    }

    rclcpp::shutdown();
    return 0;
  }


3.1 检查代码
~~~~~~~~~~~~~~~~~~~~

与服务节点类似，以下代码块创建节点并为该节点创建客户端：

.. code-block:: C++

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_client");
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client =
      node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

接下来创建请求，其结构由之前提到的 ``.srv`` 文件定义。

.. code-block:: C++

  auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  request->a = atoll(argv[1]);
  request->b = atoll(argv[2]);

``while`` 循环给客户端 1 秒的时间在网络中搜索服务节点。
如果找不到服务节点，它会继续等待。

.. code-block:: C++

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");

如果客户端被取消（例如你在终端中输入 ``Ctrl+C``），它会返回一个错误日志消息，说明它被中断了。

.. code-block:: C++

  RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");

如果找到服务节点，客户端会异步发送请求，然后等待结果。

3.2 添加可执行文件
~~~~~~~~~~~~~~~~~~~~~~

在 ``CMakeLists.txt`` 中添加可执行文件和目标，为新节点添加依赖项。
从自动生成的文件中删除一些不必要的模板代码后，你的 ``CMakeLists.txt`` 应该如下所示：

.. code-block:: console

  cmake_minimum_required(VERSION 3.5)
  project(cpp_srvcli)

  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(example_interfaces REQUIRED)

  add_executable(server src/add_two_ints_server.cpp)
  ament_target_dependencies(server rclcpp example_interfaces)

  add_executable(client src/add_two_ints_client.cpp)
  ament_target_dependencies(client rclcpp example_interfaces)

  install(TARGETS
    server
    client
    DESTINATION lib/${PROJECT_NAME})

  ament_package()


4 构建和运行
^^^^^^^^^^^^^^^

在构建之前，最好在工作空间的根目录（ ``ros2_ws`` ）中运行 ``rosdep`` 检查是否有缺少的依赖项：

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      rosdep install -i --from-path src --rosdistro {DISTRO} -y

  .. group-tab:: macOS

      rosdep only runs on Linux, so you can skip ahead to next step.

  .. group-tab:: Windows

      rosdep only runs on Linux, so you can skip ahead to next step.


返回到工作空间的根目录，也就是 ``ros2_ws``，构建新包：

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      colcon build --packages-select cpp_srvcli

  .. group-tab:: macOS

    .. code-block:: console

      colcon build --packages-select cpp_srvcli

  .. group-tab:: Windows

    .. code-block:: console

      colcon build --merge-install --packages-select cpp_srvcli

打开一个新终端，导航到 ``ros2_ws``，并 source 配置文件：

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      source install/setup.bash

  .. group-tab:: macOS

    .. code-block:: console

      . install/setup.bash

  .. group-tab:: Windows

    .. code-block:: console

      call install/setup.bat

现在运行服务节点：

.. code-block:: console

     ros2 run cpp_srvcli server

终端应该返回以下消息，然后等待：

.. code-block:: console

    [INFO] [rclcpp]: Ready to add two ints.

其他终端中，再次 source ``ros2_ws`` 中的配置文件。
运行客户端节点，后面跟两个整数，用空格分隔：

.. code-block:: console

     ros2 run cpp_srvcli client 2 3

如果你发了 ``2`` 和 ``3``，客户端会收到这样的响应：

.. code-block:: console

    [INFO] [rclcpp]: Sum: 5

返回服务节点的终端，你会看到它在接收请求、接收数据和返回响应时都发布了日志消息：

.. code-block:: console

    [INFO] [rclcpp]: Incoming request
    a: 2 b: 3
    [INFO] [rclcpp]: sending back response: [5]

按 ``Ctrl+C`` 停止节点的运行。

总结
-------

你创建了两个节点，用于通过服务请求和响应数据。
你添加了它们的依赖项和可执行文件到包配置文件中，这样你就能构建、运行，并观察到服务/客户端系统的工作情况。

下一步
----------

在最近的几个教程中，你一直在使用接口(interfaces)在 topic 和服务间传递数据。
接下来，你将学习如何 :doc:`创建自定义接口 <./Custom-ROS2-Interfaces>`。

相关内容
---------------

有几种方法可以在 C++ 中编写服务和客户端，查看 `ros2/examples <https://github.com/ros2/examples/tree/{REPOS_FILE_BRANCH}/rclcpp/services>`_ 中的 ``minimal_service`` 和 ``minimal_client`` 。
