.. redirect-from::

    Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber

.. _CppPubSub:

写一组简单的发布者和订阅者(publisher & subscriber) (C++)
===============================================================

**目标:** 用 C++ 创建并运行 publisher & subscriber 节点  .

**教程等级:** 初级

**预计时长:** 20 分钟

.. contents:: Contents
   :depth: 2
   :local:

背景
----------

:doc:`节点 <../Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes>` 是可在 ROS 中通信的可执行进程.
在这个教程中，节点将通过 :doc:`topic <../Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics>` 以字符串消息的形式相互传递信息.
这里的例子是一个简单的 "talker" 和 "listener" 系统; 一个节点发布数据，另一个订阅主题以接收数据.

这些例子中使用的代码可以在 `这里 <https://github.com/ros2/examples/tree/{REPOS_FILE_BRANCH}/rclcpp/topics>`__ 找到.

前提条件
-------------

在之前的教程中，你学会了如何 :doc:`创建工作空间 <./Creating-A-Workspace/Creating-A-Workspace>` 和 :doc:`创建包 <./Creating-Your-First-ROS2-Package>`.

任务
-----

1 创建包
^^^^^^^^^^^^^^^^^^

打开一个新终端， :doc:`source ROS 2 安装环境 <../Beginner-CLI-Tools/Configuring-ROS2-Environment>` 以便 ``ros2`` 指令能够正常工作.

进入之前 :ref:`创建的工作空间 <new-directory>` 的 ``ros2_ws`` 目录.

回想一下之前学到的，包应该在 ``src`` 目录中创建，而不是工作空间的根目录.
所以，进入 ``ros2_ws/src`` 目录，并运行包创建指令:

.. code-block:: console

    ros2 pkg create --build-type ament_cmake --license Apache-2.0 cpp_pubsub

你的终端会返回一个消息，确认 ``cpp_pubsub`` 包及其所需的文件和文件夹已经创建.

进入 ``ros2_ws/src/cpp_pubsub/src`` 目录.
这是 CMake 构建的包中包含可执行文件的目录.


2 编写发布者节点
^^^^^^^^^^^^^^^^^^^^^^^^^^

输入以下命令下载发布者节点的示例代码:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

            wget -O publisher_member_function.cpp https://raw.githubusercontent.com/ros2/examples/{REPOS_FILE_BRANCH}/rclcpp/topics/minimal_publisher/member_function.cpp

   .. group-tab:: macOS

      .. code-block:: console

            wget -O publisher_member_function.cpp https://raw.githubusercontent.com/ros2/examples/{REPOS_FILE_BRANCH}/rclcpp/topics/minimal_publisher/member_function.cpp

   .. group-tab:: Windows

      In a Windows command line prompt:

      .. code-block:: console

            curl -sk https://raw.githubusercontent.com/ros2/examples/{REPOS_FILE_BRANCH}/rclcpp/topics/minimal_publisher/member_function.cpp -o publisher_member_function.cpp

      Or in powershell:

      .. code-block:: console

            curl https://raw.githubusercontent.com/ros2/examples/{REPOS_FILE_BRANCH}/rclcpp/topics/minimal_publisher/member_function.cpp -o publisher_member_function.cpp

现在会有一个新文件 ``publisher_member_function.cpp``.
用你喜欢的文本编辑器打开这个文件.

.. code-block:: C++

    #include <chrono>
    #include <functional>
    #include <memory>
    #include <string>

    #include "rclcpp/rclcpp.hpp"
    #include "std_msgs/msg/string.hpp"

    using namespace std::chrono_literals;

    /* This example creates a subclass of Node and uses std::bind() to register a
    * member function as a callback from the timer. */

    class MinimalPublisher : public rclcpp::Node
    {
      public:
        MinimalPublisher()
        : Node("minimal_publisher"), count_(0)
        {
          publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
          timer_ = this->create_wall_timer(
          500ms, std::bind(&MinimalPublisher::timer_callback, this));
        }

      private:
        void timer_callback()
        {
          auto message = std_msgs::msg::String();
          message.data = "Hello, world! " + std::to_string(count_++);
          RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
          publisher_->publish(message);
        }
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        size_t count_;
    };

    int main(int argc, char * argv[])
    {
      rclcpp::init(argc, argv);
      rclcpp::spin(std::make_shared<MinimalPublisher>());
      rclcpp::shutdown();
      return 0;
    }

2.1 检查代码
~~~~~~~~~~~~~~~~~~~~

这个代码的最前面包含了你需要使用的 C++ 标准库头文件.
接下来 include ``rclcpp/rclcpp.hpp`` ，这样就能使用 ROS 2 系统最基本且最常用的部分.
最后 include ``std_msgs/msg/string.hpp`` ，这是你之后用来发布数据的内置消息类型.

.. code-block:: C++

    #include <chrono>
    #include <functional>
    #include <memory>
    #include <string>

    #include "rclcpp/rclcpp.hpp"
    #include "std_msgs/msg/string.hpp"

    using namespace std::chrono_literals;

这几行代码声明了发布者节点的依赖.
记住，依赖必须在 ``package.xml`` 和 ``CMakeLists.txt`` 中添加，这是下一节你要做的事.

接下来的一行通过继承 ``rclcpp::Node`` 类创建了节点类 ``MinimalPublisher``.
这里的每个 ``this`` 都指向节点.

.. code-block:: C++

    class MinimalPublisher : public rclcpp::Node

这个公共构造函数将节点命名为 ``minimal_publisher`` 并将 ``count_`` 初始化为 0.
在构造函数中，发布者使用 ``create_publisher`` 函数初始化，它的消息类型是 ``std_msgs::msg::String`` ， topic 名是 ``topic`` ，队列大小是 10，用于限制备份时的消息数量.
接下来，初始化 ``timer_`` ，让 ``timer_callback`` 函数每秒执行两次.

.. code-block:: C++

    public:
      MinimalPublisher()
      : Node("minimal_publisher"), count_(0)
      {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
      }

``timer_callback`` 函数是设置消息数据并实际发布消息的函数.
``RCLCPP_INFO`` 宏确保每个发布的消息都打印到控制台.

.. code-block:: C++

    private:
      void timer_callback()
      {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
      }

最后声明定时器、发布者和计数变量.

.. code-block:: C++

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;

``MinimalPublisher`` 类后面是 ``main`` 函数，节点在此实际执行.
``rclcpp::init`` 初始化 ROS 2 ， ``rclcpp::spin`` 开始处理节点的数据，包括来自定时器的回调.

.. code-block:: C++

    int main(int argc, char * argv[])
    {
      rclcpp::init(argc, argv);
      rclcpp::spin(std::make_shared<MinimalPublisher>());
      rclcpp::shutdown();
      return 0;
    }

2.2 添加依赖
~~~~~~~~~~~~~~~~~~~~

返回到 ``ros2_ws/src/cpp_pubsub`` 目录，这里已经创建好了 ``CMakeLists.txt`` 和 ``package.xml`` 文件.

打开 ``package.xml`` 文件.

在 :doc:`上一个教程 <./Creating-Your-First-ROS2-Package>` 中已经学过，要填写 ``<description>`` ， ``<maintainer>`` 和 ``<license>`` 中的内容:

.. code-block:: xml

      <description>Examples of minimal publisher/subscriber using rclcpp</description>
      <maintainer email="you@email.com">Your Name</maintainer>
      <license>Apache License 2.0</license>

在 ``ament_cmake`` 构建工具依赖后添加一行新行，粘贴以下依赖，这些依赖和节点的 include 语句对应:

.. code-block:: xml

    <depend>rclcpp</depend>
    <depend>std_msgs</depend>

这样声明了包在构建和执行时需要 ``rclcpp`` 和 ``std_msgs``.

记得保存文件。

2.3 CMakeLists.txt
~~~~~~~~~~~~~~~~~~

打开 ``CMakeLists.txt`` 文件.
在现有依赖 ``find_package(ament_cmake REQUIRED)`` 下面，添加以下行:

.. code-block:: console

    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)

接下来，添加可执行文件并命名为 ``talker`` ，这样你就能用 ``ros2 run`` 运行节点:

.. code-block:: console

    add_executable(talker src/publisher_member_function.cpp)
    ament_target_dependencies(talker rclcpp std_msgs)

最后，添加 ``install(TARGETS...)`` 部分，这样 ``ros2 run`` 就能找到构建生成的可执行文件:

.. code-block:: console

  install(TARGETS
    talker
    DESTINATION lib/${PROJECT_NAME})

当然你可以清理一下 ``CMakeLists.txt`` ，删除一些不必要的部分和注释，让它看起来像这样:

.. code-block:: console

  cmake_minimum_required(VERSION 3.5)
  project(cpp_pubsub)

  # Default to C++14
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
  endif()

  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()

  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)

  add_executable(talker src/publisher_member_function.cpp)
  ament_target_dependencies(talker rclcpp std_msgs)

  install(TARGETS
    talker
    DESTINATION lib/${PROJECT_NAME})

  ament_package()

现在你可以构建包了，source 配置文件，然后运行节点。不过运行之前我们先创建订阅者节点，这样两部分配合起来就能看到整个系统是如何工作的.

3 编写订阅者节点
^^^^^^^^^^^^^^^^^^^^^^^^^^^

回到 ``ros2_ws/src/cpp_pubsub/src`` 目录，创建订阅者节点的代码.
在终端输入以下命令:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

            wget -O subscriber_member_function.cpp https://raw.githubusercontent.com/ros2/examples/{REPOS_FILE_BRANCH}/rclcpp/topics/minimal_subscriber/member_function.cpp

   .. group-tab:: macOS

      .. code-block:: console

            wget -O subscriber_member_function.cpp https://raw.githubusercontent.com/ros2/examples/{REPOS_FILE_BRANCH}/rclcpp/topics/minimal_subscriber/member_function.cpp

   .. group-tab:: Windows

      In a Windows command line prompt:

      .. code-block:: console

            curl -sk https://raw.githubusercontent.com/ros2/examples/{REPOS_FILE_BRANCH}/rclcpp/topics/minimal_subscriber/member_function.cpp -o subscriber_member_function.cpp

      Or in powershell:

      .. code-block:: console

            curl https://raw.githubusercontent.com/ros2/examples/{REPOS_FILE_BRANCH}/rclcpp/topics/minimal_subscriber/member_function.cpp -o subscriber_member_function.cpp

确定这两个文件是存在的:

.. code-block:: console

    publisher_member_function.cpp  subscriber_member_function.cpp

用你喜欢的文本编辑器打开 ``subscriber_member_function.cpp`` 文件.

.. code-block:: C++

    #include <memory>

    #include "rclcpp/rclcpp.hpp"
    #include "std_msgs/msg/string.hpp"
    using std::placeholders::_1;

    class MinimalSubscriber : public rclcpp::Node
    {
      public:
        MinimalSubscriber()
        : Node("minimal_subscriber")
        {
          subscription_ = this->create_subscription<std_msgs::msg::String>(
          "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
        }

      private:
        void topic_callback(const std_msgs::msg::String & msg) const
        {
          RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
        }
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    };

    int main(int argc, char * argv[])
    {
      rclcpp::init(argc, argv);
      rclcpp::spin(std::make_shared<MinimalSubscriber>());
      rclcpp::shutdown();
      return 0;
    }

3.1 检查代码
~~~~~~~~~~~~~~~~~~~~

订阅者节点的代码几乎和发布者的一样.
现在节点叫 ``minimal_subscriber`` ，构造函数使用节点的 ``create_subscription`` 类来执行回调.

这里没有定时器，因为订阅者只需要在 ``topic`` 上有数据时作出响应.

.. code-block:: C++

    public:
      MinimalSubscriber()
      : Node("minimal_subscriber")
      {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
      }

回想在 :doc:`topic 教程 <../Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics>` 中，发布者和订阅者的 topic 名和消息类型必须匹配才能通信.

``topic_callback`` 函数接收发布的字符串消息数据，并使用 ``RCLCPP_INFO`` 宏将其写入控制台.

唯一的变量声明是就是订阅本身.

.. code-block:: C++

    private:
      void topic_callback(const std_msgs::msg::String & msg) const
      {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
      }
      rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

``main`` 函数和发布者的一样，只是现在它让 ``MinimalSubscriber`` 节点运行.
对于发布者节点，spin 意味着开始计时器，但对于订阅者节点，它只是准备好接收消息.

由于这个节点和发布者节点有相同的依赖，所以 ``package.xml`` 中不需要添加新的内容.

3.2 CMakeLists.txt
~~~~~~~~~~~~~~~~~~

再打开 ``CMakeLists.txt`` ，在发布者的条目下面添加订阅者节点的可执行文件和目标.

.. code-block:: cmake

  add_executable(listener src/subscriber_member_function.cpp)
  ament_target_dependencies(listener rclcpp std_msgs)

  install(TARGETS
    talker
    listener
    DESTINATION lib/${PROJECT_NAME})

保存文件之后，发布者和订阅者节点就都准备好了.

.. _cpppubsub-build-and-run:

4 构建和运行
^^^^^^^^^^^^^^^
很大概率你已经安装了 ``rclcpp`` 和 ``std_msgs`` 包，因为它们是 ROS 2 系统的一部分.
但是，最好在构建之前在工作空间的根目录（ ``ros2_ws`` ）下运行 ``rosdep`` 检查是否有缺少的依赖:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

            rosdep install -i --from-path src --rosdistro {DISTRO} -y

   .. group-tab:: macOS

      rosdep only runs on Linux, so you can skip ahead to next step.

   .. group-tab:: Windows

      rosdep only runs on Linux, so you can skip ahead to next step.


现在在工作空间的根目录（ ``ros2_ws`` ）下构建新包:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      colcon build --packages-select cpp_pubsub

  .. group-tab:: macOS

    .. code-block:: console

      colcon build --packages-select cpp_pubsub

  .. group-tab:: Windows

    .. code-block:: console

      colcon build --merge-install --packages-select cpp_pubsub

构建完成后，打开新终端，进入 ``ros2_ws`` , source 配置文件:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      . install/setup.bash

  .. group-tab:: macOS

    .. code-block:: console

      . install/setup.bash

  .. group-tab:: Windows

    .. code-block:: console

      call install/setup.bat

运行发布者节点:

.. code-block:: console

     ros2 run cpp_pubsub talker

终端开始每 0.5 秒发布一条消息，如下所示:

.. code-block:: console

    [INFO] [minimal_publisher]: Publishing: "Hello World: 0"
    [INFO] [minimal_publisher]: Publishing: "Hello World: 1"
    [INFO] [minimal_publisher]: Publishing: "Hello World: 2"
    [INFO] [minimal_publisher]: Publishing: "Hello World: 3"
    [INFO] [minimal_publisher]: Publishing: "Hello World: 4"

打开另一个终端，再次 source ``ros2_ws`` 中的配置文件，然后运行订阅者节点:

.. code-block:: console

     ros2 run cpp_pubsub listener

订阅者开始打印发布者当前的发布的计数到控制台，如下所示:

.. code-block:: console

  [INFO] [minimal_subscriber]: I heard: "Hello World: 10"
  [INFO] [minimal_subscriber]: I heard: "Hello World: 11"
  [INFO] [minimal_subscriber]: I heard: "Hello World: 12"
  [INFO] [minimal_subscriber]: I heard: "Hello World: 13"
  [INFO] [minimal_subscriber]: I heard: "Hello World: 14"

在每个终端中按 ``Ctrl+C`` 可以停止节点.

总结
-------

你创建了两个节点，通过 topic 发布和订阅数据.
在编译和运行之前，添加了它们的依赖和可执行文件到包配置文件中.

下一步
----------

接下来你需要创建另一个简单的 ROS 2 包，使用服务/客户端模型.
你可以选择用 :doc:`C++ <./Writing-A-Simple-Cpp-Service-And-Client>` 或者 :doc:`Python <./Writing-A-Simple-Py-Service-And-Client>` 来写.

相关内容
---------------

有很多种方法可以在 C++ 中实现发布者和订阅者; 查看 `ros2/examples <https://github.com/ros2/examples/tree/{REPOS_FILE_BRANCH}/rclcpp/topics>`_ 中的 ``minimal_publisher`` 和 ``minimal_subscriber`` 包.
