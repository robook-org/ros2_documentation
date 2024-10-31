.. redirect-from::

    Tutorials/Custom-ROS2-Interfaces

.. _CustomInterfaces:

创建自定义的 msg 和 srv
=================================

**目标:** 创建自定义的接口(interface)文件 (``.msg`` 和 ``.srv``)，在 Python 和 C++ 节点中使用它们.

**教程等级:** 初级

**预计时长:** 20 分钟

.. contents:: Contents
   :depth: 2
   :local:

背景
----------

在之前的教程中，你使用消息和服务接口学习了 :doc:`话题 <../Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics>`， :doc:`服务 <../Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services>`，以及简单的发布者/订阅者 (:doc:`C++ <./Writing-A-Simple-Cpp-Publisher-And-Subscriber>`/:doc:`Python<./Writing-A-Simple-Py-Publisher-And-Subscriber>`) 和服务/客户端 (:doc:`C++ <./Writing-A-Simple-Cpp-Service-And-Client>`/:doc:`Python<./Writing-A-Simple-Py-Service-And-Client>`) 节点。
在这些情况下，你使用的接口是预定义好的。

虽然使用预定义的接口定义是一个很好的实践，但有时你可能需要定义自己的消息和服务。
这个教程将介绍创建自定义接口的最简单方法。

前提条件
-------------

你已经创建了 :doc:`ROS 2 工作空间 <./Creating-A-Workspace/Creating-A-Workspace>`。

这个教程还使用了之前教程中创建的发布者/订阅者 (:doc:`C++ <./Writing-A-Simple-Cpp-Publisher-And-Subscriber>` 和 :doc:`Python<./Writing-A-Simple-Py-Publisher-And-Subscriber>`) 和服务/客户端 (:doc:`C++ <./Writing-A-Simple-Cpp-Service-And-Client>` 和 :doc:`Python<./Writing-A-Simple-Py-Service-And-Client>`) 节点包，以测试新的自定义消息。

任务
-----

1 创建新的包
^^^^^^^^^^^^^^^^^^^^^^^

在这个教程中，你将在自己的包中创建自定义的 ``.msg`` 和 ``.srv`` 文件，然后在另一个包中使用它们。
这些包应该在同一个工作空间中。

因为我们将使用之前教程中创建的发布者/订阅者和服务/客户端包，所以确保你在与这些包相同的工作空间中 (``ros2_ws/src``)，然后运行以下命令创建一个新包：

.. code-block:: console

  ros2 pkg create --build-type ament_cmake --license Apache-2.0 tutorial_interfaces

``tutorial_interfaces`` 是新包的名称。
请注意，它是并且只能是 CMake 包，但这并不限制你可以在哪种类型的包中使用你的消息和服务。
最后一节中将介绍如何在 CMake 包中创建自定义接口，然后在 C++ 或 Python 节点中使用它。

``.msg`` 和 ``.srv`` 文件必须放在名为 ``msg`` 和 ``srv`` 的目录中。

在 ``ros2_ws/src/tutorial_interfaces`` 中创建这些目录：

.. code-block:: console

  mkdir msg srv

2 创建自定义接口
^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1 msg 定义
~~~~~~~~~~~~~~~~~~

在 ``tutorial_interfaces/msg`` 目录中创建一个名为 ``Num.msg`` 的新文件，其中包含一行代码声明其数据结构：

.. code-block:: console

    int64 num

这是一个自定义消息，传输一个名为 ``num`` 的 64 位整数。

在你刚刚创建的 ``tutorial_interfaces/msg`` 目录中，创建一个名为 ``Sphere.msg`` 的新文件，其中包含以下内容：

.. code-block:: console

    geometry_msgs/Point center
    float64 radius

这个消息中引用了其它消息包中的消息 (这里是 ``geometry_msgs/Point``)。

2.2 srv 定义
~~~~~~~~~~~~~~~~~~

在 ``tutorial_interfaces/srv`` 目录中创建一个名为 ``AddThreeInts.srv`` 的新文件，其中包含以下请求和响应结构：

.. code-block:: console

  int64 a
  int64 b
  int64 c
  ---
  int64 sum

这是一个自定义服务，请求三个名为 ``a``、``b`` 和 ``c`` 的整数，并响应一个名为 ``sum`` 的整数。

3 ``CMakeLists.txt``
^^^^^^^^^^^^^^^^^^^^

为了将你定义的接口转换为特定语言的代码 (如 C++ 和 Python)，需要以下行添加到 ``CMakeLists.txt`` 中：

.. code-block:: cmake

  find_package(geometry_msgs REQUIRED)
  find_package(rosidl_default_generators REQUIRED)

  rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/Num.msg"
    "msg/Sphere.msg"
    "srv/AddThreeInts.srv"
    DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
  )

.. note::

  在 ``rosidl_generate_interfaces`` 中的第一个参数 (库名称) 必须与 ${PROJECT_NAME} 匹配 (参见 https://github.com/ros2/rosidl/issues/441#issuecomment-591025515).

4 ``package.xml``
^^^^^^^^^^^^^^^^^

因为接口依赖于 ``rosidl_default_generators`` 生成特定语言的代码，所以你需要声明构建工具的依赖项。
``rosidl_default_runtime`` 是运行时或执行阶段(runtime or execution-stage)的依赖项，是用接口所需要的。
``rosidl_interface_packages`` 是你的包 ``tutorial_interfaces`` 的依赖，使用 ``<member_of_group>`` 标签声明。

在 ``package.xml`` 的 ``<package>`` 元素中添加以下行：

.. code-block:: xml

  <depend>geometry_msgs</depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

5 构建 ``tutorial_interfaces`` 包
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

现在自定义接口包的所有部分都准备好了，可以构建包了。
在你的工作空间根目录 (``~/ros2_ws``) 中运行以下命令：

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      colcon build --packages-select tutorial_interfaces

  .. group-tab:: macOS

    .. code-block:: console

      colcon build --packages-select tutorial_interfaces

  .. group-tab:: Windows

    .. code-block:: console

      colcon build --merge-install --packages-select tutorial_interfaces

现在这个接口可以被其它 ROS 2 包发现。

6 确认 msg 和 srv 创建成功
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

在新的终端中的工作空间 (``ros2_ws``) 里运行以下命令来 source 它：

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

用 ``ros2 interface show`` 命令确认接口创建成功：

.. code-block:: console

  ros2 interface show tutorial_interfaces/msg/Num

应该会有这样的输出：

.. code-block:: console

    int64 num

然后再确认另一个：

.. code-block:: console

  ros2 interface show tutorial_interfaces/msg/Sphere

这个应该返回:

.. code-block:: console

    geometry_msgs/Point center
            float64 x
            float64 y
            float64 z
    float64 radius

还有最后一个：

.. code-block:: console

  ros2 interface show tutorial_interfaces/srv/AddThreeInts

它的返回应该是:

.. code-block:: console

    int64 a
    int64 b
    int64 c
    ---
    int64 sum

7 测试新接口
^^^^^^^^^^^^^^^^^^^^^^^^^

在这一步中你可以使用之前创建的包。
只需要对节点、 ``CMakeLists.txt`` 和 ``package.xml`` 做一点小修改就能使用新接口了。

7.1 使用发布/订阅模式测试 ``Num.msg``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

对之前教程(:doc:`C++ <./Writing-A-Simple-Cpp-Publisher-And-Subscriber>` 或 :doc:`Python <./Writing-A-Simple-Py-Publisher-And-Subscriber>`) 中创建的发布者/订阅者包做一些修改，你就可以看到 ``Num.msg`` 的效果了。
因为你将把发布的字符串改成了数字，所以输出会有一点不同。

**Publisher**

.. tabs::

  .. group-tab:: C++

    .. code-block:: c++

      #include <chrono>
      #include <memory>

      #include "rclcpp/rclcpp.hpp"
      #include "tutorial_interfaces/msg/num.hpp"                                            // CHANGE

      using namespace std::chrono_literals;

      class MinimalPublisher : public rclcpp::Node
      {
      public:
        MinimalPublisher()
        : Node("minimal_publisher"), count_(0)
        {
          publisher_ = this->create_publisher<tutorial_interfaces::msg::Num>("topic", 10);  // CHANGE
          timer_ = this->create_wall_timer(
            500ms, std::bind(&MinimalPublisher::timer_callback, this));
        }

      private:
        void timer_callback()
        {
          auto message = tutorial_interfaces::msg::Num();                                   // CHANGE
          message.num = this->count_++;                                                     // CHANGE
          RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.num << "'");    // CHANGE
          publisher_->publish(message);
        }
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<tutorial_interfaces::msg::Num>::SharedPtr publisher_;             // CHANGE
        size_t count_;
      };

      int main(int argc, char * argv[])
      {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<MinimalPublisher>());
        rclcpp::shutdown();
        return 0;
      }

  .. group-tab:: Python

    .. code-block:: python

      import rclpy
      from rclpy.node import Node

      from tutorial_interfaces.msg import Num                            # CHANGE


      class MinimalPublisher(Node):

          def __init__(self):
              super().__init__('minimal_publisher')
              self.publisher_ = self.create_publisher(Num, 'topic', 10)  # CHANGE
              timer_period = 0.5
              self.timer = self.create_timer(timer_period, self.timer_callback)
              self.i = 0

          def timer_callback(self):
              msg = Num()                                                # CHANGE
              msg.num = self.i                                           # CHANGE
              self.publisher_.publish(msg)
              self.get_logger().info('Publishing: "%d"' % msg.num)       # CHANGE
              self.i += 1


      def main(args=None):
          rclpy.init(args=args)

          minimal_publisher = MinimalPublisher()

          rclpy.spin(minimal_publisher)

          minimal_publisher.destroy_node()
          rclpy.shutdown()


      if __name__ == '__main__':
          main()


**Subscriber**

.. tabs::

  .. group-tab:: C++

    .. code-block:: c++

      #include <functional>
      #include <memory>

      #include "rclcpp/rclcpp.hpp"
      #include "tutorial_interfaces/msg/num.hpp"                                       // CHANGE

      using std::placeholders::_1;

      class MinimalSubscriber : public rclcpp::Node
      {
      public:
        MinimalSubscriber()
        : Node("minimal_subscriber")
        {
          subscription_ = this->create_subscription<tutorial_interfaces::msg::Num>(    // CHANGE
            "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
        }

      private:
        void topic_callback(const tutorial_interfaces::msg::Num & msg) const  // CHANGE
        {
          RCLCPP_INFO_STREAM(this->get_logger(), "I heard: '" << msg.num << "'");     // CHANGE
        }
        rclcpp::Subscription<tutorial_interfaces::msg::Num>::SharedPtr subscription_;  // CHANGE
      };

      int main(int argc, char * argv[])
      {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<MinimalSubscriber>());
        rclcpp::shutdown();
        return 0;
      }

  .. group-tab:: Python

    .. code-block:: python

      import rclpy
      from rclpy.node import Node

      from tutorial_interfaces.msg import Num                        # CHANGE


      class MinimalSubscriber(Node):

          def __init__(self):
              super().__init__('minimal_subscriber')
              self.subscription = self.create_subscription(
                  Num,                                               # CHANGE
                  'topic',
                  self.listener_callback,
                  10)
              self.subscription

          def listener_callback(self, msg):
              self.get_logger().info('I heard: "%d"' % msg.num)  # CHANGE


      def main(args=None):
          rclpy.init(args=args)

          minimal_subscriber = MinimalSubscriber()

          rclpy.spin(minimal_subscriber)

          minimal_subscriber.destroy_node()
          rclpy.shutdown()


      if __name__ == '__main__':
          main()


**CMakeLists.txt**

添加以下内容(只有 C++ 包需要)：

.. code-block:: cmake

    #...

    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(tutorial_interfaces REQUIRED)                      # CHANGE

    add_executable(talker src/publisher_member_function.cpp)
    ament_target_dependencies(talker rclcpp tutorial_interfaces)    # CHANGE

    add_executable(listener src/subscriber_member_function.cpp)
    ament_target_dependencies(listener rclcpp tutorial_interfaces)  # CHANGE

    install(TARGETS
      talker
      listener
      DESTINATION lib/${PROJECT_NAME})

    ament_package()


**package.xml**

添加以下内容：

.. tabs::

  .. group-tab:: C++

    .. code-block:: c++

      <depend>tutorial_interfaces</depend>

  .. group-tab:: Python

    .. code-block:: python

      <exec_depend>tutorial_interfaces</exec_depend>


做完上面这些修改之后，重新构建一下包：

.. tabs::

  .. group-tab:: C++

    On Linux/macOS:

    .. code-block:: console

      colcon build --packages-select cpp_pubsub

    On Windows:

    .. code-block:: console

      colcon build --merge-install --packages-select cpp_pubsub

  .. group-tab:: Python

    On Linux/macOS:

    .. code-block:: console

      colcon build --packages-select py_pubsub

    On Windows:

    .. code-block:: console

      colcon build --merge-install --packages-select py_pubsub

然后打开两个新终端，分别 source ``ros2_ws`` ，然后运行:

.. tabs::

  .. group-tab:: C++

    .. code-block:: console

          ros2 run cpp_pubsub talker

    .. code-block:: console

          ros2 run cpp_pubsub listener

  .. group-tab:: Python

    .. code-block:: console

        ros2 run py_pubsub talker

    .. code-block:: console

        ros2 run py_pubsub listener

``Num.msg`` 只传递一个整数，所以发布者只会发布整数值，而不是之前发布的字符串：

.. code-block:: console

    [INFO] [minimal_publisher]: Publishing: '0'
    [INFO] [minimal_publisher]: Publishing: '1'
    [INFO] [minimal_publisher]: Publishing: '2'


7.2 用服务/客户端测试 ``AddThreeInts.srv``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

对之前教程(:doc:`C++ <./Writing-A-Simple-Cpp-Service-And-Client>` 或 :doc:`Python <./Writing-A-Simple-Py-Service-And-Client>`) 中创建的服务/客户端包做一些修改，你就可以看到 ``AddThreeInts.srv`` 的效果了。
因为你将把原来的两个整数请求的 srv 改成了三个整数请求的 srv，所以输出会有一点不同。

**Service**

.. tabs::

  .. group-tab:: C++

    .. code-block:: c++

      #include "rclcpp/rclcpp.hpp"
      #include "tutorial_interfaces/srv/add_three_ints.hpp"                                        // CHANGE

      #include <memory>

      void add(const std::shared_ptr<tutorial_interfaces::srv::AddThreeInts::Request> request,     // CHANGE
                std::shared_ptr<tutorial_interfaces::srv::AddThreeInts::Response>       response)  // CHANGE
      {
        response->sum = request->a + request->b + request->c;                                      // CHANGE
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld" " c: %ld",  // CHANGE
                      request->a, request->b, request->c);                                         // CHANGE
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
      }

      int main(int argc, char **argv)
      {
        rclcpp::init(argc, argv);

        std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_three_ints_server");   // CHANGE

        rclcpp::Service<tutorial_interfaces::srv::AddThreeInts>::SharedPtr service =               // CHANGE
          node->create_service<tutorial_interfaces::srv::AddThreeInts>("add_three_ints",  &add);   // CHANGE

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add three ints.");                     // CHANGE

        rclcpp::spin(node);
        rclcpp::shutdown();
      }

  .. group-tab:: Python

    .. code-block:: python

      from tutorial_interfaces.srv import AddThreeInts                                                           # CHANGE

      import rclpy
      from rclpy.node import Node


      class MinimalService(Node):

          def __init__(self):
              super().__init__('minimal_service')
              self.srv = self.create_service(AddThreeInts, 'add_three_ints', self.add_three_ints_callback)       # CHANGE

          def add_three_ints_callback(self, request, response):
              response.sum = request.a + request.b + request.c                                                   # CHANGE
              self.get_logger().info('Incoming request\na: %d b: %d c: %d' % (request.a, request.b, request.c))  # CHANGE

              return response

      def main(args=None):
          rclpy.init(args=args)

          minimal_service = MinimalService()

          rclpy.spin(minimal_service)

          rclpy.shutdown()

      if __name__ == '__main__':
          main()

**Client**

.. tabs::

  .. group-tab:: C++

    .. code-block:: c++

      #include "rclcpp/rclcpp.hpp"
      #include "tutorial_interfaces/srv/add_three_ints.hpp"                                       // CHANGE

      #include <chrono>
      #include <cstdlib>
      #include <memory>

      using namespace std::chrono_literals;

      int main(int argc, char **argv)
      {
        rclcpp::init(argc, argv);

        if (argc != 4) { // CHANGE
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_three_ints_client X Y Z");      // CHANGE
            return 1;
        }

        std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_three_ints_client");  // CHANGE
        rclcpp::Client<tutorial_interfaces::srv::AddThreeInts>::SharedPtr client =                // CHANGE
          node->create_client<tutorial_interfaces::srv::AddThreeInts>("add_three_ints");          // CHANGE

        auto request = std::make_shared<tutorial_interfaces::srv::AddThreeInts::Request>();       // CHANGE
        request->a = atoll(argv[1]);
        request->b = atoll(argv[2]);
        request->c = atoll(argv[3]);                                                              // CHANGE

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
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_three_ints");    // CHANGE
        }

        rclcpp::shutdown();
        return 0;
      }

  .. group-tab:: Python

    .. code-block:: python

      from tutorial_interfaces.srv import AddThreeInts                            # CHANGE
      import sys
      import rclpy
      from rclpy.node import Node


      class MinimalClientAsync(Node):

          def __init__(self):
              super().__init__('minimal_client_async')
              self.cli = self.create_client(AddThreeInts, 'add_three_ints')       # CHANGE
              while not self.cli.wait_for_service(timeout_sec=1.0):
                  self.get_logger().info('service not available, waiting again...')
              self.req = AddThreeInts.Request()                                   # CHANGE

          def send_request(self):
              self.req.a = int(sys.argv[1])
              self.req.b = int(sys.argv[2])
              self.req.c = int(sys.argv[3])                                       # CHANGE
              self.future = self.cli.call_async(self.req)


      def main(args=None):
          rclpy.init(args=args)

          minimal_client = MinimalClientAsync()
          minimal_client.send_request()

          while rclpy.ok():
              rclpy.spin_once(minimal_client)
              if minimal_client.future.done():
                  try:
                      response = minimal_client.future.result()
                  except Exception as e:
                      minimal_client.get_logger().info(
                          'Service call failed %r' % (e,))
                  else:
                      minimal_client.get_logger().info(
                          'Result of add_three_ints: for %d + %d + %d = %d' %                                # CHANGE
                          (minimal_client.req.a, minimal_client.req.b, minimal_client.req.c, response.sum))  # CHANGE
                  break

          minimal_client.destroy_node()
          rclpy.shutdown()


      if __name__ == '__main__':
          main()


**CMakeLists.txt**

添加以下内容(只有 C++ 包需要)：

.. code-block:: cmake

    #...

    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(tutorial_interfaces REQUIRED)         # CHANGE

    add_executable(server src/add_two_ints_server.cpp)
    ament_target_dependencies(server
      rclcpp tutorial_interfaces)                      # CHANGE

    add_executable(client src/add_two_ints_client.cpp)
    ament_target_dependencies(client
      rclcpp tutorial_interfaces)                      # CHANGE

    install(TARGETS
      server
      client
      DESTINATION lib/${PROJECT_NAME})

    ament_package()


**package.xml**

添加以下内容：

.. tabs::

  .. group-tab:: C++

    .. code-block:: c++

      <depend>tutorial_interfaces</depend>

  .. group-tab:: Python

    .. code-block:: python

      <exec_depend>tutorial_interfaces</exec_depend>


做完上面这些修改之后，重新构建一下包：

.. tabs::

  .. group-tab:: C++

    On Linux/macOS:

    .. code-block:: console

      colcon build --packages-select cpp_srvcli

    On Windows:

    .. code-block:: console

      colcon build --merge-install --packages-select cpp_srvcli


  .. group-tab:: Python

    On Linux/macOS:

    .. code-block:: console

      colcon build --packages-select py_srvcli

    On Windows:

    .. code-block:: console

      colcon build --merge-install --packages-select py_srvcli

然后打开两个新终端，分别 source ``ros2_ws`` ，然后运行:

.. tabs::

  .. group-tab:: C++

    .. code-block:: console

          ros2 run cpp_srvcli server

    .. code-block:: console

          ros2 run cpp_srvcli client 2 3 1

  .. group-tab:: Python

    .. code-block:: console

        ros2 run py_srvcli service

    .. code-block:: console

        ros2 run py_srvcli client 2 3 1


总结
-------

在这个教程中，你学习了如何在自己的包中创建自定义接口，并在其它包中使用它们。

这个教程只是关于定义自定义接口的入门。
你可以在 :doc:`ROS 2 接口 <../../Concepts/Basic/About-Interfaces>` 中了解更多信息。

下一步
----------

:doc:`下一个教程 <./Single-Package-Define-And-Use-Interface>` 将介绍更多使用接口的方法。
