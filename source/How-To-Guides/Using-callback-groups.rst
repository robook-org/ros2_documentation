使用 Callback Groups
=====================

在 Multi-Threaded Executor 中运行节点时，ROS 2 提供了回调函数组作为控制不同回调执行的工具。
本页面旨在指导如何高效使用回调函数组(callback groups)。


本页面假设读者对 :doc:`executors <../Concepts/Intermediate/About-Executors>` 的概念有基本了解。


.. contents:: Table of Contents
   :local:

callback groups 基础
-------------------------

在 Multi-Threaded Executor 中运行节点时，ROS 2 提供了两种不同类型的回调函数组(callback groups)来控制回调的执行：

* Mutually Exclusive Callback Group
* Reentrant Callback Group

这些回调组以不同的方式限制其回调的执行。
简单来说：

* Mutually Exclusive Callback Group 会阻止其回调并行执行 - 本质上使得组中的回调像由 SingleThreadedExecutor 执行的一样。
* Reentrant Callback Group 允许执行器以任何方式安排和执行 组的回调，没有限制。
  这意味着，除了不同回调函数之间并行执行外，同一回调函数的不同实例也可以并发执行(译者注：比如说某个 subscription 的回调函数被快速的调用了好几次，这几次调用也可能是并行的)。
* 属于不同回调组的回调（任何类型的）总是可以并行执行。

要始终记住的是，不同的 ROS 2 实体会将其回调组传递给它们生成的所有回调。
例如，如果给一个 action client 分配了一个回调组，那么由这个 client 创建的所有回调都将分配给该回调组。

Callback groups 可以通过 rclcpp 的 ``create_callback_group`` 函数或是 rclpy 的对应组的构造函数创建。
然后，回调组可以作为参数/选项在创建 subscription、timer 等时传递进去。

.. tabs::

  .. group-tab:: C++

    .. code-block:: cpp

      my_callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

      rclcpp::SubscriptionOptions options;
      options.callback_group = my_callback_group;

      my_subscription = create_subscription<Int32>("/topic", rclcpp::SensorDataQoS(),
                                                    callback, options);

  .. group-tab:: Python

    .. code-block:: python

      my_callback_group = MutuallyExclusiveCallbackGroup()
      my_subscription = self.create_subscription(Int32, "/topic", self.callback, qos_profile=1,
                                                  callback_group=my_callback_group)

所有没有指定回调组的 subscriptions、timer 等都被分配给 *默认回调组(default callback group)* 。
默认回调组是一个 Mutually Exclusive Callback Group，在 rclcpp 中可以通过 ``NodeBaseInterface::get_default_callback_group()`` 查询，在 rclpy 中可以通过 ``Node.default_callback_group`` 查询。

关于 callbacks
^^^^^^^^^^^^^^^

在 ROS 2 和执行器(executors)的 context 中，回调(callback)指的是一个由执行器调度和执行的函数。
例如：

* subscription callbacks (从 topic 接收并处理消息),
* timer callbacks,
* service callbacks (用于在服务端处理 service requests),
* action 服务端和客户端的不同 callbacks,
* Futures 的 done-callbacks. (译者注： “Future” 是异步编程中的一个概念，并非ROS 2特有，若需要了解这个例子的含义请先学习异步编程有关的知识)

在处理回调组时，有几个关于回调的重要信息应该牢记在心。

* 在 ROS 2 中几乎所有东西都是回调！
  由执行器执行的每个函数都是回调。
  ROS 2 系统中的非回调函数主要位于系统的边缘（用户和传感器输入等）。
* 有些时候从用户/开发者 API 中可能看不到回调的存在。
  这种情况尤其出现在任何类型的“同步”调用(“synchronous” call) service 或 action 时（在 rclpy 中）。
  例如，对服务的同步调用 ``Client.call(request)`` 会添加一个 Future 的 done-callback，这个 callback 需要在函数调用期间执行，但这个回调对用户来说并不直接可见。
  (译者注：也就是说即使在用户看来， ``call()`` 这个过程是同步发生的，但其实背后也是用回调实现的，只是这个回调会阻塞 ``call()`` 到结果返回，而一般这个回调消耗的时间并不多，所以看起来也像是一个同步执行的函数。)


控制执行(Controlling execution)
----------------------------------------

为了使用回调组控制执行，可以考虑遵循以下指导。

考虑一个回调函数与自身的交互关系：

* 如果一个回调函数可能与自身并行执行，将其注册到 Reentrant Callback Group。
  一个例子是一个需要能够并行处理多个 action 调用的 action/service server。

* 如果一个回调函数 **永远** 不应该与自身并行执行，将其注册到 Mutually Exclusive Callback Group。

考虑不同回调之间的交互关系：

* 如果它们 **永远** 不应该并行执行，将它们注册到同一个 Mutually Exclusive Callback Group。
  一个例子是回调访问共享的关键资源和非线程安全资源。

如果它们应该并行执行，有两种选择，取决于个别回调是否应该能够与自身重叠：

* 将它们注册到不同的 Mutually Exclusive Callback Group（各自的回调自身不会重叠）

* 将它们注册到 Reentrant Callback Group（各自的回调自身可能重叠）

一个例子是一个节点有一个同步的 service 客户端和一个定时调用这个服务的 timer 。
查看下面的详细示例。

避免死锁(Avoiding deadlocks)
----------------------------------

错误地设置节点的回调组可能导致死锁（或其他不希望的行为），尤其是如果希望对 service 或 action 使用同步调用。
事实上，ROS 2 的 API 文档中提到，不应该在回调中进行同步调用，因为这可能导致死锁。
虽然使用异步调用(asynchronous calls)确实在这方面更安全，但也有办法让同步调用可以正常工作。
另一方面，同步调用也有其优点，例如使代码更简单易懂。
因此，本节提供了一些关于如何正确设置节点的回调组以避免死锁的指导。

首先要注意的是，每个节点的默认回调组是一个 Mutually Exclusive Callback Group。
如果用户在创建 timer、subscription、客户端等时没有指定回调组，那么这些实体创建的所有回调都将使用节点的默认回调组。
此外，如果节点的所有回调都使用相同的 Mutually Exclusive Callback Group，那么该节点实际上就像是由 Single-Threaded Executor 处理的一样，即使指定了 Multi-Threaded Executor！
因此，每当决定使用 Multi-Threaded Executor 时，应始终在节点的回调中指定一个或多个回调组。
记住上面这些信息，以下是一些指导，帮助你避免死锁：

* 如果在任何类型的回调中进行同步调用，这个回调和进行发起请求的客户端需要属于

  * 不同类型的回调组（任何类型），或
  * 一个 Reentrant Callback Group。

* 如果上述配置由于其他要求（例如线程安全和/或在等待结果时阻止其他回调）而无法实现，请使用异步调用。

如果不能满足上述第一点的要求，那么一定会导致死锁。
这种情况的一个例子是在 timer 的回调中进行同步服务调用（请参见下一节的示例）。


示例
--------

让我们看一些不同回调组设置的简单示例。
以下代码演示在 timer 回调中同步地调用服务。

Demo code
^^^^^^^^^

我们需要两个节点，其中一个包含一个简单的 service 服务端:

.. tabs::

   .. group-tab:: C++

      .. code-block:: cpp

        #include <memory>
        #include "rclcpp/rclcpp.hpp"
        #include "std_srvs/srv/empty.hpp"

        using namespace std::placeholders;

        namespace cb_group_demo
        {
        class ServiceNode : public rclcpp::Node
        {
        public:
            ServiceNode() : Node("service_node")
            {
                service_ptr_ = this->create_service<std_srvs::srv::Empty>(
                        "test_service",
                        std::bind(&ServiceNode::service_callback, this, _1, _2, _3)
                );
            }

        private:
            rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_ptr_;

            void service_callback(
                    const std::shared_ptr<rmw_request_id_t> request_header,
                    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                    const std::shared_ptr<std_srvs::srv::Empty::Response> response)
            {
                (void)request_header;
                (void)request;
                (void)response;
                RCLCPP_INFO(this->get_logger(), "Received request, responding...");
            }
        };  // class ServiceNode
        }   // namespace cb_group_demo

        int main(int argc, char* argv[])
        {
            rclcpp::init(argc, argv);
            auto service_node = std::make_shared<cb_group_demo::ServiceNode>();

            RCLCPP_INFO(service_node->get_logger(), "Starting server node, shut down with CTRL-C");
            rclcpp::spin(service_node);
            RCLCPP_INFO(service_node->get_logger(), "Keyboard interrupt, shutting down.\n");

            rclcpp::shutdown();
            return 0;
        }

   .. group-tab:: Python

      .. code-block:: python

        import rclpy
        from rclpy.node import Node
        from std_srvs.srv import Empty

        class ServiceNode(Node):
            def __init__(self):
                super().__init__('service_node')
                self.srv = self.create_service(Empty, 'test_service', callback=self.service_callback)

            def service_callback(self, request, result):
                self.get_logger().info('Received request, responding...')
                return result


        if __name__ == '__main__':
            rclpy.init()
            node = ServiceNode()
            try:
                node.get_logger().info("Starting server node, shut down with CTRL-C")
                rclpy.spin(node)
            except KeyboardInterrupt:
                node.get_logger().info('Keyboard interrupt, shutting down.\n')
            node.destroy_node()
            rclpy.shutdown()

另一个节点包含一个 service 客户端和一个 timer 用于向服务器发出请求：

.. tabs::

  .. group-tab:: C++

    *注意:* service 客户端在 rclcpp 中没有提供类似 rclpy 中的同步调用方法，
    所以我们通过等待 future 对象来模拟同步调用的效果。

    .. code-block:: cpp

      #include <chrono>
      #include <memory>
      #include "rclcpp/rclcpp.hpp"
      #include "std_srvs/srv/empty.hpp"

      using namespace std::chrono_literals;

      namespace cb_group_demo
      {
      class DemoNode : public rclcpp::Node
      {
      public:
          DemoNode() : Node("client_node")
          {
              client_cb_group_ = nullptr;
              timer_cb_group_ = nullptr;
              client_ptr_ = this->create_client<std_srvs::srv::Empty>("test_service", rmw_qos_profile_services_default,
                                                                      client_cb_group_);
              timer_ptr_ = this->create_wall_timer(1s, std::bind(&DemoNode::timer_callback, this),
                                                  timer_cb_group_);
          }

      private:
          rclcpp::CallbackGroup::SharedPtr client_cb_group_;
          rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
          rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client_ptr_;
          rclcpp::TimerBase::SharedPtr timer_ptr_;

          void timer_callback()
          {
              RCLCPP_INFO(this->get_logger(), "Sending request");
              auto request = std::make_shared<std_srvs::srv::Empty::Request>();
              auto result_future = client_ptr_->async_send_request(request);
              std::future_status status = result_future.wait_for(10s);  // timeout to guarantee a graceful finish
              if (status == std::future_status::ready) {
                  RCLCPP_INFO(this->get_logger(), "Received response");
              }
          }
      };  // class DemoNode
      }   // namespace cb_group_demo

      int main(int argc, char* argv[])
      {
          rclcpp::init(argc, argv);
          auto client_node = std::make_shared<cb_group_demo::DemoNode>();
          rclcpp::executors::MultiThreadedExecutor executor;
          executor.add_node(client_node);

          RCLCPP_INFO(client_node->get_logger(), "Starting client node, shut down with CTRL-C");
          executor.spin();
          RCLCPP_INFO(client_node->get_logger(), "Keyboard interrupt, shutting down.\n");

          rclcpp::shutdown();
          return 0;
      }

  .. group-tab:: Python

    .. code-block:: python

      import rclpy
      from rclpy.executors import MultiThreadedExecutor
      from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
      from rclpy.node import Node
      from std_srvs.srv import Empty


      class CallbackGroupDemo(Node):
          def __init__(self):
              super().__init__('client_node')

              client_cb_group = None
              timer_cb_group = None
              self.client = self.create_client(Empty, 'test_service', callback_group=client_cb_group)
              self.call_timer = self.create_timer(1, self._timer_cb, callback_group=timer_cb_group)

          def _timer_cb(self):
              self.get_logger().info('Sending request')
              _ = self.client.call(Empty.Request())
              self.get_logger().info('Received response')


      if __name__ == '__main__':
          rclpy.init()
          node = CallbackGroupDemo()
          executor = MultiThreadedExecutor()
          executor.add_node(node)

          try:
              node.get_logger().info('Beginning client, shut down with CTRL-C')
              executor.spin()
          except KeyboardInterrupt:
              node.get_logger().info('Keyboard interrupt, shutting down.\n')
          node.destroy_node()
          rclpy.shutdown()

在上面的代码中，客户端节点的构造函数包含对 service 客户端和 timer 回调组的配置。
默认设置如上（都为 ``nullptr`` / ``None``）。
此时 timer 和客户端都将使用节点的默认 Mutually Exclusive Callback Group。

问题所在
^^^^^^^^^^^

我们使用 1 秒的定时器进行服务调用，预期结果是服务每秒被调用一次，客户端会收到响应并打印 ``Received response``。
但是，如果我们在终端中运行服务器和客户端节点，我们会得到以下输出。

.. tabs::

  .. group-tab:: Client

    .. code-block:: console

      [INFO] [1653034371.758739131] [client_node]: Starting client node, shut down with CTRL-C
      [INFO] [1653034372.755865649] [client_node]: Sending request
      ^C[INFO] [1653034398.161674869] [client_node]: Keyboard interrupt, shutting down.

  .. group-tab:: Server

    .. code-block:: console

      [INFO] [1653034355.308958238] [service_node]: Starting server node, shut down with CTRL-C
      [INFO] [1653034372.758197320] [service_node]: Received request, responding...
      ^C[INFO] [1653034416.021962246] [service_node]: Keyboard interrupt, shutting down.

可以看到，service 并没有如期每秒被调用一次，而是在第一次调用后就没有再次调用。
看起来似乎客户端节点卡住了，不再发出调用。这就是说，回调在执行中死锁了！

这个现象的原因是，定时器回调和客户端都使用了相同的 Mutually Exclusive Callback Group（节点的默认回调组）。
当发出请求时，客户端将给 Future 对象（Python 版本中的 call 方法内部隐藏的 Future 对象）的回调组设置为自己的回调组，
这个 Future 的 done-callback 需要在调用的结果返回时执行。
但是因为这个 done-callback 和定时器回调在同一个 Mutually Exclusive 组中，而定时器回调仍没有返回（等待服务调用的结果），
所以 done-callback 永远不会执行。
这个卡住的定时器回调也会导致回调组里面的其他操作也不能执行，所以定时器不会被第二次触发。

译者注：Mutually Exclusive Callback Group 中的回调函数是一个一个挨着执行的，只有前一个回调函数执行完了，下一个回调函数才能执行。
那么让我们考虑上面的例子。最开始 gruop 需要运行的回调函数列表里面是空的。一开始，我们往里面添加了一个 timer 的回调函数，称之为 timer_callback。
在 timer_callback 中，我们运行了一个发出请求的函数，叫它 request_call, request_call 会向 group 内添加一个需要执行的回调函数,叫它 response_callback.
此时 group 里面有两个回调函数，前面的是 timer_callback，后面的是 response_callback。
request_call 只有得到 response_callback 的结果才会返回。
但是要想执行到 response_callback，需要先执行完 timer_callback。可是 timer_callback 里面的 request_call 又在等待 response_callback 的结果。
这样就形成了一个死锁(deadlock)。


解决方法
^^^^^^^^

所幸这个问题很容易解决 - 例如 - 通过将定时器和客户端分配给不同的回调组。
那么让我们把客户端节点的构造函数的前两行改成这样（其他内容保持不变）：

.. tabs::

  .. group-tab:: C++

    .. code-block:: cpp

      client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  .. group-tab:: Python

    .. code-block:: python

      client_cb_group = MutuallyExclusiveCallbackGroup()
      timer_cb_group = MutuallyExclusiveCallbackGroup()

现在我们得到了预期的结果，即定时器会重复触发，每次请求也都会得到结果：

.. tabs::

  .. group-tab:: Client

    .. code-block:: console

      [INFO] [1653067523.431731177] [client_node]: Starting client node, shut down with CTRL-C
      [INFO] [1653067524.431912821] [client_node]: Sending request
      [INFO] [1653067524.433230445] [client_node]: Received response
      [INFO] [1653067525.431869330] [client_node]: Sending request
      [INFO] [1653067525.432912803] [client_node]: Received response
      [INFO] [1653067526.431844726] [client_node]: Sending request
      [INFO] [1653067526.432893954] [client_node]: Received response
      [INFO] [1653067527.431828287] [client_node]: Sending request
      [INFO] [1653067527.432848369] [client_node]: Received response
      ^C[INFO] [1653067528.400052749] [client_node]: Keyboard interrupt, shutting down.

  .. group-tab:: Server

    .. code-block:: console

      [INFO] [1653067522.052866001] [service_node]: Starting server node, shut down with CTRL-C
      [INFO] [1653067524.432577720] [service_node]: Received request, responding...
      [INFO] [1653067525.432365009] [service_node]: Received request, responding...
      [INFO] [1653067526.432300261] [service_node]: Received request, responding...
      [INFO] [1653067527.432272441] [service_node]: Received request, responding...
      ^C[INFO] [1653034416.021962246] [service_node]: KeyboardInterrupt, shutting down.

这时候你可能会想：“那是不是只要不是使用节点默认的回调函数组就行了？”
其实并不是这样，如果你尝试把默认的回调组替换成其他的 Mutually Exclusive group ，问题也依然存在。
以下这样的配置也会导致之前发现的死锁。

.. tabs::

  .. group-tab:: C++

    .. code-block:: cpp

      client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      timer_cb_group_ = client_cb_group_;

  .. group-tab:: Python

    .. code-block:: python

      client_cb_group = MutuallyExclusiveCallbackGroup()
      timer_cb_group = client_cb_group

这是因为在这种情况下，定时器和客户端还是属于同一个 Mutually Exclusive group。
如下的这些配置组合才能让 timer 和 service 的表现如我们预期一般。

.. tabs::

  .. group-tab:: C++

    .. code-block:: cpp

      client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
      timer_cb_group_ = client_cb_group_;

    or

    .. code-block:: cpp

      client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      timer_cb_group_ = nullptr;

    or

    .. code-block:: cpp

      client_cb_group_ = nullptr;
      timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    or

    .. code-block:: cpp

      client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
      timer_cb_group_ = nullptr;

  .. group-tab:: Python

    .. code-block:: python

      client_cb_group = ReentrantCallbackGroup()
      timer_cb_group = client_cb_group

    or

    .. code-block:: python

      client_cb_group = MutuallyExclusiveCallbackGroup()
      timer_cb_group = None

    or

    .. code-block:: python

      client_cb_group = None
      timer_cb_group = MutuallyExclusiveCallbackGroup()

    or

    .. code-block:: python

      client_cb_group = ReentrantCallbackGroup()
      timer_cb_group = None
