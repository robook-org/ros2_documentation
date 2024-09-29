.. redirect-from::

   Concepts/About-Executors

Executors
=========

.. contents:: Table of Contents
   :local:

概览
--------

ROS 2 中的执行管理(Execution management)由 Executors 处理。
Executors 使用底层操作系统的一个或多个线程来调用 subscriptions 、定时器(timer)、service 服务器、action 服务器等的回调函数，以处理传入的消息和事件。
与 ROS 1 中的 spin 机制类似，显式的 Executor 类(在 rclcpp 中的 `executor.hpp <https://github.com/ros2/rclcpp/blob/{REPOS_FILE_BRANCH}/rclcpp/include/rclcpp/executor.hpp>`_ ,在 rclpy 中的 `executors.py <https://github.com/ros2/rclpy/blob/{REPOS_FILE_BRANCH}/rclpy/rclpy/executors.py>`_ 或是在 rclc 的 `executor.h <https://github.com/ros2/rclc/blob/master/rclc/include/rclc/executor.h>`_  ) 提供了更多的控制能力。

在下文中，我们将重点关注 C++ 客户端库 *rclcpp*。

基本使用方式
----------------

在最简单的情况下，主线程用于通过调用 ``rclcpp::spin(..)`` 处理节点的传入消息和事件：

.. code-block:: cpp

   int main(int argc, char* argv[])
   {
      // 做一些初始化.
      rclcpp::init(argc, argv);
      ...

      // 实例化一个 node.
      rclcpp::Node::SharedPtr node = ...

      // 运行 executor.
      rclcpp::spin(node);

      // 关闭并退出.
      ...
      return 0;
   }

对 ``spin(node)`` 的调用基本上展开为一个单线程执行器的实例化和调用，这是最简单的执行器：

.. code-block:: cpp

   rclcpp::executors::SingleThreadedExecutor executor;
   executor.add_node(node);
   executor.spin();

通过调用执行器实例的 ``spin()``，当前线程开始查询 rcl 和中间件层的传入消息和其他事件，并调用相应的回调函数，直到节点关闭。
为了不与中间件的 QoS 设置相冲突，传入的消息不会在客户端库层的队列中存储，而是保留在中间件中，直到被回调函数取出进行处理。
(这是与 ROS 1 的一个关键区别。)
每个队列都有一个二进制标志，用于通知 executor 在中间件层的等候区(wait set)中有可用的消息。
这个等候区还用于检测定时器是否到期。

.. image:: ../images/executors_basic_principle.png

单线程执行器也被容器进程用于 :doc:`components <./About-Composition>`，即在所有节点在没有显式地被主函数的创建和执行的情况下。

Executors 类型
------------------

目前，rclcpp 提供了三种 Executor 类型，它们都是从一个共同的父类派生的：

.. graphviz::

   digraph Flatland {

      Executor -> SingleThreadedExecutor [dir = back, arrowtail = empty];
      Executor -> MultiThreadedExecutor [dir = back, arrowtail = empty];
      Executor -> StaticSingleThreadedExecutor [dir = back, arrowtail = empty];
      Executor  [shape=polygon,sides=4];
      SingleThreadedExecutor  [shape=polygon,sides=4];
      MultiThreadedExecutor  [shape=polygon,sides=4];
      StaticSingleThreadedExecutor  [shape=polygon,sides=4];

      }

*Single-Threaded Executor* 在一个线程中处理所有消息和事件。
*Multi-Threaded Executor* 创建可配置数量的线程，以允许并行处理多个消息或事件。
*Static Single-Threaded Executor* 通过扫描节点结构优化了运行时的开销，例如 subscriptions 、timers、service 服务器、action 服务器等。
它只在节点添加时执行一次此扫描，而其他两个执行器会定期扫描这些变动。
因此，Static Single-Threaded Executor 只能与在初始化期间创建所有 subscriptions 、timers 等的节点一起使用。

所有三个执行器都可以通过为每个节点调用 ``add_node(..)`` 来与多个节点一起使用。

.. code-block:: cpp

   rclcpp::Node::SharedPtr node1 = ...
   rclcpp::Node::SharedPtr node2 = ...
   rclcpp::Node::SharedPtr node3 = ...

   rclcpp::executors::StaticSingleThreadedExecutor executor;
   executor.add_node(node1);
   executor.add_node(node2);
   executor.add_node(node3);
   executor.spin();

在上面的示例中，Static Single-Threaded Executor 的一个线程用于一起服务三个节点。
在 Multi-Threaded Executor 的情况下，实际的并行性取决于 callback groups。

回调函数组(Callback groups)
-----------------------------------

ROS 2 允许将节点的回调函数组织成组。
在 rclcpp 中，这样的 *callback group* 可以通过 Node 类的 ``create_callback_group`` 函数创建。
在 rclpy 中，可以通过调用特定回调组类型的构造函数来完成相同的操作。
回调组必须在节点的整个执行过程中存储在节点中（例如作为类成员），否则 executor 将无法触发回调。
然后，可以在创建 subscription 、timer 等时指定此回调组 - 例如为 subscription 指定：

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

所有没有指定回调组的 subscriptions、定时器等都被分配给 *默认回调组(default callback group)* 。
在 rclcpp 中，可以通过 ``NodeBaseInterface::get_default_callback_group()`` 查询默认回调组，在 rclpy 中可以通过 ``Node.default_callback_group`` 查询。

回调组有两种类型是必须在实例化时指定的：

* *Mutually exclusive:* 此组的回调不能并行执行。
* *Reentrant:* 此组的回调可以并行执行。

不同回调组的回调函数可能总是在并行执行。
Multi-Threaded Executor 使用其自身的许多线程作为一个线程池，以并行处理尽可能多的回调。
了解如何高效使用回调组的提示，请参阅 :doc:`使用 Callback Groups <../../How-To-Guides/Using-callback-groups>`。

rclcpp 中的 Executor 基类还有函数 ``add_callback_group(..)``，它允许将回调组分发给不同的 Executors。
这样，可以通过操作回调组的优先级来配置底层线程的调度。
例如，控制循环的订阅和定时器可以优先于节点的所有其他订阅和标准服务。
`examples_rclcpp_cbg_executor package <https://github.com/ros2/examples/tree/{REPOS_FILE_BRANCH}/rclcpp/executors/cbg_executor>`_ 提供了一个示例。


调度策略(Scheduling semantics)
--------------------------------

(译者注： semantics 在某些领域会被翻译成“语义”，在这里这一词汇是表示一种符号所对应的结构和含义，也就是事实上的方式、方法或策略。)

当回调函数的处理时间短于消息和事件发生的周期时，执行器基本上按照 FIFO 的顺序处理它们。（译者注：FIFO， first-in first-out，先进先出，一种处理数据的顺序，说白了就是先来的数据先处理。）
然而，如果某些回调的处理时间较长，消息和事件将排队在栈的较低层。
等候区机制只将非常少的有关队列的信息报告给 executor。
具体来说，它只报告有没有某个 topic 的消息。
executor 使用这些信息来以循环方式处理消息（包括 services 和 actions）- 但不是按照 FIFO 的顺序。
下图展示了这种调度策略。

.. image:: ../images/executors_scheduling_semantics.png

这种策略首次有 `Casini 等人在 ECRTS 2019 <https://drops.dagstuhl.de/opus/volltexte/2019/10743/pdf/LIPIcs-ECRTS-2019-6.pdf>`_ 中描述。
(注：这篇论文也解释了定时器事件优先于所有其他消息。 `这种优先级在 Eloquent 中被移除了。 <https://github.com/ros2/rclcpp/pull/841>`_ )

展望
-------

尽管 rclcpp 的三种 Executor 对于大多数应用都能很好地工作，但有一些问题使它们不适合实时应用，这些应用需要恰当定义的执行时间、确定性和对执行顺序的自定义控制。
以下是其中一些尚未解决问题的总结：

1. 复杂和混合的调度策略。
   理想情况下，您希望有明确定义的调度语义来执行正式的时间分析。
2. 回调可能受到优先级反转(priority inversion)的影响。
   优先级较高的回调可能被优先级较低的回调阻塞。
3. 无法显式控制回调的执行顺序。
4. 没有针对特定 topic 触发的内置控制。

此外， executor 在 CPU 和内存使用方面的开销相当大。
Static Single-Threaded Executor 大大减少了这种开销，但对于某些应用来说可能还不够。

这些问题的处理已经有了一些进展：

* `rclcpp WaitSet <https://github.com/ros2/rclcpp/blob/{REPOS_FILE_BRANCH}/rclcpp/include/rclcpp/wait_set.hpp>`_: The ``WaitSet`` class of rclcpp allows waiting directly on subscriptions, timers, service servers, action servers, etc. instead of using an Executor.
  It can be used to implement deterministic, user-defined processing sequences, possibly processing multiple messages from different subscriptions together.
  The `examples_rclcpp_wait_set package <https://github.com/ros2/examples/tree/{REPOS_FILE_BRANCH}/rclcpp/wait_set>`_ provides several examples for the use of this user-level wait set mechanism.
* `rclc Executor <https://github.com/ros2/rclc/blob/master/rclc/include/rclc/executor.h>`_: This Executor from the C Client Library *rclc*, developed for micro-ROS, gives the user fine-grained control over the execution order of callbacks and allows for custom trigger conditions to activate callbacks.
  Furthermore, it implements ideas of the Logical Execution Time (LET) semantics.

更多信息
-------------------

* Michael Pöhnl et al.: `"ROS 2 Executor: How to make it efficient, real-time and deterministic?" <https://www.apex.ai/roscon-21>`_. Workshop at ROS World 2021. Virtual event. 19 October 2021.
* Ralph Lange: `"Advanced Execution Management with ROS 2" <https://www.youtube.com/watch?v=Sz-nllmtcc8&t=109s>`_. ROS Industrial Conference. Virtual event. 16 December 2020.
* Daniel Casini, Tobias Blass, Ingo Lütkebohle, and Björn Brandenburg: `“Response-Time Analysis of ROS 2 Processing Chains under Reservation-Based Scheduling” <https://drops.dagstuhl.de/opus/volltexte/2019/10743/pdf/LIPIcs-ECRTS-2019-6.pdf>`_, Proc. of 31st ECRTS 2019, Stuttgart, Germany, July 2019.
