.. redirect-from::

  Guides/Sync-Vs-Async
  Tutorials/Sync-Vs-Async

.. _SyncAsync:

同步 vs. 异步 service 客户端
============================================

**教程等级:** 中级

**预计时长:** 10 分钟

.. contents:: Contents
   :depth: 2
   :local:


简介
------------

本指南旨在指出用户关于 Python 同步服务客户端 ``call()`` API 相关的风险。
在调用服务时，同步调用很容易导致死锁，因此我们不建议使用 ``call()``。

我们提供了一个示例，展示有经验的用户如何正确使用 ``call()``，以及如何避免潜在的死锁。
我们也列出了在使用同步调用中可能产生死锁的情景。

由于我们建议避免同步调用，本指南还将介绍推荐的替代方案异步调用（``call_async()``）的特性和用法。

C++ 服务调用 API 仅在异步中可用，因此本指南中的比较和示例适用于 Python 服务和客户端。
这里给出的异步定义大部分适用于 C++，也存在一些例外。

1 同步调用
-------------------

同步客户端在向服务发送请求时会阻塞调用线程，直到收到响应；在调用期间，该线程上不会发生其他事情。
调用可能需要很长的时间才能完成。
完成后，响应直接返回给客户端。

以下是正确从客户端节点执行同步服务调用的示例，类似于 :doc:`简单服务和客户端 <../Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client>` 教程中的 async 节点。

.. code-block:: python

  import sys
  from threading import Thread

  from example_interfaces.srv import AddTwoInts
  import rclpy
  from rclpy.node import Node

  class MinimalClientSync(Node):

      def __init__(self):
          super().__init__('minimal_client_sync')
          self.cli = self.create_client(AddTwoInts, 'add_two_ints')
          while not self.cli.wait_for_service(timeout_sec=1.0):
              self.get_logger().info('service not available, waiting again...')
          self.req = AddTwoInts.Request()

      def send_request(self):
          self.req.a = int(sys.argv[1])
          self.req.b = int(sys.argv[2])
          return self.cli.call(self.req)
          # 只有 rclpy.spin() 从一个单独的线程中被调用时，这个函数才会实际生效。
          # 其他的配置方式，比如在 main() 中稍后调用 rclpy.spin() 或者在定时器回调中调用这个方法，都会导致死锁。

  def main():
      rclpy.init()

      minimal_client = MinimalClientSync()

      spin_thread = Thread(target=rclpy.spin, args=(minimal_client,))
      spin_thread.start()

      response = minimal_client.send_request()
      minimal_client.get_logger().info(
          'Result of add_two_ints: for %d + %d = %d' %
          (minimal_client.req.a, minimal_client.req.b, response.sum))

      minimal_client.destroy_node()
      rclpy.shutdown()


  if __name__ == '__main__':
      main()

请注意，客户端在 ``main()`` 中调用 ``rclpy.spin`` 时使用了一个单独的线程。
``send_request`` 和 ``rclpy.spin`` 都是阻塞的，因此它们需要在不同的线程中运行。

1.1 同步死锁
-----------------

同步的 ``call()`` API 有几种方式可能导致死锁。

如上面的示例中注释所述，如果没有创建一个单独的线程来运行 ``rclpy``，就会导致死锁。
当客户端因为等待响应而阻塞了线程，但响应只能在同一线程上返回时，客户端永远不会停止等待，也不会发生其他事情。

另一个死锁的原因是在订阅、定时器回调或服务回调中同步调用服务，阻塞了 ``rclpy.spin``。
例如，如果同步客户端的 ``send_request`` 放在回调中：

.. code-block:: python

  def trigger_request(msg):
      response = minimal_client.send_request()  # 这会导致死锁
      minimal_client.get_logger().info(
          'Result of add_two_ints: for %d + %d = %d' %
          (minimal_client.req.a, minimal_client.req.b, response.sum))
  subscription = minimal_client.create_subscription(String, 'trigger', trigger_request, 10)

  rclpy.spin(minimal_client)

死锁发生是因为 ``rclpy.spin`` 不会打断回调中的 ``send_request`` 调用。
在一般情况下，回调应该只执行轻量级和快速的操作。

.. warning::

  当死锁发生时，你将不会收到任何指示服务被阻塞。
  不会有警告或异常抛出，堆栈跟踪中也没有指示，调用也不会失败。

2 异步调用
--------------------

在 ``rclpy`` 中，异步调用是完全安全的，也是调用服务的推荐方法。
它们可以在任何地方创建，而不会像同步调用那样阻塞其他 ROS 或非 ROS 进程。

异步客户端在向服务发送请求后会立即返回 ``future``，这个值表示调用和响应是否完成（而不是响应本身）。
可以随时通过返回的 ``future`` 查询响应结果。

由于发送请求不会阻塞任何东西，可以使用在同一线程中的循环来同时 spin ``rclpy`` 和检查 ``future``，例如：

.. code-block:: python

    while rclpy.ok():
        rclpy.spin_once(node)
        if future.done():
            #Get response

:doc:`服务和客户端 <../Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client>` 教程中的 Python 示例演示了如何执行异步服务调用并使用循环检索 ``future``。

也可以通过定时器或回调(比如 `这个例子 <https://github.com/ros2/examples/blob/{REPOS_FILE_BRANCH}/rclpy/services/minimal_client/examples_rclpy_minimal_client/client_async_callback.py>`_ )、一个专用的线程或者其他方法检查 ``future`` 。
这取决于你作为请求的发起方想要如何存储 ``future``、检查其状态并检查响应结果。

总结
-------

不建议实现同步服务客户端。
它们容易导致死锁，但死锁发生时不会提供任何指示。
如果必须使用同步调用，本指南中 ``1 同步调用`` 部分的示例是一个安全的方法。
你还应该了解 `1.1 同步死锁`_ 部分中列出的导致死锁的因素。
我们建议使用异步服务客户端。
