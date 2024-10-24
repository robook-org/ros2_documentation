.. redirect-from::

    Tutorials/Writing-A-Simple-Py-Service-And-Client

.. _PySrvCli:

服务与客户端(Service and Client)(Python 实现)
============================================

**目标:** 使用 Python 创建并运行服务和客户端节点.

**教程等级:** 初级

**预计时长:** 20 分钟

.. contents:: Contents
   :depth: 2
   :local:

背景
----------

在 :doc:`节点 <../Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes>` 之间使用 :doc:`服务 <../Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services>` 通信时，发送数据请求的节点称为客户端节点，而响应请求的节点称为服务节点。
请求和响应(request and response)的结构由 ``.srv`` 文件决定。

在这里使用的示例是一个简单的整数加法系统；一个节点请求两个整数的和，另一个节点返回结果。

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

  ros2 pkg create --build-type ament_python --license Apache-2.0 py_srvcli --dependencies rclpy example_interfaces

终端会返回一个消息，确认 ``py_srvcli`` 包及其所有必要的文件和文件夹已创建。
``--dependencies`` 参数会自动将必要的依赖行添加到 ``package.xml``。
``example_interfaces`` 中包含 `.srv 文件 <https://github.com/ros2/example_interfaces/blob/{REPOS_FILE_BRANCH}/srv/AddTwoInts.srv>`__ ，在这个文件中定义请求和响应的结构:

.. code-block:: console

    int64 a
    int64 b
    ---
    int64 sum

前两行定义请求，横线下面定义响应。

1.1 更新 ``package.xml``
~~~~~~~~~~~~~~~~~~~~~~~~~~

因为在包创建过程中使用了 ``--dependencies`` 选项，所以不需要手动将依赖项添加到 ``package.xml``。

和之前一样，记得向 ``package.xml`` 添加描述、维护者邮箱和姓名，以及许可信息。

.. code-block:: xml

  <description>Python client server tutorial</description>
  <maintainer email="you@email.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

1.2 更新 ``setup.py``
~~~~~~~~~~~~~~~~~~~~~~~

在 ``setup.py`` 文件中的 ``maintainer``、``maintainer_email``、``description`` 和 ``license`` 中添加同样的信息：

.. code-block:: python

    maintainer='Your Name',
    maintainer_email='you@email.com',
    description='Python client server tutorial',
    license='Apache License 2.0',

2 编写服务节点
^^^^^^^^^^^^^^^^^^^^^^^^

在 ``ros2_ws/src/py_srvcli/py_srvcli`` 目录中创建一个名为 ``service_member_function.py`` 的新文件，并粘贴以下代码：

.. code-block:: python

  from example_interfaces.srv import AddTwoInts

  import rclpy
  from rclpy.node import Node


  class MinimalService(Node):

      def __init__(self):
          super().__init__('minimal_service')
          self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

      def add_two_ints_callback(self, request, response):
          response.sum = request.a + request.b
          self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

          return response


  def main():
      rclpy.init()

      minimal_service = MinimalService()

      rclpy.spin(minimal_service)

      rclpy.shutdown()


  if __name__ == '__main__':
      main()

2.1 检查代码
~~~~~~~~~~~~~~~~~~~~

第一行的 ``import`` 语句从 ``example_interfaces`` 包中导入 ``AddTwoInts`` 服务类型。
后续两行导入 ROS 2 Python 客户端库，特别是 ``Node`` class 。

.. code-block:: python

  from example_interfaces.srv import AddTwoInts

  import rclpy
  from rclpy.node import Node

``MinimalService`` 构造函数使用 ``minimal_service`` 作为节点名称。
然后，它创建一个服务并定义类型、名称和回调。

.. code-block:: python

  def __init__(self):
      super().__init__('minimal_service')
      self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

服务回调的定义接收请求数据，对其求和，然后将求和的结果作为响应返回。

.. code-block:: python

  def add_two_ints_callback(self, request, response):
      response.sum = request.a + request.b
      self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

      return response

最后，主类初始化 ROS 2 Python 客户端库，实例化 ``MinimalService`` 类以创建服务节点，并运行节点以处理回调。

2.2 添加 entry point
~~~~~~~~~~~~~~~~~~~~~~

你必须在 ``ros2_ws/src/py_srvcli`` 目录中的 ``setup.py`` 文件中添加 entry point ，才能用 ``ros2 run`` 命令运行你的节点。

将下面这行内容添加到 ``'console_scripts':`` 中：

.. code-block:: python

  'service = py_srvcli.service_member_function:main',

3 编写客户端节点
^^^^^^^^^^^^^^^^^^^^^^^

在 ``ros2_ws/src/py_srvcli/py_srvcli`` 目录中创建一个名为 ``client_member_function.py`` 的新文件，并粘贴以下代码：

.. code-block:: python

  import sys

  from example_interfaces.srv import AddTwoInts
  import rclpy
  from rclpy.node import Node


  class MinimalClientAsync(Node):

      def __init__(self):
          super().__init__('minimal_client_async')
          self.cli = self.create_client(AddTwoInts, 'add_two_ints')
          while not self.cli.wait_for_service(timeout_sec=1.0):
              self.get_logger().info('service not available, waiting again...')
          self.req = AddTwoInts.Request()

      def send_request(self, a, b):
          self.req.a = a
          self.req.b = b
          return self.cli.call_async(self.req)


  def main():
      rclpy.init()

      minimal_client = MinimalClientAsync()
      future = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
      rclpy.spin_until_future_complete(minimal_client, future)
      response = future.result()
      minimal_client.get_logger().info(
          'Result of add_two_ints: for %d + %d = %d' %
          (int(sys.argv[1]), int(sys.argv[2]), response.sum))

      minimal_client.destroy_node()
      rclpy.shutdown()


  if __name__ == '__main__':
      main()


3.1 检查代码
~~~~~~~~~~~~~~~~~~~~

和服务端代码一样，首先 ``import`` 必要的库。

.. code-block:: python

  import sys

  from example_interfaces.srv import AddTwoInts
  import rclpy
  from rclpy.node import Node

``MinimalClientAsync`` 构造函数使用 ``minimal_client_async`` 作为节点名称。
客户端使用和服务节点匹配的类型和名称，两端必须匹配才能通信。
构造函数中的 ``while`` 循环每秒检查一次是否有匹配客户端的服务。
最后创建一个新的 ``AddTwoInts`` 请求。

.. code-block:: python

  def __init__(self):
      super().__init__('minimal_client_async')
      self.cli = self.create_client(AddTwoInts, 'add_two_ints')
      while not self.cli.wait_for_service(timeout_sec=1.0):
          self.get_logger().info('service not available, waiting again...')
      self.req = AddTwoInts.Request()

下面是 ``send_request`` 方法，它将发送请求并返回一个 future ，可以传递给 ``spin_until_future_complete``：

.. code-block:: python

  def send_request(self, a, b):
      self.req.a = a
      self.req.b = b
      return self.cli.call_async(self.req)

最后是 ``main`` 方法，它构造一个 ``MinimalClientAsync`` 对象，使用传入的命令行参数发送请求，调用 ``spin_until_future_complete`` 并记录结果：

.. code-block:: python

  def main():
      rclpy.init()

      minimal_client = MinimalClientAsync()
      future = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
      rclpy.spin_until_future_complete(minimal_client, future)
      response = future.result()
      minimal_client.get_logger().info(
          'Result of add_two_ints: for %d + %d = %d' %
          (int(sys.argv[1]), int(sys.argv[2]), response.sum))

      minimal_client.destroy_node()
      rclpy.shutdown()


3.2 添加 entry point
~~~~~~~~~~~~~~~~~~~~~~

和服务节点一样，你必须在 ``setup.py`` 文件中添加 entry point 才能运行客户端节点。

``setup.py`` 文件的 ``entry_points`` 应该如下所示：

.. code-block:: python

  entry_points={
      'console_scripts': [
          'service = py_srvcli.service_member_function:main',
          'client = py_srvcli.client_member_function:main',
      ],
  },

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


返回到工作空间的根目录， ``ros2_ws`` ，构建新包：

.. code-block:: console

  colcon build --packages-select py_srvcli

构建完成后，打开一个新终端，进入 ``ros2_ws`` 并 source setup 文件：

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

  ros2 run py_srvcli service

服务节点将等待客户端的请求。

打开另一个终端并再次 source ``ros2_ws`` 中的 setup 文件。
运行客户端节点，后面跟着两个整数，用空格分隔：

.. code-block:: console

  ros2 run py_srvcli client 2 3

如果你发了 ``2`` 和 ``3``，客户端将收到这样的响应：

.. code-block:: console

  [INFO] [minimal_client_async]: Result of add_two_ints: for 2 + 3 = 5

返回到服务节点运行的终端。
你会看到它在收到请求时发布了日志消息：

.. code-block:: console

  [INFO] [minimal_service]: Incoming request
  a: 2 b: 3

输入 ``Ctrl+C`` 服务节点。


总结
-------

你创建了两个节点，用于通过服务请求和响应数据。
将它们的依赖项和可执行文件添加到包配置文件中，以便构建和运行并观察到服务/客户端系统的的工作情况。

下一步
----------

在最近的几个教程中，你一直在使用接口(interfaces)在 topic 和服务间传递数据。
接下来，你将学习如何 :doc:`创建自定义接口 <./Custom-ROS2-Interfaces>`。

相关内容
---------------

* 有几种方法可以在 Python 中编写服务和客户端；查看 `ros2/examples <https://github.com/ros2/examples/tree/{REPOS_FILE_BRANCH}/rclpy/services>`_ 中的 ``minimal_client`` 和 ``minimal_service`` 。

* 在这个教程中，你使用了客户端节点中的 ``call_async()`` API 来调用服务。
  Python 中还有另一种服务调用 API，称为同步调用。
  我们不建议使用同步调用，但如果你想了解更多，请阅读 :doc:`同步 vs. 异步客户端 <../../How-To-Guides/Sync-Vs-Async>` 指南。
