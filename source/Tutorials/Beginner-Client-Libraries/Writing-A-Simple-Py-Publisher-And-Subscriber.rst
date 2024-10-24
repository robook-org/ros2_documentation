.. redirect-from::

    Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber

.. _PyPubSub:

发布者和订阅者(publisher & subscriber) (Python 实现)
==================================================

**目标:** 用 Python 创建并运行 publisher & subscriber 节点.

**教程等级:** 初级

**预计时长:** 20 分钟

.. contents:: Contents
   :depth: 2
   :local:

背景
----------

在此教程中，你需要创建 :doc:`节点 <../Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes>`，这些节点通过 :doc:`topic <../Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics>` 以字符串消息的形式传递信息。
这里的例子是一个简单的 "talker" 和 "listener" 系统; 一个节点发布数据，另一个订阅主题以接收数据.

这些示例中使用的代码可以在 `这里 <https://github.com/ros2/examples/tree/{REPOS_FILE_BRANCH}/rclpy/topics>`__ 找到.

前提条件
-------------

在此之前，你已经学会了如何 :doc:`创建工作空间 <./Creating-A-Workspace/Creating-A-Workspace>` 和 :doc:`创建包 <./Creating-Your-First-ROS2-Package>`.

此外你最好对 Python 有基本的了解，但也不是必须的。

任务
-----

1 创建包
^^^^^^^^^^^^^^^^^^

打开一个新的终端， :doc:`source 你的 ROS 2 安装 <../Beginner-CLI-Tools/Configuring-ROS2-Environment>`，这样 ``ros2`` 命令就能用了。

进入之前创建的 ``ros2_ws`` 目录。

回想一下，包应该在 ``src`` 目录中创建，而不是工作空间的根目录。
所以，进入 ``ros2_ws/src``，运行包创建命令:

.. code-block:: console

  ros2 pkg create --build-type ament_python --license Apache-2.0 py_pubsub

你的终端会返回一个消息，确认了包 ``py_pubsub`` 及其所有必要的文件和文件夹已经创建。

2 编写发布者节点
^^^^^^^^^^^^^^^^^^^^^^^^^^

进入 ``ros2_ws/src/py_pubsub/py_pubsub`` 目录。
回想一下，这个目录是一个与 ROS 2 包同名的 `Python 包 <https://docs.python.org/3/tutorial/modules.html#packages>`__。

输入以下命令下载发布者代码样例:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        wget https://raw.githubusercontent.com/ros2/examples/{REPOS_FILE_BRANCH}/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py

   .. group-tab:: macOS

      .. code-block:: console

        wget https://raw.githubusercontent.com/ros2/examples/{REPOS_FILE_BRANCH}/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py

   .. group-tab:: Windows

      In a Windows command line prompt:

      .. code-block:: console

            curl -sk https://raw.githubusercontent.com/ros2/examples/{REPOS_FILE_BRANCH}/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py -o publisher_member_function.py

      Or in powershell:

      .. code-block:: console

            curl https://raw.githubusercontent.com/ros2/examples/{REPOS_FILE_BRANCH}/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py -o publisher_member_function.py

现在 ``__init__.py`` 旁边会有一个新文件，名为 ``publisher_member_function.py``。

用你喜欢的文本编辑器打开这个文件。

.. code-block:: python

  import rclpy
  from rclpy.node import Node

  from std_msgs.msg import String


  class MinimalPublisher(Node):

      def __init__(self):
          super().__init__('minimal_publisher')
          self.publisher_ = self.create_publisher(String, 'topic', 10)
          timer_period = 0.5  # seconds
          self.timer = self.create_timer(timer_period, self.timer_callback)
          self.i = 0

      def timer_callback(self):
          msg = String()
          msg.data = 'Hello World: %d' % self.i
          self.publisher_.publish(msg)
          self.get_logger().info('Publishing: "%s"' % msg.data)
          self.i += 1


  def main(args=None):
      rclpy.init(args=args)

      minimal_publisher = MinimalPublisher()

      rclpy.spin(minimal_publisher)

      # Destroy the node explicitly
      # (optional - otherwise it will be done automatically
      # when the garbage collector destroys the node object)
      minimal_publisher.destroy_node()
      rclpy.shutdown()


  if __name__ == '__main__':
      main()


2.1 检查代码
~~~~~~~~~~~~~~~~~~~~

文件中，最前面注释之后的第一行代码导入 ``rclpy``，这样就可以使用它的 ``Node`` 类。

.. code-block:: python

  import rclpy
  from rclpy.node import Node

接下来的语句导入了内置的字符串消息类型，节点使用它来组织它通过主题传递的数据。

.. code-block:: python

  from std_msgs.msg import String

上面这几行表明了节点的依赖关系。
回想一下，这些依赖关系必须添加到 ``package.xml`` 中，这是下一节你要做的事情。

接下来，创建 ``MinimalPublisher`` 类，它继承自 ``Node`` （或者叫作“是 ``Node`` 的子类”）。

.. code-block:: python

  class MinimalPublisher(Node):

后面的代码是类的构造函数的定义。

``super().__init__`` 调用 ``Node`` 类的构造函数，并传递你的节点名称，这里是 ``minimal_publisher``。

``create_publisher`` 声明了节点发布 ``String`` 类型的消息（从 ``std_msgs.msg`` 模块导入），发布到名为 ``topic`` 的主题上，"queue size" 是 10。
队列大小是一个必需的 QoS（服务质量）设置，它限制了如果订阅者接收消息不够快时的最大排队消息数量。

接下来，创建一个定时器，每 0.5 秒调用一次回调函数。
``self.i`` 是回调函数中的一个计数变量。

.. code-block:: python

  def __init__(self):
      super().__init__('minimal_publisher')
      self.publisher_ = self.create_publisher(String, 'topic', 10)
      timer_period = 0.5  # seconds
      self.timer = self.create_timer(timer_period, self.timer_callback)
      self.i = 0

回调函数 ``timer_callback`` 创建一个消息，将计数变量加一，然后使用 ``get_logger().info`` 将其发布到控制台。

.. code-block:: python

  def timer_callback(self):
      msg = String()
      msg.data = 'Hello World: %d' % self.i
      self.publisher_.publish(msg)
      self.get_logger().info('Publishing: "%s"' % msg.data)
      self.i += 1

最后，主函数就定义好了：

.. code-block:: python

  def main(args=None):
      rclpy.init(args=args)

      minimal_publisher = MinimalPublisher()

      rclpy.spin(minimal_publisher)

      # Destroy the node explicitly
      # (optional - otherwise it will be done automatically
      # when the garbage collector destroys the node object)
      minimal_publisher.destroy_node()
      rclpy.shutdown()

先初始化 ``rclpy`` 库，然后创建节点，最后让节点运行(spin)起来。

2.2 添加依赖
~~~~~~~~~~~~~~~~~~~~

回到 ``ros2_ws/src/py_pubsub`` 目录，这里已经创建好了 ``setup.py``、 ``setup.cfg`` 和 ``package.xml`` 文件。

打开 ``package.xml``。

如前面的 :doc:`教程 <./Creating-Your-First-ROS2-Package>` 中提到的，确保填写了 ``<description>``, ``<maintainer>`` 和 ``<license>`` 标签:

.. code-block:: xml

  <description>Examples of minimal publisher/subscriber using rclpy</description>
  <maintainer email="you@email.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

在上面这些行的后面，添加以下依赖，对应于节点的导入语句:

.. code-block:: xml

  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>

这样声明了当代码执行时，需要 ``rclpy`` 和 ``std_msgs``。

记得保存文件。

2.3 添加 entry point
~~~~~~~~~~~~~~~~~~~~~~

打开 ``setup.py`` 文件。
然后确保 ``maintainer``, ``maintainer_email``, ``description`` 和 ``license`` 的内容与 ``package.xml`` 中的一致:

.. code-block:: python

  maintainer='YourName',
  maintainer_email='you@email.com',
  description='Examples of minimal publisher/subscriber using rclpy',
  license='Apache License 2.0',

接下来，添加以下行到 ``entry_points`` 字段的 ``console_scripts`` 括号中:

.. code-block:: python

  entry_points={
          'console_scripts': [
                  'talker = py_pubsub.publisher_member_function:main',
          ],
  },

别忘了保存文件。

2.4 检查 setup.cfg
~~~~~~~~~~~~~~~~~~~

``setup.cfg`` 文件的内容应该已经自动正确填充了，像这样:

.. code-block:: console

  [develop]
  script_dir=$base/lib/py_pubsub
  [install]
  install_scripts=$base/lib/py_pubsub

这是告诉 setuptools 将你的可执行文件放在 ``lib`` 中，因为 ``ros2 run`` 会在那里找它们。

现在你可以构建你的包了，然后运行它，但是让我们先创建订阅者节点，这样你就能看到整个系统是如何工作的。

3 编写订阅者节点
^^^^^^^^^^^^^^^^^^^^^^^^^^^

回到 ``ros2_ws/src/py_pubsub/py_pubsub`` 目录，创建下一个节点。
在终端中输入以下命令:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        wget https://raw.githubusercontent.com/ros2/examples/{REPOS_FILE_BRANCH}/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py

   .. group-tab:: macOS

      .. code-block:: console

        wget https://raw.githubusercontent.com/ros2/examples/{REPOS_FILE_BRANCH}/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py

   .. group-tab:: Windows

      In a Windows command line prompt:

      .. code-block:: console

            curl -sk https://raw.githubusercontent.com/ros2/examples/{REPOS_FILE_BRANCH}/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py -o subscriber_member_function.py

      Or in powershell:

      .. code-block:: console

            curl https://raw.githubusercontent.com/ros2/examples/{REPOS_FILE_BRANCH}/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py -o subscriber_member_function.py

现在目录中应该有这些文件:

.. code-block:: console

  __init__.py  publisher_member_function.py  subscriber_member_function.py

3.1 检查代码
~~~~~~~~~~~~~~~~~~~~

打开 ``subscriber_member_function.py``。

.. code-block:: python

  import rclpy
  from rclpy.node import Node

  from std_msgs.msg import String


  class MinimalSubscriber(Node):

      def __init__(self):
          super().__init__('minimal_subscriber')
          self.subscription = self.create_subscription(
              String,
              'topic',
              self.listener_callback,
              10)
          self.subscription  # prevent unused variable warning

      def listener_callback(self, msg):
          self.get_logger().info('I heard: "%s"' % msg.data)


  def main(args=None):
      rclpy.init(args=args)

      minimal_subscriber = MinimalSubscriber()

      rclpy.spin(minimal_subscriber)

      # Destroy the node explicitly
      # (optional - otherwise it will be done automatically
      # when the garbage collector destroys the node object)
      minimal_subscriber.destroy_node()
      rclpy.shutdown()


  if __name__ == '__main__':
      main()

订阅者节点的代码几乎和发布者的一样。
构造函数创建一个与发布者相同参数的订阅者。
从 :doc:`topics 教程 <../Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics>` 中回忆一下，发布者和订阅者的主题名称和消息类型必须匹配，才能让它们通信。

.. code-block:: python

  self.subscription = self.create_subscription(
      String,
      'topic',
      self.listener_callback,
      10)

订阅者的构造函数和回调函数都没有定时器定义，因为它不需要。
它的回调函数在接收到消息时立即调用。

回调函数的定义只是简单地将它接收到的数据打印到控制台。
回想一下，发布者定义了 ``msg.data = 'Hello World: %d' % self.i``

.. code-block:: python

  def listener_callback(self, msg):
      self.get_logger().info('I heard: "%s"' % msg.data)

``main`` 函数几乎和发布者的一样，只是用订阅者替换了发布者的创建和运行。

.. code-block:: python

  minimal_subscriber = MinimalSubscriber()

  rclpy.spin(minimal_subscriber)

这个节点的依赖和发布者的一样，所以不需要修改 ``package.xml``。
``setup.cfg`` 文件也不需要修改。


3.2 添加 entry point
~~~~~~~~~~~~~~~~~~~~~~

重新打开 ``setup.py``，在发布者的 entry point 下面添加订阅者节点的 entry point。
``entry_points`` 部分应该像这样:

.. code-block:: python

  entry_points={
          'console_scripts': [
                  'talker = py_pubsub.publisher_member_function:main',
                  'listener = py_pubsub.subscriber_member_function:main',
          ],
  },

别忘了保存文件。

4 构建和运行
^^^^^^^^^^^^^^^
你应该已经安装了 ``rclpy`` 和 ``std_msgs`` 包，他们是为 ROS 2 系统的一部分。
在构建之前，最好在工作空间的根目录（ ``ros2_ws`` ）中运行 ``rosdep`` 检查是否有缺少的依赖:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        rosdep install -i --from-path src --rosdistro {DISTRO} -y

   .. group-tab:: macOS

      rosdep only runs on Linux, so you can skip ahead to next step.

   .. group-tab:: Windows

      rosdep only runs on Linux, so you can skip ahead to next step.


然后在工作空间的根目录（ ``ros2_ws`` ）中构建你的新包:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      colcon build --packages-select py_pubsub

  .. group-tab:: macOS

    .. code-block:: console

      colcon build --packages-select py_pubsub

  .. group-tab:: Windows

    .. code-block:: console

      colcon build --merge-install --packages-select py_pubsub

打开一个新的终端，进入 ``ros2_ws``，然后 source setup 文件:

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

运行 talker 节点:

.. code-block:: console

  ros2 run py_pubsub talker

终端应该开始每 0.5 秒发布一条信息，像这样:

.. code-block:: console

  [INFO] [minimal_publisher]: Publishing: "Hello World: 0"
  [INFO] [minimal_publisher]: Publishing: "Hello World: 1"
  [INFO] [minimal_publisher]: Publishing: "Hello World: 2"
  [INFO] [minimal_publisher]: Publishing: "Hello World: 3"
  [INFO] [minimal_publisher]: Publishing: "Hello World: 4"
  ...

打开另一个终端，再次 source ``ros2_ws`` 中的 setup 文件，然后运行 listener 节点:

.. code-block:: console

  ros2 run py_pubsub listener

listener 会开始在控制台打印发布者发布的消息，从发布者当前的消息计数开始，像这样:

.. code-block:: console

  [INFO] [minimal_subscriber]: I heard: "Hello World: 10"
  [INFO] [minimal_subscriber]: I heard: "Hello World: 11"
  [INFO] [minimal_subscriber]: I heard: "Hello World: 12"
  [INFO] [minimal_subscriber]: I heard: "Hello World: 13"
  [INFO] [minimal_subscriber]: I heard: "Hello World: 14"

在每个终端中按 ``Ctrl+C`` 停止节点。

总结
-------

你创建了两个节点，通过 topic 发布和订阅数据。
运行之前还添加了它们的依赖和 entry points 到包配置文件中。

下一步
----------

接下来你可以选择用 :doc:`C++ <./Writing-A-Simple-Cpp-Service-And-Client>` 或 :doc:`Python <./Writing-A-Simple-Py-Service-And-Client>` 写一个使用服务/客户端模型的简单 ROS 2 包。

相关内容
---------------

有很多种方法可以在 Python 中实现发布者和订阅者; 在 `ros2/examples <https://github.com/ros2/examples/tree/{REPOS_FILE_BRANCH}/rclpy/topics>`_ 中可以找到.
