.. redirect-from::

    Tutorials/Actions/Writing-a-Py-Action-Server-Client

.. _ActionsPy:

实现 action 服务器和客户端(Python)
============================================

**目标:** 在 Python 中实现 action 服务器和客户端。

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

1 编写 action 服务器
^^^^^^^^^^^^^^^^^^^^^^^^^^

让我们专注于编写一个 action 服务器，使用我们在 :doc:`../Creating-an-Action` 教程中创建的 action 来计算斐波那契数列。

到目前为止，你已经创建了包并使用 ``ros2 run`` 运行节点。
为了让本教程保持简洁，我们将把 action 服务器范围限制在一个文件中。
如果你想看完整的 action 教程包是什么样子的，可以查看 `action_tutorials <https://github.com/ros2/demos/tree/{REPOS_FILE_BRANCH}/action_tutorials>`__.

在目录下创建一个新文件，我们称之为 ``fibonacci_action_server.py``，并添加以下代码：

.. literalinclude:: scripts/server_0.py
    :language: python

第 8 行定义了一个 ``FibonacciActionServer`` 类，它是 ``Node`` 的子类。
通过调用 ``Node`` 构造函数初始化类，将我们的节点命名为 ``fibonacci_action_server``:

.. literalinclude:: scripts/server_0.py
    :language: python
    :lines: 11

在构造函数中，我们还实例化了一个新的 action 服务器：

.. literalinclude:: scripts/server_0.py
    :language: python
    :lines: 12-16

一个 action 服务器需要四个参数：

1. 一个 ROS 2 节点，将 action 客户端添加到： ``self`` 。
2. action 的类型： ``Fibonacci`` （在第 5 行引入）。
3. action 名称： ``'fibonacci'``。
4. 一个用于执行接受的目标的回调函数： ``self.execute_callback``。
   这个回调 **必须** 返回一个 action 类型的结果消息。

我们还在类中定义了一个 ``execute_callback`` 方法：

.. literalinclude:: scripts/server_0.py
    :language: python
    :lines: 18-21

这个函数将在接受目标后被调用。

让我们尝试运行 action 服务器：

.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

      python3 fibonacci_action_server.py

  .. group-tab:: macOS

    .. code-block:: bash

      python3 fibonacci_action_server.py

  .. group-tab:: Windows

    .. code-block:: bash

      python fibonacci_action_server.py

在另一个终端中，我们可以使用命令行接口发送一个目标：

.. code-block:: bash

    ros2 action send_goal fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"

在运行 action 服务器的终端中，你应该看到一个记录的消息 "Executing goal..."，后面是一个警告，表示目标状态未设置。
默认情况下，如果在执行回调中未设置目标的执行状态，它会假定为 *aborted* 状态。

我们可以使用 `succeed() <http://docs.ros2.org/latest/api/rclpy/api/actions.html#rclpy.action.server.ServerGoalHandle.succeed>`_ 方法在目标处理完成后指示目标成功：

.. literalinclude:: scripts/server_1.py
    :language: python
    :lines: 18-22
    :emphasize-lines: 3

现在，如果重新启动 action 服务器并发送另一个目标，你应该看到目标以 ``SUCCEEDED`` 状态完成。

现在让我们实际计算并返回请求的斐波那契数列：

.. literalinclude:: scripts/server_2.py
    :language: python
    :lines: 18-30
    :emphasize-lines: 4-7,12

在计算序列后，我们在返回之前将其赋值给结果。

再次重新启动 action 服务器并发送另一个目标。
你应该看到目标以正确的结果序列完成。

1.2 发布反馈
~~~~~~~~~~~~~~~~~~~~~~~

action 的一个好处是在目标执行期间能向 action 客户端提供反馈。
我们可以通过调用目标的 `publish_feedback() <http://docs.ros2.org/latest/api/rclpy/api/actions.html#rclpy.action.server.ServerGoalHandle.publish_feedback>`_ 方法使 action 服务器发布反馈给 action 客户端。

我们将替换回复的 ``sequence`` 变量，用来存储目前计算到的序列。
在 for 循环中更新反馈消息后，我们会发布反馈消息，然后 sleep 一段时间来模拟计算耗时：

.. literalinclude:: scripts/server_3.py
    :language: python
    :emphasize-lines: 1,23,24,27-31,36

重新启动 action 服务器后，我们可以使用 ``--feedback`` 选项的命令行工具来确认反馈是否发布：

.. code-block:: bash

    ros2 action send_goal --feedback fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"

2 编写 action 客户端
^^^^^^^^^^^^^^^^^^^^^^^^^^

我们现在在单个文件中编写一个 action 客户端。
打开一个新文件，我们称之为 ``fibonacci_action_client.py``，并添加以下代码：

.. literalinclude:: scripts/client_0.py
    :language: python

我们定义了一个 ``FibonacciActionClient`` 类，它是 ``Node`` 的子类。
通过调用 ``Node`` 构造函数初始化类，将我们的节点命名为 ``fibonacci_action_client``:

.. literalinclude:: scripts/client_0.py
    :language: python
    :lines: 11

在类构造函数中，我们使用在 :doc:`../Creating-an-Action` 教程中定义的自定义 action 来创建一个 action 客户端：

.. literalinclude:: scripts/client_0.py
    :language: python
    :lines: 12

我们传递三个参数来创建一个 ``ActionClient``：

1. 一个 ROS 2 节点，将 action 客户端添加到： ``self``。
2. action 的类型： ``Fibonacci``。
3. action 名称： ``'fibonacci'``。

我们的 action 客户端将能够与相同 action 名称和类型的 action 服务器通信。

我们还在 ``FibonacciActionClient`` 类中定义了一个 ``send_goal`` 方法：

.. literalinclude:: scripts/client_0.py
    :language: python
    :lines: 14-20

这个方法等待 action 服务器可用，然后发送一个目标到服务器。
它返回一个我们稍后可以等待的 future。

在类定义之后，我们定义了一个 ``main()`` 函数，它初始化 ROS 2 并创建 ``FibonacciActionClient`` 节点的实例。
然后发送一个目标并等待目标完成。

最后，我们在 Python 程序的入口点中调用 ``main()``。

让我们通过运行之前构建的 action 服务器来测试我们的 action 客户端：

.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

      python3 fibonacci_action_server.py

  .. group-tab:: macOS

    .. code-block:: bash

      python3 fibonacci_action_server.py

  .. group-tab:: Windows

    .. code-block:: bash

      python fibonacci_action_server.py

在另一个终端中，运行 action 客户端：

.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

      python3 fibonacci_action_client.py

  .. group-tab:: macOS

    .. code-block:: bash

      python3 fibonacci_action_client.py

  .. group-tab:: Windows

    .. code-block:: bash

      python fibonacci_action_client.py

你应该能看到 action 服务端打印出成功执行目标的消息：

.. code-block:: bash

  [INFO] [fibonacci_action_server]: Executing goal...
  [INFO] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1])
  [INFO] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2])
  [INFO] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2, 3])
  [INFO] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2, 3, 5])
  # etc.

而 action 客户端应该会在启动后的很短时间内运行结束。
现在，我们有一个运行的 action 客户端，但我们没有看到任何结果或得到任何反馈。

2.1 获取结果
~~~~~~~~~~~~~~~~~~~~

我们可以发送目标，但是怎么知道它什么时候完成呢？
可以通过几个步骤获取结果。
首先，我们拿到我们发送的目标的 handle .
然后，我们可以使用 handle 来接收对应的结果。

以下是这个示例的完整代码：

.. literalinclude:: scripts/client_1.py
    :language: python

`ActionClient.send_goal_async() <http://docs.ros2.org/latest/api/rclpy/api/actions.html#rclpy.action.client.ActionClient.send_goal_async>`_ 会对 goal handle 返回一个 future 。
首先，我们会为 future 的完成注册一个回调函数：

.. literalinclude:: scripts/client_1.py
    :language: python
    :lines: 22

请注意，只有当 action 服务器接受或拒绝目标请求时，future 才会被标记为已完成。
现在我们来看看 ``goal_response_callback`` 的细节。
我们可以检查目标是否已经被拒绝了，如果是的话，提前返回就好了，毕竟拒绝就意味着不会有结果传回来了：

.. literalinclude:: scripts/client_1.py
    :language: python
    :lines: 24-30

现在我们有了一个 goal handle，我们可以使用它的 `get_result_async() <http://docs.ros2.org/latest/api/rclpy/api/actions.html#rclpy.action.client.ClientGoalHandle.get_result_async>`_ 方法来请求结果。
类似于发送目标，我们会得到一个 future ，它结果准备好时会被标记为已完成。
然后我我们给 future 的完成注册一个回调函数：

.. literalinclude:: scripts/client_1.py
    :language: python
    :lines: 32-33

在回调中，我们会记录结果序列，并关闭 ROS 2，以便干净利落地退出：

.. literalinclude:: scripts/client_1.py
    :language: python
    :lines: 35-38

在一个单独的终端中运行 action 服务器，然后尝试运行 Fibonacci Action 客户端！

.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

      python3 fibonacci_action_client.py

  .. group-tab:: macOS

    .. code-block:: bash

      python3 fibonacci_action_client.py

  .. group-tab:: Windows

    .. code-block:: bash

      python fibonacci_action_client.py

应该能看到表示目标被接受和最终结果的日志信息。

2.2 获取反馈
~~~~~~~~~~~~~~~~~~~~

Action 客户端可以发送目标了。
Nice!
但是如果我们能从 action 服务器得到一些关于目标的反馈就更好了了。

以下是这个示例的完整代码：

.. literalinclude:: scripts/client_2.py
    :language: python

这里是用于反馈消息的回调函数：

.. literalinclude:: scripts/client_2.py
    :language: python
    :lines: 40-42

在回调中，我们获取消息的反馈部分并将 ``partial_sequence`` 字段打印到屏幕上。

我们需要将回调注册给 action 客户端。
可以通过在发送目标时将回调传递给 action 客户端来实现：

.. literalinclude:: scripts/client_2.py
    :language: python
    :lines: 20

大功告成。如果运行 action 客户端，你应该能看到屏幕上打印出的反馈消息。

总结
-------

在本教程中，你逐行编写了一个 Python action 服务器和 action 客户端，并配置它们来传递目标、反馈和结果。

相关内容
---------------

* 有几种方法可以在 Python 中编写 action 服务器和客户端；请查看 `ros2/examples <https://github.com/ros2/examples/tree/{REPOS_FILE_BRANCH}/rclpy/actions>`_ 中的 ``minimal_action_server`` and ``minimal_action_client``.

* 更多关于 ROS actions 的详细信息，请参考 `design article <http://design.ros2.org/articles/actions.html>`__.
