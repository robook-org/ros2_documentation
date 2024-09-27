Actions
=======

.. contents:: Table of Contents
   :local:

在 ROS 2 中，action 指的是一个可远程调用的、可长时间运行的程序，它可以提供反馈并且可以取消运行。
例如，一个机器人的上层状态机可能会调用一个 action 来告诉导航系统去某个航点(waypoint)，这个过程可能需要几秒钟（或几分钟）。
在这个过程中，导航系统可以提供有关目标还有多远的反馈，而上层状态机可以在这个过程中选择取消或者中断到达那个航点。

action 的结构也体现在 action message 的定义中：

.. code::

   int32 request
   ---
   int32 response
   ---
   int32 feedback

在 ROS 2 中，action 通常被期望是一个长时间运行的程序，因为建立和监控连接会有一些开销。
如果你需要一个短时间运行的远程调用程序，考虑使用 :doc:`service <About-Services>`。

Actions 由 action 名称标识，它看起来很像一个 topic 名称（但在不同的命名空间中）。

Action 由两部分组成：action 服务器和 action 客户端。

Action 服务器(server)
----------------------

Action 服务器是接受请求并执行程序的实体。
它还负责在运行过程中发送反馈，并且回应可能的对取消/中断请求。
例如，考虑一个计算 Fibonacci 数列的 action，它的接口如下：

.. code::

   int32 order
   ---
   int32[] sequence
   ---
   int32[] sequence

Action 服务器会接收一条包含 ``order`` 的消息，开始计算 Fibonacci 数列直到 ``order`` （并且在这计算过程中提供反馈），最后在 ``sequence`` 中返回一个完整的结果数列。

.. note::

   每个 action 名称应该只有一个 action 服务器。
   如果有多个服务服务器使用相同的服务名称，那么在客户端请求时，无法确定哪个服务服务器会接收到请求。

Action 客户端(client)
-----------------------

Action 客户端是向服服务器发起请求的实体。
在上面的例子中，action 客户端是 创建包含 ``order`` 的初始消息，并等待 action 服务器计算序列并返回它（在这个过程中提供反馈） 的实体。

与 action 服务器不同，可以有任意数量的 action 客户端使用相同的 action 名称。
