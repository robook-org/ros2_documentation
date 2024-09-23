服务(Services)
===============

.. contents:: Table of Contents
   :local:

在 ROS 2 中，服务(service)指的是远程处理程序的调用。
换句话说，一个节点可以远程调用另一个节点的处理程序，随后被调用的节点会运行计算然后把结果传递回来。

这种结构也体现在 service message 的定义中：

.. code::

   uint32 request
   ---
   uint32 response

In ROS 2, services are expected to return quickly, as the client is generally waiting on the result.
Services should never be used for longer running processes, in particular processes that might need to be preempted for exceptional situations.
If you have a service that will be doing a long-running computation, consider using an :doc:`action <About-Actions>` instead.

Services are identified by a service name, which looks much like a topic name (but is in a different namespace).

A service consists of two parts: the service server and the service client.

Service server
--------------

A service server is the entity that will accept a remote procedure request, and perform some computation on it.
For instance, suppose the ROS 2 message contains the following:

.. code::

   uint32 a
   uint32 b
   ---
   uint32 sum

The service server would be the entity that receives this message, adds ``a`` and ``b`` together, and returns the ``sum``.

.. note::

   There should only ever be one service server per service name.
   It is undefined which service server will receive client requests in the case of multiple service servers on the same service name.

Service client
--------------

A service client is an entity that will request a remote service server to perform a computation on its behalf.
Following from the example above, the service client is the entity that creates the initial message containing ``a`` and ``b``, and waits for the service server to compute the sum and return the result.

Unlike the service server, there can be arbitrary numbers of service clients using the same service name.
