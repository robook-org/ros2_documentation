发现(Discovery)
================

.. contents:: Table of Contents
   :local:

节点之间的发现(discovery)是通过 ROS 2 的底层中间件自动完成的。
这个过程可以总结为如下：

#. 当一个节点启动时，它会向具有相同 ROS domain （通过 ROS_DOMAIN_ID 环境变量设置）的网络上的其他节点广播自己的存在。
   其它节点会回复这个广播，提供关于自己的信息，以便建立合适的连接并进行通信。
#. 节点会定期广播自己的存在，以便在最开始初始化的搜索过程结束之后还能及时发现新实体，并且和新出现的实体建立连接。（译者注：如果你是初学者，为了方便理解，可以认为这里的实体指的就是节点，在后续的学习中你会意识到可以不只是节点。）
#. 在节点即将离线之前，它会向其他节点广播自己的离线状态。

节点只会与具有兼容 :doc:`Quality of Service <../../Tutorials/Demos/Quality-of-Service>` 配置的节点建立连接。

:ref:`talker-listener demo <talker-listener>` 是一个很好的例子。
在一个终端中运行 C++ talker 节点，它会在一个 topic 上发布消息。
在另一个终端中运行 Python listener 节点，它会订阅相同的 topic 上的消息。

你应该看到这两个节点会自动发现彼此，并开始交换消息。
