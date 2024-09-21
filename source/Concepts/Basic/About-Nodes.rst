节点
=====

.. contents:: Table of Contents
   :local:

一个节点就是 ROS 2 graph 中的一个参与个体，它使用 :doc:`客户端库 <About-Client-Libraries>` 与其他节点进行通信。
节点可以与在同一个进程内、不同进程内或不同机器上的其它节点进行通信。
节点通常是 ROS graph 中的计算单元；每个节点应该只做做一件逻辑上(独立的)事情。

节点可以 :doc:`发布 <About-Topics>` 数据到某个 topic ，以便将数据传递给其他节点，或者 :doc:`订阅 <About-Topics>` 某个 topic 以从其他节点获取数据。
它们也可以作为 :doc:`某个服务的客户端 <About-Services>` 让另一个(作为服务器的)节点执行(它预设好应做的)计算，或者作为 :doc:`某个服务的服务器 <About-Services>` 为其他节点提供一些功能。
对于长时间运行的计算，一个节点可以作为 :doc:`action 客户端 <About-Actions>` 让另一个节点执行计算，或者作为 :doc:`action 服务器 <About-Actions>` 为其他节点提供功能。
当然，节点也能提供一些可配置的 :doc:`参数 <About-Parameters>` 以便在运行时改变行为。

节点通常是一个复杂的发布者、订阅者、service 服务器、service 客户端、action 服务器和 action 客户端的组合。

节点之间的连接是通过分布式的 :doc:`发现(discovery) <About-Discovery>` 过程建立的。
