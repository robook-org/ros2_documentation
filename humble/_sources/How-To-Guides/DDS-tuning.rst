.. redirect-from::

  Guides/DDS-tuning
  Troubleshooting/DDS-tuning

DDS 调优指南
======================

本页面提供了一些关于在 Linux 上使用各种 DDS 提供商的实现时遇到的问题的参数调优指导。
我们在 Linux 上或者某些 DDS 实现上发现了一些问题，这些问题也许会在其他平台或 DDS 上发生，但可能不会记录在这里。

调优将从如下一些建议开始：参数调整只适用于特定的系统和环境，会因为很多因素而有所不同。
在调试时，你可能需要增加或减少消息大小、网络拓扑之类的值。

同时你应该了解一个事实：调优参数可能会带来资源消耗，可能会影响到系统中的其他部分，超出了你想要的改进范围。
应该根据具体情况权衡提高可靠性带来的好处和其他方面可能产生的损失。

.. _cross-vendor-tuning:

通用调优
-------------------

**Issue:** 通过有损(lossy)连接(通常是 WiFi)发送数据会出现问题，因为一些 IP 片段(fragments)在传输中可能会丢包，有可能导致接收端的内核缓冲区被塞满。

只要 UDP 包丢失至少一个 IP 片段，接收到的其余片段会填满内核缓冲区。
默认情况下，Linux 内核在尝试重新组合数据包片段 30s 后会超时。
由于内核缓冲区在这个时候已经满了(默认大小为 256KB)，没有新的片段可以进来，所以连接会看起来“卡住”很长时间。

这个问题是所有 DDS 提供商都会遇到的，所以解决方案涉及调整内核参数。

**解决方案:** 使用尽力(best-effort)传输的 QoS 配置而不是可靠(reliable)传输。

尽力传输可以减少网络流量，因为 DDS 实现无需承担可靠通信的开销。在可靠通信中，发布者需要对发送给订阅者的信息进行确认，并且必须重新发送未正确接收的信息。

不过，如果内核的 IP 片段缓冲区满了，问题表现还是一样的（阻塞 30 秒）。
这个解决方案应该能在一定程度上改善问题，而无需调整参数。

**解决方案:** 减小 ``ipfrag_time`` 参数的值。

``net.ipv4.ipfrag_time / /proc/sys/net/ipv4/ipfrag_time`` (默认 30s) :
在内存中保留一个 IP 片段的时间，单位为秒。

通过运行以下命令改小这个值，比如设置为 3s:

.. code-block:: console

    sudo sysctl net.ipv4.ipfrag_time=3

减小这个参数的值也会减少没有接收到片段的时间窗口。
这个参数是全局的，所以需要根据不同环境的实际情况考虑是否减小这个值。

**解决方案:** 增大 ``ipfrag_high_thresh`` 参数的值。

``net.ipv4.ipfrag_high_thresh / /proc/sys/net/ipv4/ipfrag_high_thresh`` (默认 262144 字节):
用于重新组装 IP 片段的最大内存。

通过运行以下命令增大这个值，比如设置为 128MB:

.. code-block:: console

    sudo sysctl net.ipv4.ipfrag_high_thresh=134217728     # (128 MB)

增大这个参数的值是为了确保缓冲区永远不会完全填满。
不过这个值可能需要设置得很大才能在 ``ipfrag_time`` 时间窗口内保留所有接收到的数据，如果假设每个 UDP 包都缺少一个片段的话。

**Issue:** 使用非原始类型的大型可变大小数组发送自定义消息会造成较高的序列化/解序列化开销和 CPU 负载。
这会导致 publisher 在 ``publish()`` 中花费过多时间， ``ros2 topic hz`` 这样的工具报告的实际接收的消息频率不足。
请注意， ``builtin_interfaces/Time`` 这样的消息也被认为是非原始类型，会导致更高的序列化开销。
由于序列化开销增加，当将自定义消息类型从 ROS 1 迁移到 ROS 2 时，可能会观察到严重的性能下降。

**解决方案:** 使用多个原始类型的数组而不是单个自定义类型的数组，或者像 ``PointCloud2`` 中那样把信息打包到字节数组中。
例如，不要定义一个 ``FooArray`` 消息为:

.. code-block:: console

    Foo[] my_large_array

其中 ``Foo`` 是这样定义的:

.. code-block:: console

    uint64 foo_1
    uint32 foo_2

而是将 ``FooArray`` 定义为:

.. code-block:: console

    uint64[] foo_1_array
    uint32[] foo_2_array

Fast RTPS 调优
----------------

**Issue:** Fast RTPS 在 WiFi 上运行时会让网络充斥着大块数据或快速发布的数据。

See the solutions under :ref:`Cross-vendor tuning <cross-vendor-tuning>`.

Cyclone DDS 调优
------------------

**Issue:** 即使在使用了可靠的设置并在有线网络上传输大消息时，Cyclone DDS 无法可靠传输。

这个问题应该会很快得到解决，详情请参考 `这个问题 <https://github.com/eclipse-cyclonedds/cyclonedds/issues/484>`_ .
目前我们提供了以下解决方案(通过 `这个测试程序 <https://github.com/jacobperron/pc_pipe>`_ debug):

**解决方案:** 增加 Cyclone 使用的最大 Linux 内核接收缓冲区大小和最小 socket 接收缓冲区大小。

*调整以解决 9MB 消息的问题:*

设置最大接收缓冲区大小 ``rmem_max``，运行:

 .. code-block:: console

    sudo sysctl -w net.core.rmem_max=2147483647

或者通过编辑 ``/etc/sysctl.d/10-cyclone-max.conf`` 文件永久设置:

 .. code-block:: console

    net.core.rmem_max=2147483647

接下来，设置 Cyclone 请求的最小 socket 接收缓冲区大小，写好一个配置文件给Cyclone 在启动时使用，如下:

.. code-block:: xml

  <?xml version="1.0" encoding="UTF-8" ?>
  <CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config
  https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
      <Domain id="any">
          <Internal>
              <SocketReceiveBufferSize min="10MB"/>
          </Internal>
      </Domain>
  </CycloneDDS>

每次运行节点时，设置以下环境变量:

.. code-block:: console

    CYCLONEDDS_URI=file:///absolute/path/to/config_file.xml

RTI Connext 调优
------------------

**Issue:** 即使使用了可靠设置并在有线网络上传输大消息时，Connext 还是无法可靠传输。

**解决方案:** 使用这个 `Connext QoS profile <https://github.com/jacobperron/pc_pipe/blob/master/etc/ROS2TEST_QOS_PROFILES.xml>`_ , 并且调大 ``rmem_max`` 的值.

运行以下指令设置最大接收缓冲区大小 ``rmem_max``:

 .. code-block:: console

    sudo sysctl -w net.core.rmem_max=4194304

将 ``net.core.rmem_max`` 调整到 4MB 后，QoS 配置就能产生真正可靠的行为。

事实证明这种配置可以通过 SHMEM|UDPv4 和 UDPv4 在单机上可靠地传送信息。
我们还测试了 rmem_max 为 4MB 和 20MB 的多机配置（两台机器使用 1Gbps 以太网连接），结果没有出现报文丢失，平均报文传送时间分别为 700ms 和 371ms。

在未配置内核 rmem_max 的情况下，相同的 Connext QoS 配置文件需要 12 秒才能完成数据传输。 不过，它至少能完成传输。

**解决方案:** 使用这个 `Connext QoS profile <https://github.com/jacobperron/pc_pipe/blob/master/etc/ROS2TEST_QOS_PROFILES.xml>`_ 而 *不调整* ``rmem_max``.

这个 QoS 配置文件是参照 RTI 的文档 `配置流控制器 <https://community.rti.com/forum-topic/transfering-large-data-over-dds>`_ 配置的。其中包含了慢、中、快三种流控制器。（在 Connext QoS 配置文件链接中可以看到）

我们的测试结果表明，中等流控制器对我们的情况效果最好。
不过，控制器仍然需要针对它们所在的特定机器/网络/环境进行调整。
Connext 流控制器可以用来调整带宽及其发送数据的激进程度(aggressiveness)，但是一旦超过特定设置的带宽，性能还是会开始下降。
