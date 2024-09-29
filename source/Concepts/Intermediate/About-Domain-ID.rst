.. redirect-from::

   Concepts/About-Domain-ID

The ROS_DOMAIN_ID
=================

.. contents:: Table of Contents
   :local:

概览
--------

ROS 2 用于通信的默认中间件(middleware)是 DDS。
在 DDS 中，让不同的逻辑网络共享一个物理网络的主要机制被称为 Domain ID。
在同一个域中(译者注：也就是 Domain ID 一样)的 ROS 2 节点可以自由地发现彼此并向彼此发送消息，而在不同域中的 ROS 2 节点则不能。
默认情况下，所有 ROS 2 节点都使用域 domain 0。
为了避免在同一网络上运行 ROS 2 的不同设备之间的干扰，应为每组需要独立的设备设置不同的 domain ID。

选择 domain ID (简短版)
------------------------------------

下文解释了如何推导出在 ROS 2 中应该使用的 domain ID 范围。
如果你对这些细节不感兴趣，只想选择一个安全可用的数字，那么只需要选择 0 到 101 之间的一个 domain ID 即可。

选择 domain ID (详细版)
-----------------------------------

Domain ID 是 DDS 用来计算用于发现和通信的 UDP 端口号的。
具体如何计算请参见 `这篇文章 <https://community.rti.com/content/forum-topic/statically-configure-firewall-let-omg-dds-traffic-through>`__。
回顾一下基本网络知识，UDP 端口是一个 `16 位无符号整数 <https://en.wikipedia.org/wiki/User_Datagram_Protocol#Ports>`__。
那么，可以分配的最高端口号是 65535。
根据上面的公式，这意味着可以分配的最高 domain ID 是 232，而可以分配的最低 domain ID 是 0。

平台限制
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

为了最大兼容性，选择 domain ID 时应满足一些额外的平台限制。
首先是最好避免在操作系统的 `临时端口范围 <https://en.wikipedia.org/wiki/Ephemeral_port>`__ 中分配 domain ID。
这样可以避免 ROS 2 节点使用的端口与计算机上的其他网络服务之间可能发生的冲突。

以下是关于临时端口的一些特定平台的注意事项。

.. tabs::

   .. group-tab:: Linux

     默认情况下，Linux 内核使用端口 32768-60999 作为临时端口。
     这意味着 domain ID 0-101 和 215-232 可以安全地使用，而不会与临时端口发生冲突。
     临时端口范围可以在 ``/proc/sys/net/ipv4/ip_local_port_range`` 中配置。
     如果使用自定义的临时端口范围，上面说的 domain ID 可用的范围可能需要相应调整。

   .. group-tab:: macOS

     默认情况下，macOS 上的临时端口范围是 49152-65535。
     这意味着 domain ID 0-166 可以安全地使用，而不会与临时端口发生冲突。
     临时端口范围可以通过 ``net.inet.ip.portrange.first`` 和 ``net.inet.ip.portrange.last`` 的自定义 sysctl 值来配置。
     如果使用自定义的临时端口范围，上面说的 domain ID 可用的范围可能需要相应调整。

   .. group-tab:: Windows

     默认情况下，Windows 上的临时端口范围是 49152-65535。
     这意味着 domain ID 0-166 可以安全地使用，而不会与临时端口发生冲突。
     临时端口范围可以 `使用 netsh <https://docs.microsoft.com/en-us/troubleshoot/windows-server/networking/default-dynamic-port-range-tcpip-chang>`__ 来配置。
     如果使用自定义的临时端口范围，上面说的 domain ID 可用的范围可能需要相应调整。

参与者限制
^^^^^^^^^^^^^^^^^^^^^^^

在计算机上运行的每个 ROS 2 进程都会创建一个 DDS “参与者(participant)”。
由于每个 DDS 参与者占用计算机上的两个端口，因此在一台计算机上运行超过 120 个 ROS 2 进程可能会溢出到其他 domain ID 或临时端口。

为了理解为什么会发生这种情况，让我们考虑 domain ID 1 和 2的情况。

- domain ID 1 使用端口 7650 和 7651 用于 multicast 。
- domain ID 2 使用端口 7900 和 7901 用于 multicast 。
- 创建 domain ID 1 中的第一个进程(第零个参与者)时，使用端口 7660 和 7661 用于 unicast 。
- 创建 domain ID 1 中的第120个进程(第119个参与者)时，使用端口 7898 和 7899 用于 unicast 。
- 创建 domain ID 1 中的第121个进程(第120个参与者)时，使用端口 7900 和 7901 用于 unicast ，这就和与 domain ID 2 的端口重叠了。

如果可以确保设备只会在一个 domain ID 上运行，而且 domain ID 很小，那么可以创建比这个(译者注：指120个)更多的 ROS 2 进程。

当选择的 domain ID 接近特定平台的 domain ID 范围的上限时，还应考虑一个额外的约束。

假设一个 domain ID 为 101 的 Linux 设备：

- 该设备上的第零个 ROS 2 进程将连接到端口 32650、32651、32660 和 32661。
- 该设备上的第一个 ROS 2 进程将连接到端口 32650、32651、32662 和 32663。
- 该设备上的第二个 ROS 2 进程将连接到端口 32650、32651、32664 和 32665。
- 该设备上的第五十三个 ROS 2 进程将连接到端口 32650、32651、32766 和 32767。
- 该设备上的第五十四个 ROS 2 进程将连接到端口 32650、32651、32768 和 32769，这就溢出到了临时端口范围。

因此，在 Linux 上使用 domain ID 101 时应创建的最大进程数是 54。
同样可以推导出在 Linux 上使用 domain ID 232 时应创建的最大进程数是 63，因为最大端口号是 65535。

在 macOS 和 Windows 上情况类似，只是数字不同。
在 macOS 和 Windows 上，选择 domain ID 为 166(范围的最大值) 时，在设备上创建的 ROS 2 进程的最大数量是 120。

Domain ID - UDP 端口映射计算器
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. raw:: html

    <table>
      <tr>
        <td style="text-align: right; vertical-align: middle;"><label>Domain ID:</label></td>
        <td><input type="number" min="0" max="232" size="3" class="display" value="0" id="domainID" onChange="calculate(this.value)"/></td>
      </tr>
      <tr>
        <td style="text-align: right; vertical-align: middle;"><label>Participant ID:</label></td>
        <td><input type="number" min="0" size="3" class="display" value="0" id="participantID" onChange="calculate(this.value)"/></td>
      </tr>
    </table>
    <hr/>
    <table>
      <tr>
        <td style="text-align: right; vertical-align: middle;"><label>Discovery Multicast Port:</label></td>
        <td><input type="text" size="5" class="discoveryMulticastPort" disabled/></td>
      </tr>
      <tr>
        <td style="text-align: right; vertical-align: middle;"><label>User Multicast Port:</label></td>
        <td><input type="text" size="5" class="userMulticastPort" disabled/></td>
      </tr>
      <tr>
        <td style="text-align: right; vertical-align: middle;"><label>Discovery Unicast Port:</label></td>
        <td><input type="text" size="5" class="discoveryUnicastPort" disabled/></td>
      </tr>
      <tr>
        <td style="text-align: right; vertical-align: middle;"><label>User Unicast Port:</label></td>
        <td><input type="text" size="5" class="userUnicastPort" disabled/></td>
      </tr>
    </table>
    <br/>
    <br/>

    <script type="text/javascript">
      window.addEventListener('load', (event) => {
         calculate(event);
      });
      const discoveryMcastPort = document.querySelector('.discoveryMulticastPort');
      const userMcastPort = document.querySelector('.userMulticastPort');
      const discoveryUnicastPort = document.querySelector('.discoveryUnicastPort');
      const userUnicastPort = document.querySelector('.userUnicastPort');

      const domainID = document.getElementById('domainID');
      const participantID = document.getElementById('participantID');

      // calculate function
      function calculate(event) {
        const d0 = 0;
        const d2 = 1;
        const d1 = 10;
        const d3 = 11;
        const PB = 7400;
        const DG = 250;
        const PG = 2;

        discoveryMcastPort.value = PB + (DG * domainID.value) + d0;
        userMcastPort.value = PB + (DG * domainID.value) + d2;
        discoveryUnicastPort.value = PB + (DG * domainID.value) + d1 + (PG * participantID.value);
        userUnicastPort.value = PB + (DG * domainID.value) + d3 + (PG * participantID.value);
      }
    </script>
