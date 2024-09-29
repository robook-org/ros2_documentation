.. redirect-from::

   Concepts/About-Client-Interfaces
   Concepts/About-ROS-2-Client-Libraries

.. include:: ../../../global_substitutions.txt

客户端库(Client libraries)
================================

.. contents:: Table of Contents
   :local:

概览
--------

客户端库是让用户实现他们自己的 ROS 2 代码的 API。
用户可以通过客户端库使用 ROS 2 组件，例如节点、topic、service 等。
客户端库有很多种编程语言的实现，这样用户可以用最适合他们应用的语言编写 ROS 2 代码。
例如，你可能更喜欢用 Python 编写可视化工具，因为这样可以更快地进行原型迭代，而对于与效率有关的系统部分，最好用 C++ 实现。

客户端库的一个重要特性是，使用不同客户端库编写的节点可以相互共享消息，因为所有客户端库都实现了代码生成器，为用户提供了与相应语言中的 ROS 2 接口文件交互的能力。

除了特定于语言的通信工具之外，客户端库还向用户开放了许多 ROS 核心功能，使 ROS 真正成为“ROS”。（译者注：众所周知，所谓的 Robot Operating System 并不是真正意义上的操作系统，此处是想表达在这些核心功能的加持下，ROS可以做更多事情、实现更多功能。）
例如，以下是通常可以通过客户端库访问的功能列表：

* Names and namespaces
* Time (real or simulated) - 真实或模拟时间控制
* Parameters - 参数控制
* Console logging - 日志系统
* Threading model - 线程模型
* Intra-process communication - 进程内通信

支持的客户端库
--------------------------

C++ 客户端库（``rclcpp``）和 Python 客户端库（``rclpy``）都是客户端库，它们都使用 ``rcl`` (译者注：后续我们将使用 ``rcl`` 这一称呼，也就是 ros client library 的缩写表达对它自身的指代。)中的通用功能。

``rclcpp`` 包
~~~~~~~~~~~~~~~~~~~~~~

ROS C++ 客户端库（``rclcpp``）是面向用户的、符合 C++ 习惯的接口，提供了所有 ROS 客户端功能，如创建节点、发布者和订阅者。
``rclcpp`` 基于 ``rcl`` 和 ``rosidl`` |API| 构建，旨在与 ``rosidl_generator_cpp`` 生成的 C++ 消息一起使用。

``rclcpp`` 利用 C++ 和 C++17 的特性，使接口尽可能易于使用，但由于它重用了 ``rcl`` 中的实现，因此能够保持与使用 ``rcl`` |API| 的其他客户端库一致的行为。

``rclcpp`` 仓库位于 GitHub 上的 `ros2/rclcpp <https://github.com/ros2/rclcpp>`_ 。

随代码自动生成的 |API| 文档在这里可以找到：

`api/rclcpp/index.html <http://docs.ros.org/en/{DISTRO}/p/rclcpp>`_

``rclpy`` 包
~~~~~~~~~~~~~~~~~~~~~

与 C++ 客户端库相对应的则是 Python 客户端库（``rclpy``）。
与 C++ 客户端库一样，``rclpy`` 也是基于 ``rcl`` C API 实现的。
``rclpy`` 提供了一个 Python 习惯的接口，使用原生 Python 类型和模式，如 list 和 context objects。
通过在实现中使用 ``rcl`` |API|，它在功能和行为上与其他客户端库保持一致。
``rclpy`` 还基于 ``rcl`` |API| 和各种 message 的 python 实现，提供了额外的 Python 习惯的绑定，以此通过 ``threading.Thread`` 或类似的方式运行 ``rcl`` |API| 中的函数，来实现 execution model。

与 C++ 客户端库一样，``rclpy`` 为用户与 ROS 消息交互生成自定义 Python 代码，但与 C++ 不同的是，它最终将原生 Python 消息对象转换为消息的 C 版本。
所有操作都在 Python 版本的消息上进行，直到需要将其传递到 ``rcl`` 层时，才将其转换为消息的 C 版本，以便将其传递到 ``rcl`` C |API| 中。
如果可能的话，当在同一进程中的发布者和订阅者之间通信时，尽量避免这种转换，以减少转换为 Python 对象的次数。

``rclpy`` 仓库位于 GitHub 上的 `ros2/rclpy <https://github.com/ros2/rclpy>`_ 。

随代码自动生成的 |API| 文档在这里：

`api/rclpy/index.html <http://docs.ros2.org/foxy/api/rclpy/index.html>`_


社区维护的客户端库
~~~~~~~~~~~~~~~~~~~~

C++ 和 Python 客户端库由 ROS 2 核心团队维护，而ROS 2 社区的成员维护了其他客户端库：

* `Ada <https://github.com/ada-ros/ada4ros2>`__  This is a set of packages (binding to ``rcl``, message generator, binding to ``tf2``, examples and tutorials) that allows the writing of Ada applications for ROS 2.
* `C <https://github.com/ros2/rclc>`__  ``rclc`` does not put a layer on top of rcl but complements rcl to make rcl+rclc a feature-complete client library in C. See `micro.ros.org <https://micro.ros.org/>`__ for tutorials.
* `JVM and Android <https://github.com/ros2-java>`__ Java and Android bindings for ROS 2.
* `.NET Core, UWP and C# <https://github.com/esteve/ros2_dotnet>`__ This is a collection of projects (bindings, code generator, examples and more) for writing ROS 2 applications for .NET Core and .NET Standard.
* `Node.js <https://www.npmjs.com/package/rclnodejs>`__ rclnodejs is a Node.js client for ROS 2. It provides a simple and easy JavaScript API for ROS 2 programming.
* `Rust <https://github.com/ros2-rust/ros2_rust>`__ This is a set of projects (the rclrs client library, code generator, examples and more) that enables developers to write ROS 2 applications in Rust.
* `Flutter and Dart <https://github.com/rcldart>`__ Flutter and Dart bindings for ROS 2.

Older, unmaintained client libraries are:

* `C# <https://github.com/firesurfer/rclcs>`__
* `Objective C and iOS <https://github.com/esteve/ros2_objc>`__
* `Zig <https://github.com/jacobperron/rclzig>`__


通用功能: ``rcl``
-----------------------------

客户端库中的大部分功能都不是特定于客户端库的编程语言的。
例如，在理想情况下，参数的行为和命名空间的逻辑应该在所有编程语言中是相同的。
因此，客户端库并非从头开始实现常见功能，而是使用非特定语言限制的 ROS 概念的通用核心 ROS RCL 接口。
所以一般客户端库只需要用外部函数接口包装 RCL 中的通用功能。
这样可以使客户端库更轻量化，更容易开发。
为此，常见的 RCL 功能以 C 接口的形式暴露出来，因为 C 语言通常是客户端库最容易封装的语言。

除了使客户端库更轻量化之外，有一个使用通用核心的优势是，不同语言之间的行为更一致。
如果对 RCL 核心功能的逻辑/行为进行了任何更改 -- 例如命名空间 -- 所有使用 RCL 的客户端库都会反映这些更改。
此外，拥有通用核心意味着在修复 bug 时，维护多个客户端库的工作量会大大减少。

``rcl`` 的 API 文档可以在 `这里 <https://docs.ros.org/en/{DISTRO}/p/rcl/>`__ 找到.

Language-specific 功能
-------------------------------

客户端库中需要特定于语言的特性/属性的概念并没有在 RCL 中实现，而是在每个客户端库中实现。
例如，由 “spin” 函数使用的线程模型将由特定于客户端库语言来实现。.

Demo
----

为了演示 ``rclpy`` 发布者和 ``rclcpp`` 订阅者之间的消息交换，我们推荐你观看 `这个 ROSCon talk <https://vimeo.com/187696091>`__，从 17:25 开始（ `这里查看幻灯片 <https://roscon.ros.org/2016/presentations/ROSCon%202016%20-%20ROS%202%20Update.pdf>`__ ）。

与 ROS 1 的对比
-------------------

在 ROS 1 中，所有客户端库都是从头开始开发的。
这使得 ROS 1 Python 客户端库可以纯粹地用 Python 实现，例如，这样就不需要编译代码。
然而，客户端库之间的命名约定和行为并不总是一致，bug 修复必须在多个地方完成，还有很多功能只在一个客户端库中实现（例如 UDPROS）。

总结
-------

通过使用通用的核心 ROS 客户端库，更容易编写使用不同编程语言的客户端库，并且行为更一致。
