术语表
========

.. include:: ../global_substitutions.txt

Glossary of terms used throughout this documentation:

.. glossary::

   API
       An API, or Application Programming Interface, is an interface that is provided by an "application", which in this case is usually a shared library or other language appropriate shared resource. APIs are made up of files that define a contract between the software using the interface and the software providing the interface. These files typically manifest as header files in C and C++ and as Python files in Python. In either case it is important that APIs are grouped and described in documentation and that they are declared as either public or private. Public interfaces are subject to change rules and changes to the public interfaces prompt a new version number of the software that provides them.

   client_library
       A client library is an :term:`API` that provides access to the ROS graph using primitive middleware concepts like Topics, Services, and Actions.

   package
       A single unit of software, including source code, build system files, documentation, tests, and other associated resources.

   REP
        ROS Enhancement Proposal. A document that describes an enhancement, standardization, or convention for the ROS community.
        The associated REP approval process allows the community to iterate on a proposal until some consensus has been made, at which point it can be ratified and implemented, which then becomes documentation.
        All REPs are viewable from the `REP index <http://www.ros.org/reps/rep-0000.html>`_.

   VCS
       Version Control System, such as CVS, SVN, git, mercurial, etc...

   rclcpp
       The C++ specific :term:`Client Library <client_library>` for ROS. This includes any middleware related APIs as well as the related message generation of C++ data structures based on interface definitions like Messages, Services, and Actions.

   repository
       A collection of packages usually managed using a :term:`VCS` like git or mercurial and usually hosted on a site like GitHub or BitBucket.
       In the context of this document, repositories usually contain one or more |packages| of one type or another.

翻译对照表
------------

.. glossary::
    :sorted:

    workspace
        工作空间, ROS 2 中的工作空间是一个包含需要使用的 ROS 2 包的目录。工作空间中的包可以被构建、安装和运行。它的存在是为了方便用户组织和管理 ROS 2 包，尤其是在一台设备上拥有多个ROS版本的包或者多个不用依赖的项目时。

    build
        构建, 在 ROS 2 中，构建是指将源代码转换为可执行文件的过程。构建过程通常包括编译、链接和安装等。由于一些计算机学科的历史发展原因，或者在其他领域已经被广泛使用的用法的影响下，在交流中我们也有可能直接将这个过程称为编译。

    source
        执行、运行，这是一个Linux指令，意味着运行一个文件。在 ROS2 中，这个指令通常用于运行配置工作空间的脚本。

    node
        节点。

    service
        服务。

    terminal
        终端。

    command-line
        命令行。

    call
        调用。当表达 call a service 时，意为调用一个服务。

    remap
        重映射。在 ROS 2 多数语境中，重映射是指将一个节点的某个话题或服务的名称映射到另一个节点的话题或服务的名称。这样可以在不修改源代码的情况下改变节点之间的通信方式。

    request
        请求。

    response
        响应，回复。

    field
        字段。

    strong-typed
        类型严格。也叫强类型。

    concepts
        概念。

    timer
        定时器。

    callback group
        回调函数组,回调组。

    subscription
        订阅。

    executor
        执行器。

    example
        示例，样例，例子。

    deadlock
        死锁。

    bus
        总线。
