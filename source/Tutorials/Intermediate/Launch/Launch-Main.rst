.. redirect-from::

    Tutorials/Launch-Files/Launch-Main
    Tutorials/Launch/Launch-Main

.. _LaunchFilesMain:

启动(Launch)
==================

通过 ROS 2 启动文件，可以同时启动和配置多个包含 ROS 2 节点的可执行文件。

.. toctree::
   :hidden:

   Creating-Launch-Files
   Launch-system
   Using-Substitutions
   Using-Event-Handlers
   Using-ROS2-Launch-For-Large-Projects

#. :doc:`创建启动文件 <./Creating-Launch-Files>`.

   了解如何创建启动文件，一次性启动并配置节点。

#. :doc:`运行并监控多个节点 <./Launch-system>`.

   更深入地了解启动文件的工作原理。

#. :doc:`使用可被替换的变量 <./Using-Substitutions>`.

   在描述可重复使用的启动文件时，使用可被替换的变量可以提供更大的灵活性。

#. :doc:`使用事件处理器 <./Using-Event-Handlers>`.

   使用事件处理程序监控进程状态，或定义一组复杂的规则，用于动态修改启动文件。

#. :doc:`管理大型项目 <./Using-ROS2-Launch-For-Large-Projects>`.

   为大型项目构建启动文件，以便在不同情况下尽可能重复使用。
   查看不同启动工具的使用示例，如参数、YAML 文件、重映射、命名空间、默认参数和 RViz 配置。

.. note::

   如果你之前用的是 ROS 1， :doc:`ROS 启动文件迁移指南 <../../../How-To-Guides/Migrating-from-ROS1/Migrating-Launch-Files>` 可以帮助你迁移到 ROS 2。
