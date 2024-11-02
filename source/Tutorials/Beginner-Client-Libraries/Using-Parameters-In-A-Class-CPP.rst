.. redirect-from::

    Tutorials/Using-Parameters-In-A-Class-CPP

.. _CppParamNode:

在类中使用参数 (C++)
=================================

**目标:** 用 C++ 创建并运行带有参数的类.

**教程等级:** 初级

**预计时长:** 20 分钟

.. contents:: Contents
   :depth: 2
   :local:

背景
----------

在实现你自己的 :doc:`nodes <../Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes>` 时，有时需要添加可以从启动文件(launch file)中设置的参数。

本教程将向您展示如何在 C++ 类中创建这些参数，以及如何在启动文件中设置它们。

前提条件
-------------

在之前的教程中，你学习了如何 :doc:`创建工作空间 <./Creating-A-Workspace/Creating-A-Workspace>` 和 :doc:`创建包 <./Creating-Your-First-ROS2-Package>`。
你还学习了 :doc:`参数 <../Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters>` 及其在 ROS 2 系统中的功能。

任务
-----

1 创建包
^^^^^^^^^^^^^^^^^^

打开一个新的终端并 :doc:`source ROS 2 环境 <../Beginner-CLI-Tools/Configuring-ROS2-Environment>`，这样 ``ros2`` 指令就能正常工作。

按照 :ref:`这些说明 <new-directory>` 创建一个名为 ``ros2_ws`` 的新工作空间。


回想一下之前学到的，包应该在 ``src`` 目录中创建，而不是工作空间的根目录.
所以，进入 ``ros2_ws/src`` 目录，并运行包创建指令:

.. code-block:: console

  ros2 pkg create --build-type ament_cmake --license Apache-2.0 cpp_parameters --dependencies rclcpp

你的终端会返回一个消息，确认了包 ``cpp_parameters`` 及其所有必要文件和文件夹已经创建。

``--dependencies`` 参数会自动将必要的依赖添加到 ``package.xml`` 和 ``CMakeLists.txt`` 中。

1.1 更新 ``package.xml``
~~~~~~~~~~~~~~~~~~~~~~~~~~

因为在包创建过程中使用了 ``--dependencies`` 选项，所以不需要手动将依赖项添加到 ``package.xml`` 或 ``CMakeLists.txt``。

但是，记得将描述、维护者电子邮件和姓名以及许可证信息添加到 ``package.xml`` 中。、

.. code-block:: xml

  <description>C++ parameter tutorial</description>
  <maintainer email="you@email.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

2 编写 C++ 节点
^^^^^^^^^^^^^^^^^^^^

在 ``ros2_ws/src/cpp_parameters/src`` 目录中创建一个名为 ``cpp_parameters_node.cpp`` 的新文件，并粘贴以下代码:

.. code-block:: C++

    #include <chrono>
    #include <functional>
    #include <string>

    #include <rclcpp/rclcpp.hpp>

    using namespace std::chrono_literals;

    class MinimalParam : public rclcpp::Node
    {
    public:
      MinimalParam()
      : Node("minimal_param_node")
      {
        this->declare_parameter("my_parameter", "world");

        timer_ = this->create_wall_timer(
          1000ms, std::bind(&MinimalParam::timer_callback, this));
      }

      void timer_callback()
      {
        std::string my_param = this->get_parameter("my_parameter").as_string();

        RCLCPP_INFO(this->get_logger(), "Hello %s!", my_param.c_str());

        std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("my_parameter", "world")};
        this->set_parameters(all_new_parameters);
      }

    private:
      rclcpp::TimerBase::SharedPtr timer_;
    };

    int main(int argc, char ** argv)
    {
      rclcpp::init(argc, argv);
      rclcpp::spin(std::make_shared<MinimalParam>());
      rclcpp::shutdown();
      return 0;
    }

2.1 测试代码
~~~~~~~~~~~~~~~~~~~~
最前面的 ``#include`` 语句是包的依赖。

接下来的代码片段创建了一个类和构造函数。
构造函数的第一行创建了一个名为 ``my_parameter`` 的参数，其默认值为 ``world``。
参数类型是从默认值推断出来的，所以在这种情况下，它将被设置为字符串类型。
接下来， ``timer_`` 使用 1000ms 的周期初始化，这会使 ``timer_callback`` 函数每秒执行一次。

.. code-block:: C++

    class MinimalParam : public rclcpp::Node
    {
    public:
      MinimalParam()
      : Node("minimal_param_node")
      {
        this->declare_parameter("my_parameter", "world");

        timer_ = this->create_wall_timer(
          1000ms, std::bind(&MinimalParam::timer_callback, this));
      }

``timer_callback`` 函数的第一行从节点中获取参数 ``my_parameter``，并将其存储在 ``my_param`` 中。
接下来， ``RCLCPP_INFO`` 函数记录一条 log。
最后， ``set_parameters`` 函数将参数 ``my_parameter`` 设置回默认值 ``world``。
这样，即使用户在外部更改了参数，也能确保它总是被重置回原始值。

.. code-block:: C++

    void timer_callback()
    {
      std::string my_param = this->get_parameter("my_parameter").as_string();

      RCLCPP_INFO(this->get_logger(), "Hello %s!", my_param.c_str());

      std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("my_parameter", "world")};
      this->set_parameters(all_new_parameters);
    }

最后是 ``timer_`` 的声明。

.. code-block:: C++

    private:
      rclcpp::TimerBase::SharedPtr timer_;

``MinimalParam`` 后面的是 ``main``。
这里初始化了 ROS 2，创建了 ``MinimalParam`` 类的一个实例，并启动了 ``rclcpp::spin`` 来处理节点的数据。

.. code-block:: C++

    int main(int argc, char ** argv)
    {
      rclcpp::init(argc, argv);
      rclcpp::spin(std::make_shared<MinimalParam>());
      rclcpp::shutdown();
      return 0;
    }

2.1.1 (可选) 添加参数描述 (ParameterDescriptor)
"""""""""""""""""""""""""""""""""""""""""""""""""""""""
你可以为参数设置对应的描述。
Descriptor 使得你可以指定参数的文本描述和约束，比如设置为只读，指定范围等。
为了使这个功能生效，需要修改构造函数中的代码为:

.. code-block:: C++

    // ...

    class MinimalParam : public rclcpp::Node
    {
    public:
      MinimalParam()
      : Node("minimal_param_node")
      {
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "This parameter is mine!";

        this->declare_parameter("my_parameter", "world", param_desc);

        timer_ = this->create_wall_timer(
          1000ms, std::bind(&MinimalParam::timer_callback, this));
      }

其他部份的代码保持不变。
运行节点后，就可以运行 ``ros2 param describe /minimal_param_node my_parameter`` 来查看类型和描述。


2.2 添加可执行文件
~~~~~~~~~~~~~~~~~~~~~~

现在打开 ``CMakeLists.txt`` 文件。在依赖项 ``find_package(rclcpp REQUIRED)`` 下面添加以下代码。

.. code-block:: cmake

    add_executable(minimal_param_node src/cpp_parameters_node.cpp)
    ament_target_dependencies(minimal_param_node rclcpp)

    install(TARGETS
        minimal_param_node
      DESTINATION lib/${PROJECT_NAME}
    )


3 构建和运行
^^^^^^^^^^^^^^^

在构建之前，最好在工作空间的根目录 (``ros2_ws``) 中运行 ``rosdep``，检查是否有缺少的依赖项:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        rosdep install -i --from-path src --rosdistro {DISTRO} -y

   .. group-tab:: macOS

      rosdep only runs on Linux, so you can skip ahead to next step.

   .. group-tab:: Windows

      rosdep only runs on Linux, so you can skip ahead to next step.

然后返回到工作空间的根目录， ``ros2_ws``，并构建包:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      colcon build --packages-select cpp_parameters

  .. group-tab:: macOS

    .. code-block:: console

      colcon build --packages-select cpp_parameters

  .. group-tab:: Windows

    .. code-block:: console

      colcon build --merge-install --packages-select cpp_parameters

打开一个新的终端，进入 ``ros2_ws``，并 source 配置文件:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      source install/setup.bash

  .. group-tab:: macOS

    .. code-block:: console

      . install/setup.bash

  .. group-tab:: Windows

    .. code-block:: console

      call install/setup.bat

现在运行节点:

.. code-block:: console

     ros2 run cpp_parameters minimal_param_node

终端应该每秒都会返回以下消息:

.. code-block:: console

    [INFO] [minimal_param_node]: Hello world!

现在你可以看到参数的默认值，但你想要能够自己设置它。
有两种方法可以实现这一点。

3.1 从终端修改
~~~~~~~~~~~~~~~~~~~~~~~~~~

这部分将使用你从 :doc:`关于参数的教程 <../Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters>` 中学到的知识，并将其应用到你刚刚创建的节点。

确保节点正在运行:

.. code-block:: console

     ros2 run cpp_parameters minimal_param_node

打开另一个终端，再次 source ``ros2_ws`` 中的配置文件，然后输入以下命令:

.. code-block:: console

    ros2 param list

你会看到自定义参数 ``my_parameter``。
在终端中运行以下命令即可修改它:

.. code-block:: console

    ros2 param set /minimal_param_node my_parameter earth

如果输出为 ``Set parameter successful``，则说明设置成功。
此时查看另一个终端，应该能看到输出变为 ``[INFO] [minimal_param_node]: Hello earth!``

3.2 从启动文件修改
~~~~~~~~~~~~~~~~~~~~~~~~~~~~
也可以从启动文件中设置参数，但首先需要添加启动目录。
在 ``ros2_ws/src/cpp_parameters/`` 目录中创建一个名为 ``launch`` 的新目录。
在这创建一个名为 ``cpp_parameters_launch.py`` 的新文件。


.. code-block:: Python

  from launch import LaunchDescription
  from launch_ros.actions import Node

  def generate_launch_description():
      return LaunchDescription([
          Node(
              package="cpp_parameters",
              executable="minimal_param_node",
              name="custom_minimal_param_node",
              output="screen",
              emulate_tty=True,
              parameters=[
                  {"my_parameter": "earth"}
              ]
          )
      ])

从中可以看到，我们在启动节点 ``minimal_param_node`` 时将 ``my_parameter`` 设置为 ``earth``。
通过添加以下两行，我们输出能够打印在控制台中。

.. code-block:: console

          output="screen",
          emulate_tty=True,

打开 ``CMakeLists.txt`` 文件。
在你之前添加的代码之后再添加以下代码。

.. code-block:: console

    install(
      DIRECTORY launch
      DESTINATION share/${PROJECT_NAME}
    )

打开新终端，进入工作空间的根目录， ``ros2_ws``，并构建包:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      colcon build --packages-select cpp_parameters

  .. group-tab:: macOS

    .. code-block:: console

      colcon build --packages-select cpp_parameters

  .. group-tab:: Windows

    .. code-block:: console

      colcon build --merge-install --packages-select cpp_parameters

然后 source 配置文件:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      source install/setup.bash

  .. group-tab:: macOS

    .. code-block:: console

      . install/setup.bash

  .. group-tab:: Windows

    .. code-block:: console

      call install/setup.bat

现在使用我们刚刚创建的启动文件运行节点:

.. code-block:: console

     ros2 launch cpp_parameters cpp_parameters_launch.py

终端应该每秒都会返回以下消息:

.. code-block:: console

    [INFO] [custom_minimal_param_node]: Hello earth!

总结
-------

你创建了一个带有自定义参数的节点，可以从启动文件或命令行中设置。
你在包配置文件中添加了依赖项、可执行文件和启动文件，以便构建和运行它们，并查看参数的作用。

下一步
----------

现在你有了自己的包， :doc:`下一个教程 <./Getting-Started-With-Ros2doctor>` 将向你展示如何检查环境和系统中的问题，以便在出现问题时解决。
