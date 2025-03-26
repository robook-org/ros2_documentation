.. redirect-from::

    Tutorials/Using-Parameters-In-A-Class-Python

.. _PythonParamNode:

在类中使用参数 (Python)
====================================

**目标:** 用 Python 创建并运行带有参数的类.

**教程等级:** 初级

**预计时长:** 20 分钟

.. contents:: Contents
   :depth: 2
   :local:

背景
----------

在实现你自己的 :doc:`nodes <../Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes>` 时，有时需要添加可以从启动文件(launch file)中设置的参数。

本教程将向你展示如何在 Python 类中创建这些参数，并如何在启动文件中设置它们。

前提条件
-------------

在之前的教程中，你学习了如何 :doc:`创建工作空间 <./Creating-A-Workspace/Creating-A-Workspace>` 和 :doc:`创建包 <./Creating-Your-First-ROS2-Package>`。
你还了解了 :doc:`参数 <../Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters>` 及其在 ROS 2 系统中的功能。

任务
-----

1 创建包
^^^^^^^^^^^^^^^^^^

打开一个新的终端并 :doc:`配置你的 ROS 2 安装环境 <../Beginner-CLI-Tools/Configuring-ROS2-Environment>`，这样 ``ros2`` 指令就能正常工作。

按照 :ref:`这些指令 <new-directory>` 创建一个名为 ``ros2_ws`` 的新工作空间。

回想一下之前学到的，包应该在 ``src`` 目录中创建，而不是工作空间的根目录.
所以，进入 ``ros2_ws/src`` 目录，并运行包创建指令:

.. code-block:: console

  ros2 pkg create --build-type ament_python --license Apache-2.0 python_parameters --dependencies rclpy

你的终端会返回一个消息，确认了包 ``cpp_parameters`` 及其所有必要文件和文件夹已经创建。

``--dependencies`` 参数会自动将必要的依赖添加到 ``package.xml`` 和 ``CMakeLists.txt`` 中。

1.1 更新 ``package.xml``
~~~~~~~~~~~~~~~~~~~~~~~~~~

因为在包创建过程中使用了 ``--dependencies`` 选项，所以不需要手动将依赖项添加到 ``package.xml`` 或 ``CMakeLists.txt``。

但是，记得将描述、维护者电子邮件和姓名，以及许可证信息添加到 ``package.xml`` 中。

.. code-block:: xml

  <description>Python parameter tutorial</description>
  <maintainer email="you@email.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

2 编写 Python 代码
^^^^^^^^^^^^^^^^^^^^^^^

在 ``ros2_ws/src/python_parameters/python_parameters`` 目录中创建一个名为 ``python_parameters_node.py`` 的新文件，并粘贴以下代码:

.. code-block:: Python

    import rclpy
    import rclpy.node

    class MinimalParam(rclpy.node.Node):
        def __init__(self):
            super().__init__('minimal_param_node')

            self.declare_parameter('my_parameter', 'world')

            self.timer = self.create_timer(1, self.timer_callback)

        def timer_callback(self):
            my_param = self.get_parameter('my_parameter').get_parameter_value().string_value

            self.get_logger().info('Hello %s!' % my_param)

            my_new_param = rclpy.parameter.Parameter(
                'my_parameter',
                rclpy.Parameter.Type.STRING,
                'world'
            )
            all_new_parameters = [my_new_param]
            self.set_parameters(all_new_parameters)

    def main():
        rclpy.init()
        node = MinimalParam()
        rclpy.spin(node)

    if __name__ == '__main__':
        main()



2.1 检查代码
~~~~~~~~~~~~~~~~~~~~
文件最开头的 ``import`` 语句用于导入包依赖项。

接下来的代码段创建了类和构造函数。
构造函数的 ``self.declare_parameter('my_parameter', 'world')`` 创建了一个名为 ``my_parameter`` 的参数，并将其默认值设置为 ``world``。
参数类型是从默认值推断出来的，所以在这种情况下，它将被设置为字符串类型。
然后 ``timer`` 使用 1 秒的周期初始化，这会使得每秒执行一次 ``timer_callback`` 函数。

.. code-block:: Python

    class MinimalParam(rclpy.node.Node):
        def __init__(self):
            super().__init__('minimal_param_node')

            self.declare_parameter('my_parameter', 'world')

            self.timer = self.create_timer(1, self.timer_callback)

接下来的 ``timer_callback`` 函数的第一行从节点中获取参数 ``my_parameter``，并将其存储在 ``my_param`` 中。
后续 ``get_logger`` 函数确保事件被记录。
然后 ``set_parameters`` 函数将参数 ``my_parameter`` 重置为默认字符串值 ``world``。
这样，即使用户在外部更改了参数，也能确保它总是被重置为原始值。

.. code-block:: Python

      def timer_callback(self):
          my_param = self.get_parameter('my_parameter').get_parameter_value().string_value

          self.get_logger().info('Hello %s!' % my_param)

          my_new_param = rclpy.parameter.Parameter(
              'my_parameter',
              rclpy.Parameter.Type.STRING,
              'world'
          )
          all_new_parameters = [my_new_param]
          self.set_parameters(all_new_parameters)

在 ``timer_callback`` 后面的是 ``main``。
这里初始化了 ROS 2,构造了 ``MinimalParam`` 类的一个实例，并启动了 ``rclpy.spin`` 来处理节点的数据。

.. code-block:: Python

    def main():
        rclpy.init()
        node = MinimalParam()
        rclpy.spin(node)

    if __name__ == '__main__':
        main()


2.1.1 (可选) 添加参数描述 (ParameterDescriptor)
""""""""""""""""""""""""""""""""""""""""""""""""""""
你可以为参数设置对应的描述。
这样做可以指定参数的文本描述和约束，比如设置为只读，指定范围等。
为了使这个功能生效， ``__init__`` 代码必须更改为:

.. code-block:: Python

    # ...

    class MinimalParam(rclpy.node.Node):
        def __init__(self):
            super().__init__('minimal_param_node')

            from rcl_interfaces.msg import ParameterDescriptor
            my_parameter_descriptor = ParameterDescriptor(description='This parameter is mine!')

            self.declare_parameter('my_parameter', 'world', my_parameter_descriptor)

            self.timer = self.create_timer(1, self.timer_callback)


其余代码保持不变。
运行节点后，你可以运行 ``ros2 param describe /minimal_param_node my_parameter`` 来查看类型和描述。

2.2 添加 entry point
~~~~~~~~~~~~~~~~~~~~~~

打开 ``setup.py`` 文件。
填写 ``maintainer``、``maintainer_email``、``description`` 和 ``license`` 字段，使其与 ``package.xml`` 匹配:

.. code-block:: python

  maintainer='YourName',
  maintainer_email='you@email.com',
  description='Python parameter tutorial',
  license='Apache License 2.0',

接下来，将以下行添加到 ``entry_points`` 字段的 ``console_scripts`` 括号内:

.. code-block:: python

  entry_points={
      'console_scripts': [
          'minimal_param_node = python_parameters.python_parameters_node:main',
      ],
  },

别忘了保存。


3 构建和运行
^^^^^^^^^^^^^^^

在构建之前，你需要在工作空间的根目录(``ros2_ws``)中运行 ``rosdep`` 检查是否有缺失的依赖项。

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        rosdep install -i --from-path src --rosdistro {DISTRO} -y

   .. group-tab:: macOS

      rosdep only runs on Linux, so you can skip ahead to next step.

   .. group-tab:: Windows

      rosdep only runs on Linux, so you can skip ahead to next step.

然后在工作空间的根目录(``ros2_ws``)构建包:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      colcon build --packages-select python_parameters

  .. group-tab:: macOS

    .. code-block:: console

      colcon build --packages-select python_parameters

  .. group-tab:: Windows

    .. code-block:: console

      colcon build --merge-install --packages-select python_parameters

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

     ros2 run python_parameters minimal_param_node

终端应该每秒都会返回以下消息:

.. code-block:: console

    [INFO] [parameter_node]: Hello world!

现在你可以看到参数的默认值，但你希望能够自己设置它。
有两种方法可以实现这一点。

3.1 从终端修改
~~~~~~~~~~~~~~~~~~~~~~~~~~

这部分将使用从 :doc:`关于参数的教程 <../Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters>` 中学到的知识，并将其应用到你刚刚创建的节点。

确保节点正在运行:

.. code-block:: console

     ros2 run python_parameters minimal_param_node

打开另一个终端，再次从 ``ros2_ws`` 中 source 配置文件，然后输入以下命令:

.. code-block:: console

    ros2 param list

能看到自定义参数 ``my_parameter``。
只需运行以下命令即可更改它:

.. code-block:: console

    ros2 param set /minimal_param_node my_parameter earth

如果输出为 ``Set parameter successful``，则说明设置成功。
此时查看另一个终端，应该能看到输出变为 ``[INFO] [minimal_param_node]: Hello earth!``

由于节点随后将参数设置回 ``world``，因此后续输出显示 ``[INFO] [minimal_param_node]: Hello world!``

3.2 从启动文件修改
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

你也可以在启动文件中设置参数，但首先需要添加一个启动目录。
在 ``ros2_ws/src/python_parameters/`` 目录中创建一个名为 ``launch`` 的新目录。
创建一个名为 ``python_parameters_launch.py`` 的新文件。

.. code-block:: Python

  from launch import LaunchDescription
  from launch_ros.actions import Node

  def generate_launch_description():
      return LaunchDescription([
          Node(
              package='python_parameters',
              executable='minimal_param_node',
              name='custom_minimal_param_node',
              output='screen',
              emulate_tty=True,
              parameters=[
                  {'my_parameter': 'earth'}
              ]
          )
      ])

可以看到，我们在启动节点 ``parameter_node`` 时将 ``my_parameter`` 设置为 ``earth``。
下面这两行确保输出能在控制台中打印。

.. code-block:: console

          output="screen",
          emulate_tty=True,

打开 ``setup.py`` 文件。
在最开头的 ``import`` 语句中添加以下内容，以及在 ``data_files`` 参数中添加一些新内容，来包含所有启动文件:

.. code-block:: Python

    import os
    from glob import glob
    # ...

    setup(
      # ...
      data_files=[
          # ...
          (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        ]
      )

打开一个新的终端，进入 ``ros2_ws``，构建包:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      colcon build --packages-select python_parameters

  .. group-tab:: macOS

    .. code-block:: console

      colcon build --packages-select python_parameters

  .. group-tab:: Windows

    .. code-block:: console

      colcon build --merge-install --packages-select python_parameters

然后在新终端中 source 配置文件:

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

     ros2 launch python_parameters python_parameters_launch.py

终端应该每秒都会返回以下消息:

.. code-block:: console

    [INFO] [custom_minimal_param_node]: Hello earth!


总结
-------

你创建了一个带有自定义参数的节点，可以从启动文件或命令行中设置。
你在包配置文件中添加了依赖项、可执行文件和启动文件，以便构建和运行它们，并查看参数的作用。

下一步
----------

现在你有了自己的包和 ROS 2 系统， :doc:`下一个教程 <./Getting-Started-With-Ros2doctor>` 将向你展示如何检查环境和系统中的问题，以便在出现问题时解决。
