.. redirect-from::

  Tutorials/Launch-system
  Tutorials/Launch-Files/Launch-system
  Tutorials/Launch/Launch-system

将启动文件集成到 ROS 2 包中
============================================

**目标:** 在 ROS 2 包中添加启动文件。

**教程等级:** 中级

**预计时长:** 10 分钟

.. contents:: Contents
   :depth: 2
   :local:

前提条件
-------------

你应该已经学习了如何 :doc:`创建 ROS 2 包 <../../Beginner-Client-Libraries/Creating-Your-First-ROS2-Package>`。

别忘记在 :doc:`每个新终端中都要 source ROS 2 环境 <../../Beginner-CLI-Tools/Configuring-ROS2-Environment>`。

背景
----------

在 :doc:`上一个教程 <Creating-Launch-Files>` 中，我们知道了如何编写单独的启动文件。
本教程将会展示如何将启动文件添加到现有的包中，以及在使用中一些约定俗成的习惯。

任务
-----

1 创建包
^^^^^^^^^^^^^^^^^^

为这个包创建一个工作空间：

.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

      mkdir -p launch_ws/src
      cd launch_ws/src

  .. group-tab:: macOS

    .. code-block:: bash

      mkdir -p launch_ws/src
      cd launch_ws/src

  .. group-tab:: Windows

    .. code-block:: bash

      md launch_ws\src
      cd launch_ws\src

.. tabs::

  .. group-tab:: Python package

    .. code-block:: console

      ros2 pkg create --build-type ament_python --license Apache-2.0 py_launch_example

  .. group-tab:: C++ package

    .. code-block:: console

      ros2 pkg create --build-type ament_cmake --license Apache-2.0 cpp_launch_example

2 为启动文件配置好文件结构
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

习惯上，一个包的所有启动文件都放在包内的 ``launch`` 文件夹中。
请确保你创建的 ``launch`` 文件夹在包的最顶层目录下。

.. tabs::

  .. group-tab:: Python package

    对于 Python 包，文件结构应该如下所示：

    .. code-block:: console

      src/
        py_launch_example/
          launch/
          package.xml
          py_launch_example/
          resource/
          setup.cfg
          setup.py
          test/

    为了让 colcon 能够定位到启动文件，我们需要让 Python 的 setup tools 知道它们的存在。
    为此，打开 ``setup.py`` 文件，在顶部添加必要的 ``import`` 语句，并将启动文件包含到 ``setup`` 的 ``data_files`` 参数中：

    .. code-block:: python

      import os
      from glob import glob
      # Other imports ...

      package_name = 'py_launch_example'

      setup(
          # Other parameters ...
          data_files=[
              # ... Other data files
              # Include all launch files.
              (os.path.join('share', package_name, 'launch'), glob('launch/*'))
          ]
      )

  .. group-tab:: C++ package

    对于 C++ 包，我们只需要在 ``CMakeLists.txt`` 文件的末尾， ``ament_package()`` 之前添加：

    .. code-block:: cmake

      # Install launch files.
      install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}/
      )


3 编写启动文件
^^^^^^^^^^^^^^^^^^^^^^^^^

.. tabs::

  .. group-tab:: Python launch file

    在 ``launch`` 文件夹中创建一个名为 ``my_script_launch.py`` 的新启动文件。
    推荐使用 ``_launch.py`` 作为 Python 启动文件的文件后缀，但不是必须的。
    不过，启动文件的名称必须以 ``launch.py`` 结尾，以便被 ``ros2 launch`` 识别和自动补全。

    你的启动文件应该定义 ``generate_launch_description()`` 函数，该函数返回一个 ``launch.LaunchDescription()``，供 ``ros2 launch`` 命令使用。

    .. code-block:: python

      import launch
      import launch_ros.actions

      def generate_launch_description():
          return launch.LaunchDescription([
              launch_ros.actions.Node(
                  package='demo_nodes_cpp',
                  executable='talker',
                  name='talker'),
        ])

  .. group-tab:: XML launch file

    在 ``launch`` 文件夹中创建一个名为 ``my_script_launch.xml`` 的新启动文件。
    推荐使用 ``_launch.xml`` 作为 XML 启动文件的文件后缀，但不是必须的。

    .. code-block:: xml

      <launch>
        <node pkg="demo_nodes_cpp" exec="talker" name="talker"/>
      </launch>

  .. group-tab:: YAML launch file

    在 ``launch`` 文件夹中创建一个名为 ``my_script_launch.yaml`` 的新启动文件。
    推荐使用 ``_launch.yaml`` 作为 YAML 启动文件的文件后缀，但不是必须的。

    .. code-block:: yaml

      launch:

      - node:
          pkg: "demo_nodes_cpp"
          exec: "talker"
          name: "talker"


4 构建并运行启动文件
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

回到工作空间的最顶层，构建它：

.. code-block:: console

  colcon build

构建成功后，source 工作空间，就能够运行启动文件了：

.. tabs::

  .. group-tab:: Python package

    .. tabs::

      .. group-tab:: XML launch file

        .. code-block:: console

          ros2 launch py_launch_example my_script_launch.xml

      .. group-tab:: YAML launch file

        .. code-block:: console

          ros2 launch py_launch_example my_script_launch.yaml

      .. group-tab:: Python launch file

        .. code-block:: console

          ros2 launch py_launch_example my_script_launch.py

  .. group-tab:: C++ package

    .. tabs::

      .. group-tab:: XML launch file

        .. code-block:: console

          ros2 launch cpp_launch_example my_script_launch.xml

      .. group-tab:: YAML launch file

        .. code-block:: console

          ros2 launch cpp_launch_example my_script_launch.yaml


文档
-------------

`launch 文档 <https://github.com/ros2/launch/blob/{REPOS_FILE_BRANCH}/launch/doc/source/architecture.rst>`__ 提供了更多关于 ``launch_ros`` 中使用到的概念的详细信息。

更多的文档/示例将会在不久后发布。
在此期间，可以查看源代码 (https://github.com/ros2/launch 和 https://github.com/ros2/launch_ros)。
