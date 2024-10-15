.. _Colcon:

.. redirect-from::

    Colcon-Tutorial
    Tutorials/Colcon-Tutorial

使用 ``colcon`` 构建包
==================================

.. contents:: Table of Contents
   :depth: 2
   :local:

**目标:** 使用 ``colcon`` 构建一个 ROS 2 工作空间.

**教程等级:** 初级

**预计时长:** 20 分钟

这是一个关于如何使用 ``colcon`` 创建和构建 ROS 2 工作空间的简短教程。
这是一个实用教程，不是为了取代核心文档。

背景
----------

``colcon`` 是 ROS 构建工具 ``catkin_make``、 ``catkin_make_isolated``、 ``catkin_tools`` 和 ``ament_tools`` 的迭代版本。
更多关于 ``colcon`` 设计的信息请参考 `这篇文档 <https://design.ros2.org/articles/build_tool.html>`__。

``colcon`` 的源代码可以在 `colcon GitHub organization <https://github.com/colcon>`__ 中找到.

前提条件
-------------

安装 colcon
^^^^^^^^^^^^^^

.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

        sudo apt install python3-colcon-common-extensions

  .. group-tab:: macOS

    .. code-block:: bash

        python3 -m pip install colcon-common-extensions

  .. group-tab:: Windows

    .. code-block:: bash

        pip install -U colcon-common-extensions


安装 ROS 2
^^^^^^^^^^^^^

为了构建示例，你需要安装 ROS 2。

请按照 :doc:`安装指南 <../../Installation>` 进行安装。

.. attention:: 如果从 deb 包安装，请参考 :ref:`桌面版安装 <linux-install-debs-install-ros-2-packages>`。

基础知识
---------

ROS 工作空间是一个具有特定结构的目录。
通常有一个 ``src`` 子目录。
在这个子目录中是 ROS 包的源代码。
最开始这个目录一般是空的。

``colcon`` 支持在源代码之外构建。（译者注：就是说 colcon 可以不在源码的目录下运行）
默认情况下，它会创建以下目录作为 ``src`` 目录的同级目录:

* ``build`` 目录是存放中间文件的地方。
  对于每个包，都会创建一个子目录，其中会调用 CMake 等工具。
* ``install`` 目录是每个包将被安装到的地方。
  默认情况下，每个包将被安装到单独的子目录中。
* ``log`` 目录包含有关每个 colcon 调用的各种日志信息。

.. note:: 与 catkin 相比，没有 ``devel`` 目录。

创建一个 workspace
^^^^^^^^^^^^^^^^^^

首先，创建一个目录（``ros2_ws``）来包含我们的工作空间:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

       mkdir -p ~/ros2_ws/src
       cd ~/ros2_ws

  .. group-tab:: macOS

    .. code-block:: bash

       mkdir -p ~/ros2_ws/src
       cd ~/ros2_ws

  .. group-tab:: Windows

    .. code-block:: bash

       md \dev\ros2_ws\src
       cd \dev\ros2_ws

此时工作空间中只有一个空目录 ``src``:

.. code-block:: bash

    .
    └── src

    1 directory, 0 files

添加一些资源
^^^^^^^^^^^^^^^^

让我们将 `examples <https://github.com/ros2/examples>`__ 仓库克隆到工作空间里的 ``src`` 目录:

.. code-block:: bash

    git clone https://github.com/ros2/examples src/examples -b {REPOS_FILE_BRANCH}

现在工作空间应该已经有 ROS 2 示例的源代码:

.. code-block:: bash

    .
    └── src
        └── examples
            ├── CONTRIBUTING.md
            ├── LICENSE
            ├── rclcpp
            ├── rclpy
            └── README.md

    4 directories, 3 files

Source an underlay
^^^^^^^^^^^^^^^^^^

为了构建示例包，我们需要为我们的工作空间提供必要的构建依赖，也就是要 source 现有 ROS 2 的环境。
这可以通过获取二进制安装或源码安装提供的设置脚本来实现，也就是另一个 colcon 工作空间（参见 :doc:`安装 <../../Installation>`）。
我们称这个环境为 **underlay**。

我们现在这个工作空间，``ros2_ws``，将会是现有 ROS 2 安装的 **overlay**。
在一般情况下，建议在迭代少量包时使用 overlay，而不是将所有包放入同一个工作空间。

构建 the workspace
^^^^^^^^^^^^^^^^^^^

.. attention::

   如果要在 Windows 上构建包，你需要在 Visual Studio 环境中，参见 :ref:`在 Windows 上构建 ROS 2 代码 <windows-dev-build-ros2>` 获取更多细节。

在工作空间的根目录下，运行 ``colcon build``。
由于 ``ament_cmake`` 等构建类型不支持 ``devel`` 空间的概念，所以需要包被安装到安装目录，不过colcon 支持 ``--symlink-install`` 选项。
这个选项可以通过更改 ``source`` 空间中的文件（例如 Python 文件或其他非编译资源）来更改已安装的文件，以加快迭代速度。（译者注：也就是说在有这个选项的情况下，对非编译资源的源码的修改会直接反映在安装的程序上，而不需要先构建。）

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      colcon build --symlink-install

  .. group-tab:: macOS

    .. code-block:: console

      colcon build --symlink-install

  .. group-tab:: Windows

    .. code-block:: console

      colcon build --symlink-install --merge-install

    Windows 不允许长路径，所以 ``merge-install`` 将所有路径合并到 ``install`` 目录中。

构建完成后，我们应该看到 ``build``、 ``install`` 和 ``log`` 目录:

.. code-block:: bash

    .
    ├── build
    ├── install
    ├── log
    └── src

    4 directories, 0 files

.. _colcon-run-the-tests:

运行测试
^^^^^^^^^

运行我们刚刚构建的包的测试:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      colcon test

  .. group-tab:: macOS

    .. code-block:: console

      colcon test

  .. group-tab:: Windows

    Remember to use a ``x64 Native Tools Command Prompt for VS 2019`` for executing the following command, as we are going to build a workspace.

    .. code-block:: console

      colcon test --merge-install

    You also need to specify ``--merge-install`` here since we used it for building above.

.. _colcon-tutorial-source-the-environment:

Source 当前环境
^^^^^^^^^^^^^^^^^^^^^^

当 colcon 成功构建后，输出将在 ``install`` 目录中。
你需要先将它们添加到你的系统路径中才能使用已安装的可执行文件或库。
colcon 会在 ``install`` 目录中生成 bash/bat 文件来帮助设置环境。
这些文件将添加所有必需的元素到你的路径和库路径中，同时提供由包导出的 bash 或 shell 命令。

.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

       source install/setup.bash

  .. group-tab:: macOS

    .. code-block:: bash

       . install/setup.bash

  .. group-tab:: Windows

    .. code-block:: bash

       call install\setup.bat

    Or with Powershell:

    .. code-block:: bash

       install\setup.ps1

尝试 demo
^^^^^^^^^^

有了环境后，我们可以运行 colcon 构建的可执行文件。
让我们从示例中运行一个订阅者节点:

.. code-block:: bash

    ros2 run examples_rclcpp_minimal_subscriber subscriber_member_function

在另一个终端中，让我们运行一个发布者节点（不要忘记 source 环境脚本）:

.. code-block:: bash

    ros2 run examples_rclcpp_minimal_publisher publisher_member_function

你应该看到发布者和订阅者的消息，数字递增。

创建你自己的包
-----------------------

colcon 使用 `REP 149 <https://www.ros.org/reps/rep-0149.html>`__ 中定义的 ``package.xml`` 规范（也支持 `format 2 <https://www.ros.org/reps/rep-0140.html>`__）。

colcon 支持多种构建类型。
推荐的构建类型是 ``ament_cmake`` 和 ``ament_python``。
也支持纯 ``cmake`` 包。

``ament_python`` 构建的一个例子是 `ament_index_python 包 <https://github.com/ament/ament_index/tree/{REPOS_FILE_BRANCH}/ament_index_python>`__ , 其中 setup.py 是构建好的程序的入口（entry point）.

``ament_cmake`` 构建的一个例子是 `demo_nodes_cpp <https://github.com/ros2/demos/tree/{REPOS_FILE_BRANCH}/demo_nodes_cpp>`__ , 它使用 CMake 作为构建工具.

为了方便，你可以使用 ``ros2 pkg create`` 工具基于模板创建一个新包。

.. note:: 这相当于 ``catkin`` 用户知道的 ``catkin_create_package``。

配置 ``colcon_cd``
-------------------

``colcon_cd`` 命令允许你快速将 shell 的当前工作目录切换为包的目录。
例如 ``colcon_cd some_ros_package`` 将快速将你带到 ``~/ros2_ws/src/some_ros_package`` 目录。

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
        echo "export _colcon_cd_root=/opt/ros/{DISTRO}/" >> ~/.bashrc

   .. group-tab:: macOS

      .. code-block:: console

        echo "source /usr/local/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
        echo "export _colcon_cd_root=~/ros2_install" >> ~/.bashrc

   .. group-tab:: Windows

      Not yet available

根据你如何安装 ``colcon_cd`` 和你的工作空间的位置，上面的指令可能会有所不同，请参考 `文档 <https://colcon.readthedocs.io/en/released/user/installation.html#quick-directory-changes>`__ 获取更多细节。
如果你不想使用 ``colcon_cd`` 了，可以在 Linux 和 macOS 中找到系统的 shell 启动脚本并删除附加的 source 和 export 命令。

配置 ``colcon`` tab 键自动补全
-------------------------------

``colcon`` 命令支持 bash 和类似 bash 的 shell 的命令自动补全。
必须安装 ``colcon-argcomplete`` 包， 可能需要 `一些设置 <https://colcon.readthedocs.io/en/released/user/installation.html#enable-completion>`__ 才能使其工作。

Tips
----

* 如果你不想构建特定包，请在目录中放置一个名为 ``COLCON_IGNORE`` 的空文件，这个目录将不会被构建工具索引到。

* 如果你想避免在 CMake 包中配置和构建测试，可以传递: ``--cmake-args -DBUILD_TESTING=0``。

* 如果你想运行某个包中的单个测试：

  .. code-block:: bash

     colcon test --packages-select YOUR_PKG_NAME --ctest-args -R YOUR_TEST_IN_PKG
