.. redirect-from::

    Tutorials/Configuring-ROS2-Environment

.. _ConfigROS2:

配置环境
=======================

**目标:** 本教程将向您展示如何配置 ROS 2 环境。

**教程级别:** 初级

**预计时长:** 5 分钟

.. contents:: 目录
   :depth: 2
   :local:

背景
----------

ROS 2 依赖于使用 shell 环境来组合工作空间的概念。
"工作空间"是一个用于指代你在系统上开发 ROS 2 的位置的 ROS 术语。
ROS 2 的核心工作空间称为底层(underlay)工作空间。
后续的本地工作空间称为上层(overlays)工作空间。
当使用 ROS 2 进行开发时，通常会同时激活几个工作空间。

组合工作空间使得开发不同版本的 ROS 2 或不同的项目更容易。
它还允许在同一台设备上安装多个不同版本的 ROS 2（或“distros”，例如 Dashing 和 Eloquent）并在它们之间切换。

要想激活工作空间，你可以在每次打开一个新的 shell 时都 source 一下配置脚本；或者把 这条 source 指令添加到 shell 的启动脚本中，这样每次你打开一个新的 shell 都会默认运行一次你添加的 source 指令。
如果不 source 配置脚本，你就没办法直接使用 ROS 2 命令，或者直接找到、使用 ROS 2 包。换句话说，你就用不了 ROS 2.

前提条件
-------------

在开始学习本教程之前，请确保你已经完成了 :doc:`../../Installation` 中的步骤。

本教程中使用的命令是假定你是从二进制安装的 ROS 2。如果你是从源码构建并安装的，那么你的配置脚本的路径会有些不一样，需要你自己在使用时注意。
当然，如果是从源码安装的，你也没办法使用诸如 ``sudo apt install ros-<distro>-<package>`` 的命令(在初级教程中最常用的命令)。

如果你现在使用的设备是 Linux 或者 macOS 的系统，但是你不太熟悉 shell，可以先查看 `这个关于 shell 的教程 <https://www.linux.com/training-tutorials/bash-101-working-cli/>`__ 。

任务
-----

1 Source 配置脚本
^^^^^^^^^^^^^^^^^^^^^^^^

每次你打开一个新 shell 想使用 ROS 2 的指令时，都需要运行一下这个指令：

.. tabs::

   .. group-tab:: Linux

      .. code-block:: bash

        # 如果你用的不是 bash， 记得把 ".bash" 替换成你的 shell类型
        # 常见的比如: setup.bash, setup.sh, setup.zsh
        source /opt/ros/{DISTRO}/setup.bash

   .. group-tab:: macOS

      .. code-block:: console

        . ~/ros2_install/ros2-osx/setup.bash

   .. group-tab:: Windows

      .. code-block:: console

        call C:\dev\ros2\local_setup.bat

.. note::
    当然这条指令具体是不是完全和列举的一样，取决于你是怎么安装 ROS 2 的。
    如果你在运行的时候遇到些问题，请先检查上述指令中的脚本真的存在，有可能是你的安装路径和上述指令中不一样。

2 把 source 指令添加到你的 shell 启动脚本中
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

如果你不想像第一步一样每次打开新 shell 都要 source 配置脚本，你可以用下面这个指令把那条 source 指令添加到你的 shell 启动脚本中:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        echo "source /opt/ros/{DISTRO}/setup.bash" >> ~/.bashrc

     如果不想要这个功能生效了，你需要自己把刚刚添加的这条指令从系统的 shell 启动脚本（译者注： 比如此处你就需要用任意的编辑器打开 ``~/.bashrc`` 文件，删掉其中的 ``source /opt/ros/{DISTRO}/setup.bash`` ）中删掉。

   .. group-tab:: macOS

      .. code-block:: console

        echo "source ~/ros2_install/ros2-osx/setup.bash" >> ~/.bash_profile

      如果不想要这个功能生效了，你需要自己把刚刚添加的这条指令从系统的 shell 启动脚本中删掉。

   .. group-tab:: Windows

      Only for PowerShell users, create a folder in 'My Documents' called 'WindowsPowerShell'.
      Within 'WindowsPowerShell', create file 'Microsoft.PowerShell_profile.ps1'.
      Inside the file, paste:

      .. code-block:: console

        C:\dev\ros2_{DISTRO}\local_setup.ps1

      PowerShell will request permission to run this script everytime a new shell is opened.
      To avoid that issue you can run:

      .. code-block:: console

        Unblock-File C:\dev\ros2_{DISTRO}\local_setup.ps1

      To undo this, remove the new 'Microsoft.PowerShell_profile.ps1' file.

3 检查环境变量
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

source ROS 2 配置脚本这个操作会添加一些必要的系统变量。
如果你在后续使用 ROS 2 命令时遇到问题，可以先用下面的指令检查一下这些系统变量是不是正常的：

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        printenv | grep -i ROS

   .. group-tab:: macOS

      .. code-block:: console

        printenv | grep -i ROS

   .. group-tab:: Windows

      .. code-block:: console

        set | findstr -i ROS

检查如下变量，例如 ``ROS_DISTRO`` 和 ``ROS_VERSION`` 的变量已经配置好.

::

  ROS_VERSION=2
  ROS_PYTHON_VERSION=3
  ROS_DISTRO={DISTRO}

如果环境变量配置不正常，那就重新按照安装教程再细致地操作一次。
如果你需要一些更具体或者更特殊的帮助，可以 `从社区寻找帮助 <https://robotics.stackexchange.com/>`__ 。

3.1 ``ROS_DOMAIN_ID``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

查看 `domain ID <../../Concepts/Intermediate/About-Domain-ID>` 一文了解更多细节.

使用 ``ROS_DOMAIN_ID`` 环境变量可以为你的 ROS 2 节点组分配一个唯一的整数。

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        export ROS_DOMAIN_ID=<your_domain_id>

      To maintain this setting between shell sessions, you can add the command to your shell startup script:

      .. code-block:: console

        echo "export ROS_DOMAIN_ID=<your_domain_id>" >> ~/.bashrc

   .. group-tab:: macOS

      .. code-block:: console

        export ROS_DOMAIN_ID=<your_domain_id>

      To maintain this setting between shell sessions, you can add the command to your shell startup script:

      .. code-block:: console

        echo "export ROS_DOMAIN_ID=<your_domain_id>" >> ~/.bash_profile

   .. group-tab:: Windows

      .. code-block:: console

        set ROS_DOMAIN_ID=<your_domain_id>

      If you want to make this permanent between shell sessions, also run:

      .. code-block:: console

        setx ROS_DOMAIN_ID <your_domain_id>

3.2 ``ROS_LOCALHOST_ONLY``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

默认情况下，ROS 2 通信不限于 localhost。
``ROS_LOCALHOST_ONLY`` 环境变量可以让你限制 ROS 2 通信只在 localhost 上进行。
这意味着你的 ROS 2 系统和它的 topics、services 和 actions 将无法被本地网络的其他设备访问到。
``ROS_LOCALHOST_ONLY`` 在某些情况下很有用，比如在教室里，多个机器人可能会发布到同一个 topic，导致产生一些奇怪的现象。
你可以使用如下命令配置这个环境变量：

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        export ROS_LOCALHOST_ONLY=1

      To maintain this setting between shell sessions, you can add the command to your shell startup script:

      .. code-block:: console

        echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc

   .. group-tab:: macOS

      .. code-block:: console

        export ROS_LOCALHOST_ONLY=1

      To maintain this setting between shell sessions, you can add the command to your shell startup script:

      .. code-block:: console

        echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bash_profile

   .. group-tab:: Windows

      .. code-block:: console

        set ROS_LOCALHOST_ONLY=1

      If you want to make this permanent between shell sessions, also run:

      .. code-block:: console

        setx ROS_LOCALHOST_ONLY 1


总结
-------

ROS 2 开发环境需要在使用之前正确配置好。
有两种方法可以配置：一种是在每个新 shell 中都运行一次配置脚本，另一种是把 source 指令添加到你的启动脚本中。

如果你在使用 ROS 2 时遇到任何问题，首先检查你的环境变量是否设置正确，确保变量已经配置到你想要的值。

下一步
----------

现在你已经有一个可以正常工作的 ROS 2 环境，并且知道如何 source 配置脚本，你可以继续学习 ROS 2 的各种细节，比如 :doc:`turtlesim 工具 <./Introducing-Turtlesim/Introducing-Turtlesim>`。
