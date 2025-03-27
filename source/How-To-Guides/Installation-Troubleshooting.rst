.. redirect-from::

  Guides/Installation-Troubleshooting
  Troubleshooting/Installation-Troubleshooting

安装问题排查
============================

安装过程中的问题排查技巧按照适用的平台进行了分类。

.. contents:: Platforms
   :depth: 1
   :local:

共性问题
----------

共性问题的排查技巧适用于所有平台。

使能 multicast
^^^^^^^^^^^^^^^^

为了能够通过 DDS 通信，使用的网络接口必须已经使能/启用了 multicast。
在之前的经验中，当使用环回适配器(loopback adapter)时这功能可能不会默认启用（在 Ubuntu 或 OSX 上）。
可以查看 `原始问题 <https://github.com/ros2/ros2/issues/552>`__ 或者 `ros-answers 上的讨论 <https://answers.ros.org/question/300370/ros2-talker-cannot-communicate-with-listener/>`__.
可以使用 ROS 2 工具来验证当前的设置是否允许 multicast：

在终端 1 中：

.. code-block:: bash

   ros2 multicast receive

在终端 2 中：

.. code-block:: bash

   ros2 multicast send

如果第一个命令没有返回类似这样的信息：

.. code-block:: bash

   Received from xx.xxx.xxx.xx:43751: 'Hello World!'

那你就需要更新防火墙配置来允许 multicast， 使用 `ufw <https://help.ubuntu.com/community/UFW>`__。

.. code-block:: bash

   sudo ufw allow in proto udp to 224.0.0.0/4
   sudo ufw allow in proto udp from 224.0.0.0/4

你可以使用 :code:`ifconfig` 工具来检查网络接口是否已经使能了 multicast 标志，查看 flags 部分是否包含 :code:`MULTICAST`：

.. code-block:: bash

   eno1: flags=4163<...,MULTICAST>
      ...

Import 时找不到库
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

有时候 ``rclpy`` 无法被导入是因为依赖的 C 扩展库没有找到。
如果是这样，可以查看错误信息中提到的库所在的目录，然后查看这个目录中是否有类似名称的文件。
如果有个用不同版本 Python 构建的 C 拓展库（文件名前缀类似 ``_rclpy.``，后缀类似 ``.so``，但是 Python 版本/架构不同），
那么请确保运行时使用的 Python 与构建时使用的 Python 版本相同。

这样的问题可能会在升级操作系统后出现，重新构建工作空间一般就能解决这个问题。

.. _linux-troubleshooting:

Linux
-----

内部编译错误
^^^^^^^^^^^^^^^^^^^^^^^

如果在内存受限的平台上编译（比如 Raspberry PI）时遇到内部编译错误，你可能需要单线程编译（在构建命令前加上 ``MAKEFLAGS=-j1``）。

内存不足
^^^^^^^^^^^^^

``ros1_bridge`` 在当前形式下需要 4Gb 的空闲内存来编译。
如果你没有足够的内存，可以在那个文件夹中使用 ``COLCON_IGNORE`` 来跳过它的编译。

多机干扰
^^^^^^^^^^^^^^^^^^^^^^^^^^

如果在同一网络上运行多个 ROS 实例，可能会出现干扰。
为了避免这种情况，可以设置环境变量 ``ROS_DOMAIN_ID`` 为不同的整数，默认值是零。
这可以为你的系统定义 DDS domain ID。

source setup.bash 时出现异常
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. only relevant to Eloquent and Foxy

如果从源码构建后 source 环境时遇到异常，尝试使用以下命令升级 ``colcon`` 相关的包：

.. code-block:: bash

   colcon version-check  # 检查是否有可用的更新
   sudo apt install python3-colcon* --only-upgrade  # 升级 colcon 至最新版本

<<<<<<< HEAD
Anaconda Python 冲突
^^^^^^^^^^^^^^^^^^^^^^^^

``conda`` 与 ROS 2 不兼容。
确保你的 ``PATH`` 环境变量中没有任何 conda 路径。
你可能需要检查你的 ``.bashrc`` 文件，注释掉“添加 conda 路径”的那一行。
=======
Mixing conda and apt Python Conflict
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

While using ROS 2, mixing packages installed with ``apt`` with packages installed with ``conda`` does not work.
If you are using the official ``apt`` binaries for ROS 2, make sure that your ``PATH`` environment variable does not have any conda paths in it.
You may have to check your ``.bashrc`` for this line and comment it out.
>>>>>>> upstream/humble

On the other hand on Windows, the official ROS 2 installation procedure uses ``conda`` packages via the ``pixi`` package manager, and that works fine as there is no mix of different package managers

``conda`` packages for ROS 2 may be built (such as the one provided by the community-mantained [RoboStack](https://robostack.github.io/) project) but no official conda packages for ROS 2 are provided.

.. _macOS-troubleshooting:

macOS
-----

Segmentation fault when using ``pyenv``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``pyenv`` seems to default to building Python with ``.a`` files, but that causes issues with ``rclpy``, so it's recommended to build Python with Frameworks enabled on macOS when using ``pyenv``:

https://github.com/pyenv/pyenv/wiki#how-to-build-cpython-with-framework-support-on-os-x

Library not loaded; image not found
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you are seeing library loading issues at runtime (either running tests or running nodes), such as the following:

.. code-block:: bash

   ImportError: dlopen(.../ros2_<distro>/ros2-osx/lib/python3.7/site-packages/rclpy/_rclpy.cpython-37m-darwin.so, 2): Library not loaded: @rpath/librcl_interfaces__rosidl_typesupport_c.dylib
     Referenced from: .../ros2_<distro>/ros2-osx/lib/python3.7/site-packages/rclpy/_rclpy.cpython-37m-darwin.so
     Reason: image not found

Then you probably have System Integrity Protection enabled.
Follow `these instructions <https://developer.apple.com/library/content/documentation/Security/Conceptual/System_Integrity_Protection_Guide/ConfiguringSystemIntegrityProtection/ConfiguringSystemIntegrityProtection.html>`__ to disable System Integrity Protection (SIP).

Qt build error: ``unknown type name 'Q_ENUM'``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you see build errors related to Qt, e.g.:

.. code-block:: bash

   In file included from /usr/local/opt/qt/lib/QtGui.framework/Headers/qguiapplication.h:46:
   /usr/local/opt/qt/lib/QtGui.framework/Headers/qinputmethod.h:87:5: error:
         unknown type name 'Q_ENUM'
       Q_ENUM(Action)
       ^

you may be using qt4 instead of qt5: see https://github.com/ros2/ros2/issues/441

Missing symbol when opencv (and therefore libjpeg, libtiff, and libpng) are installed with Homebrew
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you have opencv installed you might get this:

.. code-block:: bash

   dyld: Symbol not found: __cg_jpeg_resync_to_restart
     Referenced from: /System/Library/Frameworks/ImageIO.framework/Versions/A/ImageIO
     Expected in: /usr/local/lib/libJPEG.dylib
    in /System/Library/Frameworks/ImageIO.framework/Versions/A/ImageIO
   /bin/sh: line 1: 25274 Trace/BPT trap: 5       /usr/local/bin/cmake

If so, to build you'll have to do this:

.. code-block:: bash

   $ brew unlink libpng libtiff libjpeg

But this will break opencv, so you'll also need to update it to continue working:

.. code-block:: bash

   $ sudo install_name_tool -change /usr/local/lib/libjpeg.8.dylib /usr/local/opt/jpeg/lib/libjpeg.8.dylib /usr/local/lib/libopencv_highgui.2.4.dylib
   $ sudo install_name_tool -change /usr/local/lib/libpng16.16.dylib /usr/local/opt/libpng/lib/libpng16.16.dylib /usr/local/lib/libopencv_highgui.2.4.dylib
   $ sudo install_name_tool -change /usr/local/lib/libtiff.5.dylib /usr/local/opt/libtiff/lib/libtiff.5.dylib /usr/local/lib/libopencv_highgui.2.4.dylib
   $ sudo install_name_tool -change /usr/local/lib/libjpeg.8.dylib /usr/local/opt/jpeg/lib/libjpeg.8.dylib /usr/local/Cellar/libtiff/4.0.4/lib/libtiff.5.dylib

The first command is necessary to avoid things built against the system libjpeg (etc.) from getting the version in /usr/local/lib.
The others are updating things built by Homebrew so that they can find the version of libjpeg (etc.) without having them in /usr/local/lib.

Xcode-select error: tool ``xcodebuild`` requires Xcode, but active developer directory is a command line instance
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. only relevant to Eloquent and Foxy

If you recently installed Xcode, you may encounter this error:

.. code-block:: bash

   Xcode: xcode-select: error: tool 'xcodebuild' requires Xcode,
   but active developer directory '/Library/Developer/CommandLineTools' is a command line tools instance

To resolve this error, you will need to:

1. Double check that you have the command line tool installed:

.. code-block:: bash

   $ xcode-select --install

2. Accept the terms and conditions of Xcode by typing in terminal:

.. code-block:: bash

   $ sudo xcodebuild -license accept

3. Ensure Xcode app is in the ``/Applications`` directory (NOT ``/Users/{user}/Applications``)

4. Point ``xcode-select`` to the Xcode app Developer directory using the following command:

.. code-block:: bash

   $ sudo xcode-select -s /Applications/Xcode.app/Contents/Developer

rosdep install error ``homebrew: Failed to detect successful installation of [qt5]``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
While following the :doc:`Creating a workspace <../Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace>` tutorial, you might encounter the following error stating that ``rosdep`` fails to install Qt5.

.. code-block:: bash

   $ rosdep install -i --from-path src --rosdistro {DISTRO} -y
   executing command [brew install qt5]
   Warning: qt 5.15.0 is already installed and up-to-date
   To reinstall 5.15.0, run `brew reinstall qt`
   ERROR: the following rosdeps failed to install
     homebrew: Failed to detect successful installation of [qt5]

This error seems to stem from a `linking issue <https://github.com/ros-infrastructure/rosdep/issues/490#issuecomment-334959426>`__ and can be resolved by running the following command.

.. code-block:: bash

   $ cd /usr/local/Cellar
   $ sudo ln -s qt qt5

Running the ``rosdep`` command should now execute normally:

.. code-block:: bash

   $ rosdep install -i --from-path src --rosdistro {DISTRO} -y
   #All required rosdeps installed successfully


.. _windows-troubleshooting:

Windows
-------

Import failing even with library present on the system
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Sometimes ``rclpy`` fails to be imported because of some missing DLLs on your system.
If so, make sure to install all the dependencies listed in the "Installing prerequisites" sections of the :ref:`installation instructions <windows-install-binary-installing-prerequisites>`).

If you are installing from binaries, you may need to update your dependencies: they must be the same version as those used to build the binaries.

If you are still having issues, you can use the `Dependencies <https://github.com/lucasg/Dependencies>`_ tool to determine which dependencies are missing on your system.
Use the tool to load the corresponding ``.pyd`` file, and it should report unavailable ``DLL`` modules.
Be sure that the current workspace is sourced before you execute the tool, otherwise there will be unresolved ROS DLL files.
Use this information to install additional dependencies or adjust your path as necessary.

CMake error setting modification time
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you run into the CMake error ``file INSTALL cannot set modification time on ...`` when installing files it is likely that an anti virus software or Windows Defender are interfering with the build.
E.g. for Windows Defender you can list the workspace location to be excluded to prevent it from scanning those files.

260 character path limit
^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   The input line is too long.
   The syntax of the command is incorrect.

Depending on your directory hierarchy, you may see path length limit errors when building ROS 2 from source or your own libraries.

To allow deeper path lengths:

Run ``regedit.exe``, navigate to ``Computer\HKEY_LOCAL_MACHINE\SYSTEM\CurrentControlSet\Control\FileSystem``, and set ``LongPathsEnabled`` to 0x00000001 (1).

Hit the windows key and type ``Edit Group Policy``.
Navigate to Local Computer Policy > Computer Configuration > Administrative Templates > System > Filesystem.
Right click ``Enable Win32 long paths``, click Edit.
In the dialog, select Enabled and click OK.

Close and open your terminal to reset the environment and try building again.

CMake packages unable to find asio, tinyxml2, tinyxml, or eigen
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

We've seen that sometimes the chocolatey packages for ``asio``, ``tinyxml2``, etc. do not add important registry entries and CMake will be unable to find them when building ROS 2.
We've not yet been able to identify the root cause, but uninstalling the chocolatey packages (with ``-n`` if the uninstall fails the first time), and then reinstalling them will fix the issue.

patch.exe opens a new command window and asks for administrator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This will also cause the build of packages which need to use patch to fail, even you allow it to use administrator rights.

- ``choco uninstall patch; colcon build --cmake-clean-cache`` - This is a bug in the `GNU Patch For Windows package <https://chocolatey.org/packages/patch>`_.
  If this package is not installed, the build process will instead use the version of Patch distributed with git.

Failed to load Fast RTPS shared library
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. does not apply to Crystal

Fast RTPS requires ``msvcr20.dll``, which is part of the ``Visual C++ Redistributable Packages for Visual Studio 2013``.
Although it is usually installed by default in Windows 10, we know that some Windows 10-like versions don't have it installed by default (e.g.: Windows Server 2019).
In case you don't have it installed, you can download it from `here <https://www.microsoft.com/en-us/download/details.aspx?id=40784>`_.

Failed to create process
^^^^^^^^^^^^^^^^^^^^^^^^

If running a ROS binary gives the error:

.. code-block::

   | failed to create process.

It is likely the Python interpreter was not found.
For each executable, the shebang (first line) of the accompanying script is used, so make sure Python is available under the expected path (default: ``C:\Python38\``).

Binary installation specific
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* If your example does not start because of missing DLLs, please verify that all libraries from external dependencies such as OpenCV are located inside your ``PATH`` variable.
* If you forget to call the ``local_setup.bat`` file from your terminal, the demo programs will most likely crash immediately.

Running RViz with WSL2
^^^^^^^^^^^^^^^^^^^^^^

If you are using `WSL2 <https://learn.microsoft.com/en-us/windows/wsl/install>`__ to run ROS 2 on Windows, you may run into an issue running RViz that looks like:

.. code-block:: console

   $ rviz2
   [INFO] [1695823660.091830699] [rviz2]: Stereo is NOT SUPPORTED
   [INFO] [1695823660.091943524] [rviz2]: OpenGl version: 4.1 (GLSL 4.1)
   D3D12: Removing Device.
   Segmentation fault

One possible solution to this is to force RViz to use software rendering:

.. code-block:: console

   $ export LIBGL_ALWAYS_SOFTWARE=true
   $ rviz2
   [INFO] [1695823660.091830699] [rviz2]: Stereo is NOT SUPPORTED
