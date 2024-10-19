.. redirect-from::

    Tutorials/Creating-Your-First-ROS2-Package

.. _CreatePkg:

创建 ROS 2 包
==================

**目标:** 用 CMake 或者 Python 创建一个新的包，然后运行对应的可执行文件.

**教程等级:** 初级

**预计时长:** 15 分钟

.. contents:: Contents
   :depth: 2
   :local:

背景
----------

1 什么是 ROS 2 包?
^^^^^^^^^^^^^^^^^^^^^^^^^^

包是 ROS 2 代码的组织单元.
如果你想安装你的代码或者与他人分享，那么你需要将它组织成一个包.
用包可以发布你的 ROS 2 工作，让其他人更容易构建和使用它.

ROS 2 中的包创建使用 ament 作为构建系统(build system)，colcon 作为构建工具(build tool).
你可以使用 CMake 或者 Python 创建一个包，这两种方法都是官方支持的，当然也有其他构建类型.

2 ROS 2 包由哪些内容构成?
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

ROS 2 Python 和 CMake 包各自有自己的最低要求内容:

.. tabs::

   .. group-tab:: CMake

      * ``CMakeLists.txt`` 描述如何构建包内代码
      * ``include/<package_name>`` 目录包含包的公共头文件
      * ``package.xml`` 包含关于包的元信息
      * ``src`` 目录包含包的源代码

   .. group-tab:: Python

      * ``package.xml`` 包含关于包的元信息
      * ``resource/<package_name>`` 包的标记文件
      * ``setup.cfg`` 当包有可执行文件时是必需的，这样 ``ros2 run`` 才能找到它们
      * ``setup.py`` 包含如何安装包的指令
      * ``<package_name>`` 是一个与包名相同的目录，ROS 2 工具用这个名字找到你的包的源码，需要含有 ``__init__.py`` 文件

最简可运行的包应该有如下的文件结构：

.. tabs::

   .. group-tab:: CMake

      .. code-block:: console

        my_package/
             CMakeLists.txt
             include/my_package/
             package.xml
             src/

   .. group-tab:: Python

      .. code-block:: console

        my_package/
              package.xml
              resource/my_package
              setup.cfg
              setup.py
              my_package/


3 工作空间中的包
^^^^^^^^^^^^^^^^^^^^^^^^^

每个工作空间可以包含任意多的包，每个包都在自己的文件夹中.
一个工作空间中可以有不同构建类型的包（CMake, Python 等）.
但是一个包不能包含另一个包，也就是说包不能嵌套.

最佳实践：在工作空间中创建一个 ``src`` 文件夹，然后在里面放你的包.
这样可以保持工作空间顶层的“干净条理”.

一个简单的工作空间看起来是这样的：

.. code-block:: console

  workspace_folder/
      src/
        cpp_package_1/
            CMakeLists.txt
            include/cpp_package_1/
            package.xml
            src/

        py_package_1/
            package.xml
            resource/py_package_1
            setup.cfg
            setup.py
            py_package_1/
        ...
        cpp_package_n/
            CMakeLists.txt
            include/cpp_package_n/
            package.xml
            src/


前提条件
-------------

在 :doc:`上一个教程 <./Creating-A-Workspace/Creating-A-Workspace>` 中按照指示操作，你应该已经有了 ROS 2 工作空间.
现在你可以在这个工作空间中创建你的包.


任务
-----

1 创建包
^^^^^^^^^^^^^^^^^^

首先，:doc:`source 你的 ROS 2 安装环境 <../Beginner-CLI-Tools/Configuring-ROS2-Environment>`.

让我们在 :ref:`上一个教程 <new-directory>` 中创建的工作空间 ``ros2_ws`` 里创建新包.

确保你在运行包创建的命令之前已经在 ``src`` 文件夹里了.

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        cd ~/ros2_ws/src

   .. group-tab:: macOS

     .. code-block:: console

       cd ~/ros2_ws/src

   .. group-tab:: Windows

     .. code-block:: console

       cd \ros2_ws\src

下面这个命令能创建一个新的包:

.. tabs::

   .. group-tab:: CMake

      .. code-block:: console

        ros2 pkg create --build-type ament_cmake --license Apache-2.0 <package_name>

   .. group-tab:: Python

      .. code-block:: console

        ros2 pkg create --build-type ament_python --license Apache-2.0 <package_name>

在这个教程中，你将学习使用可选参数 ``--node-name`` 来创建一个简单的 Hello World 式的可执行文件.

在终端中输入以下命令:

.. tabs::

   .. group-tab:: CMake

      .. code-block:: console

        ros2 pkg create --build-type ament_cmake --license Apache-2.0 --node-name my_node my_package

   .. group-tab:: Python

      .. code-block:: console

        ros2 pkg create --build-type ament_python --license Apache-2.0 --node-name my_node my_package

这个命令会在你的工作空间的 ``src`` 目录下创建一个新文件夹，名为 ``my_package``.

运行完命令后，终端会返回以下信息:

.. tabs::

   .. group-tab:: CMake

      .. code-block:: console

        going to create a new package
        package name: my_package
        destination directory: /home/user/ros2_ws/src
        package format: 3
        version: 0.0.0
        description: TODO: Package description
        maintainer: ['<name> <email>']
        licenses: ['TODO: License declaration']
        build type: ament_cmake
        dependencies: []
        node_name: my_node
        creating folder ./my_package
        creating ./my_package/package.xml
        creating source and include folder
        creating folder ./my_package/src
        creating folder ./my_package/include/my_package
        creating ./my_package/CMakeLists.txt
        creating ./my_package/src/my_node.cpp

   .. group-tab:: Python

      .. code-block:: console

        going to create a new package
        package name: my_package
        destination directory: /home/user/ros2_ws/src
        package format: 3
        version: 0.0.0
        description: TODO: Package description
        maintainer: ['<name> <email>']
        licenses: ['TODO: License declaration']
        build type: ament_python
        dependencies: []
        node_name: my_node
        creating folder ./my_package
        creating ./my_package/package.xml
        creating source folder
        creating folder ./my_package/my_package
        creating ./my_package/setup.py
        creating ./my_package/setup.cfg
        creating folder ./my_package/resource
        creating ./my_package/resource/my_package
        creating ./my_package/my_package/__init__.py
        creating folder ./my_package/test
        creating ./my_package/test/test_copyright.py
        creating ./my_package/test/test_flake8.py
        creating ./my_package/test/test_pep257.py
        creating ./my_package/my_package/my_node.py

可以看到为新创建的包自动生成的文件.

2 构建包
^^^^^^^^^^^^^^^^^

把包都放在工作空间中就可以在工作空间根目录下运行 ``colcon build`` 来一次性构建所有包.
不然你还得单独构建每个包。

回到工作空间的根目录:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        cd ~/ros2_ws

   .. group-tab:: macOS

      .. code-block:: console

        cd ~/ros2_ws

   .. group-tab:: Windows

     .. code-block:: console

       cd \ros2_ws

现在构建包:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      colcon build

  .. group-tab:: macOS

    .. code-block:: console

      colcon build

  .. group-tab:: Windows

    .. code-block:: console

      colcon build --merge-install

    Windows doesn't allow long paths, so ``merge-install`` will combine all the paths into the ``install`` directory.

还记得上一个教程中你的 ``ros2_ws`` 中也放了 ``ros_tutorials`` 包吧.
你可能已经注意到运行 ``colcon build`` 时也构建了 ``turtlesim`` 包.
这在你的工作空间中只有几个包时没什么问题，但是包多了的话， ``colcon build`` 就会消耗很长时间.

下次如果只想构建 ``my_package`` 包，可以运行:

.. code-block:: console

    colcon build --packages-select my_package

3 Source 配置文件
^^^^^^^^^^^^^^^^^^^^^^^
想要运行新包里的可执行文件，首先要 source 你的 ROS 2 安装环境.

然后，在 ``ros2_ws`` 目录内运行以下命令来 source 你的工作空间:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      source install/local_setup.bash

  .. group-tab:: macOS

    .. code-block:: console

      . install/local_setup.bash

  .. group-tab:: Windows

    .. code-block:: console

      call install/local_setup.bat

现在你的工作空间已经被添加到环境变量中，就可以使用新包的可执行文件了.

4 使用包
^^^^^^^^^^^^^^^^^

用以下指令运行在创建包的时候用 ``--node-name`` 指定生成的可执行文件:

.. code-block:: console

  ros2 run my_package my_node

终端会返回以下信息:

.. tabs::

   .. group-tab:: CMake

      .. code-block:: console

        hello world my_package package

   .. group-tab:: Python

      .. code-block:: console

        Hi from my_package.

5 检查包的内容
^^^^^^^^^^^^^^^^^^^^^^^^^^

在 ``ros2_ws/src/my_package`` 目录下可以看到 ``ros2 pkg create`` 自动创建了一些文件和文件夹:

.. tabs::

   .. group-tab:: CMake

      .. code-block:: console

        CMakeLists.txt  include  package.xml  src

      ``my_node.cpp`` is inside the ``src`` directory.
      This is where all your custom C++ nodes will go in the future.

   .. group-tab:: Python

      .. code-block:: console

        my_package  package.xml  resource  setup.cfg  setup.py  test

      ``my_node.py`` is inside the ``my_package`` directory.
      This is where all your custom Python nodes will go in the future.

6 自定义 package.xml
^^^^^^^^^^^^^^^^^^^^^^^

你应该已经注意到在创建包后返回的信息中 ``description`` 和 ``license`` 字段包含了一些 ``TODO`` 标记.
这是因为 ``package.xml`` 中的包描述和许可声明不是自动生成的，但是如果你想发布你的包，这两个字段则是是必需的.
``maintainer`` 字段也得根据需要填写.

从 ``ros2_ws/src/my_package`` 目录下用你喜欢的文本编辑器打开 ``package.xml`` 文件:

.. tabs::

   .. group-tab:: CMake

    .. code-block:: xml

     <?xml version="1.0"?>
     <?xml-model
        href="http://download.ros.org/schema/package_format3.xsd"
        schematypens="http://www.w3.org/2001/XMLSchema"?>
     <package format="3">
      <name>my_package</name>
      <version>0.0.0</version>
      <description>TODO: Package description</description>
      <maintainer email="user@todo.todo">user</maintainer>
      <license>TODO: License declaration</license>

      <buildtool_depend>ament_cmake</buildtool_depend>

      <test_depend>ament_lint_auto</test_depend>
      <test_depend>ament_lint_common</test_depend>

      <export>
        <build_type>ament_cmake</build_type>
      </export>
     </package>

   .. group-tab:: Python

    .. code-block:: xml

     <?xml version="1.0"?>
     <?xml-model
        href="http://download.ros.org/schema/package_format3.xsd"
        schematypens="http://www.w3.org/2001/XMLSchema"?>
     <package format="3">
      <name>my_package</name>
      <version>0.0.0</version>
      <description>TODO: Package description</description>
      <maintainer email="user@todo.todo">user</maintainer>
      <license>TODO: License declaration</license>

      <test_depend>ament_copyright</test_depend>
      <test_depend>ament_flake8</test_depend>
      <test_depend>ament_pep257</test_depend>
      <test_depend>python3-pytest</test_depend>

      <export>
        <build_type>ament_python</build_type>
      </export>
     </package>

如果 ``maintainer`` 字段没有自动填充，就在 ``maintainer`` 行输入你的名字和邮箱.
然后编辑 ``description`` 行来描述或概括一下包的内容或作用:

.. code-block:: xml

  <description>Beginner client libraries tutorials practice package</description>

接下来更新 ``license`` 行.
在 `这里 <https://opensource.org/licenses/alphabetical>`__ 可以了解更多关于开源许可证的信息.
因为这个包只是练习用的，所以可以使用任何许可证.
我们使用 ``Apache License 2.0``:

.. code-block:: xml

  <license>Apache License 2.0</license>

编辑完成后别忘了保存.

在 ``license`` 标签(tag)下面，你会看到一些标签名字以 ``_depend`` 结尾.
这是你的 ``package.xml`` 列出了它对其他包的依赖，colcon 会搜索这些依赖.
``my_package`` 很简单，没有依赖，但是在后续教程中你会看到这个标签被用到.

.. tabs::

   .. group-tab:: CMake

      你已经完成了全部任务！

   .. group-tab:: Python

      ``setup.py`` 文件包含了和 ``package.xml`` 一样的描述、维护者和许可证字段，所以你也需要设置这些地方的参数.
      这两个文件中的这些参数需要完全匹配.
      ``setup.py`` 中的版本和名字(``package_name``)也需要完全匹配，应该会自动填充到两个文件中.

      用你喜欢的文本编辑器打开 ``setup.py`` 文件:

      .. code-block:: python

       from setuptools import setup

       package_name = 'my_py_pkg'

       setup(
        name=package_name,
        version='0.0.0',
        packages=[package_name],
        data_files=[
            ('share/ament_index/resource_index/packages',
                    ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
          ],
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='TODO',
        maintainer_email='TODO',
        description='TODO: Package description',
        license='TODO: License declaration',
        tests_require=['pytest'],
        entry_points={
            'console_scripts': [
                    'my_node = my_py_pkg.my_node:main'
            ],
          },
       )

      编辑 ``maintainer``，``maintainer_email`` 和 ``description`` 行与 ``package.xml`` 中保持一致.

      别忘记保存文件.


总结
-------

你已经创建了一个包来组织你的代码，以及让别人更容易使用它.

你的包已经自动填充了必要的文件，然后你用 colcon 构建它，这样你就可以在本地环境中使用它的可执行文件了.

下一步
----------

接下来，让我们给包添加一些有意义的东西.
你将实现一个简单的发布/订阅系统，用 :doc:`C++ <./Writing-A-Simple-Cpp-Publisher-And-Subscriber>` 或者 :doc:`Python <./Writing-A-Simple-Py-Publisher-And-Subscriber>` 都可以.
