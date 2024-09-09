.. redirect-from::

    Developing-a-ROS-2-Package
    Guides/Developing-a-ROS-2-Package
    Tutorials/Developing-a-ROS-2-Package

开发一个 ROS 2 包
##########################

.. contents:: Table of Contents
   :depth: 2
   :local:

本教程将教你如何创建你的第一个 ROS 2 应用。
它适用于想要学习如何在 ROS 2 中创建自定义包的开发人员，而不是想要使用 ROS 2 的现有包的人。

前提条件
-------------

- :doc:`安装 ROS <../../Installation>`

- `安装 colcon <https://colcon.readthedocs.io/en/released/user/installation.html>`__

- source 你的 ROS 2 安装以设置你的工作空间。

创建一个新的 ROS 2 包
--------------------------

所有 ROS 2 包都是通过运行以下命令开始的:

.. code-block:: bash

   ros2 pkg create --license Apache-2.0 <pkg-name> --dependencies [deps]

在你的 workspace 中(通常是 ``~/ros2_ws/src``).

如果你想为特定的客户端库创建一个包:

.. tabs::

  .. group-tab:: C++

    .. code-block:: bash

       ros2 pkg create  --build-type ament_cmake --license Apache-2.0 <pkg-name> --dependencies [deps]

  .. group-tab:: Python

    .. code-block:: bash

       ros2 pkg create  --build-type ament_python --license Apache-2.0 <pkg-name> --dependencies [deps]

你可以在 ``package.xml`` 文件中更新你的包信息，如依赖、描述和作者。

C++ 包
^^^^^^^^^^^^

你会主要使用 ``add_executable()`` CMake 宏

.. code-block:: cmake

   ament_target_dependencies(<executable-name> [dependencies])

来可执行的节点并且链接依赖。

用 ``install()`` 宏安装你的启动文件和节点，放在文件末尾但在 ``ament_package()`` 宏之前。

样例：

.. code-block:: cmake

   # Install launch files
   install(
     DIRECTORY launch
     DESTINATION share/${PROJECT_NAME}
   )

   # Install nodes
   install(
     TARGETS [node-names]
     DESTINATION lib/${PROJECT_NAME}
   )

Python 包
^^^^^^^^^^^^^^^

ROS 2 遵循使用使用 ``setuptools`` 的 Python 标准模块分发流程，。
对于 Python 包，``setup.py`` 文件与 C++ 包的 ``CMakeLists.txt`` 文件作用相似。
更多关于分发的细节可以在 `官方文档 <https://docs.python.org/3/distributing/index.html#distributing-index>`_ 中找到。

在你的 ROS 2 包中，你应该有一个类似这样的 ``setup.cfg`` 文件:

.. code-block:: bash

   [develop]
   script_dir=$base/lib/<package-name>
   [install]
   install_scripts=$base/lib/<package-name>

和一个类似这样的 ``setup.py`` 文件:

.. code-block:: python

   import os
   from glob import glob
   from setuptools import setup

   package_name = 'my_package'

   setup(
       name=package_name,
       version='0.0.0',
       # Packages to export
       packages=[package_name],
       # Files we want to install, specifically launch files
       data_files=[
           # Install marker file in the package index
           ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
           # Include our package.xml file
           (os.path.join('share', package_name), ['package.xml']),
           # Include all launch files.
           (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
       ],
       # This is important as well
       install_requires=['setuptools'],
       zip_safe=True,
       author='ROS 2 Developer',
       author_email='ros2@ros.com',
       maintainer='ROS 2 Developer',
       maintainer_email='ros2@ros.com',
       keywords=['foo', 'bar'],
       classifiers=[
           'Intended Audience :: Developers',
           'License :: TODO',
           'Programming Language :: Python',
           'Topic :: Software Development',
       ],
       description='My awesome package.',
       license='TODO',
       # Like the CMakeLists add_executable macro, you can add your python
       # scripts here.
       entry_points={
           'console_scripts': [
               'my_script = my_package.my_script:main'
           ],
       },
   )


C++ and Python 混合包
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

编写既包含 C++ 代码又包含 Python 代码的包时，不使用 ``setup.py`` 文件和 ``setup.cfg`` 文件，请使用 :doc:`ament_cmake_python <./Ament-CMake-Python-Documentation>`。
