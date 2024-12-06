.. redirect-from::

    Tutorials/Launch-Files/Using-Substitutions
    Tutorials/Launch/Using-Substitutions

使用可替换变量(substitutions)
===================================

**目标:** 学习 ROS 2 启动文件中可替换变量的使用。

**教程等级:** 中级

**预计时长:** 15 分钟

.. contents:: Table of Contents
   :depth: 2
   :local:

背景
----------

启动文件用于启动节点、服务和运行进程。
这些操作可能需要参数，这些参数会影响它们的行为。
可替换变量(substitutions)可以用于参数，以提供更大的灵活性，描述可重用的启动文件。
可替换变量是只在启动文件执行期间才会计算的变量，可以用于获取特定信息，如启动配置、环境变量，或计算一些 Python 表达式。

本教程展示了 ROS 2 启动文件中可替换变量的使用。

前提条件
-------------

本教程用到了 :doc:`turtlesim <../../Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim>` 包。
本教程还假设你已经熟悉了 :doc:`创建包 <../../Beginner-Client-Libraries/Creating-Your-First-ROS2-Package>`。

记得要在 :doc:`每次打开新终端时 <../../Beginner-CLI-Tools/Configuring-ROS2-Environment>` 中 source ROS 2。

使用可替换变量
-------------------

1 创建并配置包
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

首先，创建一个名为 ``launch_tutorial`` 的新包：

.. tabs::

  .. group-tab:: Python package

    创建一个构建类型为 ``ament_python`` 的新包：

    .. code-block:: console

      ros2 pkg create --build-type ament_python --license Apache-2.0 launch_tutorial

  .. group-tab:: C++ package

    创建一个构建类型为 ``ament_cmake`` 的新包：

    .. code-block:: console

      ros2 pkg create --build-type ament_cmake --license Apache-2.0 launch_tutorial

接着，在包内创建一个名为 ``launch`` 的目录：

.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

      mkdir launch_tutorial/launch

  .. group-tab:: macOS

    .. code-block:: bash

      mkdir launch_tutorial/launch

  .. group-tab:: Windows

    .. code-block:: bash

      md launch_tutorial/launch

最后确保安装文件能正确安装：

.. tabs::

  .. group-tab:: Python package

    添加以下更改到包的 ``setup.py`` 文件：

    .. code-block:: python

      import os
      from glob import glob
      from setuptools import find_packages, setup

      package_name = 'launch_tutorial'

      setup(
          # Other parameters ...
          data_files=[
              # ... Other data files
              # Include all launch files.
              (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
          ]
      )

  .. group-tab:: C++ package

    在 ``CMakeLists.txt`` 文件的 ``ament_package()`` 之前添加以下代码：

    .. code-block:: cmake

      install(DIRECTORY
              launch
              DESTINATION share/${PROJECT_NAME}/
      )



2 外层启动文件
^^^^^^^^^^^^^^^^^^^^

让我们创建一个可以调用并传递参数给另一个启动文件的启动文件。
这个启动文件可以是 Python 或 YAML 的类型。

为此，在 ``launch_tutorial`` 包的 ``launch`` 文件夹中创建以下文件。

.. tabs::

  .. group-tab:: Python


    将完整代码复制并粘贴到 ``launch/example_main.launch.py`` 文件中：

    .. code-block:: python

      from launch_ros.substitutions import FindPackageShare

      from launch import LaunchDescription
      from launch.actions import IncludeLaunchDescription
      from launch.launch_description_sources import PythonLaunchDescriptionSource
      from launch.substitutions import PathJoinSubstitution, TextSubstitution


      def generate_launch_description():
          colors = {
              'background_r': '200'
          }

          return LaunchDescription([
              IncludeLaunchDescription(
                  PythonLaunchDescriptionSource([
                      PathJoinSubstitution([
                          FindPackageShare('launch_tutorial'),
                          'launch',
                          'example_substitutions.launch.py'
                      ])
                  ]),
                  launch_arguments={
                      'turtlesim_ns': 'turtlesim2',
                      'use_provided_red': 'True',
                      'new_background_r': TextSubstitution(text=str(colors['background_r']))
                  }.items()
              )
          ])

    ``FindPackageShare`` 用于查找 ``launch_tutorial`` 包的路径。
    然后使用 ``PathJoinSubstitution`` 将找到的包路径与 ``example_substitutions.launch.py`` 的文件名连接起来。

    .. code-block:: python

      PathJoinSubstitution([
          FindPackageShare('launch_tutorial'),
          'launch',
          'example_substitutions.launch.py'
      ])

    ``launch_arguments`` 中会包含 ``turtlesim_ns`` 和 ``use_provided_red`` ，会被传递给 ``IncludeLaunchDescription`` action。
    ``TextSubstitution`` 用于定义 ``new_background_r`` 参数，值为 ``colors`` 字典中的 ``background_r`` 所对应的值。

    .. code-block:: python

      launch_arguments={
          'turtlesim_ns': 'turtlesim2',
          'use_provided_red': 'True',
          'new_background_r': TextSubstitution(text=str(colors['background_r']))
      }.items()

  .. group-tab:: YAML


    将完整代码复制并粘贴到 ``launch/example_main.launch.yaml`` 文件中：

    .. code-block:: yaml

      launch:
        - let:
            name: 'background_r'
            value: '200'
        - include:
            file: '$(find-pkg-share launch_tutorial)/launch/example_substitutions.launch.yaml'
            arg:
              - name: 'turtlesim_ns'
                value: 'turtlesim2'
              - name: 'use_provided_red'
                value: 'True'
              - name: 'new_background_r'
                value: '$(var background_r)'

    ``$(find-pkg-share launch_tutorial)`` 用于查找 ``launch_tutorial`` 包的路径。
    然后找到的包路径会和 ``example_substitutions.launch.yaml`` 的文件名连接起来，变成一个完整路径。

    .. code-block:: yaml

      file: '$(find-pkg-share launch_tutorial)/launch/example_substitutions.launch.yaml'

    ``background_r`` 变量会和 ``turtlesim_ns`` 以及 ``use_provided_red`` 参数一起传递给 ``include`` action。
    ``$(var background_r)`` 会用 ``background_r`` 变量的值来定义 ``new_background_r`` 参数。

    .. code-block:: yaml

      arg:
        - name: 'turtlesim_ns'
          value: 'turtlesim2'
        - name: 'use_provided_red'
          value: 'True'
        - name: 'new_background_r'
          value: '$(var background_r)'

3 用到可替换变量的启动文件样例
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

现在在同一个文件夹中创建一个带有可替换变量的启动文件：

.. tabs::

  .. group-tab:: Python

    创建 ``launch/example_substitutions.launch.py`` 文件，并插入以下代码：

    .. code-block:: python

        from launch_ros.actions import Node

        from launch import LaunchDescription
        from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
        from launch.conditions import IfCondition
        from launch.substitutions import LaunchConfiguration, PythonExpression


        def generate_launch_description():
            turtlesim_ns = LaunchConfiguration('turtlesim_ns')
            use_provided_red = LaunchConfiguration('use_provided_red')
            new_background_r = LaunchConfiguration('new_background_r')

            turtlesim_ns_launch_arg = DeclareLaunchArgument(
                'turtlesim_ns',
                default_value='turtlesim1'
            )
            use_provided_red_launch_arg = DeclareLaunchArgument(
                'use_provided_red',
                default_value='False'
            )
            new_background_r_launch_arg = DeclareLaunchArgument(
                'new_background_r',
                default_value='200'
            )

            turtlesim_node = Node(
                package='turtlesim',
                namespace=turtlesim_ns,
                executable='turtlesim_node',
                name='sim'
            )
            spawn_turtle = ExecuteProcess(
                cmd=[[
                    'ros2 service call ',
                    turtlesim_ns,
                    '/spawn ',
                    'turtlesim/srv/Spawn ',
                    '"{x: 2, y: 2, theta: 0.2}"'
                ]],
                shell=True
            )
            change_background_r = ExecuteProcess(
                cmd=[[
                    'ros2 param set ',
                    turtlesim_ns,
                    '/sim background_r ',
                    '120'
                ]],
                shell=True
            )
            change_background_r_conditioned = ExecuteProcess(
                condition=IfCondition(
                    PythonExpression([
                        new_background_r,
                        ' == 200',
                        ' and ',
                        use_provided_red
                    ])
                ),
                cmd=[[
                    'ros2 param set ',
                    turtlesim_ns,
                    '/sim background_r ',
                    new_background_r
                ]],
                shell=True
            )

            return LaunchDescription([
                turtlesim_ns_launch_arg,
                use_provided_red_launch_arg,
                new_background_r_launch_arg,
                turtlesim_node,
                spawn_turtle,
                change_background_r,
                TimerAction(
                    period=2.0,
                    actions=[change_background_r_conditioned],
                )
            ])

    先定义一些启动配置变量(launch configurations) ``turtlesim_ns``、 ``use_provided_red`` 和 ``new_background_r``。
    这些变量用于存储启动参数的值，并将它们传递给所需的 actions。
    可以在启动描述(launch description)的任何部分从 ``LaunchConfiguration`` 中获取启动参数的值。

    ``DeclareLaunchArgument`` 用于定义可以从上面的启动文件或控制台传递进来的启动参数。

    .. code-block:: python

        turtlesim_ns = LaunchConfiguration('turtlesim_ns')
        use_provided_red = LaunchConfiguration('use_provided_red')
        new_background_r = LaunchConfiguration('new_background_r')

        turtlesim_ns_launch_arg = DeclareLaunchArgument(
            'turtlesim_ns',
            default_value='turtlesim1'
        )
        use_provided_red_launch_arg = DeclareLaunchArgument(
            'use_provided_red',
            default_value='False'
        )
        new_background_r_launch_arg = DeclareLaunchArgument(
            'new_background_r',
            default_value='200'
        )

    定义 ``turtlesim_node`` 节点， ``namespace`` 设置为用 ``LaunchConfiguration`` 产生的 ``turtlesim_ns`` 。

    .. code-block:: python

        turtlesim_node = Node(
            package='turtlesim',
            namespace=turtlesim_ns,
            executable='turtlesim_node',
            name='sim'
        )

    然后，定义一个名为 ``spawn_turtle`` 的 ``ExecuteProcess`` action，带有相应的 ``cmd`` 参数。
    这个命令会调用 turtlesim 节点的 spawn 服务。

    此外，使用 ``LaunchConfiguration`` 来获取 ``turtlesim_ns`` 启动参数的值，以构建命令字符串。

    .. code-block:: python

        spawn_turtle = ExecuteProcess(
            cmd=[[
                'ros2 service call ',
                turtlesim_ns,
                '/spawn ',
                'turtlesim/srv/Spawn ',
                '"{x: 2, y: 2, theta: 0.2}"'
            ]],
            shell=True
        )

    同样的方法用于 ``change_background_r`` 和 ``change_background_r_conditioned`` actions，它们用于改变 turtlesim 背景的 rgb 颜色中的红色分量。
    不同的是 ``change_background_r_conditioned`` action 只有在提供的 ``new_background_r`` 参数等于 ``200`` 且 ``use_provided_red`` 启动参数设置为 ``True`` 时才会执行。
    ``IfCondition`` 用 ``PythonExpression`` substitution 完成对这些条件的判断。

    .. code-block:: python

        change_background_r = ExecuteProcess(
            cmd=[[
                'ros2 param set ',
                turtlesim_ns,
                '/sim background_r ',
                '120'
            ]],
            shell=True
        )
        change_background_r_conditioned = ExecuteProcess(
            condition=IfCondition(
                PythonExpression([
                    new_background_r,
                    ' == 200',
                    ' and ',
                    use_provided_red
                ])
            ),
            cmd=[[
                'ros2 param set ',
                turtlesim_ns,
                '/sim background_r ',
                new_background_r
            ]],
            shell=True
        )

  .. group-tab:: YAML

    创建 ``launch/example_substitutions.launch.yaml`` 文件，并插入以下代码：

    .. code-block:: yaml

      launch:
        - arg:
            name: 'turtlesim_ns'
            default: 'turtlesim1'
        - arg:
            name: 'use_provided_red'
            default: 'False'
        - arg:
            name: 'new_background_r'
            default: '200'

        - node:
            pkg: 'turtlesim'
            namespace: '$(var turtlesim_ns)'
            exec: 'turtlesim_node'
            name: 'sim'
        - executable:
            cmd: 'ros2 service call $(var turtlesim_ns)/spawn turtlesim/srv/Spawn "{x: 5, y: 2, theta: 0.2}"'
        - executable:
            cmd: 'ros2 param set $(var turtlesim_ns)/sim background_r 120'
        - timer:
            period: 2.0
            children:
              - executable:
                  cmd: 'ros2 param set $(var turtlesim_ns)/sim background_r $(var new_background_r)'
                  if: '$(eval "$(var new_background_r) == 200 and $(var use_provided_red)")'

    此处定义好了 ``turtlesim_ns``、 ``use_provided_red`` 和 ``new_background_r`` 启动配置。
    它们用于存储上面变量的启动参数值，并将它们传递给所需的 actions。
    随后可以在启动描述的任何部分使用 ``$(var <name>)`` substitution 来获取启动参数的值。

    ``arg`` 标签用于定义可以从上面的启动文件或控制台传递进来的启动参数。

    .. code-block:: yaml

      - arg:
          name: 'turtlesim_ns'
          default: 'turtlesim1'
      - arg:
          name: 'use_provided_red'
          default: 'False'
      - arg:
          name: 'new_background_r'
          default: '200'

    定义 ``turtlesim_node`` 节点， ``namespace`` 设置为用 ``$(var <name>)`` substitution 产生的 ``turtlesim_ns`` 。

    .. code-block:: yaml

      - node:
          pkg: 'turtlesim'
          namespace: '$(var turtlesim_ns)'
          exec: 'turtlesim_node'
          name: 'sim'

    随后，定义一个名为 ``executable`` 的 action，带有相应的 ``cmd`` 参数。
    这个命令会调用 turtlesim 节点的 spawn 服务。

    此外，使用 ``$(var <name>)`` substitution 来获取 ``turtlesim_ns`` 启动参数的值，以构建命令字符串。

    .. code-block:: yaml

        - executable:
            cmd: 'ros2 service call $(var turtlesim_ns)/spawn turtlesim/srv/Spawn "{x: 5, y: 2, theta: 0.2}"'

    同样的方法用于 ``change_background_r`` 和 ``change_background_r_conditioned`` actions，它们用于改变 turtlesim 背景的 rgb 颜色中的红色分量。
    不同的是 ``change_background_r_conditioned`` action 只有在提供的 ``new_background_r`` 参数等于 ``200`` 且 ``use_provided_red`` 启动参数设置为 ``True`` 时才会执行。
    ``IfCondition`` 用 ``$(eval <python-expression>)`` substitution 完成对这些条件的判断。

    .. code-block:: yaml

        - executable:
            cmd: 'ros2 param set $(var turtlesim_ns)/sim background_r 120'
        - timer:
            period: 2.0
            children:
              - executable:
                  cmd: 'ros2 param set $(var turtlesim_ns)/sim background_r $(var new_background_r)'
                  if: '$(eval "$(var new_background_r) == 200 and $(var use_provided_red)")'

4 构建包
^^^^^^^^^^^^^^^^^^^

回到工作空间的根目录，构建包：

.. code-block:: console

  colcon build

构建后别忘了 source 工作空间。

运行示例
-----------------

现在可以使用 ``ros2 launch`` 命令来启动。

.. tabs::

  .. group-tab:: Python

    .. code-block:: console

        ros2 launch launch_tutorial example_main.launch.py

  .. group-tab:: YAML

    .. code-block:: console

        ros2 launch launch_tutorial example_main.launch.yaml

这将执行以下操作：

#. 启动一个带有蓝色背景的 turtlesim 节点
#. 生成第二只乌龟
#. 将背景颜色改为紫色
#. 如果提供的 ``background_r`` 参数为 ``200`` 且 ``use_provided_red`` 参数为 ``True``，则两秒后将颜色改为粉色

修改启动参数
--------------------------

.. tabs::

  .. group-tab:: Python

    如果要更改提供的启动参数，可以在 ``example_main.launch.py`` 的 ``launch_arguments`` 字典中更新它们，或者启动 ``example_substitutions.launch.py`` 。
    要查看可以传递给启动文件的参数，运行以下命令：

    .. code-block:: console

        ros2 launch launch_tutorial example_substitutions.launch.py --show-args

  .. group-tab:: YAML

    如果要更改提供的启动参数，可以在 ``example_main.launch.yaml`` 的 ``launch_arguments`` 字典中更新它们，或者启动 ``example_substitutions.launch.yaml`` 时传递参数进去。
    要查看可以传递给启动文件的参数，运行以下命令：

    .. code-block:: console

        ros2 launch launch_tutorial example_substitutions.launch.yaml --show-args

这将显示可以传递给启动文件的参数及其默认值。

.. code-block:: console

    Arguments (pass arguments as '<name>:=<value>'):

        'turtlesim_ns':
            no description given
            (default: 'turtlesim1')

        'use_provided_red':
            no description given
            (default: 'False')

        'new_background_r':
            no description given
            (default: '200')

现在可以通过以下方式将所需的参数传递给启动文件：

.. tabs::

  .. group-tab:: Python

    .. code-block:: console

        ros2 launch launch_tutorial example_substitutions.launch.py turtlesim_ns:='turtlesim3' use_provided_red:='True' new_background_r:=200

  .. group-tab:: YAML

    .. code-block:: console

        ros2 launch launch_tutorial example_substitutions.launch.yaml turtlesim_ns:='turtlesim3' use_provided_red:='True' new_background_r:=200

文档
-------------

`The launch documentation <https://github.com/ros2/launch/blob/{REPOS_FILE_BRANCH}/launch/doc/source/architecture.rst>`_ 提供了有关可用替换变量的详细信息。

总结
-------

在本教程中，你学习了如何在启动文件中使用可替换变量。
你学习了用它们创建带有可修改的启动变量的启动文件。

现在可以学习 :doc:`在启动文件中使用事件处理程序 <./Using-Event-Handlers>`，它们用于定义一组复杂的规则，可以用于动态修改启动文件。
