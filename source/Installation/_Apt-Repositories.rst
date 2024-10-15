你需要将 ROS 2 apt 仓库添加到你的系统中。

首先确保 `Ubuntu Universe 仓库 <https://help.ubuntu.com/community/Repositories/Ubuntu>`_ 已启用。

.. code-block:: bash

   sudo apt install software-properties-common
   sudo add-apt-repository universe

然后使用 apt 添加 ROS 2 GPG 密钥。

.. code-block:: bash

   sudo apt update && sudo apt install curl -y
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

然后将仓库添加到你的源列表中。

.. code-block:: bash

   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
