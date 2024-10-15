请确保您有一个支持 ``UTF-8`` 的区域设置。
如果您处于最简环境中（例如 docker 容器），区域设置可能是一些类似 ``POSIX`` 的选项。
我们使用以下设置通过了测试。但是，如果您使用不同的支持 ``UTF-8`` 的区域设置，也应该没问题。

.. code-block:: bash

   locale  # check for UTF-8

   sudo apt update && sudo apt install locales
   sudo locale-gen en_US en_US.UTF-8
   sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
   export LANG=en_US.UTF-8

   locale  # verify settings
