.. _SinglePkgInterface:

.. redirect-from::

    Rosidl-Tutorial
    Tutorials/Single-Package-Define-And-Use-Interface

实现自定义接口(interfaces)
==============================

**目标:** 学习在 ROS 2 中实现自定义接口的更多方法。

**教程等级:** 初级

**预计时长:** 15 分钟

.. contents:: Contents
   :depth: 2
   :local:

背景
----------

在 :doc:`上一个教程 <./Custom-ROS2-Interfaces>` 中，你学习了如何创建自定义 msg 和 srv 。

不过最佳做法是在专门的接口包中声明接口，有时在一个包中声明、创建和使用接口会更方便。

请注意，目前接口只能在 CMake 包中定义。
但是可以在 CMake 包中包含 Python 库和节点(使用 `ament_cmake_python <ament_cmake_python <https://github.com/ament/ament_cmake/tree/{REPOS_FILE_BRANCH}/ament_cmake_python>`_), 所以可以在同一个包中定义接口并实现 Python 节点.
为了方便起见，我们将在这里使用 CMake 包和 C++ 节点。

本教程将重点介绍 msg 接口类型，但这里提到的步骤适用于所有接口类型。

前提条件
-------------

我们假设你在开始本教程之前已经熟悉了 :doc:`./Custom-ROS2-Interfaces` 教程中的基础知识。

你应该已经安装了 :doc:`ROS 2 <../../Installation>`，有一个 :doc:`工作空间 <./Creating-A-Workspace/Creating-A-Workspace>`，并了解 :doc:`创建包 <./Creating-Your-First-ROS2-Package>`。

还是不要忘记，请在每次打开新终端时 :doc:`source ROS 2 <../Beginner-CLI-Tools/Configuring-ROS2-Environment>`。

任务
-----

1 创建包
^^^^^^^^^^^^^^^^^^

In your workspace ``src`` directory, create a package ``more_interfaces`` and make a directory within it for msg files:

.. code-block:: console

  ros2 pkg create --build-type ament_cmake --license Apache-2.0 more_interfaces
  mkdir more_interfaces/msg

2 创建 msg 文件
^^^^^^^^^^^^^^^^^^^

在 ``more_interfaces/msg`` 中创建一个新文件 ``AddressBook.msg``，并粘贴以下代码,创建一个包含了个人信息的消息：

::

   uint8 PHONE_TYPE_HOME=0
   uint8 PHONE_TYPE_WORK=1
   uint8 PHONE_TYPE_MOBILE=2

   string first_name
   string last_name
   string phone_number
   uint8 phone_type

这个消息由以下字段组成：

* first_name: of type string, 名字。
* last_name: of type string, 姓氏。
* phone_number: of type string, 电话号码。
* phone_type: of type uint8, with several named constant values defined, 电话类型。

请注意，可以在消息定义中为字段设置默认值。
查看 :doc:`../../Concepts/Basic/About-Interfaces` 了解更多自定义接口的方法。

接下来，我们需要确保 msg 文件被转换为 C++、Python 和其他语言的源代码。

2.1 构建 msg
~~~~~~~~~~~~~~~~~~~~

打开 ``package.xml`` 并添加以下行：

.. code-block:: xml

     <buildtool_depend>rosidl_default_generators</buildtool_depend>

     <exec_depend>rosidl_default_runtime</exec_depend>

     <member_of_group>rosidl_interface_packages</member_of_group>

注意，在构建时需要 ``rosidl_default_generators``，而在运行时只需要 ``rosidl_default_runtime``。

打开 ``CMakeLists.txt`` 并添加以下行：

找到用于生成 msg/srv 文件的包：

.. code-block:: cmake

   find_package(rosidl_default_generators REQUIRED)

声明想要生成消息的源文件列表：

.. code-block:: cmake

   set(msg_files
     "msg/AddressBook.msg"
   )

通过手动添加 ``.msg`` 文件，我们让 CMake 知道在添加其他 ``.msg`` 文件后需要重新配置项目。

生成消息：

.. code-block:: cmake

   rosidl_generate_interfaces(${PROJECT_NAME}
     ${msg_files}
   )

确保导出消息运行时依赖项：

.. code-block:: cmake

   ament_export_dependencies(rosidl_default_runtime)

现在为 msg 定义生成源文件需要的准备已经完成。
不过我们暂时先跳过构建步骤，因为我们将在下面的第 4 步一起完成。

3 在同一个包中使用接口
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

现在我们开始编写使用这个消息的代码。

在 ``more_interfaces/src`` 中创建一个名为 ``publish_address_book.cpp`` 的文件，并粘贴以下代码：

.. code-block:: c++

  #include <chrono>
  #include <memory>

  #include "rclcpp/rclcpp.hpp"
  #include "more_interfaces/msg/address_book.hpp"

  using namespace std::chrono_literals;

  class AddressBookPublisher : public rclcpp::Node
  {
  public:
    AddressBookPublisher()
    : Node("address_book_publisher")
    {
      address_book_publisher_ =
        this->create_publisher<more_interfaces::msg::AddressBook>("address_book", 10);

      auto publish_msg = [this]() -> void {
          auto message = more_interfaces::msg::AddressBook();

          message.first_name = "John";
          message.last_name = "Doe";
          message.phone_number = "1234567890";
          message.phone_type = message.PHONE_TYPE_MOBILE;

          std::cout << "Publishing Contact\nFirst:" << message.first_name <<
            "  Last:" << message.last_name << std::endl;

          this->address_book_publisher_->publish(message);
        };
      timer_ = this->create_wall_timer(1s, publish_msg);
    }

  private:
    rclcpp::Publisher<more_interfaces::msg::AddressBook>::SharedPtr address_book_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
  };


  int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AddressBookPublisher>());
    rclcpp::shutdown();

    return 0;
  }

3.1 代码解释
~~~~~~~~~~~~~~~~~~~~~~

首先包含我们新创建的 ``AddressBook.msg`` 的头文件。

.. code-block:: c++

   #include "more_interfaces/msg/address_book.hpp"

创建一个节点和一个 ``AddressBook`` 发布者。

.. code-block:: c++

   using namespace std::chrono_literals;

   class AddressBookPublisher : public rclcpp::Node
   {
   public:
     AddressBookPublisher()
     : Node("address_book_publisher")
     {
       address_book_publisher_ =
         this->create_publisher<more_interfaces::msg::AddressBook>("address_book");

创建一个回调函数以定期发布消息。

.. code-block:: c++

    auto publish_msg = [this]() -> void {

创建一条稍后要发布的 ``AddressBook`` 消息实例。

.. code-block:: c++

    auto message = more_interfaces::msg::AddressBook();

填充 ``AddressBook`` 的字段。

.. code-block:: c++

    message.first_name = "John";
    message.last_name = "Doe";
    message.phone_number = "1234567890";
    message.phone_type = message.PHONE_TYPE_MOBILE;

最后定期发送消息。

.. code-block:: c++

    std::cout << "Publishing Contact\nFirst:" << message.first_name <<
      "  Last:" << message.last_name << std::endl;

    this->address_book_publisher_->publish(message);

创建一个 1 秒定时器，每秒调用我们的 ``publish_msg`` 函数。

.. code-block:: c++

       timer_ = this->create_wall_timer(1s, publish_msg);

3.2 构建发布者
~~~~~~~~~~~~~~~~~~~~~~~

我们需要在 ``CMakeLists.txt`` 中为这个节点创建一个新目标(target)：

.. code-block:: cmake

   find_package(rclcpp REQUIRED)

   add_executable(publish_address_book src/publish_address_book.cpp)
   ament_target_dependencies(publish_address_book rclcpp)

   install(TARGETS
       publish_address_book
     DESTINATION lib/${PROJECT_NAME})

3.3 链接(link)接口
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

为了在同一个包中使用生成的消息，我们需要使用以下 CMake 代码：

.. code-block:: cmake

  rosidl_get_typesupport_target(cpp_typesupport_target
    ${PROJECT_NAME} rosidl_typesupport_cpp)

  target_link_libraries(publish_address_book "${cpp_typesupport_target}")

这会找到与 ``AddressBook.msg`` 相关的生成的 C++ 代码，并允许你的目标链接到它。

你可能已经注意到，当使用来自独立构建的不同包的接口时，这一步是不必要的。
只有在想要在定义它们的包中使用接口时才需要这段 CMake 代码。

4 尝试一下
^^^^^^^^^^^^

返回工作空间的根目录，构建包：

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      cd ~/ros2_ws
      colcon build --packages-up-to more_interfaces

  .. group-tab:: macOS

    .. code-block:: console

      cd ~/ros2_ws
      colcon build --packages-up-to more_interfaces

  .. group-tab:: Windows

    .. code-block:: console

      cd /ros2_ws
      colcon build --merge-install --packages-up-to more_interfaces

然后 source 工作空间并运行发布者：

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      source install/local_setup.bash
      ros2 run more_interfaces publish_address_book

  .. group-tab:: macOS

    .. code-block:: console

      . install/local_setup.bash
      ros2 run more_interfaces publish_address_book

  .. group-tab:: Windows

    .. code-block:: console

      call install/local_setup.bat
      ros2 run more_interfaces publish_address_book

    Or using Powershell:

    .. code-block:: console

      install/local_setup.ps1
      ros2 run more_interfaces publish_address_book

你应该看到发布者传递了你定义的消息，包括 ``publish_address_book.cpp`` 中设置的值。

为了确认消息被发布到 ``address_book`` topic 上，打开另一个终端，source 工作空间，然后调用 ``topic echo``：

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      source install/setup.bash
      ros2 topic echo /address_book

  .. group-tab:: macOS

    .. code-block:: console

      . install/setup.bash
      ros2 topic echo /address_book

  .. group-tab:: Windows

    .. code-block:: console

      call install/setup.bat
      ros2 topic echo /address_book

    Or using Powershell:

    .. code-block:: console

      install/setup.ps1
      ros2 topic echo /address_book

在本教程中，我们没有创建订阅者，但你可以尝试自己编写一个(使用 :doc:`./Writing-A-Simple-Cpp-Publisher-And-Subscriber` 进行练习)。

5 (额外内容) 使用现有接口定义
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. note::

  你可以在新接口定义中使用现有的接口定义。
  比如说，假设有一个名为 ``Contact.msg`` 的消息，属于一个名为 ``rosidl_tutorials_msgs`` 的现有 ROS 2 包。
  假设它的定义与我们之前自定义的 ``AddressBook.msg`` 接口完全相同。

  在这种情况下，你可以将 ``AddressBook.msg`` (在你自己包中的接口)定义为类型 ``Contact`` (一个你的包以外的包中的接口)。
  你甚至可以将 ``AddressBook.msg`` 定义为 ``Contact`` 类型的数组，如下所示：

  ::

     rosidl_tutorials_msgs/Contact[] address_book

  为了生成这个消息，你需要在 ``package.xml`` 中声明对 ``Contact.msg`` 的包 ``rosidl_tutorials_msgs`` 的依赖：

  .. code-block:: xml

       <build_depend>rosidl_tutorials_msgs</build_depend>

       <exec_depend>rosidl_tutorials_msgs</exec_depend>

  在 ``CMakeLists.txt`` 中添加：

  .. code-block:: cmake

     find_package(rosidl_tutorials_msgs REQUIRED)

     rosidl_generate_interfaces(${PROJECT_NAME}
       ${msg_files}
       DEPENDENCIES rosidl_tutorials_msgs
     )

  你还需要在发布者节点中包含 ``Contact.msg`` 的头文件，以便能够将 ``contacts`` 添加到你的 ``address_book`` 中。

  .. code-block:: c++

     #include "rosidl_tutorials_msgs/msg/contact.hpp"

  你可以将回调函数更改为以下内容：

  .. code-block:: c++

    auto publish_msg = [this]() -> void {
       auto msg = std::make_shared<more_interfaces::msg::AddressBook>();
       {
         rosidl_tutorials_msgs::msg::Contact contact;
         contact.first_name = "John";
         contact.last_name = "Doe";
         contact.phone_number = "1234567890";
         contact.phone_type = message.PHONE_TYPE_MOBILE;
         msg->address_book.push_back(contact);
       }
       {
         rosidl_tutorials_msgs::msg::Contact contact;
         contact.first_name = "Jane";
         contact.last_name = "Doe";
         contact.phone_number = "4254242424";
         contact.phone_type = message.PHONE_TYPE_HOME;
         msg->address_book.push_back(contact);
       }

       std::cout << "Publishing address book:" << std::endl;
       for (auto contact : msg->address_book) {
         std::cout << "First:" << contact.first_name << "  Last:" << contact.last_name <<
           std::endl;
       }

       address_book_publisher_->publish(*msg);
     };

  构建和运行这些更改将显示前面定义好的的 msg，以及上面定义的 msg 数组。

总结
-------

在本教程中，你尝试了不同的字段类型来定义接口，然后在在这个接口所在的包中构建好它。

你还学习了如何使用另一个接口作为字段类型，以及使用 ``package.xml``、 ``CMakeLists.txt`` 和 ``#include`` 来应用这个接口。

下一步
----------

接下来，你将创建一个简单的 ROS 2 包，其中包含一个自定义参数，你将学习如何从一个 launch 文件中设置这个参数。
当然，你还是可以选择用 :doc:`C++ <./Using-Parameters-In-A-Class-CPP>` 或 :doc:`Python <./Using-Parameters-In-A-Class-Python>` 中编写它。

相关内容
---------------

这有 `一些设计文档 <./Custom-ROS2-Interfaces>` 是关于 IDL (interface definition language) 的。
