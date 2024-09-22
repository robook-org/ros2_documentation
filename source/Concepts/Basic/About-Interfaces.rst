.. redirect-from::

    About-ROS-Interfaces
    Concepts/About-ROS-Interfaces

接口(Interfaces)
=================

.. contents:: Table of Contents
   :local:

背景
----------

ROS 应用程序通常通过以下三种类型的接口进行通信： :doc:`topics <About-Topics>`、 :doc:`services <About-Services>` 或 :doc:`actions <About-Actions>`。
ROS 2 使用一种简化的描述语言，接口定义语言(the interface definition language, IDL)，来描述这些接口。
这种描述语言使得 ROS 工具能够轻松地为接口在多种目标编程语言中自动生成源代码。

在这个文档中，我们将描述支持的类型：

* msg: ``.msg`` 文件是简单的文本文件，描述了 ROS 消息(message)的字段。它们用于在不同的语言中生成消息的源代码。
* srv: ``.srv`` 文件描述了一个服务。它们由两部分组成：请求(request)和响应(response)。请求和响应都是消息声明。
* action: ``.action`` 文件描述了动作。它们由三部分组成：目标(goal)、结果(result)和反馈(feedback)。每一部分都是一个消息声明。

消息(Messages)
----------------

消息是 ROS 2 节点在网络上向其他 ROS 节点发送数据的一种方式，不需要有响应。
例如，如果一个 ROS 2 节点从传感器读取温度数据，它可以使用一个 ``Temperature`` 消息将数据发布到 ROS 2 网络上。
ROS 2 网络上的其他节点可以订阅这些消息并从中接收 ``Temperature`` 消息。

消息在 ROS 包的 ``msg/`` 目录中的 ``.msg`` 文件中描述和定义。
``.msg`` 文件由两部分组成：字段和常量。

字段(Fields)
^^^^^^^^^^^^^

每个字段由类型和名称组成，用空格分隔，像这样：

.. code-block:: bash

   fieldtype1 fieldname1
   fieldtype2 fieldname2
   fieldtype3 fieldname3

例如:

.. code-block:: bash

   int32 my_int
   string my_string

字段类型(Field types)
~~~~~~~~~~~~~~~~~~~~~

字段类型可以是:

* 一种内置类型
* 一种自定义的消息类型，例如 ``geometry_msgs/PoseStamped``

*目前支持的内置类型:*

.. list-table::
   :header-rows: 1

   * - Type name
     - `C++ <https://design.ros2.org/articles/generated_interfaces_cpp.html>`__
     - `Python <https://design.ros2.org/articles/generated_interfaces_python.html>`__
     - `DDS type <https://design.ros2.org/articles/mapping_dds_types.html>`__
   * - bool
     - bool
     - builtins.bool
     - boolean
   * - byte
     - uint8_t
     - builtins.bytes*
     - octet
   * - char
     - char
     - builtins.int*
     - char
   * - float32
     - float
     - builtins.float*
     - float
   * - float64
     - double
     - builtins.float*
     - double
   * - int8
     - int8_t
     - builtins.int*
     - octet
   * - uint8
     - uint8_t
     - builtins.int*
     - octet
   * - int16
     - int16_t
     - builtins.int*
     - short
   * - uint16
     - uint16_t
     - builtins.int*
     - unsigned short
   * - int32
     - int32_t
     - builtins.int*
     - long
   * - uint32
     - uint32_t
     - builtins.int*
     - unsigned long
   * - int64
     - int64_t
     - builtins.int*
     - long long
   * - uint64
     - uint64_t
     - builtins.int*
     - unsigned long long
   * - string
     - std::string
     - builtins.str
     - string
   * - wstring
     - std::u16string
     - builtins.str
     - wstring

*可以用来定义数组的内置类型:*

.. list-table::
   :header-rows: 1

   * - Type name
     - `C++ <https://design.ros2.org/articles/generated_interfaces_cpp.html>`__
     - `Python <https://design.ros2.org/articles/generated_interfaces_python.html>`__
     - `DDS type <https://design.ros2.org/articles/mapping_dds_types.html>`__
   * - static array
     - std::array<T, N>
     - builtins.list*
     - T[N]
   * - unbounded dynamic array
     - std::vector
     - builtins.list
     - sequence
   * - bounded dynamic array
     - custom_class<T, N>
     - builtins.list*
     - sequence<T, N>
   * - bounded string
     - std::string
     - builtins.str*
     - string

比 ROS 定义更宽松的类型会被软件强限制到 ROS 的范围和长度。

*使用数组和有界类型定义消息的示例:*

.. code-block:: bash

   int32[] unbounded_integer_array
   int32[5] five_integers_array
   int32[<=5] up_to_five_integers_array

   string string_of_unbounded_size
   string<=10 up_to_ten_characters_string

   string[<=5] up_to_five_unbounded_strings
   string<=10[] unbounded_array_of_strings_up_to_ten_characters_each
   string<=10[<=5] up_to_five_strings_up_to_ten_characters_each

字段名称(Field names)
~~~~~~~~~~~~~~~~~~~~~

字段名称必须只由小写字母、数字和下划线组成。
它们必须以字母开头，不能以下划线结尾，也不能有两个连续的下划线。

字段默认值(Field default value)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

消息类型中的任何字段都可以设置默认值。
目前默认值不支持字符串数组和复杂类型（即内置类型表中不存在的类型；这适用于所有嵌套消息）。

定义默认值是通过在字段定义行中添加第三个元素来完成的，像这样:

.. code-block:: bash

   fieldtype fieldname fielddefaultvalue

例如:

.. code-block:: bash

   uint8 x 42
   int16 y -2000
   string full_name "John Doe"
   int32[] samples [-200, -100, 0, 100, 200]

.. note::

  * string 类型的值必须用单引号 ``'`` 或双引号 ``"`` 定义
  * 目前字符串值不能被转义

常量(Constants)
^^^^^^^^^^^^^^^^

常量定义就像一个字段描述，只是这个值不能在程序中被改变。
这个值的赋值通过等号 ``=`` 来表示，像这样:

.. code-block:: bash

   constanttype CONSTANTNAME=constantvalue

例如:

.. code-block:: bash

   int32 X=123
   int32 Y=-123
   string FOO="foo"
   string EXAMPLE='bar'

.. note::

  * 常量名必须全部大写

服务(Services)
----------------

服务是一种请求/响应式通信，其中客户端（请求者）等待服务器（响应者）进行简短的计算并返回结果。

Service 在 ROS 包的 ``srv/`` 目录中的 ``.srv`` 文件中描述和定义。

Service 的描述文件由请求和响应消息类型组成，使用 ``---`` 分隔。
任何两个使用 ``---`` 连接的 ``.msg`` 文件都是合法的 service 描述。

这里是一个非常简单的服务的例子，它接收一个字符串并返回一个字符串:

.. code-block:: bash

   string str
   ---
   string str

当然我们也可以定义更复杂的服务（如果你想引用同一个包中的消息，写 field type 的时候不能写包名）:

.. code-block:: bash

   # request constants
   int8 FOO=1
   int8 BAR=2
   # request fields
   int8 foobar
   another_pkg/AnotherMessage msg
   ---
   # response constants
   uint32 SECRET=123456
   # response fields
   another_pkg/YetAnotherMessage val
   CustomMessageDefinedInThisPackage value
   uint32 an_integer

Service 不能嵌套在另一个 service 中。

Actions
-------

Actions 是一种长时间运行的请求/响应式通信，其中 action 客户端（请求者）等待 action 服务器（响应者）执行某些操作并返回结果。
与服务不同，action 可以是长时间运行的（几秒或几分钟），在运行中可以提供反馈，并且可以被中断。

Action 用如下格式定义:

.. code::

   <request_type> <request_fieldname>
   ---
   <response_type> <response_fieldname>
   ---
   <feedback_type> <feedback_fieldname>

与 service 一样，请求字段在第一个三划线(``---``)之前，响应字段在第一个三划线之后。
在第二个三划线之后的字段在发送反馈时使用。

请求字段、响应字段和反馈字段都可以是任意数量的（包括零个）。

``<request_type>``, ``<response_type>`` 和 ``<feedback_type>`` 遵循消息的 ``<type>`` 的所有规则。
``<request_fieldname>``, ``<response_fieldname>`` 和 ``<feedback_fieldname>`` 遵循消息的 ``<fieldname>`` 的所有规则。

例如，``Fibonacci`` action 定义包含以下内容:

.. code::

   int32 order
   ---
   int32[] sequence
   ---
   int32[] sequence

这是一个 action 定义，其中 action 客户端发送一个表示要执行的 Fibonacci 步数的 ``int32`` 字段，并期望 action 服务器生成一个包含完整序列的 ``int32`` 数组。
在此过程中，action 服务器还可以提供一个目前计算到的序列的数组 ``int32``。（译者注：设想这样的情况，你作为客户端想问服务器计算斐波那契数列的第 n 位之前（包括第n位）的数字都是什么，那么你向服务器发出这个 request 。这个计算可能需要很久，在计算过程中，服务器会把目前计算到的序列通过 feedback 传回给你，让你知道计算的进度，同时最后计算完成之后通过 response 把最终结果返回给你。）
