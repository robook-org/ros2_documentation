.. redirect-from::

  Docs-Guide

ROS 2 文档
===================

.. toctree::
   :titlesonly:
   :maxdepth: 1
   :hidden:

   Installation
   Releases
   Tutorials
   How-To-Guides
   Concepts
   Contact
   The-ROS2-Project
   Package-Docs
   Related-Projects
   Glossary
   Citations


**机器人操作系统(The Robot Operating System / ROS) 是一系列用于构建机器人应用的软件库和工具包的集合**
从驱动和算法最佳实现到强大的开发者工具，ROS 提供了您下一个机器人项目所需的开源工具。

自从 ROS 于 2007 年启动以来，机器人领域和 ROS 社区发生了很多变化。
ROS2 的目标是适应这些变、保留 ROS1 的优势并改进不足之处。

**您是否在寻找特定 ROS 包的文档，如 MoveIt、image_proc 或 octomap？** 请查看 `ROS Index <https://index.ros.org/packages/#{DISTRO}>`__ 或查看 `所有包的文档目录 <https://docs.ros.org/en/{DISTRO}/p/>`__。

本站点包含 ROS 2 的文档，如果您正在寻找 ROS 1 的文档，请查看 `ROS wiki <https://wiki.ros.org>`__。

如果你的工作中用到了 ROS 2，请参考 :doc:`引用 <Citations>` 来引用 ROS 2。

本文档是由 `Robook <https://robook.org>`__ 维护的 ROS 2 官方文档的非官方中文翻译，所有文档均由人工校对，目的是降低简体中文网络上充斥的大量的机器翻译的低质量文档的影响，使得由于种种原因（暂时）无法使用英语阅读原始资源的用户可以准确有效地获取 ROS 2 的相关信息，而不被低质量信息磨灭探索的热情。

本文档的翻译原则是尽量保持原文的含义，但会根据中文语境和习惯以及发音音节的长短适当地调整表达方式，或者适当地添加译者注释，以便读者更容易理解。同时，为了保证一些专业词汇的具体含义不被翻译扭曲，并不苛求所有的内容都必须被翻译成中文，而将根据翻译者自身对于专业术语的理解、表达习惯和传播范围，适当保留原文或同时列出原文和译文。

同时，我们鼓励读者尽可能使用文档被编写的语言直接阅读文档，降低可能因翻译带来的潜在的误解或含义的增损。因此，我们也在 `术语表 <Glossary>` 提供了一个翻译对照表，以便读者可以更好地理解原文和译文之间的关系。

开始
----

* :doc:`安装 <Installation>`

  - 首次配置 ROS 2 的方法

* :doc:`教程 <Tutorials>`

  - 新用户最佳入门处！
  - 帮你构建必要技能的一系列示例项目

* :doc:`指南 <How-To-Guides>`

  - 无需通读 :doc:`教程 <Tutorials>` 即可快速解答你的“我该怎么做到...？”之类的问题

* :doc:`概念 <Concepts>`

  - 阐明在 :doc:`教程 <Tutorials>` 中所涉及的ROS 2 核心概念

* :doc:`联系我们 <Contact>`

  - 找到问题的答案或用来讨论问题的论坛

ROS 2 项目
------------

如果您对 ROS 2 项目的进展感兴趣：

* :doc:`贡献 <The-ROS2-Project/Contributing>`

  - 贡献到 ROS 2 的最佳实践和方法，以及迁移现有 ROS 1 内容到 ROS 2 的方法

* :doc:`发行版 <Releases>`

  - 从过去到现在所有的 ROS 2 发行版本

* :doc:`Features Status <The-ROS2-Project/Features>`

  - Features in the current release

* :doc:`Feature Ideas <The-ROS2-Project/Feature-Ideas>`

  - Ideas for nice-to-have features that are not under active development

* :doc:`Roadmap <The-ROS2-Project/Roadmap>`

  - Planned work for ROS 2 development

* :doc:`ROSCon Talks <The-ROS2-Project/ROSCon-Content>`

  - Presentations by the community on ROS 2

* :doc:`Project Governance <The-ROS2-Project/Governance>`

  - Information about the ROS Technical Steering Committee, Working Groups, and upcoming events

* :doc:`Marketing <The-ROS2-Project/Marketing>`

  - Downloadable marketing materials
  - `Information about the ROS trademark <https://www.ros.org/blog/media/>`__

ROS 社区资源
-----------------------

如果您需要帮助、有想法或想为项目做出贡献，请访问我们的 ROS 社区资源。

* `Official ROS Discord Channel for discussion and support <https://discord.com/servers/open-robotics-1077825543698927656>`__ (ROS 1, ROS 2)

* `Robotics Stack Exchange - community Q&A website <https://robotics.stackexchange.com/>`__ (ROS 1, ROS 2)

  - See :ref:`Contact Page <Using Robotics Stack Exchange>` for more information

* `ROS Discourse <https://discourse.ros.org/>`__ (ROS 1, ROS 2)

  - Forum for general discussions and announcements for the ROS community
  - See the :ref:`Contact Page <Using ROS Discourse>` for more information

* `ROS Index <https://index.ros.org/>`__ (ROS 1, ROS 2)

  - Indexed list of all packages (i.e. `Python Package Index (PyPI) <https://pypi.org/>`_ for ROS packages)
  - See which ROS distributions a package supports
  - Link to a package's repository, API documentation, or website
  - Inspect a package's license, build type, maintainers, status, and dependencies
  - Get more info for a package on `Robotics Stack Exchange <https://robotics.stackexchange.com/>`__

* `ROS resource status page <https://status.openrobotics.org/>`__ (ROS 1, ROS 2)

  - Check the current status of ROS resources like Discourse or the ROS build farm.

通用 ROS 项目资源
-----------------------------

* `ROS Enhancement Proposals (REPs) <https://ros.org/reps/rep-0000.html>`__ (ROS 1, ROS 2)

  - Proposals for new designs and conventions

* `ROS Robots <https://robots.ros.org/>`__ (ROS 1, ROS 2)

  - Showcases robots projects from the community
  - Instructions on how to contribute a robot

* `ROS Wiki <http://wiki.ros.org/>`__ (ROS 1)

  - ROS 1 documentation and user modifiable content
  - Active until at least the last ROS 1 distribution is EOL

* `ROS.org <https://www.ros.org/>`__ (ROS 1, ROS 2)

  - ROS 1 and ROS 2 product landing page, with high-level description of ROS and links to other ROS sites

Events
------

* `Official ROS Vimeo Channel <https://vimeo.com/osrfoundation>`__ (ROS 1, ROS 2)

  - Videos of ROSCon Talks, community and working group meetings, and project demos.

* `ROSCon website <https://roscon.ros.org/>`__ (ROS 1, ROS 2)

  - ROSCon is our annual ROS developer conference.
  - This page also lists regional ROS events like ROSConJP and ROSConFr.

* `Open Source Robotics Foundation official events calendar <https://calendar.google.com/calendar/u/0/embed?src=agf3kajirket8khktupm9go748@group.calendar.google.com&ctz=America/Los_Angeles>`__

  - This calendar is for official OSRF Events and working group meetings.
  - `Submit your events here <https://bit.ly/OSRFCalendarForm>`__.

* `Open Source Robotics Foundation community calendar <https://calendar.google.com/calendar/embed?src=c_3fc5c4d6ece9d80d49f136c1dcd54d7f44e1acefdbe87228c92ff268e85e2ea0%40group.calendar.google.com&ctz=America%2FLos_Angeles>`__

  - This calendar is for unofficial ROS community events.
  - `Submit your events here <https://bit.ly/OSRFCommunityCalendar>`__ .

Miscellaneous
-------------
* `Purchase official ROS swag <https://spring.ros.org/>`__

* ROS on social media

  - `@OpenRoboticsOrg <https://twitter.com/OpenRoboticsOrg>`__ and `@ROSOrg <https://twitter.com/ROSOrg>`__ on Twitter
  - `Open Robotics on LinkedIn <https://www.linkedin.com/company/open-source-robotics-foundation>`__

* Visit the `Open Source Robotics Foundation website <https://www.openrobotics.org/>`__

  - Tax deductible charitable donations to the Open Source Robotics Foundation can be sent via `DonorBox. <https://donorbox.org/support-open-robotics?utm_medium=qrcode&utm_source=qrcode>`__

已弃用资源
----------
* `ROS Answers <https://answers.ros.org/questions/>`__ (ROS 1, ROS 2)

  - ROS Answers was the ROS community Q&A website, until August, 2023. ROS Answers is currently available as a read-only resource.

* `ROS 2 Docs <https://docs.ros2.org>`_

  - API documentation up to and including Galactic

* `ROS 2 Design <http://design.ros2.org/>`__

  - Early design decisions behind ROS 2 development
  - New design proposals should be submitted via `ROS Enhancement Proposals (REPs) <https://ros.org/reps/rep-0000.html>`__
