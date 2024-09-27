.. _Releases:

发行版
=============

什么是发行版?
-----------------------

一个 ROS 发行版是一个版本化的 ROS 包的集合。
就像不同的 Linux 发行版（例如 Ubuntu）一样。
ROS 发行版是为了让开发者能够在相对稳定的代码库上工作，直到他们准备好将所有东西升级/改版。
因此，一旦一个发行版发布，我们会尽量限制对核心包（ros-desktop-full 下的所有内容）的更改，只进行 bug 修复和非破坏性的改进。
这个原则通常适用于整个社区，但对于更上层的包，限制就不那么严格了，所以特定包对应的维护者会负责避免破坏性的更改。

.. _list_of_distributions:

List of Distributions
---------------------

以下是当前和历史 ROS 2 发行版的列表。
表格中绿色的行是当前还受官方支持的发行版。

.. toctree::
   :hidden:

   Releases/Release-Jazzy-Jalisco
   Releases/Release-Iron-Irwini
   Releases/Release-Humble-Hawksbill
   Releases/Release-Rolling-Ridley
   Releases/Development
   Releases/End-of-Life
   Releases/Release-Process

.. raw:: html

   <!--
     This CSS overrides the styles of certain rows to mark them green, indicating they are supported releases.
     For the odd number rows, a line like the following must be used:

       .rst-content table.docutils:not(.field-list) tr:nth-child(1) td {background-color: #33cc66;}

     For the even number rows, a line like the following must be used:

       .rst-content tr:nth-child(2) {background-color: #33cc66;}

     No other combination I've found has worked.  Yes, this is extremely fragile.  No, I don't understand
     why it is like this.
   -->
   <style>
     .rst-content table.docutils:not(.field-list) tr:nth-child(1) td {background-color: #33cc66;}
     .rst-content table.docutils:not(.field-list) tr:nth-child(3) td {background-color: #33cc66;}
     .rst-content tr:nth-child(2) {background-color: #33cc66;}
     .rst-content tr:nth-child(3) {background-color: #33cc66;}
   </style>

.. |rolling| image:: Releases/rolling-small.png
   :alt: Rolling logo

.. |jazzy| image:: Releases/jazzy-small.png
   :alt: Jazzy logo

.. |iron| image:: Releases/iron-small.png
   :alt: Iron logo

.. |humble| image:: Releases/humble-small.png
   :alt: Humble logo

.. |galactic| image:: Releases/galactic-small.png
   :alt: Galactic logo

.. |foxy| image:: Releases/foxy-small.png
   :alt: Foxy logo

.. |eloquent| image:: Releases/eloquent-small.png
   :alt: Eloquent logo

.. |dashing| image:: Releases/dashing-small.png
   :alt: Dashing logo

.. |crystal| image:: Releases/crystal-small.png
   :alt: Crystal logo

.. |bouncy| image:: Releases/bouncy-small.png
   :alt: Bouncy logo

.. |ardent| image:: Releases/ardent-small.png
   :alt: Ardent logo

.. list-table::
   :class: distros
   :header-rows: 1
   :widths: 35 30 20 15

   * - Distro
     - Release date
     - Logo
     - EOL date
   * - :doc:`Jazzy Jalisco <Releases/Release-Jazzy-Jalisco>`
     - May 23rd, 2024
     - |jazzy|
     - May 2029
   * - :doc:`Iron Irwini <Releases/Release-Iron-Irwini>`
     - May 23rd, 2023
     - |iron|
     - November 2024
   * - :doc:`Humble Hawksbill <Releases/Release-Humble-Hawksbill>`
     - May 23rd, 2022
     - |humble|
     - May 2027
   * - :doc:`Galactic Geochelone <Releases/Release-Galactic-Geochelone>`
     - May 23rd, 2021
     - |galactic|
     - December 9th, 2022
   * - :doc:`Foxy Fitzroy <Releases/Release-Foxy-Fitzroy>`
     - June 5th, 2020
     - |foxy|
     - June 20th, 2023
   * - :doc:`Eloquent Elusor <Releases/Release-Eloquent-Elusor>`
     - November 22nd, 2019
     - |eloquent|
     - November 2020
   * - :doc:`Dashing Diademata <Releases/Release-Dashing-Diademata>`
     - May 31st, 2019
     - |dashing|
     - May 2021
   * - :doc:`Crystal Clemmys <Releases/Release-Crystal-Clemmys>`
     - December 14th, 2018
     - |crystal|
     - December 2019
   * - :doc:`Bouncy Bolson <Releases/Release-Bouncy-Bolson>`
     - July 2nd, 2018
     - |bouncy|
     - July 2019
   * - :doc:`Ardent Apalone <Releases/Release-Ardent-Apalone>`
     - December 8th, 2017
     - |ardent|
     - December 2018
   * - :doc:`beta3 <Releases/Beta3-Overview>`
     - September 13th, 2017
     -
     - December 2017
   * - :doc:`beta2 <Releases/Beta2-Overview>`
     - July 5th, 2017
     -
     - September 2017
   * - :doc:`beta1 <Releases/Beta1-Overview>`
     - December 19th, 2016
     -
     - Jul 2017
   * - :doc:`alpha1 - alpha8 <Releases/Alpha-Overview>`
     - August 31th, 2015
     -
     - December 2016

未来发行版
--------------------

有关即将推出的功能的详细信息，请参见 :doc:`roadmap <The-ROS2-Project/Roadmap>`。

以下是 2025 年 5 月 23 日将会发布的新 ROS 2 发行版。

.. list-table::
   :class: future-distros
   :header-rows: 1
   :widths: 35 30 20 15

   * - Distro
     - Release date
     - Logo
     - EOL date
   * - :doc:`Kilted Kaiju <Releases/Release-Kilted-Kaiju>`
     - May 2025
     - TBD
     - Nov 2026


.. _rolling_distribution:

滚动更新(Rolling)版
--------------------

:doc:`ROS 2 Rolling Ridley <Releases/Release-Rolling-Ridley>` 是 ROS 2 的滚动开发发行版。
它在 `REP 2002 <https://www.ros.org/reps/rep-2002.html>`_ 中有详细描述，于 2020 年 6 月首次推出。

ROS 2 的滚动发行版有两个目的：

1. 它是 ROS 2 未来稳定发行版的一个暂存区，
2. 它是最新开发进展的集合。

正如其名称所示，Rolling 是持续更新的，**可能会有包含破坏性更改的快速更新**。
因此我们建议大多数人使用最新的稳定发行版（参见 :ref:`list_of_distributions`）。

Rolling 发行版中的包将自动发布到未来的 ROS 2 稳定发行版中。
这样，开发者可以在 Rolling 发行版中尝试新功能，以确保它们在未来的稳定发行版中能够正常工作。
