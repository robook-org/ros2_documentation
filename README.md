# ROS 2 Documentation

本仓库维护了 ROS 2 官方文档的中文版本，可以在[https://ros2docs.robook.org](https://ros2docs.robook.org)访问到在线文档.

本仓库的源码会通过 GitHub Action 自动发布至 GitHub Pages。

## Contributing to the documentation

我们很欢迎大家为此仓库提交代码，帮助我们完善 ROS 2 的中文文档。

对官方文档(英文)的贡献可以参考：

Please see the [Contributing to ROS 2 Documentation](https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Contributing-To-ROS-2-Documentation.html) page to learn more.

## Contributing to ROS 2

向 ROS 2 源码项目贡献代码请参考 [ROS 2 contributing guidelines](https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing.html)。

## Prerequisites

如果你想在本地构建文档，请确保你的系统中安装了以下软件：

* make
* graphviz
* python virtualenv


在 virtualenv 中运行：

```
pip install -r requirements.txt -c constraints.txt
```

### Pinned versions

目前的开发环境使用 Jammy 作为构建平台。
所有的 Python 依赖版本都被固定在 constraints 文件中，以确保构建可复现。
如果你想升级系统，请确保一切正常，然后使用 `pip freeze > constraints.txt` 锁定版本以升级。

## Building HTML

### 本地开发测试
对当前分支进行本地测试：

`make html`

`sensible-browser build/html/index.html`

### 部署测试

对多站点版本的构建进行测试：

`make multiversion`

`sensible-browser build/html/rolling/index.html`

**注意：** 这个命令会忽略本地的工作空间变更，并从分支构建。
