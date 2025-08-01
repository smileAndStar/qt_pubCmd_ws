# qt_cmd_vel_ws

这是一个基于 ROS 的工作空间，主要功能是通过 Qt 实现一个可视化界面（UI），用于发布 ROS 的 `cmd_vel` 消息，从而实现对移动机器人的速度控制。

## 项目结构

- `src/qt_cmd_vel/`：主要源码目录，包括 Qt UI 实现、ROS 节点、资源文件等。
- `devel/`、`build/`：catkin 构建生成的文件夹。
- `CMakeLists.txt`、`package.xml`：ROS 包配置文件。

## 功能说明

- 使用 Qt 创建了一个界面，可以通过按钮或滑块等控件设置线速度和角速度。
- 点击界面上的控制按钮后，程序会向 ROS 的 `/cmd_vel` 话题发布消息，实现对机器人运动的控制。
- 支持自定义速度参数，界面友好，易于操作。

## 环境依赖

- ROS（建议 Melodic 或 Noetic）
- Qt5（建议 Qt5.9 及以上）
- catkin 工具链

## 编译与运行

1. 安装依赖：
   - ROS
   - Qt5
2. 编译工作空间：
   ```bash
   cd qt_cmd_vel_ws
   catkin_make
   ```
3. 启动 ROS Master：
   ```bash
   roscore
   ```
4. 运行节点：
   ```bash
   rosrun qt_cmd_vel qt_cmd_vel_node
   ```

## 使用方法

- 启动程序后，界面会显示速度控制相关控件。
- 设置所需速度参数，点击按钮，即可向 `/cmd_vel` 发布消息。
- 可用于机器人底盘调试、远程控制等场景。

## 贡献与反馈

如有建议或问题，欢迎提交 Issue 或 PR。

---

**项目地址**：[https://github.com/smileAndStar/qt_pubCmd_ws](https://github.com/smileAndStar/qt_pubCmd_ws)
