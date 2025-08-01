#include "qt_cmd_vel/widget.h"
#include <ros/ros.h>
#include "ros_cmd_vel/BtnControl.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    // 中文
    setlocale(LC_ALL, "");  

    // 初始化 ROS 节点
    ros::init(argc, argv, "qt_cmd_vel_node");
    // 创建 ROS 节点句柄
    ros::NodeHandle nh;

    // 等待 ROS 发布者/订阅者连接
    ros::Duration(1.0).sleep();

    // 创建 Qt 应用程序实例
    QApplication a(argc, argv);
    Widget w(nh);   // 传入 ROS 节点句柄
    w.show();
    return a.exec();
}
