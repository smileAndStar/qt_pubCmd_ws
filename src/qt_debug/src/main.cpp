#include "frmmain.h"
#include "appinit.h"
#include "qthelper.h"
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    // // 中文
    // setlocale(LC_ALL, "");
    // // 初始化 ROS 节点
    // ros::init(argc, argv, "qt_cmd_vel_node");
    // // 创建 ROS 节点句柄
    // ros::NodeHandle nh;
    // // 延时确保节点初始化完成
    // ros::Duration(0.1).sleep();

    // 创建 QT 界面实例
    QtHelper::initMain();
    QApplication a(argc, argv);
    AppInit::Instance()->start();
    QtHelper::setFont();
    QtHelper::setCode();

    // 主窗体
    // frmMain w(nh);  // 传入 ROS 节点句柄
    frmMain w;
    QtHelper::setFormInCenter(&w);
    w.show();
    return a.exec();    // exec() 进入 Qt 事件循环
    // 事件循环会处理用户输入、定时器事件等，直到应用程序退出。
    // exec() 会阻塞，直到应用程序退出。
    // 在 Qt 中，exec() 是事件循环的入口点。
}
