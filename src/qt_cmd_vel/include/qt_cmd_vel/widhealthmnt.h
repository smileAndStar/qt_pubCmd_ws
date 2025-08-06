#ifndef WIDHEALTHMNT_H
#define WIDHEALTHMNT_H

#include <QDialog>
#include <QString>
#include "healthMnt.h"
#include <ros/ros.h>
#include "qt_cmd_vel/personData.h" // 自定义消息头文件

#define DEVICE_PORT "/dev/ttyUSB0" // 设备端口

namespace Ui {
class widhealthMnt;
}

class widhealthMnt : public QDialog
{
    Q_OBJECT

public:
    explicit widhealthMnt(ros::NodeHandle &nh, QWidget *parent = nullptr);
    ~widhealthMnt();

private slots:
    void on_BtnBack2_clicked();
    void updateDisplay(QString text); // 更新显示的槽函数

signals:
    // 接收到信号后在主线程中更新UI
    void displayUpdate(QString text); // 用于线程间通信的信号

private:
    Ui::widhealthMnt *ui;

    std::atomic<bool> keep_running{true};    // 原子变量，表示是否正在采集数据
    HealthMonitor* monitor; // 健康监测对象指针
    QString outputText; // 用于存储输出文本

    ros::Publisher person_data_pub;       // 发布者对象，用于发布自定义的人体传感消息
};

#endif // WIDHEALTHMNT_H
