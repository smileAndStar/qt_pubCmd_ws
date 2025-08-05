#ifndef WIDHEALTHMNT_H
#define WIDHEALTHMNT_H

#include <QDialog>
#include <QString>
#include "healthMnt.h"

#define DEVICE_PORT "/dev/ttyUSB0" // 设备端口

namespace Ui {
class widhealthMnt;
}

class widhealthMnt : public QDialog
{
    Q_OBJECT

public:
    explicit widhealthMnt(QWidget *parent = nullptr);
    ~widhealthMnt();

private slots:
    void on_BtnBack2_clicked();

private:
    Ui::widhealthMnt *ui;

    std::atomic<bool> keep_running{true};    // 原子变量，表示是否正在采集数据
    HealthMonitor * global_monitor = nullptr; // 全局健康监测对象指针
    QString outputText; // 用于存储输出文本
};

#endif // WIDHEALTHMNT_H
