#ifndef HEALTHMONITORWIDGET_H
#define HEALTHMONITORWIDGET_H

#include <QWidget>
#include <QString>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QTextEdit>
#include <QGroupBox>
#include <QProgressBar>
#include <QTimer>
#include <atomic>
#include "healthMnt.h"
#include <ros/ros.h>
#include "qt_cmd_vel/personData.h" // 自定义消息头文件

#define DEVICE_PORT "/dev/ttyUSB0" // 设备端口

class HealthMonitorWidget : public QWidget
{
    Q_OBJECT

public:
    explicit HealthMonitorWidget(ros::NodeHandle &nh, QWidget *parent = nullptr);
    ~HealthMonitorWidget();

signals:
    /**
     * @brief 请求返回上一页的信号
     */
    void backRequested();

    /**
     * @brief 用于线程间通信的信号，更新显示数据
     */
    void displayUpdate(QString text);

    /**
     * @brief 健康数据更新信号
     */
    void healthDataUpdated(int heartRate, int spo2, int sdnn, bool userDetected);

    /**
     * @brief 连接状态变化信号
     */
    void connectionStatusChanged(bool connected);

private slots:
    void onBackClicked();
    void updateDisplay(QString text);
    void onConnectionTimeout();
    void onDataTimeout();

private:
    void setupUI();              // 设置界面布局
    void initHealthMonitor();    // 初始化健康监测
    void updateStatusIndicators(int heartRate, int spo2, int sdnn, bool userDetected);
    void showConnectionStatus(bool connected);

    // UI组件
    QPushButton *backButton_;
    QLabel *titleLabel_;
    QLabel *connectionStatusLabel_;
    QTextEdit *dataDisplay_;
    
    // 状态指示器
    QGroupBox *statusGroup_;
    QLabel *heartRateLabel_;
    QLabel *spo2Label_;
    QLabel *sdnnLabel_;
    QLabel *userDetectedLabel_;
    QProgressBar *heartRateBar_;
    QProgressBar *spo2Bar_;
    
    // 布局
    QVBoxLayout *mainLayout_;
    QHBoxLayout *topLayout_;
    QHBoxLayout *statusLayout_;
    QVBoxLayout *indicatorsLayout_;

    // 健康监测相关
    std::atomic<bool> keep_running{true};    // 原子变量，表示是否正在采集数据
    HealthMonitor* monitor_;                 // 健康监测对象指针
    QString outputText_;                     // 用于存储输出文本
    ros::Publisher person_data_pub_;         // 发布者对象，用于发布自定义的人体传感消息
    ros::NodeHandle &nh_;                    // ROS节点句柄引用

    // 定时器
    QTimer *connectionTimer_;
    QTimer *dataTimer_;
    
    // 状态变量
    bool isConnected_;
    bool isDataReceiving_;
    int dataCount_;
};

#endif // HEALTHMONITORWIDGET_H
