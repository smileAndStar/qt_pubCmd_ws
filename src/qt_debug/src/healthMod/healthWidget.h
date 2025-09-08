#ifndef HEALTHWIDGET_H
#define HEALTHWIDGET_H

#include <QObject>
#include <QString>
#include <QPushButton>
#include <QLabel>
#include <QWidget>
#include <QVector>
#include <QPointF>
#include <QTimer>
#include "healthMnt.h"
#include <ros/ros.h>
#include "page_state_widget.h"
#include "qt_debug/personData.h" // 自定义消息头文件

#define DEVICE_PORT "/dev/ttyUSB0" // 设备端口

class healthMnter : public QObject
{
    Q_OBJECT

public:
    // explicit healthMnter(ros::NodeHandle &nh, PageStateWidget *parent);
    explicit healthMnter(PageStateWidget *parent);
    ~healthMnter();

protected:
    bool eventFilter(QObject *obj, QEvent *event) override;

private slots:
    void onStartClicked();                     // 开始采集按钮点击
    void updateStatusDisplay(QString text);    // 更新状态信息显示（labelState）
    void updateDebugDisplay(QString text);     // 更新调试信息显示（新增：专门的调试信息显示控件）
    void updateHeartRateChart(QVector<QPointF> points); // 更新心率图表
    void updateHealthData(QString healthInfo); // 更新健康数据显示

signals:
    void statusUpdate(QString text);           // 状态信息更新信号（labelState使用）
    void debugUpdate(QString text);           // 调试信息更新信号（新增）
    void heartRateUpdate(QVector<QPointF> points); // 心率数据更新信号
    void healthDataUpdate(QString healthInfo); // 健康数据更新信号

private:
    std::atomic<bool> keep_running{false};     // 原子变量，表示是否正在采集数据
    HealthMonitor* monitor;                    // 健康监测对象指针
    QString outputText;                        // 用于存储输出文本
    
    PageStateWidget* parentWidget;             // 父页面指针
    // ros::Publisher person_data_pub;            // 发布者对象，用于发布自定义的人体传感消息
    
    // 心率绘图相关
    QVector<QPointF> heartRatePoints;          // 心率数据点
    QTimer* chartUpdateTimer;                  // 图表更新定时器
    
    // UI控件缓存
    QPushButton* btnSwitch;
    QLabel* labelState;
    QLabel* labelHealthMsg;
    QWidget* paintWidget;
    
    // 数据处理方法
    void processHealthData(const realtime_packet_t& data);
    void drawHeartRateChart(QPainter& painter, const QRect& rect);
    QString formatHealthMessage(const realtime_packet_t& data);
};

#endif // HEALTHWIDGET_H