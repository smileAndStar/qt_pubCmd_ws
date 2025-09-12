// 后端按钮控制实例化对象

#ifndef BTNWIDGET_H
#define BTNWIDGET_H

#include <QObject>
#include <QPushButton>
#include <QSlider>
#include <ros/ros.h>
#include "page_state_widget.h"
#include "Control.h"

class btncontrol : public QObject
{
    Q_OBJECT

public:
    /**
     * @brief btncontrol 构造函数
     * @param parent 传入ui界面指针
     */
    explicit btncontrol(ros::NodeHandle &nh, PageStateWidget *parent);
    ~btncontrol();

private slots:
    /**
     * @brief 处理左转按钮点击事件。
     * 当用户点击左转按钮时，调用 BtnController 的 startTurningLeft 方法。
     */
    void on_BtnLeft_clicked();

    /**
     * @brief 处理停止按钮点击事件。
     * 当用户点击停止按钮时，调用 BtnController 的 stopMovement 方法。
     */
    void on_BtnStop_clicked();

    /**
     * @brief 处理右转按钮点击事件。
     * 当用户点击右转按钮时，调用 BtnController 的 startTurningRight 方法。
     */
    void on_BtnRight_clicked();

    /**
     * @brief 处理前进按钮点击事件。
     * 当用户点击前进按钮时，调用 BtnController 的 startMovingForward 方法。
     */
    void on_BtnForward_clicked();

    /**
     * @brief 处理后退按钮点击事件。
     * 当用户点击后退按钮时，调用 BtnController 的 startMovingBackward 方法。
     */
    void on_BtnBackward_clicked();

    /**
     * @brief 处理线速度滑块值变化事件。
     * 当用户调整线速度滑块时，调用 BtnController 的 setLinearSpeed 方法。
     * @param value 新的线速度值 范围是 [0, 20], 对应 0.0 到 2.0 m/s。
     */
    void on_LineSpeed_valueChanged(int value);

    /**
     * @brief 处理角速度滑块值变化事件。
     * 当用户调整角速度滑块时，调用 BtnController 的 setAngularSpeed 方法。
     * @param value 新的角速度值 范围是 [0, 100], 对应 0.00 到 1.00 rad/s。
     */
    void on_AngulSpeed_valueChanged(int value);

private:
    PageStateWidget* parentWidget; // 持有父页面指针\

    // UI控件缓存
    // 按键
    QPushButton* btnLeft;   // 左转
    QPushButton* btnStop;   // 停止
    QPushButton* btnRight;  // 右转
    QPushButton* btnForward;    // 前进
    QPushButton* btnBackward;   // 后退
    // 滑块
    QSlider* lineSpeed;       // 线速度
    QSlider* angulSpeed;       // 角速度

    // ros 控制
    ros::NodeHandle nh_; // ROS节点句柄，用于与ROS系统交互
    Controller *btn_controller;  // 按钮控制器对象指针
};

#endif // BTNWIDGET_H