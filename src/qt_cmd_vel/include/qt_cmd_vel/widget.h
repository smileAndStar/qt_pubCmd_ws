#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include "ros_cmd_vel/BtnControl.h"

QT_BEGIN_NAMESPACE
namespace Ui { class Widget; }
QT_END_NAMESPACE

class Widget : public QWidget
{
    // Q_OBJECT 宏用于启用 Qt 的元对象系统功能，如信号和槽机制。
    // 告诉 Meta‑Object 编译器（moc）这个类需要生成额外的元信息代码。没有这个宏，moc 会跳过该类，不生成所需功能的支持代码
    Q_OBJECT

public:
    /**
     * @brief Widget 构造函数。
     * @param nh ROS 节点句柄的引用，用于与 ROS 系统交互。
     * @param parent 父窗口指针，默认为 nullptr。
     */
    Widget(ros::NodeHandle &nh, QWidget *parent = nullptr);
    ~Widget();

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

    /**
     * @brief 处理摇杆按钮点击事件。
     * 当用户点击摇杆按钮时，打开rocker控制窗口。
     */
    void on_BtnRocker_clicked();

private:
    Ui::Widget *ui;
    BtnController *btn_controller;
};
#endif // WIDGET_H
