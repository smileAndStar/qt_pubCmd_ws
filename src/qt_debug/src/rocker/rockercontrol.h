#ifndef ROCKERCONTROL_H
#define ROCKERCONTROL_H

#include <QWidget>
#include <QPoint>
#include <QPixmap>
#include <QPainter>
#include <QPaintEvent>
#include <QMouseEvent>
#include <QtMath>
#include "Control.h"
#include <ros/ros.h>

/**
 * @class RockerControl
 * @brief 自定义控件：摇杆控制控件
 * 
 * 该组件实现一个可拖动的摇杆，用户可以通过鼠标操作来控制摇杆的位置。
 * 摇杆的移动范围被限制在一个大圆内，松开鼠标时摇杆会自动回到中心位置。
 * 组件会发出信号通知外部摇杆的位置变化和释放事件。
 * 
 * 此外，控件会根据摇杆的实时变化，发布 ros 的 cmd_vel 话题，用于控制机器人运动。
 */
class RockerControl : public QWidget
{
    Q_OBJECT

public:
    /**
     * @brief 构造函数
     * @param parent 父窗口指针，默认为nullptr
     */
    explicit RockerControl(ros::NodeHandle &nh, QWidget *parent = nullptr);

    /**
     * @brief 析构函数
     * 释放资源
     */
    ~RockerControl() override;

signals:
    void rockerMoved(double x, double y);
    void rockerReleased();

protected:
    /**
     * @brief 重写绘制事件
     * @param event 绘制事件对象
     * 负责绘制摇杆的背景大圆和可移动的小圆（手柄）。
     */
    void paintEvent(QPaintEvent *event) override;

    /**
     * @brief 重写鼠标移动事件
     * @param event 鼠标事件对象
     * 当鼠标移动时，计算摇杆的新位置，并发出 rockerMoved 信号。    
     */
    void mouseMoveEvent(QMouseEvent *event) override;

    /**
     * @brief 重写鼠标按下事件
     * @param event 鼠标事件对象 
     */
    void mousePressEvent(QMouseEvent *event) override;

    /**
     * @brief 重写鼠标释放事件
     * @param event 鼠标事件对象
     * 当鼠标释放时，摇杆自动回到中心位置，并发出 rockerReleased 信号。 
     */
    void mouseReleaseEvent(QMouseEvent *event) override;

    /**
     * @brief 重写尺寸变化事件
     * @param event 尺寸变化事件对象
     */
    void resizeEvent(QResizeEvent *event) override; // 监听尺寸变化

public slots:
    /**
     * @brief 重置摇杆位置
     * 将摇杆位置重置为中心位置，并发出 rockerReleased 信号。
     * @note 这个槽函数可以被外部调用，以便在特定情况下重置摇杆位置。
     */
    void resetRocker(); // 提供一个公有槽用于重置

private:
    /**
     * @brief 计算摇杆位置
     * @param mousePos 当前鼠标位置
     * 根据鼠标位置计算摇杆小圆的位置，并限制在大圆范围内
     * @note 这个函数提取了计算逻辑，便于维护和复用。
     */
    void calculateRockerPosition(const QPoint &mousePos); // 提取计算逻辑

    /**
     * @brief 发射摇杆移动信号
     * 根据当前小圆位置计算归一化坐标，并发出 rockerMoved 信号
     * @note 这个函数提取了发射信号的逻辑，便于维护和复用。
     */
    void emitRockerMovedSignal(); // 提取发射信号逻辑

    // 摇杆参数
    int bigCircleRadius_;
    int smallCircleRadius_;

    // 摇杆状态
    QPoint centerPoint_;     // 摇杆中心点
    QPoint smallCirclePos_;  // 小圆（手柄）的当前坐标
    bool isMousePressed_;    // 鼠标按下的标志位

    // 图像资源
    QPixmap bigCirclePixmap_;
    QPixmap smallCirclePixmap_;

    // ros 消息控制器
    ros::NodeHandle nh_;    // ROS 节点句柄
    Controller *rockerController_;  // 控制器实例
};

#endif // ROCKERCONTROL_H
