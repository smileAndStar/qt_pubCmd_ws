#ifndef ROCKERCONTROL_H
#define ROCKERCONTROL_H

#include <QWidget>
#include <QPoint>
#include <QPixmap>
#include <QPainter>
#include <QPaintEvent>
#include <QMouseEvent>
#include <QtMath>

class RockerControl : public QWidget
{
    Q_OBJECT

public:
    explicit RockerControl(QWidget *parent = nullptr);

signals:
    void rockerMoved(double x, double y);
    void rockerReleased();

protected:
    void paintEvent(QPaintEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void resizeEvent(QResizeEvent *event) override; // 监听尺寸变化

public slots:
    void resetRocker(); // 提供一个公有槽用于重置

private:
    void calculateRockerPosition(const QPoint &mousePos); // 提取计算逻辑
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
};

#endif // ROCKERCONTROL_H
