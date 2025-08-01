#ifndef WIDROCKER_H
#define WIDROCKER_H

#include <QDialog>
#include <QPoint>
#include <QPainter>
#include <QPaintEvent>
#include <QMouseEvent>
#include <QtMath> // 用于数学计算
#include "ros_cmd_vel/BtnControl.h"

namespace Ui {
class widrocker;
}

class widrocker : public QDialog
{
    Q_OBJECT

public:
    explicit widrocker(BtnController *btnController, QWidget *parent = nullptr);
    ~widrocker();

signals:
    /**
     * @brief 当摇杆移动时发射此信号。
     * @param x 归一化的X轴位置，范围从 -1.0 (最左) 到 1.0 (最右)。
     * @param y 归一化的Y轴位置，范围从 -1.0 (最下) 到 1.0 (最上)。
     *          注意：Y轴已经过转换，向上为正，符合常规坐标系。
     */
    // void rockerMoved(double x, double y);

    /**
     * @brief 当鼠标释放，摇杆复位时发射此信号。
     */
    // void rockerReleased();

protected:
    // 重写QWidget/QDialog的事件处理函数
    void paintEvent(QPaintEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;

private:
    void Init(); // 初始化函数

    // 摇杆参数
    static const int BIG_CIRCLE_RADIUS = 260;
    static const int SMALL_CIRCLE_RADIUS = 80;

    // 摇杆状态
    QPoint BigCir_xy;     // 大圆（背景）的中心坐标
    QPoint SmallCir_xy;   // 小圆（手柄）的中心坐标
    bool MousePressFlag;  // 鼠标按下的标志位

    // 图像资源
    QPixmap big_circle_pixmap_;
    QPixmap small_circle_pixmap_;

private slots:
    void on_BtnBack_clicked();

private:
    Ui::widrocker *ui;
    BtnController *btnController_; // 引用底层按钮控制器
};

#endif // WIDROCKER_H
