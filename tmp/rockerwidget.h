#ifndef ROCKERWIDGET_H
#define ROCKERWIDGET_H

#include <QWidget>
#include <QPoint>
#include <QPainter>
#include <QPaintEvent>
#include <QMouseEvent>
#include <QtMath> // 用于数学计算
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QStackedWidget>
#include <QTimer>
#include "ros_cmd_vel/BtnControl.h"

class RockerWidget : public QWidget
{
    Q_OBJECT

public:
    explicit RockerWidget(BtnController *btnController, QWidget *parent = nullptr);
    ~RockerWidget();

    // 设置关联的 StackedWidget 和页面索引，用于检测当前页面状态
    void setStackedWidgetInfo(QStackedWidget *stackedWidget, int pageIndex);

signals:
    /**
     * @brief 当摇杆移动时发射此信号。
     * @param x 归一化的X轴位置，范围从 -1.0 (最左) 到 1.0 (最右)。
     * @param y 归一化的Y轴位置，范围从 -1.0 (最下) 到 1.0 (最上)。
     *          注意：Y轴已经过转换，向上为正，符合常规坐标系。
     */
    void rockerMoved(double x, double y);

    /**
     * @brief 当鼠标释放，摇杆复位时发射此信号。
     */
    void rockerReleased();

protected:
    // 重写QWidget的事件处理函数
    void paintEvent(QPaintEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;

private slots:
    void checkPageStatus();  // 检查当前页面状态

private:
    void setupUI();      // 设置界面布局
    void initRocker();   // 初始化摇杆参数
    bool isCurrentPage() const;  // 检查是否为当前活跃页面

    // 摇杆参数
    static const int BIG_CIRCLE_RADIUS = 200;   // 调小一些适应页面
    static const int SMALL_CIRCLE_RADIUS = 60;

    // 摇杆状态
    QPoint BigCir_xy;     // 大圆（背景）的中心坐标
    QPoint SmallCir_xy;   // 小圆（手柄）的中心坐标
    bool MousePressFlag;  // 鼠标按下的标志位

    // 图像资源
    QPixmap big_circle_pixmap_;
    QPixmap small_circle_pixmap_;

    // UI组件
    QLabel *titleLabel_;
    QLabel *statusLabel_;
    QVBoxLayout *mainLayout_;
    QWidget *rockerArea_;       // 摇杆绘制区域

    // 页面状态检测
    QStackedWidget *stackedWidget_;  // 关联的堆叠窗口
    int pageIndex_;                  // 当前页面在堆叠窗口中的索引
    QTimer *statusCheckTimer_;       // 用于定期检查页面状态的定时器

    BtnController *btnController_; // 引用底层按钮控制器
};

#endif // ROCKERWIDGET_H
