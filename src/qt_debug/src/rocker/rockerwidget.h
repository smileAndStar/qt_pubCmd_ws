#ifndef ROCKERWIDGET_H
#define ROCKERWIDGET_H

#include <QLabel>
#include <QVBoxLayout>
#include "page_state_widget.h"
#include "rockercontrol.h"
#include <ros/ros.h>

/**
 * @class RockerWidget
 * @brief 摇杆界面管理器 (在frmman.ui主窗口中没有创建实例，手动创建)
 * 这个类的主要作用是创建和管理整个摇杆控制界面的布局
 * 继承自 PageStateWidget，支持多页面状态检测，
 * 当所有监控的页面都不是当前页面时，会自动停止业务逻辑。
 */
class RockerWidget : public PageStateWidget
{
    Q_OBJECT

public:
    /**
     * @brief 构造函数
     * @param parent 父窗口指针，默认为nullptr
     */
    explicit RockerWidget(ros::NodeHandle &nh_, QWidget *parent = nullptr);
    ~RockerWidget();

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
    // 重写基类虚函数
    /**
     * @brief 当组件变为活跃状态时调用（有页面变为当前页面）
     * 子类重写此方法来启动业务逻辑
     */
    void onWidgetActivated() override;

    /**
     * @brief 当组件变为非活跃状态时调用（所有页面都不是当前页面）
     * 子类重写此方法来停止业务逻辑
     */
    void onWidgetDeactivated() override;

    /**
     * @brief 当特定页面状态改变时调用
     */
    void onPageStateChanged(const QString& pageId, bool isActive) override;

private slots:
    void onRockerMoved(double x, double y);
    void onRockerReleased();

private:
    void setupUI();      // 设置界面布局

    // UI组件
    QLabel *titleLabel_;
    QLabel *statusLabel_;
    QVBoxLayout *mainLayout_;
    RockerControl *rockerControl_;
    
    // 状态管理
    bool rockerActive_;

    // ros 节点句柄
    ros::NodeHandle nh_;
};

#endif // ROCKERWIDGET_H
