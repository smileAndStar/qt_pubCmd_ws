#ifndef ROCKERWIDGET_H
#define ROCKERWIDGET_H

#include <QLabel>
#include <QVBoxLayout>
#include "../common/page_state_widget.h"
#include "rockercontrol.h"

class RockerWidget : public PageStateWidget
{
    Q_OBJECT

public:
    explicit RockerWidget(QWidget *parent = nullptr);
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
    void onWidgetActivated() override;
    void onWidgetDeactivated() override;
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
};

#endif // ROCKERWIDGET_H
