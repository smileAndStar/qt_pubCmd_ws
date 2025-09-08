## 在 StackedWidget 页面中使用摇杆

假设你有一个 StackedWidget 页面类，在该页面中添加摇杆：

````cpp
#include "rockerwidget.h"

class YourPage : public QWidget
{
    Q_OBJECT

public:
    explicit YourPage(QWidget *parent = nullptr);

private slots:
    void onRockerMoved(double x, double y);
    void onRockerReleased();
    void onPageChanged(); // 当页面切换时调用

private:
    RockerWidget *rockerWidget;
    // ...existing code...
};
````

````cpp
#include "yourpage.h"
#include <QVBoxLayout>
#include <QHBoxLayout>

YourPage::YourPage(QWidget *parent)
    : QWidget(parent)
{
    // 创建摇杆控件
    rockerWidget = new RockerWidget(this);
    
    // 连接信号槽
    connect(rockerWidget, &RockerWidget::rockerMoved, this, &YourPage::onRockerMoved);
    connect(rockerWidget, &RockerWidget::rockerReleased, this, &YourPage::onRockerReleased);
    
    // 布局设置
    QVBoxLayout *mainLayout = new QVBoxLayout(this);
    
    // 添加其他控件...
    
    // 添加摇杆控件（可以根据需要调整位置）
    QHBoxLayout *rockerLayout = new QHBoxLayout();
    rockerLayout->addStretch();
    rockerLayout->addWidget(rockerWidget);
    rockerLayout->addStretch();
    
    mainLayout->addLayout(rockerLayout);
    mainLayout->addStretch();
    
    setLayout(mainLayout);
}

void YourPage::onRockerMoved(double x, double y)
{
    // 只有当前页面可见时才处理摇杆信号
    if (!isVisible())
        return;
        
    qDebug() << "摇杆移动：" << "X:" << x << "Y:" << y;
    // 处理摇杆移动逻辑
}

void YourPage::onRockerReleased()
{
    if (!isVisible())
        return;
        
    qDebug() << "摇杆释放";
    // 处理摇杆释放逻辑
}

void YourPage::onPageChanged()
{
    // 当页面不可见时，重置摇杆状态
    if (!isVisible() && rockerWidget)
    {
        rockerWidget->setEnabled(false);
        // 可以发送停止信号
        emit rockerReleased();
    }
    else if (isVisible() && rockerWidget)
    {
        rockerWidget->setEnabled(true);
    }
}
````

## 在主窗口中管理页面切换

````cpp
// 在你的主窗口或管理 StackedWidget 的地方
void MainWindow::onStackedWidgetCurrentChanged(int index)
{
    // 当切换到包含摇杆的页面时
    if (index == YOUR_ROCKER_PAGE_INDEX)
    {
        yourPage->onPageChanged();
    }
    // 当切换离开包含摇杆的页面时
    else
    {
        // 可以在这里停止任何正在进行的摇杆操作
        yourPage->onPageChanged();
    }
}
````

## 关键优势

1. **页面隔离**：摇杆只在对应页面可见时生效
2. **可复用**：RockerWidget 可以在多个页面中使用
3. **易于管理**：通过 `isVisible()` 和 `setEnabled()` 控制生效状态
4. **信号安全**：在信号处理函数中检查页面可见性，确保只有当前页面处理摇杆事件

这种方案既保持了代码的模块化，又确保了摇杆功能只在正确的页面上生效。