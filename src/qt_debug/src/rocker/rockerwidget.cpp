#include "rockerwidget.h"
#include <QDebug>

RockerWidget::RockerWidget(QWidget *parent) :
    PageStateWidget(parent),
    rockerActive_(false)
{
    setupUI();

    // 将 RockerControl 的信号连接到 RockerWidget 的槽函数
    connect(rockerControl_, &RockerControl::rockerMoved, this, &RockerWidget::onRockerMoved);
    connect(rockerControl_, &RockerControl::rockerReleased, this, &RockerWidget::onRockerReleased);
}

RockerWidget::~RockerWidget()
{}

void RockerWidget::setupUI()
{
    // 创建主布局（垂直布局）
    mainLayout_ = new QVBoxLayout(this);

    // 创建标题标签
    titleLabel_ = new QLabel("摇杆控制器", this);
    titleLabel_->setAlignment(Qt::AlignCenter);
    titleLabel_->setStyleSheet("QLabel { font-size: 18px; font-weight: bold; margin: 10px; }");

    // 使用自定义的 RockerControl
    rockerControl_ = new RockerControl(this);
    // 可以给它一个最小尺寸，或者让它自由伸缩
    rockerControl_->setMinimumSize(300, 300);

    // 创建状态标签
    statusLabel_ = new QLabel("移动摇杆控制机器人方向", this);
    statusLabel_->setAlignment(Qt::AlignCenter);
    statusLabel_->setStyleSheet("QLabel { color: #666; font-size: 14px; }");

    // 添加到主布局
    mainLayout_->addWidget(titleLabel_);
    mainLayout_->addWidget(rockerControl_, 1);  // 摇杆区域占主要空间
    mainLayout_->addWidget(statusLabel_);

    // 设置布局边距
    mainLayout_->setContentsMargins(20, 20, 20, 20);
    mainLayout_->setSpacing(15);
}

// 重写基类虚函数：当组件激活时调用
void RockerWidget::onWidgetActivated()
{
    rockerActive_ = true;
    statusLabel_->setText("摇杆控制已激活");
    qDebug() << "RockerWidget: 摇杆控制激活";
}

// 重写基类虚函数：当组件失活时调用
void RockerWidget::onWidgetDeactivated()
{
    rockerActive_ = false;
    statusLabel_->setText("摇杆控制已停止");
    
    // 重置摇杆状态
    if (rockerControl_) {
        rockerControl_->resetRocker();
        emit rockerReleased();
    }
    
    qDebug() << "RockerWidget: 摇杆控制停止";
}

// 重写基类虚函数：当特定页面状态改变时调用
void RockerWidget::onPageStateChanged(const QString& pageId, bool isActive)
{
    if (isActive) {
        qDebug() << "RockerWidget: 页面" << pageId << "激活";
    } else {
        qDebug() << "RockerWidget: 页面" << pageId << "失活";
    }
    
    // 更新状态标签显示当前活跃页面
    QStringList activePages = getActivePages();
    if (!activePages.isEmpty()) {
        statusLabel_->setText(QString("活跃页面: %1").arg(activePages.join(", ")));
    }
}

void RockerWidget::onRockerMoved(double x, double y)
{
    // 只有在组件活跃时才处理摇杆移动
    if (!rockerActive_ || !isAnyPageActive()) {
        qDebug() << "RockerWidget: 摇杆移动被忽略（组件未激活）";
        return;
    }
    
    // 转发信号
    emit rockerMoved(x, y);
    
    // 更新状态显示
    statusLabel_->setText(QString("摇杆位置: X=%1, Y=%2").arg(x, 0, 'f', 2).arg(y, 0, 'f', 2));
}

void RockerWidget::onRockerReleased()
{
    // 只有在组件活跃时才处理摇杆释放
    if (!rockerActive_ || !isAnyPageActive()) {
        qDebug() << "RockerWidget: 摇杆释放被忽略（组件未激活）";
        return;
    }
    
    // 转发信号
    emit rockerReleased();
    
    // 重置状态显示
    statusLabel_->setText("摇杆已释放");
}
