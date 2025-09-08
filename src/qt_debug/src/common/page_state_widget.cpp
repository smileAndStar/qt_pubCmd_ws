#include "page_state_widget.h"

PageStateWidget::PageStateWidget(QWidget *parent)
    : QWidget(parent), lastWidgetState_(false)
{
    // 创建状态检查定时器
    statusCheckTimer_ = new QTimer(this);
    connect(statusCheckTimer_, &QTimer::timeout, this, &PageStateWidget::checkPageStates);
    statusCheckTimer_->start(100); // 默认每100ms检查一次
}

PageStateWidget::~PageStateWidget()
{
    statusCheckTimer_->stop();
}

void PageStateWidget::addPageMonitor(const QString& pageId, QStackedWidget* stackedWidget, 
                                    int pageIndex, const QString& description)
{
    if (!stackedWidget) {
        qWarning() << "PageStateWidget: 无效的 StackedWidget 指针";
        return;
    }

    if (pageIndex < 0 || pageIndex >= stackedWidget->count()) {
        qWarning() << "PageStateWidget: 无效的页面索引" << pageIndex 
                   << "，StackedWidget 总页面数:" << stackedWidget->count();
        return;
    }

    PageStateInfo info(stackedWidget, pageIndex, description);
    pageMonitors_[pageId] = info;
    
    qDebug() << "PageStateWidget: 添加页面监控 -" 
             << "ID:" << pageId 
             << "索引:" << pageIndex 
             << "描述:" << (description.isEmpty() ? "无" : description);
}

void PageStateWidget::removePageMonitor(const QString& pageId)
{
    if (pageMonitors_.remove(pageId) > 0) {
        qDebug() << "PageStateWidget: 移除页面监控 -" << pageId;
    } else {
        qWarning() << "PageStateWidget: 尝试移除不存在的页面监控 -" << pageId;
    }
}

bool PageStateWidget::isPageActive(const QString& pageId) const
{
    auto it = pageMonitors_.find(pageId);
    if (it == pageMonitors_.end()) {
        return false;
    }

    const PageStateInfo& info = it.value();
    if (!info.stackedWidget) {
        return false;
    }

    return info.stackedWidget->currentIndex() == info.pageIndex;
}

bool PageStateWidget::isAnyPageActive() const
{
    for (auto it = pageMonitors_.begin(); it != pageMonitors_.end(); ++it) {
        if (isPageActive(it.key())) {
            return true;
        }
    }
    return false;
}

QStringList PageStateWidget::getActivePages() const
{
    QStringList activePages;
    for (auto it = pageMonitors_.begin(); it != pageMonitors_.end(); ++it) {
        if (isPageActive(it.key())) {
            activePages.append(it.key());
        }
    }
    return activePages;
}

void PageStateWidget::setCheckInterval(int msec)
{
    if (msec <= 0) {
        qWarning() << "PageStateWidget: 无效的检查间隔" << msec << "ms，使用默认值100ms";
        msec = 100;
    }
    
    statusCheckTimer_->setInterval(msec);
    qDebug() << "PageStateWidget: 设置状态检查间隔为" << msec << "ms";
}

void PageStateWidget::checkPageStates()
{
    static QMap<QString, bool> lastPageStates;
    
    // 检查每个页面的状态变化
    for (auto it = pageMonitors_.begin(); it != pageMonitors_.end(); ++it) {
        const QString& pageId = it.key();
        bool isActive = isPageActive(pageId);
        bool wasActive = lastPageStates.value(pageId, false);
        
        if (isActive != wasActive) {
            // 页面状态发生变化
            lastPageStates[pageId] = isActive;
            
            if (isActive) {
                qDebug() << "PageStateWidget: 页面激活 -" << pageId;
                emit pageActivated(pageId);
            } else {
                qDebug() << "PageStateWidget: 页面失活 -" << pageId;
                emit pageDeactivated(pageId);
            }
            
            // 调用虚函数，让子类处理
            onPageStateChanged(pageId, isActive);
        }
    }
    
    // 检查组件整体状态变化
    bool currentWidgetState = isAnyPageActive();
    if (currentWidgetState != lastWidgetState_) {
        lastWidgetState_ = currentWidgetState;
        
        if (currentWidgetState) {
            qDebug() << "PageStateWidget: 组件激活 -" << metaObject()->className();
            onWidgetActivated();
        } else {
            qDebug() << "PageStateWidget: 组件失活 -" << metaObject()->className();
            onWidgetDeactivated();
        }
        
        emit widgetStateChanged(currentWidgetState);
    }
}
