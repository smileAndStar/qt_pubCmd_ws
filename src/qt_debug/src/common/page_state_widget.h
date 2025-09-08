#ifndef PAGE_STATE_WIDGET_H
#define PAGE_STATE_WIDGET_H

#include <QWidget>
#include <QStackedWidget>
#include <QTimer>
#include <QMap>
#include <QDebug>

/**
 * @brief 页面状态信息结构体
 */
struct PageStateInfo {
    QStackedWidget* stackedWidget;  // 堆叠窗口指针
    int pageIndex;                  // 页面索引
    QString description;            // 页面描述（用于调试）
    
    /**
     * @brief 默认构造函数
     */
    PageStateInfo() : stackedWidget(nullptr), pageIndex(-1) {}
    PageStateInfo(QStackedWidget* widget, int index, const QString& desc = "")
        : stackedWidget(widget), pageIndex(index), description(desc) {}
};

/**
 * @brief 支持多页面状态检测的基类
 * 
 * 继承此类的组件可以监控多个 StackedWidget 的多个页面，
 * 当所有监控的页面都不是当前页面时，会自动停止业务逻辑。
 */
class PageStateWidget : public QWidget
{
    Q_OBJECT

public:
    explicit PageStateWidget(QWidget *parent = nullptr);
    virtual ~PageStateWidget();

    /**
     * @brief 添加页面监控
     * @param pageId 页面唯一标识符
     * @param stackedWidget 要监控的堆叠窗口
     * @param pageIndex 页面在堆叠窗口中的索引
     * @param description 页面描述（可选，用于调试）
     */
    void addPageMonitor(const QString& pageId, QStackedWidget* stackedWidget, 
                       int pageIndex, const QString& description = "");

    /**
     * @brief 移除页面监控
     * @param pageId 页面标识符
     */
    void removePageMonitor(const QString& pageId);

    /**
     * @brief 检查指定页面是否为当前活跃页面
     * @param pageId 页面标识符
     * @return true 如果是当前页面
     */
    bool isPageActive(const QString& pageId) const;

    /**
     * @brief 检查是否有任何监控的页面处于活跃状态
     * @return true 如果有任何页面是当前页面
     */
    bool isAnyPageActive() const;

    /**
     * @brief 获取所有活跃页面的ID列表
     * @return 活跃页面ID列表
     */
    QStringList getActivePages() const;

    /**
     * @brief 设置状态检查间隔
     * @param msec 检查间隔（毫秒），默认100ms
     */
    void setCheckInterval(int msec);

signals:
    /**
     * @brief 当有页面变为活跃状态时发射
     * @param pageId 页面ID
     */
    void pageActivated(const QString& pageId);

    /**
     * @brief 当页面失去活跃状态时发射
     * @param pageId 页面ID
     */
    void pageDeactivated(const QString& pageId);

    /**
     * @brief 当组件整体状态改变时发射
     * @param anyActive 是否有任何页面处于活跃状态
     */
    void widgetStateChanged(bool anyActive);

protected:
    /**
     * @brief 当组件变为活跃状态时调用（有页面变为当前页面）
     * 子类重写此方法来启动业务逻辑
     */
    virtual void onWidgetActivated() {}

    /**
     * @brief 当组件变为非活跃状态时调用（所有页面都不是当前页面）
     * 子类重写此方法来停止业务逻辑
     */
    virtual void onWidgetDeactivated() {}

    /**
     * @brief 当特定页面状态改变时调用
     * @param pageId 页面ID
     * @param isActive 是否变为活跃
     */
    virtual void onPageStateChanged(const QString& pageId, bool isActive) {}

private slots:
    void checkPageStates();

private:
    QMap<QString, PageStateInfo> pageMonitors_;  // 页面监控信息
    QTimer* statusCheckTimer_;                   // 状态检查定时器
    bool lastWidgetState_;                       // 上次组件整体状态
};

#endif // PAGE_STATE_WIDGET_H
