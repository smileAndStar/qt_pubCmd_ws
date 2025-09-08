# Common 组件库

## 概述
这个目录包含了可复用的通用组件，为整个项目提供基础的功能支持。这些组件设计为高度可扩展和可复用，遵循面向对象设计原则。

## 文件结构
```
common/
├── page_state_widget.h          # 页面状态管理基类头文件
├── page_state_widget.cpp        # 页面状态管理基类实现
└── README.md                    # 本文件
```

## 组件说明

### PageStateWidget - 页面状态管理基类

#### 功能概述
PageStateWidget 是一个抽象基类，为需要根据页面状态控制业务逻辑的组件提供统一的状态管理能力。它解决了在复杂Qt界面中，多个 StackedWidget 页面状态检测和业务逻辑控制的问题。

#### 核心特性
- **多页面监控**：支持同时监控多个 StackedWidget 的不同页面
- **自动状态管理**：页面状态变化时自动调用相应的虚函数
- **高度可扩展**：通过继承和重写虚函数实现自定义业务逻辑
- **统一接口**：为所有页面相关组件提供一致的API
- **调试友好**：详细的日志输出，便于问题定位

#### 设计模式
- **模板方法模式**：定义状态检测算法骨架，具体实现由子类完成
- **观察者模式**：监控页面状态变化并通知相关组件
- **策略模式**：不同的页面状态对应不同的业务处理策略

#### 类结构

```cpp
class PageStateWidget : public QWidget
{
    Q_OBJECT

public:
    // 构造和析构
    explicit PageStateWidget(QWidget *parent = nullptr);
    virtual ~PageStateWidget();

    // 页面监控管理
    void addPageMonitor(const QString& pageId, QStackedWidget* stackedWidget, 
                       int pageIndex, const QString& description = "");
    void removePageMonitor(const QString& pageId);

    // 状态查询
    bool isPageActive(const QString& pageId) const;
    bool isAnyPageActive() const;
    QStringList getActivePages() const;

    // 配置
    void setCheckInterval(int msec);

signals:
    void pageActivated(const QString& pageId);
    void pageDeactivated(const QString& pageId);
    void widgetStateChanged(bool anyActive);

protected:
    // 虚函数 - 子类重写实现具体业务逻辑
    virtual void onWidgetActivated() {}
    virtual void onWidgetDeactivated() {}
    virtual void onPageStateChanged(const QString& pageId, bool isActive) {}

private slots:
    void checkPageStates();

private:
    QMap<QString, PageStateInfo> pageMonitors_;
    QTimer* statusCheckTimer_;
    bool lastWidgetState_;
};
```

#### 使用示例

##### 1. 继承基类
```cpp
class RockerWidget : public PageStateWidget
{
    Q_OBJECT

protected:
    void onWidgetActivated() override {
        // 启动摇杆控制业务逻辑
        rockerActive_ = true;
        startRockerControl();
    }

    void onWidgetDeactivated() override {
        // 停止摇杆控制业务逻辑
        rockerActive_ = false;
        stopRockerControl();
    }

    void onPageStateChanged(const QString& pageId, bool isActive) override {
        // 处理特定页面状态变化
        qDebug() << "页面" << pageId << (isActive ? "激活" : "失活");
    }
};
```

##### 2. 配置页面监控
```cpp
// 在构造函数或初始化函数中
void MainWindow::setupRockerWidget() {
    rockerWidget_ = new RockerWidget(this);
    
    // 添加多个页面监控
    rockerWidget_->addPageMonitor("main_control", mainStackedWidget_, 1, "主控制页面");
    rockerWidget_->addPageMonitor("side_control", sideStackedWidget_, 0, "侧边控制页面");
    rockerWidget_->addPageMonitor("debug_control", debugStackedWidget_, 2, "调试控制页面");
    
    // 只要上述任一页面是当前页面，摇杆就会激活
}
```

##### 3. 页面切换
```cpp
void MainWindow::switchToControlPage() {
    // 简单的页面切换
    mainStackedWidget_->setCurrentIndex(1);
    
    // RockerWidget 会自动检测到页面变化
    // 如果之前没有监控页面是活跃的，现在 onWidgetActivated() 会被调用
    // 如果之前有监控页面是活跃的，状态保持不变
}
```

#### 高级用法

##### 动态页面监控
```cpp
class DynamicControlWidget : public PageStateWidget
{
public:
    void addControlPage(const QString& name, QStackedWidget* widget, int index) {
        QString pageId = QString("control_%1").arg(name);
        addPageMonitor(pageId, widget, index, QString("%1控制页面").arg(name));
    }
    
    void removeControlPage(const QString& name) {
        QString pageId = QString("control_%1").arg(name);
        removePageMonitor(pageId);
    }
};
```

##### 条件激活
```cpp
class ConditionalWidget : public PageStateWidget
{
protected:
    void onPageStateChanged(const QString& pageId, bool isActive) override {
        if (pageId == "critical_page" && isActive) {
            // 只有关键页面激活时才启动特殊业务逻辑
            startCriticalBusinessLogic();
        }
        
        // 检查是否有足够的页面激活
        QStringList activePages = getActivePages();
        if (activePages.size() >= 2) {
            // 至少有2个页面激活时启动协同业务逻辑
            startCollaborativeLogic();
        }
    }
};
```

##### 性能优化
```cpp
void setupHighFrequencyWidget() {
    // 对于需要高响应性的组件，可以调整检查频率
    highFrequencyWidget_->setCheckInterval(50);  // 50ms检查一次
    
    // 对于不那么重要的组件，可以降低检查频率
    backgroundWidget_->setCheckInterval(500);    // 500ms检查一次
}
```

#### 应用场景

1. **控制组件**：摇杆控制、按键控制等需要根据页面状态启停的组件
2. **监控组件**：健康监测、系统状态监控等需要在特定页面运行的组件
3. **通信组件**：网络通信、串口通信等需要根据业务页面控制的组件
4. **数据采集**：传感器数据采集、日志记录等与页面相关的功能
5. **资源管理**：摄像头、音频设备等需要独占访问的资源管理

#### 设计原则

- **单一职责**：每个继承类只负责一种业务逻辑
- **开闭原则**：对扩展开放，对修改封闭
- **里氏替换**：所有子类都可以替换基类使用
- **接口隔离**：提供最小必要的接口
- **依赖倒置**：依赖抽象而不是具体实现

#### 最佳实践

1. **合理使用监控**：只监控真正需要的页面，避免过度监控
2. **及时清理**：在组件销毁时确保正确清理资源
3. **异常处理**：在业务逻辑中添加适当的异常处理
4. **日志记录**：利用基类提供的调试信息进行问题定位
5. **性能考虑**：根据业务需求调整状态检查频率

## 扩展开发

### 添加新的通用组件

1. **创建组件**：在 common 目录下创建新的头文件和实现文件
2. **遵循规范**：使用一致的命名规范和代码风格
3. **文档完善**：添加详细的类和方法文档
4. **单元测试**：为新组件编写单元测试
5. **更新文档**：在本 README 中添加新组件的说明

### 组件设计指导

1. **高内聚低耦合**：确保组件功能集中，依赖最小化
2. **可配置性**：提供必要的配置选项和参数
3. **可扩展性**：通过继承或组合支持功能扩展
4. **线程安全**：考虑多线程环境下的安全性
5. **资源管理**：确保正确的资源获取和释放

### 版本管理

- **向后兼容**：新版本应该保持对旧版本的兼容性
- **接口稳定**：避免频繁修改公共接口
- **渐进改进**：通过小步迭代的方式改进组件
- **文档同步**：确保文档与代码同步更新

## 故障排除

### 常见问题

1. **页面状态检测不准确**
   - 检查 StackedWidget 指针是否有效
   - 确认页面索引是否正确
   - 查看调试日志确认状态变化

2. **业务逻辑未正确启停**
   - 确认重写的虚函数是否正确调用
   - 检查页面监控配置是否正确
   - 验证 isAnyPageActive() 返回值

3. **性能问题**
   - 调整状态检查间隔
   - 减少不必要的页面监控
   - 优化业务逻辑的启停代码

### 调试技巧

```cpp
// 启用详细日志
qDebug() << "当前活跃页面:" << widget->getActivePages();
qDebug() << "组件状态:" << widget->isAnyPageActive();

// 监控状态变化
connect(widget, &PageStateWidget::widgetStateChanged, 
        [](bool active) {
    qDebug() << "组件状态变化:" << (active ? "激活" : "失活");
});
```

## 许可证

本组件库遵循项目的开源许可证，可自由使用、修改和分发。

---

**项目地址**：[https://github.com/smileAndStar/qt_pubCmd_ws](https://github.com/smileAndStar/qt_pubCmd_ws)
