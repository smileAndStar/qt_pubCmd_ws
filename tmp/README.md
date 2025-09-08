# Qt StackedWidget 组件集合

## 概述
这是一个完整的 Qt 组件集合，专门设计用于 QStackedWidget 的页面管理，包含摇杆控制和健康监测功能，提供现代化的用户体验。

## 文件结构
```
tmp/
├── rockerwidget.h                       # 摇杆组件头文件
├── rockerwidget.cpp                     # 摇杆组件实现
├── healthmonitorwidget.h                # 健康监测组件头文件  
├── healthmonitorwidget.cpp              # 健康监测组件实现
├── stackedwidget_example.cpp            # 摇杆控制使用示例（旧版本）
├── stackedwidget_example_updated.cpp    # 页面状态控制完整示例（推荐）
├── health_monitor_example.cpp           # 健康监测使用示例
└── README.md                           # 本文件
```

## 组件特性

### 1. RockerWidget (摇杆控制组件)
- **功能**：虚拟摇杆控制机器人移动
- **基础**：继承自 QWidget，完全适配 QStackedWidget
- **特性**：
  - 平滑的摇杆操作体验
  - 实时坐标反馈和状态显示
  - 自动边界限制和复位
  - 与 BtnController 完美集成
  - **页面状态智能检测**：自动检测当前页面是否激活
  - **自动功能控制**：非当前页面时自动禁用摇杆功能
  - **无需返回按钮**：通过页面切换自动管理功能状态
  - 现代化UI设计

### 2. HealthMonitorWidget (健康监测组件)
- **功能**：实时健康数据监测和显示
- **基础**：继承自 QWidget，完全适配 QStackedWidget  
- **特性**：
  - 实时显示心率、血氧、SDNN等指标
  - 可视化的进度条和状态指示器
  - ROS消息自动发布到 `/person_data` 话题
  - 连接状态监控和异常处理
  - 线程安全的UI更新
  - 现代化的仪表盘界面

## 快速集成指南

### 步骤 1: 添加文件到项目
```bash
# 复制组件文件到你的项目
cp rockerwidget.h rockerwidget.cpp /path/to/your/project/include/
cp healthmonitorwidget.h healthmonitorwidget.cpp /path/to/your/project/include/
```

### 步骤 2: 更新 CMakeLists.txt
```cmake
# 添加 Qt5 支持
find_package(Qt5 REQUIRED COMPONENTS Widgets)

# 添加源文件
set(SOURCES
    ${SOURCES}
    src/rockerwidget.cpp
    src/healthmonitorwidget.cpp
)

# 添加头文件的 MOC 处理
set(MOC_HEADERS
    include/rockerwidget.h
    include/healthmonitorwidget.h
)

qt5_wrap_cpp(MOC_SOURCES ${MOC_HEADERS})

# 创建可执行文件
add_executable(${PROJECT_NAME}_node
    ${SOURCES}
    ${MOC_SOURCES}
)

# 链接库
target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    Qt5::Widgets
    health_monitor_cpp
    health_monitor_c
)
```

### 步骤 3: 在主窗口中集成
```cpp
#include "rockerwidget.h"
#include "healthmonitorwidget.h"

class MainWidget : public QWidget {
private:
    QStackedWidget *stackedWidget_;
    RockerWidget *rockerWidget_;
    HealthMonitorWidget *healthWidget_;
    BtnController *btnController_;
    ros::NodeHandle &nh_;

public:
    void setupPages() {
        stackedWidget_ = new QStackedWidget(this);
        
        // 创建摇杆页面
        rockerWidget_ = new RockerWidget(btnController_, this);
        
        // ⭐ 重要：设置页面状态检测信息
        rockerWidget_->setStackedWidgetInfo(stackedWidget_, 1); // 假设摇杆页面索引为1
        
        stackedWidget_->addWidget(rockerWidget_);
        
        // 创建健康监测页面
        healthWidget_ = new HealthMonitorWidget(nh_, this);
        stackedWidget_->addWidget(healthWidget_);
        
        // 连接返回信号（健康监测组件仍需要）
        connect(healthWidget_, &HealthMonitorWidget::backRequested, [=]() {
            stackedWidget_->setCurrentIndex(0); // 返回主页
        });
    }
    
    // 页面切换函数（模拟 leftMainClick）
    void switchToRockerControl() {
        stackedWidget_->setCurrentIndex(1); // 摇杆页面会自动激活
    }
    
    void switchToKeyControl() {
        stackedWidget_->setCurrentIndex(0); // 摇杆页面会自动停止
    }
};
```

## 信号与槽系统

### RockerWidget 信号
```cpp
// 摇杆控制（无页面导航信号）
void rockerMoved(double x, double y);          // 摇杆移动事件
void rockerReleased();                         // 摇杆释放事件
```

**注意**：新版本的 RockerWidget 不再提供 `backRequested()` 信号，因为它通过页面状态自动管理功能。

### HealthMonitorWidget 信号  
```cpp
// 页面导航
void backRequested();                          // 请求返回上一页

// 健康数据
void healthDataUpdated(int heartRate, int spo2, int sdnn, bool userDetected);
void connectionStatusChanged(bool connected);   // 连接状态变化
void displayUpdate(QString text);              // UI更新信号
```

## 🌟 页面状态智能管理

### 工作原理
RockerWidget 采用了智能的页面状态检测机制：

1. **自动状态检测**：通过定时器每100ms检查当前页面是否为摇杆控制页面
2. **功能自动控制**：
   - 当页面切换到摇杆控制时：自动激活摇杆功能
   - 当页面切换离开时：自动停止机器人移动，重置摇杆状态
3. **线程安全**：所有状态检查和控制都在主线程中进行

### 核心优势
- ✅ **无需返回按钮**：完全通过页面状态管理
- ✅ **自动安全停止**：页面切换时自动停止机器人
- ✅ **用户体验优化**：无需手动操作，系统智能处理
- ✅ **内存效率**：一次创建，持续使用
- ✅ **状态一致性**：确保UI状态与功能状态完全同步

## 使用示例

### 基础页面切换
```cpp
// 实现 leftMainClick 函数（与你的代码完全一致）
void frmMain::leftMainClick() {
    // 获取发送信号的按钮对象
    QAbstractButton *b = (QAbstractButton *)sender();
    QString name = b->text();  // 获取按钮文本作为菜单名称
    
    // 在主菜单按钮组中设置互斥选择（只能选中一个按钮）
    for (int i = 0; i < btnsMain.count(); ++i) {
        QAbstractButton *btn = btnsMain.at(i);
        // 只有当前被点击的按钮设置为选中状态
        btn->setChecked(btn == b);
    }

    // 根据按钮文字切换到对应的页面
    if (name == "按键控制") {
        ui->stackedWidget_main->setCurrentIndex(0);  // 摇杆自动停止
    } else if (name == "摇杆控制") {
        ui->stackedWidget_main->setCurrentIndex(1);  // 摇杆自动激活
    }
    // RockerWidget 会自动检测页面状态变化并相应地启用/禁用功能
}

// 无需额外的停止或启动代码，一切都是自动的！
```

### 处理健康数据
```cpp
connect(healthWidget_, &HealthMonitorWidget::healthDataUpdated,
        this, [](int hr, int spo2, int sdnn, bool detected) {
    qDebug() << "Heart Rate:" << hr << "SpO2:" << spo2;
    
    // 添加健康警报逻辑
    if (hr > 100) {
        QMessageBox::warning(this, "警告", "检测到心率过高！");
    }
    
    if (spo2 < 95) {
        QMessageBox::warning(this, "警告", "检测到血氧过低！");
    }
});
```

### ROS 消息处理
健康监测组件会自动发布数据到 ROS 话题：
```cpp
// 订阅健康数据话题
ros::Subscriber sub = nh.subscribe("/person_data", 10, 
    [](const qt_cmd_vel::personData::ConstPtr& msg) {
        ROS_INFO("Received health data: HR=%d, SpO2=%d", 
                 msg->heartrate, msg->spo2);
    });
```

## 自定义配置

### 摇杆参数调整
在 `rockerwidget.h` 中：
```cpp
static const int BIG_CIRCLE_RADIUS = 200;   // 外圆半径
static const int SMALL_CIRCLE_RADIUS = 60;  // 内圆半径
```

### 状态检测定时器配置
在 `rockerwidget.cpp` 构造函数中：
```cpp
// 状态检查定时器 - 可根据需要调整频率
statusCheckTimer_->start(100); // 每100ms检查一次

// 如果需要更快响应，可以调整为：
// statusCheckTimer_->start(50);  // 每50ms检查一次

// 如果性能优先，可以调整为：
// statusCheckTimer_->start(200); // 每200ms检查一次
```

### 健康监测参数
在 `healthmonitorwidget.cpp` 中：
```cpp
// 修改设备端口
#define DEVICE_PORT "/dev/ttyUSB0"

// 调整心率范围
heartRateBar_->setRange(50, 120);

// 调整血氧范围  
spo2Bar_->setRange(90, 100);
```

### 界面样式定制
```cpp
// 修改按钮样式
backButton_->setStyleSheet(
    "QPushButton {"
    "    background-color: #your_color;"
    "    border-radius: 15px;"
    "    // ... 其他样式"
    "}"
);

// 修改进度条样式
heartRateBar_->setStyleSheet(
    "QProgressBar::chunk {"
    "    background-color: #your_color;"
    "}"
);
```

## 与原有系统的兼容性

### 替换现有的模态对话框
```cpp
// 旧方式（模态对话框）
void Widget::on_BtnRocker_clicked() {
    widrocker *rocker = new widrocker(btnController_, this);
    rocker->setModal(true);
    rocker->exec();
    delete rocker;
}

// 新方式（StackedWidget 页面 + 智能状态管理）
void Widget::on_BtnRocker_clicked() {
    // 简单切换页面，RockerWidget 自动处理所有状态管理
    stackedWidget_->setCurrentWidget(rockerWidget_);
    
    // 无需手动调用：
    // - btnController_->stateRockerControl(); (自动调用)
    // - 停止命令 (页面切换时自动执行)
    // - 状态重置 (自动处理)
}
```

### BtnController 集成保持不变
```cpp
// RockerWidget 会自动调用：
btnController_->stateRockerControl();           // 页面激活时
btnController_->updateRockerState(x, y);        // 摇杆移动时
btnController_->stopMovement();                 // 页面切换时
```

### BtnController 集成保持不变
```cpp
// RockerWidget 会自动调用：
btnController_->moveWithNormalizedInput(x, y);  // 移动时
btnController_->stopMovement();                 // 停止时
```

## 性能优化特性

### 内存管理
- **智能指针使用**：自动管理对象生命周期
- **文本缓存限制**：防止日志文本无限增长
- **一次创建，重复使用**：避免频繁创建/销毁对象

### 线程安全
- **Qt信号槽机制**：确保UI更新在主线程
- **原子变量**：线程安全的状态控制
- **队列连接**：跨线程安全通信

### 用户体验
- **流畅切换**：无阻塞的页面切换
- **实时反馈**：即时的状态更新和提示
- **现代化设计**：美观的界面和动画效果

## 故障排除

### 常见问题及解决方案

1. **健康监测设备连接失败**
   ```cpp
   // 检查设备路径
   #define DEVICE_PORT "/dev/ttyUSB0"  // 确认正确的端口
   
   // 检查权限
   sudo chmod 666 /dev/ttyUSB0
   ```

2. **ROS 消息发布失败**
   ```cpp
   // 确认消息定义文件存在
   #include "qt_cmd_vel/personData.h"
   
   // 检查 CMakeLists.txt 中的消息生成
   add_message_files(FILES personData.msg)
   generate_messages(DEPENDENCIES std_msgs)
   ```

3. **摇杆图片不显示**
   ```cpp
   // 检查资源文件路径
   if (!big_circle_pixmap_.load(":/images/ring.png")) {
       qDebug() << "Failed to load ring.png";
   }
   ```

4. **编译错误**
   ```bash
   # 确保 Qt5 正确安装
   sudo apt-get install qt5-default qtbase5-dev
   
   # 检查 CMakeLists.txt 配置
   find_package(Qt5 REQUIRED COMPONENTS Widgets)
   ```

### 调试技巧
```cpp
// 启用详细日志
qDebug() << "Health widget created";
qDebug() << "Connection status:" << isConnected_;
qDebug() << "Data received:" << heartRate << spo2;

// 使用 ROS 日志
ROS_INFO("Health data published");
ROS_WARN("Connection timeout");
ROS_ERROR("Device connection failed");
```

## 扩展建议

### 功能扩展
1. **数据存储**：添加 SQLite 数据库存储历史数据
2. **云同步**：上传健康数据到云端服务
3. **多用户支持**：用户账户和数据隔离
4. **报告生成**：自动生成健康报告
5. **设备管理**：支持多种健康监测设备

### 界面增强
1. **主题切换**：明暗主题支持
2. **动画效果**：更流畅的过渡动画
3. **响应式布局**：适配不同屏幕尺寸
4. **自定义皮肤**：用户可选的界面主题
5. **多语言支持**：国际化支持

### 技术优化
1. **配置文件**：外部配置文件支持
2. **插件系统**：可扩展的插件架构
3. **性能监控**：实时性能指标显示
4. **错误上报**：自动错误报告系统
5. **在线更新**：应用自动更新机制

---

**项目地址**：[https://github.com/smileAndStar/qt_pubCmd_ws](https://github.com/smileAndStar/qt_pubCmd_ws)
