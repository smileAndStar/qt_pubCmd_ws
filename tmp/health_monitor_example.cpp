// HealthMonitorWidget 在 QStackedWidget 中的使用示例

#include "healthmonitorwidget.h"
#include "rockerwidget.h"  // 如果同时使用摇杆控制
#include <QApplication>
#include <QMainWindow>
#include <QStackedWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QPushButton>
#include <QWidget>
#include <QLabel>
#include <ros/ros.h>
#include "ros_cmd_vel/BtnControl.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(ros::NodeHandle &nh, QWidget *parent = nullptr) 
        : QMainWindow(parent), nh_(nh)
    {
        setupUI();
        connectSignals();
    }

    ~MainWindow() 
    {
        if (btnController_) {
            delete btnController_;
        }
    }

private slots:
    void showHealthMonitorPage()
    {
        stackedWidget_->setCurrentWidget(healthMonitorWidget_);
    }

    void showRockerPage()
    {
        stackedWidget_->setCurrentWidget(rockerWidget_);
    }

    void showMainPage()
    {
        stackedWidget_->setCurrentWidget(mainPage_);
    }

    void onHealthDataReceived(int heartRate, int spo2, int sdnn, bool userDetected)
    {
        // 处理接收到的健康数据
        qDebug() << "Health Data - HR:" << heartRate << "SpO2:" << spo2 
                 << "SDNN:" << sdnn << "User:" << userDetected;
        
        // 可以在这里添加数据记录、警报等逻辑
        if (heartRate > 100) {
            qDebug() << "Warning: High heart rate detected!";
        }
        
        if (spo2 < 95) {
            qDebug() << "Warning: Low SpO2 detected!";
        }
    }

private:
    void setupUI()
    {
        // 创建中央widget和主布局
        QWidget *centralWidget = new QWidget(this);
        setCentralWidget(centralWidget);

        // 创建 StackedWidget
        stackedWidget_ = new QStackedWidget(centralWidget);

        // 创建各个页面
        createMainPage();
        createHealthMonitorPage();
        createRockerPage();

        // 设置布局
        QVBoxLayout *layout = new QVBoxLayout(centralWidget);
        layout->addWidget(stackedWidget_);

        // 默认显示主页面
        stackedWidget_->setCurrentWidget(mainPage_);

        // 设置窗口属性
        setWindowTitle("多功能控制中心");
        resize(800, 600);
    }

    void createMainPage()
    {
        mainPage_ = new QWidget();
        
        QVBoxLayout *layout = new QVBoxLayout(mainPage_);
        
        QLabel *title = new QLabel("多功能控制中心", mainPage_);
        title->setAlignment(Qt::AlignCenter);
        title->setStyleSheet(
            "QLabel {"
            "    font-size: 28px;"
            "    font-weight: bold;"
            "    margin: 30px;"
            "    color: #2c3e50;"
            "}"
        );
        
        // 创建按钮网格布局
        QGridLayout *buttonLayout = new QGridLayout();
        
        // 健康监测按钮
        healthButton_ = new QPushButton("健康监测", mainPage_);
        styleButton(healthButton_, "#e74c3c", "监测心率、血氧等健康指标");
        
        // 摇杆控制按钮
        rockerButton_ = new QPushButton("摇杆控制", mainPage_);
        styleButton(rockerButton_, "#3498db", "使用摇杆控制机器人移动");
        
        // 设置按钮
        settingsButton_ = new QPushButton("设置", mainPage_);
        styleButton(settingsButton_, "#95a5a6", "系统设置和配置");
        
        // 退出按钮
        exitButton_ = new QPushButton("退出", mainPage_);
        styleButton(exitButton_, "#34495e", "退出应用程序");
        
        // 添加按钮到网格
        buttonLayout->addWidget(healthButton_, 0, 0);
        buttonLayout->addWidget(rockerButton_, 0, 1);
        buttonLayout->addWidget(settingsButton_, 1, 0);
        buttonLayout->addWidget(exitButton_, 1, 1);
        
        // 设置按钮间距
        buttonLayout->setSpacing(20);
        
        layout->addWidget(title);
        layout->addLayout(buttonLayout);
        layout->addStretch();
        
        // 添加到 StackedWidget
        stackedWidget_->addWidget(mainPage_);
    }

    void createHealthMonitorPage()
    {
        // 创建健康监测页面
        healthMonitorWidget_ = new HealthMonitorWidget(nh_);
        
        // 添加到 StackedWidget
        stackedWidget_->addWidget(healthMonitorWidget_);
    }

    void createRockerPage()
    {
        // 创建 BtnController（需要根据实际情况来创建）
        btnController_ = new BtnController(nh_); // 假设 BtnController 构造函数接受 NodeHandle
        
        // 创建摇杆页面
        rockerWidget_ = new RockerWidget(btnController_);
        
        // 添加到 StackedWidget
        stackedWidget_->addWidget(rockerWidget_);
    }

    void styleButton(QPushButton *button, const QString &color, const QString &tooltip)
    {
        button->setFixedSize(200, 100);
        button->setToolTip(tooltip);
        button->setStyleSheet(QString(
            "QPushButton {"
            "    background-color: %1;"
            "    color: white;"
            "    border: none;"
            "    border-radius: 15px;"
            "    font-size: 16px;"
            "    font-weight: bold;"
            "}"
            "QPushButton:hover {"
            "    background-color: %2;"
            "}"
            "QPushButton:pressed {"
            "    background-color: %3;"
            "}"
        ).arg(color)
         .arg(adjustColor(color, -20))  // 悬停时稍微变暗
         .arg(adjustColor(color, -40))  // 按下时更暗
        );
    }

    QString adjustColor(const QString &color, int adjustment)
    {
        // 简单的颜色调整函数，实际应用中可以使用更复杂的颜色处理
        QColor qcolor(color);
        qcolor = qcolor.darker(100 - adjustment);
        return qcolor.name();
    }

    void connectSignals()
    {
        // 连接主页面按钮
        connect(healthButton_, &QPushButton::clicked, 
                this, &MainWindow::showHealthMonitorPage);
        connect(rockerButton_, &QPushButton::clicked, 
                this, &MainWindow::showRockerPage);
        connect(exitButton_, &QPushButton::clicked, 
                this, &QWidget::close);

        // 连接页面返回信号
        connect(healthMonitorWidget_, &HealthMonitorWidget::backRequested, 
                this, &MainWindow::showMainPage);
        connect(rockerWidget_, &RockerWidget::backRequested, 
                this, &MainWindow::showMainPage);

        // 连接健康数据信号
        connect(healthMonitorWidget_, &HealthMonitorWidget::healthDataUpdated, 
                this, &MainWindow::onHealthDataReceived);

        // 可选：连接其他信号
        connect(healthMonitorWidget_, &HealthMonitorWidget::connectionStatusChanged, 
                this, [](bool connected) {
                    qDebug() << "Health monitor connection status:" << connected;
                });

        connect(rockerWidget_, &RockerWidget::rockerMoved, 
                this, [](double x, double y) {
                    qDebug() << "Rocker moved to:" << x << "," << y;
                });
    }

private:
    ros::NodeHandle &nh_;
    QStackedWidget *stackedWidget_;
    
    // 页面widgets
    QWidget *mainPage_;
    HealthMonitorWidget *healthMonitorWidget_;
    RockerWidget *rockerWidget_;
    
    // 主页面按钮
    QPushButton *healthButton_;
    QPushButton *rockerButton_;
    QPushButton *settingsButton_;
    QPushButton *exitButton_;
    
    // 控制器
    BtnController *btnController_;
};

// 完整的使用示例 main 函数
int main(int argc, char *argv[])
{
    // 初始化 ROS
    ros::init(argc, argv, "health_monitor_gui");
    ros::NodeHandle nh;
    
    QApplication app(argc, argv);

    MainWindow window(nh);
    window.show();

    // 在单独线程中运行 ROS spin
    std::thread rosThread([&]() {
        ros::spin();
    });

    int result = app.exec();
    
    // 清理 ROS
    ros::shutdown();
    if (rosThread.joinable()) {
        rosThread.join();
    }
    
    return result;
}

#include "health_monitor_example.moc"

/*
编译配置说明：

在 CMakeLists.txt 中添加：

```cmake
# 如果使用 Qt5
find_package(Qt5 REQUIRED COMPONENTS Widgets)

# 添加源文件
add_executable(health_monitor_gui
    health_monitor_example.cpp
    healthmonitorwidget.cpp
    rockerwidget.cpp
    # ... 其他源文件
)

# 链接库
target_link_libraries(health_monitor_gui
    ${catkin_LIBRARIES}
    Qt5::Widgets
    # ... 其他库
)

# 设置 MOC 处理
qt5_wrap_cpp(MOC_SOURCES
    healthmonitorwidget.h
    rockerwidget.h
)

target_sources(health_monitor_gui PRIVATE ${MOC_SOURCES})
```

使用特性：

1. **统一的多页面管理**：使用 QStackedWidget 管理所有功能页面
2. **现代化UI设计**：美观的界面和流畅的动画效果
3. **实时健康监测**：显示心率、血氧、SDNN等指标
4. **ROS消息发布**：自动发布健康数据到ROS话题
5. **状态指示器**：直观的进度条和状态显示
6. **异常处理**：连接超时、数据超时等异常处理
7. **内存管理**：自动清理长文本，防止内存泄漏
8. **线程安全**：所有UI更新都在主线程中进行
*/
