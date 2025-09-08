/**
 * @file stackedwidget_example_updated.cpp
 * @brief 更新的 RockerWidget 使用示例 - 通过页面状态控制功能
 * 
 * 这个示例展示了如何将新版本的 RockerWidget 集成到主窗口中，
 * 新版本通过检测页面状态自动启用/禁用摇杆功能，无需返回按钮。
 */

#include <QApplication>
#include <QWidget>
#include <QStackedWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QButtonGroup>
#include <QAbstractButton>
#include <QDebug>

#include "rockerwidget.h"
#include "ros_cmd_vel/BtnControl.h"
#include <ros/ros.h>

/**
 * @brief 主窗口类，模拟 frmMain 的功能
 */
class MainWindow : public QWidget
{
    Q_OBJECT

public:
    explicit MainWindow(ros::NodeHandle &nh, QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void leftMainClick();           // 处理主菜单按钮点击
    void onRockerMoved(double x, double y);     // 处理摇杆移动
    void onRockerReleased();        // 处理摇杆释放

private:
    void setupUI();                 // 设置界面
    void setupMainButtons();        // 设置主菜单按钮
    void setupStackedWidget();      // 设置堆叠窗口
    
    // ROS 相关
    ros::NodeHandle &nh_;
    BtnController *btnController_;
    
    // UI 组件
    QVBoxLayout *mainLayout_;
    QHBoxLayout *topLayout_;
    
    // 主菜单按钮
    QButtonGroup *btnsMain_;
    QPushButton *btnKeyControl_;
    QPushButton *btnRockerControl_;
    
    // 堆叠窗口及页面
    QStackedWidget *stackedWidget_main_;
    QWidget *keyControlPage_;       // 按键控制页面
    RockerWidget *rockerWidget_;    // 摇杆控制页面
    
    // 状态标签
    QLabel *statusLabel_;
};

MainWindow::MainWindow(ros::NodeHandle &nh, QWidget *parent)
    : QWidget(parent)
    , nh_(nh)
    , btnController_(nullptr)
{
    // 创建控制器
    btnController_ = new BtnController(nh_);
    
    setupUI();
    setupMainButtons();
    setupStackedWidget();
    
    // 设置窗口属性
    setWindowTitle("摇杆控制系统 - 页面状态检测版本");
    setMinimumSize(800, 600);
    resize(900, 700);
    
    // 默认选中按键控制
    btnKeyControl_->setChecked(true);
    stackedWidget_main_->setCurrentIndex(0);
}

MainWindow::~MainWindow()
{
    delete btnController_;
}

void MainWindow::setupUI()
{
    mainLayout_ = new QVBoxLayout(this);
    
    // 创建顶部按钮布局
    topLayout_ = new QHBoxLayout();
    
    // 创建状态标签
    statusLabel_ = new QLabel("系统就绪", this);
    statusLabel_->setStyleSheet("QLabel { font-size: 14px; color: #666; padding: 10px; }");
    
    // 添加到主布局
    mainLayout_->addLayout(topLayout_);
    mainLayout_->addWidget(statusLabel_);
    mainLayout_->setContentsMargins(20, 20, 20, 20);
    mainLayout_->setSpacing(15);
}

void MainWindow::setupMainButtons()
{
    // 创建按钮组用于互斥选择
    btnsMain_ = new QButtonGroup(this);
    
    // 创建按键控制按钮
    btnKeyControl_ = new QPushButton("按键控制", this);
    btnKeyControl_->setCheckable(true);
    btnKeyControl_->setFixedSize(120, 40);
    btnKeyControl_->setStyleSheet(
        "QPushButton {"
        "    background-color: #3498db;"
        "    color: white;"
        "    border: none;"
        "    border-radius: 20px;"
        "    font-weight: bold;"
        "    font-size: 14px;"
        "}"
        "QPushButton:checked {"
        "    background-color: #2980b9;"
        "}"
        "QPushButton:hover {"
        "    background-color: #5dade2;"
        "}"
    );
    
    // 创建摇杆控制按钮
    btnRockerControl_ = new QPushButton("摇杆控制", this);
    btnRockerControl_->setCheckable(true);
    btnRockerControl_->setFixedSize(120, 40);
    btnRockerControl_->setStyleSheet(btnKeyControl_->styleSheet());
    
    // 添加到按钮组
    btnsMain_->addButton(btnKeyControl_);
    btnsMain_->addButton(btnRockerControl_);
    
    // 连接信号
    connect(btnKeyControl_, &QPushButton::clicked, this, &MainWindow::leftMainClick);
    connect(btnRockerControl_, &QPushButton::clicked, this, &MainWindow::leftMainClick);
    
    // 添加到布局
    topLayout_->addWidget(btnKeyControl_);
    topLayout_->addWidget(btnRockerControl_);
    topLayout_->addStretch(); // 弹性空间
}

void MainWindow::setupStackedWidget()
{
    stackedWidget_main_ = new QStackedWidget(this);
    
    // 创建按键控制页面（简单的占位页面）
    keyControlPage_ = new QWidget();
    keyControlPage_->setStyleSheet("QWidget { background-color: #ecf0f1; border-radius: 10px; }");
    
    QVBoxLayout *keyLayout = new QVBoxLayout(keyControlPage_);
    QLabel *keyLabel = new QLabel("按键控制页面", keyControlPage_);
    keyLabel->setAlignment(Qt::AlignCenter);
    keyLabel->setStyleSheet("QLabel { font-size: 18px; font-weight: bold; color: #34495e; }");
    keyLayout->addWidget(keyLabel);
    
    // 添加一些示例按钮
    QHBoxLayout *buttonsLayout = new QHBoxLayout();
    QPushButton *forwardBtn = new QPushButton("前进");
    QPushButton *backwardBtn = new QPushButton("后退");
    QPushButton *leftBtn = new QPushButton("左转");
    QPushButton *rightBtn = new QPushButton("右转");
    QPushButton *stopBtn = new QPushButton("停止");
    
    QString buttonStyle = 
        "QPushButton {"
        "    background-color: #e74c3c;"
        "    color: white;"
        "    border: none;"
        "    border-radius: 15px;"
        "    font-weight: bold;"
        "    padding: 10px 20px;"
        "}"
        "QPushButton:hover {"
        "    background-color: #c0392b;"
        "}"
        "QPushButton:pressed {"
        "    background-color: #a93226;"
        "}";
    
    forwardBtn->setStyleSheet(buttonStyle);
    backwardBtn->setStyleSheet(buttonStyle);
    leftBtn->setStyleSheet(buttonStyle);
    rightBtn->setStyleSheet(buttonStyle);
    stopBtn->setStyleSheet(buttonStyle.replace("#e74c3c", "#27ae60").replace("#c0392b", "#229954").replace("#a93226", "#1e8449"));
    
    buttonsLayout->addWidget(forwardBtn);
    buttonsLayout->addWidget(backwardBtn);
    buttonsLayout->addWidget(leftBtn);
    buttonsLayout->addWidget(rightBtn);
    buttonsLayout->addWidget(stopBtn);
    
    keyLayout->addLayout(buttonsLayout);
    keyLayout->addStretch();
    
    // 连接按钮事件到 BtnController
    connect(forwardBtn, &QPushButton::clicked, [this]() { 
        btnController_->startMovingForward(); 
        statusLabel_->setText("前进中...");
    });
    connect(backwardBtn, &QPushButton::clicked, [this]() { 
        btnController_->startMovingBackward(); 
        statusLabel_->setText("后退中...");
    });
    connect(leftBtn, &QPushButton::clicked, [this]() { 
        btnController_->startTurningLeft(); 
        statusLabel_->setText("左转中...");
    });
    connect(rightBtn, &QPushButton::clicked, [this]() { 
        btnController_->startTurningRight(); 
        statusLabel_->setText("右转中...");
    });
    connect(stopBtn, &QPushButton::clicked, [this]() { 
        btnController_->stopMovement(); 
        statusLabel_->setText("已停止");
    });
    
    // 创建摇杆控制页面
    rockerWidget_ = new RockerWidget(btnController_, this);
    
    // 设置摇杆关联的堆叠窗口信息
    rockerWidget_->setStackedWidgetInfo(stackedWidget_main_, 1);
    
    // 连接摇杆信号
    connect(rockerWidget_, &RockerWidget::rockerMoved, this, &MainWindow::onRockerMoved);
    connect(rockerWidget_, &RockerWidget::rockerReleased, this, &MainWindow::onRockerReleased);
    
    // 添加页面到堆叠窗口
    stackedWidget_main_->addWidget(keyControlPage_);    // 索引 0
    stackedWidget_main_->addWidget(rockerWidget_);      // 索引 1
    
    // 添加到主布局
    mainLayout_->addWidget(stackedWidget_main_, 1);
}

void MainWindow::leftMainClick()
{
    // 获取发送信号的按钮对象
    QAbstractButton *b = qobject_cast<QAbstractButton*>(sender());
    if (!b) return;
    
    QString name = b->text();  // 获取按钮文本作为菜单名称
    
    // 在主菜单按钮组中设置互斥选择（只能选中一个按钮）
    for (int i = 0; i < btnsMain_->buttons().count(); ++i) {
        QAbstractButton *btn = btnsMain_->buttons().at(i);
        // 只有当前被点击的按钮设置为选中状态
        btn->setChecked(btn == b);
    }

    // 根据按钮文字切换到对应的页面
    if (name == "按键控制") {
        stackedWidget_main_->setCurrentIndex(0);      // 切换到第0页：按键控制
        statusLabel_->setText("切换到按键控制模式");
        qDebug() << "切换到按键控制页面";
    } else if (name == "摇杆控制") {
        stackedWidget_main_->setCurrentIndex(1);      // 切换到第1页：摇杆控制
        statusLabel_->setText("切换到摇杆控制模式");
        qDebug() << "切换到摇杆控制页面";
    }
}

void MainWindow::onRockerMoved(double x, double y)
{
    statusLabel_->setText(QString("摇杆位置: X=%1, Y=%2").arg(x, 0, 'f', 2).arg(y, 0, 'f', 2));
    qDebug() << "摇杆移动:" << x << "," << y;
}

void MainWindow::onRockerReleased()
{
    statusLabel_->setText("摇杆已释放，机器人停止");
    qDebug() << "摇杆释放";
}

/**
 * @brief 主函数
 */
int main(int argc, char *argv[])
{
    // 初始化 ROS
    ros::init(argc, argv, "rocker_widget_example");
    ros::NodeHandle nh;
    
    // 初始化 Qt
    QApplication app(argc, argv);
    
    // 创建主窗口
    MainWindow window(nh, nullptr);
    window.show();
    
    // 创建 ROS 处理定时器
    QTimer rosTimer;
    QObject::connect(&rosTimer, &QTimer::timeout, []() {
        ros::spinOnce();
    });
    rosTimer.start(10); // 每10ms处理一次ROS消息
    
    qDebug() << "摇杆控制系统已启动";
    qDebug() << "特性：";
    qDebug() << "- 通过页面状态自动控制摇杆功能";
    qDebug() << "- 切换页面时自动停止机器人移动";
    qDebug() << "- 无需返回按钮，完全集成到StackedWidget";
    
    return app.exec();
}

#include "stackedwidget_example_updated.moc"
