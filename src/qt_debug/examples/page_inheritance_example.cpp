/**
 * @file page_inheritance_example.cpp
 * @brief 演示如何选择性使用 PageStateWidget 基类
 * 
 * 这个示例展示了在一个完整的应用中，
 * 哪些页面需要继承 PageStateWidget，哪些直接使用 QWidget
 */

#include <QApplication>
#include <QMainWindow>
#include <QStackedWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QTextEdit>
#include <QFormLayout>
#include <QLineEdit>
#include <QSpinBox>
#include <QCheckBox>

#include "../common/page_state_widget.h"

// ============================================
// 需要页面状态管理的组件 (继承 PageStateWidget)
// ============================================

/**
 * @brief 摇杆控制页面 - 需要页面状态管理
 * 只有在当前页面时才响应摇杆操作，避免误操作
 */
class RockerControlPage : public PageStateWidget
{
    Q_OBJECT

public:
    explicit RockerControlPage(QWidget *parent = nullptr) : PageStateWidget(parent) {
        setupUI();
    }

protected:
    void onWidgetActivated() override {
        statusLabel_->setText("🟢 摇杆控制已激活");
        statusLabel_->setStyleSheet("color: green; font-weight: bold;");
        // 启动摇杆控制逻辑
        qDebug() << "摇杆控制激活：开始接收用户输入";
    }

    void onWidgetDeactivated() override {
        statusLabel_->setText("🔴 摇杆控制已停止");
        statusLabel_->setStyleSheet("color: red; font-weight: bold;");
        // 停止摇杆控制，发送停止命令
        qDebug() << "摇杆控制停止：发送停止移动命令";
    }

private:
    void setupUI() {
        auto layout = new QVBoxLayout(this);
        
        auto titleLabel = new QLabel("摇杆控制页面");
        titleLabel->setAlignment(Qt::AlignCenter);
        titleLabel->setStyleSheet("font-size: 18px; font-weight: bold; margin: 10px;");
        
        statusLabel_ = new QLabel("等待激活...");
        statusLabel_->setAlignment(Qt::AlignCenter);
        
        auto rockerArea = new QLabel("🕹️ 摇杆控制区域");
        rockerArea->setAlignment(Qt::AlignCenter);
        rockerArea->setMinimumHeight(200);
        rockerArea->setStyleSheet("border: 2px dashed #ccc; border-radius: 10px; font-size: 24px;");
        
        layout->addWidget(titleLabel);
        layout->addWidget(statusLabel_);
        layout->addWidget(rockerArea, 1);
    }
    
    QLabel* statusLabel_;
};

/**
 * @brief 数据采集页面 - 需要页面状态管理
 * 只有在当前页面时才开始数据采集，节省系统资源
 */
class DataCollectionPage : public PageStateWidget
{
    Q_OBJECT

public:
    explicit DataCollectionPage(QWidget *parent = nullptr) : PageStateWidget(parent) {
        setupUI();
        collectionTimer_ = new QTimer(this);
        connect(collectionTimer_, &QTimer::timeout, this, &DataCollectionPage::collectData);
    }

protected:
    void onWidgetActivated() override {
        statusLabel_->setText("🟢 数据采集中...");
        statusLabel_->setStyleSheet("color: green;");
        collectionTimer_->start(1000); // 每秒采集一次数据
        qDebug() << "数据采集激活：开始采集传感器数据";
    }

    void onWidgetDeactivated() override {
        statusLabel_->setText("🔴 数据采集已停止");
        statusLabel_->setStyleSheet("color: red;");
        collectionTimer_->stop();
        qDebug() << "数据采集停止：释放传感器资源";
    }

private slots:
    void collectData() {
        static int dataCount = 0;
        dataTextEdit_->append(QString("数据 #%1: 温度=25.%2°C, 湿度=%3%")
                             .arg(++dataCount)
                             .arg(qrand() % 100)
                             .arg(40 + qrand() % 20));
    }

private:
    void setupUI() {
        auto layout = new QVBoxLayout(this);
        
        auto titleLabel = new QLabel("数据采集页面");
        titleLabel->setAlignment(Qt::AlignCenter);
        titleLabel->setStyleSheet("font-size: 18px; font-weight: bold; margin: 10px;");
        
        statusLabel_ = new QLabel("等待激活...");
        statusLabel_->setAlignment(Qt::AlignCenter);
        
        dataTextEdit_ = new QTextEdit();
        dataTextEdit_->setMaximumHeight(150);
        dataTextEdit_->setPlaceholderText("数据采集日志将显示在这里...");
        
        layout->addWidget(titleLabel);
        layout->addWidget(statusLabel_);
        layout->addWidget(dataTextEdit_, 1);
    }
    
    QLabel* statusLabel_;
    QTextEdit* dataTextEdit_;
    QTimer* collectionTimer_;
};

// ============================================
// 不需要页面状态管理的组件 (直接继承 QWidget)
// ============================================

/**
 * @brief 主页面 - 不需要页面状态管理
 * 只是静态内容展示，无需特殊的生命周期控制
 */
class HomePage : public QWidget
{
    Q_OBJECT

public:
    explicit HomePage(QWidget *parent = nullptr) : QWidget(parent) {
        setupUI();
    }

private:
    void setupUI() {
        auto layout = new QVBoxLayout(this);
        
        auto welcomeLabel = new QLabel("欢迎使用机器人控制系统");
        welcomeLabel->setAlignment(Qt::AlignCenter);
        welcomeLabel->setStyleSheet("font-size: 24px; font-weight: bold; color: #2c3e50; margin: 20px;");
        
        auto descLabel = new QLabel(
            "这是一个演示页面状态管理的示例应用。\n\n"
            "• 摇杆控制页面：使用 PageStateWidget 进行状态管理\n"
            "• 数据采集页面：使用 PageStateWidget 进行资源管理\n"
            "• 设置页面：普通 QWidget，无需状态管理\n"
            "• 关于页面：普通 QWidget，静态内容展示"
        );
        descLabel->setAlignment(Qt::AlignCenter);
        descLabel->setStyleSheet("font-size: 14px; line-height: 1.6; color: #555;");
        
        auto iconLabel = new QLabel("🤖");
        iconLabel->setAlignment(Qt::AlignCenter);
        iconLabel->setStyleSheet("font-size: 64px; margin: 30px;");
        
        layout->addStretch();
        layout->addWidget(welcomeLabel);
        layout->addWidget(iconLabel);
        layout->addWidget(descLabel);
        layout->addStretch();
    }
};

/**
 * @brief 设置页面 - 不需要页面状态管理
 * 普通的配置表单，保存/加载配置即可
 */
class SettingsPage : public QWidget
{
    Q_OBJECT

public:
    explicit SettingsPage(QWidget *parent = nullptr) : QWidget(parent) {
        setupUI();
    }

private slots:
    void saveSettings() {
        // 保存设置逻辑
        qDebug() << "保存设置:"
                 << "机器人名称=" << robotNameEdit_->text()
                 << "最大速度=" << maxSpeedSpin_->value()
                 << "自动连接=" << autoConnectCheck_->isChecked();
    }

private:
    void setupUI() {
        auto layout = new QVBoxLayout(this);
        
        auto titleLabel = new QLabel("系统设置");
        titleLabel->setAlignment(Qt::AlignCenter);
        titleLabel->setStyleSheet("font-size: 18px; font-weight: bold; margin: 10px;");
        
        // 创建设置表单
        auto formLayout = new QFormLayout();
        
        robotNameEdit_ = new QLineEdit("MyRobot");
        formLayout->addRow("机器人名称:", robotNameEdit_);
        
        maxSpeedSpin_ = new QSpinBox();
        maxSpeedSpin_->setRange(1, 100);
        maxSpeedSpin_->setValue(50);
        maxSpeedSpin_->setSuffix(" cm/s");
        formLayout->addRow("最大速度:", maxSpeedSpin_);
        
        autoConnectCheck_ = new QCheckBox("启动时自动连接");
        autoConnectCheck_->setChecked(true);
        formLayout->addRow("", autoConnectCheck_);
        
        auto saveBtn = new QPushButton("保存设置");
        connect(saveBtn, &QPushButton::clicked, this, &SettingsPage::saveSettings);
        
        layout->addWidget(titleLabel);
        layout->addLayout(formLayout);
        layout->addStretch();
        layout->addWidget(saveBtn);
    }
    
    QLineEdit* robotNameEdit_;
    QSpinBox* maxSpeedSpin_;
    QCheckBox* autoConnectCheck_;
};

/**
 * @brief 关于页面 - 不需要页面状态管理
 * 纯静态信息展示，无需生命周期控制
 */
class AboutPage : public QWidget
{
    Q_OBJECT

public:
    explicit AboutPage(QWidget *parent = nullptr) : QWidget(parent) {
        setupUI();
    }

private:
    void setupUI() {
        auto layout = new QVBoxLayout(this);
        
        auto titleLabel = new QLabel("关于本应用");
        titleLabel->setAlignment(Qt::AlignCenter);
        titleLabel->setStyleSheet("font-size: 18px; font-weight: bold; margin: 10px;");
        
        auto infoText = new QTextEdit();
        infoText->setReadOnly(true);
        infoText->setHtml(
            "<h3>机器人控制系统 v1.0</h3>"
            "<p><b>开发者:</b> GitHub Copilot & 用户</p>"
            "<p><b>技术栈:</b> Qt5, C++, ROS</p>"
            "<p><b>功能特点:</b></p>"
            "<ul>"
            "<li>智能页面状态管理</li>"
            "<li>模块化组件设计</li>"
            "<li>资源自动管理</li>"
            "<li>可扩展架构</li>"
            "</ul>"
            "<p><b>项目地址:</b> <a href='https://github.com/smileAndStar/qt_pubCmd_ws'>GitHub</a></p>"
        );
        
        layout->addWidget(titleLabel);
        layout->addWidget(infoText);
    }
};

// ============================================
// 主窗口
// ============================================

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr) : QMainWindow(parent) {
        setupUI();
        setupPages();
        connectSignals();
        
        setWindowTitle("页面状态管理示例");
        setMinimumSize(800, 600);
        
        // 默认显示主页
        stackedWidget_->setCurrentIndex(0);
        homeBtn_->setChecked(true);
    }

private:
    void setupUI() {
        auto centralWidget = new QWidget(this);
        setCentralWidget(centralWidget);
        
        auto layout = new QVBoxLayout(centralWidget);
        
        // 菜单按钮
        auto menuLayout = new QHBoxLayout();
        
        homeBtn_ = new QPushButton("🏠 主页");
        rockerBtn_ = new QPushButton("🕹️ 摇杆控制");
        dataBtn_ = new QPushButton("📊 数据采集");
        settingsBtn_ = new QPushButton("⚙️ 设置");
        aboutBtn_ = new QPushButton("ℹ️ 关于");
        
        QString btnStyle = 
            "QPushButton {"
            "    padding: 10px 20px;"
            "    font-size: 14px;"
            "    border: none;"
            "    border-radius: 5px;"
            "    background-color: #3498db;"
            "    color: white;"
            "}"
            "QPushButton:checked {"
            "    background-color: #2980b9;"
            "}"
            "QPushButton:hover {"
            "    background-color: #5dade2;"
            "}";
        
        homeBtn_->setStyleSheet(btnStyle);
        rockerBtn_->setStyleSheet(btnStyle);
        dataBtn_->setStyleSheet(btnStyle);
        settingsBtn_->setStyleSheet(btnStyle);
        aboutBtn_->setStyleSheet(btnStyle);
        
        homeBtn_->setCheckable(true);
        rockerBtn_->setCheckable(true);
        dataBtn_->setCheckable(true);
        settingsBtn_->setCheckable(true);
        aboutBtn_->setCheckable(true);
        
        menuLayout->addWidget(homeBtn_);
        menuLayout->addWidget(rockerBtn_);
        menuLayout->addWidget(dataBtn_);
        menuLayout->addWidget(settingsBtn_);
        menuLayout->addWidget(aboutBtn_);
        menuLayout->addStretch();
        
        // 堆叠窗口
        stackedWidget_ = new QStackedWidget();
        
        layout->addLayout(menuLayout);
        layout->addWidget(stackedWidget_, 1);
    }
    
    void setupPages() {
        // 创建页面实例
        homePage_ = new HomePage();                    // 普通 QWidget
        rockerPage_ = new RockerControlPage();         // PageStateWidget
        dataPage_ = new DataCollectionPage();          // PageStateWidget
        settingsPage_ = new SettingsPage();            // 普通 QWidget
        aboutPage_ = new AboutPage();                  // 普通 QWidget
        
        // 配置需要状态管理的页面
        rockerPage_->addPageMonitor("rocker_control", stackedWidget_, 1, "摇杆控制页面");
        dataPage_->addPageMonitor("data_collection", stackedWidget_, 2, "数据采集页面");
        
        // 添加到堆叠窗口
        stackedWidget_->addWidget(homePage_);       // 索引 0
        stackedWidget_->addWidget(rockerPage_);     // 索引 1
        stackedWidget_->addWidget(dataPage_);       // 索引 2
        stackedWidget_->addWidget(settingsPage_);   // 索引 3
        stackedWidget_->addWidget(aboutPage_);      // 索引 4
    }
    
    void connectSignals() {
        connect(homeBtn_, &QPushButton::clicked, [this]() { switchToPage(0, homeBtn_); });
        connect(rockerBtn_, &QPushButton::clicked, [this]() { switchToPage(1, rockerBtn_); });
        connect(dataBtn_, &QPushButton::clicked, [this]() { switchToPage(2, dataBtn_); });
        connect(settingsBtn_, &QPushButton::clicked, [this]() { switchToPage(3, settingsBtn_); });
        connect(aboutBtn_, &QPushButton::clicked, [this]() { switchToPage(4, aboutBtn_); });
    }
    
    void switchToPage(int index, QPushButton* btn) {
        // 取消所有按钮的选中状态
        homeBtn_->setChecked(false);
        rockerBtn_->setChecked(false);
        dataBtn_->setChecked(false);
        settingsBtn_->setChecked(false);
        aboutBtn_->setChecked(false);
        
        // 选中当前按钮
        btn->setChecked(true);
        
        // 切换页面
        stackedWidget_->setCurrentIndex(index);
        
        qDebug() << "切换到页面:" << index;
    }
    
    QStackedWidget* stackedWidget_;
    QPushButton* homeBtn_;
    QPushButton* rockerBtn_;
    QPushButton* dataBtn_;
    QPushButton* settingsBtn_;
    QPushButton* aboutBtn_;
    
    // 页面实例
    HomePage* homePage_;
    RockerControlPage* rockerPage_;
    DataCollectionPage* dataPage_;
    SettingsPage* settingsPage_;
    AboutPage* aboutPage_;
};

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    
    MainWindow window;
    window.show();
    
    qDebug() << "页面继承策略演示启动";
    qDebug() << "观察不同页面的行为差异:";
    qDebug() << "- 摇杆控制页面: 使用 PageStateWidget，有状态管理";
    qDebug() << "- 数据采集页面: 使用 PageStateWidget，有资源管理";
    qDebug() << "- 主页/设置/关于: 使用普通 QWidget，无特殊管理";
    
    return app.exec();
}

#include "page_inheritance_example.moc"
