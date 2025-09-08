// 使用 RockerWidget 在 QStackedWidget 中的示例代码

#include "rockerwidget.h"
#include <QApplication>
#include <QMainWindow>
#include <QStackedWidget>
#include <QVBoxLayout>
#include <QPushButton>
#include <QWidget>
#include <QLabel>

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr) : QMainWindow(parent)
    {
        setupUI();
        connectSignals();
    }

    ~MainWindow() 
    {
        delete btnController_;
    }

private slots:
    void showRockerPage()
    {
        stackedWidget_->setCurrentWidget(rockerWidget_);
    }

    void showMainPage()
    {
        stackedWidget_->setCurrentWidget(mainPage_);
    }

private:
    void setupUI()
    {
        // 创建中央widget和主布局
        QWidget *centralWidget = new QWidget(this);
        setCentralWidget(centralWidget);

        // 创建 StackedWidget
        stackedWidget_ = new QStackedWidget(centralWidget);

        // 创建主页面
        createMainPage();

        // 创建摇杆页面
        createRockerPage();

        // 设置布局
        QVBoxLayout *layout = new QVBoxLayout(centralWidget);
        layout->addWidget(stackedWidget_);

        // 默认显示主页面
        stackedWidget_->setCurrentWidget(mainPage_);

        // 设置窗口属性
        setWindowTitle("StackedWidget 摇杆控制示例");
        resize(600, 500);
    }

    void createMainPage()
    {
        mainPage_ = new QWidget();
        
        QVBoxLayout *layout = new QVBoxLayout(mainPage_);
        
        QLabel *title = new QLabel("主页面", mainPage_);
        title->setAlignment(Qt::AlignCenter);
        title->setStyleSheet("QLabel { font-size: 24px; font-weight: bold; margin: 20px; }");
        
        openRockerButton_ = new QPushButton("打开摇杆控制", mainPage_);
        openRockerButton_->setFixedSize(200, 50);
        openRockerButton_->setStyleSheet(
            "QPushButton {"
            "    background-color: #2ecc71;"
            "    color: white;"
            "    border: none;"
            "    border-radius: 25px;"
            "    font-size: 16px;"
            "    font-weight: bold;"
            "}"
            "QPushButton:hover {"
            "    background-color: #27ae60;"
            "}"
        );
        
        layout->addStretch();
        layout->addWidget(title);
        layout->addWidget(openRockerButton_, 0, Qt::AlignCenter);
        layout->addStretch();
        
        // 添加到 StackedWidget
        stackedWidget_->addWidget(mainPage_);
    }

    void createRockerPage()
    {
        // 创建 BtnController（这里需要根据你的实际情况来创建）
        // 假设你有一个 ros::NodeHandle
        // ros::NodeHandle nh; 
        // btnController_ = new BtnController(nh);
        
        // 为了示例，这里使用 nullptr，实际使用时请传入正确的 BtnController
        btnController_ = nullptr; // 实际应用中替换为正确的控制器
        
        // 创建摇杆页面
        rockerWidget_ = new RockerWidget(btnController_);
        
        // 添加到 StackedWidget
        stackedWidget_->addWidget(rockerWidget_);
    }

    void connectSignals()
    {
        // 连接主页面按钮到摇杆页面
        connect(openRockerButton_, &QPushButton::clicked, 
                this, &MainWindow::showRockerPage);

        // 连接摇杆页面的返回信号到主页面
        connect(rockerWidget_, &RockerWidget::backRequested, 
                this, &MainWindow::showMainPage);

        // 可选：连接摇杆移动信号进行额外处理
        connect(rockerWidget_, &RockerWidget::rockerMoved, 
                this, [](double x, double y) {
                    qDebug() << "Rocker moved to:" << x << "," << y;
                });

        // 可选：连接摇杆释放信号
        connect(rockerWidget_, &RockerWidget::rockerReleased, 
                this, []() {
                    qDebug() << "Rocker released";
                });
    }

private:
    QStackedWidget *stackedWidget_;
    QWidget *mainPage_;
    RockerWidget *rockerWidget_;
    QPushButton *openRockerButton_;
    BtnController *btnController_;
};

// 完整的使用示例 main 函数
int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    MainWindow window;
    window.show();

    return app.exec();
}

#include "stackedwidget_example.moc"

/*
使用说明：

1. 在你的项目中包含 rockerwidget.h 和 rockerwidget.cpp

2. 在你的主窗口类中添加 QStackedWidget

3. 创建 RockerWidget 实例并添加到 StackedWidget：
   ```cpp
   // 创建摇杆页面（需要传入 BtnController）
   RockerWidget *rockerPage = new RockerWidget(btnController, this);
   stackedWidget->addWidget(rockerPage);
   
   // 连接返回信号
   connect(rockerPage, &RockerWidget::backRequested, [=]() {
       stackedWidget->setCurrentIndex(0); // 返回到第一页
   });
   ```

4. 切换到摇杆页面：
   ```cpp
   stackedWidget->setCurrentWidget(rockerPage);
   ```

5. 处理摇杆事件（可选）：
   ```cpp
   connect(rockerPage, &RockerWidget::rockerMoved, 
           this, [](double x, double y) {
               // 处理摇杆移动事件
               qDebug() << "摇杆位置:" << x << "," << y;
           });
   ```

优势：
- 无需模态对话框，更流畅的用户体验
- 完全集成到主界面中
- 支持自定义返回逻辑
- 可以轻松添加其他页面
- 保持与原有控制逻辑的兼容性
*/
