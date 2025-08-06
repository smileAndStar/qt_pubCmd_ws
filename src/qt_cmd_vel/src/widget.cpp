#include "qt_cmd_vel/widget.h"
#include "./ui_widget.h"
#include "ros_cmd_vel/BtnControl.h"
#include "qt_cmd_vel/worker.h"
#include <QThread>
#include "qt_cmd_vel/widrocker.h"
#include "qt_cmd_vel/widhealthmnt.h"

Widget::Widget(ros::NodeHandle &nh, QWidget *parent)
    : QWidget(parent)
    , nh_(nh)
    , ui(new Ui::Widget)
{
    // 这个函数负责创建和布局你在 Designer 中设计的所有控件。
    // 它会自动生成代码来初始化 UI 元素。
    // 例如，设置按钮、滑块、标签等的属性和布局,包括信号和槽的连接。
    // 函数调用会依次执行以下一系列操作：
    // 1.  【检查和命名】
    // 2.  【创建控件 (Widgets)】
    // 3.  【设置属性 (Properties)】
    // 4.  【创建和应用布局 (Layouts)】
    // 5.  【建立自动信号与槽连接 (The "Magic")】
    // 6.  【设置其他窗口属性】
    ui->setupUi(this);

    // 设置窗口标题
    this->setWindowTitle("底盘控制器");

    // 创建按键控制器实例
    btn_controller = new BtnController(nh_);

    //////////////////////////////////////////////////////////////////////////////////////////
    // 此线程负责发布ros消息
    // 创建并启动工作线程
    QThread * thread = new QThread();
    Worker * worker = new Worker(btn_controller);
    worker->moveToThread(thread);
    
    // 当线程启动时，开始执行 process() 循环
    connect(thread, &QThread::started, worker, &Worker::process);
    // 当 worker 发出 finished 信号时，停止线程
    connect(worker, &Worker::finished, thread, &QThread::quit);
    // 当线程结束时，删除 worker 对象
    connect(thread, &QThread::finished, worker, &QObject::deleteLater);
    // 当线程结束时，删除线程对象
    connect(thread, &QThread::finished, thread, &QObject::deleteLater);

    // 线程，启动!
    thread->start();
    //////////////////////////////////////////////////////////////////////////////////////////
}

Widget::~Widget()
{
    delete ui;
}

void Widget::on_BtnLeft_clicked()
{
    // 更新状态，无阻塞立即返回
    btn_controller->startTurningLeft();
}

void Widget::on_BtnStop_clicked()
{
    btn_controller->stopMovement();
}

void Widget::on_BtnRight_clicked()
{
    btn_controller->startTurningRight();
}

void Widget::on_BtnForward_clicked()
{
    btn_controller->startMovingForward();
}

void Widget::on_BtnBackward_clicked()
{
    btn_controller->startMovingBackward();
}

void Widget::on_LineSpeed_valueChanged(int value)
{
    double linear_speed = value / 10.0; // 将值转换为 m/s，范围是 [0, 20] 对应 0.0 到 2.0 m/s
    btn_controller->setLinearSpeed(linear_speed);
}

void Widget::on_AngulSpeed_valueChanged(int value)
{
    double angular_speed = value / 100.0; // 将值转换为 rad/s，范围是 [0, 100] 对应 0.00 到 1.00 rad/s
    btn_controller->setAngularSpeed(angular_speed);
}

void Widget::on_BtnRocker_clicked()
{
    // 切换到摇杆控制状态
    btn_controller->stateRockerControl();
    widrocker *rocker = new widrocker(btn_controller, this);
    rocker->setModal(true);         //关闭当前对话框前，无法操作其他窗口
    rocker->show();
    rocker->exec();             //阻塞程序直到对话框关闭
    delete rocker;
}

//健康检测界面
void Widget::on_BtnHealth_clicked()
{
    btn_controller->stopMovement();     //先停止运动
    widhealthMnt *health = new widhealthMnt(nh_, this);
    health->setModal(true);
    health->show();
    health->exec();
    delete health;
}
