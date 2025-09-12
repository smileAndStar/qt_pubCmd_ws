#include "btnWidgetCtl.h"

btncontrol::btncontrol(ros::NodeHandle &nh, PageStateWidget *parent) :
    QObject(parent),
    parentWidget(parent),
    nh_(nh)
{
    // 查找并缓存UI控件指针
    btnLeft = parentWidget->findChild<QPushButton*>("BtnLeft");
    btnStop = parentWidget->findChild<QPushButton*>("BtnStop");
    btnRight = parentWidget->findChild<QPushButton*>("BtnRight");
    btnForward = parentWidget->findChild<QPushButton*>("BtnForward");
    btnBackward = parentWidget->findChild<QPushButton*>("BtnBackward");
    lineSpeed = parentWidget->findChild<QSlider*>("LineSpeed");
    angulSpeed = parentWidget->findChild<QSlider*>("AngulSpeed");

    // 创建按键控制器实例
    btn_controller = new Controller(nh_);

    // 连接按钮点击信号到对应的槽函数
    connect(btnLeft, &QPushButton::clicked, this, &btncontrol::on_BtnLeft_clicked);
    connect(btnStop, &QPushButton::clicked, this, &btncontrol::on_BtnStop_clicked);
    connect(btnRight, &QPushButton::clicked, this, &btncontrol::on_BtnRight_clicked);
    connect(btnForward, &QPushButton::clicked, this, &btncontrol::on_BtnForward_clicked);
    connect(btnBackward, &QPushButton::clicked, this, &btncontrol::on_BtnBackward_clicked);

    // 连接滑块值变化信号到对应的槽函数
    connect(lineSpeed, &QSlider::valueChanged, this, &btncontrol::on_LineSpeed_valueChanged);
    connect(angulSpeed, &QSlider::valueChanged, this, &btncontrol::on_AngulSpeed_valueChanged);

    // 初始化滑块位置
    lineSpeed->setValue(10);  // 初始线速度为 1.0 m/s (10 对应 1.0 m/s)
    angulSpeed->setValue(50);  // 初始角速度为 0.5 rad/s (50 对应 0.5 rad/s)
}

btncontrol::~btncontrol()
{
    delete btn_controller;
}

void btncontrol::on_BtnLeft_clicked() {
    // 更新状态，无阻塞立即返回
    btn_controller->startTurningLeft();
}

void btncontrol::on_BtnStop_clicked() {
    btn_controller->stopMovement();
}

void btncontrol::on_BtnRight_clicked() {
    btn_controller->startTurningRight();
}

void btncontrol::on_BtnForward_clicked() {
    btn_controller->startMovingForward();
}   

void btncontrol::on_BtnBackward_clicked() {
    btn_controller->startMovingBackward();
}

void btncontrol::on_LineSpeed_valueChanged(int value) {
    double speed = static_cast<double>(value) / 10.0; // 将滑块值转换为线速度 (0.0 到 2.0 m/s)
    btn_controller->setLinearSpeed(speed);
}

void btncontrol::on_AngulSpeed_valueChanged(int value) {
    double angular_speed = value / 100.0; // 将值转换为 rad/s，范围是 [0, 100] 对应 0.00 到 1.00 rad/s
    btn_controller->setAngularSpeed(angular_speed);
}