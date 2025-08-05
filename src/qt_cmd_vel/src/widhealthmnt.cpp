#include "qt_cmd_vel/widhealthmnt.h"
#include "ui_widhealthmnt.h"
#include <QTextCursor>
#include <QMetaObject>

widhealthMnt::widhealthMnt(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::widhealthMnt),
    monitor(new HealthMonitor(DEVICE_PORT))
{
    ui->setupUi(this);
    
    // 连接信号和槽（用于线程安全的UI更新）
    connect(this, &widhealthMnt::displayUpdate, this, &widhealthMnt::updateDisplay, Qt::QueuedConnection);
    
    ui->plainTextEdit->appendPlainText("正在连接健康监测模块...");
    
    try {
        // 尝试连接设备
        if (!monitor->connect()) {
            ui->plainTextEdit->appendPlainText("无法连接到健康监测模块，请检查连接");
            return;
        }
        
        ui->plainTextEdit->appendPlainText("连接成功，开始采集数据...");
        
        // 启动数据采集，使用 lambda 回调（类似示例程序）
        monitor->startCollection([this](const realtime_packet_t& data) {
            if (!keep_running.load()) return; // 如果停止标志被设置，直接返回
            
            // 格式化数据
            QString newData;
            newData += "========================================\n";
            newData += QString("请将手中轻放在设备上，等待数秒数值稳定后观察读数\n");
            newData += QString("用户检测: %1\n").arg(is_user_detected(data.state) ? "是" : "否");
            newData += QString("心率:    %1 bpm\n").arg(data.heartrate);
            newData += QString("SpO2:          %1 %%\n").arg(data.spo2);
            newData += QString("SDNN:          %1\n").arg(data.sdnn);
            newData += QString("Fatigue:       %1\n").arg(get_fatigue_status(data.sdnn));
            
            // 处理 RR 间期数据
            auto rr_points = monitor->getRRScatterPoints(data);
            if (!rr_points.empty()) {
                newData += "RR Points:     ";
                for (const auto& p : rr_points) {
                    newData += QString("(%1,%2) ").arg(p.first).arg(p.second);
                }
                newData += "\n";
            }
            
            // 通过信号更新UI（线程安全）
            emit displayUpdate(newData);
        });
        
    } catch (const std::exception& e) {
        ui->plainTextEdit->appendPlainText(QString("连接异常: %1").arg(e.what()));
    }
}

widhealthMnt::~widhealthMnt()
{
    keep_running.store(false); // 设置停止标志
    
    // 按照示例程序的方式安全地停止和断开
    if (monitor) {
        try {
            monitor->stopCollection();
            monitor->disconnect();
            delete monitor; // 手动删除对象
            monitor = nullptr;
        } catch (const std::exception& e) {
            // 忽略析构时的异常
        }
    }
    
    delete ui;
}

void widhealthMnt::on_BtnBack2_clicked()
{
    keep_running.store(false); // 设置停止标志
    this->close(); // 关闭对话框，析构函数会处理资源清理
}

void widhealthMnt::updateDisplay(QString text)
{
    // 在主线程中更新UI - 实时刷新，不累积历史数据
    ui->plainTextEdit->setPlainText(text);
    
    // 自动滚动到底部
    ui->plainTextEdit->moveCursor(QTextCursor::End);
}
