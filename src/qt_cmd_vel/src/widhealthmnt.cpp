#include "qt_cmd_vel/widhealthmnt.h"
#include "ui_widhealthmnt.h"

widhealthMnt::widhealthMnt(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::widhealthMnt)
{
    ui->setupUi(this);

    // 开始采集数据
    HealthMonitor monitor(DEVICE_PORT);
    global_monitor = &monitor;

    if(!monitor.connect()){
        global_monitor = nullptr;
        ui->plainTextEdit->appendPlainText("无法连接到健康监测模块，请检查连接");
    }

    //lambda 回调函数，处理接收到的数据
    // 捕获列表 [&] 表示以引用的方式访问外部作用域中的所有变量
    monitor.startCollection([&](const realtime_packet_t& data){
        if(!keep_running.load()) return; // 如果不再采集，直接返回

        // 处理接收到的数据

        // 将数据格式化为字符串
        outputText += "========================================\n";
        outputText += QString("User Detected: %1\n").arg(is_user_detected(data.state) ? "Yes" : "No");
        outputText += QString("Heart Rate:    %1 bpm\n").arg(data.heartrate);
        outputText += QString("SpO2:          %1 %\n").arg(data.spo2);
        outputText += QString("SDNN:          %1\n").arg(data.sdnn);
        outputText += QString("Fatigue:       %1\n").arg(get_fatigue_status(data.sdnn));
        // 处理 RR 间期数据
        auto rr_points = monitor.getRRScatterPoints(data);
        if(!rr_points.empty()){
            outputText += "RR Points:\n";
            for(const auto& rr : rr_points){
                outputText += QString("(%1,%2) ").arg(rr.first).arg(rr.second);
            }
            outputText += "\n";
        }   

        // 使用 setPlainText() 方法刷新 QplainTextEdit 的内容
        // 该方法会清除旧内容并显示新内容。
        ui->plainTextEdit->setPlainText(outputText);
    });

}

widhealthMnt::~widhealthMnt()
{
    delete ui;

    // 停止数据采集,释放资源
    global_monitor->stopCollection();
    global_monitor->disconnect();
    global_monitor = nullptr;
}

void widhealthMnt::on_BtnBack2_clicked()
{
    keep_running.store(false); // 停止采集数据
    this->close();
}
