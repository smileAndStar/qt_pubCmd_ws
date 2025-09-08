#include "healthmonitorwidget.h"
#include <QDebug>
#include <QTextCursor>
#include <QSplitter>
#include <QFrame>

HealthMonitorWidget::HealthMonitorWidget(ros::NodeHandle &nh, QWidget *parent) :
    QWidget(parent),
    nh_(nh),
    monitor_(new HealthMonitor(DEVICE_PORT)),
    isConnected_(false),
    isDataReceiving_(false),
    dataCount_(0)
{
    setupUI();
    initHealthMonitor();
}

HealthMonitorWidget::~HealthMonitorWidget()
{
    keep_running.store(false); // 设置停止标志
    
    // 按照示例程序的方式安全地停止和断开
    if (monitor_) {
        try {
            monitor_->stopCollection();
            monitor_->disconnect();
            delete monitor_;
            monitor_ = nullptr;
        } catch (const std::exception& e) {
            // 忽略析构时的异常
        }
    }
}

void HealthMonitorWidget::setupUI()
{
    // 创建主布局
    mainLayout_ = new QVBoxLayout(this);
    
    // 创建顶部布局（标题和返回按钮）
    topLayout_ = new QHBoxLayout();
    
    // 创建标题标签
    titleLabel_ = new QLabel("健康监测中心", this);
    titleLabel_->setStyleSheet(
        "QLabel {"
        "    font-size: 20px;"
        "    font-weight: bold;"
        "    color: #2c3e50;"
        "}"
    );
    
    // 创建连接状态标签
    connectionStatusLabel_ = new QLabel("正在连接...", this);
    connectionStatusLabel_->setStyleSheet(
        "QLabel {"
        "    font-size: 12px;"
        "    color: #f39c12;"
        "    padding: 5px 10px;"
        "    border-radius: 10px;"
        "    background-color: #fef9e7;"
        "}"
    );
    
    // 创建返回按钮
    backButton_ = new QPushButton("返回", this);
    backButton_->setFixedSize(80, 35);
    backButton_->setStyleSheet(
        "QPushButton {"
        "    background-color: #e74c3c;"
        "    color: white;"
        "    border: none;"
        "    border-radius: 17px;"
        "    font-weight: bold;"
        "    font-size: 14px;"
        "}"
        "QPushButton:hover {"
        "    background-color: #c0392b;"
        "}"
        "QPushButton:pressed {"
        "    background-color: #a93226;"
        "}"
    );
    
    // 连接返回按钮信号
    connect(backButton_, &QPushButton::clicked, this, &HealthMonitorWidget::onBackClicked);
    
    // 添加到顶部布局
    topLayout_->addWidget(titleLabel_);
    topLayout_->addWidget(connectionStatusLabel_);
    topLayout_->addStretch();
    topLayout_->addWidget(backButton_);
    
    // 创建状态指示器组
    statusGroup_ = new QGroupBox("实时健康指标", this);
    statusGroup_->setStyleSheet(
        "QGroupBox {"
        "    font-weight: bold;"
        "    border: 2px solid #bdc3c7;"
        "    border-radius: 10px;"
        "    margin-top: 10px;"
        "    padding-top: 10px;"
        "}"
        "QGroupBox::title {"
        "    subcontrol-origin: margin;"
        "    left: 10px;"
        "    padding: 0 10px 0 10px;"
        "}"
    );
    
    // 创建状态指示器布局
    indicatorsLayout_ = new QVBoxLayout(statusGroup_);
    statusLayout_ = new QHBoxLayout();
    
    // 心率指示器
    QVBoxLayout *heartRateLayout = new QVBoxLayout();
    heartRateLabel_ = new QLabel("心率: -- bpm", this);
    heartRateLabel_->setStyleSheet("QLabel { font-size: 14px; font-weight: bold; color: #e74c3c; }");
    heartRateBar_ = new QProgressBar(this);
    heartRateBar_->setRange(50, 120);
    heartRateBar_->setStyleSheet(
        "QProgressBar {"
        "    border: 2px solid #bdc3c7;"
        "    border-radius: 5px;"
        "    text-align: center;"
        "}"
        "QProgressBar::chunk {"
        "    background-color: #e74c3c;"
        "    border-radius: 3px;"
        "}"
    );
    heartRateLayout->addWidget(heartRateLabel_);
    heartRateLayout->addWidget(heartRateBar_);
    
    // 血氧指示器
    QVBoxLayout *spo2Layout = new QVBoxLayout();
    spo2Label_ = new QLabel("血氧: -- %", this);
    spo2Label_->setStyleSheet("QLabel { font-size: 14px; font-weight: bold; color: #3498db; }");
    spo2Bar_ = new QProgressBar(this);
    spo2Bar_->setRange(90, 100);
    spo2Bar_->setStyleSheet(
        "QProgressBar {"
        "    border: 2px solid #bdc3c7;"
        "    border-radius: 5px;"
        "    text-align: center;"
        "}"
        "QProgressBar::chunk {"
        "    background-color: #3498db;"
        "    border-radius: 3px;"
        "}"
    );
    spo2Layout->addWidget(spo2Label_);
    spo2Layout->addWidget(spo2Bar_);
    
    // SDNN和用户检测指示器
    QVBoxLayout *otherLayout = new QVBoxLayout();
    sdnnLabel_ = new QLabel("SDNN: --", this);
    sdnnLabel_->setStyleSheet("QLabel { font-size: 14px; font-weight: bold; color: #2ecc71; }");
    userDetectedLabel_ = new QLabel("用户检测: 否", this);
    userDetectedLabel_->setStyleSheet("QLabel { font-size: 14px; font-weight: bold; color: #f39c12; }");
    otherLayout->addWidget(sdnnLabel_);
    otherLayout->addWidget(userDetectedLabel_);
    
    // 添加到状态布局
    statusLayout_->addLayout(heartRateLayout);
    statusLayout_->addLayout(spo2Layout);
    statusLayout_->addLayout(otherLayout);
    indicatorsLayout_->addLayout(statusLayout_);
    
    // 创建数据显示区域
    dataDisplay_ = new QTextEdit(this);
    dataDisplay_->setReadOnly(true);
    dataDisplay_->setMinimumHeight(200);
    dataDisplay_->setStyleSheet(
        "QTextEdit {"
        "    border: 2px solid #bdc3c7;"
        "    border-radius: 10px;"
        "    padding: 10px;"
        "    font-family: 'Courier New', monospace;"
        "    font-size: 12px;"
        "    background-color: #f8f9fa;"
        "}"
    );
    
    // 添加到主布局
    mainLayout_->addLayout(topLayout_);
    mainLayout_->addWidget(statusGroup_);
    mainLayout_->addWidget(dataDisplay_, 1); // 数据显示区域占主要空间
    
    // 设置布局边距和间距
    mainLayout_->setContentsMargins(20, 20, 20, 20);
    mainLayout_->setSpacing(15);
    
    // 创建定时器
    connectionTimer_ = new QTimer(this);
    connectionTimer_->setSingleShot(true);
    connect(connectionTimer_, &QTimer::timeout, this, &HealthMonitorWidget::onConnectionTimeout);
    
    dataTimer_ = new QTimer(this);
    connect(dataTimer_, &QTimer::timeout, this, &HealthMonitorWidget::onDataTimeout);
}

void HealthMonitorWidget::initHealthMonitor()
{
    // 连接信号和槽（用于线程安全的UI更新）
    connect(this, &HealthMonitorWidget::displayUpdate, this, &HealthMonitorWidget::updateDisplay, Qt::QueuedConnection);
    connect(this, &HealthMonitorWidget::healthDataUpdated, this, &HealthMonitorWidget::updateStatusIndicators, Qt::QueuedConnection);
    connect(this, &HealthMonitorWidget::connectionStatusChanged, this, &HealthMonitorWidget::showConnectionStatus, Qt::QueuedConnection);
    
    dataDisplay_->append("正在连接健康监测模块...");
    
    // 启动连接超时定时器
    connectionTimer_->start(5000); // 5秒超时
    
    try {
        // 尝试连接设备
        if (!monitor_->connect()) {
            emit connectionStatusChanged(false);
            dataDisplay_->append("无法连接到健康监测模块，请检查连接");
            return;
        }
        
        emit connectionStatusChanged(true);
        dataDisplay_->append("连接成功，开始采集数据...");
        dataDisplay_->append("请将手指轻放在设备上，等待数秒数值稳定后观察读数");
        
        // 启动数据采集，使用 lambda 回调
        monitor_->startCollection([this](const realtime_packet_t& data) {
            if (!keep_running.load()) return; // 如果停止标志被设置，直接返回
            
            dataCount_++;
            isDataReceiving_ = true;
            
            /////////////////////////////////////////UI消息/////////////////////////////////////////////
            // 格式化数据
            QString newData;
            newData += "========================================\n";
            newData += QString("数据序号: %1\n").arg(dataCount_);
            newData += QString("用户检测: %1\n").arg(is_user_detected(data.state) ? "是" : "否");
            newData += QString("心率: %1 bpm\n").arg(data.heartrate);
            newData += QString("血氧: %1 %%\n").arg(data.spo2);
            newData += QString("SDNN: %1\n").arg(data.sdnn);
            newData += QString("疲劳状态: %1\n").arg(get_fatigue_status(data.sdnn));
            
            // 处理 RR 间期数据
            auto rr_points = monitor_->getRRScatterPoints(data);
            if (!rr_points.empty()) {
                newData += "RR Points: ";
                for (const auto& p : rr_points) {
                    newData += QString("(%1,%2) ").arg(p.first).arg(p.second);
                }
                newData += "\n";
            }
            newData += "\n";
            
            // 通过信号更新UI（线程安全）
            emit displayUpdate(newData);
            emit healthDataUpdated(data.heartrate, data.spo2, data.sdnn, is_user_detected(data.state));

            /////////////////////////////////////////////ros消息//////////////////////////////////////
            // 发布自定义ros消息
            if (person_data_pub_.getTopic().empty()) {
                person_data_pub_ = nh_.advertise<qt_cmd_vel::personData>("/person_data", 10);
            }
            
            // 创建并填充消息 personData
            qt_cmd_vel::personData p;
            p.header = data.header;
            for (size_t i = 0; i < 64; ++i) p.acdata[i] = data.acdata[i];
            p.heartrate = data.heartrate;
            p.spo2 = data.spo2; 
            p.bk = data.bk;
            for (size_t i = 0; i < 8; ++i) p.rsv1[i] = data.rsv1[i];
            p.sdnn = data.sdnn;
            p.rmssd = data.rmssd;
            p.nn50 = data.nn50;
            p.pnn50 = data.pnn50;
            for (size_t i = 0; i < 6; ++i) p.rra[i] = data.rra[i];
            p.rsv2 = data.rsv2;
            p.state = data.state;
            
            // 发布消息
            person_data_pub_.publish(p);
        });
        
        // 启动数据监控定时器
        dataTimer_->start(10000); // 10秒检查一次数据接收状态
        
    } catch (const std::exception& e) {
        emit connectionStatusChanged(false);
        dataDisplay_->append(QString("连接异常: %1").arg(e.what()));
    }
}

void HealthMonitorWidget::onBackClicked()
{
    keep_running.store(false); // 设置停止标志
    
    // 停止定时器
    if (connectionTimer_) connectionTimer_->stop();
    if (dataTimer_) dataTimer_->stop();
    
    // 发射返回信号
    emit backRequested();
}

void HealthMonitorWidget::updateDisplay(QString text)
{
    // 在主线程中更新UI
    dataDisplay_->append(text);
    
    // 自动滚动到底部
    dataDisplay_->moveCursor(QTextCursor::End);
    
    // 限制显示内容长度，避免内存占用过大
    if (dataDisplay_->document()->blockCount() > 100) {
        QTextCursor cursor = dataDisplay_->textCursor();
        cursor.movePosition(QTextCursor::Start);
        cursor.movePosition(QTextCursor::Down, QTextCursor::KeepAnchor, 20);
        cursor.removeSelectedText();
    }
}

void HealthMonitorWidget::updateStatusIndicators(int heartRate, int spo2, int sdnn, bool userDetected)
{
    // 更新心率
    heartRateLabel_->setText(QString("心率: %1 bpm").arg(heartRate));
    if (heartRate > 0 && heartRate <= 120) {
        heartRateBar_->setValue(heartRate);
        if (heartRate < 60 || heartRate > 100) {
            heartRateBar_->setStyleSheet(
                "QProgressBar::chunk { background-color: #f39c12; }"
            );
        } else {
            heartRateBar_->setStyleSheet(
                "QProgressBar::chunk { background-color: #2ecc71; }"
            );
        }
    }
    
    // 更新血氧
    spo2Label_->setText(QString("血氧: %1 %%").arg(spo2));
    if (spo2 >= 90 && spo2 <= 100) {
        spo2Bar_->setValue(spo2);
        if (spo2 < 95) {
            spo2Bar_->setStyleSheet(
                "QProgressBar::chunk { background-color: #e74c3c; }"
            );
        } else {
            spo2Bar_->setStyleSheet(
                "QProgressBar::chunk { background-color: #2ecc71; }"
            );
        }
    }
    
    // 更新SDNN
    sdnnLabel_->setText(QString("SDNN: %1").arg(sdnn));
    
    // 更新用户检测状态
    userDetectedLabel_->setText(QString("用户检测: %1").arg(userDetected ? "是" : "否"));
    if (userDetected) {
        userDetectedLabel_->setStyleSheet("QLabel { font-size: 14px; font-weight: bold; color: #2ecc71; }");
    } else {
        userDetectedLabel_->setStyleSheet("QLabel { font-size: 14px; font-weight: bold; color: #e74c3c; }");
    }
}

void HealthMonitorWidget::showConnectionStatus(bool connected)
{
    isConnected_ = connected;
    connectionTimer_->stop();
    
    if (connected) {
        connectionStatusLabel_->setText("设备已连接");
        connectionStatusLabel_->setStyleSheet(
            "QLabel {"
            "    font-size: 12px;"
            "    color: #27ae60;"
            "    padding: 5px 10px;"
            "    border-radius: 10px;"
            "    background-color: #d5f4e6;"
            "}"
        );
    } else {
        connectionStatusLabel_->setText("设备未连接");
        connectionStatusLabel_->setStyleSheet(
            "QLabel {"
            "    font-size: 12px;"
            "    color: #e74c3c;"
            "    padding: 5px 10px;"
            "    border-radius: 10px;"
            "    background-color: #fadbd8;"
            "}"
        );
    }
}

void HealthMonitorWidget::onConnectionTimeout()
{
    if (!isConnected_) {
        dataDisplay_->append("连接超时，请检查设备连接");
        emit connectionStatusChanged(false);
    }
}

void HealthMonitorWidget::onDataTimeout()
{
    if (!isDataReceiving_) {
        dataDisplay_->append("警告：长时间未接收到数据，请检查设备状态");
    }
    isDataReceiving_ = false; // 重置标志，下次检查
}
