#include "healthWidget.h"
#include <QTextCursor>
#include <QMetaObject>
#include <QPainter>
#include <QPaintEvent>
#include <QTimer>
#include <QFont>
#include <QtGlobal>  // for qMin, qMax
#include <QDebug>    // for qDebug() - 调试输出
#include <iostream>  // for std::cout - 终端输出

// healthMnter::healthMnter(ros::NodeHandle &nh, PageStateWidget *parent) :
healthMnter::healthMnter(PageStateWidget *parent) :
    QObject(parent), 
    parentWidget(parent),
    monitor(new HealthMonitor(DEVICE_PORT)),
    // person_data_pub(nh.advertise<qt_debug::personData>("/person_data", 10)),
    chartUpdateTimer(new QTimer(this))
{
    // 缓存UI控件指针
    btnSwitch = parentWidget->findChild<QPushButton*>("BtnSwitchState");
    labelState = parentWidget->findChild<QLabel*>("label_state");
    labelHealthMsg = parentWidget->findChild<QLabel*>("label_healthMsg");
    paintWidget = parentWidget->findChild<QWidget*>("acPaintWidget");
    
    // 连接信号和槽（异步调用，用于线程安全的UI更新）
    connect(this, &healthMnter::statusUpdate, this, &healthMnter::updateStatusDisplay, Qt::QueuedConnection);
    connect(this, &healthMnter::debugUpdate, this, &healthMnter::updateDebugDisplay, Qt::QueuedConnection);
    connect(this, &healthMnter::heartRateUpdate, this, &healthMnter::updateHeartRateChart, Qt::QueuedConnection);
    connect(this, &healthMnter::healthDataUpdate, this, &healthMnter::updateHealthData, Qt::QueuedConnection);
    
    // 连接开始采集按钮点击信号与槽函数
    if (btnSwitch) {
        connect(btnSwitch, &QPushButton::clicked, this, &healthMnter::onStartClicked);
    }

    // 初始化UI显示
    if (labelState) {
        labelState->setText("点击开始采集");
        QFont font = labelState->font();
        font.setPointSize(10);          // 设置字体大小
        font.setBold(true);
        labelState->setFont(font);
        labelState->setWordWrap(true);
        labelState->setAlignment(Qt::AlignCenter);
        // 修改为灰色主题样式
        labelState->setStyleSheet(
            "QLabel {"
            "   background-color: #595857;"     // 背景
            "   color: #afafb0;"                // 文字颜色
            "   border: 2px solid #595455;"     // 边框
            "   border-radius: 10px;"
            "   padding: 10px;"
            "   font-weight: bold;"
            "}"
        );
    }
    if (labelHealthMsg) {
        labelHealthMsg->setText("等待采集生理数据...");
        QFont font = labelHealthMsg->font();
        font.setPointSize(11);
        labelHealthMsg->setFont(font);
        labelHealthMsg->setWordWrap(true);
        labelHealthMsg->setAlignment(Qt::AlignCenter);  // 改为居中对齐
        labelHealthMsg->setTextFormat(Qt::RichText);
        labelHealthMsg->setScaledContents(false);   // 防止内容变化时拉伸
        // 修改为灰色主题渐变
        labelHealthMsg->setStyleSheet(
            "QLabel {"
            "   background: qlineargradient(x1:0, y1:0, x2:0, y2:1, "
            "               stop:0 #595857, stop:1 #595455);"       // 渐变背景
            "   color: #afafb0;"                                      // 文字颜色
            "   border: 1px solid #595455;"                           // 边框
            "   border-radius: 8px;"
            "   padding: 12px;"
            "   line-height: 1.4;"
            "}"
        );
    }
    
    // 设置心率绘图区域的绘制事件
    if (paintWidget) {
        paintWidget->installEventFilter(this);
        // 修改为灰色主题与边框
        paintWidget->setStyleSheet(
            "QWidget {"
            "   background-color: #595857;"     // 背景
            "   border: 2px solid #595455;"     // 边框
            "   border-radius: 8px;"
            "   margin: 2px;"
            "}"
        );
    }
    
    // 初始化按钮状态
    if (btnSwitch) {
        btnSwitch->setChecked(false);
        btnSwitch->setText("开始采集");
    }
    
    // 设置图表更新定时器
    chartUpdateTimer->setInterval(100); // 100ms更新一次
    connect(chartUpdateTimer, &QTimer::timeout, [this]() {
        if (paintWidget) {
            paintWidget->update();
        }
    });
}

healthMnter::~healthMnter()
{
    keep_running.store(false);
    if (monitor) {
        monitor->stopCollection();
        delete monitor;
    }
}

// 开始采集按钮点击槽函数
void healthMnter::onStartClicked()
{
    if (!btnSwitch) return;
    
    if (btnSwitch->isChecked()) { //如果按键被按下
        // 开始采集
        btnSwitch->setText("停止采集");
        keep_running.store(true);
        
        if (labelState) {
            labelState->setText("正在连接设备...");
        }
        
        try {
            // 尝试连接设备
            if (!monitor->connect()) {  // 连接失败
                emit statusUpdate("连接失败\n请检查设备连接");
                btnSwitch->setChecked(false);
                btnSwitch->setText("开始采集");
                keep_running.store(false);
                return;
            }
            
            emit statusUpdate("设备连接成功\n开始采集数据...");
            chartUpdateTimer->start();      // 图表更新定时器启动
            
            // 启动数据采集
            monitor->startCollection([this](const realtime_packet_t& data) {
                if (!keep_running.load()) return;   // 如果停止采集则忽略数据
                processHealthData(data);    // 核心数据处理函数
            });
            
        } catch (const std::exception& e) {
            emit statusUpdate(QString("连接异常\n%1").arg(e.what()));
            btnSwitch->setChecked(false);
            btnSwitch->setText("开始采集");
            keep_running.store(false);
        }
    } else {
        // 停止采集
        btnSwitch->setText("开始采集");
        keep_running.store(false);
        chartUpdateTimer->stop();
        
        if (monitor) {
            monitor->stopCollection();
        }
        
        emit statusUpdate("采集已停止");
    }
}

// 处理采集到的健康数据
void healthMnter::processHealthData(const realtime_packet_t& data)
{
    // 更新label_state状态信息（简洁提示）
    emit statusUpdate("正在采集\n请将大拇指轻触传感器\n等待数秒后数值稳定");
    
    // 格式化调试信息（不显示在UI上，保留用于将来扩展）
    QString debugInfo;
    debugInfo += "=== 健康监测数据 ===\n";
    debugInfo += QString("用户: %1 | 心率: %2 bpm | SpO2: %3%%\n")
                .arg(is_user_detected(data.state) ? "✓" : "✗")
                .arg(data.heartrate)
                .arg(data.spo2);
    debugInfo += QString("SDNN: %1 | 疲劳: %2\n")
                .arg(data.sdnn)
                .arg(get_fatigue_status(data.sdnn));
    debugInfo += QString("RMSSD: %1 | NN50: %2 | pNN50: %3\n")
                .arg(data.rmssd)
                .arg(data.nn50)
                .arg(data.pnn50);
    
    // 处理 RR 间期数据（简化显示）
    auto rr_points = monitor->getRRScatterPoints(data);
    if (!rr_points.empty()) {
        debugInfo += QString("RR点数: %1 个\n").arg(rr_points.size());
        // 只显示前3个点，避免信息过多
        for (size_t i = 0; i < qMin(size_t(3), rr_points.size()); ++i) {
            debugInfo += QString("(%1,%2) ").arg(rr_points[i].first).arg(rr_points[i].second);
        }
        if (rr_points.size() > 3) {
            debugInfo += "...";
        }
        debugInfo += "\n";
    }
    
    // 更新心率图表数据
    QVector<QPointF> chartData;
    for (int i = 0; i < 64; ++i) {
        chartData.append(QPointF(i, data.acdata[i]));
    }
    emit heartRateUpdate(chartData);    // 发送心率数据更新信号
    
    // ============= 调试信息：输出心律波形数据到终端 =============
    std::cout << "\n=== 心律波形数据 (acdata[64]) ===" << std::endl;
    std::cout << "数据包头: 0x" << std::hex << (int)data.header << std::dec << std::endl;
    std::cout << "心律波形: ";
    for (int i = 0; i < 64; ++i) {
        std::cout << (int)data.acdata[i];
        if (i < 63) std::cout << ", ";
        // 每16个数据换行，便于阅读
        if ((i + 1) % 16 == 0) std::cout << "\n          ";
    }
    std::cout << std::endl;
    
    // 计算波形统计信息
    int min_val = data.acdata[0], max_val = data.acdata[0];
    long sum = 0;
    for (int i = 0; i < 64; ++i) {
        if (data.acdata[i] < min_val) min_val = data.acdata[i];
        if (data.acdata[i] > max_val) max_val = data.acdata[i];
        sum += data.acdata[i];
    }
    double avg = sum / 64.0;
    std::cout << "波形统计: 最小值=" << min_val << ", 最大值=" << max_val 
              << ", 平均值=" << avg << std::endl;
    std::cout << "============================\n" << std::endl;
    // ============= 调试信息结束 ============

    // 格式化生理数据并发送到label_healthMsg
    QString healthInfo = formatHealthMessage(data);
    emit healthDataUpdate(healthInfo);  // 发送健康数据更新信号
    
    // // 发布ROS消息
    // qt_debug::personData p;
    // p.header = data.header;
    // for (size_t i = 0; i < 64; ++i) p.acdata[i] = data.acdata[i];
    // p.heartrate = data.heartrate;
    // p.spo2 = data.spo2; 
    // p.bk = data.bk;
    // for (size_t i = 0; i < 8; ++i) p.rsv1[i] = data.rsv1[i];
    // p.sdnn = data.sdnn;
    // p.rmssd = data.rmssd;
    // p.nn50 = data.nn50;
    // p.pnn50 = data.pnn50;
    // for (size_t i = 0; i < 6; ++i) p.rra[i] = data.rra[i];
    // p.rsv2 = data.rsv2;
    // p.state = data.state;
    
    // person_data_pub.publish(p);
}

// 更新状态显示（label_state）
void healthMnter::updateStatusDisplay(QString text)
{
    if (labelState) {
        labelState->setText(text);
        
        QString styleSheet;
        if (text.contains("未连接") || text.contains("失败")) {
            // 错误状态
            styleSheet = 
               "QLabel {"
                "   background-color: #595857;"     // 背景
                "   color: #afafb0;"                // 文字颜色
                "   border: 2px solid #595455;"     // 边框
                "   border-radius: 10px;"             // 圆角
                "   padding: 10px;"                   // 内边距
                "   font-weight: bold;"               // 设置是否粗体
                "}";
        } else if (text.contains("正在采集") || text.contains("采集中")) {
            // 采集状态
            styleSheet = 
                "QLabel {"
                "   background: qlineargradient(x1:0, y1:0, x2:0, y2:1, "
                "               stop:0 #595857, stop:1 #595455);"
                "   color: #afafb0;"
                "   border: 2px solid #595455;"
                "   border-radius: 10px;"
                "   padding: 10px;"
                "   font-weight: bold;"
                "}";
        } else if (text.contains("连接") || text.contains("开始")) {
            // 连接状态
            styleSheet = 
                "QLabel {"
                "   background: qlineargradient(x1:0, y1:0, x2:0, y2:1, "
                "               stop:0 #595857, stop:1 #595455);"
                "   color: #afafb0;"
                "   border: 2px solid #595455;"
                "   border-radius: 10px;"
                "   padding: 10px;"
                "   font-weight: bold;"
                "}";
        } else {
            // 默认状态：标准灰色主题
                styleSheet = 
                "QLabel {"
                "   background-color: #595857;"     // 背景
                "   color: #afafb0;"                // 文字颜色
                "   border: 2px solid #595455;"     // 边框
                "   border-radius: 10px;"
                "   padding: 10px;"
                "   font-weight: bold;"
                "}";
        }
        labelState->setStyleSheet(styleSheet);
    }
}

void healthMnter::updateDebugDisplay(QString text)
{
    // 这个函数现在不使用，保留以防未来需要
    // 如果需要额外的调试显示控件可以在这里添加
}

// 更新心率图表数据
void healthMnter::updateHeartRateChart(QVector<QPointF> points)
{
    heartRatePoints = points;
    if (paintWidget) {
        paintWidget->update();
    }
}

// 更新健康数据显示（label_healthMsg）
void healthMnter::updateHealthData(QString healthInfo)
{
    // label_healthMsg 显示采集到的生理数据内容
    if (labelHealthMsg) {
        labelHealthMsg->setText(healthInfo);
    }
}

// 事件过滤器，用于捕获绘图事件
bool healthMnter::eventFilter(QObject *obj, QEvent *event)
{
    if (obj == paintWidget && event->type() == QEvent::Paint) {
        QPainter painter(paintWidget);
        drawHeartRateChart(painter, paintWidget->rect());
        return true;
    }
    return QObject::eventFilter(obj, event);
}

// 绘制心率图表
void healthMnter::drawHeartRateChart(QPainter& painter, const QRect& rect)
{
    painter.setRenderHint(QPainter::Antialiasing);
    
    // 绘制圆润的背景
    painter.setBrush(QBrush(QColor(0x2b, 0x2b, 0x2b)));
    painter.setPen(Qt::NoPen);
    painter.drawRoundedRect(rect.adjusted(2, 2, -2, -2), 12, 12);
    
    if (heartRatePoints.isEmpty()) {
        painter.setPen(QPen(QColor(255, 255, 255), 1));
        QFont font = painter.font();
        font.setPointSize(12);
        painter.setFont(font);
        painter.drawText(rect, Qt::AlignCenter, "等待心率数据...");
        return;
    }
    
    // 找到数据的最大值和最小值，用于更好的缩放
    qreal minVal = heartRatePoints[0].y();
    qreal maxVal = heartRatePoints[0].y();
    for (const auto& point : heartRatePoints) {
        minVal = qMin(minVal, point.y());
        maxVal = qMax(maxVal, point.y());
    }
    
    // 避免除零错误，并添加一些边距
    qreal range = maxVal - minVal;
    if (range < 1.0) range = 1.0;
    
    // 计算缩放因子，留出10%的边距
    qreal xScale = rect.width() / qreal(qMax(1, heartRatePoints.size() - 1));
    qreal yScale = rect.height() * 0.8 / range;  // 使用80%的高度
    qreal yOffset = rect.height() * 0.1;  // 10%的上下边距
    
    // 绘制背景网格线（更细致的网格）
    painter.setPen(QPen(QColor(100, 100, 100, 50), 1, Qt::DotLine));
    // 水平网格线
    for (int i = 1; i < 8; ++i) {
        int y = rect.height() * i / 8;
        painter.drawLine(0, y, rect.width(), y);
    }
    // 垂直网格线
    for (int i = 1; i < 12; ++i) {
        int x = rect.width() * i / 12;
        painter.drawLine(x, 0, x, rect.height());
    }
    
    // 绘制主要网格线（更明显）
    painter.setPen(QPen(QColor(150, 150, 150, 80), 1, Qt::DotLine));
    for (int i = 1; i < 4; ++i) {
        int y = rect.height() * i / 4;
        painter.drawLine(0, y, rect.width(), y);
    }
    for (int i = 1; i < 6; ++i) {
        int x = rect.width() * i / 6;
        painter.drawLine(x, 0, x, rect.height());
    }
    
    // 绘制心率曲线
    painter.setPen(QPen(QColor(220, 220, 220), 1));   // 灰白色
    for (int i = 1; i < heartRatePoints.size(); ++i) {
        QPointF p1(heartRatePoints[i-1].x() * xScale, 
                   rect.height() - yOffset - (heartRatePoints[i-1].y() - minVal) * yScale);
        QPointF p2(heartRatePoints[i].x() * xScale, 
                   rect.height() - yOffset - (heartRatePoints[i].y() - minVal) * yScale);
        painter.drawLine(p1, p2);
    }
    
    // 添加标题和信息
    painter.setPen(QPen(QColor(255, 255, 255), 1));
    QFont titleFont = painter.font();
    titleFont.setPointSize(10);
    titleFont.setBold(true);
    painter.setFont(titleFont);
    painter.drawText(10, 20, "实时心率波形");
    
    // 添加数值范围信息
    QFont infoFont = painter.font();
    infoFont.setPointSize(8);
    painter.setFont(infoFont);
    painter.drawText(10, rect.height() - 10, QString("范围: %1 - %2").arg(int(minVal)).arg(int(maxVal)));
}

// 格式化健康数据为HTML字符串
QString healthMnter::formatHealthMessage(const realtime_packet_t& data)
{
    const QString HTML_TAB = "&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;";        // 定义HTML非断行空格宏

    QString msg;
    msg += "<style>";
    msg += "body { font-family: 'Arial', sans-serif; margin:0; padding:0; }";
    msg += ".header { color:#afafb0; font-size:22px; font-weight:bold; margin:0 0 6px 0; text-align:center; }";  // 标题样式
    msg += ".row { font-size:17px; margin:6px 0; line-height:1.3; }";                                              // 行样式
    msg += ".item { display:inline-block; width:30%; text-align:left; margin-right:3%; }";                         // 每个项目样式
    msg += ".warning { color:#a22041; font-weight:bold; }";                                                        // 警告样式              
    msg += ".normal { color:#88cb7f; font-weight:bold; }";                                                         // 正常样式 
    msg += ".value { font-size:15px !important; font-weight:bold; color:#88cb7f !important; }";                     // 数值样式，使用!important强制应用
    // msg += ".separator { border-top: 2px solid #595455; margin: 5px 0; height: 2px; }";                          // 分隔线样式 - 使用更明显的颜色
    msg += "</style>";

    // ========== 上半部分：生命体征 ==========
    msg += "<div class='header'>生命体征</div>";

    // 第一行：心率 + 血氧 + 血压
    QString heartRateClass = (data.heartrate < 60 || data.heartrate > 100) ? "warning" : "normal";
    QString spo2Class = (data.spo2 < 95) ? "warning" : "normal";
    QString bkClass = (data.bk < 90 || data.bk > 140) ? "warning" : "normal";
    msg += "<div class='row'>";
    msg += QString("<span class='item'>心率: <span class='%1'>%2</span> bpm</span>").arg(heartRateClass).arg(data.heartrate);
    msg += HTML_TAB;
    msg += QString("<span class='item'>血氧: <span class='%1'>%2</span>%</span>").arg(spo2Class).arg(data.spo2);
    msg += HTML_TAB;
    msg += QString("<span class='item'>血压: <span class='%1'>%2</span> mmHg</span>").arg(bkClass).arg(data.bk);
    msg += "</div>";

    // 第二行：用户检测 + 疲劳状态
    bool userDetected = is_user_detected(data.state);
    QString userClass = userDetected ? "normal" : "warning";
    QString userIcon = userDetected ? "√" : "X";
    QString fatigueStatus = get_fatigue_status(data.sdnn);
    QString fatigueClass = (fatigueStatus.contains("疲劳") || fatigueStatus.contains("高")) ? "warning" : "normal";
    msg += "<div class='row'>";
    msg += QString("<span class='item'>%1 用户检测: <span class='%2'>%3</span></span>")
           .arg(userIcon).arg(userClass).arg(userDetected ? "已检测" : "未检测");
    msg += HTML_TAB;
    msg += QString("<span class='item'>疲劳状态: <span class='%1'>%2</span></span>").arg(fatigueClass).arg(fatigueStatus);
    msg += "</div>";

    // 分隔线 - 使用HR标签替代CSS边框
    msg += "<hr style='border: none; border-top: 2px solid #595455; margin: 2px 0;'>";

    // ========== 下半部分：心率变异性 ==========
    msg += "<div class='header'>心率变异性</div>";

    // 第一行：SDNN + RMSSD + NN50
    msg += "<div class='row'>";
    msg += QString("<span class='item'>SDNN: <span class='value'>%1</span> ms</span>").arg(data.sdnn);
    msg += HTML_TAB;
    msg += QString("<span class='item'>RMSSD: <span class='value'>%1</span> ms</span>").arg(data.rmssd);
    msg += "</div>";

    // 第二行：pNN50
    msg += "<div class='row'>";
    msg += QString("<span class='item'>pNN50: <span class='value'>%1</span>%</span>").arg(data.pnn50);
    msg += HTML_TAB;
    msg += QString("<span class='item'>NN50: <span class='value'>%1</span> count</span>").arg(data.nn50);
    msg += "</div>";

    return msg;
}
