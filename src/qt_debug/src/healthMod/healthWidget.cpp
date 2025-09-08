#include "healthWidget.h"
#include <QTextCursor>
#include <QMetaObject>
#include <QPainter>
#include <QPaintEvent>
#include <QTimer>
#include <QFont>
#include <QtGlobal>  // for qMin, qMax

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
    
    // 连接信号和槽（用于线程安全的UI更新）
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
        labelState->setText("设备未连接");
        // 设置适中的字体用于状态显示
        QFont font = labelState->font();
        font.setPointSize(12);  // 稍大的字体
        font.setBold(true);     // 加粗字体
        labelState->setFont(font);
        labelState->setWordWrap(true);  // 允许文字换行
        labelState->setAlignment(Qt::AlignCenter);  // 状态信息居中显示
        // 美化样式：圆角边框和背景色
        labelState->setStyleSheet(
            "QLabel {"
            "   background-color: #1d1d22ff;"
            "   color: #ECF0F1;"
            "   border: 2px solid #1e1f22ff;"
            "   border-radius: 10px;"
            "   padding: 10px;"
            "   font-weight: bold;"
            "}"
        );
    }
    if (labelHealthMsg) {
        labelHealthMsg->setText("等待采集生理数据...");
        // 设置适中的字体用于显示生理数据
        QFont font = labelHealthMsg->font();
        font.setPointSize(11);  // 生理数据用稍小字体
        labelHealthMsg->setFont(font);
        labelHealthMsg->setWordWrap(true);  // 允许文字换行
        labelHealthMsg->setAlignment(Qt::AlignTop | Qt::AlignLeft);  // 顶部左对齐
        // 启用富文本格式支持
        labelHealthMsg->setTextFormat(Qt::RichText);
        // 美化样式：渐变背景和阴影效果
        labelHealthMsg->setStyleSheet(
            "QLabel {"
            "   background: qlineargradient(x1:0, y1:0, x2:0, y2:1, "
            "               stop:0 #F8F9FA, stop:1 #E9ECEF);"
            "   color: #2C3E50;"
            "   border: 1px solid #DEE2E6;"
            "   border-radius: 8px;"
            "   padding: 12px;"
            "   line-height: 1.4;"
            "}"
        );
    }
    
    // 设置心率绘图区域的绘制事件
    if (paintWidget) {
        paintWidget->installEventFilter(this);
        // 美化绘图区域样式：深色主题与发光边框
        paintWidget->setStyleSheet(
            "QWidget {"
            "   background-color: #cacacaff;"
            "   border: 2px solid #00FF7F;"
            "   border-radius: 8px;"
            "   margin: 2px;"
            "}"
        );
        // 移除额外的尺寸设置，使用UI文件中的设置
        // paintWidget->setMinimumSize(300, 200);
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

void healthMnter::onStartClicked()
{
    if (!btnSwitch) return;
    
    if (btnSwitch->isChecked()) {
        // 开始采集
        btnSwitch->setText("停止采集");
        keep_running.store(true);
        
        if (labelState) {
            labelState->setText("正在连接设备...");
        }
        
        try {
            // 尝试连接设备
            if (!monitor->connect()) {
                emit statusUpdate("连接失败\n请检查设备连接");
                btnSwitch->setChecked(false);
                btnSwitch->setText("开始采集");
                keep_running.store(false);
                return;
            }
            
            emit statusUpdate("设备连接成功\n开始采集数据...");
            chartUpdateTimer->start();
            
            // 启动数据采集
            monitor->startCollection([this](const realtime_packet_t& data) {
                if (!keep_running.load()) return;
                processHealthData(data);
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

void healthMnter::processHealthData(const realtime_packet_t& data)
{
    // 更新label_state状态信息（简洁提示）
    emit statusUpdate("正在采集\n请将手指轻触传感器等待数秒");
    
    // 格式化调试信息（但不显示在UI上，保留用于将来扩展）
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
    emit heartRateUpdate(chartData);
    
    // 格式化生理数据并发送到label_healthMsg
    QString healthInfo = formatHealthMessage(data);
    emit healthDataUpdate(healthInfo);
    
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

void healthMnter::updateStatusDisplay(QString text)
{
    // label_state 显示设备状态、调试信息和提示
    if (labelState) {
        labelState->setText(text);
        
        // 根据状态文本动态调整样式
        QString styleSheet;
        if (text.contains("未连接") || text.contains("失败")) {
            // 错误状态：红色主题
            styleSheet = 
                "QLabel {"
                "   background-color: #E74C3C;"
                "   color: white;"
                "   border: 2px solid #C0392B;"
                "   border-radius: 10px;"
                "   padding: 10px;"
                "   font-weight: bold;"
                "}";
        } else if (text.contains("正在采集") || text.contains("采集中")) {
            // 采集状态：绿色主题
            styleSheet = 
                "QLabel {"
                "   background: qlineargradient(x1:0, y1:0, x2:0, y2:1, "
                "               stop:0 #27AE60, stop:1 #229954);"
                "   color: white;"
                "   border: 2px solid #1E8449;"
                "   border-radius: 10px;"
                "   padding: 10px;"
                "   font-weight: bold;"
                "}";
        } else if (text.contains("连接") || text.contains("开始")) {
            // 连接状态：蓝色主题
            styleSheet = 
                "QLabel {"
                "   background: qlineargradient(x1:0, y1:0, x2:0, y2:1, "
                "               stop:0 #3498DB, stop:1 #2980B9);"
                "   color: white;"
                "   border: 2px solid #1F618D;"
                "   border-radius: 10px;"
                "   padding: 10px;"
                "   font-weight: bold;"
                "}";
        } else {
            // 默认状态：深色主题
            styleSheet = 
                "QLabel {"
                "   background-color: #2C3E50;"
                "   color: #ECF0F1;"
                "   border: 2px solid #34495E;"
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

void healthMnter::updateHeartRateChart(QVector<QPointF> points)
{
    heartRatePoints = points;
    if (paintWidget) {
        paintWidget->update();
    }
}

void healthMnter::updateHealthData(QString healthInfo)
{
    // label_healthMsg 显示采集到的生理数据内容
    if (labelHealthMsg) {
        labelHealthMsg->setText(healthInfo);
    }
}

bool healthMnter::eventFilter(QObject *obj, QEvent *event)
{
    if (obj == paintWidget && event->type() == QEvent::Paint) {
        QPainter painter(paintWidget);
        drawHeartRateChart(painter, paintWidget->rect());
        return true;
    }
    return QObject::eventFilter(obj, event);
}

void healthMnter::drawHeartRateChart(QPainter& painter, const QRect& rect)
{
    // 绘制深色背景
    painter.fillRect(rect, QColor(26, 26, 26));
    
    if (heartRatePoints.isEmpty()) {
        painter.setPen(QPen(QColor(255, 255, 255, 100), 1));
        QFont font = painter.font();
        font.setPointSize(12);
        painter.setFont(font);
        painter.drawText(rect, Qt::AlignCenter, "等待心率数据...");
        return;
    }
    
    painter.setRenderHint(QPainter::Antialiasing);
    
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
    
    // 绘制心率曲线（带发光效果）
    // 外层发光效果
    painter.setPen(QPen(QColor(0, 255, 127, 50), 6));
    for (int i = 1; i < heartRatePoints.size(); ++i) {
        QPointF p1(heartRatePoints[i-1].x() * xScale, 
                   rect.height() - yOffset - (heartRatePoints[i-1].y() - minVal) * yScale);
        QPointF p2(heartRatePoints[i].x() * xScale, 
                   rect.height() - yOffset - (heartRatePoints[i].y() - minVal) * yScale);
        painter.drawLine(p1, p2);
    }
    
    // 主要曲线
    painter.setPen(QPen(QColor(0, 255, 127), 3));
    for (int i = 1; i < heartRatePoints.size(); ++i) {
        QPointF p1(heartRatePoints[i-1].x() * xScale, 
                   rect.height() - yOffset - (heartRatePoints[i-1].y() - minVal) * yScale);
        QPointF p2(heartRatePoints[i].x() * xScale, 
                   rect.height() - yOffset - (heartRatePoints[i].y() - minVal) * yScale);
        painter.drawLine(p1, p2);
    }
    
    // 内层高亮
    painter.setPen(QPen(QColor(255, 255, 255, 200), 1));
    for (int i = 1; i < heartRatePoints.size(); ++i) {
        QPointF p1(heartRatePoints[i-1].x() * xScale, 
                   rect.height() - yOffset - (heartRatePoints[i-1].y() - minVal) * yScale);
        QPointF p2(heartRatePoints[i].x() * xScale, 
                   rect.height() - yOffset - (heartRatePoints[i].y() - minVal) * yScale);
        painter.drawLine(p1, p2);
    }
    
    // 添加标题和信息
    painter.setPen(QPen(QColor(255, 255, 255, 180), 1));
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

QString healthMnter::formatHealthMessage(const realtime_packet_t& data)
{
    QString msg;
    
    // 使用HTML格式化，提供更丰富的显示效果
    msg += "<style>";
    msg += "body { font-family: 'Arial', sans-serif; line-height: 1.6; }";
    msg += ".header { color: #2C3E50; font-size: 14px; font-weight: bold; margin-bottom: 8px; }";
    msg += ".vital { color: #27AE60; font-size: 13px; margin: 4px 0; }";
    msg += ".warning { color: #E74C3C; font-weight: bold; }";
    msg += ".normal { color: #2ECC71; font-weight: bold; }";
    msg += ".value { font-weight: bold; color: #3498DB; }";
    msg += "</style>";
    
    msg += "<div class='header'>📊 生理参数监测</div>";
    
    // 心率显示（根据范围添加颜色）
    QString heartRateClass = (data.heartrate < 60 || data.heartrate > 100) ? "warning" : "normal";
    msg += QString("<div class='vital'>💓 心率: <span class='%1'>%2</span> bpm</div>")
           .arg(heartRateClass).arg(data.heartrate);
    
    // 血氧显示
    QString spo2Class = (data.spo2 < 95) ? "warning" : "normal";
    msg += QString("<div class='vital'>🩸 血氧: <span class='%1'>%2</span>%</div>")
           .arg(spo2Class).arg(data.spo2);
    
    msg += "<hr style='border: 1px solid #BDC3C7; margin: 8px 0;'>";
    msg += "<div class='header'>📈 心率变异性分析</div>";
    
    // HRV参数
    msg += QString("<div class='vital'>📊 SDNN: <span class='value'>%1</span> ms</div>").arg(data.sdnn);
    msg += QString("<div class='vital'>⚡ RMSSD: <span class='value'>%1</span> ms</div>").arg(data.rmssd);
    msg += QString("<div class='vital'>📈 NN50: <span class='value'>%1</span></div>").arg(data.nn50);
    msg += QString("<div class='vital'>📉 pNN50: <span class='value'>%1</span>%</div>").arg(data.pnn50);
    
    msg += "<hr style='border: 1px solid #BDC3C7; margin: 8px 0;'>";
    
    // 疲劳状态
    QString fatigueStatus = get_fatigue_status(data.sdnn);
    QString fatigueClass = (fatigueStatus.contains("疲劳") || fatigueStatus.contains("高")) ? "warning" : "normal";
    msg += QString("<div class='vital'>� 疲劳状态: <span class='%1'>%2</span></div>")
           .arg(fatigueClass).arg(fatigueStatus);
    
    // 用户检测状态
    bool userDetected = is_user_detected(data.state);
    QString userClass = userDetected ? "normal" : "warning";
    QString userIcon = userDetected ? "✅" : "❌";
    msg += QString("<div class='vital'>%1 用户检测: <span class='%2'>%3</span></div>")
           .arg(userIcon).arg(userClass).arg(userDetected ? "已检测" : "未检测");
    
    return msg;
}
