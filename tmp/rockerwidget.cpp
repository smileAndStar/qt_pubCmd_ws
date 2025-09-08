#include "rockerwidget.h"
#include <QDebug>

RockerWidget::RockerWidget(BtnController *btnController, QWidget *parent) :
    QWidget(parent),
    btnController_(btnController),
    MousePressFlag(false),
    stackedWidget_(nullptr),
    pageIndex_(-1)
{
    setupUI();
    initRocker();
    
    // 设置摇杆控制状态
    if (btnController_) {
        btnController_->stateRockerControl();
    }
    
    // 创建状态检查定时器
    statusCheckTimer_ = new QTimer(this);
    connect(statusCheckTimer_, &QTimer::timeout, this, &RockerWidget::checkPageStatus);
    statusCheckTimer_->start(100); // 每100ms检查一次页面状态
}

RockerWidget::~RockerWidget()
{
    // 切换到停止状态，保险起见
    if (btnController_) {
        btnController_->stopMovement();
    }
}

void RockerWidget::setupUI()
{
    // 创建主布局
    mainLayout_ = new QVBoxLayout(this);
    
    // 创建标题标签
    titleLabel_ = new QLabel("摇杆控制器", this);
    titleLabel_->setAlignment(Qt::AlignCenter);
    titleLabel_->setStyleSheet("QLabel { font-size: 18px; font-weight: bold; margin: 10px; }");
    
    // 创建摇杆区域（这个widget用于摇杆绘制）
    rockerArea_ = new QWidget(this);
    rockerArea_->setMinimumSize(BIG_CIRCLE_RADIUS * 2 + 40, BIG_CIRCLE_RADIUS * 2 + 40);
    rockerArea_->setStyleSheet("QWidget { background-color: #f0f0f0; border-radius: 10px; }");
    
    // 创建状态标签
    statusLabel_ = new QLabel("移动摇杆控制机器人方向", this);
    statusLabel_->setAlignment(Qt::AlignCenter);
    statusLabel_->setStyleSheet("QLabel { color: #666; font-size: 14px; }");
    
    // 添加到主布局
    mainLayout_->addWidget(titleLabel_);
    mainLayout_->addWidget(rockerArea_, 1); // 摇杆区域占主要空间
    mainLayout_->addWidget(statusLabel_);
    
    // 设置布局边距
    mainLayout_->setContentsMargins(20, 20, 20, 20);
    mainLayout_->setSpacing(15);
}

void RockerWidget::initRocker()
{
    // 在构造时加载图片，提高效率
    // 加载大圆图片
    if (!big_circle_pixmap_.load(":/images/ring.png")) {
        qDebug() << "Failed to load ring.png";
        // 创建默认圆形作为备用
        big_circle_pixmap_ = QPixmap(BIG_CIRCLE_RADIUS * 2, BIG_CIRCLE_RADIUS * 2);
        big_circle_pixmap_.fill(Qt::transparent);
        QPainter painter(&big_circle_pixmap_);
        painter.setRenderHint(QPainter::Antialiasing);
        painter.setPen(QPen(Qt::gray, 3));
        painter.setBrush(QBrush(Qt::lightGray, Qt::SolidPattern));
        painter.drawEllipse(0, 0, BIG_CIRCLE_RADIUS * 2, BIG_CIRCLE_RADIUS * 2);
    }
    
    // 加载小圆图片
    if (!small_circle_pixmap_.load(":/images/circle.png")) {
        qDebug() << "Failed to load circle.png";
        // 创建默认圆形作为备用
        small_circle_pixmap_ = QPixmap(SMALL_CIRCLE_RADIUS * 2, SMALL_CIRCLE_RADIUS * 2);
        small_circle_pixmap_.fill(Qt::transparent);
        QPainter painter(&small_circle_pixmap_);
        painter.setRenderHint(QPainter::Antialiasing);
        painter.setPen(QPen(Qt::darkGray, 2));
        painter.setBrush(QBrush(Qt::blue, Qt::SolidPattern));
        painter.drawEllipse(0, 0, SMALL_CIRCLE_RADIUS * 2, SMALL_CIRCLE_RADIUS * 2);
    }
    
    // 缩放图片到合适大小
    big_circle_pixmap_ = big_circle_pixmap_.scaled(BIG_CIRCLE_RADIUS * 2, BIG_CIRCLE_RADIUS * 2, 
                                                  Qt::KeepAspectRatio, Qt::SmoothTransformation);
    small_circle_pixmap_ = small_circle_pixmap_.scaled(SMALL_CIRCLE_RADIUS * 2, SMALL_CIRCLE_RADIUS * 2, 
                                                      Qt::KeepAspectRatio, Qt::SmoothTransformation);
}

void RockerWidget::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event)
    
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    
    // 计算摇杆绘制区域的中心
    QRect rockerRect = rockerArea_->geometry();
    BigCir_xy.setX(rockerRect.center().x());
    BigCir_xy.setY(rockerRect.center().y());
    
    // 如果鼠标没有按下，小圆回到中心
    if (!MousePressFlag) {
        SmallCir_xy = BigCir_xy;
    }
    
    // 绘制大圆（背景圆环）
    QPoint big_top_left(BigCir_xy.x() - BIG_CIRCLE_RADIUS, BigCir_xy.y() - BIG_CIRCLE_RADIUS);
    painter.drawPixmap(big_top_left, big_circle_pixmap_);
    
    // 绘制小圆（操纵杆）
    QPoint small_top_left(SmallCir_xy.x() - SMALL_CIRCLE_RADIUS, SmallCir_xy.y() - SMALL_CIRCLE_RADIUS);
    painter.drawPixmap(small_top_left, small_circle_pixmap_);
}

void RockerWidget::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
        // 检查点击是否在大圆范围内
        QPoint clickPos = event->pos();
        int distance = qSqrt(qPow(clickPos.x() - BigCir_xy.x(), 2) + qPow(clickPos.y() - BigCir_xy.y(), 2));
        
        if (distance <= BIG_CIRCLE_RADIUS) {
            MousePressFlag = true;
            SmallCir_xy = clickPos;
            
            // 限制小圆在大圆内部
            int distanceFromCenter = qSqrt(qPow(SmallCir_xy.x() - BigCir_xy.x(), 2) + qPow(SmallCir_xy.y() - BigCir_xy.y(), 2));
            if (distanceFromCenter > BIG_CIRCLE_RADIUS - SMALL_CIRCLE_RADIUS) {
                // 计算边界位置
                double angle = qAtan2(SmallCir_xy.y() - BigCir_xy.y(), SmallCir_xy.x() - BigCir_xy.x());
                SmallCir_xy.setX(BigCir_xy.x() + (BIG_CIRCLE_RADIUS - SMALL_CIRCLE_RADIUS) * qCos(angle));
                SmallCir_xy.setY(BigCir_xy.y() + (BIG_CIRCLE_RADIUS - SMALL_CIRCLE_RADIUS) * qSin(angle));
            }
            
            update(); // 触发重绘
            
            // 计算归一化坐标并发送控制命令
            double normalizedX = (double)(SmallCir_xy.x() - BigCir_xy.x()) / (BIG_CIRCLE_RADIUS - SMALL_CIRCLE_RADIUS);
            double normalizedY = -(double)(SmallCir_xy.y() - BigCir_xy.y()) / (BIG_CIRCLE_RADIUS - SMALL_CIRCLE_RADIUS); // Y轴翻转
            
            emit rockerMoved(normalizedX, normalizedY);
            
            // 发送控制命令 - 只有当前页面激活时才发送
            if (btnController_ && isCurrentPage()) {
                btnController_->stateRockerControl(); // 切换到摇杆控制状态
                btnController_->updateRockerState(normalizedX, normalizedY);
            }
        }
    }
    
    QWidget::mousePressEvent(event);
}

void RockerWidget::mouseMoveEvent(QMouseEvent *event)
{
    if (MousePressFlag) {
        SmallCir_xy = event->pos();
        
        // 限制小圆在大圆内部
        int distanceFromCenter = qSqrt(qPow(SmallCir_xy.x() - BigCir_xy.x(), 2) + qPow(SmallCir_xy.y() - BigCir_xy.y(), 2));
        if (distanceFromCenter > BIG_CIRCLE_RADIUS - SMALL_CIRCLE_RADIUS) {
            // 计算边界位置
            double angle = qAtan2(SmallCir_xy.y() - BigCir_xy.y(), SmallCir_xy.x() - BigCir_xy.x());
            SmallCir_xy.setX(BigCir_xy.x() + (BIG_CIRCLE_RADIUS - SMALL_CIRCLE_RADIUS) * qCos(angle));
            SmallCir_xy.setY(BigCir_xy.y() + (BIG_CIRCLE_RADIUS - SMALL_CIRCLE_RADIUS) * qSin(angle));
        }
        
        update(); // 触发重绘
        
        // 计算归一化坐标并发送控制命令
        double normalizedX = (double)(SmallCir_xy.x() - BigCir_xy.x()) / (BIG_CIRCLE_RADIUS - SMALL_CIRCLE_RADIUS);
        double normalizedY = -(double)(SmallCir_xy.y() - BigCir_xy.y()) / (BIG_CIRCLE_RADIUS - SMALL_CIRCLE_RADIUS); // Y轴翻转
        
        emit rockerMoved(normalizedX, normalizedY);
        
        // 发送控制命令 - 只有当前页面激活时才发送
        if (btnController_ && isCurrentPage()) {
            btnController_->updateRockerState(normalizedX, normalizedY);
        }
        
        // 更新状态标签
        statusLabel_->setText(QString("X: %1, Y: %2").arg(normalizedX, 0, 'f', 2).arg(normalizedY, 0, 'f', 2));
    }
    
    QWidget::mouseMoveEvent(event);
}

void RockerWidget::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton && MousePressFlag) {
        MousePressFlag = false;
        SmallCir_xy = BigCir_xy; // 复位到中心
        update(); // 触发重绘
        
        emit rockerReleased();
        
        // 停止移动
        if (btnController_) {
            btnController_->stopMovement();
        }
        
        // 恢复状态标签
        statusLabel_->setText("移动摇杆控制机器人方向");
    }
    
    QWidget::mouseReleaseEvent(event);
}

void RockerWidget::setStackedWidgetInfo(QStackedWidget *stackedWidget, int pageIndex)
{
    stackedWidget_ = stackedWidget;
    pageIndex_ = pageIndex;
}

bool RockerWidget::isCurrentPage() const
{
    if (!stackedWidget_ || pageIndex_ < 0) {
        return false;
    }
    return stackedWidget_->currentIndex() == pageIndex_;
}

void RockerWidget::checkPageStatus()
{
    static bool wasActive = false;
    bool isActive = isCurrentPage();
    
    // 如果从活跃状态变为非活跃状态，立即停止移动
    if (wasActive && !isActive) {
        if (btnController_ && MousePressFlag) {
            btnController_->stopMovement();
            MousePressFlag = false; // 重置鼠标按下状态
            
            // 重置摇杆位置到中心
            BigCir_xy = QPoint(rockerArea_->width() / 2, rockerArea_->height() / 2);
            SmallCir_xy = BigCir_xy;
            
            // 更新显示
            update();
            
            // 发射信号
            emit rockerReleased();
            
            qDebug() << "摇杆控制页面切换离开，自动停止移动";
        }
    }
    
    wasActive = isActive;
}
