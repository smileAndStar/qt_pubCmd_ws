#include "rockercontrol.h"
#include <QDebug>

RockerControl::RockerControl(QWidget *parent)
    : QWidget(parent), isMousePressed_(false)
{
    // 加载并缩放图片
    if (!bigCirclePixmap_.load(":/image/ring.png")) {
        qWarning() << "错误：无法加载大圆图片资源 ':/image/ring.png'";
    }
    if (!smallCirclePixmap_.load(":/image/circle.png")) {
        qWarning() << "错误：无法加载小圆图片资源 ':/image/circle.png'";
    }

    // 设置鼠标跟踪，即使不按下鼠标也能捕获mousemove事件（如果需要的话）
    // setMouseTracking(true);
}

void RockerControl::resizeEvent(QResizeEvent *event)
{
    Q_UNUSED(event);
    // 控件尺寸变化时，重新计算半径和中心点
    int size = qMin(width(), height());
    bigCircleRadius_ = size / 2 * 0.9; // 留出一些边距
    smallCircleRadius_ = bigCircleRadius_ * 0.3;

    bigCirclePixmap_ = bigCirclePixmap_.scaled(bigCircleRadius_ * 2, bigCircleRadius_ * 2, Qt::KeepAspectRatio, Qt::SmoothTransformation);
    smallCirclePixmap_ = smallCirclePixmap_.scaled(smallCircleRadius_ * 2, smallCircleRadius_ * 2, Qt::KeepAspectRatio, Qt::SmoothTransformation);

    centerPoint_ = QPoint(width() / 2, height() / 2);
    if (!isMousePressed_) {
        smallCirclePos_ = centerPoint_;
    }
    update();
}

void RockerControl::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // 绘制大圆
    painter.drawPixmap(centerPoint_.x() - bigCircleRadius_, centerPoint_.y() - bigCircleRadius_, bigCirclePixmap_);

    // 绘制小圆
    painter.drawPixmap(smallCirclePos_.x() - smallCircleRadius_, smallCirclePos_.y() - smallCircleRadius_, smallCirclePixmap_);
}

void RockerControl::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
        int distance = qSqrt(qPow(event->pos().x() - centerPoint_.x(), 2) + qPow(event->pos().y() - centerPoint_.y(), 2));
        if (distance <= bigCircleRadius_) {
            isMousePressed_ = true;
            calculateRockerPosition(event->pos());
            update();
        }
    }
}

void RockerControl::mouseMoveEvent(QMouseEvent *event)
{
    if (isMousePressed_) {
        calculateRockerPosition(event->pos());
        update();
    }
}

void RockerControl::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton && isMousePressed_) {
        resetRocker();
        emit rockerReleased();
    }
}

void RockerControl::resetRocker()
{
    isMousePressed_ = false;
    smallCirclePos_ = centerPoint_;
    update();
}

void RockerControl::calculateRockerPosition(const QPoint &mousePos)
{
    smallCirclePos_ = mousePos;
    double maxDistance = bigCircleRadius_ - smallCircleRadius_;
    double distanceFromCenter = qSqrt(qPow(smallCirclePos_.x() - centerPoint_.x(), 2) + qPow(smallCirclePos_.y() - centerPoint_.y(), 2));

    if (distanceFromCenter > maxDistance) {
        double angle = qAtan2(smallCirclePos_.y() - centerPoint_.y(), smallCirclePos_.x() - centerPoint_.x());
        smallCirclePos_.setX(centerPoint_.x() + maxDistance * qCos(angle));
        smallCirclePos_.setY(centerPoint_.y() + maxDistance * qSin(angle));
    }
    emitRockerMovedSignal();
}

void RockerControl::emitRockerMovedSignal()
{
    double maxDistance = bigCircleRadius_ - smallCircleRadius_;
    if (maxDistance <= 0) return;

    // 计算归一化坐标，注意Y轴翻转
    double dx = smallCirclePos_.x() - centerPoint_.x();
    double dy = -(smallCirclePos_.y() - centerPoint_.y()); // Y向上为正

    double normalizedX = dx / maxDistance;
    double normalizedY = dy / maxDistance;

    emit rockerMoved(normalizedX, normalizedY);
}
