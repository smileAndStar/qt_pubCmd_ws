#include "qt_cmd_vel/widrocker.h"
#include "ui_widrocker.h"
#include <QDebug>

widrocker::widrocker(BtnController *btnController, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::widrocker),
    btnController_(btnController)
{
    ui->setupUi(this);
    Init();
}

widrocker::~widrocker()
{
    // 切换到停止状态，保险起见
    btnController_->stopMovement();
    delete ui;
}

void widrocker::on_BtnBack_clicked()
{
    this->close();
}

// 初始化函数
void widrocker::Init()
{

    // 设置窗口固定大小以容纳摇杆
    // 定义窗口的尺寸
    int windowWidth = BIG_CIRCLE_RADIUS * 2 + 20;
    int windowHeight = BIG_CIRCLE_RADIUS * 2 + 20;
    this->setFixedSize(windowWidth, windowHeight);
    this->setWindowTitle("摇杆控制器");

    // 设置大圆圆心位置在窗口中心
    BigCir_xy.setX(windowWidth / 2);
    BigCir_xy.setY(windowHeight / 2);

    // 初始时，小圆圆心与大圆相同
    SmallCir_xy = BigCir_xy;

    // 鼠标点击标志初始化
    MousePressFlag = false;

    // 在构造时加载图片，提高效率
    // Tip. Qt 中文件路径的最前面加上":",代表不是从绝对路径或相对路径寻找，而是通过资源文件路径查找
    // 加载大圆图片
   if (!big_circle_pixmap_.load(":/images/ring.png"))
   {
       qWarning() << "错误：无法加载大圆图片资源 ':images/ring.png'";
   }

    // 加载小圆图片
   if (!small_circle_pixmap_.load(":/images/circle.png"))
   {
       qWarning() << "错误：无法加载小圆图片资源 ':images/circle.png'";
   }
}

// 绘图事件
void widrocker::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setRenderHint(QPainter::SmoothPixmapTransform, true);

    // 绘制大圆（背景）
    painter.drawPixmap(BigCir_xy.x() - BIG_CIRCLE_RADIUS, BigCir_xy.y() - BIG_CIRCLE_RADIUS,
                       BIG_CIRCLE_RADIUS * 2, BIG_CIRCLE_RADIUS * 2, big_circle_pixmap_);

    // 绘制小圆（手柄）
    painter.drawPixmap(SmallCir_xy.x() - SMALL_CIRCLE_RADIUS, SmallCir_xy.y() - SMALL_CIRCLE_RADIUS,
                       SMALL_CIRCLE_RADIUS * 2, SMALL_CIRCLE_RADIUS * 2, small_circle_pixmap_);
}

// 鼠标按下事件
void widrocker::mousePressEvent(QMouseEvent *e)
{
    // 计算鼠标点击位置到大圆中心的距离
    double distance = qSqrt(qPow(e->pos().x() - BigCir_xy.x(), 2) + qPow(e->pos().y() - BigCir_xy.y(), 2));

    // 如果点击在大圆内部，则激活摇杆
    if (distance <= BIG_CIRCLE_RADIUS)
    {
       MousePressFlag = true;
       // 立即处理移动，即使只是点击
       mouseMoveEvent(e);
    }
}

// 鼠标移动事件
void widrocker::mouseMoveEvent(QMouseEvent *e)
{
    if (!MousePressFlag) // 如果鼠标没有按下，则不处理
        return;

    QPoint mouse_pos = e->pos();

    // 计算鼠标到大圆中心的距离和角度
    double distance = qSqrt(qPow(mouse_pos.x() - BigCir_xy.x(), 2) + qPow(mouse_pos.y() - BigCir_xy.y(), 2));
    double angle = qAtan2(mouse_pos.y() - BigCir_xy.y(), mouse_pos.x() - BigCir_xy.x());

    // 如果鼠标在圆内，小圆就跟随鼠标
    if (distance <= BIG_CIRCLE_RADIUS)
    {
        SmallCir_xy = mouse_pos;
    }
    // 如果鼠标在圆外，小圆就限制在大圆的边缘上
    else
    {
        SmallCir_xy.setX(BigCir_xy.x() + BIG_CIRCLE_RADIUS * qCos(angle));
        SmallCir_xy.setY(BigCir_xy.y() + BIG_CIRCLE_RADIUS * qSin(angle));
    }

    // --- 核心：计算归一化值并发送信号 ---
    double normalized_x = (SmallCir_xy.x() - BigCir_xy.x()) / static_cast<double>(BIG_CIRCLE_RADIUS);
    // Y轴在屏幕坐标系中向下为正，我们需要转换为向上为正，所以取反
    double normalized_y = -(SmallCir_xy.y() - BigCir_xy.y()) / static_cast<double>(BIG_CIRCLE_RADIUS);

    // emit rockerMoved(normalized_x, normalized_y);

    // 发布底盘控制消息
    btnController_->updateRockerState(normalized_x, normalized_y);

    update(); // 请求重绘界面
}

// 鼠标释放事件
void widrocker::mouseReleaseEvent(QMouseEvent *e)
{
    Q_UNUSED(e);

    MousePressFlag = false;

    // 小圆圆心复位到中心
    SmallCir_xy = BigCir_xy;

    // 发射“摇杆已释放”的信号
    // emit rockerReleased();

    // 发布底盘控制消息，重置状态
    btnController_->updateRockerState(0.0, 0.0);    //停止

    update(); // 更新绘图，让小圆在视觉上回到中心
}
