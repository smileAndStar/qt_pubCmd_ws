# Qt SmoothCurve 平滑曲线绘制教程

这是一个Qt应用程序，专门用于演示和学习如何在Qt中绘制平滑曲线。通过本项目，你将学会如何使用贝塞尔曲线和三次样条插值来生成美观的平滑曲线。

## 📁 项目结构

```
smoothcurve/
├── main.cpp                 # 应用程序入口点
├── frmsmoothcurve.h        # 主窗口类声明
├── frmsmoothcurve.cpp      # 主窗口类实现
├── frmsmoothcurve.ui       # UI界面文件
├── smoothcurve.h           # 平滑曲线工具类声明
├── smoothcurve.cpp         # 平滑曲线工具类实现
├── smoothcurve.pro         # Qt项目文件
└── README.md               # 本文档
```

## ✨ 功能特性

- **三种曲线类型**：
  - 正常曲线（直线连接）
  - 平滑曲线1（高级贝塞尔插值）
  - 平滑曲线2（简单贝塞尔插值）
- **交互式界面**：可选择显示/隐藏数据点
- **抗锯齿渲染**：使用QPainter高质量绘制
- **随机数据生成**：自动生成测试数据点

## 🎯 学习目标

通过本项目，你将学会：
1. Qt绘图系统的基础使用
2. QPainterPath的创建和使用
3. 贝塞尔曲线的原理和实现
4. 三次样条插值算法
5. 数据可视化的最佳实践

## 🚀 快速开始

### 编译运行

```bash
# 生成Makefile
qmake smoothcurve.pro

# 编译项目
make

# 运行程序（Linux/macOS）
./smoothcurve

# 运行程序（Windows）
smoothcurve.exe
```

### 界面操作

1. **正常曲线**：查看原始数据点的直线连接效果
2. **平滑曲线1**：体验高质量的平滑插值效果
3. **平滑曲线2**：对比简单插值的效果
4. **显示坐标点**：切换数据点的显示状态

## 📖 详细教程

### 第1步：理解数据结构

```cpp
// 数据点容器
QVector<QPointF> datas;

// 随机生成测试数据
int x = -300;
while (x < 300) {
    datas << QPointF(x, rand() % 300 - 100);
    x += qMin(rand() % 30 + 5, 300);
}
```

### 第2步：创建基础路径（正常曲线）

```cpp
QPainterPath pathNormal;
pathNormal.moveTo(datas.at(0));  // 移动到起始点
for (int i = 1; i < datas.size(); ++i) {
    pathNormal.lineTo(datas.at(i));  // 直线连接到下一点
}
```

### 第3步：生成平滑曲线

#### 方法1：高级平滑算法（推荐）

```cpp
QPainterPath pathSmooth1 = SmoothCurve::createSmoothCurve(datas);
```

这个方法使用三次样条插值算法，通过计算控制点来生成平滑的贝塞尔曲线：

```cpp
// 核心算法伪代码
1. 计算第一控制点和第二控制点
2. 使用三次贝塞尔曲线连接相邻数据点
3. 确保曲线在连接点处平滑过渡
```

#### 方法2：简单平滑算法

```cpp
QPainterPath pathSmooth2 = SmoothCurve::createSmoothCurve2(datas);
```

这个方法使用简化的贝塞尔插值：

```cpp
for (int i = 0; i < count - 1; ++i) {
    QPointF sp = points.at(i);     // 起始点
    QPointF ep = points.at(i + 1); // 结束点
    
    // 控制点位于两点x坐标中间
    QPointF c1 = QPointF((sp.x() + ep.x()) / 2, sp.y());
    QPointF c2 = QPointF((sp.x() + ep.x()) / 2, ep.y());
    
    path.cubicTo(c1, c2, ep);  // 三次贝塞尔曲线
}
```

### 第4步：绘制实现

```cpp
void frmSmoothCurve::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);  // 开启抗锯齿
    painter.translate(width() / 2, height() / 2);   // 坐标系居中
    painter.scale(1, -1);                           // Y轴向上为正

    // 绘制坐标轴
    painter.setPen(QColor(180, 180, 180));
    painter.drawLine(-250, 0, 250, 0);  // X轴
    painter.drawLine(0, 150, 0, -150);  // Y轴

    // 绘制选中的曲线
    painter.setPen(QPen(QColor(80, 80, 80), 2));
    if (ui->rbtnPathSmooth1->isChecked()) {
        painter.drawPath(pathSmooth1);
    } else if (ui->rbtnPathSmooth2->isChecked()) {
        painter.drawPath(pathSmooth2);
    } else {
        painter.drawPath(pathNormal);
    }

    // 可选：绘制数据点
    if (ui->ckShowPoint->isChecked()) {
        painter.setPen(Qt::black);
        painter.setBrush(Qt::gray);
        foreach (QPointF point, datas) {
            painter.drawEllipse(point, 3, 3);
        }
    }
}
```

## 🛠️ 自定义使用指南

### 在你的项目中使用SmoothCurve

#### 1. 复制文件
将 `smoothcurve.h` 和 `smoothcurve.cpp` 复制到你的项目中。

#### 2. 包含头文件
```cpp
#include "smoothcurve.h"
```

#### 3. 创建你的绘图widget

```cpp
class MyWidget : public QWidget
{
    Q_OBJECT

public:
    MyWidget(QWidget *parent = nullptr) : QWidget(parent)
    {
        // 准备你的数据
        QVector<QPointF> myData;
        myData << QPointF(0, 10) << QPointF(20, 30) << QPointF(40, 15) 
               << QPointF(60, 45) << QPointF(80, 25);
        
        // 生成平滑路径
        smoothPath = SmoothCurve::createSmoothCurve(myData);
    }

protected:
    void paintEvent(QPaintEvent *event) override
    {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);
        
        // 设置绘图样式
        painter.setPen(QPen(Qt::blue, 3));
        
        // 绘制平滑曲线
        painter.drawPath(smoothPath);
    }

private:
    QPainterPath smoothPath;
};
```

#### 4. 实时数据更新示例

```cpp
class RealTimeChart : public QWidget
{
    Q_OBJECT

public slots:
    void addDataPoint(double x, double y)
    {
        dataPoints.append(QPointF(x, y));
        
        // 限制数据点数量（可选）
        if (dataPoints.size() > 100) {
            dataPoints.removeFirst();
        }
        
        // 重新生成平滑路径
        smoothPath = SmoothCurve::createSmoothCurve(dataPoints);
        
        // 触发重绘
        update();
    }

private:
    QVector<QPointF> dataPoints;
    QPainterPath smoothPath;
};
```

## 🔧 算法详解

### 三次样条插值原理

三次样条插值是一种数学方法，用于在给定数据点之间生成平滑曲线。其特点是：

1. **连续性**：曲线在所有数据点处连续
2. **平滑性**：一阶和二阶导数连续
3. **局部控制**：修改一个数据点只影响附近的曲线段

### 控制点计算

```cpp
// 简化的控制点计算逻辑
for (int i = 1; i < n - 1; ++i) {
    rhsx[i] = 4 * datas[i].x() + 2 * datas[i + 1].x();
    rhsy[i] = 4 * datas[i].y() + 2 * datas[i + 1].y();
}
```

这个公式确保了曲线在数据点处的平滑过渡。

## 📊 性能对比

| 算法 | 计算复杂度 | 平滑度 | 适用场景 |
|------|------------|--------|----------|
| 正常曲线 | O(n) | 无 | 快速预览 |
| 平滑曲线1 | O(n²) | 极高 | 高质量展示 |
| 平滑曲线2 | O(n) | 中等 | 实时应用 |

## 🎨 自定义样式

### 修改曲线外观

```cpp
// 渐变画笔
QLinearGradient gradient(0, 0, 0, 100);
gradient.setColorAt(0, Qt::blue);
gradient.setColorAt(1, Qt::cyan);
painter.setPen(QPen(QBrush(gradient), 3));

// 虚线样式
painter.setPen(QPen(Qt::red, 2, Qt::DashLine));

// 圆角连接
QPen pen(Qt::blue, 4);
pen.setJoinStyle(Qt::RoundJoin);
pen.setCapStyle(Qt::RoundCap);
painter.setPen(pen);
```

### 添加阴影效果

```cpp
// 绘制阴影
painter.save();
painter.translate(2, 2);
painter.setPen(QPen(QColor(0, 0, 0, 50), 3));
painter.drawPath(smoothPath);
painter.restore();

// 绘制主曲线
painter.setPen(QPen(Qt::blue, 3));
painter.drawPath(smoothPath);
```

## 🐛 常见问题

### Q: 为什么我的曲线不平滑？
A: 确保数据点之间有足够的间距，点太密集可能导致过度拟合。

### Q: 如何处理大量数据点？
A: 对于超过1000个点的数据，建议使用 `createSmoothCurve2` 或者数据抽样。

### Q: 能否自定义插值强度？
A: 可以修改 `smoothcurve.cpp` 中的系数来调整平滑程度。

## 📚 扩展学习

- [Qt官方绘图教程](https://doc.qt.io/qt-5/paintsystem.html)
- [贝塞尔曲线数学原理](https://en.wikipedia.org/wiki/Bézier_curve)
- [三次样条插值](https://en.wikipedia.org/wiki/Spline_interpolation)

## 🤝 贡献

欢迎提交Issue和Pull Request来改进这个项目！

## 📄 许可证

本项目仅供学习和演示使用。
