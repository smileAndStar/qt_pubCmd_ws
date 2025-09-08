/**
 * @file frmmain.cpp
 * @brief 主窗口实现文件
 * @details 这是一个Qt控制面板程序的主窗口实现，包含以下主要功能：
 *          1. 自定义无边框窗口，支持最小化、最大化、关闭操作
 *          2. 顶部导航栏，支持多个功能模块切换（主    // 定义按钮图标尺寸和宽度
    QSize icoSize(32, 32);   // 图标尺寸：32x32像素
    int icoWidth = 85;       // 按钮最小宽度：85像素

    //配置顶部导航栏的所有按钮
    QList<QAbstractButton *> tbtns = ui->widgetTop->findChildren<QAbstractButton *>();
    foreach (QAbstractButton *btn, tbtns) {
        btn->setIconSize(icoSize);         // 统一设置按钮图标大小
        btn->setMinimumWidth(icoWidth);    // 统一设置按钮最小宽度
        btn->setCheckable(true);           // 设置按钮可选中（实现互斥选择效果）
        // 连接所有按钮的点击信号到统一的槽函数
        connect(btn, SIGNAL(clicked()), this, SLOT(buttonClick()));
    }

    // 默认选中主界面按钮
    ui->btnMain->click();

    // 为左侧菜单容器设置样式属性标识
    ui->widgetLeftMain->setProperty("flag", "left");     // 主菜单容器标识
    ui->widgetLeftConfig->setProperty("flag", "left");   // 配置菜单容器标识
    
    // 为不同页面的左侧按钮设置高度样式
    // page1（主界面）的左侧按钮高度为60px
    ui->page1->setStyleSheet(QString("QWidget[flag=\"left\"] QAbstractButton{min-height:%1px;max-height:%1px;}").arg(60));
    // page2（系统设置）的左侧按钮高度为25px
    ui->page2->setStyleSheet(QString("QWidget[flag=\"left\"] QAbstractButton{min-height:%1px;max-height:%1px;}").arg(25));
}        3. 左侧菜单系统，分为主菜单和配置菜单两个层级
 *          4. 动态样式表加载和颜色主题配置
 *          5. FontAwesome图标字体支持
 */

#pragma execution_character_set("utf-8")  // 设置源文件编码为UTF-8，确保中文字符正确显示

#include "frmmain.h"
#include "ui_frmmain.h"
#include "iconhelper.h"  // 图标助手类，用于设置FontAwesome图标
#include "qthelper.h"    // Qt助手类，提供常用功能如无边框窗口设置
#include "rocker/rockerwidget.h"
#include "healthMod/healthWidget.h"  // 健康监测模块

/**
 * @brief 主窗口构造函数
 * @param parent 父窗口指针，通常为nullptr表示顶级窗口
 * @details 构造函数执行以下初始化步骤：
 *          1. setupUi(this) - 根据UI设计文件初始化界面控件
 *          2. initForm() - 初始化窗口基本设置（无边框、图标、事件等）
 *          3. initStyle() - 初始化样式表和颜色配置
 *          4. initLeftMain() - 初始化左侧主菜单按钮
 *          5. initLeftConfig() - 初始化左侧配置菜单按钮
 */
frmMain::frmMain(QWidget *parent) : QWidget(parent), ui(new Ui::frmMain)
{
    ui->setupUi(this);      // 初始化UI界面，创建所有控件
    this->initForm();       // 初始化窗口基本设置
    this->initStyle();      // 初始化样式表和颜色配置
    this->initLeftMain();   // 初始化左侧主菜单按钮
    this->initLeftConfig(); // 初始化左侧配置菜单按钮
}

/**
 * @brief 析构函数，释放资源
 * @details 释放UI指针占用的内存，Qt会自动处理子控件的释放
 */
frmMain::~frmMain()
{
    delete ui;
}

/**
 * @brief 事件过滤器，用于处理特定控件的特殊事件
 * @param watched 被监视的对象指针
 * @param event 事件对象指针
 * @return 返回true表示事件已处理，false表示继续传递事件
 * @details 当前实现功能：
 *          - 监听标题栏的双击事件，双击时触发窗口最大化/还原操作
 *          - 这是实现自定义标题栏双击功能的关键代码
 */
bool frmMain::eventFilter(QObject *watched, QEvent *event)
{
    // 检查是否是标题栏控件被监视
    if (watched == ui->widgetTitle) {
        // 检查是否是鼠标双击事件
        if (event->type() == QEvent::MouseButtonDblClick) {
            on_btnMenu_Max_clicked();  // 调用最大化按钮的槽函数
        }
    }
    // 调用父类的事件过滤器处理其他事件
    return QWidget::eventFilter(watched, event);
}

/**
 * @brief 从QSS样式表中提取指定标识的颜色值
 * @param qss 完整的样式表字符串
 * @param flag 颜色标识符（如："TextColor:"、"BorderColor:"等）
 * @param color 输出参数，用于存储提取到的颜色值
 * @details 功能说明：
 *          - 在样式表中查找指定的颜色标识符
 *          - 提取标识符后面7个字符的颜色值（格式：#RRGGBB）
 *          - 这种方法允许在QSS文件中定义颜色变量，程序动态提取使用
 */
void frmMain::getQssColor(const QString &qss, const QString &flag, QString &color)
{
    int index = qss.indexOf(flag);  // 在样式表中查找标识符的位置
    if (index >= 0) {
        // 从标识符位置开始，提取后面7个字符作为颜色值
        color = qss.mid(index + flag.length(), 7);
    }
    //qDebug() << TIMEMS << flag << color;  // 调试输出（已注释）
}

/**
 * @brief 从QSS样式表中批量提取多个颜色值
 * @param qss 完整的样式表字符串
 * @param textColor 输出参数：文本颜色
 * @param panelColor 输出参数：面板颜色
 * @param borderColor 输出参数：边框颜色
 * @param normalColorStart 输出参数：正常状态渐变起始颜色
 * @param normalColorEnd 输出参数：正常状态渐变结束颜色
 * @param darkColorStart 输出参数：深色状态渐变起始颜色
 * @param darkColorEnd 输出参数：深色状态渐变结束颜色
 * @param highColor 输出参数：高亮颜色
 * @details 这个重载函数用于一次性提取样式表中的所有颜色配置，
 *          避免多次调用单个颜色提取函数，提高效率
 */
void frmMain::getQssColor(const QString &qss, QString &textColor, QString &panelColor,
                          QString &borderColor, QString &normalColorStart, QString &normalColorEnd,
                          QString &darkColorStart, QString &darkColorEnd, QString &highColor)
{
    // 依次提取各种颜色值
    getQssColor(qss, "TextColor:", textColor);              // 文本颜色
    getQssColor(qss, "PanelColor:", panelColor);            // 面板颜色
    getQssColor(qss, "BorderColor:", borderColor);          // 边框颜色
    getQssColor(qss, "NormalColorStart:", normalColorStart); // 正常状态渐变起始色
    getQssColor(qss, "NormalColorEnd:", normalColorEnd);     // 正常状态渐变结束色
    getQssColor(qss, "DarkColorStart:", darkColorStart);     // 深色状态渐变起始色
    getQssColor(qss, "DarkColorEnd:", darkColorEnd);         // 深色状态渐变结束色
    getQssColor(qss, "HighColor:", highColor);               // 高亮颜色
}

/**
 * @brief 初始化窗口基本设置和控件配置
 * @details 这个函数负责窗口的基础设置，包括：
 *          1. 设置无边框窗口样式
 *          2. 配置标题栏图标和按钮
 *          3. 设置事件过滤器
 *          4. 初始化顶部导航按钮
 *          5. 设置默认字体和样式
 */
void frmMain::initForm()
{
    //设置无边框窗口（移除系统默认标题栏，使用自定义标题栏）
    QtHelper::setFramelessForm(this);
    
    //设置标题栏各个按钮的图标（使用FontAwesome字体图标）
    IconHelper::setIcon(ui->labIco, 0xf1b6, 30);        // 设置窗口图标（机械臂图标）
    IconHelper::setIcon(ui->btnMenu_Min, 0xf068);       // 最小化按钮图标（减号）
    IconHelper::setIcon(ui->btnMenu_Max, 0xf067);       // 最大化按钮图标（加号）
    IconHelper::setIcon(ui->btnMenu_Close, 0xf00d);     // 关闭按钮图标（×）

    //ui->widgetMenu->setVisible(false);  // 隐藏菜单控件（已注释，保留备用）
    
    // 为控件设置样式属性，用于CSS选择器
    ui->widgetTitle->setProperty("form", "title");       // 标题栏样式属性
    //关联事件过滤器用于双击放大
    ui->widgetTitle->installEventFilter(this);          // 安装事件过滤器，监听双击事件
    ui->widgetTop->setProperty("nav", "top");            // 顶部导航样式属性

    // 设置标题文字的字体和内容
    QFont font;
    font.setPixelSize(25);                              // 设置字体大小为25像素
    ui->labTitle->setFont(font);                        // 应用字体到标题标签
    ui->labTitle->setText("控制面板");                   // 设置标题文本
    this->setWindowTitle(ui->labTitle->text());         // 设置窗口标题（用于任务栏显示）

    // 设置堆叠窗口内标签的字体样式
    ui->stackedWidget->setStyleSheet("QLabel{font:21px;}");

    // 定义按钮图标尺寸和宽度
    QSize icoSize(32, 32);   // 图标尺寸：32x32像素
    int icoWidth = 85;       // 按钮最小宽度：85像素

    //设置顶部导航按钮
    QList<QAbstractButton *> tbtns = ui->widgetTop->findChildren<QAbstractButton *>();
    foreach (QAbstractButton *btn, tbtns) {
        btn->setIconSize(icoSize);
        btn->setMinimumWidth(icoWidth);
        btn->setCheckable(true);
        connect(btn, SIGNAL(clicked()), this, SLOT(buttonClick()));
    }

    ui->btnMain->click();

    ui->widgetLeftMain->setProperty("flag", "left");
    ui->widgetLeftConfig->setProperty("flag", "left");
    ui->page1->setStyleSheet(QString("QWidget[flag=\"left\"] QAbstractButton{min-height:%1px;max-height:%1px;}").arg(60));
    ui->page2->setStyleSheet(QString("QWidget[flag=\"left\"] QAbstractButton{min-height:%1px;max-height:%1px;}").arg(25));

    // 提升page3（人体检测界面为自定义Widget）
    // page3在stackedWidget中，索引为2（第3个页面，从0开始计数）
    ui->page3->addPageMonitor("health_monitor", ui->stackedWidget, 2, "健康监测页面");
}

/**
 * @brief 初始化样式表和颜色配置
 * @details 这个函数负责：
 *          1. 从资源文件加载QSS样式表
 *          2. 设置应用程序的调色板
 *          3. 从样式表中提取颜色变量
 *          4. 将颜色值保存到成员变量中供后续使用
 */
void frmMain::initStyle()
{
    //从资源文件加载样式表文件
    QString qss = QtHelper::getStyle(":/qss/blacksoft.css");
    if (!qss.isEmpty()) {
        // 从样式表的第20个字符开始提取7个字符作为调色板颜色
        QString paletteColor = qss.mid(20, 7);
        // 设置应用程序的调色板颜色（影响默认控件颜色）
        qApp->setPalette(QPalette(QColor(paletteColor)));
        // 将样式表应用到整个应用程序
        qApp->setStyleSheet(qss);
    }

    //从样式表中提取各种颜色值用于程序中的颜色配置
    QString textColor, panelColor, borderColor, normalColorStart, normalColorEnd, darkColorStart, darkColorEnd, highColor;
    getQssColor(qss, textColor, panelColor, borderColor, normalColorStart, normalColorEnd, darkColorStart, darkColorEnd, highColor);

    //将提取的颜色值保存到成员变量中，供其他函数使用
    this->borderColor = highColor;          // 边框颜色使用高亮色
    this->normalBgColor = normalColorStart; // 正常状态背景色
    this->darkBgColor = panelColor;         // 深色状态背景色使用面板色
    this->normalTextColor = textColor;      // 正常状态文本颜色
    this->darkTextColor = normalTextColor;  // 深色状态文本颜色与正常状态相同
}

/**
 * @brief 顶部导航按钮点击事件处理函数
 * @details 这个函数处理所有顶部导航按钮的点击事件，实现：
 *          1. 获取被点击的按钮及其文本
 *          2. 设置按钮的互斥选择状态（只有一个按钮被选中）
 *          3. 根据按钮文本切换到对应的页面
 *          4. 特殊处理"用户退出"按钮直接退出程序
 */
void frmMain::buttonClick()
{
    // 获取发送信号的按钮对象（哪个按钮被点击了）
    QAbstractButton *b = (QAbstractButton *)sender();
    QString name = b->text();  // 获取按钮上的文字

    // 遍历所有顶部导航按钮，实现互斥选择效果
    QList<QAbstractButton *> tbtns = ui->widgetTop->findChildren<QAbstractButton *>();
    foreach (QAbstractButton *btn, tbtns) {
        // 只有当前被点击的按钮设置为选中状态，其他按钮取消选中
        btn->setChecked(btn == b);
    }

    // 根据按钮文字切换到对应的页面
    if (name == "主界面") {
        ui->stackedWidget->setCurrentIndex(0);      // 切换到第0页：主界面
    } else if (name == "系统设置") {
        ui->stackedWidget->setCurrentIndex(1);      // 切换到第1页：系统设置
    } else if (name == "人体检测") {
        ui->stackedWidget->setCurrentIndex(2);      // 切换到第2页：人体检测
    } else if (name == "保留界面") {
        ui->stackedWidget->setCurrentIndex(3);      // 切换到第3页：保留界面
    } else if (name == "用户退出") {
        exit(0);  // 直接退出程序
    }
}

/**
 * @brief 初始化左侧主菜单按钮
 * @details 这个函数负责配置左侧主菜单的按钮，包括：
 *          1. 定义按钮图标（FontAwesome编码）
 *          2. 获取按钮对象引用
 *          3. 设置按钮属性和事件连接
 *          4. 应用统一的样式和颜色配置
 *          5. 默认选中第一个按钮
 */
void frmMain::initLeftMain()
{
    // 定义主菜单按钮的图标编码（FontAwesome字体图标的十六进制编码）
    iconsMain << 0xf193 << 0xf11b;
    // 获取主菜单按钮对象的引用列表
    btnsMain << ui->tbtnMain1 << ui->tbtnMain2;

    // 遍历配置每个主菜单按钮
    for (int i = 0; i < btnsMain.count(); ++i) {
        QToolButton *btn = (QToolButton *)btnsMain.at(i);
        btn->setCheckable(true);  // 设置按钮可选中（支持选中状态）
        // 设置按钮样式：图标在上方，文字在下方
        btn->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
        // 连接按钮点击信号到对应的槽函数
        connect(btn, SIGNAL(clicked(bool)), this, SLOT(leftMainClick()));
    }

    // 配置按钮的样式和颜色
    IconHelper::StyleColor styleColor;
    styleColor.position = "left";               // 位置标识：左侧
    styleColor.iconSize = 18;                   // 图标大小：18像素
    styleColor.iconWidth = 35;                  // 图标宽度：35像素
    styleColor.iconHeight = 25;                 // 图标高度：25像素
    styleColor.borderWidth = 4;                 // 边框宽度：4像素
    styleColor.borderColor = borderColor;       // 边框颜色：使用配置的边框色
    // 设置颜色方案：正常状态和选中状态的背景色、文字色
    styleColor.setColor(normalBgColor, normalTextColor, darkBgColor, darkTextColor);
    // 将样式应用到主菜单按钮组
    IconHelper::setStyle(ui->widgetLeftMain, btnsMain, iconsMain, styleColor);

    // 创建摇杆页面
    rockerWidget_ = new RockerWidget(this);
    // 先添加到 StackedWidget（成为索引1）
    ui->stackedWidget_main->addWidget(rockerWidget_);
    // 然后添加页面状态监控
    rockerWidget_->addPageMonitor("main_rocker", ui->stackedWidget_main, 1, "主摇杆控制页面");
    // 默认选中第一个主菜单按钮
    ui->tbtnMain1->click();

    // 实例化健康监测模块
    healthMonitor_ = new healthMnter(ui->page3);
}

/**
 * @brief 初始化左侧配置菜单按钮
 * @details 这个函数负责配置左侧配置菜单的按钮，功能类似initLeftMain()，但有以下区别：
 *          1. 按钮数量更多（6个vs3个）
 *          2. 按钮尺寸更小，适合二级菜单
 *          3. 布局方式不同：图标在左，文字在右
 *          4. 边框宽度较小
 */
void frmMain::initLeftConfig()
{
    // 定义配置菜单按钮的图标编码（6个按钮对应6个图标）
    iconsConfig << 0xf031 << 0xf036 << 0xf249 << 0xf055 << 0xf05a << 0xf249;
    // 获取配置菜单按钮对象的引用列表
    btnsConfig << ui->tbtnConfig1 << ui->tbtnConfig2 << ui->tbtnConfig3 << ui->tbtnConfig4 << ui->tbtnConfig5 << ui->tbtnConfig6;

    // 遍历配置每个配置菜单按钮
    for (int i = 0; i < btnsConfig.count(); ++i) {
        QToolButton *btn = (QToolButton *)btnsConfig.at(i);
        btn->setCheckable(true);  // 设置按钮可选中
        // 设置按钮样式：图标在左侧，文字在右侧（适合较小的二级菜单）
        btn->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
        // 连接按钮点击信号到对应的槽函数
        connect(btn, SIGNAL(clicked(bool)), this, SLOT(leftConfigClick()));
    }

    // 配置按钮的样式和颜色（相比主菜单，尺寸更小）
    IconHelper::StyleColor styleColor;
    styleColor.position = "left";               // 位置标识：左侧
    styleColor.iconSize = 16;                   // 图标大小：16像素（比主菜单小）
    styleColor.iconWidth = 20;                  // 图标宽度：20像素
    styleColor.iconHeight = 20;                 // 图标高度：20像素
    styleColor.borderWidth = 3;                 // 边框宽度：3像素（比主菜单小）
    styleColor.borderColor = borderColor;       // 边框颜色：使用配置的边框色
    // 设置颜色方案：与主菜单相同的颜色配置
    styleColor.setColor(normalBgColor, normalTextColor, darkBgColor, darkTextColor);
    // 将样式应用到配置菜单按钮组
    IconHelper::setStyle(ui->widgetLeftConfig, btnsConfig, iconsConfig, styleColor);
    
    // 默认选中第一个配置菜单按钮
    ui->tbtnConfig1->click();
}

/**
 * @brief 左侧主菜单按钮点击事件处理函数
 * @details 当用户点击左侧主菜单中的任意按钮时调用此函数，实现：
 *          1. 获取被点击的按钮及其文本
 *          2. 在主菜单按钮组中设置互斥选择状态
 *          3. 在界面上显示当前选中的菜单名称
 */
void frmMain::leftMainClick()
{
    // 获取发送信号的按钮对象
    QAbstractButton *b = (QAbstractButton *)sender();
    QString name = b->text();  // 获取按钮文本作为菜单名称
    
    // 在主菜单按钮组中设置互斥选择（只能选中一个按钮）
    for (int i = 0; i < btnsMain.count(); ++i) {
        QAbstractButton *btn = btnsMain.at(i);
        // 只有当前被点击的按钮设置为选中状态
        btn->setChecked(btn == b);
    }

    // 根据按钮文字切换到对应的页面
    if (name == "按键控制") {
        ui->stackedWidget_main->setCurrentIndex(0);      // 切换到第0页：按键控制
    } else if (name == "摇杆控制") {
        ui->stackedWidget_main->setCurrentIndex(1);      // 切换到第1页：摇杆控制
    }
}

/**
 * @brief 左侧配置菜单按钮点击事件处理函数
 * @details 当用户点击左侧配置菜单中的任意按钮时调用此函数，功能类似leftMainClick()：
 *          1. 获取被点击的按钮及其文本
 *          2. 在配置菜单按钮组中设置互斥选择状态
 *          3. 在界面上显示当前选中的配置菜单名称
 */
void frmMain::leftConfigClick()
{
    // 获取发送信号的按钮对象
    QToolButton *b = (QToolButton *)sender();
    QString name = b->text();  // 获取按钮文本作为菜单名称
    
    // 在配置菜单按钮组中设置互斥选择（只能选中一个按钮）
    for (int i = 0; i < btnsConfig.count(); ++i) {
        QAbstractButton *btn = btnsConfig.at(i);
        // 只有当前被点击的按钮设置为选中状态
        btn->setChecked(btn == b);
    }

    // 在标签控件中显示当前选中的配置菜单名称
    ui->lab2->setText(name);
}

/**
 * @brief 窗口最小化按钮点击事件处理函数
 * @details 当用户点击标题栏的最小化按钮时调用，将窗口最小化到任务栏
 */
void frmMain::on_btnMenu_Min_clicked()
{
    showMinimized();  // 调用Qt内置函数最小化窗口
}

/**
 * @brief 窗口最大化/还原按钮点击事件处理函数
 * @details 这个函数实现窗口在最大化和还原状态之间的切换：
 *          1. 使用静态变量记录当前状态和窗口位置
 *          2. 最大化时保存当前位置，设置为全屏尺寸
 *          3. 还原时恢复之前保存的位置和尺寸
 *          4. 设置窗口移动属性（最大化时禁止移动）
 */
void frmMain::on_btnMenu_Max_clicked()
{
    static bool max = false;                    // 静态变量：记录当前是否为最大化状态
    static QRect location = this->geometry();   // 静态变量：保存窗口还原时的位置尺寸

    if (max) {
        // 当前是最大化状态，需要还原窗口
        this->setGeometry(location);            // 恢复到之前保存的位置和尺寸
    } else {
        // 当前是正常状态，需要最大化窗口
        location = this->geometry();            // 保存当前窗口的位置和尺寸
        this->setGeometry(QtHelper::getScreenRect());  // 设置窗口为全屏尺寸
    }

    // 设置窗口是否可移动的属性（最大化时不允许移动窗口）
    this->setProperty("canMove", max);
    max = !max;  // 切换状态标识
}

/**
 * @brief 窗口关闭按钮点击事件处理函数
 * @details 当用户点击标题栏的关闭按钮时调用，关闭窗口并退出程序
 */
void frmMain::on_btnMenu_Close_clicked()
{
    close();  // 调用Qt内置函数关闭窗口
}

