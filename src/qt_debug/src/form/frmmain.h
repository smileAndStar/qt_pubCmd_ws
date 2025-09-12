/**
 * @file frmmain.h
 * @brief 主窗口类声明文件
 * @details 定义了frmMain类，这是应用程序的主界面窗口类
 *          继承自QWidget，实现了控制面板的主要功能界面
 * 
 * @author 程序开发者
 * @date 创建日期
 * @version 1.0
 */

#ifndef FRMMAIN_H
#define FRMMAIN_H

#include <QWidget>
#include <rocker/rockerwidget.h>
#include <ros/ros.h>  // ROS头文件

class QAbstractButton;  // 前向声明，避免包含整个头文件
class healthMnter;      // 健康监测类前向声明

namespace Ui {
class frmMain;  // UI类的前向声明
}

/**
 * @brief 主窗口类
 * @details 实现控制面板的主界面，包含顶部导航、左侧菜单、内容区域等
 */
class frmMain : public QWidget
{
    Q_OBJECT  // Qt元对象系统宏，支持信号槽机制

public:
    /**
     * @brief 构造函数
     * @param parent 父窗口指针，默认为nullptr
     */
    // explicit frmMain(QWidget *parent = nullptr);
    explicit frmMain(ros::NodeHandle &nh, QWidget *parent = nullptr);
    
    /**
     * @brief 析构函数
     */
    ~frmMain();

protected:
    /**
     * @brief 事件过滤器
     * @param watched 被监视的对象
     * @param event 事件指针
     * @return 是否处理该事件
     */
    bool eventFilter(QObject *watched, QEvent *event);

private:
    Ui::frmMain *ui;  // UI界面指针

    // 左侧主菜单相关数据
    QList<int> iconsMain;               // 主菜单图标编码列表
    QList<QAbstractButton *> btnsMain;  // 主菜单按钮对象列表

    // 摇杆控制类
    RockerWidget *rockerWidget_;
    
    // 健康监测类
    healthMnter *healthMonitor_;

    // 左侧配置菜单相关数据
    QList<int> iconsConfig;               // 配置菜单图标编码列表
    QList<QAbstractButton *> btnsConfig; // 配置菜单按钮对象列表

    // ROS节点句柄
    ros::NodeHandle nh_;

private:
    //样式表颜色配置成员变量
    QString borderColor;      // 边框颜色
    QString normalBgColor;    // 正常状态背景色
    QString darkBgColor;      // 深色状态背景色
    QString normalTextColor;  // 正常状态文本颜色
    QString darkTextColor;    // 深色状态文本颜色

    /**
     * @brief 从QSS样式表中提取指定标识的颜色值
     * @param qss 样式表字符串
     * @param flag 颜色标识符
     * @param color 输出参数，存储提取到的颜色值
     */
    void getQssColor(const QString &qss, const QString &flag, QString &color);
    
    /**
     * @brief 从QSS样式表中提取多个颜色值
     * @param qss 样式表字符串
     * @param textColor 文本颜色
     * @param panelColor 面板颜色
     * @param borderColor 边框颜色
     * @param normalColorStart 正常状态渐变起始颜色
     * @param normalColorEnd 正常状态渐变结束颜色
     * @param darkColorStart 深色状态渐变起始颜色
     * @param darkColorEnd 深色状态渐变结束颜色
     * @param highColor 高亮颜色
     */
    void getQssColor(const QString &qss, QString &textColor,
                     QString &panelColor, QString &borderColor,
                     QString &normalColorStart, QString &normalColorEnd,
                     QString &darkColorStart, QString &darkColorEnd,
                     QString &highColor);

private slots:
    /**
     * @brief 初始化窗口基本设置
     */
    void initForm();
    
    /**
     * @brief 初始化样式表和颜色配置
     */
    void initStyle();
    
    /**
     * @brief 顶部导航按钮点击事件处理
     */
    void buttonClick();
    /**
     * @brief 初始化左侧主菜单按钮
     */
    void initLeftMain();
    
    /**
     * @brief 初始化左侧配置菜单按钮
     */
    void initLeftConfig();
    
    /**
     * @brief 左侧主菜单按钮点击事件处理
     */
    void leftMainClick();
    
    /**
     * @brief 左侧配置菜单按钮点击事件处理
     */
    void leftConfigClick();

private slots:
    /**
     * @brief 窗口最小化按钮点击事件
     */
    void on_btnMenu_Min_clicked();
    
    /**
     * @brief 窗口最大化/还原按钮点击事件
     */
    void on_btnMenu_Max_clicked();
    
    /**
     * @brief 窗口关闭按钮点击事件
     */
    void on_btnMenu_Close_clicked();
};

#endif // FRMMAIN_H
