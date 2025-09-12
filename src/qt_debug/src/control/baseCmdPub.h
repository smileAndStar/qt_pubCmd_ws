#ifndef CMDPUB_H
#define CMDPUB_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <mutex>
#include <atomic>
#include <thread>

/**
 * @class CmdPub
 * @brief 一个异步的底层 cmd_vel 组织发布类，可控制所有6个自由度。
 *
 * 这个类提供对 geometry_msgs::Twist 全部参数的精细控制，并封装了
 * 常用的运动模式，如平面移动(摇杆控制)和原地转向(原地旋转)。
 * 支持异步发布，自动以固定频率发布最新的控制指令。
 */
class CmdPub
{
public:
    /**
     * @brief 构造函数。
     * @param nh ROS 节点句柄的引用。
     * @param publish_rate 发布频率 (Hz)，默认10Hz
     */
    CmdPub(ros::NodeHandle &nh, double publish_rate = 10.0);

    /**
     * @brief 析构函数，在对象销毁时会自动停止机器人和发布线程。
     */
    ~CmdPub();

    /**
     * @brief 启动异步发布线程
     */
    void startPublishing();

    /**
     * @brief 停止异步发布线程
     */
    void stopPublishing();

private:
    // --- 核心控制函数 ---
    /**
     * @brief 组织一个完整的 cmd 消息，线程安全更新内部消息。
     * @param linear_x X轴线速度 (前进/后退)
     * @param linear_y Y轴线速度 (左/右平移)
     * @param linear_z Z轴线速度 (上升/下降)
     * @param angular_x X轴角速度 (翻滚/Roll)
     * @param angular_y Y轴角速度 (俯仰/Pitch)
     * @param angular_z Z轴角速度 (偏航/Yaw, 即左右转向)
     */
    void organizePublish(double linear_x = 0.0, double linear_y = 0.0, double linear_z = 0.0, 
                      double angular_x = 0.0, double angular_y = 0.0, double angular_z = 0.0);

    /**
     * @brief 发布线程函数
     */
    void publishThreadFunc();

    // --- 保留 ---
    // --- 单独设置每个自由度的函数 ---
    
    // /** @brief 只设置X轴线速度 (前进/后退) */
    // void setLinearX(double speed);
    // /** @brief 只设置Y轴线速度 (左/右平移) */
    // void setLinearY(double speed);
    // /** @brief 只设置Z轴线速度 (上升/下降) */
    // void setLinearZ(double speed);
    // /** @brief 只设置X轴角速度 (翻滚/Roll) */
    // void setAngularRoll(double speed);
    // /** @brief 只设置Y轴角速度 (俯仰/Pitch) */
    // void setAngularPitch(double speed);
    // /** @brief 只设置Z轴角速度 (偏航/Yaw) - 等同于 setTurnSpeed */
    // void setAngularYaw(double speed);

public:
    // === 封装简易使用的控制API ===

    /**
     * @brief 停止所有运动。
     */
    void stop();

    /**
     * @brief 控制机器人在XY平面上进行任意方向的移动。
     *        这对于全向轮 (mecanum) 或腿式机器人非常有用。
     * @param linear_x X轴方向的速度分量。
     * @param linear_y Y轴方向的速度分量。
     */
    void moveInXYPlane(double linear_x, double linear_y);

    /**
     * @brief 设置机器人的转向速度（绕Z轴偏航）。
     *        这是最常用的转向方式。异步更新，立即生效。
     * @param yaw_speed 偏航角速度 (rad/s)。正值为左转(逆时针)，负值为右转(顺时针)。
     */
    void setTurnSpeed(double yaw_speed);

    /**
     * @brief 设置发布频率
     * @param rate 新的发布频率 (Hz)
     */
    void setPublishRate(double rate);

    /**
     * @brief 获取当前发布频率
     * @return 当前发布频率 (Hz)
     */
    double getPublishRate() const;

private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;         // 发布者对象，用于发布 cmd_vel 消息
    geometry_msgs::Twist twist_msg_;     // 发布的消息实例
    
    // 异步发布相关
    std::thread publish_thread_;         // 发布线程
    std::atomic<bool> is_publishing_;    // 是否正在发布的标志
    std::atomic<double> publish_rate_;   // 发布频率
    mutable std::mutex twist_mutex_;     // 保护twist_msg_的互斥锁
};

#endif // CMDPUB_H