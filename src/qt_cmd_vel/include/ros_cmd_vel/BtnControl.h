#ifndef BTNCONTROL_H
#define BTNCONTROL_H

#include "ros_cmd_vel/CmdPub.h" // 包含底层的发布器
#include <ros/ros.h>


/**
 * @enum MovementState
 * @brief 定义机器人当前的运动意图。
 */
enum MovementState {
    STOPPED,
    MOVING_FORWARD,
    MOVING_BACKWARD,
    TURNING_LEFT,
    TURNING_RIGHT,
    ROCKER_CONTROL      // 摇杆控制状态
};

/**
 * @class BtnController
 * @brief 一个带有状态管理的机器人控制器。
 * 
 * 这个类负责维护机器人的运动状态（如前进、后退）和速度值。
 * 当用户改变动作或速度时，它会计算出正确的速度指令并让 CmdVelPublisher 发布出去。
 * 这使得机器人可以持续运动，并且在运动中平滑地改变速度。
 */
class BtnController
{
public:
    /**
     * @brief BtnController 的构造函数。
     * @param nh ROS 节点句柄的引用。
     */
    BtnController(ros::NodeHandle &nh);

    /**
     * @brief 析构函数。
     */
    ~BtnController() = default;

    // --- 动作控制接口 ---

    /**
     * @brief 开始或继续前进。
     */
    void startMovingForward();

    /**
     * @brief 开始或继续后退。
     */
    void startMovingBackward();

    /**
     * @brief 开始或继续左转。
     */
    void startTurningLeft();

    /**
     * @brief 开始或继续右转。
     */
    void startTurningRight();

    /**
     * @brief 命令机器人停止。
     */
    void stopMovement();

    /**
     * @brief 切换到摇杆控制模式。
     * @note 这个函数会将当前状态设置为 ROCKER_CONTROL，并且不立即发布消息。
     */
    void stateRockerControl();

    // --- 速度设置接口 (通常由 UI 滑块或输入框调用) ---

    /**
     * @brief 设置线速度值。
     * @param speed 新的线速度值 (m/s)。
     * @note 调用此函数后，如果机器人正在前进或后退，它将以新速度继续运动。
     */
    void setLinearSpeed(double speed);

    /**
     * @brief 设置角速度值。
     * @param angular_speed 新的角速度值 (rad/s)。
     * @note 调用此函数后，如果机器人正在转向，它将以新速度继续转向。
     */
    void setAngularSpeed(double angular_speed);

    // --- 摇杆控制逻辑
    /**
     * @brief 更新摇杆状态。
     * @param normalized_x 小圆圆心与大圆圆心的X轴距离归一化值，范围从 -1.0 (最左) 到 1.0 (最右)。
     * @param normalized_y 小圆圆心与大圆圆心的Y轴距离归一化值，范围从 -1.0 (最下) 到 1.0 (最上)。
     * @note 这个函数会根据摇杆位置更新当前状态。
     */
    void updateRockerState(double normalized_x, double normalized_y);

    /**
     * @brief 公共函数。
     * @note 这个函数应该被一个高频率的循环（如在工作线程中）调用。
     */
    void publishOnce();
        
private:
    /**
     * @brief 根据当前状态和速度值，更新cmd消息内容，并发送消息一次。
     * 这是一个核心的私有函数，所有公共接口最终都会调用它。
     */
    void updateMovement();

    ros::NodeHandle nh_;
    CmdPub cmdpub; // 组合了底层的发布器

    MovementState current_state_; // 当前的运动状态
    double current_linear_speed_; // 保存当前设定的线速度,按键控制时使用
    double current_angular_speed_; // 保存当前设定的角速度，按键控制时使用
    double linear_x = 0; // 摇杆控制时的线速度X分量
    double linear_y = 0; // 摇杆控制时的线速度Y分量
};

#endif // BTNCONTROL_H