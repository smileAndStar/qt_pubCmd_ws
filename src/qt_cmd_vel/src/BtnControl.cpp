#include "ros_cmd_vel/BtnControl.h"

// 在构造函数中进行初始化
BtnController::BtnController(ros::NodeHandle &nh)
    : nh_(nh),
      cmdpub(nh), // 初始化底层的发布器
      current_state_(STOPPED),
      current_linear_speed_(0.2), // 设置一个默认的线速度
      current_angular_speed_(0.5)  // 设置一个默认的角速度
{
    ROS_INFO("机器人状态控制器已初始化。");
}

void BtnController::startMovingForward() {
    current_state_ = MOVING_FORWARD;
}

void BtnController::startMovingBackward() {
    current_state_ = MOVING_BACKWARD;
}

void BtnController::startTurningLeft() {
    current_state_ = TURNING_LEFT;
}

void BtnController::startTurningRight() {
    current_state_ = TURNING_RIGHT;
}

void BtnController::stopMovement() {
    current_state_ = STOPPED;
}

void BtnController::stateRockerControl() {
    // 切换到摇杆控制状态
    current_state_ = ROCKER_CONTROL;
}

void BtnController::setLinearSpeed(double speed) {
    // 只是更新速度值，然后让 updateMovement 根据当前状态决定如何使用它
    current_linear_speed_ = std::abs(speed);
}

void BtnController::setAngularSpeed(double angular_speed) {
    // 只是更新速度值，然后让 updateMovement 根据当前状态决定如何使用它
    current_angular_speed_ = std::abs(angular_speed);
}

void BtnController::updateRockerState(double normalized_x, double normalized_y) {
    // 根据摇杆位置更新当前状态
        // 计算速度分量
        // ros消息 x轴正方向是前进，y轴正方向是左平移，需要坐标变换
        // 位置关系归一化范围是 -1.0 到 1.0,对应速度范围是 -2.0 到 2.0 m/s
        linear_x = normalized_y * 2.0;
        linear_y = -normalized_x * 2.0;
}

void BtnController::publishOnce() {
    // 调用私有函数
    updateMovement();
}

void BtnController::updateMovement() {
    switch (current_state_) {
        case MOVING_FORWARD:
            cmdpub.moveInXYPlane(current_linear_speed_, 0.0);
            break;
        case MOVING_BACKWARD:
            cmdpub.moveInXYPlane(-current_linear_speed_, 0.0);
            break;
        case TURNING_LEFT:
            cmdpub.setTurnSpeed(current_angular_speed_);
            break;
        case TURNING_RIGHT:
            cmdpub.setTurnSpeed(-current_angular_speed_);
            break;
        case ROCKER_CONTROL:
            // 在摇杆控制状态下，使用摇杆的线速度分量
            cmdpub.moveInXYPlane(linear_x, linear_y);
            break;
        case STOPPED:
        default:
            cmdpub.stop();
            break;
    }
    // 发布消息
    cmdpub.publish();
}