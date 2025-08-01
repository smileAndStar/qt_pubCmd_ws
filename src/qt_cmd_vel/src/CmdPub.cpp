#include "ros_cmd_vel/CmdPub.h"

CmdPub::CmdPub(ros::NodeHandle &nh) : nh_(nh)
{
    // 创建 发布者 对象
    // 发布到 /cmd_vel 主题，队列大小为 10
    //cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    // 测试
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    ROS_INFO("CmdPub 初始化完毕。");
}

CmdPub::~CmdPub()
{
    // 确保在程序退出时机器人是停止的
    stop();
}

void CmdPub::organizePublish(double linear_x, double linear_y, double linear_z, 
                                           double angular_x, double angular_y, double angular_z)
{
    twist_msg.linear.x = linear_x;
    twist_msg.linear.y = linear_y;
    twist_msg.linear.z = linear_z;
    twist_msg.angular.x = angular_x;
    twist_msg.angular.y = angular_y;
    twist_msg.angular.z = angular_z;
}

void CmdPub::stop()
{
    organizePublish(); // 调用时使用所有默认参数 (0.0)
}

void CmdPub::moveInXYPlane(double linear_x, double linear_y)
{
    organizePublish(linear_x, linear_y);
}

void CmdPub::setTurnSpeed(double yaw_speed)
{
    organizePublish(0.0, 0.0, 0.0, 0.0, 0.0, yaw_speed);
}

void CmdPub::setLinearX(double speed)
{
    organizePublish(speed);
}

void CmdPub::setLinearY(double speed)
{
    organizePublish(0.0, speed);
}

void CmdPub::setLinearZ(double speed)
{
    organizePublish(0.0, 0.0, speed);
}

void CmdPub::setAngularRoll(double speed)
{
    organizePublish(0.0, 0.0, 0.0, speed);
}

void CmdPub::setAngularPitch(double speed)
{
    organizePublish(0.0, 0.0, 0.0, 0.0, speed);
}

void CmdPub::setAngularYaw(double speed)
{
    setTurnSpeed(speed); // 直接复用 setTurnSpeed 函数
}

void CmdPub::publish()
{
    cmd_vel_pub_.publish(twist_msg);
}