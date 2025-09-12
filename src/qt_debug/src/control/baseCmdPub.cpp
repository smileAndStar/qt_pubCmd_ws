#include "baseCmdPub.h"
#include <chrono>

CmdPub::CmdPub(ros::NodeHandle &nh, double publish_rate) 
    : nh_(nh), is_publishing_(false), publish_rate_(publish_rate)
{
    // 创建 发布者 对象
    // 发布到 /cmd_vel 主题，队列大小为 10
    //cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    // 测试
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    
    // 初始化消息为零
    {
        /* 自动解锁的原理
        std::lock_guard 构造时加锁: 当 lock_guard 对象创建时，它会自动调用 mutex.lock()
        std::lock_guard 析构时解锁: 当 lock_guard 对象离开作用域（即到达 "}" 时），它的析构函数会自动调用 mutex.unlock()
        */
        std::lock_guard<std::mutex> lock(twist_mutex_);     // 自动加锁
        twist_msg_.linear.x = twist_msg_.linear.y = twist_msg_.linear.z = 0.0;
        twist_msg_.angular.x = twist_msg_.angular.y = twist_msg_.angular.z = 0.0;
    }
    
    ROS_INFO("异步CmdPub 初始化完毕，发布频率: %.1f Hz", publish_rate);
}

CmdPub::~CmdPub()
{
    // 确保在程序退出时机器人是停止的，并停止发布线程
    stopPublishing();
    stop();
}

void CmdPub::startPublishing()
{
    if (!is_publishing_.load()) {       // 如果还没有启动发布线程
        is_publishing_.store(true);     // 设置标志为正在发布
        publish_thread_ = std::thread(&CmdPub::publishThreadFunc, this);
        ROS_INFO("异步发布线程已启动");
    }
}

void CmdPub::stopPublishing()
{
    if (is_publishing_.load()) {
        is_publishing_.store(false);
        if (publish_thread_.joinable()) {   // 等待线程结束
            publish_thread_.join();         // 阻塞直到线程退出
        }
        ROS_INFO("异步发布线程已停止");
    }
}

void CmdPub::publishThreadFunc()
{
    auto period = std::chrono::duration<double>(1.0 / publish_rate_.load());
    
    while (is_publishing_.load() && ros::ok()) {
        auto start_time = std::chrono::steady_clock::now();
        
        // 发布当前消息
        {
            std::lock_guard<std::mutex> lock(twist_mutex_);
            cmd_vel_pub_.publish(twist_msg_);
        }
        
        // 检查频率是否更改
        auto current_rate = publish_rate_.load();
        period = std::chrono::duration<double>(1.0 / current_rate);
        
        // 精确定时
        auto end_time = start_time + period;
        std::this_thread::sleep_until(end_time);
    }
}

void CmdPub::organizePublish(double linear_x, double linear_y, double linear_z, 
                                           double angular_x, double angular_y, double angular_z)
{
    std::lock_guard<std::mutex> lock(twist_mutex_);
    twist_msg_.linear.x = linear_x;
    twist_msg_.linear.y = linear_y;
    twist_msg_.linear.z = linear_z;
    twist_msg_.angular.x = angular_x;
    twist_msg_.angular.y = angular_y;
    twist_msg_.angular.z = angular_z;
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

void CmdPub::setPublishRate(double rate)
{
    if (rate > 0.0) {
        publish_rate_.store(rate);
        ROS_INFO("发布频率已更新为: %.1f Hz", rate);
    } else {
        ROS_WARN("无效的发布频率: %.1f，必须大于0", rate);
    }
}

double CmdPub::getPublishRate() const
{
    return publish_rate_.load();
}

/*
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
    organizePublish(0.0, 0.0, 0.0, 0.0, 0.0, speed);
}
*/