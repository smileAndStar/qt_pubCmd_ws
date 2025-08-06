#include "qt_cmd_vel/worker.h"
#include <ros/ros.h>
    
Worker::Worker(BtnController* controller) 
    : robot_controller_(controller), is_running_(true)
{}

Worker::~Worker() {
    is_running_ = false;
}

void Worker::process()
{
    ros::Rate loop_rate(5); // 设定5Hz频率

    while (ros::ok() && is_running_)
    {
        // 这里就是工作线程的核心任务：
        // 根据UI线程更新的状态，发布消息
        robot_controller_->publishOnce();

        // ros::spinOnce();
        loop_rate.sleep();
    }
    emit finished(); // 循环结束后发出完成信号
}