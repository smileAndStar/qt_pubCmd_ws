#ifndef WORKER_H
#define WORKER_H

#include <QObject>
#include "ros_cmd_vel/BtnControl.h" // 包含你的状态控制器

class Worker : public QObject
{
    Q_OBJECT

public:
    Worker(BtnController* controller);
    ~Worker();

public slots:
    // 耗时工作都放在槽函数中
    // 这个槽函数将包含我们的 while 循环
    void process();

signals:
    // 可以定义信号，用于从工作线程向UI线程传递信息（例如，错误信息）
    void error(QString err);
    // 工作完成后发出信号
    void finished();

private:
    BtnController* robot_controller_;
    bool is_running_;
};

#endif // WORKER_H