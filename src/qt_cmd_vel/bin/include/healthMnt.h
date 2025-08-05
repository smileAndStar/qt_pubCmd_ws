#pragma once

#include <string>
#include <vector>
#include <functional>
#include <thread>
#include <atomic>
#include <utility>

// 包含 C 库的头文件，并用 extern "C" 包裹
// 这告诉 C++ 编译器这些函数是用 C 语言链接规则编译的
extern "C" {
#include "health_monitor.h"
}

class HealthMonitor {
public:
    // 定义回调函数类型
    using DataCallback = std::function<void(const realtime_packet_t& data)>;

    /**
     * @brief 构造函数
     * @param port Linux下的串口设备路径
     */
    explicit HealthMonitor(std::string port);

    /**
     * @brief 析构函数 (RAII)
     *        自动断开连接并释放资源
     */
    ~HealthMonitor();

    // 禁止拷贝构造和拷贝赋值，避免资源管理问题
    HealthMonitor(const HealthMonitor&) = delete;
    HealthMonitor& operator=(const HealthMonitor&) = delete;

    /**
     * @brief 连接到模块
     * @return 成功返回 true, 失败返回 false
     */
    bool connect();

    /**
     * @brief 断开与模块的连接
     */
    void disconnect();
    
    /**
     * @brief 启动采集并通过回调函数异步返回数据
     * @param callback 当接收到新数据时调用的函数
     */
    void startCollection(DataCallback callback);
    
    /**
     * @brief 停止数据采集
     */
    void stopCollection();

    /**
     * @brief 让模块进入休眠
     */
    void enterSleep();

    /**
     * @brief 检查是否已连接
     */
    bool isConnected() const;

    /**
     * @brief (有状态) 计算RR间期散点图坐标
     *        此方法会记住上一包的最后一个有效值，以处理跨包情况
     * @param packet 当前的数据包
     * @return 一个包含(x, y)坐标对的vector
     */
    std::vector<std::pair<int, int>> getRRScatterPoints(const realtime_packet_t& packet);

private:
    void readLoop(); // 供后台线程运行的循环

    std::string m_port;     // 串口设备路径，比如 "/dev/ttyUSB0"
    health_monitor_handle m_handle; // C库的句柄
    bool m_connected = false;   // 是否已连接

    // 用于异步读取的成员
    std::thread m_readThread;   //读取数据线程
    std::atomic<bool> m_isRunning{false};
    DataCallback m_dataCallback;    // 回调函数

    // 用于跨包RR间期计算的状态
    uint8_t m_last_rr_value = 0;
};
