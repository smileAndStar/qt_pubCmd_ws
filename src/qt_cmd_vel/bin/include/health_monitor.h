#ifndef HEALTH_MONITOR_H
#define HEALTH_MONITOR_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// --- 指令定义 ---
#define CMD_COLLECTION_START_LEN 1
extern const uint8_t CMD_COLLECTION_START[CMD_COLLECTION_START_LEN];

#define CMD_COLLECTION_STOP_LEN 1
extern const uint8_t CMD_COLLECTION_STOP[CMD_COLLECTION_STOP_LEN];

#define CMD_SLEEP_LEN 1
extern const uint8_t CMD_SLEEP[CMD_SLEEP_LEN];

// --- 数据结构 ---

// 使用 __attribute__((packed)) 确保编译器不会在成员之间添加填充
// 这对于直接从串口读取原始数据到结构体中至关重要
typedef struct __attribute__((packed)) {
    uint8_t header;       // 0xFF数据包头
    int8_t  acdata[64];   // 心律波形数据 (有符号)
    uint8_t heartrate;    // 心率
    uint8_t spo2;         // 血氧
    uint8_t bk;           // 微循环
    uint8_t rsv1[8];      // 保留数据 1
    uint8_t sdnn;         // 心率变异性
    uint8_t rmssd;
    uint8_t nn50;
    uint8_t pnn50;
    uint8_t rra[6];       // RR间期
    uint8_t rsv2;         // 保留数据 2
    uint8_t state;        // 模块状态
} realtime_packet_t;


// 为我们的库句柄使用一个不透明指针，隐藏实现细节
struct health_monitor_s;        //结构体声明
typedef struct health_monitor_s* health_monitor_handle;

// --- API 函数 ---

/**
 * @brief 创建并初始化一个健康监护仪句柄
 * @param port_name Linux下的串口设备路径, 例如 "/dev/ttyUSB0"
 * @return 成功则返回句柄，失败返回 NULL
 */
health_monitor_handle hm_create(const char* port_name);

/**
 * @brief 销毁句柄并释放所有资源
 * @param handle hm_create 返回的句柄
 */
void hm_destroy(health_monitor_handle handle);

/**
 * @brief 连接到串口设备并进行配置
 * @param handle hm_create 返回的句柄
 * @return 成功返回 0, 失败返回 -1
 */
int hm_connect(health_monitor_handle handle);

/**
 * @brief 断开串口连接
 * @param handle hm_create 返回的句柄
 */
void hm_disconnect(health_monitor_handle handle);

/**
 * @brief 向模块发送一个通用指令
 * @param handle hm_create 返回的句柄
 * @param cmd 指向指令字节数组的指针
 * @param len 指令的长度
 * @return 成功返回 0, 失败返回 -1
 */
int hm_send_command(health_monitor_handle handle, const uint8_t* cmd, size_t len);

/**
 * @brief 读取一个完整的实时数据包 (阻塞函数)
 *        此函数会一直等待，直到收到一个以 0xFF 开头的完整数据包
 * @param handle hm_create 返回的句柄
 * @param packet 指向用于存储数据的 realtime_packet_t 结构体的指针
 * @return 成功返回 0, 失败返回 -1 (例如，读取错误或超时)
 */
int hm_read_realtime_packet(health_monitor_handle handle, realtime_packet_t* packet);


// --- 辅助功能 ---

/**
 * @brief 根据SDNN值获取疲劳状态
 * @param sdnn 从数据包中获取的SDNN值
 * @return 描述疲劳状态的字符串
 */
static inline const char* get_fatigue_status(uint8_t sdnn) {
    if (sdnn > 25) return "正常";
    if (sdnn >= 15) return "轻度疲劳";
    return "重度疲劳";
}

/**
 * @brief 检查状态字节以确定是否检测到用户
 * @param state 从数据包中获取的状态字节
 * @return 如果检测到用户则返回 true
 */
static inline bool is_user_detected(uint8_t state) {
    // 检查 bit2 是否为 0 (协议规定 bit2 置 1 表示未检测到)
    return (state & 0x04) == 0;
}

#endif // HEALTH_MONITOR_H