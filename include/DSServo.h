#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/uart.h"

// ================= 协议常量 =================
#define DS_HEADER_TX        0xFF  // 发送包字头
#define DS_HEADER_RX_1      0xFF  // 接收包字头1
#define DS_HEADER_RX_2      0xF5  // 接收包字头2
#define DS_BROADCAST_ID     254   // 广播ID (0xFE)

// ================= 指令类型 =================
#define DS_CMD_PING         0x01  // 查询
#define DS_CMD_READ         0x02  // 读数据
#define DS_CMD_WRITE        0x03  // 写数据
#define DS_CMD_REG_WRITE    0x04  // 异步写 (配合ACTION执行)
#define DS_CMD_ACTION       0x05  // 执行异步写
#define DS_CMD_RESET        0x06  // 恢复出厂设置
#define DS_CMD_SYNC_WRITE   0x83  // 同步写

// ================= 核心寄存器地址 =================
#define DS_REG_ID           0x05  // 舵机ID (读写)
#define DS_REG_TORQUE_EN    0x1D  // 扭矩开关 (读写, 8bit)
#define DS_REG_BAUD_RATE    0x1E  // 波特率 (读写, 8bit)
#define DS_REG_TARGET_POS   0x2A  // 目标位置 (读写, 16bit)
#define DS_REG_RUN_TIME     0x2C  // 运行时间 (读写, 16bit)
#define DS_REG_CUR_POS      0x38  // 当前位置 (读, 16bit)
#define DS_REG_CUR_SPEED    0x3A  // 当前速度 (读, 16bit)
#define DS_REG_CUR_VOLT     0x3E  // 当前电压 (读, 8bit)
#define DS_REG_CUR_TEMP     0x3F  // 当前温度 (读, 8bit)
#define DS_REG_CAL_OFS_L    0x14  // 中位偏移低字节 (读写)
#define DS_REG_CAL_OFS_H    0x15  // 中位偏移高字节 (读写)

// 同步写结构体，用于一次性控制多个舵机
struct DSSyncWriteData {
    uint8_t id;
    uint16_t position;
    uint16_t time_ms;
};

class DSServo {
private:
    uart_port_t _uart_num;
    uint8_t _last_status[256]; // 缓存每个ID最后一次返回的应答状态
    
    // 底层数据包拼装与收发
    uint8_t calcChecksum(uint8_t id, uint8_t length, uint8_t cmd, const uint8_t *params, uint8_t param_len);
    esp_err_t sendPacket(uint8_t id, uint8_t cmd, const uint8_t *params, uint8_t param_len);
    int receivePacket(uint8_t id, uint8_t *out_params, uint8_t max_param_len, int timeout_ms);

public:
    DSServo();
    ~DSServo() = default;

    /**
     * @brief 初始化串口硬件
     */
    esp_err_t begin(uart_port_t uart_num, int tx_pin, int rx_pin, uint32_t baud_rate = 115200);

    /**
     * @brief 获取对应ID最后一次返回的状态码 (异常监控专用)
     */
    uint8_t getServoStatus(uint8_t id) { return _last_status[id]; }

    // ================= 1. 系统控制类 =================
    esp_err_t ping(uint8_t id);
    esp_err_t reset(uint8_t id);
    esp_err_t action();
    esp_err_t setServoBaudRate(uint8_t id, uint8_t baud_val);
    esp_err_t changeHostBaudRate(uint32_t new_baud_rate);
    esp_err_t calibrateOffset(uint8_t id);

    // ================= 2. 基础写控制 =================
    esp_err_t setTorque(uint8_t id, bool enable);
    esp_err_t setPosition(uint8_t id, uint16_t position, uint16_t time_ms = 0);
    
    // ================= 3. 异步写控制 =================
    esp_err_t setPositionAsync(uint8_t id, uint16_t position, uint16_t time_ms = 0);

    // ================= 4. 同步写控制 =================
    esp_err_t syncWritePosition(DSSyncWriteData *data_array, uint8_t servo_count);

    // ================= 5. 状态读取 =================
    int16_t getPosition(uint8_t id);
    int16_t getSpeed(uint8_t id);
    int16_t getVoltage(uint8_t id);
    int16_t getTemperature(uint8_t id);
};