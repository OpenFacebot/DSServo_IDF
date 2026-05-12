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
#define DS_CMD_SYNC_READ    0x82  // 同步读

// ================= 核心寄存器地址 =================
#define DS_REG_ID           0x05  // 舵机ID (读写)
#define DS_REG_TORQUE_EN    0x1D  // 扭矩开关 (读写, 8bit)
#define DS_REG_BAUD_RATE    0x1E  // 波特率 (读写, 8bit)
#define DS_REG_TARGET_POS   0x2A  // 目标位置 (读写, 16bit)
#define DS_REG_RUN_TIME     0x2C  // 运行时间 (读写, 16bit)
#define DS_REG_MODE         0x21  // 工作模式 (读写, 8bit, 0=舵机 1=恒速)
#define DS_REG_CUR_POS      0x38  // 当前位置 (读, 16bit)
#define DS_REG_CUR_SPEED    0x3A  // 当前速度 (读, 16bit)
#define DS_REG_CUR_VOLT     0x3E  // 当前电压 (读, 8bit)
#define DS_REG_CUR_TEMP     0x3F  // 当前温度 (读, 8bit)
#define DS_REG_CAL_OFS_L    0x14  // 中位偏移低字节 (读写)
#define DS_REG_CAL_OFS_H    0x15  // 中位偏移高字节 (读写)
#define DS_REG_CUR_LOAD     0x3C  // 当前负载 (读, 16bit)

// 同步写结构体，用于一次性控制多个舵机
struct DSSyncWriteData {
    uint8_t id;
    uint16_t position;
    uint16_t time_ms;
};

// 一次回读所有状态的缓存结构体
struct ServoFeedback {
    int16_t pos;
    int16_t speed;
    int16_t load;
    int16_t voltage;
    int16_t temperature;
    int16_t moving;   // 1=运动中, 0=静止
    int16_t current;  // 电流 (mA, 舵机不支持则为-1)
    bool valid;
};

class DSServo {
private:
    uart_port_t _uart_num;
    uint8_t _last_status[256];
    ServoFeedback _feedback[256];
    int16_t _last_pos[256];
    bool _last_error;

    // 底层数据包拼装与收发
    uint8_t calcChecksum(uint8_t id, uint8_t length, uint8_t cmd, const uint8_t *params, uint8_t param_len);
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

    /**
     * @brief 检查上一次操作是否出错
     */
    bool getLastError() { return _last_error; }

    // ================= 0. 底层直通 (高级用户) =================
    /// 直接发送原始数据包。id=254=广播, cmd=DS_CMD_WRITE/READ/...
    /// 用户需自行拼 params 数组和处理校验和
    esp_err_t sendPacket(uint8_t id, uint8_t cmd, const uint8_t *params, uint8_t param_len);

    // ================= 1. 系统控制类 =================
    esp_err_t ping(uint8_t id);
    esp_err_t reset(uint8_t id);
    esp_err_t action();
    esp_err_t setServoBaudRate(uint8_t id, uint8_t baud_val);
    esp_err_t changeHostBaudRate(uint32_t new_baud_rate);
    esp_err_t calibrateOffset(uint8_t id);
    esp_err_t unLockEprom(uint8_t id);
    esp_err_t LockEprom(uint8_t id);
    esp_err_t writeByte(uint8_t id, uint8_t reg, uint8_t value);

    // ================= 2. 基础写控制 =================
    esp_err_t setTorque(uint8_t id, bool enable);
    esp_err_t setPosition(uint8_t id, uint16_t position, uint16_t time_ms = 0);

    // ================= 2.5 恒速模式 =================
    esp_err_t WheelMode(uint8_t id);
    esp_err_t ServoMode(uint8_t id);
    esp_err_t WriteSpe(uint8_t id, int16_t speed, uint8_t acc = 50);

    // ================= 3. 异步写控制 =================
    esp_err_t setPositionAsync(uint8_t id, uint16_t position, uint16_t time_ms = 0);

    // ================= 4. 同步写控制 =================
    esp_err_t syncWritePosition(DSSyncWriteData *data_array, uint8_t servo_count);

    // ================= 4.5 同步读控制 =================
    /**
     * @brief 发送同步读广播指令，请求多个舵机同时返回指定寄存器的数据
     * @param ids 舵机 ID 数组
     * @param id_count ID 数量
     * @param start_reg 起始寄存器地址 (如 DS_REG_CUR_POS=0x38)
     * @param bytes_per 每个舵机返回的字节数 (如位置=2字节)
     */
    esp_err_t syncReadPacketTx(const uint8_t *ids, uint8_t id_count,
                                uint8_t start_reg, uint8_t bytes_per);

    /**
     * @brief 接收同步读返回的某个舵机数据
     * @param id 要接收的舵机 ID
     * @param rx_buf 存放接收数据的缓冲区
     * @param rx_len 缓冲区大小
     * @return 实际接收到的参数字节数，失败返回 -1
     */
    int16_t syncReadPacketRx(uint8_t id, uint8_t *rx_buf, uint8_t rx_len, int timeout_ms = 50);

    /**
     * @brief 从同步读返回包中提取 16 位整数 (大端)
     * @param rx_buf 同步读接收到的数据
     * @param offset 参数中的偏移位置 (从 0 开始)
     * @return int16_t 值
     */
    int16_t syncReadRxPacketToWord(const uint8_t *rx_buf, uint8_t offset);

    // ================= 5. 状态读取 (单寄存器) =================
    int16_t getPosition(uint8_t id);
    int16_t getSpeed(uint8_t id);
    int16_t getLoad(uint8_t id);
    int16_t getVoltage(uint8_t id);
    int16_t getTemperature(uint8_t id);

    // ================= 6. 批量回读 (FeedBack 风格) =================
    /**
     * @brief 一次性回读舵机的全部反馈参数 (位置/速度/负载/电压/温度/运动状态)
     *        结果缓存在内部，之后通过 ReadPos/ReadSpeed/... 系列函数读取
     * @param id 舵机 ID
     * @return ESP_OK 成功, ESP_FAIL 通信失败
     */
    esp_err_t FeedBack(uint8_t id);

    /// 读取 FeedBack 缓存的位置值
    int16_t ReadPos(uint8_t id) { return _feedback[id].pos; }
    /// 读取 FeedBack 缓存的速度值
    int16_t ReadSpeed(uint8_t id) { return _feedback[id].speed; }
    /// 读取 FeedBack 缓存的负载值
    int16_t ReadLoad(uint8_t id) { return _feedback[id].load; }
    /// 读取 FeedBack 缓存的电压值
    int16_t ReadVoltage(uint8_t id) { return _feedback[id].voltage; }
    /// 读取 FeedBack 缓存的温度值
    int16_t ReadTemper(uint8_t id) { return _feedback[id].temperature; }
    /// 读取 FeedBack 缓存的运动状态 (1=运动中, 0=静止)
    int16_t ReadMove(uint8_t id) { return _feedback[id].moving; }
    /// 读取 FeedBack 缓存的电流值 (mA, 不支持则为 -1)
    int16_t ReadCurrent(uint8_t id) { return _feedback[id].current; }
};