#include "DSServo.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

DSServo::DSServo() : _uart_num(UART_NUM_MAX) {
    // 初始化时清空状态缓存数组
    memset(_last_status, 0, sizeof(_last_status));
}

esp_err_t DSServo::begin(uart_port_t uart_num, int tx_pin, int rx_pin, uint32_t baud_rate) {
    _uart_num = uart_num;
    uart_config_t uart_config = {
        .baud_rate = (int)baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t err = uart_param_config(_uart_num, &uart_config);
    if (err != ESP_OK) return err;
    err = uart_set_pin(_uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) return err;
    return uart_driver_install(_uart_num, 1024, 1024, 0, NULL, 0);
}

// ================= 底层数据包拼装与收发 =================

uint8_t DSServo::calcChecksum(uint8_t id, uint8_t length, uint8_t cmd, const uint8_t *params, uint8_t param_len) {
    uint32_t sum = id + length + cmd;
    for(int i = 0; i < param_len; i++) sum += params[i];
    return (~sum) & 0xFF;
}

esp_err_t DSServo::sendPacket(uint8_t id, uint8_t cmd, const uint8_t *params, uint8_t param_len) {
    if (_uart_num == UART_NUM_MAX) return ESP_FAIL;
    uint8_t length = param_len + 2;
    uint8_t checksum = calcChecksum(id, length, cmd, params, param_len);

    uint8_t *tx_buf = (uint8_t*)malloc(param_len + 6);
    int idx = 0;
    tx_buf[idx++] = DS_HEADER_TX;
    tx_buf[idx++] = DS_HEADER_TX;
    tx_buf[idx++] = id;
    tx_buf[idx++] = length;
    tx_buf[idx++] = cmd;
    for (int i = 0; i < param_len; i++) tx_buf[idx++] = params[i];
    tx_buf[idx++] = checksum;

    // 【关键防御】发送前清空历史垃圾数据，防止干扰
    uart_flush_input(_uart_num); 
    
    uart_write_bytes(_uart_num, (const char*)tx_buf, idx);
    uart_wait_tx_done(_uart_num, pdMS_TO_TICKS(100)); // 阻塞等待硬件发送完毕
    
    free(tx_buf);
    return ESP_OK;
}

int DSServo::receivePacket(uint8_t id, uint8_t *out_params, uint8_t max_param_len, int timeout_ms) {
    uint8_t rx_buf[64];

    int len = uart_read_bytes(_uart_num, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(timeout_ms));
    if (len >= 6) {
        for (int i = 0; i < len - 5; i++) {
            // 【兼容模式】同时兼容 0xFF 和 0xF5 两种常见的返回字头
            if (rx_buf[i] == DS_HEADER_RX_1 && (rx_buf[i+1] == 0xFF || rx_buf[i+1] == 0xF5)) {
                uint8_t rx_id = rx_buf[i+2];
                uint8_t rx_len = rx_buf[i+3];
                uint8_t rx_status = rx_buf[i+4];
                
                if (rx_id != id) continue;
                if (i + 3 + rx_len >= len) continue;

                uint8_t param_count = rx_len - 2;
                uint8_t *rx_params = &rx_buf[i+5];
                uint8_t checksum = calcChecksum(rx_id, rx_len, rx_status, rx_params, param_count);
                
                if (checksum == rx_buf[i + 3 + rx_len]) {
                    // ★ 核心新增：只要校验通过，就把该舵机的真实异常状态保存下来！
                    _last_status[rx_id] = rx_status; 
                    
                    int copy_len = (param_count > max_param_len) ? max_param_len : param_count;
                    if (copy_len > 0 && out_params != nullptr) {
                        memcpy(out_params, rx_params, copy_len);
                    }
                    return copy_len;
                }
            }
        }
    }
    return -1;
}

// ================= API 实现 (大端模式适配版) =================

esp_err_t DSServo::ping(uint8_t id) {
    sendPacket(id, DS_CMD_PING, nullptr, 0);
    return (receivePacket(id, nullptr, 0, 20) >= 0) ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t DSServo::reset(uint8_t id) {
    return sendPacket(id, DS_CMD_RESET, nullptr, 0);
}

esp_err_t DSServo::action() {
    return sendPacket(DS_BROADCAST_ID, DS_CMD_ACTION, nullptr, 0);
}

esp_err_t DSServo::changeHostBaudRate(uint32_t new_baud_rate) {
    uart_wait_tx_done(_uart_num, pdMS_TO_TICKS(100));
    return uart_set_baudrate(_uart_num, new_baud_rate);
}

// ★ 带有 EEPROM 硬件解锁机制的波特率修改函数 ★
esp_err_t DSServo::setServoBaudRate(uint8_t id, uint8_t baud_val) {
    // 1. 🔑 发送解锁指令：向 0x30 (锁标志) 写入 0x00
    uint8_t unlock_params[2] = {0x30, 0x00};
    sendPacket(id, DS_CMD_WRITE, unlock_params, 2);
    
    // 给舵机内部 Flash 解锁留一点点时间
    vTaskDelay(pdMS_TO_TICKS(20)); 

    // 2. 📝 发送波特率修改指令：向 0x1E (波特率) 写入 baud_val (1M 对应 9)
    uint8_t baud_params[2] = {0x1E, baud_val};
    esp_err_t ret = sendPacket(id, DS_CMD_WRITE, baud_params, 2);
    
    // 给 Flash 擦写留一点点时间
    vTaskDelay(pdMS_TO_TICKS(20));

    // 3. 🔒 发送上锁指令：向 0x30 (锁标志) 重新写入 0x01，保护底层数据
    uint8_t lock_params[2] = {0x30, 0x01};
    sendPacket(id, DS_CMD_WRITE, lock_params, 2);

    return ret;
}

esp_err_t DSServo::setTorque(uint8_t id, bool enable) {
    uint8_t params[2] = {DS_REG_TORQUE_EN, (uint8_t)(enable ? 1 : 0)};
    uint8_t cmd = (id == DS_BROADCAST_ID) ? DS_CMD_REG_WRITE : DS_CMD_WRITE;
    esp_err_t ret = sendPacket(id, cmd, params, 2);
    if (id == DS_BROADCAST_ID) action();
    return ret;
}

esp_err_t DSServo::setPosition(uint8_t id, uint16_t position, uint16_t time_ms) {
    uint8_t params[5] = { 
        DS_REG_TARGET_POS, 
        (uint8_t)(position >> 8), (uint8_t)(position & 0xFF), // 严格大端：高位在前
        (uint8_t)(time_ms >> 8), (uint8_t)(time_ms & 0xFF) 
    };
    uint8_t cmd = (id == DS_BROADCAST_ID) ? DS_CMD_REG_WRITE : DS_CMD_WRITE;
    esp_err_t ret = sendPacket(id, cmd, params, 5);
    if (id == DS_BROADCAST_ID) action();
    return ret;
}

esp_err_t DSServo::setPositionAsync(uint8_t id, uint16_t position, uint16_t time_ms) {
    uint8_t params[5] = { 
        DS_REG_TARGET_POS, 
        (uint8_t)(position >> 8), (uint8_t)(position & 0xFF), 
        (uint8_t)(time_ms >> 8), (uint8_t)(time_ms & 0xFF) 
    };
    return sendPacket(id, DS_CMD_REG_WRITE, params, 5);
}

esp_err_t DSServo::syncWritePosition(DSSyncWriteData *data_array, uint8_t servo_count) {
    uint8_t param_len = 2 + (servo_count * 5); 
    uint8_t *params = (uint8_t*)malloc(param_len);
    
    params[0] = DS_REG_TARGET_POS;  
    params[1] = 0x04;               
    
    int idx = 2;
    for(int i = 0; i < servo_count; i++) {
        params[idx++] = data_array[i].id;
        params[idx++] = (data_array[i].position >> 8) & 0xFF; // 大端
        params[idx++] = data_array[i].position & 0xFF;        
        params[idx++] = (data_array[i].time_ms >> 8) & 0xFF;
        params[idx++] = data_array[i].time_ms & 0xFF;
    }

    esp_err_t ret = sendPacket(DS_BROADCAST_ID, DS_CMD_SYNC_WRITE, params, param_len);
    free(params);
    action(); 
    return ret;
}

int16_t DSServo::getPosition(uint8_t id) {
    uint8_t params[2] = {DS_REG_CUR_POS, 2};
    sendPacket(id, DS_CMD_READ, params, 2);
    uint8_t rx[2];
    if (receivePacket(id, rx, 2, 20) == 2) {
        return (int16_t)((rx[0] << 8) | rx[1]); // 大端解析：高位在rx[0]
    }
    return -1;
}

int16_t DSServo::getSpeed(uint8_t id) {
    uint8_t params[2] = {DS_REG_CUR_SPEED, 2};
    sendPacket(id, DS_CMD_READ, params, 2);
    uint8_t rx[2];
    if (receivePacket(id, rx, 2, 20) == 2) {
        return (int16_t)((rx[0] << 8) | rx[1]); 
    }
    return -1;
}

int16_t DSServo::getVoltage(uint8_t id) {
    uint8_t params[2] = {DS_REG_CUR_VOLT, 1};
    sendPacket(id, DS_CMD_READ, params, 2);
    uint8_t rx[1];
    if (receivePacket(id, rx, 1, 20) == 1) {
        return (int16_t)rx[0];
    }
    return -1;
}

int16_t DSServo::getTemperature(uint8_t id) {
    uint8_t params[2] = {DS_REG_CUR_TEMP, 1};
    sendPacket(id, DS_CMD_READ, params, 2);
    uint8_t rx[1];
    if (receivePacket(id, rx, 1, 20) == 1) {
        return (int16_t)rx[0];
    }
    return -1;
}

esp_err_t DSServo::calibrateOffset(uint8_t id) {
    printf("\n========================================\n");
    printf("[校准] ID=%d 中位校准开始\n", id);
    printf("========================================\n");

    // Step 1: 开扭矩，停到中位 2048
    setTorque(id, true);
    vTaskDelay(pdMS_TO_TICKS(50));
    setPosition(id, 2048, 500);
    vTaskDelay(pdMS_TO_TICKS(800));

    // Step 2: 读取当前位置（看舵机实际到达的位置）
    int16_t actual_pos = 0;
    int attempts = 0;
    while (attempts < 5) {
        actual_pos = getPosition(id);
        if (actual_pos > 0) break;
        attempts++;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    printf("[校准] 目标位置: 2048, 实际位置: %d\n", actual_pos);

    // Step 3: 计算偏移 (目标2048 vs 实际)
    int16_t offset = 2048 - actual_pos;
    printf("[校准] 计算偏移量: %d (0x%04X)\n", offset, (uint16_t)offset & 0xFFFF);

    // Step 4: 写入偏移到 EEPROM (先解锁)
    uint8_t unlock[2] = {0x30, 0x00};
    sendPacket(id, DS_CMD_WRITE, unlock, 2);
    vTaskDelay(pdMS_TO_TICKS(20));

    uint8_t ofs_params[3] = {
        DS_REG_CAL_OFS_L,
        (uint8_t)(offset & 0xFF),           // 低字节
        (uint8_t)((offset >> 8) & 0xFF)     // 高字节
    };
    esp_err_t ret = sendPacket(id, DS_CMD_WRITE, ofs_params, 3);
    vTaskDelay(pdMS_TO_TICKS(20));

    // Step 5: 上锁
    uint8_t lock[2] = {0x30, 0x01};
    sendPacket(id, DS_CMD_WRITE, lock, 2);
    vTaskDelay(pdMS_TO_TICKS(20));

    // Step 6: 验证 — 读回偏移值
    uint8_t rd_params[2] = {DS_REG_CAL_OFS_L, 2};
    sendPacket(id, DS_CMD_READ, rd_params, 2);
    uint8_t rd_buf[2] = {0};
    int rd_len = receivePacket(id, rd_buf, 2, 30);
    if (rd_len == 2) {
        int16_t saved_ofs = (int16_t)((rd_buf[1] << 8) | rd_buf[0]);
        printf("[校准] 读回偏移值: %d (0x%04X)\n", saved_ofs, (uint16_t)saved_ofs & 0xFFFF);
    } else {
        printf("[校准] 警告: 无法读回偏移值验证 (len=%d)\n", rd_len);
    }

    printf("[校准] ID=%d 校准完成 (ret=%d)\n", id, ret);
    printf("========================================\n\n");
    return ret;
}