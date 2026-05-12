# DSServo_IDF 示例

## 目录结构

```
examples/
├── DS-S009/          # DS-S009 系列舵机示例
│   ├── Broadcast/    # 广播控制
│   ├── CalibrationOfs/  # 中位校准
│   ├── FeedBack/     # 反馈读取
│   ├── Ping/         # Ping测试
│   ├── ProgramEprom/ # EPROM编程
│   ├── RegWritePos/  # 异步写位置
│   ├── SyncRead/     # 同步读
│   ├── SyncWritePos/ # 同步写位置
│   ├── SyncWriteSpe/ # 同步写速度
│   ├── WritePos/     # 位置控制
│   └── WriteSpe/     # 恒速模式

```

## 使用方法

1. 选择需要的示例代码文件（如 `DS-S009/WritePos/WritePos.cpp`）
2. 将示例代码复制到您的项目 `main/` 目录下
3. 修改 UART 引脚配置以匹配您的硬件：as

```cpp
#define SERVO_UART_NUM    UART_NUM_1
#define SERVO_BAUDRATE    1000000
#define SERVO_TX_PIN      37   // 根据实际接线修改
#define SERVO_RX_PIN      38   // 根据实际接线修改
```

4. 编译并烧录：

```bash
idf.py build
idf.py flash monitor
```


## 注意事项

- 这些示例文件为独立的 `.cpp` 文件，不会被 DSServo_IDF 组件自动编译
- 使用时请将示例代码复制到您项目的 `main/` 目录，或将其内容整合到您现有的代码中
- 确保 CMakeLists.txt 中正确引用了 DSServo_IDF 组件