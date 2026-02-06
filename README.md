# 使用STM32F103C8T6读取由9个ICM42605组成的阵列数据

[![Language](https://img.shields.io/badge/Language-C-green?logo=github&logoColor=white)](https://github.com/410659962/9-IMU-Array-Data-Acquisition.git)
[![Platform](https://img.shields.io/badge/ST-STM32F103C8T6-blue?style=for-the-badge&logo=stmicroelectronics&logoColor=white)](https://github.com/410659962/9-IMU-Array-Data-Acquisition.git)

STM32F103C8T6 的底层程序通过 SPI 协议同步采集 9 个 ICM-42605 传感器的加速度和角速度数据，并以 100Hz 的频率通过串行端口将数据发送至上层的 Qt 系统。

## 🔌 硬件连接

| 组件          | 备注                                        |
| ------------- | ------------------------------------------- |
| STM32F103C8T6 | 主控芯片 (72 MHz)                           |
| 9× ICM-42605  | SPI模式, 单独片选 (PB0, PA4-PA0, PC15-PC13) |
| 电源支持      | 3V                                          |
| 串口输出      | USART1 (PA9/TX, PA10/RX) @ 460800 bps       |

## 📡 数据协议

每帧的大小为 222 字节，输出速率连续稳定在 100 Hz。

```
[0xAA 0x55] + [54×float] + [0x00 0x00 0x80 0x7F]
  ↑             ↑             ↑
 帧头           9 IMU        帧尾（NaN）
            (Accel X/Y/Z + Gyro X/Y/Z)
```

- **加速度计:** ±4g 量程, 单位 g (1g = 9.8m/s²)
- **陀螺仪:** ±250 dps 量程, 单位 °/s
- **精度:** 加速度计 16-bit (0.00012g), 陀螺仪 16-bit (0.0076°/s)

## ⚙️ 编译与烧录

**1.支持环境:**

- STM32CubeIDE 1.16.0
- HAL库(included in the project)

**2.编译步骤:**

```
#使用STM32CubeIDE
1. 导入项目
2. 构建
3. 使用FlyMcu把hex文件烧录到芯片
```

**3.烧录工具:**

- ST-Link V2
- USB-TTL

## 🔗 配备了上位机

此固件与上位机配合使用。

👉 [IMU Serial Acquisition Tool (Qt)](https://github.com/410659962/imu-serial-acquisition)

上位机功能：

- 实时数据展示与可视化
- CSV 数据记录（带有毫秒级时间戳）
- 长时采集模式

## 📌 注意

- **读ID:** 该程序会自动识别 9 个传感器标识（0x42）。如果识别失败，它会不断发送错误信息。
- **错误信息格式:** 串行端口输出错误消息格式“init_error_sensor_X,YY”（其中 X 表示传感器编号，YY 表示读取值）

## 📬 支持与贡献

- 请告知是否还有其他性能优化策略可供采用。
