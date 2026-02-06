# 9-IMU Array Data Acquisition (STM32F103C8T6)
[![Language](https://img.shields.io/badge/Language-C-green?logo=github&logoColor=white)](https://github.com/410659962/9-IMU-Array-Data-Acquisition.git)
[![Platform](https://img.shields.io/badge/ST-STM32F103C8T6-blue?style=for-the-badge&logo=stmicroelectronics&logoColor=white)](https://github.com/410659962/9-IMU-Array-Data-Acquisition.git)
The lower-level program of STM32F103C8T6 collects the acceleration and angular velocity data of 9 ICM-42605 sensors synchronously via SPI, and sends the data at a frequency of 100Hz through the serial port to the upper-level Qt system.

## 🔌 Hardware connection

| Component          | Connection method                                           |
| ------------------ | ----------------------------------------------------------- |
| STM32F103C8T6      | Main Controller (72 MHz)                                    |
| 9× ICM-42605       | SPI mode, independent chip select (PB0, PA4-PA0, PC15-PC13) |
| Power supply       | 3V                                                          |
| Serial port output | USART1 (PA9/TX, PA10/RX) @ 460800 bps                       |

## 📡 Data Protocol

Each frame is 222 bytes in size, with a continuous output rate of 100Hz.

```
[0xAA 0x55] + [54×float] + [0x00 0x00 0x80 0x7F]
  ↑             ↑             ↑
 Frame header   9 IMU      Frame end（NaN）
            (Accel X/Y/Z + Gyro X/Y/Z)
```

- **Acceleration:** ±4g range, unit g (1g = 9.8m/s²)
- **Gyroscope:** ±250 dps range, unit °/s
- **Accuracy:** Acceleration 16-bit (0.00012g), Gyroscope 16-bit (0.0076°/s)

## ⚙️ Compilation and burning

**1.Development Environment:**

- STM32CubeIDE 1.16.0
- HAL library (included in the project)

**2.Compilation steps:**
  
```
#Use STM32CubeIDE
1. Import project directory
2. Project → Build All
3. Use FlyMcu to burn the hex file
```
**3.Burn-in tool:**
* ST-Link V2
* USB-TTL (press the BOOT0 button to power on and enter DFU mode)
## 🔗 Equipped with a host computer
This firmware must be used in conjunction with the Qt host computer.

👉 [IMU Serial Acquisition Tool (Qt)](https://github.com/410659962/imu-serial-acquisition)
Upper computer functions:
* Real-time data display and visualization
* CSV data recording (with millisecond-level timestamp)
* Automatic serial port identification and frame synchronization
## 📌 Notes for Attention
* **First run:** The program will automatically detect 9 sensor IDs (0x42). If it fails, it will repeatedly send error messages.
* **Debugging:** Serial port outputs error message format init_error_sensor_X,YY (X = sensor number, YY = reading value)
## 📬 Support & Contributing 
* Pleaseadvise whether any other performance optimization strategies are available.
