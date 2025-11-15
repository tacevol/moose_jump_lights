# moose_jump_lights

This project connects the Adafruit BNO085 IMU to an ESP32 dev board using UART_RVC "robot vaccuum cleaner" protocol.

## Hardware
- Seeed Studio XIAO ESP32C3
- Adafruit BNO085 (STEMMA QT). 
  - PS0 tied high via solder jumper to set Mode to UART-RVC

## Features
- IMU data streaming over Wi-Fi
- Real-time data visualization via HTTP port
- Optional OTA flashing
