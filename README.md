# moose_jump_lights

This project connects the Adafruit BNO085 IMU to an ESP32 dev board using UART_RVC "robot vaccuum cleaner" protocol.

## Hardware
- Seeed Studio XIAO ESP32C3
- Adafruit BNO085 (STEMMA QT). 
  - PS0 tied high via solder jumper to set Mode to UART-RVC
- Enclosure: https://cad.onshape.com/documents/02fdb3756ca32e95a18bf463/w/40d46b2ee5e18198ef4add3b/e/a4df8785b36b905605e8964c?renderMode=0&uiState=695c757f162304913c98f0e5

## Features
- IMU data streaming over Wi-Fi
- Real-time data visualization via HTTP port
- Optional OTA flashing
