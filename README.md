# SensorFusion
 * [AEBS](https://en.wikipedia.org/wiki/AEBS) & [ADAS](https://en.wikipedia.org/wiki/ADAS)
## platform
 * FreeRTOS on STM32F413
## peripherals
 * CAN1 to send distance received from Radar(DBC)
 * CAN2 to receive Radar data & config Radar
 * CAN3(250kbps) to communicate with vehicle & gyroscope
---
 * USART1(RS232) to receive cmd from labview & send Radar data to labview(not using)
 * USART3(TTL) to receive ADAS warnings
---
 * LED0~6 to display system state