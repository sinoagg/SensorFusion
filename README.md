# SensorFusion
 * [AEBS](https://en.wikipedia.org/wiki/AEBS) & [ADAS](https://en.wikipedia.org/wiki/ADAS)
## platform
 * FreeRTOS on STM32F413
## version
 * 20180719: **macro defines** changed to switch RADAR & VEHICLE, can3(vehicle & gyro) filter using multi channel
 * 20180716: **Radar(EMRR)** test in car, not with ADAS
 * 20180711: **gyro** read from stm32f1, mpu6050, via can3 
 * 20180524: **ADAS warning** then Radar FCW(macro defines to switch off DBC, Radar, uart send, ADAS, Radar & filter config)
 * 20180523: Radar & filter config
 * 20180522: ADAS, USART3
 * 20180521: CAN3
 * 20180514: test in ChongQing
## peripherals
* CAN
    * CAN1 to send distance received from Radar(**DBC**)
    * CAN2 to receive **Radar** data & config Radar
    * CAN3(250kbps) to communicate with **vehicle & gyroscope**
* USART
    * USART1(RS232) to receive cmd from **labview** & send Radar data to labview(not using)
    * USART3(TTL) to receive **ADAS** warnings
* LED
    * LED0: system start
    * LED1: **Radar** communication
    * LED2: **ADAS** communicatn
    * LED3: UART1(cmd) from **labview**
    * LED4: sound warning
    * LED5: Radar data Tx(**DBC**)
    * LED6: CAN Rx from **Vehicle**(speed & gyroscope)
* BUZZER
    * FCW: 1second(high), 0.5s(low)
    * LDW: 2s
