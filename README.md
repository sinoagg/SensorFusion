# SensorFusion
 * [AEBS](https://en.wikipedia.org/wiki/AEBS) & [ADAS](https://en.wikipedia.org/wiki/ADAS)
## platform
 * FreeRTOS on STM32F413
## version
 * 20180816: gyro via CAN1(500kbps) & Turning warning, for Benz
 * 20180811: led 0 warning
 * 20180804: fix bug of getting Radar for the "<<" priority is low
 * 20180730: init periphrals in separate files to simplify code structure
 * 20180724: **ATM(adc)** read in DMA
 * 20180719: **Radar(EMRR)** test on car, with ADAS
 * 20180716: **Radar(EMRR)** test on car, not with ADAS
 * 20180711: **gyro** read from stm32f1, mpu6050, via can3 
 * 20180524: **ADAS warning** then Radar FCW(macro defines to switch off DBC, Radar, uart send, ADAS, Radar & filter config)
 * 20180523: Radar & filter config
 * 20180522: ADAS, USART3
 * 20180521: CAN3
 * 20180514: test in ChongQing
## peripherals
* CAN
    * CAN1(500kbps) to communicate with **gyroscope**
    * CAN2 to receive **Radar** data & config Radar
* ADC
    * ADC1 in DMA2_stream0 to read ATM
* USART
    * USART1(RS232) to receive cmd from **labview** & send Radar data to labview(not using)
    * USART3(TTL) to receive **ADAS** warnings
* LED
    * LED0: Warning
    * LED1: **Radar** communication
    * LED2: **ADAS** communicatn
    * LED3: UART1(cmd) from **labview**
    * LED4: Not using
    * LED5: Radar data Tx(**DBC**)
    * LED6: CAN Rx from **Vehicle**(speed & gyroscope)
* BUZZER
    * FCW: 1second(high), 0.5s(low)
    * LDW: 2s
