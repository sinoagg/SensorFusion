# SensorFusion
 * [AEBS](https://en.wikipedia.org/wiki/AEBS) & [ADAS](https://en.wikipedia.org/wiki/ADAS)

## platform
 * FreeRTOS on STM32F413

## version
 * 20181213: **AEBS** modular enhanced
 * 20181212: added **dialing switch**
 * 20180920: Radar using CAN filter1, EMRR getting RadarData faster, Gyroscope in HighSensitivity
 * 20180910: **gyro** can1/3, **ARS408** config & turning warning
 * 20180816: Turning warning
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
    * CAN1 to send distance received from Radar(**DBC**)
    * CAN2 to receive **Radar** data & config Radar
    * CAN3(250kbps) to communicate with **vehicle & gyroscope**
* ADC
    * ADC1 in DMA2_stream0 to read ATM
* USART
    * USART1(RS232) to receive cmd from **labview** & send Radar data to labview(not using)
    * USART3(TTL) to receive **ADAS** warnings
* LED
    * LED0: Warning
    * LED1: **Radar** communication
    * LED2: **ADAS** communication
    * LED3: not using
    * LED4: **Vehicle** Speed & Status
    * LED5: Radar data Tx(**DBC**)
    * LED6: **gyroscope** communication
* BUZZER
    * FCW: 0.1s/0.1s pause(high), 0.3s/0.3s pause(low)
    * LDW: 1s/1s pause
