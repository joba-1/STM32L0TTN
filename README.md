# STM32L0TTN

Send BME280 data to TTN with STM32L0

## Build

* Download and install STM32CubeIDE (as of now version 1.4.2)
* Import this repo
* Build Release or Debug

## Wiring

connect Gnd of all components

### STM32L011

Not checked without, but generally recommended to use a small ceramic (~100nF) and big electrolyt condensator (~100µF) between + and - near STM pins

* Look at the ioc file with CubeMX pinout view.
* Vdd _and_ Vdda to 1.8-3.3V
* Vss to Gnd
* Boot0 pulled to Gnd (~10k) to safely boot from flash
* STLink: SWD, SWC, Gnd, NRst (and could provide Vdd via JP1)

### BME280

* SDA=MOSI, SDC=MISO, others should be obvious

### RFM95W

* Ant=~8cm wire (vary length and position to get best average RSSI as reported by TTN)
* NSS=CS, others should be obvious

### Serial

Rx of a usb to serial converter sends at 115200. I use this command (serial port name ttyUSB_ser_red is defined in my /etc/udev/rules.d)
    pio device monitor -p /dev/ttyUSB_ser_red -b 115200

### LED

can be used without resistor. I use 1kOhm to save battery

## References

Good resource for STM32 is [Stefan Frings](http://stefanfrings.de/stm32/stm32l0.html)
He has links to reference, datasheet, errata and more. E.g.:

* [STM32L011 Datasheet](https://www.st.com/resource/en/datasheet/stm32l011d3.pdf)
* [STM32L011 Reference](https://www.st.com/resource/en/reference_manual/dm00108282-ultra-low-power-stm32l0x1-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf)
* [STM32L011 Errata](https://www.st.com/resource/en/errata_sheet/dm00237371-stm32l011x3-4-device-limitations-stmicroelectronics.pdf)

[Power consumption](http://www.st.com/content/ccc/resource/technical/document/application_note/27/58/8e/81/79/fb/4f/ac/DM00108286.pdf/files/DM00108286.pdf/jcr:content/translations/en.DM00108286.pdf)

## Power Info Bits

* Examples for LL are here: STM32Cube/Repository/STM32Cube_FW_L0_V1.11.2/Projects/NUCLEO-L073RZ/Examples_LL/RTC/RTC_ExitStandbyWithWakeUpTimer
* Periodic wakeup timer is in Reference 22.4.6 (Periodic auto-wakeup)
* 1s to 18h periodic wakeup with ck_spre = 1s (see 22.4.3) and WUCKSEL [2:1] = 10
* Enable by WUTE bit in RTC_CR register
* Once setup it runs in normal and low power. 
* Sets WUTF flag in RTC_ISR if elapsed and reloads RTC_WUTR (timer counter). User resets WUTF
* Set WUTIE bit in RTC_CR to generate interrupt to wake up from low power modes
* System reset and Sleep, Stop, Standby have no influence on the WUT (wakeup timer)
* Save Lora frame counter in backup register: Reference 22.7.20 RTC backup registers (RTC_BKPxR)
* I couldnt make it work with LL lib, had to use big fat HAL for RCC and RTC

## Status

Works fine. 
Power consumption to be measured, but I expect it is lower than with ATTiny84

## Todo

* Sleep while waiting for sensor or transmitter (90% of the ~100ms active cycle is waiting)
* Cut power of sensor and transmitter between measurements? Standby consumption is already quite low.
* Check recommended hookup of STM32L011K4 (capacitors, boot mode lines, reset, ...?). Since it works, it is probably ok as described above :)
* PCBs for coin cell or 2xAAA
* 3D-printed housings
