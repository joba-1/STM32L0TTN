# STM32L0TTN

Send BME280 data and voltage to TTN with STM32L0

## Build

* Download and install STM32CubeIDE (as of now version 1.4.2)
* Import this repo
* Open .ioc file and select "Project/Generate Code" (should download needed "firmware" files for the chip from ST)
* Build Release or Debug (currently RAM 95%, FLASH 88% used)

## Wiring

connect Gnd of all components

### STM32L011

Not checked without, but generally recommended to use a small ceramic (~10nF) and big electrolyt condensator (~10µF) between + and - near STM pins

![image](https://user-images.githubusercontent.com/32450554/130995507-a8109346-f09b-417d-b519-a5dc06214890.png)

* Look at the ioc file with CubeMX pinout view.
* Vdd _and_ Vdda to 1.8-3.3V
* Vss to Gnd
* PB9-Boot0 pulled to Gnd (~10k) to safely boot from flash
* STLink for flashing and debug: SWD, SWC, Gnd, NRst (and could provide Vdd via JP1)

### BME280

* SDA=MOSI, SDC=MISO, others should be obvious

### RFM95W

* Ant=~8cm wire (vary length and position to get best average RSSI as reported by TTN)
* NSS=CS, others should be obvious (DIO0, DIO5, MOSI, MISO, SCK and RESET)

RFM95W pin labels for reuse of ESP32 adapter breadboard (print with 600dpi)
![image](https://user-images.githubusercontent.com/32450554/130999682-5b1fd090-4951-47cc-89e3-87a70b8aead5.png)

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
* [Power consumption](http://www.st.com/content/ccc/resource/technical/document/application_note/27/58/8e/81/79/fb/4f/ac/DM00108286.pdf/files/DM00108286.pdf/jcr:content/translations/en.DM00108286.pdf)

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
* I couldnt make it work with LL lib, had to use big fat HAL for RCC and RTC. What is missing?

        LL_RTC_DisableWriteProtection(RTC);
        LL_RTC_WAKEUP_Disable(RTC);
        while( !LL_RTC_IsActiveFlag_WUTW(RTC) );
        LL_RTC_WAKEUP_SetClock(RTC, LL_RTC_WAKEUPCLOCK_CKSPRE);
        LL_RTC_WAKEUP_SetAutoReload(RTC, 3); // wake up every 3+1 s
        LL_RTC_EnableIT_WUT(RTC);
        LL_RTC_WAKEUP_Enable(RTC);
        LL_RTC_EnableWriteProtection(RTC);
        while( LL_RTC_IsActiveFlag_WUTW(RTC) );
        LL_RTC_ClearFlag_WUT(RTC);
        NVIC_ClearPendingIRQ(RTC_IRQn);

## Status

Works fine.
Power consumption much lower than with my similar ATTiny84 board

![Bildschirmfoto von 2021-08-20 11-51-18](https://user-images.githubusercontent.com/32450554/130215898-25305e28-8d08-4e3f-8a16-b70d2e87ce26.png)

Unfortunately it seems TTN will throw me out in september and I'm not sure I find the time and am able to support enough of lorawan on this chip until then to stay with them :(

## Done
Not needed. by far the most power is consumed by sending. These tasks are not relevant
* Sleep while waiting for sensor or transmitter (90% of the ~100ms active cycle is waiting)
* Cut power of sensor and transmitter between measurements? Standby consumption is already quite low.
Works fine for a long time now. No problems so far
* Check recommended hookup of STM32L011K4 (capacitors, boot mode lines, reset, ...?). Since it works, it is probably ok as described above :)

## Todo
* PCBs for coin cell or 2xAAA: coin cell not at all good at low temp -> abandoned for BMP280 (outdoor) variants
* 3D-printed housings
