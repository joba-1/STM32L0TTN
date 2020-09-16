EESchema Schematic File Version 2
LIBS:stm32
LIBS:power
LIBS:device
LIBS:switches
LIBS:relays
LIBS:motors
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:RFM9X
LIBS:Sensor
LIBS:ST32ML011K4-Button-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "STM32L0TTN"
Date "2020-09-14"
Rev "1"
Comp "Joachim Banzhaf"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L STM32L031F4Px U1
U 1 1 5F5F321C
P 5700 2400
F 0 "U1" H 2400 3325 50  0000 L BNN
F 1 "STM32L031F4Px" H 9000 3325 50  0000 R BNN
F 2 "Housings_SSOP:TSSOP-20_4.4x6.5mm_Pitch0.65mm" H 9000 3275 50  0001 R TNN
F 3 "" H 5700 2400 50  0001 C CNN
	1    5700 2400
	1    0    0    -1  
$EndComp
$Comp
L RFM9X U3
U 1 1 5F5F359C
P 2700 5050
F 0 "U3" H 2700 4350 60  0000 C CNN
F 1 "RFM9X" H 2700 5750 60  0000 C CNN
F 2 "RF_Modules:Hopref_RFM9XW_SMD" H 2700 4800 60  0001 C CNN
F 3 "" H 2700 4800 60  0001 C CNN
	1    2700 5050
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 5F5F3A50
P 2150 2000
F 0 "R2" V 2230 2000 50  0000 C CNN
F 1 "10k" V 2150 2000 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 2080 2000 50  0001 C CNN
F 3 "" H 2150 2000 50  0001 C CNN
	1    2150 2000
	0    1    1    0   
$EndComp
Text GLabel 4100 5400 2    60   Input ~ 0
MOSI
Text GLabel 4100 5600 2    60   Output ~ 0
MISO
Text GLabel 4100 5300 2    60   Input ~ 0
SCK
$Comp
L Earth #PWR01
U 1 1 5F5F4B15
P 4600 5200
F 0 "#PWR01" H 4600 4950 50  0001 C CNN
F 1 "Earth" H 4600 5050 50  0001 C CNN
F 2 "" H 4600 5200 50  0001 C CNN
F 3 "" H 4600 5200 50  0001 C CNN
	1    4600 5200
	1    0    0    -1  
$EndComp
Text GLabel 4100 5500 2    60   Input ~ 0
BME_CS
$Comp
L +3.3V #PWR02
U 1 1 5F5F4CD2
P 4100 5100
F 0 "#PWR02" H 4100 4950 50  0001 C CNN
F 1 "+3.3V" H 4100 5240 50  0000 C CNN
F 2 "" H 4100 5100 50  0001 C CNN
F 3 "" H 4100 5100 50  0001 C CNN
	1    4100 5100
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR03
U 1 1 5F5F4D98
P 3100 5050
F 0 "#PWR03" H 3100 4900 50  0001 C CNN
F 1 "+3.3V" H 3100 5190 50  0000 C CNN
F 2 "" H 3100 5050 50  0001 C CNN
F 3 "" H 3100 5050 50  0001 C CNN
	1    3100 5050
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR04
U 1 1 5F5F4DB4
P 3100 5600
F 0 "#PWR04" H 3100 5350 50  0001 C CNN
F 1 "Earth" H 3100 5450 50  0001 C CNN
F 2 "" H 3100 5600 50  0001 C CNN
F 3 "" H 3100 5600 50  0001 C CNN
	1    3100 5600
	1    0    0    -1  
$EndComp
Text GLabel 2300 5600 0    60   Input ~ 0
RFM_CS
Text GLabel 2300 5400 0    60   Input ~ 0
MOSI
Text GLabel 2300 5300 0    60   Output ~ 0
MISO
Text GLabel 2300 5500 0    60   Input ~ 0
SCK
Text GLabel 2300 5000 0    60   Output ~ 0
RX_READY
Text GLabel 2300 4500 0    60   Output ~ 0
TX_READY
$Comp
L Earth #PWR05
U 1 1 5F5F538E
P 3300 4300
F 0 "#PWR05" H 3300 4050 50  0001 C CNN
F 1 "Earth" H 3300 4150 50  0001 C CNN
F 2 "" H 3300 4300 50  0001 C CNN
F 3 "" H 3300 4300 50  0001 C CNN
	1    3300 4300
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR06
U 1 1 5F5F59D6
P 2000 2100
F 0 "#PWR06" H 2000 1850 50  0001 C CNN
F 1 "Earth" H 2000 1950 50  0001 C CNN
F 2 "" H 2000 2100 50  0001 C CNN
F 3 "" H 2000 2100 50  0001 C CNN
	1    2000 2100
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR07
U 1 1 5F5F5A91
P 5700 1400
F 0 "#PWR07" H 5700 1250 50  0001 C CNN
F 1 "+3.3V" H 5700 1540 50  0000 C CNN
F 2 "" H 5700 1400 50  0001 C CNN
F 3 "" H 5700 1400 50  0001 C CNN
	1    5700 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5600 1400 5700 1400
$Comp
L Earth #PWR08
U 1 1 5F5F5B08
P 5700 3300
F 0 "#PWR08" H 5700 3050 50  0001 C CNN
F 1 "Earth" H 5700 3150 50  0001 C CNN
F 2 "" H 5700 3300 50  0001 C CNN
F 3 "" H 5700 3300 50  0001 C CNN
	1    5700 3300
	1    0    0    -1  
$EndComp
Text GLabel 9100 2800 2    60   Input ~ 0
SWDIO
Text GLabel 9100 2900 2    60   Input ~ 0
SWCLK
Text GLabel 9100 2400 2    60   Input ~ 0
MISO
Text GLabel 9100 2500 2    60   Output ~ 0
MOSI
Text GLabel 9100 2300 2    60   Output ~ 0
SCK
NoConn ~ 3100 5400
NoConn ~ 3100 5500
NoConn ~ 2300 2600
NoConn ~ 2300 2700
NoConn ~ 2300 4600
NoConn ~ 2300 4700
NoConn ~ 2300 4800
NoConn ~ 2300 4900
Text GLabel 2300 2900 0    60   Input ~ 0
RX_READY
Text GLabel 9100 2200 2    60   Output ~ 0
RFM_CS
Text GLabel 9100 2100 2    60   Output ~ 0
BME_CS
$Comp
L R R1
U 1 1 5F5F6442
P 9750 2150
F 0 "R1" V 9830 2150 50  0000 C CNN
F 1 "470" V 9750 2150 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 9680 2150 50  0001 C CNN
F 3 "" H 9750 2150 50  0001 C CNN
	1    9750 2150
	-1   0    0    1   
$EndComp
$Comp
L LED D1
U 1 1 5F5F6485
P 9750 2450
F 0 "D1" H 9750 2550 50  0000 C CNN
F 1 "LED" H 9750 2350 50  0000 C CNN
F 2 "LEDs:LED_0805_HandSoldering" H 9750 2450 50  0001 C CNN
F 3 "" H 9750 2450 50  0001 C CNN
	1    9750 2450
	0    -1   -1   0   
$EndComp
$Comp
L Earth #PWR09
U 1 1 5F5F64DC
P 9750 2600
F 0 "#PWR09" H 9750 2350 50  0001 C CNN
F 1 "Earth" H 9750 2450 50  0001 C CNN
F 2 "" H 9750 2600 50  0001 C CNN
F 3 "" H 9750 2600 50  0001 C CNN
	1    9750 2600
	1    0    0    -1  
$EndComp
Text GLabel 9100 1900 2    60   Output ~ 0
TX
Text GLabel 9100 1800 2    60   Input ~ 0
TX_READY
Text GLabel 8300 4250 2    60   Output ~ 0
SWCLK
$Comp
L Earth #PWR010
U 1 1 5F5F66F1
P 8800 4350
F 0 "#PWR010" H 8800 4100 50  0001 C CNN
F 1 "Earth" H 8800 4200 50  0001 C CNN
F 2 "" H 8800 4350 50  0001 C CNN
F 3 "" H 8800 4350 50  0001 C CNN
	1    8800 4350
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x04_Male J2
U 1 1 5F5F67BE
P 8100 4350
F 0 "J2" H 8100 4550 50  0000 C CNN
F 1 "Conn_01x04_Male" H 8100 4050 50  0000 C CNN
F 2 "Connectors_JST:JST_EH_B04B-EH-A_04x2.50mm_Straight" H 8100 4350 50  0001 C CNN
F 3 "" H 8100 4350 50  0001 C CNN
	1    8100 4350
	1    0    0    -1  
$EndComp
Text GLabel 8300 4450 2    60   Output ~ 0
SWDIO
Text GLabel 8300 4550 2    60   Output ~ 0
NRESET
Text GLabel 2300 1800 0    60   Input ~ 0
NRESET
Text GLabel 2300 5150 0    60   Input ~ 0
NRESET
Wire Wire Line
	8800 4350 8300 4350
Wire Wire Line
	9750 2000 9100 2000
Wire Wire Line
	2000 2000 2000 2100
$Comp
L Conn_01x02_Male J1
U 1 1 5F5F74EC
P 5100 4300
F 0 "J1" H 5100 4400 50  0000 C CNN
F 1 "Conn_01x02_Male" H 5100 4100 50  0000 C CNN
F 2 "Battery_Holders:Keystone_1058_1x2032-CoinCell" H 5100 4300 50  0001 C CNN
F 3 "" H 5100 4300 50  0001 C CNN
	1    5100 4300
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR011
U 1 1 5F5F7573
P 5300 4300
F 0 "#PWR011" H 5300 4150 50  0001 C CNN
F 1 "+3.3V" H 5300 4440 50  0000 C CNN
F 2 "" H 5300 4300 50  0001 C CNN
F 3 "" H 5300 4300 50  0001 C CNN
	1    5300 4300
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR012
U 1 1 5F5F7599
P 5300 4400
F 0 "#PWR012" H 5300 4150 50  0001 C CNN
F 1 "Earth" H 5300 4250 50  0001 C CNN
F 2 "" H 5300 4400 50  0001 C CNN
F 3 "" H 5300 4400 50  0001 C CNN
	1    5300 4400
	1    0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 5F5F792E
P 9250 4450
F 0 "C1" H 9275 4550 50  0000 L CNN
F 1 "100nF" H 9275 4350 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 9288 4300 50  0001 C CNN
F 3 "" H 9250 4450 50  0001 C CNN
	1    9250 4450
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 5F5F7A6F
P 9550 4450
F 0 "C2" H 9575 4550 50  0000 L CNN
F 1 "100nF" H 9575 4350 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 9588 4300 50  0001 C CNN
F 3 "" H 9550 4450 50  0001 C CNN
	1    9550 4450
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR013
U 1 1 5F5F7B63
P 9550 4300
F 0 "#PWR013" H 9550 4150 50  0001 C CNN
F 1 "+3.3V" H 9550 4440 50  0000 C CNN
F 2 "" H 9550 4300 50  0001 C CNN
F 3 "" H 9550 4300 50  0001 C CNN
	1    9550 4300
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR014
U 1 1 5F5F7B8D
P 9550 4600
F 0 "#PWR014" H 9550 4350 50  0001 C CNN
F 1 "Earth" H 9550 4450 50  0001 C CNN
F 2 "" H 9550 4600 50  0001 C CNN
F 3 "" H 9550 4600 50  0001 C CNN
	1    9550 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	9250 4600 9550 4600
Wire Wire Line
	9250 4300 9550 4300
$Comp
L Conn_01x03_Male J3
U 1 1 5F5F8CC4
P 6950 4400
F 0 "J3" H 6950 4600 50  0000 C CNN
F 1 "Conn_01x03_Male" H 6950 4200 50  0000 C CNN
F 2 "Connectors_JST:JST_EH_B03B-EH-A_03x2.50mm_Straight" H 6950 4400 50  0001 C CNN
F 3 "" H 6950 4400 50  0001 C CNN
	1    6950 4400
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR015
U 1 1 5F5F8D87
P 7150 4300
F 0 "#PWR015" H 7150 4150 50  0001 C CNN
F 1 "+3.3V" H 7150 4440 50  0000 C CNN
F 2 "" H 7150 4300 50  0001 C CNN
F 3 "" H 7150 4300 50  0001 C CNN
	1    7150 4300
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR016
U 1 1 5F5F8DB3
P 7550 4400
F 0 "#PWR016" H 7550 4150 50  0001 C CNN
F 1 "Earth" H 7550 4250 50  0001 C CNN
F 2 "" H 7550 4400 50  0001 C CNN
F 3 "" H 7550 4400 50  0001 C CNN
	1    7550 4400
	1    0    0    -1  
$EndComp
Text GLabel 7150 4500 2    60   Input ~ 0
TX
$Comp
L PWR_FLAG #FLG017
U 1 1 5F5F93C4
P 6150 4700
F 0 "#FLG017" H 6150 4775 50  0001 C CNN
F 1 "PWR_FLAG" H 6150 4850 50  0000 C CNN
F 2 "" H 6150 4700 50  0001 C CNN
F 3 "3.3" H 6150 4700 50  0001 C CNN
	1    6150 4700
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG018
U 1 1 5F5F93F0
P 6150 4300
F 0 "#FLG018" H 6150 4375 50  0001 C CNN
F 1 "PWR_FLAG" H 6150 4450 50  0000 C CNN
F 2 "" H 6150 4300 50  0001 C CNN
F 3 "0" H 6150 4300 50  0001 C CNN
	1    6150 4300
	-1   0    0    1   
$EndComp
$Comp
L Earth #PWR019
U 1 1 5F5F9AF3
P 6150 4700
F 0 "#PWR019" H 6150 4450 50  0001 C CNN
F 1 "Earth" H 6150 4550 50  0001 C CNN
F 2 "" H 6150 4700 50  0001 C CNN
F 3 "" H 6150 4700 50  0001 C CNN
	1    6150 4700
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR020
U 1 1 5F5F9B5B
P 6150 4300
F 0 "#PWR020" H 6150 4150 50  0001 C CNN
F 1 "+3.3V" H 6150 4440 50  0000 C CNN
F 2 "" H 6150 4300 50  0001 C CNN
F 3 "" H 6150 4300 50  0001 C CNN
	1    6150 4300
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x06_Male J4
U 1 1 5F5FA3E2
P 3900 5300
F 0 "J4" H 3900 5600 50  0000 C CNN
F 1 "Conn_01x06_Male" H 3900 4900 50  0000 C CNN
F 2 "Connectors_JST:JST_EH_B06B-EH-A_06x2.50mm_Straight" H 3900 5300 50  0001 C CNN
F 3 "" H 3900 5300 50  0001 C CNN
	1    3900 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 5250 4600 5200
Wire Wire Line
	4600 5200 4100 5200
$Comp
L Conn_01x02_Male J5
U 1 1 5F5FB796
P 3300 4600
F 0 "J5" H 3300 4700 50  0000 C CNN
F 1 "Conn_01x02_Male" H 3300 4500 50  0000 R CNN
F 2 "Connectors_JST:JST_EH_B02B-EH-A_02x2.50mm_Straight" H 3300 4600 50  0001 C CNN
F 3 "" H 3300 4600 50  0001 C CNN
	1    3300 4600
	-1   0    0    1   
$EndComp
Wire Wire Line
	3100 4500 3100 4300
Wire Wire Line
	3100 4300 3300 4300
$Comp
L +3.3V #PWR021
U 1 1 5F5FFB3B
P 5700 5300
F 0 "#PWR021" H 5700 5150 50  0001 C CNN
F 1 "+3.3V" H 5700 5440 50  0000 C CNN
F 2 "" H 5700 5300 50  0001 C CNN
F 3 "" H 5700 5300 50  0001 C CNN
	1    5700 5300
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR022
U 1 1 5F5FFB87
P 5700 6250
F 0 "#PWR022" H 5700 6000 50  0001 C CNN
F 1 "Earth" H 5700 6100 50  0001 C CNN
F 2 "" H 5700 6250 50  0001 C CNN
F 3 "" H 5700 6250 50  0001 C CNN
	1    5700 6250
	1    0    0    -1  
$EndComp
$Comp
L R R4
U 1 1 5F5FFCDB
P 5700 5450
F 0 "R4" V 5780 5450 50  0000 C CNN
F 1 "10k" V 5700 5450 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 5630 5450 50  0001 C CNN
F 3 "" H 5700 5450 50  0001 C CNN
	1    5700 5450
	-1   0    0    1   
$EndComp
$Comp
L R R3
U 1 1 5F5FFF18
P 6050 5450
F 0 "R3" V 6130 5450 50  0000 C CNN
F 1 "10k" V 6050 5450 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 5980 5450 50  0001 C CNN
F 3 "" H 6050 5450 50  0001 C CNN
	1    6050 5450
	-1   0    0    1   
$EndComp
Text GLabel 5700 5650 2    60   Input ~ 0
SCL
Text GLabel 5700 5950 2    60   Input ~ 0
SDA
Text GLabel 9100 2600 2    60   Output ~ 0
SCL
Text GLabel 9100 2700 2    60   Output ~ 0
SDA
$Comp
L TEST TP4
U 1 1 5F600B26
P 5700 6250
F 0 "TP4" H 5700 6550 50  0000 C BNN
F 1 "TEST" H 5700 6500 50  0000 C CNN
F 2 "Connectors:PINTST" H 5700 6250 50  0001 C CNN
F 3 "" H 5700 6250 50  0001 C CNN
	1    5700 6250
	0    -1   -1   0   
$EndComp
$Comp
L TEST TP3
U 1 1 5F600B88
P 5700 5950
F 0 "TP3" H 5700 6250 50  0000 C BNN
F 1 "TEST" H 5700 6200 50  0000 C CNN
F 2 "Connectors:PINTST" H 5700 5950 50  0001 C CNN
F 3 "" H 5700 5950 50  0001 C CNN
	1    5700 5950
	0    -1   -1   0   
$EndComp
$Comp
L TEST TP2
U 1 1 5F600C7C
P 5700 5650
F 0 "TP2" H 5700 5950 50  0000 C BNN
F 1 "TEST" H 5700 5900 50  0000 C CNN
F 2 "Connectors:PINTST" H 5700 5650 50  0001 C CNN
F 3 "" H 5700 5650 50  0001 C CNN
	1    5700 5650
	0    -1   -1   0   
$EndComp
$Comp
L TEST TP1
U 1 1 5F600F94
P 5700 5300
F 0 "TP1" H 5700 5600 50  0000 C BNN
F 1 "TEST" H 5700 5550 50  0000 C CNN
F 2 "Connectors:PINTST" H 5700 5300 50  0001 C CNN
F 3 "" H 5700 5300 50  0001 C CNN
	1    5700 5300
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5700 5800 5700 5950
Wire Wire Line
	6050 5300 5700 5300
Wire Wire Line
	6050 5800 5700 5800
Wire Wire Line
	6050 5600 6050 5800
Wire Wire Line
	5700 5600 5700 5650
Wire Wire Line
	7550 4400 7150 4400
$EndSCHEMATC
