EESchema Schematic File Version 2  date Fri 06 Jul 2012 05:40:57 PM EEST
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:special
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
LIBS:ad8551
LIBS:thermometer-cache
EELAYER 25  0
EELAYER END
$Descr A4 11700 8267
encoding utf-8
Sheet 1 1
Title ""
Date "6 jul 2012"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Wire Wire Line
	3900 4000 4650 4000
Wire Wire Line
	3900 5500 3900 4000
Wire Wire Line
	6850 5950 6850 5800
Wire Wire Line
	6850 5800 6950 5800
Connection ~ 2450 1850
Wire Wire Line
	4150 1850 1950 1850
Wire Notes Line
	1450 3150 1450 900 
Wire Notes Line
	1450 3150 5850 3150
Wire Notes Line
	5850 3150 5850 900 
Wire Notes Line
	5850 900  1450 900 
Wire Wire Line
	4050 1750 4150 1750
Connection ~ 3200 1500
Wire Wire Line
	3100 1500 3200 1500
Wire Wire Line
	5250 2350 5250 2900
Connection ~ 5500 1750
Wire Wire Line
	5950 1750 4850 1750
Wire Wire Line
	5250 1950 5250 1750
Wire Wire Line
	2950 1150 2800 1150
Connection ~ 3650 1850
Wire Wire Line
	3300 2350 3300 2900
Wire Wire Line
	3300 1850 3300 1950
Wire Wire Line
	2150 1500 2050 1500
Wire Wire Line
	4900 4850 5050 4850
Wire Wire Line
	4900 5150 5050 5150
Wire Wire Line
	4400 5350 4400 4600
Wire Wire Line
	4400 5850 4400 6100
Wire Wire Line
	4050 5700 3400 5700
Wire Wire Line
	3400 5700 3400 6100
Wire Wire Line
	3400 6100 3200 6100
Wire Notes Line
	2050 4600 1450 4600
Wire Notes Line
	2050 4600 2050 5900
Wire Notes Line
	2050 5900 1450 5900
Wire Notes Line
	1450 5900 1450 4600
Connection ~ 2600 5700
Wire Wire Line
	2000 5700 2600 5700
Connection ~ 2600 4000
Wire Wire Line
	2600 3850 2600 4300
Wire Wire Line
	1550 4850 1200 4850
Wire Wire Line
	2600 4800 2600 4900
Wire Wire Line
	3050 6350 3050 6550
Wire Wire Line
	2250 6350 2250 6550
Wire Wire Line
	2600 6350 2600 6550
Wire Wire Line
	2600 5400 2600 5850
Wire Wire Line
	1950 4850 2600 4850
Connection ~ 2600 4850
Wire Wire Line
	2600 4000 3050 4000
Wire Wire Line
	3050 4000 3050 4300
Wire Wire Line
	2000 5500 3200 5500
Wire Wire Line
	2250 5850 2250 5500
Connection ~ 2250 5500
Wire Wire Line
	3050 4800 3050 5850
Wire Wire Line
	4050 5500 3700 5500
Wire Wire Line
	4400 5150 4500 5150
Connection ~ 4400 5150
Wire Wire Line
	4500 4850 4400 4850
Connection ~ 4400 4850
Connection ~ 3900 5500
Connection ~ 5400 5600
Wire Wire Line
	1950 2050 2100 2050
Wire Wire Line
	2100 2050 2100 2900
Wire Wire Line
	2450 2300 2100 2300
Connection ~ 2100 2300
Wire Wire Line
	2550 1500 2600 1500
Wire Wire Line
	3650 1850 3650 1950
Connection ~ 3300 1850
Wire Wire Line
	3650 2350 3650 2900
Wire Wire Line
	4850 1950 5000 1950
Wire Wire Line
	5000 1950 5000 2100
Wire Wire Line
	3200 1250 3200 1750
Wire Wire Line
	5500 1750 5500 1950
Connection ~ 5250 1750
Wire Wire Line
	5500 2350 5500 2900
Wire Wire Line
	3200 1750 3550 1750
Wire Wire Line
	3450 1850 3450 1150
Connection ~ 3450 1850
Wire Wire Line
	2450 1850 2450 1900
Wire Wire Line
	6950 5600 4850 5600
Wire Wire Line
	5400 5600 5400 3900
Wire Wire Line
	5400 3900 4300 3900
Wire Wire Line
	4300 3900 4300 4150
Wire Wire Line
	4300 4150 4400 4150
Text Label 6850 5950 3    60   ~ 0
avss
$Comp
L CONN_2 P3
U 1 1 4FEDC132
P 7300 5700
F 0 "P3" V 7250 5700 40  0000 C CNN
F 1 "CONN_2" V 7350 5700 40  0000 C CNN
F 2 "TS-501" H 7300 5700 60  0001 C CNN
	1    7300 5700
	1    0    0    -1  
$EndComp
$Comp
L C C3
U 1 1 4FEDAF6F
P 4700 4850
F 0 "C3" H 4750 4950 50  0000 L CNN
F 1 "10u" H 4750 4750 50  0000 L CNN
F 2 "SM0805" H 4700 4850 60  0001 C CNN
	1    4700 4850
	0    -1   -1   0   
$EndComp
Text Label 5750 5600 0    60   ~ 0
TEMPERATURE
$Comp
L BA033CC0WFP U1
U 1 1 4FEC4AA0
P 4500 1850
F 0 "U1" H 4450 1650 60  0000 C CNN
F 1 "BA033CC0WFP" H 4500 2050 60  0000 C CNN
F 2 "BA033CCOWFP" H 4500 1850 60  0001 C CNN
	1    4500 1850
	1    0    0    -1  
$EndComp
Text Label 5950 1750 0    60   ~ 0
avdd5
Text Label 5500 2900 3    60   ~ 0
avss
Text Label 5250 2900 3    60   ~ 0
avss
$Comp
L C C6
U 1 1 4FEC47BD
P 5500 2150
F 0 "C6" H 5550 2250 50  0000 L CNN
F 1 "100n" H 5550 2050 50  0000 L CNN
F 2 "SM0805" H 5500 2150 60  0001 C CNN
	1    5500 2150
	1    0    0    -1  
$EndComp
$Comp
L C C5
U 1 1 4FEC47BA
P 5250 2150
F 0 "C5" H 5300 2250 50  0000 L CNN
F 1 "10u" H 5300 2050 50  0000 L CNN
F 2 "SM0805" H 5250 2150 60  0001 C CNN
	1    5250 2150
	1    0    0    -1  
$EndComp
Text Label 2800 1150 2    60   ~ 0
avss
$Comp
L JUMPER3 JP1
U 1 1 4FEC4735
P 3200 1150
F 0 "JP1" H 3250 1050 40  0000 L CNN
F 1 "JUMPER3" H 3200 1250 40  0000 C CNN
F 2 "JUMPER3" H 3200 1150 60  0001 C CNN
	1    3200 1150
	1    0    0    -1  
$EndComp
$Comp
L R R8
U 1 1 4FEC46F7
P 3800 1750
F 0 "R8" V 3880 1750 50  0000 C CNN
F 1 "10K" V 3800 1750 50  0000 C CNN
F 2 "SM0805" H 3800 1750 60  0001 C CNN
	1    3800 1750
	0    -1   -1   0   
$EndComp
Text Label 5000 2100 3    60   ~ 0
avss
NoConn ~ 4850 1850
NoConn ~ 4150 1950
Text Label 3650 2900 3    60   ~ 0
avss
Text Label 3300 2900 3    60   ~ 0
avss
$Comp
L CP1 C1
U 1 1 4FEC4646
P 3300 2150
F 0 "C1" H 3350 2250 50  0000 L CNN
F 1 "CP1" H 3350 2050 50  0000 L CNN
F 2 "CAP_POL_10u" H 3300 2150 60  0001 C CNN
	1    3300 2150
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 4FEC463A
P 3650 2150
F 0 "C2" H 3700 2250 50  0000 L CNN
F 1 "100n" H 3700 2050 50  0000 L CNN
F 2 "SM0805" H 3650 2150 60  0001 C CNN
	1    3650 2150
	1    0    0    -1  
$EndComp
Text Label 2050 1500 2    60   ~ 0
avss
$Comp
L LED D2
U 1 1 4FEC4613
P 2350 1500
F 0 "D2" H 2350 1600 50  0000 C CNN
F 1 "RED" H 2350 1400 50  0000 C CNN
F 2 "KP-2012ZGC" H 2350 1500 60  0001 C CNN
	1    2350 1500
	-1   0    0    1   
$EndComp
$Comp
L R R5
U 1 1 4FEC4601
P 2850 1500
F 0 "R5" V 2930 1500 50  0000 C CNN
F 1 "5K" V 2850 1500 50  0000 C CNN
F 2 "SM0805" H 2850 1500 60  0001 C CNN
	1    2850 1500
	0    1    1    0   
$EndComp
$Comp
L DIODE D3
U 1 1 4FEC45D8
P 2450 2100
F 0 "D3" H 2450 2200 40  0000 C CNN
F 1 "1N4007" H 2450 2000 40  0000 C CNN
F 2 "melf" H 2450 2100 60  0001 C CNN
	1    2450 2100
	0    -1   -1   0   
$EndComp
Text Label 2100 2900 3    60   ~ 0
avss
$Comp
L CONN_2 P1
U 1 1 4FEC457A
P 1600 1950
F 0 "P1" V 1550 1950 40  0000 C CNN
F 1 "CONN_2" V 1650 1950 40  0000 C CNN
F 2 "TS-501" H 1600 1950 60  0001 C CNN
	1    1600 1950
	-1   0    0    -1  
$EndComp
NoConn ~ 4900 4150
Text Notes 3850 3750 0    39   ~ 0
Measure temperature - up to 500 deg - RV = 124 KOhms\n\nMeasure temperature - up to 250 deg - RV = 240 KOhms
$Comp
L POT RV2
U 1 1 4FEC44A1
P 4650 4150
F 0 "RV2" H 4650 4050 50  0000 C CNN
F 1 "240K" H 4650 4150 50  0000 C CNN
F 2 "BOURNS_TRIMMER" H 4650 4150 60  0001 C CNN
	1    4650 4150
	1    0    0    -1  
$EndComp
Text Label 5050 4850 0    60   ~ 0
avss
Text Label 5050 5150 0    60   ~ 0
avss
$Comp
L C C4
U 1 1 4FEC41FF
P 4700 5150
F 0 "C4" H 4750 5250 50  0000 L CNN
F 1 "100n" H 4750 5050 50  0000 L CNN
F 2 "SM0805" H 4700 5150 60  0001 C CNN
	1    4700 5150
	0    -1   -1   0   
$EndComp
Text Label 4400 4600 1    60   ~ 0
avdd5
NoConn ~ 4650 5850
NoConn ~ 4600 5850
NoConn ~ 4550 5850
Text Label 4400 6100 3    60   ~ 0
avss
$Comp
L AD8551 U?1
U 1 1 4FEC41B2
P 4300 5600
F 0 "U?1" H 4150 6000 39  0000 C CNN
F 1 "AD8551" H 4200 5900 39  0000 C CNN
F 2 "MSOP_8" H 4300 5600 60  0001 C CNN
	1    4300 5600
	1    0    0    -1  
$EndComp
$Comp
L CONN_2 P2
U 1 1 4FEC3CB3
P 1650 5600
F 0 "P2" V 1600 5600 40  0000 C CNN
F 1 "CONN_2" V 1700 5600 40  0000 C CNN
F 2 "2EDGR" H 1650 5600 60  0001 C CNN
	1    1650 5600
	-1   0    0    -1  
$EndComp
Text Label 2600 3850 1    60   ~ 0
avdd5
Text Label 1200 4850 2    60   ~ 0
avss
$Comp
L DIODE D1
U 1 1 4FEC3C4C
P 1750 4850
F 0 "D1" H 1750 4950 40  0000 C CNN
F 1 "1N4148" H 1750 4750 40  0000 C CNN
F 2 "melf" H 1750 4850 60  0001 C CNN
	1    1750 4850
	-1   0    0    1   
$EndComp
Text Label 3050 6550 3    60   ~ 0
avss
Text Label 2600 6550 3    60   ~ 0
avss
Text Label 2250 6550 3    60   ~ 0
avss
$Comp
L R R1
U 1 1 4FEC3BE4
P 2250 6100
F 0 "R1" V 2330 6100 50  0000 C CNN
F 1 "5.62K" V 2250 6100 50  0000 C CNN
F 2 "SM0805" H 2250 6100 60  0001 C CNN
	1    2250 6100
	1    0    0    -1  
$EndComp
$Comp
L R R4
U 1 1 4FEC3BCA
P 2600 6100
F 0 "R4" V 2680 6100 50  0000 C CNN
F 1 "53.6K" V 2600 6100 50  0000 C CNN
F 2 "SM0805" H 2600 6100 60  0001 C CNN
	1    2600 6100
	1    0    0    -1  
$EndComp
$Comp
L POT RV1
U 1 1 4FEC3BA7
P 3050 6100
F 0 "RV1" H 3050 6000 50  0000 C CNN
F 1 "200" H 3050 6100 50  0000 C CNN
F 2 "BOURNS_TRIMMER" H 3050 6100 60  0001 C CNN
	1    3050 6100
	0    1    1    0   
$EndComp
$Comp
L R R7
U 1 1 4FEC3B63
P 3450 5500
F 0 "R7" V 3530 5500 50  0000 C CNN
F 1 "453K" V 3450 5500 50  0000 C CNN
F 2 "SM0805" H 3450 5500 60  0001 C CNN
	1    3450 5500
	0    -1   -1   0   
$EndComp
$Comp
L R R3
U 1 1 4FEC3B48
P 2600 5150
F 0 "R3" V 2680 5150 50  0000 C CNN
F 1 "2.74K" V 2600 5150 50  0000 C CNN
F 2 "SM0805" H 2600 5150 60  0001 C CNN
	1    2600 5150
	1    0    0    -1  
$EndComp
$Comp
L R R6
U 1 1 4FEC3B2B
P 3050 4550
F 0 "R6" V 3130 4550 50  0000 C CNN
F 1 "40.2K" V 3050 4550 50  0000 C CNN
F 2 "SM0805" H 3050 4550 60  0001 C CNN
	1    3050 4550
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 4FEC3B03
P 2600 4550
F 0 "R2" V 2680 4550 50  0000 C CNN
F 1 "10.7K" V 2600 4550 50  0000 C CNN
F 2 "SM0805" H 2600 4550 60  0001 C CNN
	1    2600 4550
	1    0    0    -1  
$EndComp
$EndSCHEMATC
