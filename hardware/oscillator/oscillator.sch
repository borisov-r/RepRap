EESchema Schematic File Version 2  date Tue 19 Jun 2012 05:25:46 PM EEST
LIBS:power
LIBS:schematicLib
LIBS:oscillator-cache
EELAYER 25  0
EELAYER END
$Descr A4 11700 8267
encoding utf-8
Sheet 1 1
Title "Oscillator with Timer 555"
Date "19 jun 2012"
Rev "A"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text Notes 750  2700 0    118  ~ 0
timer555
Wire Notes Line
	750  750  3750 750 
Wire Notes Line
	750  750  750  2500
Wire Notes Line
	750  2500 3750 2500
Wire Notes Line
	3750 2500 3750 750 
Connection ~ 3000 1300
Wire Wire Line
	3000 1400 3000 1300
Wire Wire Line
	1000 2050 1000 2300
Wire Wire Line
	2350 2200 2350 2300
Wire Wire Line
	2350 1950 1100 1950
Wire Wire Line
	2400 1650 2350 1650
Connection ~ 1650 1100
Wire Wire Line
	1600 1100 1650 1100
Wire Wire Line
	1650 1400 1600 1400
Wire Wire Line
	1600 950  1650 950 
Connection ~ 1000 1250
Wire Wire Line
	1100 1250 1000 1250
Connection ~ 1000 1400
Wire Wire Line
	900  1400 1400 1400
Wire Wire Line
	1000 1400 1000 950 
Wire Wire Line
	1000 950  1100 950 
Wire Wire Line
	1300 1100 1400 1100
Wire Wire Line
	2050 1650 2050 1700
Connection ~ 2050 1700
Connection ~ 2300 1550
Wire Wire Line
	2300 1550 2400 1550
Connection ~ 2050 1400
Wire Wire Line
	2050 1200 2050 1450
Wire Wire Line
	2150 1400 2050 1400
Wire Wire Line
	2400 1500 2300 1500
Wire Wire Line
	2350 1200 2350 1300
Wire Wire Line
	2350 1300 2400 1300
Wire Wire Line
	2650 950  2650 1200
Wire Wire Line
	2350 1000 2350 950 
Wire Wire Line
	2650 1750 2650 1850
Wire Wire Line
	2400 1400 2350 1400
Wire Wire Line
	2050 1000 2050 950 
Connection ~ 2350 950 
Wire Wire Line
	2050 950  2700 950 
Connection ~ 2650 950 
Wire Wire Line
	2900 950  3000 950 
Wire Wire Line
	2300 1500 2300 1700
Wire Wire Line
	1300 950  1400 950 
Wire Wire Line
	1100 1100 1000 1100
Connection ~ 1000 1100
Wire Wire Line
	1400 1250 1300 1250
Wire Wire Line
	1600 1250 1650 1250
Connection ~ 1650 1250
Connection ~ 1650 1400
Wire Wire Line
	1650 950  1650 1700
Wire Wire Line
	1650 1700 2300 1700
Wire Wire Line
	2350 1650 2350 2000
Connection ~ 2350 1950
Wire Wire Line
	1000 1850 1000 1750
Wire Wire Line
	2900 1300 3450 1300
Wire Wire Line
	3000 1600 3000 1850
Text Label 3450 1300 0    39   ~ 0
output
Text Label 3000 1850 3    39   ~ 0
gnd
$Comp
L CAP_0603 C?
U 1 1 4FE08B78
P 3000 1500
F 0 "C?" H 3000 1575 60  0000 C CNN
F 1 "CAP_0603" H 3000 1600 60  0001 C CNN
	1    3000 1500
	0    1    1    0   
$EndComp
Text Label 1000 1750 1    39   ~ 0
vdd5
Text Label 1000 2300 3    39   ~ 0
gnd
Text Label 2350 2300 3    39   ~ 0
gnd
$Comp
L CAP_0603 C?
U 1 1 4FE08A9E
P 2350 2100
F 0 "C?" H 2350 2175 60  0000 C CNN
F 1 "CAP_0603" H 2350 2200 60  0001 C CNN
	1    2350 2100
	0    1    1    0   
$EndComp
$Comp
L POT R?
U 1 1 4FE08A8B
P 1000 1950
F 0 "R?" H 995 1855 60  0000 C CNN
F 1 "POT" H 995 2085 60  0001 C CNN
	1    1000 1950
	0    1    1    0   
$EndComp
NoConn ~ 1950 1550
$Comp
L POT R?
U 1 1 4FE08A83
P 2050 1550
F 0 "R?" H 2045 1455 60  0000 C CNN
F 1 "POT" H 2045 1685 60  0001 C CNN
	1    2050 1550
	0    -1   -1   0   
$EndComp
NoConn ~ 1950 1100
$Comp
L POT R?
U 1 1 4FE08A61
P 2050 1100
F 0 "R?" H 2045 1005 60  0000 C CNN
F 1 "POT" H 2045 1235 60  0001 C CNN
	1    2050 1100
	0    -1   -1   0   
$EndComp
$Comp
L J2PIN J?
U 1 1 4FE087EE
P 1500 1250
F 0 "J?" H 1650 1200 40  0000 C CNN
F 1 "J2PIN" H 1250 1250 40  0000 C CNN
	1    1500 1250
	-1   0    0    1   
$EndComp
$Comp
L CAP_0603 C?
U 1 1 4FE087EB
P 1200 1250
F 0 "C?" H 1200 1200 39  0000 C CNN
F 1 "100 nF" H 1200 1350 39  0001 C CNN
	1    1200 1250
	-1   0    0    1   
$EndComp
Text Label 900  1400 2    39   ~ 0
gnd
$Comp
L CAP_0603 C?
U 1 1 4FE08755
P 1200 1100
F 0 "C?" H 1200 1050 39  0000 C CNN
F 1 "10 nF" H 1200 1200 39  0001 C CNN
	1    1200 1100
	-1   0    0    1   
$EndComp
$Comp
L CAP_0603 C?
U 1 1 4FE0874A
P 1200 950
F 0 "C?" H 1200 900 39  0000 C CNN
F 1 "1 nF" H 1200 1050 39  0001 C CNN
	1    1200 950 
	-1   0    0    1   
$EndComp
$Comp
L J2PIN J?
U 1 1 4FE08732
P 1500 1400
F 0 "J?" H 1650 1350 40  0000 C CNN
F 1 "J2PIN" H 1250 1400 40  0000 C CNN
	1    1500 1400
	-1   0    0    1   
$EndComp
$Comp
L J2PIN J?
U 1 1 4FE0872F
P 1500 1100
F 0 "J?" H 1650 1050 40  0000 C CNN
F 1 "J2PIN" H 1250 1100 40  0000 C CNN
	1    1500 1100
	-1   0    0    1   
$EndComp
$Comp
L J2PIN J?
U 1 1 4FE08715
P 1500 950
F 0 "J?" H 1650 900 40  0000 C CNN
F 1 "J2PIN" H 1250 950 40  0000 C CNN
	1    1500 950 
	-1   0    0    1   
$EndComp
Text Label 3000 950  0    39   ~ 0
gnd
$Comp
L CAP_0603 C?
U 1 1 4FE0868B
P 2800 950
F 0 "C?" H 2800 1025 60  0000 C CNN
F 1 "CAP_0603" H 2800 1050 60  0001 C CNN
	1    2800 950 
	1    0    0    -1  
$EndComp
$Comp
L RES_0603 R?
U 1 1 4FE08622
P 2250 1400
F 0 "R?" H 2250 1475 60  0000 C CNN
F 1 "500" H 2250 1500 31  0001 C CNN
	1    2250 1400
	1    0    0    -1  
$EndComp
Text Label 2650 1850 3    39   ~ 0
gnd
$Comp
L NE555 U?
U 1 1 4FE0847D
P 2650 1500
F 0 "U?" H 2550 1800 31  0000 C CNN
F 1 "NE555" H 2750 1250 31  0000 C CNN
F 2 "DIP-8_300" H 2650 1500 60  0001 C CNN
F 3 "http://store.comet.bg/bg/Catalogue/Product/588/" H 2650 1500 60  0001 C CNN
	1    2650 1500
	1    0    0    -1  
$EndComp
$Comp
L RES_0603 R?
U 1 1 4FE08100
P 2350 1100
F 0 "R?" H 2350 1175 60  0000 C CNN
F 1 "10K" H 2350 1200 31  0001 C CNN
	1    2350 1100
	0    -1   -1   0   
$EndComp
$EndSCHEMATC
