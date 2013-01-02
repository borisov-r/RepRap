EESchema Schematic File Version 2  date  3.01.2013 (чт)  1,24,30 EET
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
EELAYER 27 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "7 SEGMENT LED DISPLAY FOR PIPO"
Date "2 jan 2013"
Rev "rev.A"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L 7SEGM S?
U 1 1 50E4B02C
P 7950 3450
F 0 "S?" H 7950 4100 60  0000 C CNN
F 1 "7SEGM" H 7950 2800 60  0000 C CNN
	1    7950 3450
	1    0    0    -1  
$EndComp
$Comp
L 4094 U?
U 1 1 50E4C062
P 4900 3700
F 0 "U?" H 4700 4350 60  0000 C CNN
F 1 "4094" H 4900 3200 60  0000 C CNN
	1    4900 3700
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 50E4C08F
P 5900 3150
F 0 "R?" V 5980 3150 50  0000 C CNN
F 1 "R" V 5900 3150 50  0000 C CNN
	1    5900 3150
	0    -1   -1   0   
$EndComp
$Comp
L R R?
U 1 1 50E4C0AB
P 5900 3250
F 0 "R?" V 5980 3250 50  0000 C CNN
F 1 "R" V 5900 3250 50  0000 C CNN
	1    5900 3250
	0    -1   -1   0   
$EndComp
$Comp
L R R?
U 1 1 50E4C0B1
P 5900 3350
F 0 "R?" V 5980 3350 50  0000 C CNN
F 1 "R" V 5900 3350 50  0000 C CNN
	1    5900 3350
	0    -1   -1   0   
$EndComp
$Comp
L R R?
U 1 1 50E4C0B7
P 5900 3450
F 0 "R?" V 5980 3450 50  0000 C CNN
F 1 "R" V 5900 3450 50  0000 C CNN
	1    5900 3450
	0    -1   -1   0   
$EndComp
$Comp
L R R?
U 1 1 50E4C0BD
P 5900 3550
F 0 "R?" V 5980 3550 50  0000 C CNN
F 1 "R" V 5900 3550 50  0000 C CNN
	1    5900 3550
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5650 3150 5350 3150
Wire Wire Line
	5350 3250 5650 3250
Wire Wire Line
	5350 3350 5650 3350
Wire Wire Line
	5350 3450 5650 3450
Wire Wire Line
	5350 3550 5650 3550
$Comp
L R R?
U 1 1 50E4C0DC
P 5900 3650
F 0 "R?" V 5980 3650 50  0000 C CNN
F 1 "R" V 5900 3650 50  0000 C CNN
	1    5900 3650
	0    -1   -1   0   
$EndComp
$Comp
L R R?
U 1 1 50E4C0E2
P 5900 3750
F 0 "R?" V 5980 3750 50  0000 C CNN
F 1 "R" V 5900 3750 50  0000 C CNN
	1    5900 3750
	0    -1   -1   0   
$EndComp
$Comp
L R R?
U 1 1 50E4C0E8
P 5900 3850
F 0 "R?" V 5980 3850 50  0000 C CNN
F 1 "R" V 5900 3850 50  0000 C CNN
	1    5900 3850
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5350 3650 5650 3650
Wire Wire Line
	5350 3750 5650 3750
Wire Wire Line
	5350 3850 5650 3850
Wire Wire Line
	6150 3150 6400 3150
Wire Wire Line
	6150 3250 6400 3250
Wire Wire Line
	6150 3350 6400 3350
Wire Wire Line
	6150 3450 6400 3450
Wire Wire Line
	6150 3550 6400 3550
Wire Wire Line
	6150 3650 6400 3650
Wire Wire Line
	6150 3750 6400 3750
Wire Wire Line
	6150 3850 6400 3850
Wire Wire Line
	7200 3150 6950 3150
Wire Wire Line
	7200 3250 6950 3250
Wire Wire Line
	7200 3350 6950 3350
Wire Wire Line
	7200 3450 6950 3450
Wire Wire Line
	7200 3550 6950 3550
Wire Wire Line
	7200 3650 6950 3650
Wire Wire Line
	7200 3750 6950 3750
Wire Wire Line
	7200 3850 6950 3850
Wire Wire Line
	7150 3950 7200 3950
Wire Wire Line
	7150 2900 7150 3950
Wire Wire Line
	7150 3050 7200 3050
Connection ~ 7150 3050
Text Label 7150 2900 1    60   ~ 0
VDD
Text Label 6400 3150 0    60   ~ 0
Q1
Text Label 6400 3250 0    60   ~ 0
Q2
Text Label 6400 3350 0    60   ~ 0
Q3
Text Label 6400 3450 0    60   ~ 0
Q4
Text Label 6400 3550 0    60   ~ 0
Q5
Text Label 6400 3650 0    60   ~ 0
Q6
Text Label 6400 3750 0    60   ~ 0
Q7
Text Label 6400 3850 0    60   ~ 0
Q8
Text Label 6950 3550 2    60   ~ 0
Q1
Text Label 6950 3450 2    60   ~ 0
Q2
Text Label 6950 3350 2    60   ~ 0
Q3
Text Label 6950 3850 2    60   ~ 0
Q4
Text Label 6950 3250 2    60   ~ 0
Q8
Text Label 6950 3150 2    60   ~ 0
Q7
Text Label 6950 3650 2    60   ~ 0
Q6
Text Label 6950 3750 2    60   ~ 0
Q5
$Comp
L CONN_5 P?
U 1 1 50E4C2BB
P 3250 3350
F 0 "P?" V 3200 3350 50  0000 C CNN
F 1 "CONN_5" V 3300 3350 50  0000 C CNN
	1    3250 3350
	-1   0    0    -1  
$EndComp
Wire Wire Line
	3650 3150 4450 3150
Wire Wire Line
	3650 3250 4150 3250
Wire Wire Line
	4150 3250 4150 4100
Wire Wire Line
	4150 4100 4450 4100
Wire Wire Line
	3650 3350 4450 3350
Wire Wire Line
	3650 3450 4450 3450
Wire Wire Line
	3650 3550 4450 3550
Wire Wire Line
	4450 3650 4350 3650
Wire Wire Line
	4350 3650 4350 3150
Connection ~ 4350 3150
Wire Wire Line
	3800 3000 3800 4150
Connection ~ 3800 3150
Text Label 3800 3000 1    60   ~ 0
VDD
Text Label 3850 4700 3    60   ~ 0
VSS
$Comp
L CONN_5 P?
U 1 1 50E4C3F1
P 3250 4350
F 0 "P?" V 3200 4350 50  0000 C CNN
F 1 "CONN_5" V 3300 4350 50  0000 C CNN
	1    3250 4350
	-1   0    0    -1  
$EndComp
Text Notes 3100 3150 3    100  ~ 0
INPUT
Text Notes 3100 4100 3    100  ~ 0
OUTPUT
Wire Wire Line
	3850 3250 3850 4700
Wire Wire Line
	3850 4250 3650 4250
Connection ~ 3850 3250
Connection ~ 3850 4250
Wire Wire Line
	3800 4150 3650 4150
Wire Wire Line
	3900 3350 3900 4350
Wire Wire Line
	3900 4350 3650 4350
Connection ~ 3900 3350
Wire Wire Line
	3650 4450 5450 4450
Wire Wire Line
	5450 4450 5450 4000
Wire Wire Line
	5450 4000 5350 4000
Wire Wire Line
	3950 3550 3950 4550
Wire Wire Line
	3950 4550 3650 4550
Connection ~ 3950 3550
$EndSCHEMATC
