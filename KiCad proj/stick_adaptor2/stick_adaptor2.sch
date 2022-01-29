EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Connector:Conn_01x05_Female J2
U 1 1 61BF7536
P 5650 2200
F 0 "J2" H 5678 2226 50  0000 L CNN
F 1 "Conn_01x05_Female" H 5678 2135 50  0000 L CNN
F 2 "Connector_PinHeader_1.27mm:PinHeader_1x05_P1.27mm_Vertical" H 5650 2200 50  0001 C CNN
F 3 "~" H 5650 2200 50  0001 C CNN
	1    5650 2200
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x09_Female J1
U 1 1 61BFAF20
P 3200 2150
F 0 "J1" H 3092 2735 50  0000 C CNN
F 1 "Conn_01x09_Female" H 3092 2644 50  0000 C CNN
F 2 "stick:Stick_china" H 3200 2150 50  0001 C CNN
F 3 "~" H 3200 2150 50  0001 C CNN
	1    3200 2150
	-1   0    0    -1  
$EndComp
Text GLabel 5050 2000 0    50   Input ~ 0
3.3
Text GLabel 5050 2100 0    50   Input ~ 0
Y
Text GLabel 5050 2200 0    50   Input ~ 0
BTN
Text GLabel 5050 2300 0    50   Input ~ 0
GND
Text GLabel 5050 2400 0    50   Input ~ 0
X
Wire Wire Line
	4400 1750 3400 1750
Wire Wire Line
	4400 2300 5450 2300
$Comp
L Device:R R1
U 1 1 61C30BE8
P 4700 1750
F 0 "R1" H 4770 1796 50  0000 L CNN
F 1 "10k" H 4770 1705 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 4630 1750 50  0001 C CNN
F 3 "~" H 4700 1750 50  0001 C CNN
	1    4700 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	5450 2000 4700 2000
Wire Wire Line
	4700 2000 4700 1900
Wire Wire Line
	4700 1600 4700 1550
Wire Wire Line
	4700 1550 5200 1550
Wire Wire Line
	5200 1550 5200 2200
Wire Wire Line
	5200 2200 5450 2200
Wire Wire Line
	5200 2200 4300 2200
Connection ~ 5200 2200
Wire Wire Line
	3400 2550 4400 2550
Wire Wire Line
	4400 2550 4400 2350
Connection ~ 4400 2300
Wire Wire Line
	3400 1950 3900 1950
Wire Wire Line
	3900 1950 3900 2400
Wire Wire Line
	3900 2400 5450 2400
Wire Wire Line
	3400 2250 3500 2250
Wire Wire Line
	3500 2250 3500 2100
Wire Wire Line
	3500 2100 5450 2100
Wire Wire Line
	4300 2450 4300 2200
Wire Wire Line
	3400 2450 4300 2450
Connection ~ 4700 2000
Wire Wire Line
	3400 2150 4700 2150
Wire Wire Line
	3400 2350 4400 2350
Connection ~ 4400 2350
Wire Wire Line
	4400 2350 4400 2300
Wire Wire Line
	4700 2000 4700 2150
Wire Wire Line
	3400 2050 4400 2050
Connection ~ 4400 2050
Wire Wire Line
	4400 2050 4400 2300
Wire Wire Line
	4400 1750 4400 2050
Wire Wire Line
	3400 1850 4550 1850
Wire Wire Line
	4550 1850 4550 2000
Wire Wire Line
	4550 2000 4700 2000
$EndSCHEMATC
