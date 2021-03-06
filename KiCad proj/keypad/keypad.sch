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
L Switch:SW_Push_LED SW1
U 1 1 60F25ED1
P 2500 2000
F 0 "SW1" H 2500 2385 50  0000 C CNN
F 1 "SW_Push_LED" H 2500 2294 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 2500 2300 50  0001 C CNN
F 3 "~" H 2500 2300 50  0001 C CNN
	1    2500 2000
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push_LED SW5
U 1 1 60F277FD
P 2550 3250
F 0 "SW5" H 2550 3635 50  0000 C CNN
F 1 "SW_Push_LED" H 2550 3544 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 2550 3550 50  0001 C CNN
F 3 "~" H 2550 3550 50  0001 C CNN
	1    2550 3250
	1    0    0    -1  
$EndComp
$Comp
L Diode:BAV70 D1
U 1 1 60F289D4
P 1600 2600
F 0 "D1" V 1554 2680 50  0000 L CNN
F 1 "BAV70" V 1645 2680 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 1600 2600 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/BAV70_SER.pdf" H 1600 2600 50  0001 C CNN
	1    1600 2600
	0    1    1    0   
$EndComp
$Comp
L Device:R R1
U 1 1 60F2AC28
P 2900 2300
F 0 "R1" H 2970 2346 50  0000 L CNN
F 1 "1k" H 2970 2255 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 2830 2300 50  0001 C CNN
F 3 "~" H 2900 2300 50  0001 C CNN
	1    2900 2300
	1    0    0    -1  
$EndComp
$Comp
L Device:R R5
U 1 1 60F2B26E
P 2900 2900
F 0 "R5" H 2970 2946 50  0000 L CNN
F 1 "1k" H 2970 2855 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 2830 2900 50  0001 C CNN
F 3 "~" H 2900 2900 50  0001 C CNN
	1    2900 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 2000 2900 2000
Wire Wire Line
	2900 2000 2900 2150
Wire Wire Line
	2750 3250 2900 3250
Wire Wire Line
	2900 3050 2900 3250
Wire Wire Line
	2900 2450 2900 2600
Wire Wire Line
	2300 2000 2100 2000
Wire Wire Line
	2100 2000 2100 3250
Wire Wire Line
	2100 3250 2350 3250
Wire Wire Line
	2300 1900 1600 1900
Wire Wire Line
	1600 1900 1600 2300
Wire Wire Line
	2350 3150 1600 3150
Wire Wire Line
	1600 3150 1600 2900
$Comp
L Switch:SW_Push_LED SW2
U 1 1 60F3B7F4
P 4450 2000
F 0 "SW2" H 4450 2385 50  0000 C CNN
F 1 "SW_Push_LED" H 4450 2294 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 4450 2300 50  0001 C CNN
F 3 "~" H 4450 2300 50  0001 C CNN
	1    4450 2000
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push_LED SW6
U 1 1 60F3B7FA
P 4500 3250
F 0 "SW6" H 4500 3635 50  0000 C CNN
F 1 "SW_Push_LED" H 4500 3544 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 4500 3550 50  0001 C CNN
F 3 "~" H 4500 3550 50  0001 C CNN
	1    4500 3250
	1    0    0    -1  
$EndComp
$Comp
L Diode:BAV70 D2
U 1 1 60F3B800
P 3550 2600
F 0 "D2" V 3504 2680 50  0000 L CNN
F 1 "BAV70" V 3595 2680 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 3550 2600 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/BAV70_SER.pdf" H 3550 2600 50  0001 C CNN
	1    3550 2600
	0    1    1    0   
$EndComp
$Comp
L Device:R R2
U 1 1 60F3B806
P 4850 2300
F 0 "R2" H 4920 2346 50  0000 L CNN
F 1 "1k" H 4920 2255 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 4780 2300 50  0001 C CNN
F 3 "~" H 4850 2300 50  0001 C CNN
	1    4850 2300
	1    0    0    -1  
$EndComp
$Comp
L Device:R R6
U 1 1 60F3B80C
P 4850 2900
F 0 "R6" H 4920 2946 50  0000 L CNN
F 1 "1k" H 4920 2855 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 4780 2900 50  0001 C CNN
F 3 "~" H 4850 2900 50  0001 C CNN
	1    4850 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	4650 2000 4850 2000
Wire Wire Line
	4850 2000 4850 2150
Wire Wire Line
	4700 3250 4850 3250
Wire Wire Line
	4850 3050 4850 3250
Wire Wire Line
	4850 2450 4850 2600
Wire Wire Line
	4250 2000 4050 2000
Wire Wire Line
	4050 2000 4050 3250
Wire Wire Line
	4050 3250 4300 3250
Wire Wire Line
	4250 1900 3550 1900
Wire Wire Line
	3550 1900 3550 2300
Wire Wire Line
	4300 3150 3550 3150
Wire Wire Line
	3550 3150 3550 2900
$Comp
L Switch:SW_Push_LED SW3
U 1 1 60F3F5A8
P 6450 2000
F 0 "SW3" H 6450 2385 50  0000 C CNN
F 1 "SW_Push_LED" H 6450 2294 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 6450 2300 50  0001 C CNN
F 3 "~" H 6450 2300 50  0001 C CNN
	1    6450 2000
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push_LED SW7
U 1 1 60F3F5AE
P 6500 3250
F 0 "SW7" H 6500 3635 50  0000 C CNN
F 1 "SW_Push_LED" H 6500 3544 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 6500 3550 50  0001 C CNN
F 3 "~" H 6500 3550 50  0001 C CNN
	1    6500 3250
	1    0    0    -1  
$EndComp
$Comp
L Diode:BAV70 D3
U 1 1 60F3F5B4
P 5550 2600
F 0 "D3" V 5504 2680 50  0000 L CNN
F 1 "BAV70" V 5595 2680 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 5550 2600 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/BAV70_SER.pdf" H 5550 2600 50  0001 C CNN
	1    5550 2600
	0    1    1    0   
$EndComp
$Comp
L Device:R R3
U 1 1 60F3F5BA
P 6850 2300
F 0 "R3" H 6920 2346 50  0000 L CNN
F 1 "1k" H 6920 2255 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 6780 2300 50  0001 C CNN
F 3 "~" H 6850 2300 50  0001 C CNN
	1    6850 2300
	1    0    0    -1  
$EndComp
$Comp
L Device:R R7
U 1 1 60F3F5C0
P 6850 2900
F 0 "R7" H 6920 2946 50  0000 L CNN
F 1 "1k" H 6920 2855 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 6780 2900 50  0001 C CNN
F 3 "~" H 6850 2900 50  0001 C CNN
	1    6850 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	6650 2000 6850 2000
Wire Wire Line
	6850 2000 6850 2150
Wire Wire Line
	6700 3250 6850 3250
Wire Wire Line
	6850 3050 6850 3250
Wire Wire Line
	6850 2450 6850 2600
Wire Wire Line
	6250 2000 6050 2000
Wire Wire Line
	6050 2000 6050 3250
Wire Wire Line
	6050 3250 6300 3250
Wire Wire Line
	6250 1900 5550 1900
Wire Wire Line
	5550 1900 5550 2300
Wire Wire Line
	6300 3150 5550 3150
Wire Wire Line
	5550 3150 5550 2900
$Comp
L Switch:SW_Push_LED SW4
U 1 1 60F44472
P 8450 2000
F 0 "SW4" H 8450 2385 50  0000 C CNN
F 1 "SW_Push_LED" H 8450 2294 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 8450 2300 50  0001 C CNN
F 3 "~" H 8450 2300 50  0001 C CNN
	1    8450 2000
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push_LED SW8
U 1 1 60F44478
P 8500 3250
F 0 "SW8" H 8500 3635 50  0000 C CNN
F 1 "SW_Push_LED" H 8500 3544 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 8500 3550 50  0001 C CNN
F 3 "~" H 8500 3550 50  0001 C CNN
	1    8500 3250
	1    0    0    -1  
$EndComp
$Comp
L Diode:BAV70 D4
U 1 1 60F4447E
P 7550 2600
F 0 "D4" V 7504 2680 50  0000 L CNN
F 1 "BAV70" V 7595 2680 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 7550 2600 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/BAV70_SER.pdf" H 7550 2600 50  0001 C CNN
	1    7550 2600
	0    1    1    0   
$EndComp
$Comp
L Device:R R4
U 1 1 60F44484
P 8850 2300
F 0 "R4" H 8920 2346 50  0000 L CNN
F 1 "1k" H 8920 2255 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 8780 2300 50  0001 C CNN
F 3 "~" H 8850 2300 50  0001 C CNN
	1    8850 2300
	1    0    0    -1  
$EndComp
$Comp
L Device:R R8
U 1 1 60F4448A
P 8850 2900
F 0 "R8" H 8920 2946 50  0000 L CNN
F 1 "1k" H 8920 2855 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 8780 2900 50  0001 C CNN
F 3 "~" H 8850 2900 50  0001 C CNN
	1    8850 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	8650 2000 8850 2000
Wire Wire Line
	8850 2000 8850 2150
Wire Wire Line
	8700 3250 8850 3250
Wire Wire Line
	8850 3050 8850 3250
Wire Wire Line
	8850 2450 8850 2600
Wire Wire Line
	8250 2000 8050 2000
Wire Wire Line
	8050 2000 8050 3250
Wire Wire Line
	8050 3250 8300 3250
Wire Wire Line
	8250 1900 7550 1900
Wire Wire Line
	7550 1900 7550 2300
Wire Wire Line
	8300 3150 7550 3150
Wire Wire Line
	7550 3150 7550 2900
Wire Wire Line
	2750 3150 3000 3150
Wire Wire Line
	3000 3150 3000 3550
Wire Wire Line
	3000 3550 4950 3550
Wire Wire Line
	4700 3150 4950 3150
Wire Wire Line
	4950 3150 4950 3550
Connection ~ 4950 3550
Wire Wire Line
	6700 3150 6950 3150
Wire Wire Line
	6950 3150 6950 3550
Connection ~ 6950 3550
Wire Wire Line
	8700 3150 8950 3150
Wire Wire Line
	8950 3150 8950 3550
Wire Wire Line
	6950 3550 8950 3550
Wire Wire Line
	2700 1900 2900 1900
Wire Wire Line
	2900 1900 2900 1500
Wire Wire Line
	2900 1500 4850 1500
Wire Wire Line
	8850 1500 8850 1900
Wire Wire Line
	8850 1900 8650 1900
Wire Wire Line
	6650 1900 6850 1900
Wire Wire Line
	6850 1900 6850 1500
Connection ~ 6850 1500
Wire Wire Line
	6850 1500 8850 1500
Wire Wire Line
	4650 1900 4850 1900
Wire Wire Line
	4850 1900 4850 1500
Connection ~ 4850 1500
Wire Wire Line
	2900 2600 3150 2600
Wire Wire Line
	3150 2600 3150 3700
Wire Wire Line
	9100 3700 9100 2600
Wire Wire Line
	9100 2600 8850 2600
Connection ~ 2900 2600
Wire Wire Line
	2900 2600 2900 2750
Connection ~ 8850 2600
Wire Wire Line
	8850 2600 8850 2750
Wire Wire Line
	6850 2600 7100 2600
Wire Wire Line
	7100 2600 7100 3700
Connection ~ 6850 2600
Wire Wire Line
	6850 2600 6850 2750
Connection ~ 7100 3700
Wire Wire Line
	7100 3700 9100 3700
Wire Wire Line
	4850 2600 5100 2600
Wire Wire Line
	5100 2600 5100 3700
Connection ~ 4850 2600
Wire Wire Line
	4850 2600 4850 2750
Connection ~ 5100 3700
$Comp
L Connector:Conn_01x10_Male J1
U 1 1 60F5B546
P 5550 5000
F 0 "J1" V 5477 4928 50  0000 C CNN
F 1 "Keyboard_connector" V 5386 4928 50  0000 C CNN
F 2 "Connector_FFC-FPC:Hirose_FH12-10S-0.5SH_1x10-1MP_P0.50mm_Horizontal" H 5550 5000 50  0001 C CNN
F 3 "~" H 5550 5000 50  0001 C CNN
	1    5550 5000
	0    -1   -1   0   
$EndComp
$Comp
L Device:Q_PNP_BEC Q1
U 1 1 60F5D6C5
P 3250 4150
F 0 "Q1" H 3441 4196 50  0000 L CNN
F 1 "Q_PNP_BEC" H 3441 4105 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 3450 4250 50  0001 C CNN
F 3 "~" H 3250 4150 50  0001 C CNN
	1    3250 4150
	-1   0    0    -1  
$EndComp
Wire Wire Line
	3150 3700 3150 3950
Wire Wire Line
	5350 4800 5350 4150
Wire Wire Line
	3150 4350 3150 4550
Wire Wire Line
	3150 4550 3600 4550
Wire Wire Line
	5950 4550 5950 4800
$Comp
L Device:R R9
U 1 1 60F79F27
P 3600 4350
F 0 "R9" H 3670 4396 50  0000 L CNN
F 1 "1k" H 3670 4305 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 3530 4350 50  0001 C CNN
F 3 "~" H 3600 4350 50  0001 C CNN
	1    3600 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 4200 3600 4150
Connection ~ 3600 4150
Wire Wire Line
	3600 4150 3450 4150
Wire Wire Line
	3600 4500 3600 4550
Connection ~ 3600 4550
Wire Wire Line
	1400 2600 1400 3850
Wire Wire Line
	1400 3850 5550 3850
Wire Wire Line
	5550 3850 5550 4800
Wire Wire Line
	3350 2600 3350 3750
Wire Wire Line
	3350 3750 5650 3750
Wire Wire Line
	5650 3750 5650 4800
Wire Wire Line
	5350 2600 5350 3400
Wire Wire Line
	5350 3400 5750 3400
Wire Wire Line
	5750 3400 5750 4800
Wire Wire Line
	7350 2600 7350 3850
Wire Wire Line
	7350 3850 5850 3850
Wire Wire Line
	5850 3850 5850 4800
Wire Wire Line
	2100 4700 4050 4700
Wire Wire Line
	6050 4700 6050 4800
Connection ~ 3150 3700
Wire Wire Line
	3150 3700 5100 3700
Wire Wire Line
	2100 3250 2100 4700
Connection ~ 2100 3250
Wire Wire Line
	3600 4150 5350 4150
Wire Wire Line
	3600 4550 5950 4550
Wire Wire Line
	4050 3250 4050 4700
Connection ~ 4050 3250
Connection ~ 4050 4700
Wire Wire Line
	4050 4700 6050 4700
Wire Wire Line
	6050 3250 6050 4700
Connection ~ 6050 3250
Connection ~ 6050 4700
Wire Wire Line
	6050 4700 8050 4700
Wire Wire Line
	8050 4700 8050 3250
Connection ~ 8050 3250
Text GLabel 2100 4600 0    50   Input ~ 0
Ground
Text GLabel 4550 4150 0    50   Input ~ 0
led_ctrl
Text GLabel 4650 4550 0    50   Input ~ 0
pwr
Text GLabel 3150 3700 0    50   Input ~ 0
led_pwr
Text GLabel 7850 3550 0    50   Input ~ 0
row1
Text GLabel 7250 1500 0    50   Input ~ 0
row2
Text GLabel 1400 3400 0    50   Input ~ 0
key1
Text GLabel 3350 3250 0    50   Input ~ 0
key2
Text GLabel 5650 3400 0    50   Input ~ 0
key3
Text GLabel 7350 3300 0    50   Input ~ 0
key4
Wire Wire Line
	4950 3550 5150 3550
$Comp
L Device:R R10
U 1 1 60F48D5B
P 5150 4350
F 0 "R10" H 5220 4396 50  0000 L CNN
F 1 "200" H 5220 4305 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 5080 4350 50  0001 C CNN
F 3 "~" H 5150 4350 50  0001 C CNN
	1    5150 4350
	1    0    0    -1  
$EndComp
$Comp
L Device:R R11
U 1 1 60F49845
P 5250 1900
F 0 "R11" H 5320 1946 50  0000 L CNN
F 1 "200" H 5320 1855 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 5180 1900 50  0001 C CNN
F 3 "~" H 5250 1900 50  0001 C CNN
	1    5250 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	5150 4800 5150 4500
Wire Wire Line
	5250 4800 5250 2050
Wire Wire Line
	4850 1500 5250 1500
Wire Wire Line
	5250 1750 5250 1500
Connection ~ 5250 1500
Wire Wire Line
	5250 1500 6850 1500
Wire Wire Line
	5100 3700 7100 3700
Wire Wire Line
	5150 4200 5150 3550
Connection ~ 5150 3550
Wire Wire Line
	5150 3550 6950 3550
Text GLabel 5150 4650 0    50   Input ~ 0
pb0
Text GLabel 5250 3900 3    50   Input ~ 0
pb1
Text GLabel 5350 4650 0    50   Input ~ 0
pb2
Text GLabel 6300 4700 3    50   Input ~ 0
gnd
Text GLabel 5550 4150 3    50   Input ~ 0
pb11
Text GLabel 5650 4150 3    50   Input ~ 0
pb12
Text GLabel 5750 4150 3    50   Input ~ 0
pb13
Text GLabel 5850 4150 3    50   Input ~ 0
pb14
$EndSCHEMATC
