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
L Connector:Conn_01x02_Male J2
U 1 1 612762CC
P 1450 2050
F 0 "J2" H 1550 2350 50  0000 C CNN
F 1 "IN" H 1550 2250 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x02_P2.54mm_Vertical" H 1450 2050 50  0001 C CNN
F 3 "~" H 1450 2050 50  0001 C CNN
	1    1450 2050
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x02_Male J1
U 1 1 61276A77
P 1425 3025
F 0 "J1" H 1525 3325 50  0000 C CNN
F 1 "OUT" H 1525 3225 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x02_P2.54mm_Vertical" H 1425 3025 50  0001 C CNN
F 3 "~" H 1425 3025 50  0001 C CNN
	1    1425 3025
	1    0    0    -1  
$EndComp
$Comp
L Device:D_Small_ALT D4
U 1 1 61278134
P 3700 2400
F 0 "D4" V 3700 2332 50  0000 R CNN
F 1 "D_Small_ALT" V 3745 2468 50  0001 L CNN
F 2 "Diode_SMD:D_MiniMELF_Handsoldering" V 3700 2400 50  0001 C CNN
F 3 "~" V 3700 2400 50  0001 C CNN
	1    3700 2400
	0    -1   -1   0   
$EndComp
$Comp
L Device:D_Small_ALT D3
U 1 1 61278903
P 3500 2300
F 0 "D3" H 3500 2187 50  0000 C CNN
F 1 "D_Small_ALT" V 3545 2368 50  0001 L CNN
F 2 "Diode_SMD:D_MiniMELF_Handsoldering" V 3500 2300 50  0001 C CNN
F 3 "~" V 3500 2300 50  0001 C CNN
	1    3500 2300
	-1   0    0    1   
$EndComp
$Comp
L Device:D_Small_ALT D2
U 1 1 61278C65
P 3275 2200
F 0 "D2" V 3275 2350 50  0000 R CNN
F 1 "D_Small_ALT" V 3320 2268 50  0001 L CNN
F 2 "Diode_SMD:D_MiniMELF_Handsoldering" V 3275 2200 50  0001 C CNN
F 3 "~" V 3275 2200 50  0001 C CNN
	1    3275 2200
	0    -1   -1   0   
$EndComp
$Comp
L Device:D_Zener_Small_ALT D1
U 1 1 61279B58
P 2900 2975
F 0 "D1" V 2854 3043 50  0000 L CNN
F 1 "3.3V or 4.7V" H 2700 2875 50  0000 L CNN
F 2 "Diode_THT:D_5W_P10.16mm_Horizontal" V 2900 2975 50  0001 C CNN
F 3 "~" V 2900 2975 50  0001 C CNN
	1    2900 2975
	0    1    1    0   
$EndComp
$Comp
L power:VCC #PWR03
U 1 1 6127A5A5
P 1850 1950
F 0 "#PWR03" H 1850 1800 50  0001 C CNN
F 1 "VCC" H 1867 2123 50  0000 C CNN
F 2 "" H 1850 1950 50  0001 C CNN
F 3 "" H 1850 1950 50  0001 C CNN
	1    1850 1950
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR01
U 1 1 6127AA92
P 1825 2925
F 0 "#PWR01" H 1825 2775 50  0001 C CNN
F 1 "+5V" H 1840 3098 50  0000 C CNN
F 2 "" H 1825 2925 50  0001 C CNN
F 3 "" H 1825 2925 50  0001 C CNN
	1    1825 2925
	1    0    0    -1  
$EndComp
Wire Wire Line
	1625 3025 1825 3025
Wire Wire Line
	1825 3025 1825 2925
$Comp
L power:GNDA #PWR02
U 1 1 6127BD96
P 1825 3225
F 0 "#PWR02" H 1825 2975 50  0001 C CNN
F 1 "GNDA" H 1830 3052 50  0000 C CNN
F 2 "" H 1825 3225 50  0001 C CNN
F 3 "" H 1825 3225 50  0001 C CNN
	1    1825 3225
	1    0    0    -1  
$EndComp
Wire Wire Line
	1625 3125 1825 3125
Wire Wire Line
	1825 3125 1825 3225
Wire Wire Line
	1650 2050 1850 2050
Wire Wire Line
	1850 2050 1850 1950
$Comp
L power:GNDA #PWR04
U 1 1 6127FC6C
P 1850 2250
F 0 "#PWR04" H 1850 2000 50  0001 C CNN
F 1 "GNDA" H 1855 2077 50  0000 C CNN
F 2 "" H 1850 2250 50  0001 C CNN
F 3 "" H 1850 2250 50  0001 C CNN
	1    1850 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1650 2150 1850 2150
Wire Wire Line
	1850 2150 1850 2250
$Comp
L Device:Q_PMOS_GDS Q2
U 1 1 612873F5
P 4400 2125
F 0 "Q2" V 4651 2125 50  0000 C CNN
F 1 "Q_PMOS_GDS" V 4652 2125 50  0001 C CNN
F 2 "Package_TO_SOT_SMD:TO-263-3_TabPin2" H 4600 2225 50  0001 C CNN
F 3 "~" H 4400 2125 50  0001 C CNN
	1    4400 2125
	0    1    -1   0   
$EndComp
$Comp
L Device:R_Small R3
U 1 1 6129DB4E
P 3700 3200
F 0 "R3" H 3759 3246 50  0000 L CNN
F 1 "10K" H 3759 3155 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 3700 3200 50  0001 C CNN
F 3 "~" H 3700 3200 50  0001 C CNN
	1    3700 3200
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R1
U 1 1 612A9072
P 2900 2575
F 0 "R1" H 2950 2575 50  0000 L CNN
F 1 "1K" V 2825 2525 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 2900 2575 50  0001 C CNN
F 3 "~" H 2900 2575 50  0001 C CNN
	1    2900 2575
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_PNP_BEC Q1
U 1 1 6128690F
P 3600 2775
F 0 "Q1" H 3791 2775 50  0000 L CNN
F 1 "Q_PNP_BEC" H 3790 2730 50  0001 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23_Handsoldering" H 3800 2875 50  0001 C CNN
F 3 "~" H 3600 2775 50  0001 C CNN
	1    3600 2775
	1    0    0    1   
$EndComp
Wire Wire Line
	2900 2475 2900 2025
Wire Wire Line
	3700 2975 3700 3025
Wire Wire Line
	2575 2025 2900 2025
Connection ~ 2900 2025
Wire Wire Line
	2900 2025 3275 2025
Wire Wire Line
	2900 2675 2900 2775
$Comp
L Device:R_Small R2
U 1 1 612BB1FF
P 3175 2775
F 0 "R2" V 3100 2775 50  0000 C CNN
F 1 "10K" V 3250 2775 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 3175 2775 50  0001 C CNN
F 3 "~" H 3175 2775 50  0001 C CNN
	1    3175 2775
	0    1    1    0   
$EndComp
Wire Wire Line
	3075 2775 2900 2775
Connection ~ 2900 2775
Wire Wire Line
	2900 2775 2900 2875
Wire Wire Line
	3275 2775 3400 2775
$Comp
L power:GNDA #PWR06
U 1 1 612AD1E4
P 3700 3400
F 0 "#PWR06" H 3700 3150 50  0001 C CNN
F 1 "GNDA" H 3705 3227 50  0000 C CNN
F 2 "" H 3700 3400 50  0001 C CNN
F 3 "" H 3700 3400 50  0001 C CNN
	1    3700 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 3300 3700 3350
Wire Wire Line
	2900 3075 2900 3350
Wire Wire Line
	2900 3350 3700 3350
Connection ~ 3700 3350
Wire Wire Line
	3700 3350 3700 3400
Wire Wire Line
	4400 2325 4400 3025
Wire Wire Line
	4400 3025 3700 3025
Connection ~ 3700 3025
Wire Wire Line
	3700 3025 3700 3100
Wire Wire Line
	3275 2100 3275 2025
Connection ~ 3275 2025
Wire Wire Line
	3275 2025 4200 2025
Wire Wire Line
	3275 2300 3400 2300
Wire Wire Line
	3600 2300 3700 2300
Wire Wire Line
	3700 2500 3700 2575
$Comp
L power:VCC #PWR05
U 1 1 6129A01A
P 2575 1625
F 0 "#PWR05" H 2575 1475 50  0001 C CNN
F 1 "VCC" H 2592 1798 50  0000 C CNN
F 2 "" H 2575 1625 50  0001 C CNN
F 3 "" H 2575 1625 50  0001 C CNN
	1    2575 1625
	1    0    0    -1  
$EndComp
Wire Wire Line
	2575 2025 2575 1925
$Comp
L power:+5V #PWR07
U 1 1 612E3217
P 4825 1925
F 0 "#PWR07" H 4825 1775 50  0001 C CNN
F 1 "+5V" H 4840 2098 50  0000 C CNN
F 2 "" H 4825 1925 50  0001 C CNN
F 3 "" H 4825 1925 50  0001 C CNN
	1    4825 1925
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 2025 4825 2025
Wire Wire Line
	4825 2025 4825 1925
$Comp
L Device:Polyfuse_Small F1
U 1 1 6128F5C5
P 2575 1825
F 0 "F1" H 2643 1871 50  0000 L CNN
F 1 "5V 500mA" H 2643 1780 50  0000 L CNN
F 2 "Fuse:Fuse_1812_4532Metric_Pad1.30x3.40mm_HandSolder" H 2625 1625 50  0001 L CNN
F 3 "~" H 2575 1825 50  0001 C CNN
	1    2575 1825
	1    0    0    -1  
$EndComp
Wire Wire Line
	2575 1625 2575 1725
$EndSCHEMATC