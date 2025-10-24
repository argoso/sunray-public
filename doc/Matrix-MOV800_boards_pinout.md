# Matrix MOW800 Reverse Engineering / Pinbelegung

## Mainboard / Motertreiber-Board
3x MC33035DW 	Brushless DC Motor Controller  
3x MC33039 		Closed loop speed control adapter  
9x FDD8424H 	Dual MOSFET  
74HC4052		Perimer loop connection 
TPS3840 (ZA80)	Voltage monitoring 4x

---------------
## CPU-Board
STM32F103ZET6	main CPU (72MHz, 512KB Flash, 64KB SRAM, 2x I2C, 5x USART)  
MMA8452Q		3-axis, 12-bit/8-bit digital accelerometer  
BUZZER

### USB_USART Stecker
| Pin | Connection on the CPU board |
|---|---|
| 1 | +5V |
| 2 | CPU-138 (BOOT0) | 
| 3 | CPU-25  (NRST = RESET) | 
| 4 | +3.3V (VDD) |
| 5 | CPU-102 (PA10 = USART1-RxD) |
| 6 | CPU-101 (PA9  = USART1-TxD) |
| 7 | GND |

### SWD Stecker (Programmierung CPU)
| Pin | Connection on the CPU board |
|---|---|
| 1 | +3.3V (VDD) |
| 2 | CPU-105 (PA13 = SWDIO) |
| 3 | CPU-109 (PA14 = SWCLK) |
| 4 | CPU-25 (NRST = RESET) |
| 5 | GND |

### J1 Stecker (zum Main-Board)
| Stecker PIN | Connection on the CPU board | Connection on the mainboard |
|-------------|------------------------------|------------------------------|
| 1  (GND)    |   |   |
| 2  (GND)    |   |   |
| 3  (GND)    |   |   |
| 4  (+3,3V)  |   |   |
| 5  (ON/OFF) | Data-Stecker-1 1kOhm | Power on |
| 6  (+3,3V)  |   |   |
| 7  (CHECK)  | CPU-65 (PE12) |	Diode D31     |
| 8	 (+5V)    |   |   |
| 9	 (CK_W)   | CPU-45 (PC5)  |	Rain sensor |
| 10 (+5V)    |   |   |
| 11 (MC12)	  |	CPU-44 (PC4)  |	(Bridge to pin 12 separated = R159 0Ohm removed) Mower motor speed (Self-occupancy) |
| 12 (MC1)	  |	CPU-43 (PA7)  |	(Bridge to pin 11 separated = R159 0Ohm removed) (Current measurement Mower motor) |
| 13 (SPDR)   |	CPU-35 (PA1)  |	(Motor R speed)    |
| 14 (RR12)	  | CPU-42 (PA6)  |	(Bridge to pin 16 separated = R161 0Ohm removed) Motor R speed (Self-occupancy) |
| 15 (CK_V)   |	CPU-37 (PA3)  |	(Measuring the battery voltage) |
| 16 (RR1)    |	CPU-29 (PC3)  |	(Bridge to pin 14 separated = R161 0Ohm removed) (Current measurement Motor R) |
| 17 (DIRR0)  |	CPU-15 (PF5)  |	(Motor R direction) |
| 18 (RL12)	  |	CPU-28 	(PC2) |	(Bridge to pin 20 separated = R160 0Ohm removed) Motor L speed (Self-occupancy) |
| 19 (C_ADY3) |	CPU-21 (PF9)  | |
| 20 (RL1)	  |	CPU-27 (PC1)  |	(Bridge to pin 18 separated = R160 0Ohm removed) (Current measurement Motor L) |
| 21 (ADDR1)  |	CPU-115 (PD1) |	|
| 22 (CHARGE_1) | CPU-26 (PC0) | (Charging current measurement) |

### Data Stecker (zu LCD Platiene)
| Pin | Connection on the CPU board |
|---|---|
| 1 | J1-Stecker-5 (ON/OFF) 1kOhm |
| 2	| CPU-70	(PB11 USART3_RX) 	1kOhm (Pin on my board strangely interrupted) |
| 3 | CPU-69	(PB10 USART3_TX)	1kOhm |

### J2 Stecker (zum Main-Board)
| Stecker PIN       | Connection on the CPU board | Connection on the motherboard |
|-------------------|---|---|
| 1	 (IROUT)        | Infrearet-Stecker-2 |
| 2	 (-)	        | CPU-73 (PB12) | (Motor L+R Brake off ?) |
| 3	 (IR RECIVE)    | Infrearet-Stecker-3 |
| 4	 (-)		    | CPU-74 (PB13) | (Motor L+R ON/OFF) |
| 5	 (CK-RF)        | CPU-139 (PB8) | Rain sensor (own occupancy) |
| 6	 (-)		    | CPU-75 (PB14) | (Perimeter Schlefen Soensor select ?) |
| 7	 (C_ADY)	    | CPU-18 (PF6) | (Perimeter Schlefen Soensor analog signal ?) |
| 8	 (-)		    | CPU-76 (PB15) | (Perimeter Schlefen Soensor select ?) |
| 9	 (C_ADY1)	    | CPU-19 (PF7) | Sonar left trigger (custom assignment) | 				 
| 10 (C_ADY2)	    | CPU-20 (PF8) | Sonar right trigger (custom assignment) |
| 11 (XV1)		    | CPU-85 (PD14) | (Bridge to pin 12) | Power supply via charging station is ON |
| 12 (XV)		    | CPU-86 (PD15)	| (Bridge to pin 11) |
| 13 (FLAT_SENSOR1) | CPU-58 (PE7)	| (Hood lifting sensor L+R 2) |
| 14 (CNCRUNE)	    | CPU-7	(PC13)  | (Mower motor ON/OFF ?) |
| 15 (FLAT_SENSOR)	| CPU-79 (PD10)	| (Hood lifting sensor L+R 1) |
| 16 (CNRUNE)		| CPU-112 (PC11) | UART4_RX GPS (Self-occupancy) |
| 17 (DIRL0)		| CPU-98 (PC8)	 | (Motor L direction) |
| 18 (SPDL)			| CPU-34 (PA0)	 | (Motor L speed ?) |
| 19 (CHECKM1)		| CPU-118 (PD4)	 | (all engines brake off ?) |
| 20 (TRK)			| CPU-113 (PC12) | (Mower motor brake off ?) |
| 21 (CNBRK)		| CPU-111 (PC10) | UART4_TX GPS (Self-occupancy) |
| 22 (+5V)          |                |  |


### HMC5883L Stecker
| Pin | Connection on the CPU board | new use |
|---|---| |
| 1 | GND | GND Sonar |
| 2 | CPU-116 (PD2) | Sonar center Trigger (Self-occupancy) |
| 3 | CPU-117 (PD3) | Sonar center Echo (Self-occupancy) |
| 4 | +3,3V | VDD Sonar |

### Dub-Stecker
| Pin | Connection on the CPU board |
|---|---|
| 1 | GND (PE2) |
| 2 | CPU-1 |

### MMA8452Q Chip
| Pin | Connection on the CPU board |
|---|---|
| SDA | CPU-137  (PB7 = SDA) |
| SCL | CPU-136  (PB6 = SCL) |

### Crash Stecker
| Stecker Pin | Connection on the CPU board | Connection on the crash board |
|-------------|------------------------------|---|
| 1 | CPU-36 (PA2) | CPU-16 (PA6) over 10k resistance
| 2 | VDD +3.3V | VDD +3.3V
| 3 | CPU-77 (PD8) | HAL rechts
| 4 | CPU-78 (PD9) | HAL links
| 5 | CPU-47 (PB1) | (CPU-26 (PB13) SPI2-SCK getrennt)    Sonar rechts Echo (Self-occupancy)
| 6 | CPU-46 (PB0) | (CPU-27 (PB14) SPI2-MISO getrennt)   Sonar links Echo (Self-occupancy)
| 7 | GND | GND

### CPU
| Pin | Connection on the crash board |
|---|---|
| 89 (PG4) | BUZZER |
| 119 (PD5 = UART2_TX) | ESP32 Serial6 | 
| 122 (PD6 = UART2_RX) | ESP32 Serial6 |

### Interrupt Übersicht
| Pin | Funktion |
|---|---|
| PC2  | PWM Motor L (Self-occupancy)
| PC4  | PWM Motor R (Self-occupancy)
| PA6  | PWM Motor rechts (Self-occupancy)
| PE7  | Hood lifting sensor L+R 2
| PD8  | HAL Bumper rechts
| PD9  | HAL Bumper links
| PD10 | Hood lifting sensor L+R 1
| PB0  | Sonar left echo
| PD3  | Sonar center echo
| PB1  | Sonar right echo

---------------
## Crash Board
CPU STM32F103C86

### J1 Stecker
| Pin | Connection on the board |
|---|---|
| 1 | +3,3V |
| 2 | CPU-34 (PA13 = SWDIO) |
| 3 | CPU-37 (PA14 = SWCLK) |
| 4 | GND |

### CPU 
| Pin | Connection on the board |
|---|---|
| 40 (PB4) | LED |
