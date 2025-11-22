Helper programs to find Einhell lawn mower GC-RM 500 pins
and may other lawn mower with STM32F103ZED6 (GD32F303ZET6 analog) CPU's

in PCB we found
3x MC33035DW 	Brushless DC Motor Controller  
3x MC33039 	Closed loop speed control adapter  
9x DTU6704 	Dual MOSFET  (FDD8424H) 3x Mootori väljundid
2x DTU6661	P-channel 60V 61A (toites/aku juures)
9579GM P-Channel 60-V (D-S) MOSFET (toite/aku juures)
3x LM358 OP
MZE4030 Logic quad 2-input EX-OR
LM2576HV 12V Voltage Regulator
LM2576 5.0V
GH16D 3.3V
2x MP6500 IMU

Detected pin's:
IMU #1 PB7  (SDA), PB6  (SCL)  confirmed (I2C1)
IMU #2 PB11 (SDA), PB10 (SCL)  confirmed (I2C2)

Buzer Alarm: ENABLE: PD3 (HIGH)
pin PC8 = All MC33035's pin 7 and pin 23 HIGH (active/electrical break)


Left MC33035 pin 3 PF3 = Dir (HIGH=Reverse, LOW=Forward)
Left MC33035 pin 23 PF2 = Enable HIGH (pin 23 = LOW)
Left MC33035 pin 11 PA4 = OP IN+ (PWM)

Right MC33035 pin 3 PF9 = Dir (HIGH=Reverse, LOW=Forward)
Right MC33035 pin 23 PF8 = Enable HIGH (pin 23 = LOW)
Right MC33035 pin 11 PA5 = OP IN+ (PWM)

Mow MC33035 pin 3 PF14 = Dir (HIGH=Reverse, LOW=Forward)
Mow MC33035 pin 23 PF13 = Enable HIGH (pin 23 = LOW)
Mow MC33035 pin 11 =

PF10 (HIGH) activates blue LED on PCB

  {PA9,  "Console TX"},
  {PA10, "Console RX"},
  {PB6,  "I2C1 SCL"},
  {PB7,  "I2C1 SDA"},
  {PB10, "I2C2 SCL"},
  {PB11, "I2C2 SDA"},
  {PD3,  "BUZZER/ALARM"},
  {PC8,  "Main ENABLE"},
  {PF2,  "Left ENABLE"},
  {PA4,  "Left PWM (OP IN+)"},
  {PF3,  "Left DIR"},
  {PF8,  "Right ENABLE"},
  {PF9,  "Right DIR"},
  {PA5,  "Right PWM (OP IN+)"},
  {PF13,  "Mow ENABLE"},
  {PF14,  "Mow DIR"}//,
  //{PA,  "Mow PWM (OP IN+)"} didn't find any

PA2 OP U3 out1
PA3 OP U3 out2
PC4 OP U4 out2
PD1 Power ON/OFF

** MC33035**
- **Pin 7: Output Enable (HIGH=run, LOW=stop)
- **Pin 3: Forward/Reverse (muudab suunda)
- **Pin 11: PWM
- **Pin 23: Brake (LOW=run, HIGH=brake)
**POWER & REFERENCE:**
- **Pin 17: VCC** (10-30V)
- **Pin 18: VC** (bottom drive power)
- **Pin 8: Reference Output** (6.25V)
**HALL SENSOR INPUTS**
- **Pin 4: SA** (sensor A)
- **Pin 5: SB** (sensor B)
- **Pin 6: SC** (sensor C)
