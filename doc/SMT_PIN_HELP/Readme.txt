Helper programs to find Einhell lawn mower GC-RM 500 pins
and my other lawn mower with STM32F103ZED6 (GD32F303ZET6 analog) CPU's

Einhell lawn mower GC-RM 500
STM32F103ZED6 Pins

IMU #1 PB7  (SDA), PB6  (SCL)  Confirmed (I2C1)
IMU #2 PB11 (SDA), PB10 (SCL)  Confirmed (I2C2)

Buzer Alarm: ENABLE: PD3 (HIGH)
MC33035 pin PC8 = All pin 7 HIGH , LOW All Brake

Serial
PA9 "Console TX", PA10 "Console RX"
PD5 "Serial6 TX", PD6 "Serial6 RX"
PC10 "Serial4 TX", PC11 "Serial4 RX"

PD5 R255 (Right Obstacle)
PD6 R252 (Left Lift)
PD4 R253 (Left Obstacle)
PD7 R254 (Right Lift)
PC9 (PCB Wifi pin5)

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
  {PC10, "Serial4 TX"},
  {PA11, "Serial4 RX"},
  {PD5,  "Serial6 TX R Obs"},
  {PD6,  "Serial6 RX L Lift"},
  {PA13, "SWDIO"},
  {PA14, "SWDCLK"},
  {PB6,  "I2C1 SCL"},
  {PB7,  "I2C1 SDA"},
  {PB10, "I2C2 SCL"},
  {PB11, "I2C2 SDA"},
  {PD3,  "BUZZER/ALARM"},
  {PD1,  "Power ON/OFF"},
  {PC3,  "Battery Voltage"},
  {PC8,  "Main ENABLE"},
  {PF2,  "Left ENABLE"},
  {PA4,  "Left PWM (OP IN+)"},
  {PA2,  "Left sense/current"},
  {PF3,  "Left DIR"},
  {PF8,  "Right ENABLE"},
  {PF9,  "Right DIR"},
  {PA5,  "Right PWM (OP IN+)"},
  {PA3,  "Right sense/current"},
  {PF13,  "Mow ENABLE"},
  {PF14, "Mow DIR"},
//  {PA?,  "Mow PWM"},
  {PF10, "Blue Led on PCB"},
  {PC4,  "(Battery sense/current?)"},
  {PA12, "Start button"},
  {PE7,  "Led Green Lock"},
  {PE10, "Led Red Lock"},
  {PE11, "Speaker"},
  {PE12, "Led Green Battery"},
  {PE13, "Led Green 4h"},
  {PE14, "Led Green 10h"},
  {PE15, "Lock Button"},
  {PD4,  "Left Obstacle"},
  {PD7,  "Right Lift"}

  {, ""},
  {, ""},
  {, ""},

PA12 : 0 -> 1 [CHANGED] Start button
PE7  : 0 -> 1 [CHANGED] Led Green Lock
PE10 : 0 -> 1 [CHANGED] Led Red Lock
PE11 : 0 -> 1 [CHANGED] Speaker 
PE12 : 0 -> 1 [CHANGED] Led Green Battery
PE13 : 0 -> 1 [CHANGED] Led Green 4h
PE14 : 0 -> 1 [CHANGED] Led Green 10h
PE15 : 0 -> 1 [CHANGED] Lock Button

PB0 : 0 -> 1 [CHANGED]
PB9 : 0 -> 1 [CHANGED] Power button?
PD7 : 0 -> 1 [CHANGED] Lift or bumper
PD8 : 0 -> 1 [CHANGED] Lift or bumper

PA2 op out1 U3 (Left sense/current)
PA3 op out2 U3 (Right sense/current)
PC4 op out2 U4 (Battery sense/current?)
PD1 Power ON/OFF
PC3 Battery Voltage


MC33035
**Guide signals:**
- **Pin 7: Output Enable (HIGH=run, LOW=stop)
- **Pin 3: Forward/Reverse
- **Pin 11: PWM
- **Pin 23: Brake (LOW=run, HIGH=brake)


**Power & REFERENCE:**
- **Pin 17: VCC** (10-30V) - kontrolli kas on pinge
- **Pin 18: VC** (bottom drive toide)
- **Pin 8: Reference Output** (6.25V väljund)

**Hall sensors inputs:**
- **Pin 4: SA** (sensor A)
- **Pin 5: SB** (sensor B)
- **Pin 6: SC** (sensor C)
