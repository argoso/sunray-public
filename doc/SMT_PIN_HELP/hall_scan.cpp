#include <Arduino.h>

// ---------- Sinu Console --------------
HardwareSerial Console(USART1);

// ---------- PIN MAP (sama mis sinul) --------------
struct PinEntry { int pin; const char* name; };
struct ExcludedPin { int pin; const char* reason; };

const PinEntry pinMap[] = {
    {PA0,"PA0"},{PA1,"PA1"},{PA2,"PA2"},{PA3,"PA3"},{PA4,"PA4"},{PA5,"PA5"},{PA6,"PA6"},{PA7,"PA7"},
    {PA8,"PA8"},{PA9,"PA9"},{PA10,"PA10"},{PA11,"PA11"},{PA12,"PA12"},{PA13,"PA13"},{PA14,"PA14"},{PA15,"PA15"},
    {PB0,"PB0"},{PB1,"PB1"},{PB2,"PB2"},{PB3,"PB3"},{PB4,"PB4"},{PB5,"PB5"},{PB6,"PB6"},{PB7,"PB7"},
    {PB8,"PB8"},{PB9,"PB9"},{PB10,"PB10"},{PB11,"PB11"},{PB12,"PB12"},{PB13,"PB13"},{PB14,"PB14"},{PB15,"PB15"},
    {PC0,"PC0"},{PC1,"PC1"},{PC2,"PC2"},{PC3,"PC3"},{PC4,"PC4"},{PC5,"PC5"},{PC6,"PC6"},{PC7,"PC7"},
    {PC8,"PC8"},{PC9,"PC9"},{PC10,"PC10"},{PC11,"PC11"},{PC12,"PC12"},{PC13,"PC13"},{PC14,"PC14"},{PC15,"PC15"},
    {PD0,"PD0"},{PD1,"PD1"},{PD2,"PD2"},{PD3,"PD3"},{PD4,"PD4"},{PD5,"PD5"},{PD6,"PD6"},{PD7,"PD7"},
    {PD8,"PD8"},{PD9,"PD9"},{PD10,"PD10"},{PD11,"PD11"},{PD12,"PD12"},{PD13,"PD13"},{PD14,"PD14"},{PD15,"PD15"},
    {PE0,"PE0"},{PE1,"PE1"},{PE2,"PE2"},{PE3,"PE3"},{PE4,"PE4"},{PE5,"PE5"},{PE6,"PE6"},{PE7,"PE7"},
    {PE8,"PE8"},{PE9,"PE9"},{PE10,"PE10"},{PE11,"PE11"},{PE12,"PE12"},{PE13,"PE13"},{PE14,"PE14"},{PE15,"PE15"},
    {PF0,"PF0"},{PF1,"PF1"},{PF2,"PF2"},{PF3,"PF3"},{PF4,"PF4"},{PF5,"PF5"},{PF6,"PF6"},{PF7,"PF7"},
    {PF8,"PF8"},{PF9,"PF9"},{PF10,"PF10"},{PF11,"PF11"},{PF12,"PF12"},{PF13,"PF13"},{PF14,"PF14"},{PF15,"PF15"}
};
const int totalPinCount = sizeof(pinMap)/sizeof(pinMap[0]);

const ExcludedPin excludedPins[] = {
  {PA9,  "Console TX"},
  {PA10, "Console RX"},
  {PC10, "Serial4 TX"},
  {PA11, "Serial4 RX"},
  {PD5,  "Serial6 TX"},
  {PD6,  "Serial6 RX"},
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
  {PD7,  "Lift or bumper"}
};
const int excludedCount = sizeof(excludedPins)/sizeof(excludedPins[0]);

bool isExcluded(int p) {
    for (int i=0; i<excludedCount; i++)
        if (excludedPins[i].pin == p) return true;
    return false;
}

// --------- MOTOR TEST PINID -----------
const int GLOBAL_ENABLE = PC8;
const int MOTOR_ENABLE   = PF8; //PF8 - Right, PF2 - Left, PF13 - Mow
const int MOTOR_DIR      = PF9; //PF9 - Right, PF3 - Left, PF14 - MOw
const int MOTOR_PWM      = PA5; //PA5 - Right, PA4 - Left, ? - Mow

// --------- MOOTORI KÄIVITUS ----------
void startMotor() {
    pinMode(GLOBAL_ENABLE, OUTPUT);
    pinMode(MOTOR_ENABLE, OUTPUT);
    pinMode(MOTOR_DIR, OUTPUT);
    pinMode(MOTOR_PWM, OUTPUT);

    digitalWrite(GLOBAL_ENABLE, HIGH);
    digitalWrite(MOTOR_ENABLE, HIGH);
    digitalWrite(MOTOR_DIR, HIGH);
    analogWrite(MOTOR_PWM, 50);
}

void stopMotor() {
    analogWrite(MOTOR_PWM, 0);
    digitalWrite(MOTOR_ENABLE, LOW);
    digitalWrite(GLOBAL_ENABLE, LOW);
}

// --------- IMPULSSIDE MÕÕTMINE --------
int measurePulses(int pin) {
    pinMode(pin, INPUT_PULLUP);

    int last = digitalRead(pin);
    int changes = 0;
    unsigned long start = millis();
    unsigned long lastChangeTime = 0;

    while (millis() - start < 600) {  // pikem aken
        int v = digitalRead(pin);
        if (v != last) {
            if (millis() - lastChangeTime > 2) { // ignoreeri müra
                changes++;
                last = v;
                lastChangeTime = millis();
            }
        }
    }
    return changes;
}

void setup() {
    Console.begin(115200);
    delay(2000);

    // Power ON
    pinMode(PD1, OUTPUT);
    digitalWrite(PD1, HIGH); 

    Console.println("\n=== HALL SENSOR AUTO-SCAN START ===");
    startMotor();
    delay(1500); // lase mootoril ühtlustuda

    for (int i = 0; i < totalPinCount; i++) {
        int p = pinMap[i].pin;
        const char* name = pinMap[i].name;

        if (isExcluded(p)) {
            Console.print(name);
            Console.println(" ... SKIP");
            continue;
        }

        int cnt = measurePulses(p);

        if (cnt >= 8) {
            Console.print("FOUND SIGNAL on ");
            Console.print(name);
            Console.print("  changes=");
            Console.println(cnt);
        }
        else if (cnt >= 3) {
            Console.print("Possible signal on ");
            Console.print(name);
            Console.print("  changes=");
            Console.println(cnt);
        }
    }

    stopMotor();
    Console.println("=== SCAN FINISHED ===");
    Console.println("Power OFF");
    digitalWrite(PD1, LOW);

}

void loop() {
}
