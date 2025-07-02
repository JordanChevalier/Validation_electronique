#include <Arduino.h>
#include <Wire.h>
#include <BluetoothSerial.h>
#include <math.h>

// === Pin Definitions ===
#define LED_VERTE 14    // CMD_LED_GREEN
#define LED_ROUGE 15    // CMD_LED_RED
#define BUZZER 13       // Buzzer-PWM
#define NTC1_PIN 26     // NTC 1 analog input
#define NTC2_PIN 25     // NTC 2 analog input

// === PWM for Buzzer ===
#define BUZZER_CHANNEL 0
#define PWM_FREQUENCY 2000
#define PWM_RESOLUTION 8

// === INA237 I²C Address and Registers ===
#define INA237_ADDR 0x40
#define INA237_REG_CONFIG 0x00
#define INA237_REG_VSHUNT 0x01
#define INA237_REG_VBUS 0x02
#define INA237_REG_CURRENT 0x04
#define INA237_REG_POWER 0x05

// === NTC Parameters ===
#define R1 10000.0      // 10kΩ series resistor
#define R0 10000.0      // NTC resistance at 25°C
#define BETA 3950.0     // NTC beta coefficient
#define T0 298.15       // Reference temperature (25°C in Kelvin)

// === Bluetooth ===
BluetoothSerial SerialBT;

void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);
  
  // Initialize Bluetooth
  SerialBT.begin("ESP32_Sensor"); // Bluetooth device name
  Serial.println("Bluetooth started. Pair with 'ESP32_Sensor'.");
  SerialBT.println("Bluetooth started. Pair with 'ESP32_Sensor'.");

  // Initialize I²C (SDA = IO21, SCL = IO22)
  Wire.begin(21, 22);
  delay(100);

  // Initialize INA237
  Wire.beginTransmission(INA237_ADDR);
  Wire.write(INA237_REG_CONFIG);
  Wire.write(0x4C); // 12-bit, 1.1ms conversion, continuous mode
  Wire.write(0x1F); // ±163.84mV shunt range
  if (Wire.endTransmission() != 0) {
    Serial.println("INA237 not detected!");
    SerialBT.println("INA237 not detected!");
    while (1);
  }
  Serial.println("INA237 detected.");
  SerialBT.println("INA237 detected.");

  // Initialize LEDs
  pinMode(LED_VERTE, OUTPUT);
  pinMode(LED_ROUGE, OUTPUT);

  // Initialize PWM for buzzer
  ledcSetup(BUZZER_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(BUZZER, BUZZER_CHANNEL);
  ledcWrite(BUZZER_CHANNEL, 0); // Silence buzzer

  // Turn off LEDs at startup
  digitalWrite(LED_VERTE, LOW);
  digitalWrite(LED_ROUGE, LOW);

  // Configure ADC
  analogReadResolution(12); // 12-bit resolution (0–4095)
}

float readINA237Register(uint8_t reg) {
  Wire.beginTransmission(INA237_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(INA237_ADDR, 2);
  if (Wire.available() >= 2) {
    uint16_t value = (Wire.read() << 8) | Wire.read();
    if (reg == INA237_REG_VSHUNT || reg == INA237_REG_CURRENT) {
      return (int16_t)value; // Signed for shunt and current
    }
    return value;
  }
  return -9999.0; // Error
}

float readNTCTemperature(int pin) {
  int analogValue = analogRead(pin);
  float voltage = (analogValue / 4095.0) * 3.3; // 3.3V reference
  if (voltage < 0.01 || voltage > 3.29) return -9999.0; // Invalid reading
  float rNTC = R1 * (3.3 / voltage - 1); // NTC resistance
  float ln = log(rNTC / R0);
  float tempK = 1.0 / (1.0 / T0 + ln / BETA); // Steinhart-Hart
  return tempK - 273.15; // Convert to °C
}

void loop() {
  // Turn on LEDs and buzzer
  digitalWrite(LED_VERTE, HIGH);
  digitalWrite(LED_ROUGE, HIGH);
  ledcWrite(BUZZER_CHANNEL, 128); // Buzzer at 50% PWM

  // Read INA237 data
  String data = "[IHM] LED et buzzer ON\t";
  float vbus = readINA237Register(INA237_REG_VBUS);
  float vshunt = readINA237Register(INA237_REG_VSHUNT);
  float current = readINA237Register(INA237_REG_CURRENT);
  float power = readINA237Register(INA237_REG_POWER);

  if (vbus != -9999.0 && vshunt != -9999.0 && current != -9999.0 && power != -9999.0) {
    float tension_V = vbus * 0.0001; // 100µV/LSB
    float courant_mA = current * 0.005; // 5µA/LSB for 0.01Ω shunt
    float puissance_mW = power * 0.125; // 125µW/LSB
    data += "Tension = " + String(tension_V, 2) + " V\t";
    data += "Courant = " + String(courant_mA, 2) + " mA\t";
    data += "Puissance = " + String(puissance_mW, 2) + " mW\t";
  } else {
    data += "Erreur de lecture INA237\t";
  }

  // Read NTC temperatures
  float temp1_C = readNTCTemperature(NTC1_PIN);
  float temp2_C = readNTCTemperature(NTC2_PIN);
  if (temp1_C > -50 && temp1_C < 150) {
    data += "Temp NTC1 = " + String(temp1_C, 2) + " °C\t";
  } else {
    data += "Erreur NTC1\t";
  }
  if (temp2_C > -50 && temp2_C < 150) {
    data += "Temp NTC2 = " + String(temp2_C, 2) + " °C";
  } else {
    data += "Erreur NTC2";
  }

  // Print to Serial and Bluetooth
  Serial.println(data);
  SerialBT.println(data);

  delay(2000);

  // Turn off LEDs and buzzer
  digitalWrite(LED_VERTE, LOW);
  digitalWrite(LED_ROUGE, LOW);
  ledcWrite(BUZZER_CHANNEL, 0);
  Serial.println("[IHM] LED et buzzer OFF\n");
  SerialBT.println("[IHM] LED et buzzer OFF\n");

  delay(2000);
}
