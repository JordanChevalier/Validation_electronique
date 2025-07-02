#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <BluetoothSerial.h>

// === Pin Definitions ===
#define LED_VERTE 14    // Green LED for HMI
#define LED_ROUGE 15    // Red LED for HMI
#define TMP126_CS 5     // TMP126 chip select (SPI)

// === INA237 I²C Address and Registers ===
#define INA237_ADDR 0x40
#define INA237_REG_CONFIG 0x00
#define INA237_REG_VSHUNT 0x01
#define INA237_REG_VBUS 0x02
#define INA237_REG_CURRENT 0x04
#define INA237_REG_POWER 0x05
#define INA237_REG_CAL 0x06

// === Shunt Resistor ===
#define SHUNT_RESISTOR 0.01 // 10mΩ shunt resistor
#define CURRENT_LSB 0.000005 // 5µA/LSB

// === SPI Settings for TMP126 ===
#define SPI_CLK 1000000 // 1MHz SPI clock
#define TMP126_REG_TEMP 0x00 // Temperature register
#define TMP126_REG_CONFIG 0x01 // Configuration register
#define TMP126_REG_DEVICE_ID 0x0F // Device ID register

// === Bluetooth ===
BluetoothSerial SerialBT;

// === Function Prototypes ===
void initINA237();
bool initTMP126();
float readINA237Register(uint8_t reg);
float readTMP126Temperature();
void sendData(String data);

// === Setup ===
void setup() {
  // Initialize Serial
  Serial.begin(115200);
  Serial.println("System initializing...");

  // Initialize Bluetooth
  SerialBT.begin("ESP32_Sensor");
  Serial.println("Bluetooth initialized. Pair with 'ESP32_Sensor'.");
  SerialBT.println("Bluetooth initialized. Pair with 'ESP32_Sensor'.");

  // Initialize I²C (SDA=21, SCL=22)
  Wire.begin(21, 22);
  delay(100);

  // Initialize SPI (VSPI: SCLK=18, MOSI=23, MISO=19, CS=5)
  SPI.begin(18, 19, 23, TMP126_CS);
  pinMode(TMP126_CS, OUTPUT);
  digitalWrite(TMP126_CS, HIGH);

  // Initialize Sensors
  initINA237();
  if (!initTMP126()) {
    Serial.println("TMP126 initialization failed. Continuing with limited functionality.");
    SerialBT.println("TMP126 initialization failed. Continuing with limited functionality.");
  } else {
    Serial.println("TMP126 initialized.");
    SerialBT.println("TMP126 initialized.");
  }

  // Initialize LEDs
  pinMode(LED_VERTE, OUTPUT);
  pinMode(LED_ROUGE, OUTPUT);
  digitalWrite(LED_VERTE, LOW);
  digitalWrite(LED_ROUGE, LOW);

  Serial.println("System initialized.");
  SerialBT.println("System initialized.");
}

// === Initialize INA237 ===
void initINA237() {
  // Configure INA237: 12-bit, 1.1ms conversion, continuous mode, ±163.84mV shunt range
  Wire.beginTransmission(INA237_ADDR);
  Wire.write(INA237_REG_CONFIG);
  Wire.write(0x41); // AVG=1, VBUS_CT=1.1ms, VSH_CT=1.1ms, MODE=continuous
  Wire.write(0x9F); // Shunt range ±163.84mV
  if (Wire.endTransmission() != 0) {
    Serial.println("INA237 not detected!");
    SerialBT.println("INA237 not detected!");
    while (1);
  }

  // Calibrate INA237: Cal = 0.00512 / (Current_LSB * Rshunt)
  uint16_t cal = (uint16_t)(0.00512 / (CURRENT_LSB * SHUNT_RESISTOR));
  Wire.beginTransmission(INA237_ADDR);
  Wire.write(INA237_REG_CAL);
  Wire.write((cal >> 8) & 0xFF); // MSB
  Wire.write(cal & 0xFF);       // LSB
  if (Wire.endTransmission() != 0) {
    Serial.println("INA237 calibration failed!");
    SerialBT.println("INA237 calibration failed!");
    while (1);
  }

  Serial.println("INA237 initialized.");
  SerialBT.println("INA237 initialized.");
}

// === Initialize Каждый TMP126 ===
bool initTMP126() {
  // Check Device ID
  digitalWrite(TMP126_CS, LOW);
  delayMicroseconds(1); // t_CSS = 50ns min
  SPI.beginTransaction(SPISettings(SPI_CLK, MSBFIRST, SPI_MODE0));
  SPI.transfer(TMP126_REG_DEVICE_ID); // Select device ID register
  uint8_t msb = SPI.transfer(0x00);
  uint8_t lsb = SPI.transfer(0x00);
  SPI.endTransaction();
  digitalWrite(TMP126_CS, HIGH);
  delayMicroseconds(1); // t_CSH = 50ns min

  uint16_t device_id = (msb << 8) | lsb;
  Serial.println("TMP126 Device ID: 0x" + String(device_id, HEX));
  SerialBT.println("TMP126 Device ID: 0x" + String(device_id, HEX));
  if (device_id != 0x0126) {
    Serial.println("TMP126 not detected!");
    SerialBT.println("TMP126 not detected!");
    return false;
  }

  // Configure TMP126: Continuous conversion mode (MOD=11)
  digitalWrite(TMP126_CS, LOW);
  delayMicroseconds(1);
  SPI.beginTransaction(SPISettings(SPI_CLK, MSBFIRST, SPI_MODE0));
  SPI.transfer(0x80 | TMP126_REG_CONFIG); // Write to config register
  SPI.transfer(0x0C); // MOD[1:0]=11 (continuous), others default
  SPI.transfer(0x00); // Reserved
  SPI.endTransaction();
  digitalWrite(TMP126_CS, HIGH);
  delayMicroseconds(1);

  // Verify configuration
  digitalWrite(TMP126_CS, LOW);
  delayMicroseconds(1);
  SPI.beginTransaction(SPISettings(SPI_CLK, MSBFIRST, SPI_MODE0));
  SPI.transfer(TMP126_REG_CONFIG); // Read config
  msb = SPI.transfer(0x00);
  lsb = SPI.transfer(0x00);
  SPI.endTransaction();
  digitalWrite(TMP126_CS, HIGH);
  delayMicroseconds(1);

  Serial.println("TMP126 Config: 0x" + String(msb, HEX) + String(lsb, HEX));
  SerialBT.println("TMP126 Config: 0x" + String(msb, HEX) + String(lsb, HEX));
  if ((msb & 0x0C) != 0x0C) {
    return false;
  }
  return true;
}

// === Read INA237 Register ===
float readINA237Register(uint8_t reg) {
  Wire.beginTransmission(INA237_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return -9999.0; // I²C error
  }
  Wire.requestFrom(INA237_ADDR, 2);
  if (Wire.available() >= 2) {
    uint16_t value = (Wire.read() << 8) | Wire.read();
    if (reg == INA237_REG_VSHUNT || reg == INA237_REG_CURRENT) {
      return (int16_t)value; // Signed for shunt and current
    }
    return value;
  }
  return -9999.0; // Read error
}

// === Read TMP126 Temperature ===
float readTMP126Temperature() {
  digitalWrite(TMP126_CS, LOW);
  delayMicroseconds(1);
  SPI.beginTransaction(SPISettings(SPI_CLK, MSBFIRST, SPI_MODE0));
  SPI.transfer(TMP126_REG_TEMP); // Select temperature register
  uint8_t msb = SPI.transfer(0x00);
  uint8_t lsb = SPI.transfer(0x00);
  SPI.endTransaction();
  digitalWrite(TMP126_CS, HIGH);
  delayMicroseconds(1);

  uint16_t raw = (msb << 8) | lsb;
  Serial.println("TMP126 Raw: 0x" + String(raw, HEX));
  SerialBT.println("TMP126 Raw: 0x" + String(raw, HEX));
  raw >>= 2; // 14-bit data
  float temp_C = raw * 0.03125; // 0.03125°C/LSB
  if (temp_C < -55 || temp_C > 150) {
    return -9999.0; // Invalid reading
  }
  return temp_C;
}

// === Send Data ===
void sendData(String data) {
  Serial.println(data);
  SerialBT.println(data);
}

// === Main Loop ===
void loop() {
  // Turn on LEDs
  digitalWrite(LED_VERTE, HIGH);
  digitalWrite(LED_ROUGE, HIGH);
  String data = "[HMI] LED ON\t";

  // Read INA237
  float vbus = readINA237Register(INA237_REG_VBUS);
  float vshunt = readINA237Register(INA237_REG_VSHUNT);
  float current = readINA237Register(INA237_REG_CURRENT);
  float power = readINA237Register(INA237_REG_POWER);

  if (vbus != -9999.0 && vshunt != -9999.0 && current != -9999.0 && power != -9999.0) {
    float tension_V = vbus * 0.003125; // 3.125mV/LSB
    float courant_mA = current * 0.005; // 5µA/LSB
    float puissance_mW = power * 0.125; // 25 × Current_LSB
    data += "Tension = " + String(tension_V, 2) + " V\t";
    data += "Courant = " + String(courant_mA, 2) + " mA\t";
    data += "Puissance = " + String(puissance_mW, 2) + " mW\t";
  } else {
    data += "Erreur de lecture INA237\t";
  }

  // Read TMP126
  float temp_C = readTMP126Temperature();
  if (temp_C != -9999.0) {
    data += "Temp = " + String(temp_C, 2) + " °C";
  } else {
    data += "Erreur TMP126";
  }

  // Send data
  sendData(data);

  delay(2000);

  // Turn off LEDs
  digitalWrite(LED_VERTE, LOW);
  digitalWrite(LED_ROUGE, LOW);
  sendData("[HMI] LED OFF");

  delay(2000);
}
