#include <Wire.h>

const int MPU = 0x68; // MPU6050 I2C address

void setup() {
  Serial.begin(115200);
  Wire.begin(); // SDA = A4, SCL = A5 (default for Uno)

  // Wake up MPU6050 (it starts in sleep mode)
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0x00); // Set to 0 to wake up
  Wire.endTransmission(true);

  delay(100);
  Serial.println("MPU6050 Test Started");
  Serial.println("Accel(g) X | Y | Z || Gyro(°/s) X | Y | Z || Temp(°C)");
  Serial.println("------------------------------------------------------------");
}

void loop() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Starting register for accelerometer data
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true); // Request 14 bytes

  // Raw values
  int16_t AcX = (Wire.read() << 8) | Wire.read();
  int16_t AcY = (Wire.read() << 8) | Wire.read();
  int16_t AcZ = (Wire.read() << 8) | Wire.read();
  int16_t Tmp = (Wire.read() << 8) | Wire.read();
  int16_t GyX = (Wire.read() << 8) | Wire.read();
  int16_t GyY = (Wire.read() << 8) | Wire.read();
  int16_t GyZ = (Wire.read() << 8) | Wire.read();

  // Convert to real units
  float ax = AcX / 16384.0; // ±2g range → LSB/g = 16384
  float ay = AcY / 16384.0;
  float az = AcZ / 16384.0;
  float gx = GyX / 131.0;   // ±250°/s range → LSB/(°/s) = 131
  float gy = GyY / 131.0;
  float gz = GyZ / 131.0;
  float temp = (Tmp / 340.0) + 36.53; // Datasheet formula

  // Print
  Serial.print("Ac: ");
  Serial.print(ax, 2); Serial.print("g | ");
  Serial.print(ay, 2); Serial.print("g | ");
  Serial.print(az, 2); Serial.print("g  ||  Gy: ");
  Serial.print(gx, 1); Serial.print("°/s | ");
  Serial.print(gy, 1); Serial.print("°/s | ");
  Serial.print(gz, 1); Serial.print("°/s  ||  Temp: ");
  Serial.print(temp, 1); Serial.println("°C");

  delay(500);
}