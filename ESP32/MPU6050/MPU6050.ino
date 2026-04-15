#include <Wire.h>

const uint8_t MPU = 0x68;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin(4, 5); // SDA = GPIO4, SCL = GPIO5
  Wire.setClock(100000); // 100kHz standard mode

  // Check WHO_AM_I
  Wire.beginTransmission(MPU);
  Wire.write(0x75);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, (size_t)1, true);
  uint8_t whoami = Wire.read();
  Serial.print("WHO_AM_I: 0x");
  Serial.println(whoami, HEX); // Must be 0x68

  // Reset
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x80);
  Wire.endTransmission(true);
  delay(200);

  // Wake up
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
  delay(200);

  Serial.println("MPU6050 Ready");
  Serial.println("Accel(g) X | Y | Z || Gyro(°/s) X | Y | Z || Temp(°C)");
  Serial.println("------------------------------------------------------------");
}

void loop() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, (size_t)14, true);

  int16_t AcX = (Wire.read() << 8) | Wire.read();
  int16_t AcY = (Wire.read() << 8) | Wire.read();
  int16_t AcZ = (Wire.read() << 8) | Wire.read();
  int16_t Tmp = (Wire.read() << 8) | Wire.read();
  int16_t GyX = (Wire.read() << 8) | Wire.read();
  int16_t GyY = (Wire.read() << 8) | Wire.read();
  int16_t GyZ = (Wire.read() << 8) | Wire.read();

  float ax = AcX / 16384.0;
  float ay = AcY / 16384.0;
  float az = AcZ / 16384.0;
  float gx = GyX / 131.0;
  float gy = GyY / 131.0;
  float gz = GyZ / 131.0;
  float temp = (Tmp / 340.0) + 36.53;

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