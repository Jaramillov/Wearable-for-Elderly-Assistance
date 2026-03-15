#include <Wire.h>

// Dirección I2C del MPU6050 (normalmente 0x68)
const uint8_t MPU_ADDR = 0x68;

// Pines I2C en ESP32
const int I2C_SDA = 21;
const int I2C_SCL = 22;

// Variables para lecturas
int16_t ax, ay, az;  // Acelerómetro
int16_t gx, gy, gz;  // Giroscopio
int16_t tempRaw;     // Temperatura cruda

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Iniciar I2C en los pines de ESP32
  Wire.begin(I2C_SDA, I2C_SCL);

  // Despertar el MPU6050 (por defecto viene en sleep)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);      // Registro PWR_MGMT_1
  Wire.write(0);         // Poner a 0 para despertar
  Wire.endTransmission(true);

  Serial.println("MPU6050 iniciado. Leyendo datos...");
}

void loop() {
  leerMPU6050();
  imprimirDatos();

  delay(200); // 5 lecturas por segundo aprox.
}

void leerMPU6050() {
  // Empezar lectura desde el registro 0x3B (ACCEL_XOUT_H)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  // Leer 14 bytes: accel(6) + temp(2) + gyro(6)
  Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true);

  ax = (Wire.read() << 8) | Wire.read();  // ACCEL_XOUT
  ay = (Wire.read() << 8) | Wire.read();  // ACCEL_YOUT
  az = (Wire.read() << 8) | Wire.read();  // ACCEL_ZOUT

  tempRaw = (Wire.read() << 8) | Wire.read(); // TEMP_OUT

  gx = (Wire.read() << 8) | Wire.read();  // GYRO_XOUT
  gy = (Wire.read() << 8) | Wire.read();  // GYRO_YOUT
  gz = (Wire.read() << 8) | Wire.read();  // GYRO_ZOUT
}

void imprimirDatos() {
  // Convertir a unidades físicas aproximadas
  // Sensibilidad por defecto:
  //  Acelerómetro: 16384 LSB/g  (±2g)
  //  Giroscopio:   131 LSB/(°/s) (±250 °/s)

  float ax_g = ax / 16384.0;
  float ay_g = ay / 16384.0;
  float az_g = az / 16384.0;

  float gx_dps = gx / 131.0;
  float gy_dps = gy / 131.0;
  float gz_dps = gz / 131.0;

  // Temperatura (fórmula del datasheet)
  float tempC = (tempRaw / 340.0) + 36.53;

  Serial.println("======== MPU6050 ========");
  Serial.print("Accel [g]  -> X: "); Serial.print(ax_g, 3);
  Serial.print("  Y: ");             Serial.print(ay_g, 3);
  Serial.print("  Z: ");             Serial.println(az_g, 3);

  Serial.print("Gyro [°/s] -> X: "); Serial.print(gx_dps, 2);
  Serial.print("  Y: ");             Serial.print(gy_dps, 2);
  Serial.print("  Z: ");             Serial.println(gz_dps, 2);

  Serial.print("Temp: "); Serial.print(tempC, 2); Serial.println(" °C");
  Serial.println();
}
