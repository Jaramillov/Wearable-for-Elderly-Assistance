#include <Wire.h>
#include <math.h>

// =========================
// Configuración MPU6050
// =========================
const uint8_t MPU_ADDR       = 0x68;
const float   ACC_LSB_PER_G  = 16384.0;  // ±2g
const float   GYRO_LSB_PER_DPS = 131.0;  // ±250 °/s

// Pines I2C en ESP32
const int I2C_SDA = 21;
const int I2C_SCL = 22;

// =========================
// Configuración Fuzzy & Lógica de caída
// =========================
const float FALL_ALERT_THRESHOLD = 0.7f;  // rojo
const float RISK_THRESHOLD        = 0.4f;  // amarillo

// Para defuzzificación por centroide
const float FALL_Z_MIN = 0.0f;
const float FALL_Z_MAX = 1.0f;
const float FALL_Z_STEP = 0.01f;  // 0..1 con paso 0.01

// =========================
// LED RGB ÁNODO COMÚN
// =========================
#define LED_R 4      // Rojo
#define LED_G 2      // Verde
#define LED_B 15     // Azul

// Offsets de calibración del acelerómetro
float offset_ax = 0, offset_ay = 0, offset_az = 0;

// =========================
// Utilidades genéricas
// =========================
float trimf(float x, float a, float b, float c) {
  if (x <= a || x >= c) return 0.0f;
  if (x == b)          return 1.0f;
  if (x > a && x < b)  return (x - a) / (b - a);
  // x > b && x < c
  return (c - x) / (c - b);
}

float mag3(float x, float y, float z) {
  return sqrtf(x*x + y*y + z*z);
}

// Para LED RGB de ÁNODO COMÚN: LOW=encendido, HIGH=apagado
void setLEDs(const char* color) {
  if (strcmp(color, "verde") == 0) {
    digitalWrite(LED_R, HIGH);  // Rojo apagado
    digitalWrite(LED_G, LOW);   // Verde encendido
    digitalWrite(LED_B, HIGH);  // Azul apagado
  } else if (strcmp(color, "amarillo") == 0) {
    digitalWrite(LED_R, LOW);   // Rojo encendido
    digitalWrite(LED_G, LOW);   // Verde encendido
    digitalWrite(LED_B, HIGH);  // Azul apagado
  } else if (strcmp(color, "rojo") == 0) {
    digitalWrite(LED_R, LOW);   // Rojo encendido
    digitalWrite(LED_G, HIGH);  // Verde apagado
    digitalWrite(LED_B, HIGH);  // Azul apagado
  }
}

// =========================
// Lectura MPU6050
// =========================
int16_t read_raw16(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)2, (uint8_t)true);

  int16_t value = (Wire.read() << 8) | Wire.read();
  return value;
}

void read_mpu(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  // Acelerómetro
  int16_t raw_ax = read_raw16(0x3B);
  int16_t raw_ay = read_raw16(0x3D);
  int16_t raw_az = read_raw16(0x3F);

  // Giroscopio
  int16_t raw_gx = read_raw16(0x43);
  int16_t raw_gy = read_raw16(0x45);
  int16_t raw_gz = read_raw16(0x47);

  ax = (float)raw_ax / ACC_LSB_PER_G;
  ay = (float)raw_ay / ACC_LSB_PER_G;
  az = (float)raw_az / ACC_LSB_PER_G;

  gx = (float)raw_gx / GYRO_LSB_PER_DPS;
  gy = (float)raw_gy / GYRO_LSB_PER_DPS;
  gz = (float)raw_gz / GYRO_LSB_PER_DPS;
}

void mpu_init() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1
  Wire.write(0x00); // Quitar sleep
  Wire.endTransmission(true);

  // Acelerómetro ±2g
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); // ACCEL_CONFIG
  Wire.write(0x00);
  Wire.endTransmission(true);

  // Giroscopio ±250 °/s
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); // GYRO_CONFIG
  Wire.write(0x00);
  Wire.endTransmission(true);

  delay(50);
}

// =========================
// Lógica difusa (misma que fylogic.py)
// =========================
// Memberships para aceleración (g)
void acc_memberships(float acc_mag,
                     float &mu_bajo,
                     float &mu_medio,
                     float &mu_alto) {
  mu_bajo  = trimf(acc_mag, 0.0f, 0.4f, 0.9f);
  mu_medio = trimf(acc_mag, 0.7f, 1.0f, 1.6f);
  mu_alto  = trimf(acc_mag, 1.2f, 2.2f, 3.0f);
}

// Memberships para giro (°/s)
void gyro_memberships(float gyro_mag,
                      float &mu_lento,
                      float &mu_medio,
                      float &mu_rapido) {
  mu_lento  = trimf(gyro_mag, 0.0f, 40.0f, 90.0f);
  mu_medio  = trimf(gyro_mag, 60.0f, 160.0f, 260.0f);
  mu_rapido = trimf(gyro_mag, 180.0f, 320.0f, 500.0f);
}

// Memberships para salida "fall" (0..1)
float fall_bajo_mf(float z) {
  return trimf(z, 0.0f, 0.2f, 0.5f);
}

float fall_medio_mf(float z) {
  return trimf(z, 0.3f, 0.5f, 0.7f);
}

float fall_alto_mf(float z) {
  return trimf(z, 0.6f, 0.85f, 1.0f);
}

// Implementación equivalente a fuzzy_fall_score de Python
float fuzzy_fall_score(float acc_mag, float gyro_mag) {
  // clamp como en Python
  if (acc_mag  < 0.0f)  acc_mag  = 0.0f;
  if (acc_mag  > 3.4f)  acc_mag  = 3.4f;
  if (gyro_mag < 0.0f)  gyro_mag = 0.0f;
  if (gyro_mag > 599.0f) gyro_mag = 599.0f;

  // 1) fuzzificación
  float acc_bajo, acc_medio, acc_alto;
  float gyro_lento, gyro_medio, gyro_rapido;

  acc_memberships(acc_mag, acc_bajo, acc_medio, acc_alto);
  gyro_memberships(gyro_mag, gyro_lento, gyro_medio, gyro_rapido);

  // 2) reglas (Mamdani)
  float r1 = fmin(acc_alto, gyro_rapido);
  float r2 = fmin(acc_alto, gyro_medio);
  float r3 = fmin(acc_medio, gyro_rapido);
  float r4 = acc_bajo;
  float r5 = fmin(gyro_lento, acc_medio);
  float r6 = fmin(acc_alto, gyro_lento);

  // Activación por etiqueta de salida
  float lambda_bajo  = fmax(r4, r5);
  float lambda_medio = fmax(fmax(r2, r3), r6);
  float lambda_alto  = r1;

  // 3) agregación + 4) defuzzificación (centroide)
  float num = 0.0f;
  float den = 0.0f;

  for (float z = FALL_Z_MIN; z <= FALL_Z_MAX + 1e-5; z += FALL_Z_STEP) {
    float mu_bajo_z  = fall_bajo_mf(z);
    float mu_medio_z = fall_medio_mf(z);
    float mu_alto_z  = fall_alto_mf(z);

    float mu_out = fmax(
                    fmax(fmin(lambda_bajo,  mu_bajo_z),
                         fmin(lambda_medio, mu_medio_z)),
                    fmin(lambda_alto,  mu_alto_z)
                   );

    num += z * mu_out;
    den += mu_out;
  }

  if (den == 0.0f) return 0.0f;
  return num / den;
}

// =========================
// Calibración acelerómetro
// =========================
void calibrate_accelerometer(int samples = 50) {
  Serial.println("Calibrando acelerometro, mantener quieto el sensor...");
  float sum_ax = 0, sum_ay = 0, sum_az = 0;

  for (int i = 0; i < samples; i++) {
    float ax, ay, az, gx, gy, gz;
    read_mpu(ax, ay, az, gx, gy, gz);

    sum_ax += ax;
    sum_ay += ay;
    sum_az += az;

    delay(20);
  }

  offset_ax = sum_ax / samples;
  offset_ay = sum_ay / samples;
  offset_az = sum_az / samples;

  Serial.print("Offsets: ax="); Serial.print(offset_ax, 3);
  Serial.print(" ay=");        Serial.print(offset_ay, 3);
  Serial.print(" az=");        Serial.println(offset_az, 3);
}

// =========================
// setup / loop
// =========================
void setup() {
  Serial.begin(115200);
  delay(1000);

  // I2C
  Wire.begin(I2C_SDA, I2C_SCL);

  // Configurar pines del LED RGB como salida
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  
  // Inicializar en verde (para ánodo común: R=HIGH, G=LOW, B=HIGH)
  setLEDs("verde");

  // Iniciar MPU
  mpu_init();
  calibrate_accelerometer();
}

void loop() {
  static unsigned long last_ms = 0;
  const unsigned long DT_MS = 500; // 2 Hz

  unsigned long now = millis();
  if (now - last_ms < DT_MS) {
    return;
  }
  last_ms = now;

  float ax, ay, az, gx, gy, gz;
  read_mpu(ax, ay, az, gx, gy, gz);

  // Aplicar offsets de acelerómetro
  ax -= offset_ax;
  ay -= offset_ay;
  az -= offset_az;

  float a_mag = mag3(ax, ay, az);
  float g_mag = mag3(gx, gy, gz);

  float fall_score = fuzzy_fall_score(a_mag, g_mag);

  // --------------------------
  // Lógica de decisión (LEDs)
  // --------------------------
  if (fall_score >= FALL_ALERT_THRESHOLD) {
    setLEDs("rojo");
    Serial.print("[ALERTA CAIDA] ");
  } else if (fall_score >= RISK_THRESHOLD) {
    setLEDs("amarillo");
    Serial.print("[RIESGO] ");
  } else {
    setLEDs("verde");
    Serial.print("[OK] ");
  }

  // Enviar datos por Serial para debug o log
  Serial.print("score=");
  Serial.print(fall_score, 3);
  Serial.print(" | acc_mag=");
  Serial.print(a_mag, 2);
  Serial.print(" g | gyro_mag=");
  Serial.print(g_mag, 1);
  Serial.println(" dps");
}