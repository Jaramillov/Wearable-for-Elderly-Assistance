/*
 * SISTEMA UNIFICADO DE DETECCIÓN DE CAÍDAS Y ALERTAS
 * Integra: MPU6050 (Fuzzy Logic) + INMP441 (Voice Detection) + A7670SA (SMS)
 * LED RGB: Verde=OK, Amarillo=Riesgo, Rojo=Alerta/Ayuda + envío SMS
 */

#include <Wire.h>
#include <math.h>
#include <HardwareSerial.h>
#include <driver/i2s.h>
#include <voice_detection_with_INMP441_inferencing.h>

// =========================
// CONFIGURACIÓN MPU6050
// =========================
const uint8_t MPU_ADDR       = 0x68;
const float   ACC_LSB_PER_G  = 16384.0;
const float   GYRO_LSB_PER_DPS = 131.0;
const int I2C_SDA = 21;
const int I2C_SCL = 22;

// =========================
// CONFIGURACIÓN I2S (INMP441)
// =========================
#define I2S_WS 33
#define I2S_SD 32
#define I2S_SCK 25
#define I2S_PORT I2S_NUM_0
#define SAMPLE_RATE 16000
#define EI_BUFFER_SIZE EI_CLASSIFIER_RAW_SAMPLE_COUNT

// =========================
// CONFIGURACIÓN A7670SA
// =========================
const int RX_PIN = 16;
const int TX_PIN = 17;
const int PWRKEY_PIN = -1;
HardwareSerial gsmSerial(2);

// =========================
// LED RGB (ÁNODO COMÚN)
// =========================
#define LED_R 4
#define LED_G 2
#define LED_B 15

// =========================
// UMBRALES Y CONFIGURACIÓN
// =========================
const float FALL_ALERT_THRESHOLD = 0.7f;  // Caída confirmada
const float RISK_THRESHOLD = 0.4f;         // Riesgo de caída
const float VOICE_THRESHOLD = 0.60;        // Umbral "ayuda"
const unsigned long COOLDOWN_SMS = 30000;  // 30 segundos entre SMS

// =========================
// VARIABLES GLOBALES
// =========================
float offset_ax = 0, offset_ay = 0, offset_az = 0;
int16_t bufferI2S[EI_BUFFER_SIZE];
float bufferFloat[EI_BUFFER_SIZE];
unsigned long lastSMSTime = 0;
bool alertaActiva = false;

// Número de destino para SMS (CONFIGURABLE)
const String NUMERO_EMERGENCIA = "+5731034xxxxx";

// =========================
// FUNCIONES AUXILIARES
// =========================
float trimf(float x, float a, float b, float c) {
  if (x <= a || x >= c) return 0.0f;
  if (x == b) return 1.0f;
  if (x > a && x < b) return (x - a) / (b - a);
  return (c - x) / (c - b);
}

float mag3(float x, float y, float z) {
  return sqrtf(x*x + y*y + z*z);
}

void setLEDs(const char* color) {
  if (strcmp(color, "verde") == 0) {
    digitalWrite(LED_R, HIGH);
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_B, HIGH);
  } else if (strcmp(color, "amarillo") == 0) {
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_B, HIGH);
  } else if (strcmp(color, "rojo") == 0) {
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, HIGH);
    digitalWrite(LED_B, HIGH);
  }
}

// =========================
// MPU6050 - LECTURA Y CALIBRACIÓN
// =========================
int16_t read_raw16(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)2, (uint8_t)true);
  return (Wire.read() << 8) | Wire.read();
}

void read_mpu(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  ax = (float)read_raw16(0x3B) / ACC_LSB_PER_G;
  ay = (float)read_raw16(0x3D) / ACC_LSB_PER_G;
  az = (float)read_raw16(0x3F) / ACC_LSB_PER_G;
  gx = (float)read_raw16(0x43) / GYRO_LSB_PER_DPS;
  gy = (float)read_raw16(0x45) / GYRO_LSB_PER_DPS;
  gz = (float)read_raw16(0x47) / GYRO_LSB_PER_DPS;
}

void mpu_init() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  delay(50);
}

void calibrate_accelerometer(int samples = 50) {
  Serial.println("Calibrando acelerometro...");
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
  Serial.print(" ay="); Serial.print(offset_ay, 3);
  Serial.print(" az="); Serial.println(offset_az, 3);
}

// =========================
// LÓGICA DIFUSA
// =========================
void acc_memberships(float acc_mag, float &mu_bajo, float &mu_medio, float &mu_alto) {
  mu_bajo  = trimf(acc_mag, 0.0f, 0.4f, 0.9f);
  mu_medio = trimf(acc_mag, 0.7f, 1.0f, 1.6f);
  mu_alto  = trimf(acc_mag, 1.2f, 2.2f, 3.0f);
}

void gyro_memberships(float gyro_mag, float &mu_lento, float &mu_medio, float &mu_rapido) {
  mu_lento  = trimf(gyro_mag, 0.0f, 40.0f, 90.0f);
  mu_medio  = trimf(gyro_mag, 60.0f, 160.0f, 260.0f);
  mu_rapido = trimf(gyro_mag, 180.0f, 320.0f, 500.0f);
}

float fall_bajo_mf(float z) { return trimf(z, 0.0f, 0.2f, 0.5f); }
float fall_medio_mf(float z) { return trimf(z, 0.3f, 0.5f, 0.7f); }
float fall_alto_mf(float z) { return trimf(z, 0.6f, 0.85f, 1.0f); }

float fuzzy_fall_score(float acc_mag, float gyro_mag) {
  if (acc_mag < 0.0f) acc_mag = 0.0f;
  if (acc_mag > 3.4f) acc_mag = 3.4f;
  if (gyro_mag < 0.0f) gyro_mag = 0.0f;
  if (gyro_mag > 599.0f) gyro_mag = 599.0f;

  float acc_bajo, acc_medio, acc_alto;
  float gyro_lento, gyro_medio, gyro_rapido;

  acc_memberships(acc_mag, acc_bajo, acc_medio, acc_alto);
  gyro_memberships(gyro_mag, gyro_lento, gyro_medio, gyro_rapido);

  float r1 = fmin(acc_alto, gyro_rapido);
  float r2 = fmin(acc_alto, gyro_medio);
  float r3 = fmin(acc_medio, gyro_rapido);
  float r4 = acc_bajo;
  float r5 = fmin(gyro_lento, acc_medio);
  float r6 = fmin(acc_alto, gyro_lento);

  float lambda_bajo  = fmax(r4, r5);
  float lambda_medio = fmax(fmax(r2, r3), r6);
  float lambda_alto  = r1;

  float num = 0.0f;
  float den = 0.0f;

  for (float z = 0.0f; z <= 1.0f + 1e-5; z += 0.01f) {
    float mu_bajo_z  = fall_bajo_mf(z);
    float mu_medio_z = fall_medio_mf(z);
    float mu_alto_z  = fall_alto_mf(z);

    float mu_out = fmax(fmax(fmin(lambda_bajo, mu_bajo_z),
                             fmin(lambda_medio, mu_medio_z)),
                        fmin(lambda_alto, mu_alto_z));

    num += z * mu_out;
    den += mu_out;
  }

  if (den == 0.0f) return 0.0f;
  return num / den;
}

// =========================
// ENVÍO DE SMS
// =========================
void enviarSMS(String numero, String mensaje) {
  Serial.println("--- ENVIANDO SMS ---");
  
  gsmSerial.println("AT+CMGF=1");
  delay(200);
  
  gsmSerial.print("AT+CMGS=\"");
  gsmSerial.print(numero);
  gsmSerial.println("\"");
  delay(200);
  
  gsmSerial.print(mensaje);
  delay(200);
  
  gsmSerial.write(26);
  
  Serial.println("SMS enviado. Esperando confirmacion del modulo...");
  delay(2000);
}

void encenderModulo() {
  if (PWRKEY_PIN < 0) {
    Serial.println("PWRKEY deshabilitado. Enciende el modulo manualmente.");
    return;
  }

  Serial.println("Encendiendo modulo GSM...");
  digitalWrite(PWRKEY_PIN, HIGH);
  delay(500);
  digitalWrite(PWRKEY_PIN, LOW);
  delay(2500);
  digitalWrite(PWRKEY_PIN, HIGH);
  delay(6000);
  Serial.println("Modulo GSM listo.");
}

// =========================
// SETUP
// =========================
void setup() {
  Serial.begin(115200);
  delay(1000);

  // LED RGB
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  setLEDs("verde");

  // I2C y MPU6050
  Wire.begin(I2C_SDA, I2C_SCL);
  mpu_init();
  calibrate_accelerometer();

  // I2S y INMP441
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 32,
    .dma_buf_len = 64,
    .use_apll = false
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = -1,
    .data_in_num = I2S_SD
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
  i2s_zero_dma_buffer(I2S_PORT);

  // GSM A7670SA
  if (PWRKEY_PIN >= 0) {
    pinMode(PWRKEY_PIN, OUTPUT);
    digitalWrite(PWRKEY_PIN, HIGH);
  }

  gsmSerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  encenderModulo();

  Serial.println("=== SISTEMA UNIFICADO INICIADO ===");
  Serial.println("Escribe 'S' para probar envio de SMS");
}

// =========================
// LOOP PRINCIPAL
// =========================
void loop() {
  static unsigned long last_mpu_check = 0;
  const unsigned long MPU_INTERVAL = 500;
  unsigned long now = millis();

  // ========== VERIFICAR CAÍDA (MPU6050) ==========
  if (now - last_mpu_check >= MPU_INTERVAL) {
    last_mpu_check = now;

    float ax, ay, az, gx, gy, gz;
    read_mpu(ax, ay, az, gx, gy, gz);

    ax -= offset_ax;
    ay -= offset_ay;
    az -= offset_az;

    float a_mag = mag3(ax, ay, az);
    float g_mag = mag3(gx, gy, gz);
    float fall_score = fuzzy_fall_score(a_mag, g_mag);

    if (fall_score >= FALL_ALERT_THRESHOLD) {
      Serial.println("[ALERTA CAIDA DETECTADA]");
      setLEDs("rojo");
      alertaActiva = true;

      if (now - lastSMSTime > COOLDOWN_SMS) {
        enviarSMS(NUMERO_EMERGENCIA, "ALERTA: Caida detectada. Requiere asistencia inmediata.");
        lastSMSTime = now;
      }
    } else if (fall_score >= RISK_THRESHOLD) {
      Serial.println("[RIESGO DE CAIDA]");
      setLEDs("amarillo");
      alertaActiva = false;
    } else {
      if (!alertaActiva) {
        setLEDs("verde");
      }
    }

    Serial.print("Fall score: "); Serial.print(fall_score, 3);
    Serial.print(" | acc: "); Serial.print(a_mag, 2);
    Serial.print(" g | gyro: "); Serial.print(g_mag, 1);
    Serial.println(" dps");
  }

  // ========== DETECCIÓN DE VOZ (INMP441) ==========
  size_t bytes_read = 0;
  i2s_read(I2S_PORT, (void *)bufferI2S, EI_BUFFER_SIZE * sizeof(int16_t), &bytes_read, 0);

  if (bytes_read == EI_BUFFER_SIZE * sizeof(int16_t)) {
    for (int i = 0; i < EI_BUFFER_SIZE; i++) {
      bufferFloat[i] = (float)bufferI2S[i] / 32768.0f;
    }

    signal_t signal;
    ei::numpy::signal_from_buffer(bufferFloat, EI_BUFFER_SIZE, &signal);

    ei_impulse_result_t result;
    if (run_classifier(&signal, &result, false) == EI_IMPULSE_OK) {
      float ayuda = 0;

      for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        String label = result.classification[i].label;
        if (label == "ayuda") {
          ayuda = result.classification[i].value;
        }
      }

      if (ayuda >= VOICE_THRESHOLD) {
        Serial.println("[LLAMADO DE AYUDA DETECTADO]");
        setLEDs("rojo");
        alertaActiva = true;

        if (now - lastSMSTime > COOLDOWN_SMS) {
          enviarSMS(NUMERO_EMERGENCIA, "ALERTA: Llamado de ayuda detectado. Requiere asistencia inmediata.");
          lastSMSTime = now;
        }
      }
    }
  }

  // ========== COMANDOS SERIAL (Prueba SMS y Módulo GSM) ==========
  while (gsmSerial.available()) {
    Serial.write(gsmSerial.read());
  }

  while (Serial.available()) {
    char c = Serial.read();
    
    if (c == 'S' || c == 's') {
      enviarSMS(NUMERO_EMERGENCIA, "Prueba: El sistema esta funcionando correctamente.");
    } else {
      gsmSerial.write(c);
    }
  }
}