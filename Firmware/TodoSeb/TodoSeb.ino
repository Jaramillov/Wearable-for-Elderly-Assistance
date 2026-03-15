/*
 * SISTEMA COMPLETO DE SEGURIDAD INTELIGENTE
 * Integra: MPU6050 + GPS NEO-6M + INMP441 (Voice) + A7670SA (SMS)
 * LED RGB + Buzzer para alertas jerárquicas
 * 
 * PRIORIDADES:
 * 1. CAÍDA (Latch 10s)    -> ROJO FIJO + Buzzer Continuo + SMS
 * 2. LLAMADO AYUDA        -> ROJO FIJO + Buzzer Rápido + SMS
 * 3. RIESGO CAÍDA         -> AMARILLO PARPADEO + Beep Intermitente
 * 4. FUERA DE ZONA        -> ROJO PARPADEO + Alarma Lenta + SMS
 * 5. BUSCANDO GPS         -> AZUL PARPADEO
 * 6. NORMAL               -> VERDE FIJO
 */

#include <Wire.h>
#include <math.h>
#include <HardwareSerial.h>
#include <driver/i2s.h>
#include <voice_detection_with_INMP441_inferencing.h>
#include <TinyGPS++.h>

// =========================
// CONFIGURACIÓN MPU6050
// =========================
const uint8_t MPU_ADDR = 0x68;
const float ACC_LSB_PER_G = 16384.0;
const float GYRO_LSB_PER_DPS = 131.0;
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
// CONFIGURACIÓN GPS NEO-6M
// =========================
const int GPS_RX = 18;
const int GPS_TX = 19;
const long GPS_BAUD = 9600;
const double SAFE_RADIUS_M = 15.0;

// =========================
// CONFIGURACIÓN A7670SA
// =========================
const int MODEM_RX = 16;
const int MODEM_TX = 17;
const long MODEM_BAUD = 115200;

// =========================
// LED RGB (ÁNODO COMÚN) + BUZZER
// =========================
#define LED_R 4
#define LED_G 2
#define LED_B 15
#define PIN_BUZZER 26

// =========================
// UMBRALES Y CONFIGURACIÓN
// =========================
const float FALL_ALERT_THRESHOLD = 0.7f;
const float RISK_THRESHOLD = 0.4f;
const float VOICE_THRESHOLD = 0.60;
const unsigned long COOLDOWN_SMS = 30000;  // 30 segundos entre SMS
const unsigned long ALERT_DURATION = 10000; // 10 segundos alerta caída
const unsigned long VOICE_ALERT_DURATION = 5000; // 5 segundos alerta voz

// Número de destino para SMS
const char NUMERO_EMERGENCIA[] = "+5731034xxxxx";

// =========================
// OBJETOS GLOBALES
// =========================
TinyGPSPlus gps;
HardwareSerial modemSerial(1);
HardwareSerial gpsSerial(2);

// =========================
// VARIABLES GLOBALES
// =========================
// MPU
float offset_ax = 0, offset_ay = 0, offset_az = 0;

// Audio
int16_t bufferI2S[EI_BUFFER_SIZE];
float bufferFloat[EI_BUFFER_SIZE];

// GPS
double safeLat = 0.0, safeLon = 0.0;
bool homeSet = false;
bool isOutsideZone = false;
bool geoAlertSent = false;
unsigned long lastGpsPrint = 0;

// SMS
unsigned long lastSMSTime = 0;

// Estados y Alertas
enum SystemState { 
  SYS_NORMAL, 
  SYS_GPS_SEARCH, 
  SYS_OUTSIDE, 
  SYS_RISK, 
  SYS_FALL,
  SYS_VOICE_ALERT 
};

SystemState currentState = SYS_GPS_SEARCH;
unsigned long alertStartTime = 0;
unsigned long blinkMillis = 0;
bool blinkState = false;

// =========================
// FUNCIONES LED Y BUZZER
// =========================
void setRGB(bool r, bool g, bool b) {
  digitalWrite(LED_R, r ? LOW : HIGH);
  digitalWrite(LED_G, g ? LOW : HIGH);
  digitalWrite(LED_B, b ? LOW : HIGH);
}

void buzzerOn()  { digitalWrite(PIN_BUZZER, HIGH); }
void buzzerOff() { digitalWrite(PIN_BUZZER, LOW); }

// =========================
// MPU6050 - FUNCIONES
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

bool mpu_init() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) return false;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission();

  delay(50);
  return true;
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

float mag3(float x, float y, float z) {
  return sqrtf(x*x + y*y + z*z);
}

float trimf(float x, float a, float b, float c) {
  if (x <= a || x >= c) return 0.0f;
  if (x == b) return 1.0f;
  if (x > a && x < b) return (x - a) / (b - a);
  return (c - x) / (c - b);
}

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
// GPS - FUNCIONES
// =========================
void imprimirDatosGPS() {
  if (millis() - lastGpsPrint < 2000) return;
  lastGpsPrint = millis();

  Serial.print("📡 GPS: ");
  if (gps.location.isValid()) {
    Serial.print("Lat: "); Serial.print(gps.location.lat(), 6);
    Serial.print(" | Lon: "); Serial.print(gps.location.lng(), 6);
    Serial.print(" | Sats: "); Serial.print(gps.satellites.value());
    
    if (homeSet) {
      double dist = TinyGPSPlus::distanceBetween(
        gps.location.lat(), gps.location.lng(), 
        safeLat, safeLon
      );
      Serial.print(" | Dist Home: "); Serial.print(dist, 1); Serial.print("m");
      if (dist > SAFE_RADIUS_M) Serial.print(" [FUERA]");
      else Serial.print(" [DENTRO]");
    } else {
      Serial.print(" | Home: NO CONFIG (>4 sats req)");
    }
  } else {
    Serial.print("Buscando señal... Sats: ");
    if (gps.satellites.isValid()) Serial.print(gps.satellites.value());
    else Serial.print("0");
  }
  Serial.println();
}

void actualizarGPS() {
  if (!gps.location.isValid()) return;

  if (!homeSet && gps.satellites.value() > 4) {
    safeLat = gps.location.lat();
    safeLon = gps.location.lng();
    homeSet = true;
    Serial.println(F("🏠 HOME CONFIGURADO"));
  }

  if (homeSet) {
    double dist = TinyGPSPlus::distanceBetween(
      gps.location.lat(), gps.location.lng(), 
      safeLat, safeLon
    );
    
    if (dist > SAFE_RADIUS_M) {
      isOutsideZone = true;
      if (!geoAlertSent) {
        String msg = "ALERTA: SALIO DE ZONA SEGURA. Dist: " + String(dist, 1) + "m";
        enviarSMS(msg, true);
        geoAlertSent = true;
      }
    } else {
      isOutsideZone = false;
      geoAlertSent = false;
    }
  }
}

// =========================
// SMS - FUNCIONES
// =========================
void enviarComandoModem(const char* cmd, uint16_t esperaMs = 500) {
  modemSerial.println(cmd);
  unsigned long start = millis();
  while(millis() - start < esperaMs) {
    while(modemSerial.available()) Serial.write(modemSerial.read());
  }
}

void inicializarSMS() {
  Serial.println(F("Configurando Modem..."));
  enviarComandoModem("AT");
  enviarComandoModem("ATE0");
  enviarComandoModem("AT+CMGF=1");
  enviarComandoModem("AT+CSCS=\"GSM\"");
}

void enviarSMS(String mensaje, bool incluirGPS) {
  Serial.println(F("📩 Enviando SMS..."));
  modemSerial.println("AT+CMGF=1"); 
  delay(200);
  
  modemSerial.print("AT+CMGS=\"");
  modemSerial.print(NUMERO_EMERGENCIA);
  modemSerial.println("\"");
  delay(200);

  modemSerial.println(mensaje);
  
  if (incluirGPS && gps.location.isValid()) {
    String url = "http://maps.google.com/?q=";
    url += String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
    modemSerial.print("Ubicacion: ");
    modemSerial.println(url);
  } else if (incluirGPS) {
    modemSerial.println("Ubicacion: GPS sin señal.");
  }
  
  modemSerial.write(26);
  Serial.println(F("✅ SMS enviado"));
}

// =========================
// GESTIÓN DE ESTADOS
// =========================
void gestionarIndicadores() {
  unsigned long now = millis();
  
  // Determinar estado actual por jerarquía
  SystemState newState = SYS_NORMAL;
  
  // Prioridad 1: Caída (con latch de tiempo)
  if (currentState == SYS_FALL) {
    if (now - alertStartTime < ALERT_DURATION) {
      newState = SYS_FALL;
    } else {
      Serial.println("[ALERTA CAIDA FINALIZADA]");
    }
  }
  
  // Prioridad 2: Voz (con latch de tiempo)
  if (currentState == SYS_VOICE_ALERT && newState == SYS_NORMAL) {
    if (now - alertStartTime < VOICE_ALERT_DURATION) {
      newState = SYS_VOICE_ALERT;
    } else {
      Serial.println("[ALERTA VOZ FINALIZADA]");
    }
  }
  
  // Prioridad 3-6: Otros estados (sin latch, evalúan en tiempo real)
  if (newState == SYS_NORMAL) {
    if (!homeSet || !gps.location.isValid()) {
      newState = SYS_GPS_SEARCH;
    } else if (homeSet && isOutsideZone) {
      newState = SYS_OUTSIDE;
    }
    // RISK se maneja en el loop MPU
  }
  
  currentState = newState;
  
  // Aplicar indicadores según estado
  switch (currentState) {
    case SYS_FALL:
      setRGB(true, false, false); // Rojo fijo
      if ((now / 150) % 2 == 0) buzzerOn();
      else buzzerOff();
      break;
      
    case SYS_VOICE_ALERT:
      setRGB(true, false, false); // Rojo fijo
      if ((now / 100) % 2 == 0) buzzerOn(); // Beep más rápido
      else buzzerOff();
      break;
      
    case SYS_RISK:
      if (now - blinkMillis > 200) {
        blinkMillis = now;
        blinkState = !blinkState;
        if (blinkState) {
          setRGB(true, true, false); // Amarillo
          buzzerOn();
        } else {
          setRGB(false, false, false);
          buzzerOff();
        }
      }
      break;
      
    case SYS_OUTSIDE:
      if (now - blinkMillis > 500) {
        blinkMillis = now;
        blinkState = !blinkState;
        if (blinkState) {
          setRGB(true, false, false); // Rojo parpadeo
          buzzerOn();
        } else {
          setRGB(false, false, false);
          buzzerOff();
        }
      }
      break;
      
    case SYS_GPS_SEARCH:
      if (now - blinkMillis > 1000) {
        blinkMillis = now;
        blinkState = !blinkState;
        if (blinkState) setRGB(false, false, true); // Azul
        else setRGB(false, false, false);
      }
      buzzerOff();
      break;
      
    case SYS_NORMAL:
      setRGB(false, true, false); // Verde fijo
      buzzerOff();
      break;
  }
}

// =========================
// SETUP
// =========================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("=== SISTEMA COMPLETO DE SEGURIDAD ==="));

  // LED RGB y Buzzer
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  setRGB(false, false, true); // Azul inicio
  buzzerOff();

  // I2C y MPU6050
  Wire.begin(I2C_SDA, I2C_SCL);
  if (mpu_init()) {
    Serial.println("✅ MPU6050 OK");
    calibrate_accelerometer();
  } else {
    Serial.println("❌ MPU6050 FALLO");
  }

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
  Serial.println("✅ INMP441 OK");

  // GPS NEO-6M
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("✅ GPS NEO-6M OK");

  // Módem A7670SA
  modemSerial.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
  inicializarSMS();
  Serial.println("✅ MODEM A7670SA OK");

  Serial.println("=== SISTEMA INICIADO ===");
  Serial.println("Escribe 'S' para probar SMS");
}

// =========================
// LOOP PRINCIPAL
// =========================
void loop() {
  static unsigned long last_mpu_check = 0;
  const unsigned long MPU_INTERVAL = 500;
  unsigned long now = millis();

  // ========== LEER GPS ==========
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
  actualizarGPS();
  imprimirDatosGPS();

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
      Serial.println("[🚨 CAIDA DETECTADA]");
      
      if (currentState != SYS_FALL) {
        currentState = SYS_FALL;
        alertStartTime = now;

        if (now - lastSMSTime > COOLDOWN_SMS) {
          enviarSMS("ALERTA: Caida detectada. Requiere asistencia inmediata.", true);
          lastSMSTime = now;
        }
      }
    } else if (fall_score >= RISK_THRESHOLD) {
      if (currentState != SYS_FALL && currentState != SYS_VOICE_ALERT) {
        Serial.println("[⚠️ RIESGO DE CAIDA]");
        currentState = SYS_RISK;
      }
    }

    Serial.print("Fall: "); Serial.print(fall_score, 3);
    Serial.print(" | acc: "); Serial.print(a_mag, 2);
    Serial.print("g | gyro: "); Serial.print(g_mag, 1);
    Serial.println("dps");
  }

  // ========== DETECCIÓN DE VOZ (INMP441) ==========
  size_t bytes_read = 0;
  i2s_read(I2S_PORT, (void *)bufferI2S, EI_BUFFER_SIZE * sizeof(int16_t), 
           &bytes_read, portMAX_DELAY);

  int samples_read = bytes_read / sizeof(int16_t);
  if (samples_read == EI_BUFFER_SIZE) {
    for (int i = 0; i < EI_BUFFER_SIZE; i++) {
      bufferFloat[i] = (float)bufferI2S[i] / 32768.0f;
    }

    signal_t signal;
    ei::numpy::signal_from_buffer(bufferFloat, EI_BUFFER_SIZE, &signal);

    ei_impulse_result_t result;
    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);
    
    if (err == EI_IMPULSE_OK) {
      float ayuda = 0;
      float noise = 0;
      float unknown = 0;

      for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        String label = result.classification[i].label;
        if (label == "ayuda") ayuda = result.classification[i].value;
        else if (label == "noise") noise = result.classification[i].value;
        else if (label == "unknown") unknown = result.classification[i].value;
      }

      float otros = noise + unknown;
      Serial.print("[VOZ] ayuda: "); Serial.print(ayuda, 3);
      Serial.print(" | otros: "); Serial.println(otros, 3);

      if (ayuda >= VOICE_THRESHOLD) {
        Serial.println("[🗣️ LLAMADO DE AYUDA DETECTADO]");
        
        if (currentState != SYS_FALL && currentState != SYS_VOICE_ALERT) {
          currentState = SYS_VOICE_ALERT;
          alertStartTime = now;

          if (now - lastSMSTime > COOLDOWN_SMS) {
            enviarSMS("ALERTA: Llamado de ayuda detectado. Requiere asistencia inmediata.", true);
            lastSMSTime = now;
          }
        }
      }
    }
  }

  // ========== GESTIONAR INDICADORES ==========
  gestionarIndicadores();

  // ========== COMANDOS SERIAL ==========
  while (modemSerial.available()) {
    Serial.write(modemSerial.read());
  }

  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'S' || c == 's') {
      enviarSMS("Prueba: Sistema funcionando correctamente.", true);
    } else {
      modemSerial.write(c);
    }
  }
}