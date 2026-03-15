/*
 * ESP32 + MPU6050 + A7670SA (Versión MPU Solo)
 * LÓGICA LED CORREGIDA:
 * - NORMAL -> 🟢 VERDE Fijo
 * - RIESGO -> 🟡 AMARILLO Parpadeando
 * - CAIDA  -> 🔴 ROJO Fijo (+ Buzzer enclavado 10s)
 */

#include <HardwareSerial.h>
#include <Wire.h>
#include <math.h>

// ---------- CONFIGURACIÓN DE PINES MÓDULO A7670SA ----------
const int RX_PIN      = 16;  // ESP32 RX2  <- TX del A7670SA
const int TX_PIN      = 17;  // ESP32 TX2  -> RX del A7670SA
const int PWRKEY_PIN  = -1;  // GPIO a PWRKEY del A7670SA

// ---------- UART DEL MÓDULO ----------
const long MODEM_BAUD = 115200; 

// ---------- NÚMERO PARA SMS DE ALERTA ----------
const char ALERT_PHONE_NUMBER[] = "+573103435841"; 

HardwareSerial SerialAT(2); 

// -------------------------------------------------------------------
// CONFIGURACIÓN MPU6050
// -------------------------------------------------------------------
const uint8_t MPU_ADDR          = 0x68;
const float   ACC_LSB_PER_G     = 16384.0f; 
const float   GYRO_LSB_PER_DPS  = 131.0f;   

const int I2C_SDA = 21;
const int I2C_SCL = 22;

// Cooldown entre SMS de caída
const unsigned long FALL_SMS_COOLDOWN_MS = 20000; 
unsigned long lastSmsMillis = 0;

// Variable para el enclavamiento (Latch) de la alarma
unsigned long finAlarmaMillis = 0; 

// Estados de la lógica difusa
enum FallState {
  ESTADO_NORMAL = 0,
  ESTADO_RIESGO = 1,
  ESTADO_CAIDA  = 2
};

FallState estadoActual = ESTADO_NORMAL;

// -------------------------------------------------------------------
// INDICADORES: LED RGB + BUZZER
// -------------------------------------------------------------------
// AJUSTE DE PINES (Corregido respecto al código pegado)
const int LED_R = 4;   // Rojo (antes tenías 4)
const int LED_G = 2;  // Verde
const int LED_B = 15;   // Azul (antes tenías 2)
const int BUZZER_PIN = 26;

inline void buzzerOn()  { digitalWrite(BUZZER_PIN, HIGH); } // Ajusta HIGH/LOW según tu buzzer
inline void buzzerOff() { digitalWrite(BUZZER_PIN, LOW);  }

// Función para controlar LED RGB Ánodo Común (LOW = Encendido)
void setRGB(bool rOn, bool gOn, bool bOn) {
  digitalWrite(LED_R, rOn ? LOW : HIGH);
  digitalWrite(LED_G, gOn ? LOW : HIGH);
  digitalWrite(LED_B, bOn ? LOW : HIGH);
}

void rgbOff()     { setRGB(false, false, false); }
void rgbRed()     { setRGB(true,  false, false); } // 🔴
void rgbGreen()   { setRGB(false, true,  false); } // 🟢
void rgbYellow()  { setRGB(true,  true,  false); } // 🟡 (Rojo + Verde)

// Parpadeo para RIESGO (Amarillo - Apagado)
void updateRiskBlink() {
  static unsigned long t0 = 0;
  static bool on = false;
  const unsigned long PERIOD_MS = 500; // Velocidad del parpadeo

  unsigned long now = millis();
  if (now - t0 >= PERIOD_MS) {
    t0 = now;
    on = !on;
    if (on) rgbYellow();
    else    rgbOff();
  }
}

// ---------- Beep corto al entrar en RIESGO ----------
bool riskBeepActive = false;
unsigned long riskBeepStartMs = 0;

void riskBeepStart() {
  riskBeepActive = true;
  riskBeepStartMs = millis();
  buzzerOn();
}

void riskBeepUpdate() {
  if (!riskBeepActive) return;
  if (millis() - riskBeepStartMs >= 150) { 
    buzzerOff();
    riskBeepActive = false;
  }
}

// Patrón repetitivo para CAÍDA: beep-beep-beep
void updateFallBeepPattern() {
  static uint8_t step = 0;
  static unsigned long tStep = 0;

  unsigned long now = millis();
  unsigned long dt = now - tStep;

  switch (step) {
    case 0: buzzerOn();  tStep = now; step = 1; break;
    case 1: if (dt >= 120) { buzzerOff(); tStep = now; step = 2; } break;
    case 2: if (dt >= 120) { buzzerOn();  tStep = now; step = 3; } break;
    case 3: if (dt >= 120) { buzzerOff(); tStep = now; step = 4; } break;
    case 4: if (dt >= 120) { buzzerOn();  tStep = now; step = 5; } break;
    case 5: if (dt >= 120) { buzzerOff(); tStep = now; step = 6; } break;
    case 6: if (dt >= 600) { step = 0; } break;
  }
}

// *** LÓGICA PRINCIPAL DE INDICADORES ***
void updateIndicators() {
  // Prioridad 1: CAIDA (Rojo Fijo + Alarma sonora)
  if (estadoActual == ESTADO_CAIDA) {
    rgbRed();
    updateFallBeepPattern();
  }
  // Prioridad 2: RIESGO (Amarillo Parpadeando + Silencio tras el beep inicial)
  else if (estadoActual == ESTADO_RIESGO) {
    updateRiskBlink();
    riskBeepUpdate(); // Gestiona el beep corto si está activo
  }
  // Prioridad 3: NORMAL (Verde Fijo + Silencio)
  else { 
    rgbGreen();
    // Aseguramos que el buzzer se apague si venía de otro estado
    if (!riskBeepActive) buzzerOff(); 
  }
}

// -------------------------------------------------------------------
// FUNCIONES MÓDULO SMS
// -------------------------------------------------------------------
bool waitForResponse(const char* target, uint32_t timeout_ms) {
  uint32_t start = millis();
  String resp;
  while (millis() - start < timeout_ms) {
    updateIndicators(); // Mantener luces/sonido
    riskBeepUpdate();
    while (SerialAT.available()) {
      char c = SerialAT.read();
      resp += c;
      Serial.write(c); 
      if (resp.indexOf(target) != -1) return true;
    }
    delay(2);
  }
  return false;
}

void powerOnModule() {
  if (PWRKEY_PIN < 0) return;
  pinMode(PWRKEY_PIN, OUTPUT);
  digitalWrite(PWRKEY_PIN, LOW); delay(100);
  digitalWrite(PWRKEY_PIN, HIGH); delay(2000);
  digitalWrite(PWRKEY_PIN, LOW); delay(8000);
}

void enviarSMSCaida() {
  unsigned long now = millis();
  if (lastSmsMillis != 0 && (now - lastSmsMillis < FALL_SMS_COOLDOWN_MS)) {
    Serial.println("⏳ SMS ignorado (cooldown).");
    return;
  }
  Serial.println("\n📩 Iniciando envío SMS...");
  SerialAT.println("AT");
  if (!waitForResponse("OK", 2000)) return;
  SerialAT.println("AT+CMGF=1");
  if (!waitForResponse("OK", 2000)) return;
  SerialAT.println("AT+CSCS=\"GSM\"");
  waitForResponse("OK", 2000);
  SerialAT.print("AT+CMGS=\"");
  SerialAT.print(ALERT_PHONE_NUMBER);
  SerialAT.println("\"");
  if (!waitForResponse(">", 5000)) return;
  SerialAT.print("ALERTA: CAIDA DETECTADA (Sistema MPU).");
  SerialAT.write(26); 
  if (waitForResponse("OK", 15000)) {
    Serial.println("✅ SMS enviado.");
    lastSmsMillis = now;
  } else {
    Serial.println("❌ Error enviando SMS.");
  }
}

// -------------------------------------------------------------------
// MPU6050
// -------------------------------------------------------------------
bool initMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x00);
  if (Wire.endTransmission() != 0) return false;
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); Wire.write(0x00);
  if (Wire.endTransmission() != 0) return false;
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); Wire.write(0x00);
  return (Wire.endTransmission() == 0);
}

bool leerMPU(float &ax_g, float &ay_g, float &az_g,
             float &gx_dps, float &gy_dps, float &gz_dps) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true);
  if (Wire.available() < 14) return false;
  int16_t accX = (Wire.read() << 8) | Wire.read();
  int16_t accY = (Wire.read() << 8) | Wire.read();
  int16_t accZ = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); 
  int16_t gyroX = (Wire.read() << 8) | Wire.read();
  int16_t gyroY = (Wire.read() << 8) | Wire.read();
  int16_t gyroZ = (Wire.read() << 8) | Wire.read();
  ax_g = accX / ACC_LSB_PER_G;
  ay_g = accY / ACC_LSB_PER_G;
  az_g = accZ / ACC_LSB_PER_G;
  gx_dps = gyroX / GYRO_LSB_PER_DPS;
  gy_dps = gyroY / GYRO_LSB_PER_DPS;
  gz_dps = gyroZ / GYRO_LSB_PER_DPS;
  return true;
}

// -------------------------------------------------------------------
// LÓGICA DIFUSA
// -------------------------------------------------------------------
float clamp01(float x) {
  if (x < 0.0f) return 0.0f;
  if (x > 1.0f) return 1.0f;
  return x;
}

float calcularFallScore(float acc_mag, float gyro_mag) {
  float accImpact = (acc_mag - 1.0f) / 2.0f; 
  accImpact = clamp01(accImpact);
  float gyroImpact = gyro_mag / 300.0f;
  gyroImpact = clamp01(gyroImpact);
  return clamp01(0.6f * accImpact + 0.4f * gyroImpact);
}

FallState evaluarEstadoDifuso(float acc_mag, float gyro_mag) {
  float fallScore = calcularFallScore(acc_mag, gyro_mag);
  // UMBRALES: <0.4 Normal, 0.4-0.7 Riesgo, >0.7 Caída
  if (fallScore < 0.4f) return ESTADO_NORMAL;
  if (fallScore < 0.7f) return ESTADO_RIESGO;
  return ESTADO_CAIDA;
}

void actualizarLogicaDifusa() {
  unsigned long now = millis();

  // 1. LATCH (Enclavamiento) de 10 segundos tras caída
  if (finAlarmaMillis > 0) {
    if (now < finAlarmaMillis) {
      estadoActual = ESTADO_CAIDA; // Forzamos estado
      return; // No leemos sensores
    } else {
      finAlarmaMillis = 0; // Fin alarma
    }
  }

  // 2. Lectura sensores
  static unsigned long lastRead = 0;
  if (now - lastRead < 200) return;
  lastRead = now;

  float ax, ay, az, gx, gy, gz;
  if (!leerMPU(ax, ay, az, gx, gy, gz)) return;

  float acc_mag  = sqrt(ax*ax + ay*ay + az*az);
  float gyro_mag = sqrt(gx*gx + gy*gy + gz*gz);

  FallState nuevo = evaluarEstadoDifuso(acc_mag, gyro_mag);

  // Detectar entrada a RIESGO para beep corto
  if (estadoActual != ESTADO_RIESGO && nuevo == ESTADO_RIESGO) {
    riskBeepStart();
  }

  // Detectar entrada a CAIDA para activar LATCH y SMS
  if (estadoActual != ESTADO_CAIDA && nuevo == ESTADO_CAIDA) {
    Serial.println("🚨 CAIDA DETECTADA -> ALARMA 10s");
    finAlarmaMillis = now + 10000;
    estadoActual = ESTADO_CAIDA;
    enviarSMSCaida();
  } else {
    estadoActual = nuevo;
  }
}

// -------------------------------------------------------------------
// SETUP & LOOP
// -------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  rgbOff();
  buzzerOff();

  Wire.begin(I2C_SDA, I2C_SCL);
  initMPU();

  SerialAT.begin(MODEM_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
  powerOnModule();

  Serial.println("Sistema Listo: Verde=OK, Amarillo=Riesgo, Rojo=Caida.");
}

void loop() {
  // Pasarela Serial
  if (Serial.available())   SerialAT.write(Serial.read());
  if (SerialAT.available()) Serial.write(SerialAT.read());

  // Lógica principal
  actualizarLogicaDifusa();
  updateIndicators();
}