/*
 * ESP32 + GPS NEO-6M + MPU6050 + A7670SA
 * - SMS cuando sale de la zona segura (GPS)
 * - SMS cuando la lógica de caídas detecta estado CAIDA (MPU)
 */

#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <math.h>

// ================== GPS ==================
TinyGPSPlus gps;

// Pines GPS (ajusta si usas otros)
static const int GPS_RX_PIN = 18;  // GPS TX -> ESP32 RX2
static const int GPS_TX_PIN = 19;  // GPS RX -> ESP32 TX2
static const uint32_t GPS_BAUD   = 9600;

// Zona horaria Colombia (UTC -5)
const int TIMEZONE_OFFSET = -5;

// Punto seguro (se configurará automáticamente)
double safeLat = 0.0;
double safeLon = 0.0;
bool   homeSet = false;

// Radio seguro en metros (AJUSTA según tu proyecto)
const double SAFE_RADIUS_M = 15.0;


// ================== MÓDULO SMS A7670SA ==================
HardwareSerial modem(1);  // Usamos UART1 para el módulo LTE/A7670SA

static const int MODEM_RX   = 16;       // A7670SA TXD -> ESP32 GPIO26
static const int MODEM_TX   = 17;       // A7670SA RXD -> ESP32 GPIO27
static const uint32_t MODEM_BAUD = 115200;

// Número al que se enviarán las alertas (YA CON TU NÚMERO)
const char NUMERO_ALERTA[] = "+573213186468"; 

// Para no spamear SMS de geocerca
bool geoAlertSent = false;


// ================== MPU6050 (CAÍDAS) ==================
const uint8_t MPU_ADDR          = 0x68;
const float   ACC_LSB_PER_G     = 16384.0f; // ±2g
const float   GYRO_LSB_PER_DPS  = 131.0f;   // ±250 dps

// Pines I2C típica ESP32 (ajusta si usas otros)
const int I2C_SDA = 21;
const int I2C_SCL = 22;

// Cooldown entre SMS de caída (para no spamear)
const unsigned long FALL_SMS_COOLDOWN_MS = 20000; // 20 s
unsigned long lastFallSmsMillis = 0;

// Estados de la lógica de caídas
enum FallState {
  ESTADO_NORMAL = 0,
  ESTADO_RIESGO = 1,
  ESTADO_CAIDA  = 2
};

FallState estadoActual = ESTADO_NORMAL;


// ============= PROTOTIPOS DE FUNCIONES =============
// SMS genérico
void inicializarModuloSMS();
void enviarComandoModem(const char* cmd, uint16_t esperaMs = 500);

// SMS GPS
void enviarSMSAlertaGPS(double lat, double lon, double distancia);

// SMS CAÍDA
void enviarSMSCaida(float fallScore, float acc_mag, float gyro_mag);

// GPS / Geocerca
void configurarHomeSiEsNecesario();
void procesarGeocerca();
void imprimirDatosGPS();

// MPU / caídas
bool initMPU();
bool leerMPU(float &ax_g, float &ay_g, float &az_g,
             float &gx_dps, float &gy_dps, float &gz_dps);

float clamp01(float x);
float calcularFallScore(float acc_mag, float gyro_mag);
FallState evaluarEstado(float fallScore, float acc_mag, float gyro_mag);
void actualizarLogicaCaidas();


// ===================================================
//                     SETUP
// ===================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("\n=== ESP32 + GPS + MPU6050 + A7670SA (SMS) ==="));

  // UART GPS (Serial2)
  Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  // LED/buzzer de geocerca
  pinMode(ALERT_PIN, OUTPUT);
  digitalWrite(ALERT_PIN, LOW);

  // I2C + MPU6050
  Wire.begin(I2C_SDA, I2C_SCL);
  if (initMPU()) Serial.println("✅ MPU6050 inicializado.");
  else           Serial.println("❌ Error inicializando MPU6050.");

  // UART MÓDULO A7670SA (modem = Serial1)
  modem.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);  // tiempo para que el módulo arranque

  inicializarModuloSMS();

  Serial.println(F("Sistema listo: GPS + Geocerca + MPU (caídas) + SMS.\n"));
}


// ===================================================
//                     LOOP
// ===================================================
void loop() {
  // --- Pasarela Serial PC <-> Módem (útil para debug AT) ---
  if (Serial.available()) {
    modem.write(Serial.read());
  }
  if (modem.available()) {
    Serial.write(modem.read());
  }

  // --- GPS: leer datos y procesar geocerca ---
  while (Serial2.available() > 0) {
    char c = Serial2.read();
    gps.encode(c);

    if (gps.location.isUpdated()) {
      configurarHomeSiEsNecesario();
      procesarGeocerca();   // aquí se dispara SMS de geocerca
      imprimirDatosGPS();   // línea compacta de GPS
    }
  }

  // --- MPU: actualizar lógica de caídas ---
  actualizarLogicaCaidas();
}


// ===================================================
//                MÓDULO SMS (COMÚN)
// ===================================================
void inicializarModuloSMS() {
  Serial.println(F("Inicializando módulo A7670SA para SMS..."));

  enviarComandoModem("AT");
  enviarComandoModem("ATE0");             // eco off (opcional)
  enviarComandoModem("AT+CMGF=1");        // modo texto
  enviarComandoModem("AT+CSCS=\"GSM\"");  // juego de caracteres

  Serial.println(F("Módulo SMS configurado."));
}

void enviarComandoModem(const char* cmd, uint16_t esperaMs) {
  modem.println(cmd);
  delay(esperaMs);

  // (Opcional) leer respuesta del módulo y mostrarla por Serial
  while (modem.available()) {
    char c = modem.read();
    Serial.write(c);
  }
}


// ===================================================
//                SMS GEOCERCA (GPS)
// ===================================================
void enviarSMSAlertaGPS(double lat, double lon, double distancia) {
  Serial.println(F("📩 Enviando SMS de alerta por geocerca..."));

  modem.println("AT+CMGF=1");  // asegurar modo texto
  delay(500);

  // Comando para iniciar SMS
  modem.print("AT+CMGS=\"");
  modem.print(NUMERO_ALERTA);
  modem.println("\"");
  delay(500);

  // Generar URL de Google Maps
  String url = "https://www.google.com/maps/?q=";
  url += String(lat, 6);
  url += ",";
  url += String(lon, 6);

  // Texto del mensaje
  modem.print("ALERTA: Fuera de la zona segura.\n");
  modem.print("Distancia: ");
  modem.print(distancia, 1);
  modem.println(" m.");
  modem.print("Ubicacion: ");
  modem.println(url);
  modem.println("Dispositivo: ESP32 tracker");

  // Fin del mensaje con Ctrl+Z (ASCII 26)
  modem.write(26);
  modem.println();

  // Esperar a que el módulo procese y envíe
  delay(5000);

  Serial.println(F("✅ SMS de geocerca enviado (si la red está disponible)."));

  // Leer respuesta (opcional)
  while (modem.available()) {
    char c = modem.read();
    Serial.write(c);
  }
}


// ===================================================
//                SMS CAÍDA (MPU6050)
//   -> AHORA SOLO UBICACIÓN (si hay fix), NO SCORES
// ===================================================
void enviarSMSCaida(float fallScore, float acc_mag, float gyro_mag) {
  (void)fallScore; // para evitar warnings por no usar
  (void)acc_mag;
  (void)gyro_mag;

  unsigned long now = millis();

  // NO aplicar cooldown al primer SMS (lastFallSmsMillis = 0)
  if (lastFallSmsMillis != 0 &&
      (now - lastFallSmsMillis < FALL_SMS_COOLDOWN_MS)) {
    Serial.println("⏳ SMS de caída ignorado (cooldown activo).");
    return;
  }

  Serial.println("\n📩 Enviando SMS de alerta de CAÍDA...");

  // Obtener última ubicación conocida
  bool hasLocation = gps.location.isValid();
  double lat = 0.0;
  double lon = 0.0;

  if (hasLocation) {
    lat = gps.location.lat();
    lon = gps.location.lng();
    Serial.print("Ubicacion caida (lat, lon): ");
    Serial.print(lat, 6);
    Serial.print(", ");
    Serial.println(lon, 6);
  } else {
    Serial.println("Ubicacion caida: sin fix GPS.");
  }

  modem.println("AT+CMGF=1");  // asegurar modo texto
  delay(500);

  modem.print("AT+CMGS=\"");
  modem.print(NUMERO_ALERTA);
  modem.println("\"");
  delay(500);

  modem.println("ALERTA CAIDA: posible caida detectada.");

  if (hasLocation) {
    String url = "https://www.google.com/maps/?q=";
    url += String(lat, 6);
    url += ",";
    url += String(lon, 6);

    modem.println("Ubicacion aproximada:");
    modem.println(url);
  } else {
    modem.println("Ubicacion no disponible (sin fix GPS).");
  }

  modem.println("Dispositivo: ESP32 caidas + GPS");

  modem.write(26);  // CTRL+Z
  modem.println();

  delay(5000);

  Serial.println("✅ SMS de CAÍDA enviado.");
  lastFallSmsMillis = now;
}


// ===================================================
//                GEO-CERCA / GPS
// ===================================================
void configurarHomeSiEsNecesario() {
  if (homeSet) return;                 // ya está configurado
  if (!gps.location.isValid()) return; // aún no hay fix de posición

  // condición: satélites válidos y > 5
  if (!gps.satellites.isValid()) return;
  if (gps.satellites.value() <= 5) {
    return;
  }

  // Primera posición válida con >5 satélites → se convierte en la posición segura
  safeLat = gps.location.lat();
  safeLon = gps.location.lng();
  homeSet = true;

  Serial.print(F("🏠 HOME fijado | lat="));
  Serial.print(safeLat, 6);
  Serial.print(F(" lon="));
  Serial.println(safeLon, 6);
}

void procesarGeocerca() {
  if (!homeSet) {
    Serial.println(F("GEOFENCE | HOME no configurado (esperando fix con Sats > 5)"));
    digitalWrite(ALERT_PIN, LOW);
    return;
  }

  if (!gps.location.isValid()) {
    Serial.println(F("GEOFENCE | sin fix actual, no se evalúa"));
    digitalWrite(ALERT_PIN, LOW);
    return;
  }

  double lat = gps.location.lat();
  double lon = gps.location.lng();

  double distancia = TinyGPSPlus::distanceBetween(lat, lon, safeLat, safeLon);

  // Línea compacta de estado de geocerca
  Serial.print("GEOFENCE | dist=");
  Serial.print(distancia, 1);
  Serial.print(" m | estado=");
  if (distancia > SAFE_RADIUS_M) {
    Serial.println("FUERA");
    digitalWrite(ALERT_PIN, HIGH);

    if (!geoAlertSent) {
      enviarSMSAlertaGPS(lat, lon, distancia);
      geoAlertSent = true;   // no volver a mandar hasta re-entrar
    }
  } else {
    Serial.println("DENTRO");
    digitalWrite(ALERT_PIN, LOW);
    geoAlertSent = false;    // permite enviar otro SMS si vuelve a salir
  }
}

void imprimirDatosGPS() {
  // Formato compacto: una sola línea con lo más útil
  Serial.print("GPS | ");

  if (gps.location.isValid()) {
    Serial.print("lat=");
    Serial.print(gps.location.lat(), 6);
    Serial.print(" lon=");
    Serial.print(gps.location.lng(), 6);
  } else {
    Serial.print("lat=--- lon=---");
  }

  Serial.print(" | sats=");
  if (gps.satellites.isValid()) Serial.print(gps.satellites.value());
  else                          Serial.print("-");

  Serial.print(" | hdop=");
  if (gps.hdop.isValid()) Serial.print(gps.hdop.value());
  else                     Serial.print("-");

  Serial.print(" | vel=");
  if (gps.speed.isValid()) Serial.print(gps.speed.kmph());
  else                     Serial.print("-");
  Serial.println(" km/h");
}


// ===================================================
//                MPU6050 / CAÍDAS
// ===================================================
bool initMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);      // PWR_MGMT_1
  Wire.write(0x00);      // despertar
  if (Wire.endTransmission() != 0) return false;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);      // ACCEL_CONFIG
  Wire.write(0x00);      // ±2g
  if (Wire.endTransmission() != 0) return false;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);      // GYRO_CONFIG
  Wire.write(0x00);      // ±250 dps
  if (Wire.endTransmission() != 0) return false;

  return true;
}

bool leerMPU(float &ax_g, float &ay_g, float &az_g,
             float &gx_dps, float &gy_dps, float &gz_dps) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true);
  if (Wire.available() < 14) return false;

  int16_t accX = (Wire.read() << 8) | Wire.read();
  int16_t accY = (Wire.read() << 8) | Wire.read();
  int16_t accZ = (Wire.read() << 8) | Wire.read();
  int16_t temp = (Wire.read() << 8) | Wire.read(); // sin usar
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


// ---------- Lógica de "score" de caída ----------
float clamp01(float x) {
  if (x < 0.0f) return 0.0f;
  if (x > 1.0f) return 1.0f;
  return x;
}

// Combina aceleración y giro en un score 0..1
float calcularFallScore(float acc_mag, float gyro_mag) {
  // 1g -> 0, 3g -> 1
  float accImpact = (acc_mag - 1.0f) / (3.0f - 1.0f);
  accImpact = clamp01(accImpact);

  // 0 dps -> 0, 300 dps -> 1
  float gyroImpact = gyro_mag / 300.0f;
  gyroImpact = clamp01(gyroImpact);

  return clamp01(0.6f * accImpact + 0.4f * gyroImpact);
}

// Evalúa el estado en función del fallScore (FORMATO COMPACTO)
FallState evaluarEstado(float fallScore, float acc_mag, float gyro_mag) {
  FallState nuevoEstado;
  if (fallScore < 0.3f) {
    nuevoEstado = ESTADO_NORMAL;
  } else if (fallScore < 0.7f) {
    nuevoEstado = ESTADO_RIESGO;
  } else {
    nuevoEstado = ESTADO_CAIDA;
  }

  Serial.print("MPU | acc=");
  Serial.print(acc_mag, 2);
  Serial.print(" g gyro=");
  Serial.print(gyro_mag, 1);
  Serial.print(" dps score=");
  Serial.print(fallScore, 2);
  Serial.print(" | estado=");

  if (nuevoEstado == ESTADO_NORMAL)      Serial.println("NORMAL");
  else if (nuevoEstado == ESTADO_RIESGO) Serial.println("RIESGO");
  else                                   Serial.println("CAIDA");

  return nuevoEstado;
}

void actualizarLogicaCaidas() {
  static unsigned long lastRead = 0;
  const unsigned long READ_INTERVAL_MS = 200; // lee cada 200 ms (~5 Hz)

  if (millis() - lastRead < READ_INTERVAL_MS) return;
  lastRead = millis();

  float ax, ay, az, gx, gy, gz;
  if (!leerMPU(ax, ay, az, gx, gy, gz)) {
    Serial.println("⚠️ Error leyendo MPU6050");
    return;
  }

  float acc_mag  = sqrt(ax*ax + ay*ay + az*az);
  float gyro_mag = sqrt(gx*gx + gy*gy + gz*gz);

  float fallScore = calcularFallScore(acc_mag, gyro_mag);
  FallState nuevo = evaluarEstado(fallScore, acc_mag, gyro_mag);

  if (estadoActual != ESTADO_CAIDA && nuevo == ESTADO_CAIDA) {
    Serial.println("🚨 CAIDA detectada -> enviando SMS...");
    enviarSMSCaida(fallScore, acc_mag, gyro_mag);
  }

  estadoActual = nuevo;
}
