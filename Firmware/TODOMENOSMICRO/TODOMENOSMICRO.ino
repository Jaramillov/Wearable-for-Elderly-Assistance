/*
 * SISTEMA INTEGRADO: ESP32 + MPU6050 + GPS NEO-6M + A7670SA
 * CORRECCIONES:
 * - Agregada impresión serial de datos GPS.
 * - Arreglado el Buzzer en estado de RIESGO (ahora pita con el LED).
 * * PRIORIDADES:
 * 1. CAÍDA (Latch 10s) -> ROJO FIJO + Buzzer Continuo
 * 2. RIESGO            -> AMARILLO PARPADEO + Beep Intermitente
 * 3. FUERA DE ZONA     -> ROJO PARPADEO + Alarma Sonora Lenta
 * 4. BUSCANDO GPS      -> AZUL PARPADEO
 * 5. NORMAL            -> VERDE FIJO
 */

#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <math.h>

// ================== PINES Y HARDWARE ==================
// LED RGB (Ánodo Común 3.3V -> LOW=ON, HIGH=OFF)
const int LED_R = 4;
const int LED_G = 2;
const int LED_B = 15;

// Buzzer (Activo HIGH)
const int PIN_BUZZER = 26;

// Módulo SMS A7670SA (UART 1)
const int MODEM_RX = 16; 
const int MODEM_TX = 17; 
const long MODEM_BAUD = 115200;

// GPS NEO-6M (UART 2)
const int GPS_RX = 18; 
const int GPS_TX = 19; 
const long GPS_BAUD = 9600;

// MPU6050 (I2C)
const int I2C_SDA = 21;
const int I2C_SCL = 22;
const uint8_t MPU_ADDR = 0x68;

// Configuración Usuario
const char NUMERO_ALERTA[] = "+573213xxxx"; 
const double SAFE_RADIUS_M = 15.0; 

// ================== OBJETOS ==================
TinyGPSPlus gps;
HardwareSerial modem(1);

// ================== VARIABLES GLOBALES ==================
// GPS
double safeLat = 0.0, safeLon = 0.0;
bool homeSet = false;
bool isOutsideZone = false;
bool geoAlertSent = false;
unsigned long lastGpsPrint = 0; // Timer para imprimir

// MPU
const float ACC_LSB = 16384.0f;
const float GYRO_LSB = 131.0f;
unsigned long lastFallSmsMillis = 0;
const unsigned long FALL_COOLDOWN = 20000;
unsigned long finAlarmaCaidaMillis = 0; 

enum MpuState { EST_NORMAL=0, EST_RIESGO=1, EST_CAIDA=2 };
MpuState estadoMPU = EST_NORMAL;

// Sistema
enum SystemState { SYS_INIT, SYS_NORMAL, SYS_GPS_SEARCH, SYS_OUTSIDE, SYS_RISK, SYS_FALL };
SystemState currentSysState = SYS_INIT;

unsigned long blinkMillis = 0;
bool blinkState = false;

// ================== FUNCIONES HARDWARE ==================
void setRGB(bool r, bool g, bool b) {
  digitalWrite(LED_R, r ? LOW : HIGH);
  digitalWrite(LED_G, g ? LOW : HIGH);
  digitalWrite(LED_B, b ? LOW : HIGH);
}
void buzzerOn()  { digitalWrite(PIN_BUZZER, HIGH); }
void buzzerOff() { digitalWrite(PIN_BUZZER, LOW); }

// ================== MODEM SMS ==================
void enviarComandoModem(const char* cmd, uint16_t esperaMs = 500) {
  modem.println(cmd);
  unsigned long start = millis();
  while(millis() - start < esperaMs) {
    while(modem.available()) Serial.write(modem.read());
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
  modem.println("AT+CMGF=1"); delay(200);
  modem.print("AT+CMGS=\""); modem.print(NUMERO_ALERTA); modem.println("\""); delay(200);

  modem.println(mensaje);
  if (incluirGPS && gps.location.isValid()) {
    String url = "http://maps.google.com/?q=";
    url += String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
    modem.print("Ubicacion: "); modem.println(url);
  } else if (incluirGPS) {
    modem.println("Ubicacion: GPS sin señal.");
  }
  modem.write(26); 
  Serial.println(F("✅ Comando SMS enviado."));
}

// ================== IMPRESIÓN DATOS GPS (NUEVO) ==================
void imprimirDatosGPS() {
  if (millis() - lastGpsPrint < 2000) return; // Imprimir cada 2 seg
  lastGpsPrint = millis();

  Serial.print("📡 GPS: ");
  if (gps.location.isValid()) {
    Serial.print("Lat: "); Serial.print(gps.location.lat(), 6);
    Serial.print(" | Lon: "); Serial.print(gps.location.lng(), 6);
    Serial.print(" | Sats: "); Serial.print(gps.satellites.value());
    
    if (homeSet) {
      double dist = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), safeLat, safeLon);
      Serial.print(" | Dist Home: "); Serial.print(dist, 1); Serial.print("m");
      if (dist > SAFE_RADIUS_M) Serial.print(" [FUERA]");
      else Serial.print(" [DENTRO]");
    } else {
      Serial.print(" | Home: NO CONFIGURADO (>5 sats req)");
    }
  } else {
    Serial.print("Buscando señal... Sats: ");
    if (gps.satellites.isValid()) Serial.print(gps.satellites.value());
    else Serial.print("0");
  }
  Serial.println();
}

// ================== MPU6050 ==================
bool initMPU() {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0x00);
  if(Wire.endTransmission() != 0) return false;
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1C); Wire.write(0x00); Wire.endTransmission();
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1B); Wire.write(0x00); Wire.endTransmission();
  return true;
}

bool leerMPU(float &accMag, float &gyroMag) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true);
  if (Wire.available() < 14) return false;

  int16_t ax = (Wire.read()<<8)|Wire.read();
  int16_t ay = (Wire.read()<<8)|Wire.read();
  int16_t az = (Wire.read()<<8)|Wire.read();
  Wire.read(); Wire.read(); 
  int16_t gx = (Wire.read()<<8)|Wire.read();
  int16_t gy = (Wire.read()<<8)|Wire.read();
  int16_t gz = (Wire.read()<<8)|Wire.read();

  float fax = ax/ACC_LSB, fay = ay/ACC_LSB, faz = az/ACC_LSB;
  float fgx = gx/GYRO_LSB, fgy = gy/GYRO_LSB, fgz = gz/GYRO_LSB;

  accMag = sqrt(fax*fax + fay*fay + faz*faz);
  gyroMag = sqrt(fgx*fgx + fgy*fgy + fgz*fgz);
  return true;
}

float clamp01(float x) { return (x<0)?0:(x>1)?1:x; }

void actualizarMPU() {
  unsigned long now = millis();
  
  if (finAlarmaCaidaMillis > 0) {
    if (now < finAlarmaCaidaMillis) {
      estadoMPU = EST_CAIDA; return; 
    } else {
      finAlarmaCaidaMillis = 0; 
    }
  }

  static unsigned long lastRead = 0;
  if (now - lastRead < 200) return;
  lastRead = now;

  float acc, gyro;
  if (!leerMPU(acc, gyro)) return;

  float accImp = clamp01((acc - 1.0f) / 2.0f);
  float gyroImp = clamp01(gyro / 300.0f);
  float score = clamp01(0.6f*accImp + 0.4f*gyroImp);

  MpuState nuevoEstado = EST_NORMAL;
  if (score >= 0.7f) nuevoEstado = EST_CAIDA;
  else if (score >= 0.4f) nuevoEstado = EST_RIESGO;

  if (estadoMPU != EST_CAIDA && nuevoEstado == EST_CAIDA) {
    Serial.println(F("🚨 CAIDA DETECTADA -> ALARMA 10s"));
    finAlarmaCaidaMillis = now + 10000; 
    estadoMPU = EST_CAIDA;
    
    if (now - lastFallSmsMillis > FALL_COOLDOWN || lastFallSmsMillis == 0) {
      enviarSMS("ALERTA: CAIDA DETECTADA!", true);
      lastFallSmsMillis = now;
    }
  } else {
    estadoMPU = nuevoEstado;
  }
}

// ================== GPS LOGIC ==================
void actualizarGPS() {
  if (!gps.location.isValid()) return;

  if (!homeSet && gps.satellites.value() > 4) {
    safeLat = gps.location.lat();
    safeLon = gps.location.lng();
    homeSet = true;
    Serial.println(F("🏠 HOME CONFIGURADO"));
  }

  if (homeSet) {
    double dist = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), safeLat, safeLon);
    
    if (dist > SAFE_RADIUS_M) {
      isOutsideZone = true;
      if (!geoAlertSent) {
        enviarSMS("ALERTA: SALIO DE ZONA SEGURA. Dist: " + String(dist,1) + "m", true);
        geoAlertSent = true;
      }
    } else {
      isOutsideZone = false;
      geoAlertSent = false;
    }
  }
}

// ================== INDICADORES ==================
void gestionarIndicadores() {
  // Jerarquía
  if (estadoMPU == EST_CAIDA) currentSysState = SYS_FALL;
  else if (estadoMPU == EST_RIESGO) currentSysState = SYS_RISK;
  else if (homeSet && isOutsideZone) currentSysState = SYS_OUTSIDE;
  else if (!homeSet || !gps.location.isValid()) currentSysState = SYS_GPS_SEARCH;
  else currentSysState = SYS_NORMAL;

  unsigned long now = millis();
  
  switch (currentSysState) {
    case SYS_FALL:
      setRGB(true, false, false); // Rojo Fijo
      if ((now / 150) % 2 == 0) buzzerOn(); else buzzerOff(); // Beep-Beep rápido
      break;

    case SYS_RISK:
      // CORRECCIÓN: Hacemos parpadear el Amarillo Y el Buzzer juntos
      if (now - blinkMillis > 200) { // Velocidad parpadeo
        blinkMillis = now; 
        blinkState = !blinkState;
        if(blinkState) {
          setRGB(true, true, false); // Amarillo ON
          buzzerOn();                // Beep ON
        } else {
          setRGB(false, false, false); // Apagado
          buzzerOff();                 // Beep OFF
        }
      }
      break;

    case SYS_OUTSIDE:
      if (now - blinkMillis > 500) {
        blinkMillis = now; blinkState = !blinkState;
        if(blinkState) {
          setRGB(true, false, false); // Rojo ON
          buzzerOn(); 
        } else {
          setRGB(false, false, false);
          buzzerOff();
        }
      }
      break;

    case SYS_GPS_SEARCH:
      if (now - blinkMillis > 1000) {
        blinkMillis = now; blinkState = !blinkState;
        if(blinkState) setRGB(false, false, true); // Azul ON
        else setRGB(false, false, false);
      }
      buzzerOff();
      break;

    case SYS_NORMAL:
      setRGB(false, true, false); // Verde Fijo
      buzzerOff();
      break;
  }
}

// ================== SETUP & LOOP ==================
void setup() {
  Serial.begin(115200);
  Serial.println(F("=== SISTEMA INTEGRADO DE SEGURIDAD ==="));

  pinMode(LED_R, OUTPUT); pinMode(LED_G, OUTPUT); pinMode(LED_B, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  
  setRGB(false, false, true); // Azul inicio
  buzzerOff();

  Wire.begin(I2C_SDA, I2C_SCL);
  if(initMPU()) Serial.println("✅ MPU OK");
  else Serial.println("❌ MPU FALLO");

  modem.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
  inicializarSMS();

  Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
}

void loop() {
  while (Serial2.available() > 0) {
    gps.encode(Serial2.read());
  }

  actualizarGPS();
  actualizarMPU();
  gestionarIndicadores();
  imprimirDatosGPS(); // <--- Nueva función para ver info
}