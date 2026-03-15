#include <TinyGPS++.h>
#include <HardwareSerial.h>

// ================== CONFIGURACIÓN HARDWARE ==================
// LED RGB (Ánodo Común a 3.3V) -> ON=LOW, OFF=HIGH
const int LED_R = 4;   // Rojo
const int LED_G = 2;   // Verde
const int LED_B = 15;  // Azul

// Buzzer (Activo HIGH)
const int PIN_BUZZER = 26; 

// ================== GPS (NEO-6M) ==================
TinyGPSPlus gps;
static const int RXPin = 18;  // GPS TX -> ESP32 RX2
static const int TXPin = 19;  // GPS RX -> ESP32 TX2
static const uint32_t GPSBaud = 9600;

// Configuración de Zona
double safeLat = 0.0;
double safeLon = 0.0;
bool homeSet = false;
const double SAFE_RADIUS_M = 25.0; // Radio en metros

bool isOutsideZone = false; 

// ================== MÓDULO SMS A7670SA ==================
HardwareSerial modem(1);
static const int MODEM_RX = 16;
static const int MODEM_TX = 17;
static const uint32_t MODEM_BAUD = 115200;

const char NUMERO_ALERTA[] = "+573213186468"; 
bool alertSent = false;

// ================== TIMERS (MILLIS) ==================
unsigned long previousMillisAlarm = 0;
bool alarmState = false; 
const long INTERVALO_ALARMA = 200; // Velocidad del parpadeo (ms)

// ============= PROTOTIPOS =============
void inicializarModuloSMS();
void enviarSMSAlerta(double lat, double lon, double distancia);
void enviarComandoModem(const char* cmd, uint16_t esperaMs = 500);
void configurarHomeSiEsNecesario();
void procesarGeocerca();
void manejarIndicadores(); 
void setRGB(bool r, bool g, bool b); // Función auxiliar para colores
void imprimirDatos();

void setup() {
  Serial.begin(115200);
  Serial.println(F("=== GPS GEOFENCE + RGB ANODO COMUN ==="));

  // Configurar pines
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);

  // Estado inicial: Apagar todo (HIGH para ánodo común)
  setRGB(false, false, false); 
  digitalWrite(PIN_BUZZER, LOW);

  // Iniciar GPS y Módem
  Serial2.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);
  modem.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);
  
  inicializarModuloSMS();
}

void loop() {
  // 1. Leer datos GPS
  while (Serial2.available() > 0) {
    char c = Serial2.read();
    gps.encode(c);
    if (gps.location.isUpdated()) {
      configurarHomeSiEsNecesario();
      procesarGeocerca();
      imprimirDatos();
    }
  }

  // 2. Controlar Luces y Sonido (Sin delay)
  manejarIndicadores();
}

// ------------------ CONTROL DE LED RGB (ÁNODO COMÚN) ------------------
// true = ENCENDIDO (LOW), false = APAGADO (HIGH)
void setRGB(bool r, bool g, bool b) {
  digitalWrite(LED_R, r ? LOW : HIGH);
  digitalWrite(LED_G, g ? LOW : HIGH);
  digitalWrite(LED_B, b ? LOW : HIGH);
}

// ------------------ LÓGICA DE INDICADORES ------------------
void manejarIndicadores() {
  // CASO 1: ZONA SEGURA (DENTRO)
  if (!isOutsideZone) {
    // Verde Fijo, Buzzer apagado
    setRGB(false, true, false); 
    digitalWrite(PIN_BUZZER, LOW);
    alarmState = false;
  } 
  // CASO 2: PELIGRO (FUERA) -> ALARMA
  else {
    unsigned long currentMillis = millis();
    
    // Timer para parpadeo
    if (currentMillis - previousMillisAlarm >= INTERVALO_ALARMA) {
      previousMillisAlarm = currentMillis;
      alarmState = !alarmState;

      if (alarmState) {
        // ENCENDER: Rojo ON, Buzzer ON
        setRGB(true, false, false);
        digitalWrite(PIN_BUZZER, HIGH);
      } else {
        // APAGAR: Todo OFF, Buzzer OFF
        setRGB(false, false, false); 
        digitalWrite(PIN_BUZZER, LOW);
      }
    }
  }
}

// ------------------ GEO-CERCA LOGIC ------------------
void configurarHomeSiEsNecesario() {
  if (homeSet) return;
  if (!gps.location.isValid()) return;
  if (!gps.satellites.isValid()) return;
  if (gps.satellites.value() <= 5) return; // Esperar buena señal

  safeLat = gps.location.lat();
  safeLon = gps.location.lng();
  homeSet = true;
  Serial.println(F("🏠 HOME CONFIGURADO (LED debe estar VERDE)"));
}

void procesarGeocerca() {
  if (!homeSet || !gps.location.isValid()) {
    isOutsideZone = false; 
    return;
  }

  double lat = gps.location.lat();
  double lon = gps.location.lng();
  double distancia = TinyGPSPlus::distanceBetween(lat, lon, safeLat, safeLon);

  if (distancia > SAFE_RADIUS_M) {
    // SALIÓ DE LA ZONA
    if (!isOutsideZone) Serial.println(F("⚠️ SALIENDO DE ZONA -> ALARMA ROJA"));
    isOutsideZone = true;

    if (!alertSent) {
      enviarSMSAlerta(lat, lon, distancia);
      alertSent = true;
    }
  } else {
    // ESTÁ DENTRO
    if (isOutsideZone) Serial.println(F("✅ REGRESO A ZONA -> LED VERDE"));
    isOutsideZone = false;
    alertSent = false; 
  }
}

// ------------------ SMS -----------------------
void inicializarModuloSMS() {
  enviarComandoModem("AT");
  enviarComandoModem("ATE0");
  enviarComandoModem("AT+CMGF=1");
  enviarComandoModem("AT+CSCS=\"GSM\"");
}

void enviarComandoModem(const char* cmd, uint16_t esperaMs) {
  modem.println(cmd);
  unsigned long start = millis();
  while (millis() - start < esperaMs) {
    while (modem.available()) Serial.write(modem.read());
  }
}

void enviarSMSAlerta(double lat, double lon, double distancia) {
  Serial.println(F("Enviando SMS Geocerca..."));
  modem.println("AT+CMGF=1");
  delay(200); 
  modem.print("AT+CMGS=\"");
  modem.print(NUMERO_ALERTA);
  modem.println("\"");
  delay(200);

  String url = "http://maps.google.com/?q=";
  url += String(lat, 6);
  url += ",";
  url += String(lon, 6);

  modem.print("ALERTA: Fuera de zona segura!\r\n");
  modem.print("Distancia: ");
  modem.print(distancia, 1);
  modem.println("m");
  modem.print("Ubicacion: ");
  modem.println(url);

  modem.write(26); 
  Serial.println(F("SMS Enviado."));
}

void imprimirDatos() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint < 2000) return;
  lastPrint = millis();

  if (gps.location.isValid()) {
    Serial.print(F("GPS OK | Distancia Home: "));
    if (homeSet) {
      Serial.print(TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), safeLat, safeLon));
      Serial.print(F("m | Estado: "));
      Serial.println(isOutsideZone ? "FUERA (ROJO PARPADEANDO)" : "DENTRO (VERDE FIJO)");
    } else {
      Serial.println(F("Esperando Configurar Home (>5 Sats)..."));
    }
  } else {
    Serial.println(F("Buscando satélites..."));
  }
}