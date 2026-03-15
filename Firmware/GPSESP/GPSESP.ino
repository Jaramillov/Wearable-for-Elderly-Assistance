#include <TinyGPS++.h>

TinyGPSPlus gps;

// Pines para el GPS
static const int RXPin = 18;  // GPS TX -> ESP32 GPIO16 (RX2)
static const int TXPin = 19;  // GPS RX -> ESP32 GPIO17 (TX2)
static const uint32_t GPSBaud = 9600;

// Zona horaria Colombia (UTC -5)
const int TIMEZONE_OFFSET = -5;

void setup() {
  Serial.begin(115200);
  Serial.println(F("=== ESP32 + GPS NEO-6M + TinyGPS++ ==="));
  Serial.println(F("Esperando señal GPS... Sal al exterior si no ves fix."));
  Serial.println();

  // Iniciar UART2 con los pines indicados
  Serial2.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);
}

void loop() {
  // Leer todos los caracteres que vengan del GPS
  while (Serial2.available() > 0) {
    char c = Serial2.read();
    gps.encode(c);

    // Cada vez que haya una actualización de ubicación, imprimimos
    if (gps.location.isUpdated()) {
      imprimirDatos();
    }
  }

  // También imprimir cada 2 segundos aunque no haya update
  static unsigned long ultimaImpresion = 0;
  if (millis() - ultimaImpresion > 2000) {
    imprimirDatos();
    ultimaImpresion = millis();
  }
}

void imprimirDatos() {
  Serial.println(F("--------------- GPS ---------------"));

  // 1️⃣ Posición
  if (gps.location.isValid()) {
    Serial.print(F("Latitud : "));
    Serial.println(gps.location.lat(), 6);
    Serial.print(F("Longitud: "));
    Serial.println(gps.location.lng(), 6);
  } else {
    Serial.println(F("Lat/Long: sin fix aún"));
  }

  // 2️⃣ Satélites
  if (gps.satellites.isValid()) {
    Serial.print(F("Satélites: "));
    Serial.println(gps.satellites.value());
  } else {
    Serial.println(F("Satélites: no disponible"));
  }

  // 3️⃣ Precisión (HDOP)
  if (gps.hdop.isValid()) {
    Serial.print(F("HDOP (precisión): "));
    Serial.println(gps.hdop.value());
  }

  // 4️⃣ Fecha y hora LOCAL (UTC-5)
  if (gps.date.isValid() && gps.time.isValid()) {
    int hour = gps.time.hour() + TIMEZONE_OFFSET;
    int day = gps.date.day();
    int month = gps.date.month();
    int year = gps.date.year();

    if (hour < 0) {
      hour += 24;
      day -= 1; // Simplificado
    }

    Serial.print(F("Fecha: "));
    Serial.print(day);
    Serial.print(F("/"));
    Serial.print(month);
    Serial.print(F("/"));
    Serial.println(year);

    Serial.print(F("Hora local: "));
    if (hour < 10) Serial.print(F("0"));
    Serial.print(hour);
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.println(gps.time.second());
  } else {
    Serial.println(F("Fecha/Hora: no disponible"));
  }

  // 5️⃣ Velocidad
  if (gps.speed.isValid()) {
    Serial.print(F("Velocidad: "));
    Serial.print(gps.speed.kmph());
    Serial.println(F(" km/h"));
  } else {
    Serial.println(F("Velocidad: no disponible"));
  }

  // 6️⃣ Altura
  if (gps.altitude.isValid()) {
    Serial.print(F("Altura: "));
    Serial.print(gps.altitude.meters());
    Serial.println(F(" m"));
  } else {
    Serial.println(F("Altura: no disponible"));
  }

  // 7️⃣ Rumbo (opcional)
  if (gps.course.isValid()) {
    Serial.print(F("Rumbo: "));
    Serial.print(gps.course.deg());
    Serial.println(F(" grados"));
  }

  Serial.println(F("-----------------------------------\n"));
}
