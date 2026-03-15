#include <TinyGPS++.h>

TinyGPSPlus gps;

// Pines GPS (ajusta si usas otros)
static const int RXPin = 18;  // GPS TX -> ESP32 RX2
static const int TXPin = 19;  // GPS RX -> ESP32 TX2
static const uint32_t GPSBaud = 9600;

// Zona horaria Colombia (UTC -5)
const int TIMEZONE_OFFSET = -5;

// Punto seguro (se configurará automáticamente)
double safeLat = 0.0;
double safeLon = 0.0;
bool homeSet = false;

// Radio seguro en metros
const double SAFE_RADIUS_M = 10.0;

void setup() {
  Serial.begin(115200);
  Serial.println(F("=== ESP32 + GPS NEO-6M + Geocerca Auto-Home (Sats>5) ==="));

  Serial2.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);

  pinMode(ALERT_PIN, OUTPUT);
  digitalWrite(ALERT_PIN, LOW);
}

void loop() {
  while (Serial2.available() > 0) {
    char c = Serial2.read();
    gps.encode(c);

    if (gps.location.isUpdated()) {
      configurarHomeSiEsNecesario();
      procesarGeocerca();
      imprimirDatos();
    }
  }
}

// Configura la posición "segura" automáticamente con el primer fix bueno
void configurarHomeSiEsNecesario() {
  if (homeSet) return;                 // ya está configurado
  if (!gps.location.isValid()) return; // aún no hay fix de posición

  // ✅ ahora la condición es: satélites válidos y > 5
  if (!gps.satellites.isValid()) return;
  if (gps.satellites.value() <= 3) {
    // aún pocos satélites, esperamos
    return;
  }

  // Primera posición válida con >5 satélites → se convierte en la posición segura
  safeLat = gps.location.lat();
  safeLon = gps.location.lng();
  homeSet = true;

  Serial.println(F("🏠 Punto seguro configurado automáticamente (Sats > 3):"));
  Serial.print(F("   Lat: ")); Serial.println(safeLat, 6);
  Serial.print(F("   Lon: ")); Serial.println(safeLon, 6);
}

// Calcula distancia y decide si está dentro o fuera del perímetro
void procesarGeocerca() {
  if (!homeSet) {
    Serial.println(F("Punto seguro AÚN no configurado. Esperando primer fix con Sats > 4..."));
    digitalWrite(ALERT_PIN, LOW);
    return;
  }

  if (!gps.location.isValid()) {
    Serial.println(F("Sin fix actual. No se puede evaluar geocerca."));
    digitalWrite(ALERT_PIN, LOW);
    return;
  }

  double lat = gps.location.lat();
  double lon = gps.location.lng();

  double distancia = TinyGPSPlus::distanceBetween(lat, lon, safeLat, safeLon);

  Serial.print(F("Distancia al punto seguro: "));
  Serial.print(distancia);
  Serial.println(F(" m"));

  if (distancia > SAFE_RADIUS_M) {
    Serial.println(F("⚠️ FUERA DE LA ZONA SEGURA"));
    digitalWrite(ALERT_PIN, HIGH);
  } else {
    Serial.println(F("✅ Dentro de la zona segura"));
    digitalWrite(ALERT_PIN, LOW);
  }
}

void imprimirDatos() {
  Serial.println(F("--------------- GPS ---------------"));

  // Posición
  if (gps.location.isValid()) {
    Serial.print(F("Latitud : "));
    Serial.println(gps.location.lat(), 6);
    Serial.print(F("Longitud: "));
    Serial.println(gps.location.lng(), 6);
  } else {
    Serial.println(F("Lat/Long: sin fix aún"));
  }

  // Info del punto seguro
  if (homeSet) {
    Serial.print(F("Home Lat: "));
    Serial.println(safeLat, 6);
    Serial.print(F("Home Lon: "));
    Serial.println(safeLon, 6);
  } else {
    Serial.println(F("Home: (aún no configurado)"));
  }

  // Satélites
  if (gps.satellites.isValid()) {
    Serial.print(F("Satélites: "));
    Serial.println(gps.satellites.value());
  }

  // HDOP (solo informativo, ya no decide nada)
  if (gps.hdop.isValid()) {
    Serial.print(F("HDOP (precisión): "));
    Serial.println(gps.hdop.value());
  }

  // Fecha y hora local
  if (gps.date.isValid() && gps.time.isValid()) {
    int hour = gps.time.hour() + TIMEZONE_OFFSET;
    int day = gps.date.day();
    int month = gps.date.month();
    int year = gps.date.year();

    if (hour < 0) {
      hour += 24;
      day -= 1; // simplificado
    }

    Serial.print(F("Fecha: "));
    Serial.print(day); Serial.print("/");
    Serial.print(month); Serial.print("/");
    Serial.println(year);

    Serial.print(F("Hora local: "));
    if (hour < 10) Serial.print('0');
    Serial.print(hour); Serial.print(':');
    if (gps.time.minute() < 10) Serial.print('0');
    Serial.print(gps.time.minute()); Serial.print(':');
    if (gps.time.second() < 10) Serial.print('0');
    Serial.println(gps.time.second());
  }

  // Velocidad
  if (gps.speed.isValid()) {
    Serial.print(F("Velocidad: "));
    Serial.print(gps.speed.kmph());
    Serial.println(F(" km/h"));
  }

  // Altura
  if (gps.altitude.isValid()) {
    Serial.print(F("Altura: "));
    Serial.print(gps.altitude.meters());
    Serial.println(F(" m"));
  }

  Serial.println(F("-----------------------------------\n"));
}
