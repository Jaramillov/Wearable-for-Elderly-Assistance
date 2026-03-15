// === PINES ===
#define LED_R 2      // Rojo (cátodo del LED rojo)
#define LED_G 15     // Verde
#define LED_B 4      // Azul
#define BUZZER 26    // Buzzer entre 3.3V y GPIO26

void setup() {
  Serial.begin(115200);
  Serial.println("Prueba LED RGB (anodo comun) + BUZZER activo en LOW");

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  // IMPORTANTE: como todo es activo en LOW,
  // iniciamos todo APAGADO poniéndolo en HIGH.
  digitalWrite(LED_R, HIGH);
  digitalWrite(LED_G, HIGH);
  digitalWrite(LED_B, HIGH);
  digitalWrite(BUZZER, HIGH);
}

void buzzerOn()  { digitalWrite(BUZZER, LOW); }   // LOW -> suena
void buzzerOff() { digitalWrite(BUZZER, HIGH); }  // HIGH -> se apaga

// Encender colores teniendo en cuenta que el LED es ANODO COMUN:
// ON  -> LOW
// OFF -> HIGH
void setColor(bool rOn, bool gOn, bool bOn) {
  digitalWrite(LED_R, rOn ? LOW : HIGH);
  digitalWrite(LED_G, gOn ? LOW : HIGH);
  digitalWrite(LED_B, bOn ? LOW : HIGH);
}

void loop() {
  // 1) ROJO + beep
  Serial.println("FASE 1: ROJO + beep");
  setColor(true, false, false);  // solo rojo encendido

  buzzerOn();
  delay(300);
  buzzerOff();
  delay(700);

  // 2) VERDE (sin beep)
  Serial.println("FASE 2: VERDE");
  setColor(false, true, false);
  delay(1000);

  // 3) AZUL (sin beep)
  Serial.println("FASE 3: AZUL");
  setColor(false, false, true);
  delay(1000);

  // 4) AMARILLO (ROJO + VERDE)
  Serial.println("FASE 4: AMARILLO");
  setColor(true, true, false);
  delay(1000);

  // 5) MAGENTA (ROJO + AZUL)
  Serial.println("FASE 5: MAGENTA");
  setColor(true, false, true);
  delay(1000);

  // 6) CIAN (VERDE + AZUL)
  Serial.println("FASE 6: CIAN");
  setColor(false, true, true);
  delay(1000);

  // 7) BLANCO (los tres encendidos)
  Serial.println("FASE 7: BLANCO");
  setColor(true, true, true);
  delay(1000);

  // 8) APAGADO
  Serial.println("FASE 8: APAGADO");
  setColor(false, false, false);
  delay(1000);
}
