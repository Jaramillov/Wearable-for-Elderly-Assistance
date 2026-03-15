// === PINES ===
#define LED_G 2      // Verde
#define LED_B 15     // Azul
#define LED_R 4      // Rojo


// Tiempo que durará encendido el LED (en milisegundos)
const int TIEMPO_ENCENDIDO = 2000; 

void setup() {
  Serial.begin(115200);
  Serial.println("--- CONTROL RGB POR SERIAL ---");
  Serial.println("Escribe: 'r' (Rojo), 'g' (Verde) o 'b' (Azul)");

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  // ESTADO INICIAL: Todo apagado 
  // (HIGH apaga porque es Ánodo Común)
  digitalWrite(LED_R, HIGH);
  digitalWrite(LED_G, HIGH);
  digitalWrite(LED_B, HIGH);
}

// Función para controlar el LED Anodo Común
// true = Encender (LOW), false = Apagar (HIGH)
void setColor(bool rOn, bool gOn, bool bOn) {
  digitalWrite(LED_R, rOn ? LOW : HIGH);
  digitalWrite(LED_G, gOn ? LOW : HIGH);
  digitalWrite(LED_B, bOn ? LOW : HIGH);
}

void loop() {
  // Verificamos si hay datos en el monitor serial
  if (Serial.available() > 0) {
    char comando = Serial.read();

    // Ignoramos saltos de línea o espacios
    if (comando == '\n' || comando == '\r' || comando == ' ') return;

    Serial.print("Comando recibido: ");
    Serial.println(comando);

    switch (comando) {
      case 'r': // ROJO
        Serial.println("-> Encendiendo ROJO");
        setColor(true, false, false);
        delay(TIEMPO_ENCENDIDO);
        break;

      case 'g': // VERDE
        Serial.println("-> Encendiendo VERDE");
        setColor(false, true, false);
        delay(TIEMPO_ENCENDIDO);
        break;

      case 'b': // AZUL
        Serial.println("-> Encendiendo AZUL");
        setColor(false, false, true);
        delay(TIEMPO_ENCENDIDO);
        break;

      default:
        Serial.println("Comando no reconocido. Usa 'r', 'g' o 'b'.");
        break;
    }

    // Al terminar el tiempo, apagamos todo
    setColor(false, false, false);
  }
}