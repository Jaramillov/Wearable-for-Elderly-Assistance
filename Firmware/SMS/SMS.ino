/*
 * PROYECTO: Pasarela Serial ESP32 (30 pines) <-> A7670SA
 * AUTOR: Ingeniería Electrónica UNAL (Estudiante)
 * DESCRIPCIÓN: Enciende el módulo A7670SA automáticamente (si hay PWRKEY)
 * y establece un puente de comunicación para comandos AT y recepción de SMS.
 */

#include <HardwareSerial.h>

// --- DEFINICIÓN DE PINES (ESP32 30 pines DevKit V1) ---
// RX2  -> GPIO16 (conectar al TXD del A7670SA)
// TX2  -> GPIO17 (conectar al RXD del A7670SA)
const int RX_PIN = 16;      // RX2 del ESP32
const int TX_PIN = 17;      // TX2 del ESP32

// Si NO tienes el PWRKEY cableado, lo dejamos en -1 y no lo usamos
// Si algún día lo conectas, pon aquí el GPIO que uses, por ejemplo 4, 5, etc.
const int PWRKEY_PIN = -1;  // -1 = sin control de encendido por software

// --- PINES LED RGB (ÁNODO COMÚN - HIGH apaga, LOW enciende) ---
#define LED_R 2      // Rojo (cátodo del LED rojo)
#define LED_G 15     // Verde
#define LED_B 4      // Azul

// Instancia del UART hardware #2 (Serial2 en ESP32 clásico)
HardwareSerial gsmSerial(2);

// Variables de estado
bool moduloEncendido = false;

void encenderModulo() {
  // Si no hay pin de PWRKEY, no hacemos nada
  if (PWRKEY_PIN < 0) {
    Serial.println("--- PWRKEY deshabilitado (PWRKEY_PIN = -1) ---");
    Serial.println("Enciende el módulo A7670SA manualmente.");
    return;
  }

  Serial.println("--- INICIANDO SECUENCIA DE ENCENDIDO ---");
  Serial.println("Estado: Enviando pulso de arranque al pin K...");

  // El A7670SA se enciende con un pulso BAJO (LOW) en PWRKEY
  // Aseguramos estado alto primero
  digitalWrite(PWRKEY_PIN, HIGH);
  delay(500);
  
  // Pulso de encendido (simula presionar el botón)
  digitalWrite(PWRKEY_PIN, LOW);
  delay(2500); // Mínimo 2 segundos según datasheet
  digitalWrite(PWRKEY_PIN, HIGH); // Soltamos el botón
  
  Serial.println("Estado: Pulso completado. Esperando arranque del SO del módulo...");
  
  // Esperamos unos segundos para que el módulo cargue y encuentre red
  delay(6000); 
  
  Serial.println("--- MÓDULO LISTO PARA COMANDOS ---");
  Serial.println("Prueba escribiendo: AT");
  moduloEncendido = true;
}

void enviarSMS(String numero, String mensaje) {
  Serial.println("--- ENVIANDO SMS ---");
  
  // 1. Configurar modo texto (Obligatorio)
  gsmSerial.println("AT+CMGF=1"); 
  delay(200);
  
  // 2. Iniciar comando de envío con el número destino
  gsmSerial.print("AT+CMGS=\"");
  gsmSerial.print(numero);
  gsmSerial.println("\"");
  
  delay(200); // Esperar a que el módulo responda con el signo '>'
  
  // 3. Escribir el cuerpo del mensaje
  gsmSerial.print(mensaje);
  
  delay(200);
  
  // 4. EL PASO MÁGICO: Enviar el código ASCII 26 (CTRL+Z)
  gsmSerial.write(26); 
  
  Serial.println("Instrucción enviada. Esperando confirmación del módulo...");
}

void setup() {
  // 1. Iniciar comunicación con PC (UART0 por USB)
  Serial.begin(115200);
  delay(3000); // Tiempo para que conecte el monitor serie

  Serial.println("=== Pasarela ESP32 (30 pines) <-> A7670SA ===");

  // 2. APAGAR TODOS LOS LEDs (ÁNODO COMÚN - HIGH apaga)
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  
  digitalWrite(LED_R, HIGH);  // HIGH apaga el LED (ánodo común)
  digitalWrite(LED_G, HIGH);  // HIGH apaga el LED (ánodo común)
  digitalWrite(LED_B, HIGH);  // HIGH apaga el LED (ánodo común)
  
  Serial.println("LEDs RGB apagados (protección del PCB - ánodo común).");

  // 3. Configurar pin de encendido SOLO si está definido
  if (PWRKEY_PIN >= 0) {
    pinMode(PWRKEY_PIN, OUTPUT);
    digitalWrite(PWRKEY_PIN, HIGH); // Estado inactivo por defecto
  } else {
    Serial.println("PWRKEY_PIN = -1 -> No se controlará el encendido por software.");
  }

  // 4. Iniciar comunicación con el módulo GSM en UART2
  // Velocidad estándar de fábrica del A7670SA es 115200
  gsmSerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial.println("UART2 inicializado (RX2=GPIO16, TX2=GPIO17).");

  // 5. Intentar encendido solo si hay PWRKEY
  encenderModulo();

  Serial.println("Listo. Escribe comandos AT en el monitor serie.");
  Serial.println("Escribe 'S' o 's' para enviar el SMS de prueba.");
}

void loop() {
  // ESCUCHAR AL MÓDULO (Lo que responde el A7670SA)
  while (gsmSerial.available()) {
    Serial.write(gsmSerial.read());
  }

  // ESCUCHAR AL PC (Lo que tú escribes)
  while (Serial.available()) {
    char c = Serial.read();
    
    // Si escribes 'S' o 's', detonamos el envío del SMS
    if (c == 'S' || c == 's') {
      // CAMBIA ESTE NÚMERO POR EL TUYO REAL
      enviarSMS("+573103435841", "Hola! El modulo A7670SA funciona correctamente");
    } else {
      // Cualquier otra letra se envía normal como comando AT
      gsmSerial.write(c);
    }
  }
}