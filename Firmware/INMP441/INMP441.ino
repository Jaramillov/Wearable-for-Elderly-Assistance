#include <driver/i2s.h>

// --- Configuración de Pines I2S ---
#define I2S_WS 33
#define I2S_SD 32
#define I2S_SCK 25

// --- Puerto I2S ---
#define I2S_PORT I2S_NUM_0

// --- Pines LED RGB (ÁNODO COMÚN) ---
#define LED_R 4
#define LED_G 2
#define LED_B 15

// Configuración del buffer
#define bufferLen 64
int16_t sBuffer[bufferLen];

void setup() {
  Serial.begin(115200);
  Serial.println("Inicializando INMP441...");

  // ========================================
  // APAGAR TODOS LOS LEDs (ÁNODO COMÚN)
  // ========================================
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  
  // Para ánodo común: HIGH = apagado
  digitalWrite(LED_R, HIGH);
  digitalWrite(LED_G, HIGH);
  digitalWrite(LED_B, HIGH);
  
  Serial.println("LEDs desactivados para protección.");

  // Inicializar I2S
  i2s_install();
  i2s_setpin();
  i2s_start(I2S_PORT);
  
  delay(500);
  Serial.println("Micrófono listo. Abre el Serial Plotter.");
}

void loop() {
  size_t bytesIn = 0;
  esp_err_t result = i2s_read(I2S_PORT, &sBuffer, bufferLen * sizeof(int16_t), &bytesIn, portMAX_DELAY);

  if (result == ESP_OK) {
    // Leemos el buffer y enviamos las muestras al Serial Plotter
    for (int i = 0; i < bytesIn / sizeof(int16_t); i++) {
      // Imprimimos el valor para ver la onda en el plotter
      Serial.println(sBuffer[i]);
    }
  }
}

void i2s_install() {
  const i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX), // Modo Maestro, Recepción
    .sample_rate = 16000,                              // Frecuencia de muestreo
    .bits_per_sample = i2s_bits_per_sample_t(16),      // 16 bits por muestra
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,       // Solo canal izquierdo
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
    .intr_alloc_flags = 0,                             // Interrupción por defecto
    .dma_buf_count = 32,
    .dma_buf_len = bufferLen,
    .use_apll = false
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
}

void i2s_setpin() {
  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = -1, // No usamos salida de audio
    .data_in_num = I2S_SD
  };

  i2s_set_pin(I2S_PORT, &pin_config);
}