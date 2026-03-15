# Wearable Assistance Device for Elderly Care

An ESP32-based low-cost wearable device for real-time monitoring and emergency assistance of elderly individuals. Features hardware-accelerated fall detection via IMU sensor fusion, GPS-based localization, dual-channel alerting (SMS/WiFi), and on-device keyword spotting (KWS) using a TinyML model deployed directly on the microcontroller.

---

## Project Overview

The wearable is designed as a compact, unobtrusive clip or pendant that the user can attach to their clothing. The device operates autonomously throughout the day, continuously analyzing inertial data for fall events, listening for voice trigger words, and maintaining a communication link through either cellular SMS or WiFi depending on availability.

The system is optimized for energy efficiency, targeting a minimum 24-hour operational window on a single Li-Po charge, with aggressive power gating of the GPS and cellular modules between events.

---

## Project Objectives

- Continuously monitor inertial data (acceleration + angular velocity) to detect falls and prolonged immobility events with low false-positive rates.
- Transmit GPS coordinates and device status to emergency contacts via SMS (A7670SA module) when WiFi is unavailable.
- Route notifications through Telegram, email, or REST APIs when WiFi connectivity is established.
- Enable the user to manually trigger an SOS alert via a single large button or a recognized voice keyword.
- Maintain a total device weight ≤ 50 g and a form factor ≤ 5 × 5 × 2 cm.
- Sustain a minimum 24-hour operational window under normal usage conditions.

---

## Hardware Stack

| Component | Part | Role |
|-----------|------|------|
| Microcontroller | ESP32 (mini/standard) | Main CPU, WiFi, BT, inference host |
| IMU | MPU-6050 (I²C) | 3-axis accel + 3-axis gyro for fall detection |
| Microphone | INMP441 (I²S) | Digital MEMS mic for keyword spotting |
| Cellular | SIMCom A7670SA (LTE Cat-1 / 2G fallback) | SMS alerts with GPS payload |
| GPS | GY-NEO6M V2 (u-blox NEO-6M, UART) | Position fix (lat/lon/accuracy) |
| Battery | Li-Po 3.7 V (≥ 1000 mAh target) | Primary power source |
| Power Management | LDO + step-up/step-down regulator | Stable 3.3 V rail for ESP32 and peripherals |
| Charging | USB charging IC | In-device USB recharge |
| UI | 1× SOS button, RGB LED, buzzer, optional vibration motor | User feedback and emergency input |

---

## Key Features

### 1. Fall Detection
The MPU-6050 is polled continuously via I²C. A two-stage detection pipeline combines:
- **Threshold gating**: a sudden spike in resultant acceleration (configurable, e.g. > 2.5 g) followed by a low-activity window to distinguish a free-fall + impact signature from normal motion.
- On a fall event: the device triggers an audible/visual alert, then enters a short cancellation window (e.g. 15 s). If the user does not cancel, an SMS (and WiFi notification if available) is dispatched with timestamp, GPS coordinates, and battery level.

### 2. SOS Button
A long-press of the single button immediately dispatches an emergency alert payload: alert type, last valid GPS fix, battery percentage, and device uptime.

### 3. GPS Localization
The GY-NEO6M V2 module communicates over UART using the NMEA-0183 protocol. The firmware parses `$GPRMC` / `$GPGGA` sentences to extract latitude, longitude, and fix quality. To minimize current draw, the GPS can be duty-cycled between periodic fixes.

### 4. On-Device Keyword Spotting (TinyML / Edge Impulse)
Voice command recognition runs entirely on the ESP32 — no cloud inference required. The pipeline is:

- **Sensor**: INMP441 MEMS microphone connected via I²S (SCK → GPIO 25, WS → GPIO 33, SD → GPIO 32).
- **Audio capture**: Raw PCM samples at 16 kHz, 16-bit resolution, captured via the ESP32 I²S DMA interface.
- **Model**: A neural network classifier trained and quantized using [Edge Impulse](https://studio.edgeimpulse.com/public/827176/live). The exported C++ library (`voice_detection_with_INMP441_inferencing.h`) is flashed alongside the firmware.
- **Classes**: `ayuda` (help), `noise`, `unknown`.
- **Decision threshold**: 0.60 confidence score on the `ayuda` label to trigger an alert; otherwise classified as background.

#### Training Methodology
Training data was recorded directly with the INMP441 connected to the ESP32, using the `edge-impulse-data-forwarder` CLI to stream serial audio data into Edge Impulse Studio. Samples were captured across multiple sessions in different acoustic environments (indoor rooms, outdoor spaces, with ambient noise) to improve generalization and reduce environment-specific overfitting. The same hardware path used in production (I²S → ESP32 → serial) was maintained throughout data collection to preserve signal fidelity and ensure the model performs consistently on-device.

The trained impulse was exported as an Arduino-compatible C++ library and embedded into the firmware, enabling real-time inference without external connectivity.

#### Keyword Spotting Code

```cpp
#include <driver/i2s.h>
#include <voice_detection_with_INMP441_inferencing.h>

// I2S pin mapping
#define I2S_WS  33
#define I2S_SD  32
#define I2S_SCK 25
#define I2S_PORT I2S_NUM_0

#define SAMPLE_RATE     16000
#define EI_BUFFER_SIZE  EI_CLASSIFIER_RAW_SAMPLE_COUNT

// RGB LED pins (common anode — LOW = ON)
#define LED_R 4    // Red  → AYUDA detected
#define LED_G 2    // Green
#define LED_B 15   // Blue → background / unknown

#define THRESHOLD 0.60f

int16_t bufferI2S[EI_BUFFER_SIZE];
float   bufferFloat[EI_BUFFER_SIZE];

void setup() {
  Serial.begin(115200);

  pinMode(LED_R, OUTPUT); pinMode(LED_G, OUTPUT); pinMode(LED_B, OUTPUT);
  digitalWrite(LED_R, HIGH); digitalWrite(LED_G, HIGH); digitalWrite(LED_B, HIGH);

  i2s_config_t i2s_config = {
    .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate          = SAMPLE_RATE,
    .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count        = 32,
    .dma_buf_len          = 64,
    .use_apll             = false
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num   = I2S_SCK,
    .ws_io_num    = I2S_WS,
    .data_out_num = -1,
    .data_in_num  = I2S_SD
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
  i2s_zero_dma_buffer(I2S_PORT);
  delay(300);
}

void loop() {
  size_t bytes_read = 0;
  i2s_read(I2S_PORT, (void *)bufferI2S,
           EI_BUFFER_SIZE * sizeof(int16_t), &bytes_read, portMAX_DELAY);

  if ((int)(bytes_read / sizeof(int16_t)) != EI_BUFFER_SIZE) return;

  // Normalize PCM to [-1.0, 1.0]
  for (int i = 0; i < EI_BUFFER_SIZE; i++)
    bufferFloat[i] = (float)bufferI2S[i] / 32768.0f;

  signal_t signal;
  ei::numpy::signal_from_buffer(bufferFloat, EI_BUFFER_SIZE, &signal);

  ei_impulse_result_t result;
  if (run_classifier(&signal, &result, false) != EI_IMPULSE_OK) return;

  float ayuda = 0, noise = 0, unknown = 0;
  for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    String label = result.classification[i].label;
    if      (label == "ayuda")   ayuda   = result.classification[i].value;
    else if (label == "noise")   noise   = result.classification[i].value;
    else if (label == "unknown") unknown = result.classification[i].value;
  }

  // Threshold decision
  if (ayuda >= THRESHOLD) {
    digitalWrite(LED_R, LOW);  digitalWrite(LED_G, HIGH); digitalWrite(LED_B, HIGH);
    Serial.println("CLASSIFIED: AYUDA → trigger SOS");
  } else {
    digitalWrite(LED_R, HIGH); digitalWrite(LED_G, HIGH); digitalWrite(LED_B, LOW);
    Serial.println("CLASSIFIED: BACKGROUND (noise + unknown)");
  }

  Serial.print("ayuda: ");  Serial.println(ayuda,           3);
  Serial.print("others: "); Serial.println(noise + unknown, 3);
  Serial.println("---");
}
```

### 5. Power Management
The firmware implements event-driven power gating: the GPS and A7670SA modules are kept in low-power or sleep mode during idle periods and only activated when a fall event, SOS press, or voice keyword is detected. The ESP32 itself can enter light-sleep between IMU polling cycles to reduce average current consumption below the 100 mA target.

---

## Requirements & Constraints

| Parameter | Target |
|-----------|--------|
| Battery autonomy | ≥ 24 h normal use |
| Maximum weight | ≤ 50 g |
| Form factor | ≤ 5 × 5 × 2 cm |
| BOM cost | ≤ USD 50 |
| Average current draw | < 100 mA |
| Inference latency | Real-time (< 1 inference window ≈ 1 s) |
| Operating temperature | 0 – 45 °C |

---

## System Operation Flow

```
Power on
    │
    ▼
Monitoring mode
  ├─ IMU polling (fall detection)
  ├─ KWS inference loop (INMP441 → Edge Impulse model)
  └─ GPS periodic fix acquisition
    │
    ├─ Fall detected?  ──► Buzzer + LED alert ──► 15 s cancellation window
    │                                                  │
    │                                          No cancel?
    │                                                  │
    ├─ SOS button pressed?  ──────────────────────────►│
    │                                                  │
    └─ "ayuda" keyword (≥ 0.60)?  ────────────────────►▼
                                               Build alert payload
                                        (type, GPS fix, battery %, timestamp)
                                                       │
                                            WiFi available?
                                           Yes ◄──────┤──────► No
                                            │                   │
                                   Telegram / REST API      SMS via A7670SA
```

---

## Edge Impulse Public Project

The trained model, dataset, DSP configuration, and classifier metrics are publicly accessible:

**[https://studio.edgeimpulse.com/public/827176/live](https://studio.edgeimpulse.com/public/827176/live)**
