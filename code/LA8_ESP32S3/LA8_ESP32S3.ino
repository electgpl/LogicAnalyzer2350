/*
  GEN_Electgpl_S3 — Generador free-run de tramas UART / SPI / I2C / SMBus (ESP32-S3)
  ---------------------------------------------------------------------------------
  Se alimenta y transmite solo, en ciclo continuo, como una placa de demo de
  osciloscopio. No requiere configuración ni dispositivos esclavos.

  Arduino IDE: core esp32 (Espressif), placa "ESP32S3 Dev Module".

  Pines y canales del LA8:
    UART  TX  GPIO17  -> CH0   115200 8N1, "Electgpl\r\n"
    SPI   SCK GPIO12  -> CH1   modo 0, 1 MHz, "Electgpl" con CS activo
          MOSI GPIO11 -> CH2
          CS  GPIO10  -> CH3
    I2C   SDA GPIO8   -> CH4   100 kHz, escritura a 0x3C con "Electgpl"
          SCL GPIO9   -> CH5
    SMBus SDA GPIO4   -> CH6   100 kHz, Block Write "Electgpl" + PEC y Read Word + PEC
          SCL GPIO5   -> CH7   (dirección 0x0B, "Smart Battery")
    I2S   BCLK GPIO6  -> CH1   192 kHz, 16 bits estéreo (Philips): L = seno 1 kHz, R = seno 2 kHz
          SD  GPIO15  -> CH2   (para probar I2S, mover las puntas CH1–CH3 desde el SPI)
          WS  GPIO7   -> CH3
    GND               -> GND del LA8

  I2C y SMBus se generan por software y el propio generador emite los ACK del
  "esclavo" (y los datos de respuesta en Read Word), así que las transacciones
  aparecen completas sin ningún dispositivo conectado. Las salidas son push-pull:
  no conectar estos pines a un bus I2C real.

  ELECTGPL
*/

#include <Arduino.h>
#include <SPI.h>
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  #include <ESP_I2S.h>          // core 3.x (ESP-IDF 5)
#else
  #include <driver/i2s.h>       // core 2.x (ESP-IDF 4.4, driver legacy)
#endif

// ============================ CONFIGURACIÓN ============================
#define MESSAGE       "Electgpl"
#define CYCLE_MS      5           // período del ciclo completo de tramas

#define PIN_UART_TX   17
#define PIN_UART_RX   18          // no se usa
#define UART_BAUD     115200

#define PIN_SPI_SCK   12
#define PIN_SPI_MOSI  11
#define PIN_SPI_MISO  13          // no se usa
#define PIN_SPI_CS    10
#define SPI_FREQ      1000000
#define SPI_MODE_SEL  SPI_MODE0

#define PIN_I2C_SDA   8
#define PIN_I2C_SCL   9
#define I2C_FREQ      100000
#define I2C_ADDR      0x3C

#define PIN_SMB_SDA   4
#define PIN_SMB_SCL   5
#define SMB_FREQ      100000
#define SMB_ADDR      0x0B
#define SMB_CMD_BLOCK 0x20        // comando para el Block Write
#define SMB_CMD_WORD  0x09        // comando para el Read Word (Voltage, Smart Battery)
#define SMB_WORD      0x1234      // dato que "responde" el esclavo emulado

#define I2S_ENABLE    1           // 0 = sin salida I2S
#define PIN_I2S_BCLK  6
#define PIN_I2S_WS    7
#define PIN_I2S_DOUT  15
#define I2S_FS        192000      // 44100, 48000, 96000 o 192000 Hz
#define I2S_BITS      16          // 16 o 32 bits por muestra (= ancho de slot)
                                  // BCLK = I2S_FS x 2 x I2S_BITS -> 192 kHz/16 bits = 6,144 MHz
                                  //                               192 kHz/32 bits = 12,288 MHz
#define I2S_LEVEL     0.30f       // amplitud de los tonos, fracción de la escala completa
// =======================================================================

SPIClass spi(FSPI);

// ---------------- I2S: tono continuo generado por DMA desde una tarea en el core 0 ----------------
// Tabla de 10 ms (I2S_FS / 100 cuadros, entero también para 44,1 kHz):
// L = 1 kHz (10 ciclos), R = 2 kHz (20 ciclos), independiente de la frecuencia de muestreo
#if I2S_BITS == 32
typedef int32_t i2s_sample_t;
  #define I2S_FULLSCALE  2147483647.0f
#else
typedef int16_t i2s_sample_t;
  #define I2S_FULLSCALE  32767.0f
#endif
#define I2S_TBL_FRAMES  (I2S_FS / 100)
static i2s_sample_t i2s_tbl[I2S_TBL_FRAMES * 2];

#if ESP_ARDUINO_VERSION_MAJOR >= 3
static I2SClass i2s;
static bool i2s_start() {
  i2s.setPins(PIN_I2S_BCLK, PIN_I2S_WS, PIN_I2S_DOUT);
  return i2s.begin(I2S_MODE_STD, I2S_FS,
                   I2S_BITS == 32 ? I2S_DATA_BIT_WIDTH_32BIT : I2S_DATA_BIT_WIDTH_16BIT,
                   I2S_SLOT_MODE_STEREO);
}
static void i2s_push() { i2s.write((uint8_t*)i2s_tbl, sizeof(i2s_tbl)); }   // bloquea hasta que hay lugar
#else
static bool i2s_start() {
  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  cfg.sample_rate = I2S_FS;
  cfg.bits_per_sample = (I2S_BITS == 32) ? I2S_BITS_PER_SAMPLE_32BIT : I2S_BITS_PER_SAMPLE_16BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.dma_buf_count = 6;
  cfg.dma_buf_len = 240;
  cfg.tx_desc_auto_clear = true;
  i2s_pin_config_t pins = {};
  pins.mck_io_num = I2S_PIN_NO_CHANGE;
  pins.bck_io_num = PIN_I2S_BCLK;
  pins.ws_io_num = PIN_I2S_WS;
  pins.data_out_num = PIN_I2S_DOUT;
  pins.data_in_num = I2S_PIN_NO_CHANGE;
  if (i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL) != ESP_OK) return false;
  return i2s_set_pin(I2S_NUM_0, &pins) == ESP_OK;
}
static void i2s_push() {
  size_t written;
  i2s_write(I2S_NUM_0, i2s_tbl, sizeof(i2s_tbl), &written, portMAX_DELAY);
}
#endif

static void i2s_task(void*) {
  for (;;) i2s_push();
}
static uint8_t cnt = 0;

// ---------------- retardo preciso por contador de ciclos de CPU ----------------
static inline void delay_ns(uint32_t ns) {
  const uint32_t cycles = (uint32_t)((uint64_t)ns * ESP.getCpuFreqMHz() / 1000);
  const uint32_t t0 = ESP.getCycleCount();
  while (ESP.getCycleCount() - t0 < cycles) { }
}

// ---------------- I2C por software (push-pull, con ACK emulado) ----------------
struct SoftI2C {
  int sda, scl;
  uint32_t q;                                   // cuarto de período en ns
  void begin(int sda_, int scl_, uint32_t freq) {
    sda = sda_; scl = scl_; q = 250000000UL / freq;
    pinMode(sda, OUTPUT); pinMode(scl, OUTPUT);
    digitalWrite(sda, HIGH); digitalWrite(scl, HIGH);
  }
  void start()  { digitalWrite(sda, HIGH); digitalWrite(scl, HIGH); delay_ns(q);
                  digitalWrite(sda, LOW);  delay_ns(q); digitalWrite(scl, LOW); delay_ns(q); }
  void rstart() { digitalWrite(sda, HIGH); delay_ns(q); digitalWrite(scl, HIGH); delay_ns(q);
                  digitalWrite(sda, LOW);  delay_ns(q); digitalWrite(scl, LOW);  delay_ns(q); }
  void stop()   { digitalWrite(sda, LOW);  delay_ns(q); digitalWrite(scl, HIGH); delay_ns(q);
                  digitalWrite(sda, HIGH); delay_ns(2 * q); }
  void send_bit(int b) { digitalWrite(sda, b ? HIGH : LOW); delay_ns(q);
                    digitalWrite(scl, HIGH); delay_ns(2 * q);
                    digitalWrite(scl, LOW);  delay_ns(q); }
  void byte(uint8_t v, bool ack = true) {
    for (int i = 7; i >= 0; i--) send_bit((v >> i) & 1);
    send_bit(ack ? 0 : 1);                         // 9.º bit: ACK (0) o NACK (1)
  }
};

static SoftI2C i2c, smb;

static uint8_t smbus_pec(const uint8_t* d, size_t n) {
  uint8_t crc = 0;
  for (size_t i = 0; i < n; i++) {
    crc ^= d[i];
    for (int b = 0; b < 8; b++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
  }
  return crc;
}

// ---------------- tramas ----------------
static void frame_uart() {
  Serial1.print(MESSAGE "\r\n");               // la UART transmite por hardware en segundo plano
}

static void frame_spi() {
  spi.beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE_SEL));
  digitalWrite(PIN_SPI_CS, LOW);
  spi.writeBytes((const uint8_t*)MESSAGE, strlen(MESSAGE));
  digitalWrite(PIN_SPI_CS, HIGH);
  spi.endTransaction();
}

static void frame_i2c() {
  // START | 0x3C W | "Electgpl" | STOP
  i2c.start();
  i2c.byte((uint8_t)(I2C_ADDR << 1));
  for (const char* p = MESSAGE; *p; p++) i2c.byte((uint8_t)*p);
  i2c.stop();
}

static void frame_smbus() {
  const uint8_t aw = (uint8_t)(SMB_ADDR << 1), ar = aw | 1;
  const uint8_t n = (uint8_t)strlen(MESSAGE);

  // Block Write: START | addrW | cmd | count | data... | PEC | STOP
  uint8_t pk[40];
  size_t k = 0;
  pk[k++] = aw; pk[k++] = SMB_CMD_BLOCK; pk[k++] = n;
  for (uint8_t i = 0; i < n; i++) pk[k++] = (uint8_t)MESSAGE[i];
  const uint8_t pec_w = smbus_pec(pk, k);
  smb.start();
  for (size_t i = 0; i < k; i++) smb.byte(pk[i]);
  smb.byte(pec_w);
  smb.stop();

  delay_ns(100000);

  // Read Word: START | addrW | cmd | Sr | addrR | lo | hi | PEC (NACK) | STOP
  const uint8_t lo = SMB_WORD & 0xFF, hi = (SMB_WORD >> 8) & 0xFF;
  const uint8_t rd[5] = { aw, SMB_CMD_WORD, ar, lo, hi };
  smb.start();
  smb.byte(aw);
  smb.byte(SMB_CMD_WORD);
  smb.rstart();
  smb.byte(ar);
  smb.byte(lo);                                 // el master hace ACK de los datos
  smb.byte(hi);
  smb.byte(smbus_pec(rd, 5), false);            // y NACK al último byte (PEC)
  smb.stop();
}

static void print_map() {
  Serial.println("\nGEN_Electgpl_S3 - generador free-run");
  Serial.printf("  UART  TX=GPIO%d -> CH0  %d 8N1 \"%s\\r\\n\"\n", PIN_UART_TX, UART_BAUD, MESSAGE);
  Serial.printf("  SPI   SCK=GPIO%d MOSI=GPIO%d CS=GPIO%d -> CH1/CH2/CH3  %d Hz modo 0\n",
                PIN_SPI_SCK, PIN_SPI_MOSI, PIN_SPI_CS, SPI_FREQ);
  Serial.printf("  I2C   SDA=GPIO%d SCL=GPIO%d -> CH4/CH5  %d Hz  addr 0x%02X\n",
                PIN_I2C_SDA, PIN_I2C_SCL, I2C_FREQ, I2C_ADDR);
  Serial.printf("  SMBus SDA=GPIO%d SCL=GPIO%d -> CH6/CH7  %d Hz  addr 0x%02X (Block Write + Read Word, PEC)\n",
                PIN_SMB_SDA, PIN_SMB_SCL, SMB_FREQ, SMB_ADDR);
  if (I2S_ENABLE)
    Serial.printf("  I2S   BCLK=GPIO%d WS=GPIO%d SD=GPIO%d  %d Hz %d bits estéreo, BCLK %.3f MHz "
                  "(L 1 kHz, R 2 kHz)\n", PIN_I2S_BCLK, PIN_I2S_WS, PIN_I2S_DOUT, I2S_FS, I2S_BITS,
                  I2S_FS * 2.0 * I2S_BITS / 1e6);
  Serial.printf("  Ciclo cada %d ms\n", CYCLE_MS);
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(UART_BAUD, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);
  spi.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, -1);
  pinMode(PIN_SPI_CS, OUTPUT);
  digitalWrite(PIN_SPI_CS, HIGH);
  i2c.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_FREQ);
  smb.begin(PIN_SMB_SDA, PIN_SMB_SCL, SMB_FREQ);
  if (I2S_ENABLE) {
    const float a = I2S_LEVEL * I2S_FULLSCALE;
    for (int k = 0; k < I2S_TBL_FRAMES; k++) {
      const float t = (float)k / (float)I2S_FS;
      i2s_tbl[2 * k]     = (i2s_sample_t)(a * sinf(2.0f * PI * 1000.0f * t));   // L: 1 kHz
      i2s_tbl[2 * k + 1] = (i2s_sample_t)(a * sinf(2.0f * PI * 2000.0f * t));   // R: 2 kHz
    }
    if (i2s_start()) xTaskCreatePinnedToCore(i2s_task, "i2s", 4096, NULL, 2, NULL, 0);
    else Serial.println("ERROR: no se pudo iniciar I2S");
  }
  delay(500);
  print_map();
}

void loop() {
  const uint32_t t0 = millis();

  frame_uart();                // arranca la UART (corre en paralelo por hardware)
  frame_spi();
  delayMicroseconds(100);
  frame_i2c();
  delayMicroseconds(100);
  frame_smbus();
  cnt++;

  if (Serial.available()) {    // cualquier tecla en la consola reimprime el mapa de pines
    while (Serial.available()) Serial.read();
    print_map();
  }
  while (millis() - t0 < CYCLE_MS) delay(1);
}
