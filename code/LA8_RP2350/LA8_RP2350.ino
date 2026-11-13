/*
  LA8_Electgpl — Analizador lógico de 8 canales para RP2040 / RP2350
  -------------------------------------------------------------------
  Core      : Earle Philhower "Raspberry Pi Pico/RP2040/RP2350" (arduino-pico >= 4.x)
  Placa     : Raspberry Pi Pico (RP2040) o Pico 2 (RP2350)
  USB Stack : Pico SDK (por defecto)

  Arquitectura
   - PIO SM "captura": 1 instrucción `in pins, 8` + autopush 32 bits -> 1 muestra/ciclo PIO.
     fs = f_sys / clkdiv. 4 muestras de 8 bits por palabra de 32 bits.
   - DMA "data": PIO RX FIFO -> buffer circular en SRAM. Al terminar el buffer encadena
     al DMA "control", que reescribe WRITE_ADDR del DMA data y lo re-dispara (AL2_WRITE_ADDR_TRIG).
     Resultado: buffer circular de tamaño arbitrario (sin el límite de 32 KB del ring wrap).
   - PIO SM "trigger": `wait` sobre el GPIO (flanco) a f_sys completa -> `irq set 0`.
     La CPU solo detecta el evento; la posición exacta del flanco se obtiene buscando
     la transición en los datos capturados (resolución = 1 muestra, independiente de la
     latencia de la CPU).
   - Generador de prueba (core1): UART + SPI + SMBus (con PEC) para lazo de autoprueba.

  Mapeo de canales: CH0..CH7 = GP0..GP7 (bit k del byte = GP(k))

  Protocolo (USB CDC, comandos ASCII terminados en '\n'):
   I                          -> "LA8 <chip> <f_sys> <max_muestras> <pin_base>"
   C <fs> <n> <pre> <modo> <ch>  modo: 0=inmediato 1=flanco subida 2=flanco bajada
                              -> ["ARM"] "DATA <fs_real> <n> <idx_trig> <exacto> <overflow>"
                                 + n bytes binarios + "END"
   X  (durante una captura)   -> "ABORT"
   G <0|1>                    -> generador de prueba off/on -> "OK"
   V <0|3|5>                  -> VCCB del TXS0108E: 0=apagado, 3=3V3, 5=VBUS
                              -> "VCC <sel> <vccb_mV> OK" | "ERR VCCB <sel> <vccb_mV>"
   S                          -> "STAT <sel> <oe> <vccb_mV> <vref_mV>"

  Front-end TXS0108E (lado A = Pico a 3V3, lado B = circuito bajo prueba):
   - OE   <- GP18 (pull-down 10 kΩ: traductor en alta impedancia durante el reset)
   - VCCB <- LDO ajustable (PMOS pass, p. ej. MIC5205-ADJ) alimentado desde VBUS
             GP20 -> EN del LDO (pull-down: LDO apagado durante el reset)
             GP19 -> NMOS que conecta Rp en paralelo con R2 del feedback:
                     0 = 3,3 V ; 1 = consigna 5,5 V -> LDO en dropout -> VCCB ~ VBUS
   - GP26 (ADC0) mide VCCB con divisor 100k/100k
   - GP27 (ADC1) mide VREF del circuito bajo prueba con divisor 100k/100k (opcional)

  ELECTGPL — uso educativo. No pretende reemplazar un analizador comercial.
*/

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/structs/busctrl.h"

// ===================== CONFIGURACIÓN =====================
#define CH_BASE_PIN     0          // GP0..GP7
#define CH_COUNT        8

#if defined(PICO_RP2350)
  #define BUF_BYTES     (384u * 1024u)   // RP2350: 520 KB SRAM
  #define CHIP_NAME     "RP2350"
#else
  #define BUF_BYTES     (192u * 1024u)   // RP2040: 264 KB SRAM
  #define CHIP_NAME     "RP2040"
#endif
#define SAFETY_SAMPLES  8192u            // margen entre pre+post y el tamaño del buffer
#define POST_MARGIN_W   16u              // palabras extra luego del trigger (latencias FIFO/ISR)
#define TRIG_BACKSCAN   16384u           // ventana de búsqueda hacia atrás del flanco exacto

// Pines del generador de prueba (puentear a los canales para autoprueba)
#define GEN_UART_TX     8    // UART1 TX   -> CH0
#define GEN_UART_RX     9
#define GEN_SPI_SCK     10   // SPI1 SCK   -> CH1
#define GEN_SPI_MOSI    11   // SPI1 TX    -> CH2
#define GEN_SPI_MISO    12   // SPI1 RX
#define GEN_SPI_CS      13   // GPIO       -> CH3
#define GEN_I2C_SDA     14   // I2C1 SDA (master) -> CH6 (decoder SMBus)
#define GEN_I2C_SCL     15   // I2C1 SCL (master) -> CH7 (decoder SMBus)
#define GEN_SLV_SDA     16   // I2C0 SDA (slave 0x0B) puentear con GP14
#define GEN_SLV_SCL     17   // I2C0 SCL (slave 0x0B) puentear con GP15
#define GEN_SMB_ADDR    0x0B // dirección "Smart Battery"

// Front-end TXS0108E
#define FE_OE_PIN       18   // OE del TXS0108E (activo alto, referenciado a VCCA = 3V3)
#define FE_SEL5_PIN     19   // NMOS del feedback: 1 = modo 5 V (dropout), 0 = 3,3 V
#define FE_EN_PIN       20   // EN del LDO
#define FE_VCCB_ADC     26   // ADC0: VCCB / 2
#define FE_VREF_ADC     27   // ADC1: VREF del circuito bajo prueba / 2 (opcional)
#define FE_DIV_RATIO    2.0f // divisor 100k / 100k
#define FE_ADC_VREF     3.30f
#define FE_DEFAULT_SEL  3    // tensión al arrancar (3 = 3V3)
// ==========================================================

static uint32_t capbuf[BUF_BYTES / 4] __attribute__((aligned(4)));
static uint8_t* const capbuf8 = (uint8_t*)capbuf;
static const uint32_t BUF_WORDS = BUF_BYTES / 4;
static uint32_t buf_addr_var;          // origen del DMA de control

static PIO pio = pio0;
static int sm_cap = -1, sm_trg = -1;
static uint off_cap = 0, off_trg = 0;
static int dma_data = -1, dma_ctrl = -1;

static uint16_t cap_instr[1];
static uint16_t trg_instr[4];

volatile bool gen_enabled = false;

// ---------------------------------------------------------------------------
static void pio_load_programs() {
  cap_instr[0] = pio_encode_in(pio_pins, CH_COUNT);
  pio_program_t pc = {};
  pc.instructions = cap_instr; pc.length = 1; pc.origin = -1;
  sm_cap = pio_claim_unused_sm(pio, true);
  off_cap = pio_add_program(pio, &pc);

  // Placeholder del trigger (se reescribe en cada captura)
  for (int i = 0; i < 4; i++) trg_instr[i] = pio_encode_nop();
  pio_program_t pt = {};
  pt.instructions = trg_instr; pt.length = 4; pt.origin = -1;
  sm_trg = pio_claim_unused_sm(pio, true);
  off_trg = pio_add_program(pio, &pt);
}

static void setup_capture_sm(float div) {
  pio_sm_set_enabled(pio, sm_cap, false);
  pio_sm_config c = pio_get_default_sm_config();
  sm_config_set_in_pins(&c, CH_BASE_PIN);
  sm_config_set_wrap(&c, off_cap, off_cap);
  sm_config_set_in_shift(&c, true, true, 32);      // shift right: muestra 0 en bits 7:0
  sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);   // FIFO RX de 8 palabras
  sm_config_set_clkdiv(&c, div);
  pio_sm_init(pio, sm_cap, off_cap, &c);
  pio_sm_clear_fifos(pio, sm_cap);
  pio->fdebug = (1u << (PIO_FDEBUG_RXSTALL_LSB + sm_cap));   // limpia flag de overflow
}

// modo 1: subida, 2: bajada
static void setup_trigger_sm(int mode, int ch) {
  pio_sm_set_enabled(pio, sm_trg, false);
  uint gpio = CH_BASE_PIN + ch;
  bool first = (mode == 1) ? false : true;
  pio->instr_mem[off_trg + 0] = pio_encode_wait_gpio(first, gpio);
  pio->instr_mem[off_trg + 1] = pio_encode_wait_gpio(!first, gpio);
  pio->instr_mem[off_trg + 2] = pio_encode_irq_set(false, 0);
  pio->instr_mem[off_trg + 3] = pio_encode_nop();
  pio_sm_config c = pio_get_default_sm_config();
  sm_config_set_wrap(&c, off_trg + 3, off_trg + 3);   // queda girando en el nop
  sm_config_set_clkdiv(&c, 1.0f);                     // trigger a f_sys completa
  pio_sm_init(pio, sm_trg, off_trg, &c);
  pio_interrupt_clear(pio, 0);
}

static void start_dma() {
  buf_addr_var = (uint32_t)capbuf;

  dma_channel_config cd = dma_channel_get_default_config(dma_data);
  channel_config_set_transfer_data_size(&cd, DMA_SIZE_32);
  channel_config_set_read_increment(&cd, false);
  channel_config_set_write_increment(&cd, true);
  channel_config_set_dreq(&cd, pio_get_dreq(pio, sm_cap, false));
  channel_config_set_chain_to(&cd, dma_ctrl);
  channel_config_set_high_priority(&cd, true);
  dma_channel_configure(dma_data, &cd, capbuf, &pio->rxf[sm_cap], BUF_WORDS, false);

  dma_channel_config cc = dma_channel_get_default_config(dma_ctrl);
  channel_config_set_transfer_data_size(&cc, DMA_SIZE_32);
  channel_config_set_read_increment(&cc, false);
  channel_config_set_write_increment(&cc, false);
  channel_config_set_high_priority(&cc, true);
  dma_channel_configure(dma_ctrl, &cc, &dma_hw->ch[dma_data].al2_write_addr_trig,
                        &buf_addr_var, 1, false);

  dma_channel_start(dma_data);
}

static void stop_all() {
  pio_sm_set_enabled(pio, sm_cap, false);
  pio_sm_set_enabled(pio, sm_trg, false);
  busy_wait_us(10);   // deja que el DMA vacíe la FIFO RX
}

static void abort_dma() {
  // Desencadena (chain_to = sí mismo) antes de abortar para evitar re-disparos
  hw_write_masked(&dma_hw->ch[dma_data].al1_ctrl,
                  (uint32_t)dma_data << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB,
                  DMA_CH0_CTRL_TRIG_CHAIN_TO_BITS);
  dma_channel_abort(dma_ctrl);
  dma_channel_abort(dma_data);
  pio_sm_clear_fifos(pio, sm_cap);
}

static inline uint32_t dma_word_pos() {
  uint32_t off = (dma_hw->ch[dma_data].write_addr - (uint32_t)capbuf) >> 2;
  if (off >= BUF_WORDS) off -= BUF_WORDS;
  return off;
}

static inline uint32_t ring_idx(uint64_t abs_sample) {
  return (uint32_t)(abs_sample % BUF_BYTES);
}

// ---------------------------------------------------------------------------
static void do_capture(uint32_t fs_req, uint32_t nsamp, uint32_t pre, int tmode, int tch) {
  const uint32_t fsys = clock_get_hz(clk_sys);
  const uint32_t max_s = BUF_BYTES - SAFETY_SAMPLES;

  if (fs_req == 0 || fs_req > fsys) fs_req = fsys;
  if (nsamp < 64) nsamp = 64;
  if (nsamp > max_s) nsamp = max_s;
  if (tmode < 0 || tmode > 2) tmode = 0;
  if (tch < 0 || tch >= CH_COUNT) tch = 0;
  if (tmode == 0) pre = 0;
  if (pre > nsamp) pre = nsamp;
  const uint32_t post = nsamp - pre;

  float div = (float)fsys / (float)fs_req;
  if (div < 1.0f) div = 1.0f;
  if (div > 65535.0f) div = 65535.0f;

  setup_capture_sm(div);
  // Lee el divisor real programado (INT 16 bits + FRAC 8 bits)
  uint32_t cd = pio->sm[sm_cap].clkdiv;
  double divr = (double)(cd >> 16) + (double)((cd >> 8) & 0xFF) / 256.0;
  if (divr < 1.0) divr = 65536.0;          // INT=0 significa 65536
  const double fs_act = (double)fsys / divr;

  if (tmode != 0) setup_trigger_sm(tmode, tch);

  start_dma();
  pio_sm_set_enabled(pio, sm_cap, true);

  uint64_t total_w = 0;     // palabras escritas por el DMA (monótono)
  uint32_t last = 0;
  bool armed = (tmode == 0), trig = (tmode == 0);
  uint64_t det_s = 0;
  const uint64_t pre_need_s = (uint64_t)pre + 64;
  const uint64_t imm_need_s = (uint64_t)nsamp + 64;
  bool aborted = false;

  for (;;) {
    uint32_t p = dma_word_pos();
    uint32_t d = (p >= last) ? (p - last) : (p + BUF_WORDS - last);
    last = p;
    total_w += d;
    const uint64_t total_s = total_w * 4;

    if (tmode == 0) {
      if (total_s >= imm_need_s) break;
    } else {
      if (!armed && total_s >= pre_need_s) {
        pio_interrupt_clear(pio, 0);
        pio_sm_set_enabled(pio, sm_trg, true);
        armed = true;
        Serial.print("ARM\n");
      }
      if (armed && !trig && (pio->irq & 1u)) {
        det_s = total_s;
        trig = true;
      }
      if (trig && total_s >= det_s + post + POST_MARGIN_W * 4) break;
    }

    if (Serial.available()) {
      int c = Serial.read();
      if (c == 'X') { aborted = true; break; }
    }
  }

  stop_all();
  {
    uint32_t p = dma_word_pos();
    uint32_t d = (p >= last) ? (p - last) : (p + BUF_WORDS - last);
    total_w += d;
  }
  const bool ovf = (pio->fdebug & (1u << (PIO_FDEBUG_RXSTALL_LSB + sm_cap))) != 0;
  abort_dma();
  pio_interrupt_clear(pio, 0);

  if (aborted) { Serial.print("ABORT\n"); return; }

  const uint64_t end_s = total_w * 4;
  const uint64_t valid_start = (end_s > BUF_BYTES) ? (end_s - BUF_BYTES) : 0;

  uint64_t trig_s = 0;
  bool exact = true;
  uint64_t start;

  if (tmode == 0) {
    start = end_s - nsamp;
  } else {
    // Búsqueda hacia atrás de la transición (pre_level -> post_level) en el canal de trigger
    const int pre_level  = (tmode == 1) ? 0 : 1;
    uint64_t hi = det_s + 64;
    if (hi > end_s - 1) hi = end_s - 1;
    uint64_t lo = (det_s > TRIG_BACKSCAN) ? (det_s - TRIG_BACKSCAN) : 0;
    if (lo < valid_start) lo = valid_start;
    exact = false;
    for (uint64_t s = hi; s > lo; s--) {
      int b1 = (capbuf8[ring_idx(s)] >> tch) & 1;
      int b0 = (capbuf8[ring_idx(s - 1)] >> tch) & 1;
      if (b0 == pre_level && b1 != pre_level) { trig_s = s; exact = true; break; }
    }
    if (!exact) trig_s = det_s;     // pulso más angosto que 1/fs: posición aproximada
    start = (trig_s >= pre) ? (trig_s - pre) : 0;
  }

  if (start < valid_start) start = valid_start;
  uint32_t n_out = nsamp;
  if (start + n_out > end_s) n_out = (uint32_t)(end_s - start);
  const int32_t trig_rel = (tmode == 0) ? 0 : (int32_t)(trig_s - start);

  Serial.printf("DATA %.3f %lu %ld %d %d\n", fs_act, (unsigned long)n_out,
                (long)trig_rel, exact ? 1 : 0, ovf ? 1 : 0);

  uint32_t idx = ring_idx(start);
  uint32_t remaining = n_out;
  while (remaining) {
    uint32_t chunk = BUF_BYTES - idx;
    if (chunk > remaining) chunk = remaining;
    Serial.write(capbuf8 + idx, chunk);
    remaining -= chunk;
    idx = 0;
  }
  Serial.print("END\n");
  Serial.flush();
}

// ---------------------------------------------------------------------------
// =================== Front-end: selección de VCCB ===================
static int fe_sel = 0;
static bool fe_oe = false;

static uint32_t fe_read_mv(int pin) {
  uint32_t acc = 0;
  for (int i = 0; i < 16; i++) acc += analogRead(pin);
  float v = (acc / 16.0f) * FE_ADC_VREF / 4095.0f * FE_DIV_RATIO;
  return (uint32_t)(v * 1000.0f + 0.5f);
}

static void fe_set_oe(bool on) {
  digitalWrite(FE_OE_PIN, on ? HIGH : LOW);
  fe_oe = on;
}

// Secuencia: OE off -> ambos caminos off (break-before-make) -> seleccionar ->
// esperar estabilización -> verificar por ADC -> OE on
static bool fe_select(int sel, uint32_t* mv_out) {
  fe_set_oe(false);
  fe_sel = 0;
  if (sel == 0) {
    digitalWrite(FE_EN_PIN, LOW);
    digitalWrite(FE_SEL5_PIN, LOW);
    delay(5);
    *mv_out = fe_read_mv(FE_VCCB_ADC);
    return true;
  }
  digitalWrite(FE_SEL5_PIN, sel == 5 ? HIGH : LOW);
  digitalWrite(FE_EN_PIN, HIGH);
  const uint32_t lo = (sel == 5) ? 4400 : 3000;   // ventanas de aceptación
  const uint32_t hi = (sel == 5) ? 5500 : 3600;
  uint32_t mv = 0;
  for (int t = 0; t < 50; t++) {                   // hasta ~50 ms
    delay(1);
    mv = fe_read_mv(FE_VCCB_ADC);
    if (mv >= lo && mv <= hi) {
      fe_sel = sel;
      fe_set_oe(true);
      *mv_out = mv;
      return true;
    }
  }
  digitalWrite(FE_EN_PIN, LOW);                    // falla: LDO apagado
  digitalWrite(FE_SEL5_PIN, LOW);
  *mv_out = mv;
  return false;
}

void setup() {
  Serial.begin(115200);       // USB CDC: el baudrate es irrelevante
  Serial.setTimeout(50);

  for (int i = 0; i < CH_COUNT; i++) {
    gpio_init(CH_BASE_PIN + i);              // en RP2350 también libera el aislamiento del pad
    gpio_set_dir(CH_BASE_PIN + i, GPIO_IN);
    gpio_disable_pulls(CH_BASE_PIN + i);     // RP2350-E9: evitar pull-down interno
    gpio_set_input_hysteresis_enabled(CH_BASE_PIN + i, true);
  }

  pio_load_programs();
  dma_data = dma_claim_unused_channel(true);
  dma_ctrl = dma_claim_unused_channel(true);

  // Prioridad de bus para el DMA (lectura y escritura)
  busctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_W_BITS | BUSCTRL_BUS_PRIORITY_DMA_R_BITS;

  // Front-end: arranca deshabilitado y luego selecciona la tensión por defecto
  pinMode(FE_OE_PIN, OUTPUT);   digitalWrite(FE_OE_PIN, LOW);
  pinMode(FE_SEL5_PIN, OUTPUT); digitalWrite(FE_SEL5_PIN, LOW);
  pinMode(FE_EN_PIN, OUTPUT);   digitalWrite(FE_EN_PIN, LOW);
  analogReadResolution(12);
  uint32_t mv;
  fe_select(FE_DEFAULT_SEL, &mv);
}

static void print_banner() {
  Serial.printf("# LA8 Electgpl - analizador logico 8 canales (%s, f_sys %lu Hz)\n",
                CHIP_NAME, (unsigned long)clock_get_hz(clk_sys));
  Serial.print("# Comandos: I (info) | S (estado front-end) | V 0/3/5 (VCCB) | G 0/1 (generador)\n");
  Serial.print("#           C <fs> <n> <pre> <modo> <ch> (captura binaria, usar desde el viewer) | H (ayuda)\n");
}

void loop() {
  // Banner al abrir el puerto (flanco de DTR): el Serial Monitor y el viewer lo reciben
  static bool was_connected = false;
  bool connected = (bool)Serial;
  if (connected && !was_connected) { delay(50); print_banner(); }
  was_connected = connected;

  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  char cmd = line.charAt(0);
  if (cmd == 'I') {
    Serial.printf("LA8 %s %lu %lu %d\n", CHIP_NAME, (unsigned long)clock_get_hz(clk_sys),
                  (unsigned long)(BUF_BYTES - SAFETY_SAMPLES), CH_BASE_PIN);
  } else if (cmd == 'C') {
    unsigned long fs = 0, n = 0, pre = 0; int mode = 0, ch = 0;
    if (sscanf(line.c_str(), "C %lu %lu %lu %d %d", &fs, &n, &pre, &mode, &ch) == 5) {
      do_capture(fs, n, pre, mode, ch);
    } else {
      Serial.print("ERR sintaxis\n");
    }
  } else if (cmd == 'V') {
    int sel = (line.length() > 2) ? line.charAt(2) - '0' : -1;
    if (sel == 0 || sel == 3 || sel == 5) {
      uint32_t mv = 0;
      bool ok = fe_select(sel, &mv);
      if (ok) Serial.printf("VCC %d %lu OK\n", sel, (unsigned long)mv);
      else    Serial.printf("ERR VCCB %d %lu\n", sel, (unsigned long)mv);
    } else {
      Serial.print("ERR parametro\n");
    }
  } else if (cmd == 'S') {
    Serial.printf("STAT %d %d %lu %lu\n", fe_sel, fe_oe ? 1 : 0,
                  (unsigned long)fe_read_mv(FE_VCCB_ADC), (unsigned long)fe_read_mv(FE_VREF_ADC));
  } else if (cmd == 'G') {
    gen_enabled = (line.length() > 2 && line.charAt(2) == '1');
    Serial.print("OK\n");
  } else if (cmd == 'H' || cmd == '?') {
    print_banner();
  } else if (cmd == 'X') {
    // abort sin captura en curso: ignorar
  } else {
    Serial.print("ERR comando\n");
  }
}

// =================== Generador de prueba (core1) ===================
static volatile uint8_t slv_cmd = 0;

static uint8_t smbus_pec(const uint8_t* d, size_t n) {
  uint8_t crc = 0;
  for (size_t i = 0; i < n; i++) {
    crc ^= d[i];
    for (int b = 0; b < 8; b++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
  }
  return crc;
}

static void slv_on_receive(int len) {
  if (Wire.available()) slv_cmd = Wire.read();
  while (Wire.available()) (void)Wire.read();
  (void)len;
}

static void slv_on_request() {
  // SMBus Read Word: respuesta 0x1234 + PEC sobre [addrW, cmd, addrR, lo, hi]
  uint8_t lo = 0x34, hi = 0x12;
  uint8_t pk[5] = { (uint8_t)(GEN_SMB_ADDR << 1), slv_cmd, (uint8_t)((GEN_SMB_ADDR << 1) | 1), lo, hi };
  uint8_t out[3] = { lo, hi, smbus_pec(pk, 5) };
  Wire.write(out, 3);
}

static bool gen_ready = false;
static uint8_t gen_cnt = 0;

static void gen_init() {
  Serial2.setTX(GEN_UART_TX);
  Serial2.setRX(GEN_UART_RX);
  Serial2.begin(115200);

  SPI1.setSCK(GEN_SPI_SCK);
  SPI1.setTX(GEN_SPI_MOSI);
  SPI1.setRX(GEN_SPI_MISO);
  SPI1.begin();
  pinMode(GEN_SPI_CS, OUTPUT);
  digitalWrite(GEN_SPI_CS, HIGH);

  Wire.setSDA(GEN_SLV_SDA);          // I2C0 como esclavo SMBus
  Wire.setSCL(GEN_SLV_SCL);
  Wire.onReceive(slv_on_receive);
  Wire.onRequest(slv_on_request);
  Wire.begin(GEN_SMB_ADDR);

  Wire1.setSDA(GEN_I2C_SDA);         // I2C1 como master
  Wire1.setSCL(GEN_I2C_SCL);
  Wire1.setClock(100000);
  Wire1.begin();
  gen_ready = true;
}

void setup1() {}

void loop1() {
  if (!gen_enabled) { delay(5); return; }
  if (!gen_ready) gen_init();

  // 1) UART 115200 8N1
  Serial2.print("ELECTGPL\r\n");
  Serial2.flush();
  delayMicroseconds(150);

  // 2) SPI modo 0, 1 MHz, 4 bytes con CS
  SPI1.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(GEN_SPI_CS, LOW);
  const uint8_t tx[4] = { 0x9F, 0xA5, 0x3C, gen_cnt };
  for (int i = 0; i < 4; i++) (void)SPI1.transfer(tx[i]);
  digitalWrite(GEN_SPI_CS, HIGH);
  SPI1.endTransaction();
  delayMicroseconds(150);

  // 3) SMBus Write Word con PEC: [addrW, cmd, lo, hi, PEC]
  uint8_t pk[4] = { (uint8_t)(GEN_SMB_ADDR << 1), 0x09, gen_cnt, 0x5A };
  Wire1.beginTransmission(GEN_SMB_ADDR);
  Wire1.write(pk[1]); Wire1.write(pk[2]); Wire1.write(pk[3]);
  Wire1.write(smbus_pec(pk, 4));
  Wire1.endTransmission();
  delayMicroseconds(150);

  // 4) SMBus Read Word con PEC: S addrW cmd Sr addrR lo hi PEC(NACK) P
  Wire1.beginTransmission(GEN_SMB_ADDR);
  Wire1.write((uint8_t)0x09);
  Wire1.endTransmission(false);
  Wire1.requestFrom((uint8_t)GEN_SMB_ADDR, (uint8_t)3);
  while (Wire1.available()) (void)Wire1.read();

  gen_cnt++;
  delayMicroseconds(500);
}
