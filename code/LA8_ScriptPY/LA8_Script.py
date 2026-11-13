#!/usr/bin/env python3
"""
LA8 Viewer — Analizador lógico 8 canales RP2040/RP2350 — ELECTGPL
Interfaz estilo Saleae: trazas digitales, cursores, decoders UART / SPI / I2C / SMBus (PEC)
y tabla de tramas con columnas HEX / DEC / BIN / ASCII.

Requisitos:  pip install pyserial numpy pyqtgraph PySide6
Ejecución:   desde Thonny (F5) o "python la8_viewer.py". No usa argumentos de línea de comandos.
Sin hardware: botón "Demo" genera una captura sintética con UART + SPI + SMBus.
"""

from __future__ import annotations

# ============================ CONFIGURACIÓN ============================
PORT            = "COM5"            # puerto serie de la Pico (Linux: "/dev/ttyACM0")
SERIAL_TIMEOUT  = 0.2               # s, timeout de lectura por línea
DATA_TIMEOUT    = 10.0              # s, timeout para recibir el bloque binario

# Las etiquetas de los canales se generan solas a partir de los analizadores configurados
# (p. ej. "CH0 I2S BCLK"); un canal sin analizador queda como "CH0". Nombre fijo opcional:
CH_ALIAS = {}                       # ej. {7: "CH7 Trigger externo"}
VREF_SENSE = False                  # True solo si la placa tiene el divisor 100k/100k en GP27 (borne VREF);
                                    # en una Pico sin divisor, GP27 queda flotante y lee cualquier valor
VREF_NC_MV = 300                    # con divisor: VREF por debajo de esto = borne sin conectar

DEFAULT_FS        = 10_000_000      # muestras/s
DEFAULT_SAMPLES   = 200_000
DEFAULT_PRE_PCT   = 10              # % de pre-trigger
DEFAULT_TRIG_MODE = "Bajada"        # "Ninguno" | "Subida" | "Bajada"
DEFAULT_TRIG_CH   = 3
# Front-end: "LV4T125" (SN74LV4T125, 1,8–5 V) | "LVC" (74LVC125A/244A, 2,5–5 V)
#            "HC244" (74HC244 a 3,3 V, solo lógica de 3,3 V) | "TXS0108" (VCCB conmutable)
FRONTEND          = "LV4T125"
DEFAULT_VCCB      = "3,3 V"         # solo TXS0108: "3,3 V" | "5 V" | "Apagado"
# Rango de VREF aceptado por front-end (mV): fuera de él, el viewer avisa
FRONTEND_LIMITS   = {"LV4T125": (1650, 5500, "1,8–5 V"),
                     "LVC":     (2500, 5500, "2,5–5 V"),
                     "HC244":   (2800, 3600, "3,3 V")}
VREF_MISMATCH_PCT = 12              # alerta si VREF del circuito difiere de VCCB más que esto

# Logo (mismo archivo y criterio que los otros dashboards: en la misma carpeta que el script)
LOGO_FILE         = "electgpl_logo.png"
LOGO_HEIGHT       = 34              # px de alto en el header (se mantiene la relación de aspecto)

# Tema visual (dark, acento rojo estilo Node-RED)
THEME_ACCENT      = "#8f0000"       # rojo Node-RED (header, botón principal)
THEME_ACCENT_HI   = "#ad1625"       # hover / foco
THEME_BG          = "#1a1a1a"       # fondo general
THEME_PANEL       = "#262626"       # barras y paneles
THEME_PLOT_BG     = "#151515"       # fondo de las trazas

# Decoders activos al iniciar (coinciden con el generador GEN_Electgpl_S3 y la autoprueba del LA8)
DEFAULT_DECODERS = [
    {"type": "UART",  "ch": 0, "baud": 115200, "bits": 8, "parity": "N", "stop": 1, "invert": 0},
    {"type": "SPI",   "clk": 1, "mosi": 2, "miso": -1, "cs": 3, "mode": 0, "bits": 8, "msb": 1},
    {"type": "I2C",   "sda": 4, "scl": 5},
    {"type": "SMBus", "sda": 6, "scl": 7},
]

FS_PRESETS      = [1_000, 10_000, 100_000, 500_000, 1_000_000, 2_000_000, 5_000_000,
                   10_000_000, 25_000_000, 50_000_000, 75_000_000, 150_000_000]
SAMPLE_PRESETS  = [10_000, 50_000, 100_000, 200_000, 300_000, 380_000]
# ======================================================================

import os
import sys
import time
import threading
from dataclasses import dataclass, field

import numpy as np

# ----------------------------------------------------------------------
#                          MODELO DE TRAMAS
# ----------------------------------------------------------------------
@dataclass
class Frame:
    s0: float            # muestra de inicio (puede ser fraccional)
    s1: float            # muestra de fin
    decoder: str         # "UART#1", "SPI#2", ...
    kind: str            # "data", "addr", "start", "stop", "rstart", "pec", "err"
    ch: int              # canal sobre el que se dibuja la anotación
    value: int | None = None
    info: str = ""
    error: bool = False
    bits: int = 8        # ancho del valor (I2S: 16/24/32 bits)
    signed: bool = False # DEC en complemento a dos (muestras de audio)


def to_signed(v, bits):
    return v - (1 << bits) if v & (1 << (bits - 1)) else v


def fmt_value(v, radix, bits=8, signed=False):
    if v is None:
        return ""
    if radix == "HEX":
        return f"0x{v:0{(bits + 3) // 4}X}"
    if radix == "DEC":
        return str(to_signed(v, bits) if signed else v)
    if radix == "BIN":
        return f"0b{v:0{bits}b}"
    if radix == "ASCII":
        if bits > 8:      # una muestra de audio no tiene representación ASCII: DEC
            return str(to_signed(v, bits) if signed else v)
        return ascii_repr(v)
    return str(v)


def ascii_repr(v):
    if v is None:
        return ""
    names = {0: "NUL", 9: "\\t", 10: "\\n", 13: "\\r", 27: "ESC", 32: "SP", 127: "DEL"}
    if v in names:
        return names[v]
    if 32 < v < 127:
        return chr(v)
    return "·"


def smbus_crc8(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc

# ----------------------------------------------------------------------
#                              DECODERS
#   d: ndarray uint8 (8, N) con 0/1 por canal; fs en Hz.
#   Índices de muestra: la transición entre i e i+1 se ubica en i+0.5.
# ----------------------------------------------------------------------
def decode_uart(d, fs, name, ch, baud=115200, bits=8, parity="N", stop=1, invert=0):
    x = d[ch].astype(np.int8)
    if invert:
        x = 1 - x
    n = x.size
    spb = fs / float(baud)
    if spb < 2.0:
        return [Frame(0, n - 1, name, "err", ch, None, f"fs insuficiente ({spb:.2f} muestras/bit)", True)]
    nbits = bits + (0 if parity == "N" else 1)
    falls = np.flatnonzero((x[:-1] == 1) & (x[1:] == 0))     # x[i]=1, x[i+1]=0
    # Exigir línea en reposo (alta) al menos 1 bit antes del primer start
    frames = []
    nxt = int(spb)
    k = 0
    while k < falls.size:
        i = falls[k]
        if i < nxt:
            k = np.searchsorted(falls, nxt)
            continue
        e = i + 0.5
        end = e + (1 + nbits + stop) * spb
        if end >= n:
            break

        def c(bit):
            return int(e + (bit + 0.5) * spb)

        if x[c(0)] != 0:                       # glitch, no es start
            k += 1
            continue
        val = 0
        for b in range(bits):
            val |= int(x[c(1 + b)]) << b       # LSB primero
        err = []
        if parity != "N":
            p = int(x[c(1 + bits)])
            ones = bin(val).count("1") + p
            if (parity == "E" and ones % 2) or (parity == "O" and ones % 2 == 0):
                err.append("paridad")
        for sb in range(stop):
            if x[c(1 + nbits + sb)] != 1:
                err.append("framing")
                break
        frames.append(Frame(e, end, name, "data", ch, val,
                            ("Error: " + ", ".join(err)) if err else "", bool(err)))
        nxt = int(e + (1 + nbits + 0.5) * spb)  # centro del (primer) stop bit
        k = np.searchsorted(falls, nxt)
    return frames


def decode_spi(d, fs, name, clk, mosi, miso=-1, cs=-1, mode=0, bits=8, msb=1):
    c = d[clk]
    cpol, cpha = (mode >> 1) & 1, mode & 1
    if cpol == cpha:                            # modos 0 y 3: muestreo en flanco de subida
        edges = np.flatnonzero((c[:-1] == 0) & (c[1:] == 1))
    else:                                       # modos 1 y 2: muestreo en flanco de bajada
        edges = np.flatnonzero((c[:-1] == 1) & (c[1:] == 0))
    if edges.size == 0:
        return []
    frames = []
    if cs >= 0:
        csd = d[cs]
        active = csd[edges] == 0
        cs_fall = np.flatnonzero((csd[:-1] == 1) & (csd[1:] == 0))
        cs_rise = np.flatnonzero((csd[:-1] == 0) & (csd[1:] == 1))
        session = np.searchsorted(cs_fall, edges)
        for f in cs_fall:
            frames.append(Frame(f + 0.5, f + 0.5, name, "start", cs, None, "CS activo"))
        for r in cs_rise:
            frames.append(Frame(r + 0.5, r + 0.5, name, "stop", cs, None, "CS inactivo"))
    else:
        active = np.ones(edges.size, dtype=bool)
        session = np.zeros(edges.size, dtype=np.int64)

    mo = d[mosi][edges]                         # dato estable en la muestra previa al flanco
    mi = d[miso][edges] if miso >= 0 else None
    cur_s, cnt, vmo, vmi, first = -1, 0, 0, 0, 0
    period = np.median(np.diff(edges)) if edges.size > 1 else 2
    for k in range(edges.size):
        if not active[k]:
            continue
        if session[k] != cur_s:
            cur_s, cnt, vmo, vmi = session[k], 0, 0, 0
        if cnt == 0:
            first = edges[k]
        bmo = int(mo[k])
        bmi = int(mi[k]) if mi is not None else 0
        if msb:
            vmo = (vmo << 1) | bmo
            vmi = (vmi << 1) | bmi
        else:
            vmo |= bmo << cnt
            vmi |= bmi << cnt
        cnt += 1
        if cnt == bits:
            s0 = first + 0.5 - period / 2
            s1 = edges[k] + 0.5 + period / 2
            frames.append(Frame(s0, s1, name, "data", mosi, vmo, "MOSI"))
            if mi is not None:
                frames.append(Frame(s0, s1, name, "data", miso, vmi, "MISO"))
            cnt, vmo, vmi = 0, 0, 0
    frames.sort(key=lambda f: f.s0)
    return frames


def decode_i2c(d, fs, name, sda, scl, smbus=False):
    a, c = d[sda], d[scl]
    se = np.flatnonzero(a[:-1] != a[1:])
    se = se[(c[se] == 1) & (c[se + 1] == 1)]                 # cambios de SDA con SCL alto
    ev_pos = [se, np.flatnonzero((c[:-1] == 0) & (c[1:] == 1))]
    ev_typ = [np.where(a[se] == 1, 0, 1),                     # 0=START (1->0), 1=STOP (0->1)
              np.full(ev_pos[1].size, 2)]                     # 2=flanco subida SCL
    pos = np.concatenate(ev_pos)
    typ = np.concatenate(ev_typ)
    order = np.argsort(pos, kind="stable")
    pos, typ = pos[order], typ[order]

    frames = []
    state = "idle"
    bits_l, bstart = [], 0
    tx = []                 # (byte, índice de frame) para PEC
    is_write_cmd = False

    def close_tx(endpos):
        if smbus and len(tx) >= 3:
            data = [b for b, _ in tx]
            exp = smbus_crc8(data[:-1])
            fi = tx[-1][1]
            if exp == data[-1]:
                frames[fi].kind = "pec"
                frames[fi].info = f"PEC OK, {frames[fi].info}"
            else:
                frames[fi].info = f"{frames[fi].info}  (sin PEC válido: CRC-8 esperado 0x{exp:02X})"

    for p, t in zip(pos, typ):
        if t == 0:                                         # START / START repetido
            frames.append(Frame(p + 0.5, p + 0.5, name, "start" if state == "idle" else "rstart",
                                sda, None, "START" if state == "idle" else "START rep."))
            if state == "idle":
                tx = []
            state, bits_l = "addr", []
        elif t == 1:                                       # STOP
            if state != "idle":
                frames.append(Frame(p + 0.5, p + 0.5, name, "stop", sda, None, "STOP"))
                close_tx(p)
            state, bits_l = "idle", []
        else:                                              # flanco de subida de SCL
            if state == "idle":
                continue
            j = min(p + 1, a.size - 1)
            if not bits_l:
                bstart = p
            bits_l.append(int(a[j]))
            if len(bits_l) == 9:
                byte = 0
                for b in bits_l[:8]:
                    byte = (byte << 1) | b
                nack = bits_l[8] == 1
                tb = (p - bstart) / 8.0
                s0, s1 = bstart + 0.5 - tb / 2, p + 0.5 + tb / 2
                ack = "NACK" if nack else "ACK"
                if state == "addr":
                    rw = "R" if byte & 1 else "W"
                    frames.append(Frame(s0, s1, name, "addr", sda, byte >> 1,
                                        f"Dir. 7 bit {rw} {ack} (byte 0x{byte:02X})", nack))
                    is_write_cmd = (rw == "W")
                    state = "data"
                else:
                    info = ack
                    if smbus and is_write_cmd and len(tx) == 1:
                        info = f"Command {ack}"
                    frames.append(Frame(s0, s1, name, "data", sda, byte, info))
                tx.append((byte, len(frames) - 1))
                bits_l = []
    return frames


def decode_i2s(d, fs, name, sck, ws, sd, bits=16, fmt="I2S", edge="Subida"):
    """I2S Philips (MSB un ciclo de BCLK después del cambio de WS, WS=0 izquierdo) o
    left-justified (MSB en el mismo ciclo del cambio de WS, WS=1 izquierdo). MSB primero."""
    c = d[sck]
    if edge == "Subida":
        e = np.flatnonzero((c[:-1] == 0) & (c[1:] == 1))
    else:
        e = np.flatnonzero((c[:-1] == 1) & (c[1:] == 0))
    if e.size < bits + 2:
        return []
    w = d[ws][e].astype(np.int8)          # WS y SD estables en la muestra previa al flanco
    x = d[sd][e].astype(np.int64)
    period = float(np.median(np.diff(e)))
    delay = 1 if fmt == "I2S" else 0
    starts = np.flatnonzero(w[1:] != w[:-1]) + 1   # primer flanco con el nuevo nivel de WS
    frames = []
    weights = (1 << np.arange(bits - 1, -1, -1, dtype=np.int64))
    for k in starts:
        a = k + delay
        b = a + bits
        if b > e.size:
            break
        val = int(np.dot(x[a:b], weights))
        left = (w[k] == 0) if fmt == "I2S" else (w[k] == 1)
        sval = to_signed(val, bits)
        frames.append(Frame(e[a] + 0.5 - period / 2, e[b - 1] + 0.5 + period / 2, name, "data", sd,
                            val, f"{'L' if left else 'R'}  {sval:+d}", False, bits, True))
    return frames


DECODER_SPECS = {
    "UART":  [("ch", "Canal RX/TX", "ch"), ("baud", "Baudrate", "int"), ("bits", "Bits de datos", "int"),
              ("parity", "Paridad", ["N", "E", "O"]), ("stop", "Bits de stop", "int"),
              ("invert", "Invertido", "bool")],
    "SPI":   [("clk", "SCK", "ch"), ("mosi", "MOSI", "ch"), ("miso", "MISO (-1 = no)", "chopt"),
              ("cs", "CS (-1 = no)", "chopt"), ("mode", "Modo (CPOL/CPHA)", "int"),
              ("bits", "Bits por palabra", "int"), ("msb", "MSB primero", "bool")],
    "I2C":   [("sda", "SDA", "ch"), ("scl", "SCL", "ch")],
    "SMBus": [("sda", "SDA", "ch"), ("scl", "SCL", "ch")],
    "I2S":   [("sck", "BCLK (SCK)", "ch"), ("ws", "WS / LRCLK", "ch"), ("sd", "SD (datos)", "ch"),
              ("bits", "Bits por muestra", "int"), ("fmt", "Formato", ["I2S", "LJ"]),
              ("edge", "Flanco de muestreo", ["Subida", "Bajada"])],
}


def run_decoder(d, fs, cfg, idx):
    t = cfg["type"]
    name = f"{t}#{idx + 1}"
    p = {k: v for k, v in cfg.items() if k != "type"}
    if t == "UART":
        return decode_uart(d, fs, name, **p)
    if t == "SPI":
        return decode_spi(d, fs, name, **p)
    if t == "I2C":
        return decode_i2c(d, fs, name, smbus=False, **p)
    if t == "SMBus":
        return decode_i2c(d, fs, name, smbus=True, **p)
    if t == "I2S":
        return decode_i2s(d, fs, name, **p)
    return []


def channel_labels(decoders):
    """Etiqueta de cada canal según los roles que le asignan los analizadores."""
    roles = {i: [] for i in range(8)}
    keys = {"UART":  [("ch", "UART")],
            "SPI":   [("clk", "SCK"), ("mosi", "MOSI"), ("miso", "MISO"), ("cs", "CS")],
            "I2C":   [("sda", "I2C SDA"), ("scl", "I2C SCL")],
            "SMBus": [("sda", "SMB SDA"), ("scl", "SMB SCL")],
            "I2S":   [("sck", "I2S BCLK"), ("ws", "I2S WS"), ("sd", "I2S SD")]}
    for cfg in decoders:
        for key, role in keys.get(cfg["type"], []):
            ch = cfg.get(key, -1)
            if isinstance(ch, int) and 0 <= ch < 8 and role not in roles[ch]:
                roles[ch].append(role)
    out = []
    for i in range(8):
        if i in CH_ALIAS:
            out.append(CH_ALIAS[i])
        else:
            out.append(f"CH{i} " + "/".join(roles[i]) if roles[i] else f"CH{i}")
    return out


def decoder_label(cfg):
    t = cfg["type"]
    if t == "UART":
        return f"UART  CH{cfg['ch']}  {cfg['baud']} {cfg['bits']}{cfg['parity']}{cfg['stop']}"
    if t == "SPI":
        return f"SPI  SCK{cfg['clk']} MOSI{cfg['mosi']} MISO{cfg['miso']} CS{cfg['cs']} modo {cfg['mode']}"
    if t == "I2S":
        return f"I2S  BCLK{cfg['sck']} WS{cfg['ws']} SD{cfg['sd']}  {cfg['bits']} bits {cfg['fmt']}"
    return f"{t}  SDA{cfg['sda']} SCL{cfg['scl']}"

# ----------------------------------------------------------------------
#                 SEÑAL SINTÉTICA (Demo sin hardware)
# ----------------------------------------------------------------------
def synth_capture(fs=10_000_000, n=200_000, pre=20_000):
    d = np.zeros((8, n), dtype=np.uint8)
    d[0, :] = 1          # UART en reposo alto
    d[3, :] = 1          # CS inactivo
    d[4:8, :] = 1        # SDA/SCL de I2C (CH4/CH5) y SMBus (CH6/CH7) en reposo alto

    def seg(ch, t0, t1, v):
        a, b = max(0, int(round(t0 * fs))), min(n, int(round(t1 * fs)))
        if b > a:
            d[ch, a:b] = v

    def uart(t, text, baud=115200):
        tb = 1.0 / baud
        for chv in text.encode():
            seg(0, t, t + tb, 0)
            for b in range(8):
                seg(0, t + (1 + b) * tb, t + (2 + b) * tb, (chv >> b) & 1)
            seg(0, t + 9 * tb, t + 10 * tb, 1)
            t += 10 * tb
        return t

    def spi(t, data, f=1_000_000):
        T = 1.0 / f
        seg(3, t, t + T * (len(data) * 8 + 2), 0)
        t0 = t + T
        j = 0
        for byte in data:
            for b in range(7, -1, -1):
                seg(2, t0 + j * T, t0 + (j + 1) * T, (byte >> b) & 1)
                seg(1, t0 + j * T + T / 2, t0 + (j + 1) * T, 1)
                j += 1
        return t + T * (len(data) * 8 + 2)

    def i2c(t, seq, f=100_000, sda=4, scl=5):
        Q = 1.0 / f / 4
        steps = []
        for item in seq:
            if item == "S":
                steps += [(1, 1), (1, 0), (0, 0), (0, 0)]
            elif item == "Sr":
                steps += [(0, 1), (1, 1), (1, 0), (0, 0)]
            elif item == "P":
                steps += [(0, 0), (1, 0), (1, 1), (1, 1)]
            else:
                byte, ack = item
                for b in [(byte >> k) & 1 for k in range(7, -1, -1)] + [ack]:
                    steps += [(0, b), (1, b), (1, b), (0, b)]
        for k, (sc, sd) in enumerate(steps):
            seg(scl, t + k * Q, t + (k + 1) * Q, sc)
            seg(sda, t + k * Q, t + (k + 1) * Q, sd)
        return t + len(steps) * Q

    # Mismo ciclo que GEN_Electgpl_S3: UART + SPI + I2C + SMBus, cada 5 ms
    msg = b"Electgpl"
    t = pre / fs - 20e-6
    while t < n / fs:
        t0 = t
        uart(t0 + 20e-6, "Electgpl\r\n")                    # la UART corre en paralelo
        t = spi(t0 + 20e-6, list(msg)) + 100e-6
        t = i2c(t, ["S", (0x3C << 1, 0)] + [(c, 0) for c in msg] + ["P"]) + 100e-6
        aw, ar = 0x0B << 1, (0x0B << 1) | 1
        blk = [aw, 0x20, len(msg)] + list(msg)
        t = i2c(t, ["S"] + [(b, 0) for b in blk] + [(smbus_crc8(blk), 0), "P"], sda=6, scl=7) + 100e-6
        rw = [aw, 0x09, ar, 0x34, 0x12]
        t = i2c(t, ["S", (aw, 0), (0x09, 0), "Sr", (ar, 0), (0x34, 0), (0x12, 0),
                    (smbus_crc8(rw), 1), "P"], sda=6, scl=7)
        t = t0 + 5e-3
    trig = pre
    # Trigger: primer flanco de bajada de CS a partir de pre
    f = np.flatnonzero((d[3, :-1] == 1) & (d[3, 1:] == 0))
    if f.size:
        trig = int(f[0] + 1)
    return d, float(fs), trig

# ----------------------------------------------------------------------
#                          COMUNICACIÓN CON LA PICO
# ----------------------------------------------------------------------
class LA8Device:
    def __init__(self, port):
        import serial
        self.ser = serial.Serial(port, 115200, timeout=SERIAL_TIMEOUT)
        time.sleep(0.1)
        self.ser.reset_input_buffer()
        self.chip, self.fsys, self.max_samples = "?", 0, 0

    def close(self):
        self.ser.close()

    def _line(self, timeout=2.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            ln = self.ser.readline()
            if ln:
                txt = ln.decode(errors="ignore").strip()
                if txt.startswith("#") or not txt:     # banner / comentarios del firmware
                    continue
                return txt
        return None

    def info(self):
        self.ser.reset_input_buffer()
        self.ser.write(b"I\n")
        ln = self._line()
        if not ln or not ln.startswith("LA8"):
            raise RuntimeError(f"Respuesta inesperada a 'I': {ln!r}")
        p = ln.split()
        self.chip, self.fsys, self.max_samples = p[1], int(p[2]), int(p[3])
        return self.chip, self.fsys, self.max_samples

    def set_vccb(self, sel):
        """sel: 0, 3 o 5. Devuelve (ok, mV)."""
        self.ser.reset_input_buffer()
        self.ser.write(f"V {sel}\n".encode())
        ln = self._line(timeout=2.0) or ""
        p = ln.split()
        if ln.startswith("VCC"):
            return True, int(p[2])
        if ln.startswith("ERR VCCB"):
            return False, int(p[3])
        raise RuntimeError(f"Respuesta inesperada a 'V': {ln!r}")

    def status(self):
        """Devuelve (sel, oe, vccb_mV, vref_mV)."""
        self.ser.reset_input_buffer()
        self.ser.write(b"S\n")
        ln = self._line() or ""
        if not ln.startswith("STAT"):
            raise RuntimeError(f"Respuesta inesperada a 'S': {ln!r}")
        _, sel, oe, vccb, vref = ln.split()
        return int(sel), oe == "1", int(vccb), int(vref)

    def testgen(self, on):
        self.ser.write(f"G {1 if on else 0}\n".encode())
        self._line()

    def capture(self, fs, n, pre, mode, ch, status_cb, abort_evt):
        self.ser.reset_input_buffer()
        self.ser.write(f"C {int(fs)} {int(n)} {int(pre)} {int(mode)} {int(ch)}\n".encode())
        sent_abort = False
        while True:
            if abort_evt.is_set() and not sent_abort:
                self.ser.write(b"X")
                sent_abort = True
            ln = self.ser.readline()
            if not ln:
                continue
            ln = ln.decode(errors="ignore").strip()
            if ln.startswith("#") or not ln:
                continue
            if ln == "ARM":
                status_cb("Armado: esperando trigger…")
            elif ln == "ABORT":
                return None
            elif ln.startswith("ERR"):
                raise RuntimeError(ln)
            elif ln.startswith("DATA"):
                _, fs_act, n_out, trig, exact, ovf = ln.split()
                n_out = int(n_out)
                status_cb(f"Recibiendo {n_out} muestras…")
                raw = bytearray()
                t0 = time.time()
                while len(raw) < n_out:
                    chunk = self.ser.read(n_out - len(raw))
                    if chunk:
                        raw += chunk
                    elif time.time() - t0 > DATA_TIMEOUT:
                        raise RuntimeError("Timeout recibiendo datos")
                end = self._line()
                if end != "END":
                    raise RuntimeError(f"Fin de bloque inesperado: {end!r}")
                b = np.frombuffer(bytes(raw), dtype=np.uint8)
                d = ((b[None, :] >> np.arange(8, dtype=np.uint8)[:, None]) & 1).astype(np.uint8)
                return d, float(fs_act), int(trig), exact == "1", ovf == "1"

# ----------------------------------------------------------------------
#                                   GUI
# ----------------------------------------------------------------------
WINDOWS = []          # ventanas abiertas (la principal y las de comparación)


def main():
    import pyqtgraph as pg
    from pyqtgraph.Qt import QtCore, QtGui, QtWidgets

    pg.setConfigOptions(antialias=False, background=THEME_PLOT_BG, foreground="#a8a8a8")

    QSS = f"""
    QWidget {{ background: {THEME_BG}; color: #d6d6d6; font-family: "Segoe UI", "Helvetica Neue", sans-serif; font-size: 10pt; }}
    QMainWindow, QDialog {{ background: {THEME_BG}; }}
    QToolBar {{ background: {THEME_PANEL}; border: none; border-bottom: 1px solid #333; spacing: 6px; padding: 5px; }}
    QToolBar QLabel {{ background: transparent; color: #8e8e8e; }}
    QToolBar::separator {{ background: #3a3a3a; width: 1px; margin: 4px 6px; }}
    QPushButton {{ background: #333; border: 1px solid #444; border-radius: 4px; padding: 4px 12px; }}
    QPushButton:hover {{ background: #3c3c3c; border-color: {THEME_ACCENT_HI}; }}
    QPushButton:pressed {{ background: {THEME_ACCENT}; }}
    QPushButton:disabled {{ color: #5c5c5c; background: #2a2a2a; border-color: #333; }}
    QPushButton#primary {{ background: {THEME_ACCENT}; border-color: {THEME_ACCENT_HI}; color: #fff; font-weight: bold; padding: 4px 16px; }}
    QPushButton#primary:hover {{ background: {THEME_ACCENT_HI}; }}
    QPushButton#primary:disabled {{ background: #4a1a1a; color: #a07070; }}
    QComboBox, QLineEdit, QSpinBox {{ background: #2b2b2b; border: 1px solid #444; border-radius: 4px; padding: 2px 6px; min-height: 20px; }}
    QComboBox:hover, QLineEdit:hover, QSpinBox:hover {{ border-color: #5a5a5a; }}
    QComboBox:focus, QLineEdit:focus, QSpinBox:focus {{ border-color: {THEME_ACCENT_HI}; }}
    QComboBox QAbstractItemView {{ background: #2b2b2b; border: 1px solid #444; selection-background-color: {THEME_ACCENT}; selection-color: #fff; }}
    QCheckBox {{ background: transparent; spacing: 6px; }}
    QCheckBox::indicator {{ width: 14px; height: 14px; border: 1px solid #555; border-radius: 3px; background: #2b2b2b; }}
    QCheckBox::indicator:checked {{ background: {THEME_ACCENT_HI}; border-color: {THEME_ACCENT_HI}; }}
    QTableView {{ background: #1c1c1c; alternate-background-color: #222; gridline-color: #2c2c2c; border: 1px solid #2f2f2f;
                  selection-background-color: {THEME_ACCENT}; selection-color: #fff; }}
    QHeaderView::section {{ background: {THEME_PANEL}; color: #b0b0b0; border: none; border-right: 1px solid #333;
                            border-bottom: 2px solid {THEME_ACCENT}; padding: 4px 6px; font-weight: bold; }}
    QListWidget {{ background: #1c1c1c; border: 1px solid #2f2f2f; border-radius: 4px; padding: 2px; }}
    QListWidget::item {{ padding: 4px; border-radius: 3px; }}
    QListWidget::item:selected {{ background: {THEME_ACCENT}; color: #fff; }}
    QSplitter::handle {{ background: #2a2a2a; }}
    QSplitter::handle:hover {{ background: {THEME_ACCENT}; }}
    QStatusBar {{ background: {THEME_PANEL}; color: #a8a8a8; border-top: 1px solid #333; }}
    QStatusBar QLabel {{ background: transparent; }}
    QScrollBar:vertical, QScrollBar:horizontal {{ background: #1c1c1c; border: none; width: 10px; height: 10px; }}
    QScrollBar::handle {{ background: #444; border-radius: 4px; min-height: 24px; min-width: 24px; }}
    QScrollBar::handle:hover {{ background: {THEME_ACCENT_HI}; }}
    QScrollBar::add-line, QScrollBar::sub-line {{ width: 0; height: 0; }}
    QScrollBar::add-page, QScrollBar::sub-page {{ background: none; }}
    QMessageBox {{ background: {THEME_PANEL}; }}
    QToolTip {{ background: #2b2b2b; color: #ddd; border: 1px solid {THEME_ACCENT}; }}
    #header {{ background: {THEME_ACCENT}; }}
    #header QLabel {{ background: transparent; color: #fff; }}
    #title {{ font-size: 15pt; font-weight: bold; letter-spacing: 1px; }}
    #subtitle {{ color: #f0c8c8; font-size: 10pt; }}
    #pill {{ border-radius: 10px; padding: 2px 10px; font-size: 9pt; font-weight: bold; }}
    #section {{ color: #e0e0e0; font-weight: bold; padding: 2px 0; border-bottom: 2px solid {THEME_ACCENT}; }}
    #cursors {{ background: #1c1c1c; border: 1px solid #2f2f2f; border-radius: 4px; padding: 6px;
                font-family: Consolas, "DejaVu Sans Mono", monospace; font-size: 10pt; color: #e0e0e0; }}
    """

    CH_COLORS = ["#e8b33b", "#e05d5d", "#4fb3e8", "#6bd17a",
                 "#c27be8", "#e8873b", "#3be8c8", "#b0b0b0"]
    DEC_COLORS = {"UART": "#2e7dd1", "SPI": "#8a3fd1", "I2C": "#2e9e5a", "SMBus": "#c27a1d",
                  "I2S": "#1d8f86"}
    ROW, HI = 2.8, 1.0

    def ybase(ch):
        return (7 - ch) * ROW

    # ---------------- Ítem de anotaciones (burbujas tipo Saleae) ----------------
    class AnnotItem(pg.GraphicsObject):
        def __init__(self):
            super().__init__()
            self.x0 = self.x1 = self.y0 = np.zeros(0)
            self.labels, self.colors, self.marks = [], [], []
            self._br = QtCore.QRectF()

        def set_items(self, x0, x1, y0, labels, colors, marks):
            self.prepareGeometryChange()
            self.x0, self.x1, self.y0 = np.asarray(x0), np.asarray(x1), np.asarray(y0)
            self.labels, self.colors, self.marks = labels, colors, marks
            if self.x0.size or marks:
                xs = list(self.x0) + [m[0] for m in marks]
                xe = list(self.x1) + [m[0] for m in marks]
                self._br = QtCore.QRectF(min(xs), -1, max(xe) - min(xs) + 1e-12, 8 * ROW + 2)
            else:
                self._br = QtCore.QRectF()
            self.update()

        def boundingRect(self):
            return self._br

        def paint(self, p, *args):
            vb = self.getViewBox()
            if vb is None:
                return
            (vx0, vx1), _ = vb.viewRange()
            tr = p.transform()
            p.save()
            p.resetTransform()
            font = QtGui.QFont("Consolas", 8)
            p.setFont(font)
            fm = QtGui.QFontMetrics(font)
            # marcadores (START/STOP/CS)
            for (x, yb, txt, col) in self.marks:
                if x < vx0 or x > vx1:
                    continue
                pt = tr.map(QtCore.QPointF(x, yb + HI + 0.65))
                p.setPen(QtGui.QPen(QtGui.QColor(col), 1))
                p.setBrush(QtGui.QColor(col))
                p.drawEllipse(pt, 3, 3)
            if self.x0.size:
                vis = np.flatnonzero((self.x1 >= vx0) & (self.x0 <= vx1))
                if vis.size > 20000:
                    vis = vis[:: vis.size // 20000 + 1]
                for i in vis:
                    r = tr.mapRect(QtCore.QRectF(self.x0[i], self.y0[i] + HI + 0.25,
                                                 self.x1[i] - self.x0[i], 0.85))
                    if r.width() < 1.0:
                        r.setWidth(1.0)
                    col = QtGui.QColor(self.colors[i])
                    p.setPen(QtGui.QPen(col.lighter(130), 1))
                    fill = QtGui.QColor(col)
                    fill.setAlpha(170)
                    p.setBrush(fill)
                    p.drawRoundedRect(r, 3, 3)
                    lab = self.labels[i]
                    if r.width() > fm.horizontalAdvance(lab) + 6:
                        p.setPen(QtGui.QColor("#ffffff"))
                        p.drawText(r, QtCore.Qt.AlignmentFlag.AlignCenter, lab)
            p.restore()

    # ---------------- Modelo de la tabla ----------------
    class FrameModel(QtCore.QAbstractTableModel):
        HEAD = ["#", "Tiempo", "Analizador", "Tipo", "HEX", "DEC", "BIN", "ASCII", "Info"]

        def __init__(self):
            super().__init__()
            self.rows, self.fs, self.trig = [], 1.0, 0

        def set_rows(self, rows, fs, trig):
            self.beginResetModel()
            self.rows, self.fs, self.trig = rows, fs, trig
            self.endResetModel()

        def rowCount(self, parent=QtCore.QModelIndex()):
            return len(self.rows)

        def columnCount(self, parent=QtCore.QModelIndex()):
            return len(self.HEAD)

        def headerData(self, s, o, role=QtCore.Qt.ItemDataRole.DisplayRole):
            if role == QtCore.Qt.ItemDataRole.DisplayRole and o == QtCore.Qt.Orientation.Horizontal:
                return self.HEAD[s]
            return None

        def data(self, idx, role=QtCore.Qt.ItemDataRole.DisplayRole):
            f = self.rows[idx.row()]
            c = idx.column()
            if role == QtCore.Qt.ItemDataRole.ForegroundRole and f.error:
                return QtGui.QColor("#ff6b6b")
            if role != QtCore.Qt.ItemDataRole.DisplayRole:
                return None
            bits = 7 if f.kind == "addr" else f.bits
            if c == 0:
                return idx.row()
            if c == 1:
                return pg.siFormat((f.s0 - self.trig) / self.fs, suffix="s", precision=6)
            if c == 2:
                return f.decoder
            if c == 3:
                return {"data": "Dato", "addr": "Dirección", "start": "START", "stop": "STOP",
                        "rstart": "START rep.", "pec": "PEC", "err": "Error"}.get(f.kind, f.kind)
            if f.value is None:
                return f.info if c == 8 else ""
            if c == 4:
                return fmt_value(f.value, "HEX", bits)
            if c == 5:
                return fmt_value(f.value, "DEC", bits, f.signed)
            if c == 6:
                return fmt_value(f.value, "BIN", bits)
            if c == 7:
                return ascii_repr(f.value) if (f.kind != "addr" and bits <= 8) else ""
            if c == 8:
                return f.info
            return None

    # ---------------- Diálogo de nuevo analizador ----------------
    class DecoderDialog(QtWidgets.QDialog):
        def __init__(self, parent=None):
            super().__init__(parent)
            self.setWindowTitle("Agregar analizador")
            self.form = QtWidgets.QFormLayout(self)
            self.type_cb = QtWidgets.QComboBox()
            self.type_cb.addItems(list(DECODER_SPECS.keys()))
            self.form.addRow("Protocolo", self.type_cb)
            self.fields_w = QtWidgets.QWidget()
            self.fields_l = QtWidgets.QFormLayout(self.fields_w)
            self.form.addRow(self.fields_w)
            bb = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.StandardButton.Ok |
                                            QtWidgets.QDialogButtonBox.StandardButton.Cancel)
            bb.accepted.connect(self.accept)
            bb.rejected.connect(self.reject)
            self.form.addRow(bb)
            self.widgets = {}
            self.type_cb.currentTextChanged.connect(self._build)
            self._build(self.type_cb.currentText())

        def _build(self, t):
            while self.fields_l.rowCount():
                self.fields_l.removeRow(0)
            self.widgets = {}
            defaults = {"baud": 115200, "bits": 8, "stop": 1, "mode": 0, "msb": 1,
                        "miso": -1, "cs": -1, "sda": 6, "scl": 7, "clk": 1, "mosi": 2, "ch": 0,
                        "sck": 1, "ws": 3, "sd": 2}
            if t == "I2S":
                defaults["bits"] = 16
            for key, label, kind in DECODER_SPECS[t]:
                if kind in ("ch", "chopt", "int"):
                    w = QtWidgets.QSpinBox()
                    if kind == "ch":
                        w.setRange(0, 7)
                    elif kind == "chopt":
                        w.setRange(-1, 7)
                    else:
                        w.setRange(0, 10_000_000)
                    w.setValue(defaults.get(key, 0))
                elif kind == "bool":
                    w = QtWidgets.QCheckBox()
                    w.setChecked(bool(defaults.get(key, 0)))
                else:
                    w = QtWidgets.QComboBox()
                    w.addItems(kind)
                self.widgets[key] = (w, kind)
                self.fields_l.addRow(label, w)

        def config(self):
            cfg = {"type": self.type_cb.currentText()}
            for key, (w, kind) in self.widgets.items():
                if isinstance(w, QtWidgets.QSpinBox):
                    cfg[key] = w.value()
                elif isinstance(w, QtWidgets.QCheckBox):
                    cfg[key] = int(w.isChecked())
                else:
                    cfg[key] = w.currentText()
            return cfg

    # ---------------- Ventana principal ----------------
    class MainWindow(QtWidgets.QMainWindow):
        sig_done = QtCore.Signal(object)
        sig_status = QtCore.Signal(str)
        sig_error = QtCore.Signal(str)

        def __init__(self):
            super().__init__()
            self.setWindowTitle("LA8 Viewer — ELECTGPL")
            self.resize(1400, 850)
            self.dev = None
            self.data, self.fs, self.trig, self.t0 = None, 1.0, 0, -0.5
            self.decoders = [dict(x) for x in DEFAULT_DECODERS]
            self.frames = []
            self.abort_evt = threading.Event()
            self._build_ui()
            self.sig_done.connect(self._on_capture_done)
            self.sig_status.connect(self.statusBar().showMessage)
            self.sig_error.connect(self._on_error)

        # ---- UI ----
        def _build_ui(self):
            tb = QtWidgets.QToolBar()
            tb.setMovable(False)
            self.addToolBar(tb)
            self.port_le = QtWidgets.QLineEdit(PORT)
            self.port_le.setMaximumWidth(120)
            tb.addWidget(QtWidgets.QLabel(" Puerto "))
            tb.addWidget(self.port_le)
            self.btn_conn = QtWidgets.QPushButton("Conectar")
            self.btn_conn.clicked.connect(self._connect)
            tb.addWidget(self.btn_conn)
            tb.addSeparator()

            self.fs_cb = QtWidgets.QComboBox()
            self.fs_cb.setEditable(True)
            for f in FS_PRESETS:
                self.fs_cb.addItem(pg.siFormat(f, suffix="S/s"), f)
            self.fs_cb.setCurrentIndex(FS_PRESETS.index(DEFAULT_FS) if DEFAULT_FS in FS_PRESETS else 0)
            tb.addWidget(QtWidgets.QLabel(" fs "))
            tb.addWidget(self.fs_cb)

            self.n_cb = QtWidgets.QComboBox()
            for s in SAMPLE_PRESETS:
                self.n_cb.addItem(f"{s // 1000} k", s)
            self.n_cb.setCurrentIndex(SAMPLE_PRESETS.index(DEFAULT_SAMPLES)
                                      if DEFAULT_SAMPLES in SAMPLE_PRESETS else 0)
            tb.addWidget(QtWidgets.QLabel(" Muestras "))
            tb.addWidget(self.n_cb)

            self.pre_sb = QtWidgets.QSpinBox()
            self.pre_sb.setRange(0, 95)
            self.pre_sb.setValue(DEFAULT_PRE_PCT)
            self.pre_sb.setSuffix(" %")
            tb.addWidget(QtWidgets.QLabel(" Pre-trig "))
            tb.addWidget(self.pre_sb)

            self.tmode_cb = QtWidgets.QComboBox()
            self.tmode_cb.addItems(["Ninguno", "Subida", "Bajada"])
            self.tmode_cb.setCurrentText(DEFAULT_TRIG_MODE)
            self.tch_cb = QtWidgets.QComboBox()
            self.tch_cb.addItems(channel_labels(DEFAULT_DECODERS))
            self.tch_cb.setCurrentIndex(DEFAULT_TRIG_CH)
            tb.addWidget(QtWidgets.QLabel(" Trigger "))
            tb.addWidget(self.tmode_cb)
            tb.addWidget(self.tch_cb)
            tb.addSeparator()

            self.btn_cap = QtWidgets.QPushButton("▶ Capturar")
            self.btn_cap.setObjectName("primary")
            self.btn_cap.clicked.connect(self._capture)
            self.btn_abort = QtWidgets.QPushButton("■ Abortar")
            self.btn_abort.clicked.connect(lambda: self.abort_evt.set())
            self.btn_abort.setEnabled(False)
            self.vccb_cb = QtWidgets.QComboBox()
            self.vccb_cb.addItems(["3,3 V", "5 V", "Apagado"])
            self.vccb_cb.setCurrentText(DEFAULT_VCCB)
            self.vccb_cb.activated.connect(lambda _: self._apply_vccb())
            if FRONTEND == "TXS0108":
                tb.addWidget(QtWidgets.QLabel(" VCCB "))
                tb.addWidget(self.vccb_cb)
            else:
                tb.addWidget(QtWidgets.QLabel(f" Entrada {FRONTEND_LIMITS.get(FRONTEND, (0, 0, '?'))[2]} "))
            self.btn_volt = QtWidgets.QPushButton("Medir")
            self.btn_volt.clicked.connect(self._read_volts)
            if FRONTEND == "TXS0108" or VREF_SENSE:
                tb.addWidget(self.btn_volt)
            self.volt_lbl = QtWidgets.QLabel("  VCCB: —  VREF: —  " if FRONTEND == "TXS0108" else "  VREF: —  ")
            self.volt_lbl.setStyleSheet("font-family: Consolas; color: #7fd88f;")
            tb.addWidget(self.volt_lbl)
            tb.addSeparator()
            self.chk_gen = QtWidgets.QCheckBox("Generador de prueba")
            self.chk_gen.toggled.connect(self._toggle_gen)
            for w in (self.btn_cap, self.btn_abort):
                tb.addWidget(w)

            # Segunda fila: archivos, demo, generador y formato
            self.addToolBarBreak()
            tb = QtWidgets.QToolBar()
            tb.setMovable(False)
            self.addToolBar(tb)
            tb.addWidget(self.chk_gen)
            tb.addSeparator()

            for txt, fn in (("Demo", self._demo), ("Guardar .npz", self._save),
                            ("Abrir .npz", self._load), ("Exportar VCD", self._export_vcd)):
                b = QtWidgets.QPushButton(txt)
                b.clicked.connect(fn)
                tb.addWidget(b)
            tb.addSeparator()
            # Comparación de capturas
            for txt, fn in (("Abrir en ventana…", self._open_windows),
                            ("Superponer .npz…", self._load_overlay),
                            ("Quitar superposición", self._clear_overlay)):
                b = QtWidgets.QPushButton(txt)
                b.clicked.connect(fn)
                tb.addWidget(b)
            self.chk_sync = QtWidgets.QCheckBox("Sincronizar zoom")
            self.chk_sync.toggled.connect(self._toggle_sync)
            tb.addWidget(self.chk_sync)

            # Tercera fila: desplazamiento de la referencia superpuesta
            self.addToolBarBreak()
            self.tb_ov = QtWidgets.QToolBar()
            self.tb_ov.setMovable(False)
            self.addToolBar(self.tb_ov)
            self.ov_name = QtWidgets.QLabel(" Referencia: — ")
            self.tb_ov.addWidget(self.ov_name)
            self.tb_ov.addWidget(QtWidgets.QLabel("  Desplazamiento "))
            self.ov_shift = QtWidgets.QDoubleSpinBox()
            self.ov_shift.setDecimals(1)
            self.ov_shift.setRange(-1e9, 1e9)
            self.ov_shift.setSuffix(" ns")
            self.ov_shift.setMinimumWidth(130)
            self.ov_shift.setKeyboardTracking(False)
            self.ov_shift.valueChanged.connect(lambda _: self._draw_overlay())
            self.tb_ov.addWidget(self.ov_shift)
            for txt, k in (("◀◀ 10", -10), ("◀ 1", -1), ("1 ▶", 1), ("10 ▶▶", 10)):
                b = QtWidgets.QPushButton(txt)
                b.setToolTip(f"Desplaza la referencia {abs(k)} muestra(s)")
                b.clicked.connect(lambda _, k=k: self._nudge_overlay(k))
                self.tb_ov.addWidget(b)
            b = QtWidgets.QPushButton("Alinear auto")
            b.setToolTip("Busca el desplazamiento que mejor hace coincidir la referencia con la "
                         "captura actual, dentro del tramo visible")
            b.clicked.connect(self._auto_align_overlay)
            self.tb_ov.addWidget(b)
            b = QtWidgets.QPushButton("Cero")
            b.clicked.connect(lambda: self.ov_shift.setValue(0.0))
            self.tb_ov.addWidget(b)
            self.chk_ov_sep = QtWidgets.QCheckBox("Separar verticalmente")
            self.chk_ov_sep.setToolTip("Dibuja la referencia un poco más abajo para ver ambas formas de onda")
            self.chk_ov_sep.toggled.connect(lambda _: self._draw_overlay())
            self.tb_ov.addWidget(self.chk_ov_sep)
            self.tb_ov.setVisible(False)
            self.radix_cb = QtWidgets.QComboBox()
            self.radix_cb.addItems(["HEX", "DEC", "BIN", "ASCII"])
            self.radix_cb.currentTextChanged.connect(lambda _: self._draw_annotations())
            tb.addWidget(QtWidgets.QLabel(" Formato "))
            tb.addWidget(self.radix_cb)

            # Plot
            self.plot = pg.PlotWidget()
            self.plot.showGrid(x=True, y=False, alpha=0.15)
            self.plot.setLabel("bottom", "Tiempo", units="s")
            vb = self.plot.getViewBox()
            vb.setMouseEnabled(x=True, y=False)
            self.plot.setYRange(-0.5, 8 * ROW, padding=0)
            ax = self.plot.getAxis("left")
            self.left_axis = ax
            ax.setWidth(120)
            self.curves = []
            # Superposición de referencia (debajo de la captura actual) y marcas de diferencia
            self.ref = None
            self.ref_curves, self.diff_curves = [], []
            for i in range(8):
                col = QtGui.QColor(CH_COLORS[i])
                col.setAlpha(110)
                rc = pg.PlotDataItem(pen=pg.mkPen(col, width=5))
                self.plot.addItem(rc)
                self.ref_curves.append(rc)
                dc = pg.PlotDataItem(pen=pg.mkPen("#ff3b3b", width=4), connect="finite")
                self.plot.addItem(dc)
                self.diff_curves.append(dc)
            for i in range(8):
                c = pg.PlotDataItem(pen=pg.mkPen(CH_COLORS[i], width=2))
                self.plot.addItem(c)
                self.curves.append(c)
            self.annot = AnnotItem()
            self.plot.addItem(self.annot)
            self.trig_line = pg.InfiniteLine(0, angle=90, pen=pg.mkPen("#ffffff", style=QtCore.Qt.PenStyle.DashLine))
            self.plot.addItem(self.trig_line)
            self.cur = []
            for k, col in enumerate(("#ffd84d", "#4dd2ff")):
                ln = pg.InfiniteLine(0, angle=90, movable=True, pen=pg.mkPen(col, width=1.5),
                                     label=f"T{k + 1}", labelOpts={"position": 0.97, "color": col})
                ln.sigPositionChanged.connect(self._update_cursors)
                self.plot.addItem(ln)
                self.cur.append(ln)

            # Panel inferior: analizadores + tabla
            dec_w = QtWidgets.QWidget()
            dl = QtWidgets.QVBoxLayout(dec_w)
            sec = QtWidgets.QLabel("ANALIZADORES")
            sec.setObjectName("section")
            dl.addWidget(sec)
            self.dec_list = QtWidgets.QListWidget()
            dl.addWidget(self.dec_list)
            hb = QtWidgets.QHBoxLayout()
            b_add = QtWidgets.QPushButton("Agregar")
            b_add.clicked.connect(self._add_decoder)
            b_del = QtWidgets.QPushButton("Quitar")
            b_del.clicked.connect(self._del_decoder)
            hb.addWidget(b_add)
            hb.addWidget(b_del)
            dl.addLayout(hb)
            self.cur_lbl = QtWidgets.QLabel("—")
            self.cur_lbl.setObjectName("cursors")
            dl.addWidget(self.cur_lbl)
            b_cur = QtWidgets.QPushButton("Traer cursores a la vista")
            b_cur.setToolTip("Ubica T1 y T2 dentro del tramo visible. "
                             "También: doble clic = T1, Shift + doble clic = T2")
            b_cur.clicked.connect(self._bring_cursors)
            dl.addWidget(b_cur)
            self.plot.scene().sigMouseClicked.connect(self._plot_clicked)

            self.model = FrameModel()
            self.table = QtWidgets.QTableView()
            self.table.setModel(self.model)
            self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectionBehavior.SelectRows)
            self.table.verticalHeader().setVisible(False)
            self.table.setAlternatingRowColors(True)
            self.table.setShowGrid(False)
            self.table.horizontalHeader().setStretchLastSection(True)
            self.table.setStyleSheet("QTableView { font-family: Consolas, 'DejaVu Sans Mono', monospace; font-size: 10pt; }")
            self.table.clicked.connect(self._goto_frame)

            bottom = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
            bottom.addWidget(dec_w)
            bottom.addWidget(self.table)
            bottom.setSizes([300, 1100])
            split = QtWidgets.QSplitter(QtCore.Qt.Orientation.Vertical)
            split.addWidget(self.plot)
            split.addWidget(bottom)
            split.setSizes([560, 290])
            # Header estilo Node-RED
            header = QtWidgets.QFrame()
            header.setObjectName("header")
            header.setFixedHeight(46)
            hl = QtWidgets.QHBoxLayout(header)
            hl.setContentsMargins(10, 4, 14, 4)
            # Logo: se busca junto al script; si no está, queda solo el título de texto
            logo_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), LOGO_FILE)
            pm = QtGui.QPixmap(logo_path)
            if not pm.isNull():
                dpr = self.devicePixelRatioF()
                pm = pm.scaledToHeight(int(LOGO_HEIGHT * dpr),
                                       QtCore.Qt.TransformationMode.SmoothTransformation)
                pm.setDevicePixelRatio(dpr)
                logo = QtWidgets.QLabel()
                logo.setPixmap(pm)
                hl.addWidget(logo)
                sep = QtWidgets.QFrame()
                sep.setFixedSize(1, LOGO_HEIGHT - 6)
                sep.setStyleSheet("background: rgba(255,255,255,0.35);")
                hl.addSpacing(10)
                hl.addWidget(sep)
                hl.addSpacing(10)
            t = QtWidgets.QLabel("LA8")
            t.setObjectName("title")
            st = QtWidgets.QLabel("Logic Analyzer · RP2040 / RP2350 · ELECTGPL")
            st.setObjectName("subtitle")
            hl.addWidget(t)
            hl.addSpacing(10)
            hl.addWidget(st)
            hl.addStretch(1)
            self.pill = QtWidgets.QLabel()
            self.pill.setObjectName("pill")
            hl.addWidget(self.pill)
            self._set_pill(False)

            self.setMenuWidget(header)      # queda por encima de la toolbar
            self.setCentralWidget(split)
            self._refresh_dec_list()
            self.statusBar().showMessage("Sin conexión. Use 'Conectar' o 'Demo'.")

        # ---- Dispositivo ----
        def _connect(self):
            try:
                if self.dev:
                    self.dev.close()
                self.dev = LA8Device(self.port_le.text().strip())
                chip, fsys, maxs = self.dev.info()
                self.statusBar().showMessage(
                    f"Conectado: {chip}, f_sys = {pg.siFormat(fsys, suffix='Hz')}, "
                    f"profundidad máx. = {maxs} muestras")
                self._set_pill(True, chip)
                if FRONTEND == "TXS0108":
                    self._apply_vccb()
                else:
                    self._read_volts()
            except Exception as e:
                self.dev = None
                self._set_pill(False)
                self._on_error(f"No se pudo conectar: {e}")

        # ---- Front-end TXS0108E ----
        def _apply_vccb(self):
            if not self.dev:
                return
            sel = {"3,3 V": 3, "5 V": 5, "Apagado": 0}[self.vccb_cb.currentText()]
            try:
                ok, mv = self.dev.set_vccb(sel)
                if not ok:
                    self._on_error(f"VCCB fuera de rango: {mv / 1000:.2f} V. "
                                   "El front-end quedó deshabilitado (OE = 0).")
                self._read_volts()
            except Exception as e:
                self._on_error(str(e))

        def _read_volts(self):
            if not self.dev:
                return
            try:
                sel, oe, vccb, vref = self.dev.status()
            except Exception as e:
                self._on_error(str(e))
                return
            if FRONTEND != "TXS0108":
                self._check_vref_lv4t(vref)
                return
            txt = f"  VCCB: {vccb / 1000:.2f} V  OE: {'on' if oe else 'off'}  "
            if vref > 500:
                txt += f"VREF: {vref / 1000:.2f} V  "
            else:
                txt += "VREF: n/c  "
            self.volt_lbl.setText(txt)
            if vref > 500 and vccb > 500 and abs(vref - vccb) / vccb * 100 > VREF_MISMATCH_PCT:
                self.volt_lbl.setStyleSheet("font-family: Consolas; color: #ff6b6b;")
                self._on_error(f"VREF del circuito ({vref / 1000:.2f} V) no coincide con VCCB "
                               f"({vccb / 1000:.2f} V). Cambie la selección de VCCB: los pull-ups "
                               "de 4 kΩ del TXS0108E inyectarían corriente o los umbrales "
                               "quedarían sin margen.")
            else:
                self.volt_lbl.setStyleSheet("font-family: Consolas; color: #7fd88f;")

        def _check_vref_lv4t(self, vref):
            ok_style = "font-family: Consolas; color: #7fd88f;"
            bad_style = "font-family: Consolas; color: #ff6b6b;"
            if not VREF_SENSE:
                self.volt_lbl.setText("")
                return
            if vref <= VREF_NC_MV:
                self.volt_lbl.setText("  VREF: n/c  ")
                self.volt_lbl.setStyleSheet(ok_style)
                return
            self.volt_lbl.setText(f"  VREF: {vref / 1000:.2f} V  ")
            vmin, vmax, rango = FRONTEND_LIMITS.get(FRONTEND, (0, 5500, "?"))
            state = "hi" if vref > vmax else "lo" if vref < vmin else "ok"
            first = state != getattr(self, "_vref_state", None)   # el aviso emergente sale una sola vez
            self._vref_state = state
            if vref > vmax:
                self.volt_lbl.setStyleSheet(bad_style)
                if first: self._on_error(f"VREF = {vref / 1000:.2f} V supera el máximo del front-end {FRONTEND} "
                               f"({vmax / 1000:.1f} V). Desconecte las puntas para no dañar las entradas.")
            elif vref < vmin:
                self.volt_lbl.setStyleSheet(bad_style)
                if first: self._on_error(f"VREF = {vref / 1000:.2f} V está por debajo del rango del front-end "
                               f"{FRONTEND} ({rango}): el nivel alto queda cerca del umbral de entrada "
                               "y la captura puede no ser confiable.")
            else:
                self.volt_lbl.setStyleSheet(ok_style)

        def _set_pill(self, ok, chip=""):
            if ok:
                self.pill.setText(f"●  {chip} conectado")
                self.pill.setStyleSheet("background: #1f5f2e; color: #b9f5c6;")
            else:
                self.pill.setText("●  Desconectado")
                self.pill.setStyleSheet("background: #3a0a0a; color: #f0b0b0;")

        def _toggle_gen(self, on):
            if self.dev:
                try:
                    self.dev.testgen(on)
                except Exception as e:
                    self._on_error(str(e))

        def _fs_value(self):
            d = self.fs_cb.currentData()
            if d is not None and self.fs_cb.currentText() == self.fs_cb.itemText(self.fs_cb.currentIndex()):
                return int(d)
            txt = self.fs_cb.currentText().replace("S/s", "").strip()
            mult = {"k": 1e3, "M": 1e6}
            try:
                if txt and txt[-1] in mult:
                    return int(float(txt[:-1]) * mult[txt[-1]])
                return int(float(txt))
            except ValueError:
                return DEFAULT_FS

        def _capture(self):
            if not self.dev:
                self._on_error("Conecte primero el dispositivo (o use Demo).")
                return
            fs = self._fs_value()
            n = int(self.n_cb.currentData())
            if self.dev.max_samples:
                n = min(n, self.dev.max_samples)
            pre = int(n * self.pre_sb.value() / 100)
            mode = self.tmode_cb.currentIndex()
            ch = self.tch_cb.currentIndex()
            self.abort_evt.clear()
            self.btn_cap.setEnabled(False)
            self.btn_abort.setEnabled(True)
            self.statusBar().showMessage("Capturando…")

            def worker():
                try:
                    r = self.dev.capture(fs, n, pre, mode, ch, self.sig_status.emit, self.abort_evt)
                    self.sig_done.emit(r)
                except Exception as e:
                    self.sig_error.emit(str(e))
                    self.sig_done.emit(None)

            threading.Thread(target=worker, daemon=True).start()

        def _on_capture_done(self, r):
            self.btn_cap.setEnabled(True)
            self.btn_abort.setEnabled(False)
            if r is None:
                self.statusBar().showMessage("Captura abortada o fallida.")
                return
            d, fs, trig, exact, ovf = r
            msg = f"{d.shape[1]} muestras a {pg.siFormat(fs, suffix='S/s')}"
            if not exact:
                msg += "  | trigger aproximado (pulso < 1/fs)"
            if ovf:
                msg += "  | ATENCIÓN: overflow de FIFO, se perdieron muestras"
            self._set_data(d, fs, trig)
            self.statusBar().showMessage(msg)

        def _on_error(self, msg):
            QtWidgets.QMessageBox.warning(self, "LA8", msg)

        # ---- Datos ----
        def _demo(self):
            fs = 10_000_000
            d, fs, trig = synth_capture(fs, 200_000, 20_000)
            self._set_data(d, fs, trig)
            self.statusBar().showMessage("Demo sintética: UART + SPI + I2C + SMBus (con PEC), igual que GEN_Electgpl_S3")

        def _set_data(self, d, fs, trig):
            self.data, self.fs, self.trig = d, fs, trig
            # Convención: la transición entre la muestra k-1 y k se ubica en k-0.5;
            # t = 0 es el flanco de trigger (trig = primera muestra con el nivel nuevo).
            self.t0 = trig - 0.5
            n = d.shape[1]
            for i in range(8):
                v = d[i]
                tr = np.flatnonzero(v[1:] != v[:-1]) + 1
                starts = np.concatenate(([0], tr))
                y = v[starts].astype(float) * HI + ybase(i)
                x = (np.concatenate((starts, [n])).astype(float) - 0.5 - self.t0) / fs
                self.curves[i].setData(x, y, stepMode="center")
            x0, x1 = (-0.5 - self.t0) / fs, (n - 0.5 - self.t0) / fs
            vb = self.plot.getViewBox()
            vb.setLimits(xMin=x0, xMax=x1, minXRange=4.0 / fs)
            self.plot.setXRange(x0, x1, padding=0)
            self.cur[0].setValue(x0 + 0.25 * (x1 - x0))
            self.cur[1].setValue(x0 + 0.30 * (x1 - x0))
            self._run_decoders()
            if self.ref is not None:
                self._draw_overlay()

        def _run_decoders(self):
            self.frames = []
            if self.data is None:
                return
            for i, cfg in enumerate(self.decoders):
                try:
                    self.frames += run_decoder(self.data, self.fs, cfg, i)
                except Exception as e:
                    self.statusBar().showMessage(f"Error en {cfg['type']}: {e}")
            self.frames.sort(key=lambda f: f.s0)
            self.model.set_rows(self.frames, self.fs, self.t0)
            self.table.resizeColumnsToContents()
            self._draw_annotations()

        def _draw_annotations(self):
            radix = self.radix_cb.currentText()
            x0, x1, y0, labels, colors, marks = [], [], [], [], [], []
            for f in self.frames:
                dtype = f.decoder.split("#")[0]
                col = DEC_COLORS.get(dtype, "#777777")
                if f.error:
                    col = "#c0392b"
                if f.value is None and f.s1 - f.s0 < 1e-9:
                    mcol = {"start": "#3ddc84", "rstart": "#ffd84d", "stop": "#ff6b6b"}.get(f.kind, "#aaaaaa")
                    marks.append(((f.s0 - self.t0) / self.fs, ybase(f.ch), f.info, mcol))
                    continue
                if f.value is None:
                    continue
                bits = 7 if f.kind == "addr" else f.bits
                lab = fmt_value(f.value, radix, bits, f.signed)
                if dtype == "I2S":
                    lab = f"{f.info[0]} {lab}"
                if f.kind == "addr":
                    lab = f"{'Rd' if ' R ' in f.info else 'Wr'} {lab}"
                if f.kind == "pec":
                    lab = f"PEC {lab}"
                x0.append((f.s0 - self.t0) / self.fs)
                x1.append((f.s1 - self.t0) / self.fs)
                y0.append(ybase(f.ch))
                labels.append(lab)
                colors.append(col)
            self.annot.set_items(x0, x1, y0, labels, colors, marks)

        def _refresh_dec_list(self):
            self.dec_list.clear()
            for i, cfg in enumerate(self.decoders):
                self.dec_list.addItem(f"#{i + 1}  {decoder_label(cfg)}")
            self._update_channel_labels()

        def _update_channel_labels(self):
            self.ch_labels = channel_labels(self.decoders)
            self.left_axis.setTicks([[(ybase(i) + HI / 2, self.ch_labels[i]) for i in range(8)]])
            idx = self.tch_cb.currentIndex()
            for i, name in enumerate(self.ch_labels):
                self.tch_cb.setItemText(i, name)
            self.tch_cb.setCurrentIndex(idx)

        def _add_decoder(self):
            dlg = DecoderDialog(self)
            if dlg.exec():
                self.decoders.append(dlg.config())
                self._refresh_dec_list()
                self._run_decoders()

        def _del_decoder(self):
            r = self.dec_list.currentRow()
            if r >= 0:
                del self.decoders[r]
                self._refresh_dec_list()
                self._run_decoders()

        # ---- Navegación / cursores ----
        def _goto_frame(self, idx):
            f = self.frames[idx.row()]
            (vx0, vx1), _ = self.plot.getViewBox().viewRange()
            w = vx1 - vx0
            fw = max((f.s1 - f.s0) / self.fs, 1.0 / self.fs)
            if w > 200 * fw or w < 2 * fw:
                w = 30 * fw
            c = ((f.s0 + f.s1) / 2 - self.t0) / self.fs
            self.plot.setXRange(c - w / 2, c + w / 2, padding=0)

        def _bring_cursors(self):
            (vx0, vx1), _ = self.plot.getViewBox().viewRange()
            w = vx1 - vx0
            self.cur[0].setValue(vx0 + w / 3)
            self.cur[1].setValue(vx0 + 2 * w / 3)

        def _plot_clicked(self, ev):
            if not ev.double():
                return
            vb = self.plot.getViewBox()
            if not vb.sceneBoundingRect().contains(ev.scenePos()):
                return
            x = vb.mapSceneToView(ev.scenePos()).x()
            shift = bool(ev.modifiers() & QtCore.Qt.KeyboardModifier.ShiftModifier)
            self.cur[1 if shift else 0].setValue(x)

        def _update_cursors(self):
            t1, t2 = self.cur[0].value(), self.cur[1].value()
            dt = t2 - t1
            fr = pg.siFormat(1.0 / abs(dt), suffix="Hz") if abs(dt) > 0 else "—"
            self.cur_lbl.setText(f"T1 = {pg.siFormat(t1, suffix='s', precision=6)}\n"
                                 f"T2 = {pg.siFormat(t2, suffix='s', precision=6)}\n"
                                 f"ΔT = {pg.siFormat(dt, suffix='s', precision=6)}\n"
                                 f"1/ΔT = {fr}")

        # ---- Archivos ----
        def _save(self):
            if self.data is None:
                return
            fn, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Guardar captura", "captura.npz", "*.npz")
            if fn:
                np.savez_compressed(fn, data=self.data, fs=self.fs, trig=self.trig,
                                    names=np.array(self.ch_labels))

        def _load(self):
            fn, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Abrir captura", "", "*.npz")
            if fn:
                z = np.load(fn)
                self._set_data(z["data"], float(z["fs"]), int(z["trig"]))
                self.setWindowTitle(f"LA8 Viewer — {os.path.basename(fn)}")

        # ---- Comparación de capturas ----
        def _open_windows(self):
            fns, _ = QtWidgets.QFileDialog.getOpenFileNames(self, "Abrir capturas en ventanas nuevas", "", "*.npz")
            for k, fn in enumerate(fns):
                z = np.load(fn)
                nw = MainWindow()
                nw.decoders = [dict(c) for c in self.decoders]      # mismos analizadores
                nw.radix_cb.setCurrentText(self.radix_cb.currentText())
                nw._refresh_dec_list()
                nw._set_data(z["data"], float(z["fs"]), int(z["trig"]))
                nw.setWindowTitle(f"LA8 Viewer — {os.path.basename(fn)}")
                nw.resize(self.size())
                nw.move(self.pos() + QtCore.QPoint(40 * (k + 1), 40 * (k + 1)))
                nw.show()
                WINDOWS.append(nw)
                if self.chk_sync.isChecked():
                    nw.chk_sync.setChecked(True)

        def _toggle_sync(self, on):
            """Vincula el eje de tiempo (t = 0 en el trigger) de todas las ventanas abiertas."""
            for w in WINDOWS:
                if w.chk_sync.isChecked() != on:
                    w.chk_sync.blockSignals(True)
                    w.chk_sync.setChecked(on)
                    w.chk_sync.blockSignals(False)
            for w in WINDOWS:
                vb = w.plot.getViewBox()
                if on and not getattr(w, "_sync_connected", False):
                    vb.sigXRangeChanged.connect(w._on_xrange)
                    w._sync_connected = True
                    vb.setLimits(xMin=None, xMax=None)
            if on:
                self._on_xrange(self.plot.getViewBox(), self.plot.getViewBox().viewRange()[0])

        def _on_xrange(self, vb, rng):
            # Mismo rango de tiempo en todas las ventanas (alineadas por t = 0 = trigger).
            # No se usa setXLink porque pyqtgraph alinea por posición en pantalla.
            if not self.chk_sync.isChecked() or getattr(MainWindow, "_syncing", False):
                return
            MainWindow._syncing = True
            try:
                for w in WINDOWS:
                    if w is not self and w.chk_sync.isChecked():
                        w.plot.setXRange(rng[0], rng[1], padding=0)
            finally:
                MainWindow._syncing = False

        def _load_overlay(self):
            if self.data is None:
                self._on_error("Primero cargue o capture una señal para comparar.")
                return
            fn, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Superponer captura de referencia", "", "*.npz")
            if not fn:
                return
            z = np.load(fn)
            self.ref = (z["data"], float(z["fs"]), int(z["trig"]) - 0.5, os.path.basename(fn))
            self.ov_name.setText(f" Referencia: {os.path.basename(fn)} ")
            self.ov_shift.blockSignals(True)
            self.ov_shift.setValue(0.0)
            self.ov_shift.setSingleStep(1e9 / self.fs)          # una muestra de la captura actual
            self.ov_shift.blockSignals(False)
            self.tb_ov.setVisible(True)
            self._draw_overlay()

        def _nudge_overlay(self, k):
            if self.ref is not None:
                self.ov_shift.setValue(self.ov_shift.value() + k * 1e9 / self.fs)

        def _auto_align_overlay(self):
            """Correlación cruzada (FFT) de los 8 canales en el tramo visible: elige el
            desplazamiento entero, en muestras de la captura actual, con más coincidencias."""
            if self.ref is None or self.data is None:
                return
            rd, rfs, rt0, _ = self.ref
            d, fs, t0 = self.data, self.fs, self.t0
            n = d.shape[1]
            (vx0, vx1), _ = self.plot.getViewBox().viewRange()
            i0 = int(max(0, np.floor(vx0 * fs + t0)))
            i1 = int(min(n, np.ceil(vx1 * fs + t0) + 1))
            if i1 - i0 < 16:
                return
            idx = np.arange(i0, i1)
            t = (idx - t0) / fs
            span = i1 - i0
            maxlag = max(1, span // 2)
            # referencia remuestreada en la grilla actual, en una ventana extendida ±maxlag
            ext = np.arange(i0 - maxlag, i1 + maxlag)
            te = (ext - t0) / fs
            ri = np.rint(te * rfs + rt0).astype(np.int64)
            valid = (ri >= 0) & (ri < rd.shape[1])
            ric = np.clip(ri, 0, rd.shape[1] - 1)
            N = 1 << int(np.ceil(np.log2(len(ext) + span)))
            acc = np.zeros(N)
            for ch in range(8):
                a = d[ch, i0:i1].astype(float) * 2 - 1
                b = np.where(valid, rd[ch][ric].astype(float) * 2 - 1, 0.0)
                if a.std() == 0 and b.std() == 0:
                    continue
                acc += np.fft.irfft(np.conj(np.fft.rfft(a, N)) * np.fft.rfft(b, N), N)
            # acc[m] = sum a[n] * b[n + m]; la referencia en b[n + m] coincide con a[n]
            # -> hay que correr la referencia m - maxlag muestras hacia atrás
            lags = np.arange(0, 2 * maxlag + 1)
            m = int(lags[np.argmax(acc[lags])])
            shift_samples = -(m - maxlag)
            self.ov_shift.setValue(shift_samples * 1e9 / fs)
            self.statusBar().showMessage(self.statusBar().currentMessage() +
                                         f"  · alineada automáticamente ({shift_samples:+d} muestras)")

        def _clear_overlay(self):
            self.ref = None
            self.tb_ov.setVisible(False)
            for i in range(8):
                self.ref_curves[i].setData([], [])
                self.diff_curves[i].setData([], [])
            self.statusBar().showMessage("Superposición quitada")

        def _draw_overlay(self):
            if self.ref is None:
                return
            rd, rfs, rt0, name = self.ref
            sh = self.ov_shift.value() * 1e-9                    # desplazamiento en s
            dy = -0.45 if self.chk_ov_sep.isChecked() else 0.0
            n_r = rd.shape[1]
            # Referencia alineada por el trigger (t = 0), con su propia fs
            for i in range(8):
                v = rd[i]
                tr = np.flatnonzero(v[1:] != v[:-1]) + 1
                starts = np.concatenate(([0], tr))
                y = v[starts].astype(float) * HI + ybase(i) + dy
                x = (np.concatenate((starts, [n_r])).astype(float) - 0.5 - rt0) / rfs + sh
                self.ref_curves[i].setData(x, y, stepMode="center")
            # Diferencias evaluadas en la grilla de la captura actual
            d, fs, t0 = self.data, self.fs, self.t0
            n = d.shape[1]
            t = (np.arange(n) - t0) / fs
            ri = np.rint((t - sh) * rfs + rt0).astype(np.int64)
            valid = (ri >= 0) & (ri < n_r)
            ric = np.clip(ri, 0, n_r - 1)
            report = []
            for i in range(8):
                diff = valid & (d[i] != rd[i][ric])
                cnt = int(diff.sum())
                if cnt:
                    edges = np.flatnonzero(np.diff(np.concatenate(([0], diff.view(np.int8), [0]))))
                    a, b = edges[0::2], edges[1::2]
                    xs = np.empty(a.size * 3); ys = np.empty(a.size * 3)
                    xs[0::3] = (a - 0.5 - t0) / fs
                    xs[1::3] = (b - 0.5 - t0) / fs
                    xs[2::3] = np.nan
                    ys[:] = ybase(i) - (0.65 if self.chk_ov_sep.isChecked() else 0.35)
                    ys[2::3] = np.nan
                    self.diff_curves[i].setData(xs, ys, connect="finite")
                    report.append(f"{self.ch_labels[i].split()[0]}: {cnt}")
                else:
                    self.diff_curves[i].setData([], [])
            overlap = int(valid.sum())
            msg = f"Superpuesta: {name} (trigger {'+' if sh >= 0 else ''}{pg.siFormat(sh, suffix='s')}). "
            msg += ("Muestras distintas — " + ", ".join(report)) if report else "Sin diferencias en el tramo común."
            msg += f"  [tramo común: {overlap} muestras]"
            if abs(rfs - fs) > 1e-6 * fs:
                msg += f"  · fs distintas ({pg.siFormat(rfs, suffix='S/s')} vs {pg.siFormat(fs, suffix='S/s')})"
            self.statusBar().showMessage(msg)

        def _export_vcd(self):
            if self.data is None:
                return
            fn, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Exportar VCD", "captura.vcd", "*.vcd")
            if fn:
                export_vcd(fn, self.data, self.fs, self.ch_labels)
                self.statusBar().showMessage(f"VCD exportado: {fn}")

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setStyleSheet(QSS)
    w = MainWindow()
    WINDOWS.append(w)
    w.show()
    if "pytest" not in sys.modules:
        app.exec()
    return w


def export_vcd(fn, d, fs, names=None):
    """VCD con timescale 1 ps (GTKWave / PulseView)."""
    ids = [chr(33 + i) for i in range(8)]
    ps = 1e12 / fs
    packed = np.packbits(d[::-1].T, axis=1).ravel()      # 1 byte por muestra
    chg = np.flatnonzero(packed[1:] != packed[:-1]) + 1
    with open(fn, "w") as f:
        f.write("$timescale 1ps $end\n$scope module la8 $end\n")
        for i in range(8):
            nm = (names[i] if names else f"CH{i}").replace(" ", "_").replace("/", "_")
            f.write(f"$var wire 1 {ids[i]} {nm} $end\n")
        f.write("$upscope $end\n$enddefinitions $end\n#0\n$dumpvars\n")
        for i in range(8):
            f.write(f"{d[i, 0]}{ids[i]}\n")
        f.write("$end\n")
        prev = d[:, 0].copy()
        for k in chg:
            f.write(f"#{int(round(k * ps))}\n")
            cur = d[:, k]
            for i in np.flatnonzero(cur != prev):
                f.write(f"{cur[i]}{ids[i]}\n")
            prev = cur
        f.write(f"#{int(round(d.shape[1] * ps))}\n")


if __name__ == "__main__":
    main()
