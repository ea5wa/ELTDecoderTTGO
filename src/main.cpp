// ELT Decoder TTGO
// Decodificador de balizas de emergencia COSPAS-SARSAT 406 MHz (ELT/PLB/EPIRB)
// para LilyGo T-Beam v1.2 (version 433 MHz: SX1278 + AXP2101) con OLED SSD1306.
//
// Principio de funcionamiento: la baliza emite PSK +/-1,1 rad a 400 bps
// (bifase-L). El SX1278 no demodula PSK, pero en modo FSK continuo su
// discriminador FM convierte cada transicion de fase en un impulso en DIO2.
// El ESP32 mide el tiempo entre impulsos (1,25 / 2,5 ms) y reconstruye los
// bits, busca el sincronismo de trama, verifica los codigos BCH y extrae el
// identificador de 15 caracteres hexadecimales (bits 26-85).
//
// SOLO RECEPCION. Este equipo no transmite nunca.

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <esp_sleep.h>
#include <RadioLib.h>
#include <U8g2lib.h>

// PMU con autodeteccion: AXP2101 (T-Beam v1.2) o AXP192 (T-Beam v1.0/v1.1)
#include <XPowersLibInterface.hpp>
#include <XPowersAXP2101.tpp>
#include <XPowersAXP192.tpp>

#include "config.h"
#include "decoder406.h"

// ---------------------------------------------------------------- globales
static Module*  loraMod = nullptr;
static SX1278*  radio   = nullptr;
static XPowersLibInterface* pmu = nullptr;
static const char* pmuName = "ninguna";
static U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE,
                                                PIN_I2C_SCL, PIN_I2C_SDA);
static elt406::Decoder decoder;

// Buffer circular de flancos escrito desde la ISR de DIO2
static constexpr uint32_t EDGE_BUF = 2048;
static volatile uint32_t edgeTime[EDGE_BUF];
static volatile uint32_t edgeHead = 0;
static uint32_t edgeTail = 0;

static int     chanIdx      = 0;
static float   squelchDbm   = DEFAULT_SQUELCH_DBM;
static int32_t freqOffsetHz = DEFAULT_FREQ_OFFSET_HZ;

static bool     capturing      = false;
static uint32_t captureStartMs = 0;
static uint32_t lastAboveMs    = 0;
static float    rssiNow        = -127.0f;
static float    rssiPeak       = -127.0f;

static elt406::Frame lastFrame;
static uint32_t frameCount    = 0;
static uint32_t lastFrameMs   = 0;
static float    lastFrameRssi = -127.0f;
// Trama recien detectada retenida en pantalla hasta confirmar con IO38
static bool     frameHold     = false;

static bool pmuOk = false;

// Menu en pantalla (se abre con pulsacion larga del boton IO38)
static bool     menuOpen     = false;
static uint8_t  menuLevel    = 0;   // 0 = principal, 1 = lista de canales
static uint8_t  menuSel      = 0;
static uint32_t menuTouchMs  = 0;
static const char* const MENU_ITEMS[] = {"Apagar", "Borrar stats",
                                         "Cambiar frecuencia", "Salir"};
static constexpr uint8_t MENU_N = 4;
static constexpr uint32_t MENU_TIMEOUT_MS = 8000;

// Correccion automatica de frecuencia (FEI) durante la portadora inicial
static int32_t  trimHz     = 0;   // correccion aplicada a la rafaga en curso
static int32_t  feiLastHz  = 0;   // ultima medida (para diagnostico)
static int32_t  feiAccum   = 0;
static uint8_t  feiSamples = 0;
static bool     feiDone    = true;
static uint32_t feiNextMs  = 0;

// Diagnostico de intervalos entre flancos
// 0 = apagado, 1 = resumen + histograma, 2 = ademas volcado crudo
static uint8_t  diagMode = 0;
static constexpr uint32_t HIST_BIN_US = 100;
static constexpr int      HIST_BINS   = 40;       // 0..4 ms
static uint32_t hist[HIST_BINS + 1];              // ultimo bin = desborde
static uint32_t cntDebounce, cntShort, cntHalf, cntFull, cntLong;
static constexpr int RAW_MAX = 4000;
static uint16_t rawDt[RAW_MAX];
static int      rawCount = 0;
static uint32_t burstEdges = 0;
static uint32_t lastRawEdgeUs = 0;
static bool     haveRawEdge = false;
static uint32_t syncAtStart = 0;

// ------------------------------------------------------------------- radio
void IRAM_ATTR onDio2Edge() {
  uint32_t h = edgeHead;
  edgeTime[h % EDGE_BUF] = micros();
  edgeHead = h + 1;
}

static float readRssiDbm() {
  // RegRssiValue (FSK) = 0x11; RSSI[dBm] = -valor/2
  return -0.5f * (float)loraMod->SPIreadRegister(0x11);
}

static int32_t readFeiHz() {
  // RegFeiMsb/Lsb (FSK) = 0x1D/0x1E, complemento a dos, paso 61,035 Hz
  int16_t raw = (int16_t)(((uint16_t)loraMod->SPIreadRegister(0x1D) << 8) |
                          loraMod->SPIreadRegister(0x1E));
  return (int32_t)((float)raw * 61.035f);
}

// Sintoniza canal + offset de cristal + correccion FEI de la rafaga en curso
static void applyFrequency() {
  radio->standby();
  radio->setFrequency(CHANNELS[chanIdx] +
                      (float)(freqOffsetHz + trimHz) / 1e6f);
  // Desactivar el sincronizador de bits (RegOokPeak[5] = BitSyncOn) para
  // obtener la salida cruda del discriminador en DIO2.
  loraMod->SPIsetRegValue(0x14, 0x00, 5, 5);
  radio->receiveDirect();
}

// ------------------------------------------------------------- diagnostico
static void diagReset() {
  memset(hist, 0, sizeof(hist));
  cntDebounce = cntShort = cntHalf = cntFull = cntLong = 0;
  rawCount = 0;
  burstEdges = 0;
  haveRawEdge = false;
}

static void noteEdge(uint32_t t) {
  ++burstEdges;
  if (haveRawEdge) {
    uint32_t dt = t - lastRawEdgeUs;
    uint32_t bin = dt / HIST_BIN_US;
    ++hist[bin > (uint32_t)HIST_BINS ? HIST_BINS : bin];
    if      (dt < 600)  ++cntDebounce;
    else if (dt < 850)  ++cntShort;
    else if (dt <= 1750) ++cntHalf;
    else if (dt <= 3300) ++cntFull;
    else                ++cntLong;
    if (diagMode >= 2 && rawCount < RAW_MAX)
      rawDt[rawCount++] = (uint16_t)(dt > 65535 ? 65535 : dt);
  }
  haveRawEdge = true;
  lastRawEdgeUs = t;
}

static void printDiag() {
  Serial.println("--- Diagnostico de rafaga ---");
  Serial.printf("Flancos: %lu  Duracion: %lu ms  RSSI pico: %.0f dBm  "
                "FEI: %+ld Hz  Trim: %+ld Hz  Syncs: %lu\r\n",
                (unsigned long)burstEdges,
                (unsigned long)(lastAboveMs - captureStartMs),
                rssiPeak, (long)feiLastHz, (long)trimHz,
                (unsigned long)(decoder.syncCount() - syncAtStart));
  Serial.printf("Intervalos: <0.6ms:%lu  0.6-0.85:%lu  medio(0.85-1.75):%lu  "
                "bit(1.75-3.3):%lu  >3.3ms:%lu\r\n",
                (unsigned long)cntDebounce, (unsigned long)cntShort,
                (unsigned long)cntHalf, (unsigned long)cntFull,
                (unsigned long)cntLong);
  // Histograma: una rafaga limpia es bimodal en 1,25 y 2,5 ms; un continuo
  // en intervalos cortos delata chatter del data slicer.
  uint32_t maxv = 1;
  for (int i = 0; i <= HIST_BINS; ++i)
    if (hist[i] > maxv) maxv = hist[i];
  for (int i = 0; i <= HIST_BINS; ++i) {
    if (!hist[i]) continue;
    char bar[33];
    int len = (int)((hist[i] * 32 + maxv - 1) / maxv);
    memset(bar, '#', len);
    bar[len] = '\0';
    if (i == HIST_BINS)
      Serial.printf(" >%4lu us |%-32s %lu\r\n",
                    (unsigned long)(HIST_BINS * HIST_BIN_US), bar,
                    (unsigned long)hist[i]);
    else
      Serial.printf("%5lu us  |%-32s %lu\r\n",
                    (unsigned long)(i * HIST_BIN_US), bar,
                    (unsigned long)hist[i]);
  }
  if (diagMode >= 2 && rawCount) {
    Serial.printf("Volcado crudo (%d intervalos, us):\r\n", rawCount);
    for (int i = 0; i < rawCount; ++i) {
      Serial.print(rawDt[i]);
      Serial.print((i % 16 == 15 || i == rawCount - 1) ? "\r\n" : ",");
    }
  }
  Serial.println("-----------------------------");
}

// ----------------------------------------------------------------- captura
static void handleFrame();

static void drainEdges() {
  while (edgeTail != edgeHead) {
    uint32_t t = edgeTime[edgeTail % EDGE_BUF];
    ++edgeTail;
    noteEdge(t);
    if (decoder.onEdge(t)) handleFrame();
  }
}

static void startCapture() {
  edgeTail = edgeHead;     // descartar flancos viejos
  decoder.onIdle();
  diagReset();
  rssiPeak = rssiNow;
  captureStartMs = millis();
  syncAtStart = decoder.syncCount();
  feiAccum = 0;
  feiSamples = 0;
  feiDone = !FEI_ENABLED;
  feiNextMs = captureStartMs + 20;   // dejar asentar el receptor
  attachInterrupt(digitalPinToInterrupt(PIN_LORA_DIO2), onDio2Edge, CHANGE);
  capturing = true;
}

static void stopCapture() {
  if (!capturing) return;
  drainEdges();
  detachInterrupt(digitalPinToInterrupt(PIN_LORA_DIO2));
  capturing = false;
  decoder.onIdle();
  if (diagMode) printDiag();
  if (trimHz != 0) {       // volver a la frecuencia base del canal
    trimHz = 0;
    applyFrequency();
  }
}

static void tuneTo(int idx) {
  stopCapture();
  chanIdx = idx;
  trimHz = 0;
  applyFrequency();
  Serial.printf("Sintonizado %.4f MHz (offset %+d Hz)\r\n",
                CHANNELS[chanIdx], (int)freqOffsetHz);
}

// La baliza emite ~160 ms de portadora pura antes de los datos: tiempo de
// sobra para medir el error de frecuencia (FEI) y re-sintonizar al vuelo.
// Absorbe tanto el error del cristal propio como el de la baliza.
static void pollFei(uint32_t now) {
  if (feiDone || !capturing || now < feiNextMs) return;
  if (now - captureStartMs > 120) {  // demasiado tarde: ya llegan datos
    feiDone = true;
    return;
  }
  int32_t f = readFeiHz();
  feiNextMs = now + 5;
  if (f > -30000 && f < 30000) {     // descartar lecturas absurdas
    feiAccum += f;
    if (++feiSamples >= 4) {
      feiDone = true;
      int32_t avg = feiAccum / feiSamples;
      feiLastHz = avg;
      if (avg >= FEI_MIN_TRIM_HZ || avg <= -FEI_MIN_TRIM_HZ) {
        // FEI > 0: la senal esta por encima del LO -> subir la sintonia.
        // Si en el banco la correccion empeora el RSSI, invertir FEI_SIGN.
        trimHz = FEI_SIGN * avg;
        applyFrequency();
        decoder.onIdle();
        edgeTail = edgeHead;
        haveRawEdge = false;
        Serial.printf("FEI %+ld Hz -> re-sintonizado\r\n", (long)avg);
      } else {
        Serial.printf("FEI %+ld Hz (sin correccion)\r\n", (long)avg);
      }
    }
  }
}

// ------------------------------------------------------------------ apagado
static void powerOff() {
  Serial.println("Apagando...");
  Serial.flush();
  stopCapture();
  radio->sleep();

  oled.clearBuffer();
  oled.setFont(u8g2_font_7x13B_tf);
  oled.drawStr(22, 30, "Apagando...");
  oled.setFont(u8g2_font_6x10_tf);
  oled.drawStr(6, 50, pmuOk ? "Encender: boton PWR" : "Encender: boton IO38");
  oled.sendBuffer();
  delay(1200);
  oled.setPowerSave(1);

  if (pmuOk) {
    // En el estado off la PMU sigue viva cargando la bateria si hay USB;
    // el LED azul queda gobernado por el cargador: solo luce mientras carga.
    pmu->setChargingLedMode(XPOWERS_CHG_LED_CTRL_CHG);
    pmu->clearIrqStatus();
    // La PMU corta todas las salidas; se vuelve a encender con el boton
    // PWR de la placa (PEK).
    pmu->shutdown();
  }
  // Sin PMU (o si shutdown fallara): sueno profundo con despertar por el
  // mismo boton de usuario. Esperar a que se suelte para no despertar
  // al instante si aun sigue pulsado.
  while (digitalRead(PIN_BUTTON) == LOW) delay(10);
  delay(50);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_BUTTON, 0);
  esp_deep_sleep_start();
}

// ----------------------------------------------------------------- pantalla
static void drawMenu() {
  char line[24];
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tf);
  oled.drawHLine(0, 11, 128);
  if (menuLevel == 0) {
    oled.drawStr(0, 8, "MENU  corta:mover");
    oled.drawStr(36, 64, "larga:elegir");
    for (uint8_t i = 0; i < MENU_N; ++i) {
      int y = 22 + i * 11;
      if (i == menuSel) oled.drawStr(4, y, ">");
      oled.drawStr(16, y, MENU_ITEMS[i]);
    }
  } else {
    // Submenu de canales: '>' cursor, '*' canal activo
    oled.drawStr(0, 8, "FRECUENCIA (MHz)");
    for (int i = 0; i < NUM_CHANNELS; ++i) {
      int y = 21 + i * 10;
      if (i == menuSel) oled.drawStr(4, y, ">");
      snprintf(line, sizeof line, "%.3f%s", CHANNELS[i],
               i == chanIdx ? " *" : "");
      oled.drawStr(28, y, line);
    }
  }
  oled.sendBuffer();
}

// Pantalla de deteccion: se mantiene hasta confirmar con el boton IO38
static void drawFrameHold() {
  char line[40];
  oled.clearBuffer();

  // Cabecera invertida para llamar la atencion
  oled.setFont(u8g2_font_6x10_tf);
  oled.drawBox(0, 0, 128, 11);
  oled.setDrawColor(0);
  oled.drawStr(13, 9, "BALIZA DETECTADA");
  oled.setDrawColor(1);

  oled.setFont(u8g2_font_7x13B_tf);
  oled.drawStr(5, 28, lastFrame.hexId);

  oled.setFont(u8g2_font_6x10_tf);
  snprintf(line, sizeof line, "MID %u  %s%s", lastFrame.countryCode,
           lastFrame.longFormat ? "LARGA" : "CORTA",
           lastFrame.selfTest ? " TEST" : "");
  oled.drawStr(0, 41, line);

  snprintf(line, sizeof line, "BCH1 %s%s  %.0fdBm",
           lastFrame.bch1Ok ? "OK" : "ERR",
           lastFrame.longFormat
               ? (lastFrame.bch2Ok ? " BCH2 OK" : " BCH2 ERR") : "",
           lastFrameRssi);
  oled.drawStr(0, 52, line);

  uint32_t age = (millis() - lastFrameMs) / 1000;
  snprintf(line, sizeof line, "hace %lus   IO38=seguir", (unsigned long)age);
  oled.drawStr(0, 64, line);
  oled.sendBuffer();
}

static void drawScreen() {
  char line[40];
  if (menuOpen) {
    drawMenu();
    return;
  }
  if (frameHold) {
    drawFrameHold();
    return;
  }
  oled.clearBuffer();

  // Encabezado fijo + linea horizontal
  oled.setFont(u8g2_font_6x10_tf);
  oled.drawStr(0, 8, "ELT DECODER  by CUCA");
  oled.drawHLine(0, 11, 128);

  // Frecuencia y RSSI bajo la linea
  snprintf(line, sizeof line, "%.3f MHz", CHANNELS[chanIdx]);
  oled.drawStr(0, 22, line);
  snprintf(line, sizeof line, "%4.0f dBm", rssiNow);
  oled.drawStr(74, 22, line);
  if (capturing) oled.drawBox(124, 14, 4, 8);

  // Estado habitual: a la espera de trama (la ultima detectada solo se
  // muestra en la pantalla de retencion, hasta confirmarla con IO38)
  oled.drawStr(10, 40, "Esperando baliza...");
  snprintf(line, sizeof line, "squelch %.0f dBm", squelchDbm);
  oled.drawStr(10, 52, line);
  if (frameCount > 0) {
    snprintf(line, sizeof line, "tramas recibidas: %lu",
             (unsigned long)frameCount);
    oled.drawStr(10, 64, line);
  }
  oled.sendBuffer();
}

// ------------------------------------------------------------------- tramas
static void printFrame(const elt406::Frame& f) {
  Serial.println();
  Serial.println("=== TRAMA 406 MHz RECIBIDA ===");
  Serial.printf("Canal: %.4f MHz   RSSI pico: %.1f dBm   Trim FEI: %+ld Hz\r\n",
                CHANNELS[chanIdx], rssiPeak, (long)trimHz);
  Serial.printf("Formato: %s   Modo: %s\r\n",
                f.longFormat ? "LARGO (144 bits)" : "CORTO (112 bits)",
                f.selfTest ? "AUTOTEST" : "NORMAL");
  Serial.printf("BCH1: %s", f.bch1Ok ? "OK" : "ERROR");
  if (f.longFormat) Serial.printf("   BCH2: %s", f.bch2Ok ? "OK" : "ERROR");
  Serial.println();
  Serial.printf("ID 15-HEX: %s\r\n", f.hexId);
  Serial.printf("Pais (MID): %u\r\n", f.countryCode);

  Serial.print("Bits 25-");
  Serial.print(f.numBits);
  Serial.print(" (hex): ");
  uint8_t v = 0;
  int n = 0;
  for (int i = 25; i <= f.numBits; ++i) {
    v = (v << 1) | f.bits[i];
    if (++n == 4) {
      Serial.print("0123456789ABCDEF"[v]);
      v = 0;
      n = 0;
    }
  }
  if (n) Serial.print("0123456789ABCDEF"[(uint8_t)(v << (4 - n))]);
  Serial.println();
  Serial.println("==============================");
}

static void handleFrame() {
  lastFrame = decoder.frame();
  ++frameCount;
  lastFrameMs = millis();
  lastFrameRssi = rssiPeak;
  frameHold = true;          // retener en pantalla hasta confirmar con IO38
  printFrame(lastFrame);
  drawScreen();
}

// ------------------------------------------------------------ puerto serie
static void serialHelp() {
  Serial.println("Comandos:");
  Serial.println("  f <MHz>  frecuencia (ej. f 406.028)");
  Serial.println("  s <dBm>  umbral de squelch (ej. s -105)");
  Serial.println("  o <Hz>   offset de cristal (ej. o 1500)");
  Serial.println("  d [0-2]  diagnostico: 0 off, 1 histograma, 2 +volcado");
  Serial.println("  ?        estado");
}

static void pollSerial() {
  static char buf[32];
  static size_t len = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      buf[len] = '\0';
      len = 0;
      if (buf[0] == '\0') continue;
      float arg = atof(buf + 1);
      switch (buf[0]) {
        case 'f': {
          int best = chanIdx;
          for (int i = 0; i < NUM_CHANNELS; ++i)
            if (fabsf(CHANNELS[i] - arg) < 0.0015f) best = i;
          tuneTo(best);
          break;
        }
        case 's':
          squelchDbm = arg;
          Serial.printf("Squelch = %.0f dBm\r\n", squelchDbm);
          break;
        case 'o':
          freqOffsetHz = (int32_t)arg;
          tuneTo(chanIdx);
          break;
        case 'd':
          diagMode = (buf[1] == '\0') ? (diagMode + 1) % 3 : ((uint8_t)arg) % 3;
          Serial.printf("Diagnostico = %u\r\n", diagMode);
          break;
        case '?':
          Serial.printf("Canal %.4f MHz, offset %+d Hz, squelch %.0f dBm, "
                        "RSSI %.1f dBm, FEI %+ld Hz, diag %u, tramas %lu\r\n",
                        CHANNELS[chanIdx], (int)freqOffsetHz, squelchDbm,
                        rssiNow, (long)feiLastHz, diagMode,
                        (unsigned long)frameCount);
          if (pmuOk)
            Serial.printf("PMU %s: USB %s, bateria %s (%d%%), %s\r\n",
                          pmuName,
                          pmu->isVbusIn() ? "conectado" : "no",
                          pmu->isBatteryConnect() ? "presente" : "ausente",
                          pmu->getBatteryPercent(),
                          pmu->isCharging() ? "cargando" : "sin carga");
          else
            Serial.println("PMU: NO DETECTADA");
          break;
        default:
          serialHelp();
      }
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    }
  }
}

// -------------------------------------------------------------------- boton
static void menuExecute() {
  if (menuLevel == 1) {        // submenu: sintonizar el canal elegido
    tuneTo(menuSel);
    menuOpen = false;
    drawScreen();
    return;
  }
  switch (menuSel) {
    case 0:                    // Apagar
      powerOff();
      break;                   // no retorna
    case 1:                    // Borrar stats
      frameCount = 0;
      frameHold = false;
      memset(&lastFrame, 0, sizeof(lastFrame));
      Serial.println("Estadisticas borradas");
      break;
    case 2:                    // Cambiar frecuencia: abrir submenu
      menuLevel = 1;
      menuSel = (uint8_t)chanIdx;
      menuTouchMs = millis();
      drawScreen();
      return;
  }
  menuOpen = false;
  drawScreen();
}

// Boton PWR de la placa (PEK del AXP2101): cualquier pulsacion = apagado
// limpio. El encendido desde off lo hace la propia PMU con ese mismo boton.
static void pollPmuButton() {
  static uint32_t tPmu = 0;
  uint32_t now = millis();
  if (!pmuOk || now - tPmu < 100) return;
  tPmu = now;
  uint64_t st = pmu->getIrqStatus();
  if (!st) return;
  bool off = pmu->isPekeyShortPressIrq() || pmu->isPekeyLongPressIrq();
  pmu->clearIrqStatus();
  Serial.printf("PMU IRQ 0x%08lX%s\r\n", (unsigned long)st,
                off ? " (boton PWR)" : "");
  // Ignorar la pulsacion con la que se acaba de encender el equipo
  if (off && now > 5000) powerOff();
}

static constexpr uint32_t LONG_PRESS_MS = 2000;

static void pollButton() {
  static bool pressed = false;
  static bool longFired = false;
  static uint32_t pressMs = 0;
  uint32_t ms = millis();
  bool now = digitalRead(PIN_BUTTON) == LOW;

  if (now && !pressed) {
    pressed = true;
    longFired = false;
    pressMs = ms;
  } else if (now && pressed && !longFired && ms - pressMs >= LONG_PRESS_MS) {
    // pulsacion larga: se dispara al cumplirse el tiempo, sin esperar a
    // que se suelte el boton
    longFired = true;
    if (!menuOpen) {
      menuOpen = true;
      menuLevel = 0;
      menuSel = 0;
      menuTouchMs = ms;
      drawScreen();
    } else {
      menuExecute();
    }
  } else if (!now && pressed) {
    pressed = false;
    if (longFired) return;     // la accion larga ya se ejecuto
    uint32_t held = ms - pressMs;
    if (held < 30) return;     // rebote
    if (menuOpen) {
      // pulsacion corta: mover el cursor del menu
      menuSel = (menuSel + 1) % (menuLevel ? NUM_CHANNELS : MENU_N);
      menuTouchMs = ms;
      drawScreen();
    } else if (frameHold) {
      // confirmar la baliza detectada y volver a la pantalla normal
      frameHold = false;
      drawScreen();
    }
  }
}

// -------------------------------------------------------------------- setup
static void fatal(const char* msg) {
  Serial.println(msg);
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tf);
  oled.drawStr(0, 20, "ERROR:");
  oled.drawStr(0, 34, msg);
  oled.sendBuffer();
  for (;;) delay(1000);
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\r\nELT Decoder TTGO - COSPAS-SARSAT 406 MHz");

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  // PMU con autodeteccion: AXP2101 (T-Beam v1.2) o AXP192 (v1.0/v1.1).
  pmu = new XPowersAXP2101(Wire, PIN_I2C_SDA, PIN_I2C_SCL);
  if (pmu->init()) {
    pmuName = "AXP2101";
  } else {
    delete pmu;
    pmu = new XPowersAXP192(Wire, PIN_I2C_SDA, PIN_I2C_SCL);
    if (pmu->init()) {
      pmuName = "AXP192";
    } else {
      delete pmu;
      pmu = nullptr;
    }
  }
  pmuOk = pmu != nullptr;
  if (pmuOk) {
    // Radio: ALDO2 en v1.2 (AXP2101), LDO2 en v1.0/v1.1 (AXP192).
    // GPS: ALDO3 / LDO3 -> apagado (no se usa y mete ruido).
    uint8_t chLora = pmu->isChannelAvailable(XPOWERS_ALDO2) ? XPOWERS_ALDO2
                                                            : XPOWERS_LDO2;
    uint8_t chGps  = pmu->isChannelAvailable(XPOWERS_ALDO3) ? XPOWERS_ALDO3
                                                            : XPOWERS_LDO3;
    pmu->setPowerChannelVoltage(chLora, 3300);
    pmu->enablePowerOutput(chLora);
    pmu->disablePowerOutput(chGps);
    // LED azul gobernado por el cargador: solo luce mientras carga por USB
    pmu->setChargingLedMode(XPOWERS_CHG_LED_CTRL_CHG);
    // Boton PWR (PEK): pulsacion corta para encender; el firmware atiende
    // las IRQ (corta/larga) para apagar (ver pollPmuButton). El apagado
    // forzado por hardware queda en 4 s por si el firmware se cuelga.
    pmu->setPowerKeyPressOnTime(XPOWERS_POWERON_128MS);
    pmu->setPowerKeyPressOffTime(XPOWERS_POWEROFF_4S);
    pmu->disableIRQ(XPOWERS_ALL_INT);
    pmu->clearIrqStatus();
    pmu->enableIRQ(XPOWERS_PWR_BTN_CLICK_INT | XPOWERS_PWR_BTN_LONGPRESSED_INT);
    Serial.printf("PMU %s OK (radio ON, GPS OFF, PWR=on/off)\r\n", pmuName);
  } else {
    Serial.println("AVISO: PMU no detectada (ni AXP2101 ni AXP192)");
  }
  delay(100);   // estabilizacion de la alimentacion de la radio

  oled.begin();
  oled.clearBuffer();
  oled.setFont(u8g2_font_7x13B_tf);
  oled.drawStr(25, 26, "ELT DECODER");
  oled.setFont(u8g2_font_6x10_tf);
  oled.drawStr(40, 42, "by CUCA");
  oled.drawStr(22, 56, pmuOk ? "COSPAS-SARSAT 406" : "PMU NO DETECTADA!");
  oled.sendBuffer();
  // Mantener el splash legible (el refresco periodico lo machaca enseguida)
  delay(pmuOk ? 1200 : 4000);

  SPI.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_CS);
  loraMod = new Module(PIN_LORA_CS, PIN_LORA_DIO0, PIN_LORA_RST, PIN_LORA_DIO1);
  radio = new SX1278(loraMod);

  int16_t st = radio->beginFSK(434.0f, 4.8f, 5.0f, RXBW_KHZ, 10, 16);
  if (st != RADIOLIB_ERR_NONE) {
    Serial.printf("beginFSK fallo: %d\r\n", st);
    fatal("radio SX1278 no responde");
  }

  pinMode(PIN_LORA_DIO2, INPUT);
  pinMode(PIN_BUTTON, INPUT);
  tuneTo(0);
  serialHelp();
}

// --------------------------------------------------------------------- loop
void loop() {
  uint32_t now = millis();

  // Squelch: muestrear RSSI cada 2 ms y abrir/cerrar la captura de flancos
  static uint32_t tRssi = 0;
  if (now - tRssi >= 2) {
    tRssi = now;
    rssiNow = readRssiDbm();
    if (rssiNow >= squelchDbm) {
      lastAboveMs = now;
      if (!capturing) {
        startCapture();
        Serial.printf("Portadora detectada (%.0f dBm), capturando...\r\n",
                      rssiNow);
      }
      if (rssiNow > rssiPeak) rssiPeak = rssiNow;
    } else if (capturing && now - lastAboveMs > CAPTURE_HANG_MS) {
      stopCapture();
      Serial.println("Fin de rafaga");
    }
  }

  pollFei(now);
  drainEdges();
  pollButton();
  pollPmuButton();
  pollSerial();

  if (menuOpen && now - menuTouchMs > MENU_TIMEOUT_MS) {
    menuOpen = false;          // cerrar el menu por inactividad
  }

  static uint32_t tDraw = 0;
  if (now - tDraw >= 250) {
    tDraw = now;
    drawScreen();
  }
}
