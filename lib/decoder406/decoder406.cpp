#include "decoder406.h"
#include <string.h>

namespace elt406 {

// Polinomios generadores BCH de C/S T.001.
// BCH-1: g1(x) = x^21+x^18+x^17+x^15+x^14+x^12+x^11+x^8+x^7+x^6+x^5+x+1
//        (binario 1001101101100111100011), protege los bits 25-85 con
//        21 bits de paridad en 86-106.
// BCH-2: g2(x) = x^12+x^10+x^8+x^5+x^4+x^3+1 (binario 1010100111001),
//        protege los bits 107-132 con 12 bits de paridad en 133-144.
static constexpr uint32_t BCH1_POLY = 0x26D9E3;
static constexpr int      BCH1_DEG  = 21;
static constexpr uint32_t BCH2_POLY = 0x1539;
static constexpr int      BCH2_DEG  = 12;

// El bloque completo (datos + paridad) debe ser divisible por el generador.
static bool bchOk(const uint8_t* bits, int from, int to, uint32_t poly, int deg) {
  uint32_t rem = 0;
  for (int i = from; i <= to; ++i) {
    rem = (rem << 1) | bits[i];
    if (rem & (1UL << deg)) rem ^= poly;
  }
  return rem == 0;
}

void Decoder::reset() {
  state_ = State::SEARCH;
  haveEdge_ = false;
  lastEdgeUs_ = 0;
  invert_ = false;
  syncCount_ = 0;
  memset(&frame_, 0, sizeof(frame_));
  resetBitSync();
}

void Decoder::resetBitSync() {
  pendingHalf_ = false;
  lastBit_ = 0;
  window_ = 0;
  windowLen_ = 0;
  bitIndex_ = 0;
  state_ = State::SEARCH;
}

void Decoder::onIdle() {
  haveEdge_ = false;
  resetBitSync();
}

bool Decoder::onEdge(uint32_t tUs) {
  if (!haveEdge_) {
    haveEdge_ = true;
    lastEdgeUs_ = tUs;
    return false;
  }
  uint32_t dt = tUs - lastEdgeUs_;
  if (dt < DEBOUNCE_US) {
    // Segundo flanco del mismo impulso del discriminador: se ignora sin
    // mover la referencia temporal (el impulso "es" su primer flanco).
    return false;
  }
  lastEdgeUs_ = tUs;

  bool full;
  if (dt >= HALF_MIN_US && dt <= HALF_MAX_US) {
    full = false;
  } else if (dt > HALF_MAX_US && dt <= FULL_MAX_US) {
    full = true;
  } else {
    resetBitSync();
    return false;
  }

  // Decodificacion bifase-L por tiempos: un intervalo completo entre
  // transiciones centrales significa que el bit cambia; dos medios
  // intervalos (centro -> frontera -> centro) significan que se repite.
  if (full) {
    // Una media suelta antes de un intervalo completo solo ocurre si
    // empezamos a escuchar desalineados: se descarta y se realinea.
    pendingHalf_ = false;
    return emitBit(lastBit_ ^ 1);
  }
  if (pendingHalf_) {
    pendingHalf_ = false;
    return emitBit(lastBit_);
  }
  pendingHalf_ = true;
  return false;
}

bool Decoder::emitBit(uint8_t b) {
  lastBit_ = b;
  return pushBit(b);
}

bool Decoder::pushBit(uint8_t b) {
  if (state_ == State::SEARCH) {
    window_ = ((window_ << 1) | b) & SYNC_MASK;
    if (windowLen_ < 12) {
      ++windowLen_;
      if (windowLen_ < 12) return false;
    }
    uint32_t wi = (~window_) & SYNC_MASK;
    bool match = true;
    if      (window_ == SYNC_NORMAL) { invert_ = false; frame_.selfTest = false; }
    else if (window_ == SYNC_TEST)   { invert_ = false; frame_.selfTest = true;  }
    else if (wi == SYNC_NORMAL)      { invert_ = true;  frame_.selfTest = false; }
    else if (wi == SYNC_TEST)        { invert_ = true;  frame_.selfTest = true;  }
    else match = false;

    if (match) {
      ++syncCount_;
      state_ = State::COLLECT;
      bitIndex_ = 25;
    }
    return false;
  }

  // COLLECT: almacenar bits 25..numBits
  frame_.bits[bitIndex_++] = invert_ ? (b ^ 1) : b;
  if (bitIndex_ == 26) {
    frame_.longFormat = frame_.bits[25] != 0;
    frame_.numBits = frame_.longFormat ? 144 : 112;
  }
  if (bitIndex_ > frame_.numBits) {
    finalize();
    return true;
  }
  return false;
}

void Decoder::finalize() {
  Frame& f = frame_;
  f.bch1Ok = bchOk(f.bits, 25, 106, BCH1_POLY, BCH1_DEG);
  f.bch2Ok = f.longFormat ? bchOk(f.bits, 107, 144, BCH2_POLY, BCH2_DEG) : false;

  // Identificador de baliza: bits 26-85 (60 bits -> 15 caracteres hex)
  for (int n = 0; n < 15; ++n) {
    uint8_t v = 0;
    for (int k = 0; k < 4; ++k) {
      v = (v << 1) | f.bits[26 + n * 4 + k];
    }
    f.hexId[n] = "0123456789ABCDEF"[v];
  }
  f.hexId[15] = '\0';

  uint16_t cc = 0;
  for (int i = 27; i <= 36; ++i) cc = (cc << 1) | f.bits[i];
  f.countryCode = cc;

  // Volver a busqueda sin perder el sincronismo de bit.
  state_ = State::SEARCH;
  window_ = 0;
  windowLen_ = 0;
  pendingHalf_ = false;
  bitIndex_ = 0;
}

} // namespace elt406
