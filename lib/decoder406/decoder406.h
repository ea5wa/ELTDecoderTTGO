// Decodificador de tramas COSPAS-SARSAT 406 MHz (C/S T.001)
//
// Entrada: marcas de tiempo (en microsegundos) de los flancos que entrega el
// discriminador FM del SX1278 (pin DIO2 en modo FSK continuo). Cada transicion
// de fase de la portadora PSK produce un impulso; el tiempo entre impulsos es
// ~1,25 ms (medio bit) o ~2,5 ms (bit completo) a 400 bps bifase-L.
//
// Salida: trama de 112 (corta) o 144 bits (larga) con verificacion BCH y el
// identificador de baliza de 15 caracteres hexadecimales (bits 26-85).
//
// Codigo independiente de Arduino para poder testearlo en el entorno nativo.

#pragma once
#include <stdint.h>

namespace elt406 {

constexpr int MAX_BITS = 144;

struct Frame {
  // Bits de mensaje indexados como en la especificacion C/S T.001 (1..144).
  // Solo son validos los indices 25..numBits; los bits 1-24 (preambulo +
  // sincronismo de trama) no se almacenan.
  uint8_t bits[MAX_BITS + 1];
  int numBits;          // 112 (corta) o 144 (larga)
  bool longFormat;      // bit 25
  bool selfTest;        // sincronismo de trama de autotest (011010000)
  bool bch1Ok;          // BCH-1 sobre bits 25-106
  bool bch2Ok;          // BCH-2 sobre bits 107-144 (solo formato largo)
  char hexId[16];       // ID de baliza, 15 hex + terminador
  uint16_t countryCode; // MID, bits 27-36
};

class Decoder {
public:
  Decoder() { reset(); }

  // Borra todo el estado (incluida la ultima trama).
  void reset();

  // Procesa un flanco en t (microsegundos, puede desbordar: se usa resta
  // sin signo). Devuelve true cuando se ha completado una trama; en ese
  // caso frame() contiene el resultado hasta la siguiente trama.
  bool onEdge(uint32_t tUs);

  // Llamar cuando cesa la actividad (fin de rafaga): aborta capturas a medias.
  void onIdle();

  const Frame& frame() const { return frame_; }
  uint32_t syncCount() const { return syncCount_; }

private:
  enum class State : uint8_t { SEARCH, COLLECT };

  void resetBitSync();
  bool emitBit(uint8_t b);
  bool pushBit(uint8_t b);
  void finalize();

  // Ventanas de clasificacion de intervalos (µs). Nominal: 1250 / 2500.
  static constexpr uint32_t DEBOUNCE_US = 600;  // doble flanco del mismo impulso
  static constexpr uint32_t HALF_MIN_US = 850;
  static constexpr uint32_t HALF_MAX_US = 1750;
  static constexpr uint32_t FULL_MAX_US = 3300;

  // Patron de busqueda: 3 ultimos bits del preambulo (111) + 9 bits de
  // sincronismo de trama. Normal: 000101111. Autotest: 011010000.
  // Tambien se busca el patron invertido porque la decodificacion por
  // tiempos no conoce la polaridad absoluta de los bits.
  static constexpr uint32_t SYNC_MASK   = 0xFFF;
  static constexpr uint32_t SYNC_NORMAL = 0b111000101111;
  static constexpr uint32_t SYNC_TEST   = 0b111011010000;

  State state_;
  uint32_t lastEdgeUs_;
  bool haveEdge_;
  bool pendingHalf_;
  uint8_t lastBit_;
  bool invert_;
  uint32_t window_;
  uint8_t windowLen_;
  int bitIndex_;
  uint32_t syncCount_;
  Frame frame_;
};

} // namespace elt406
