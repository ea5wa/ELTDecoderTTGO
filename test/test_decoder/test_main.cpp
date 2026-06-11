// Tests del decodificador 406 MHz (entorno nativo: pio test -e native)
//
// Se construye una trama COSPAS-SARSAT sintetica (con paridad BCH valida),
// se convierte en la secuencia de flancos que produciria el discriminador
// FM del SX1278 y se comprueba que el decodificador recupera el ID 15-hex.

#include <unity.h>
#include <cstdint>
#include <cstring>
#include <vector>

#include "decoder406.h"

using elt406::Decoder;

static const char* HEX_ID = "9D0E2F1234ABC78";  // 60 bits, bits 26-85

// ----------------------------------------------------- construccion de trama
static uint32_t bchRemainder(const uint8_t* bits, int from, int to,
                             int zeros, uint32_t poly, int deg) {
  uint32_t rem = 0;
  for (int i = from; i <= to + zeros; ++i) {
    uint8_t b = (i <= to) ? bits[i] : 0;
    rem = (rem << 1) | b;
    if (rem & (1UL << deg)) rem ^= poly;
  }
  return rem;
}

// bits[1..144]; devuelve numero total de bits (112 o 144)
static int buildFrame(uint8_t* bits, bool longFormat, bool selfTest) {
  memset(bits, 0, 145);
  for (int i = 1; i <= 15; ++i) bits[i] = 1;             // preambulo
  const char* sync = selfTest ? "011010000" : "000101111";
  for (int i = 0; i < 9; ++i) bits[16 + i] = sync[i] - '0';
  bits[25] = longFormat ? 1 : 0;

  // bits 26-85 desde HEX_ID
  for (int n = 0; n < 15; ++n) {
    const char c = HEX_ID[n];
    const uint8_t v = (c <= '9') ? (c - '0') : (c - 'A' + 10);
    for (int k = 0; k < 4; ++k)
      bits[26 + n * 4 + k] = (v >> (3 - k)) & 1;
  }

  // BCH-1: paridad de bits 25-85 en bits 86-106
  uint32_t r = bchRemainder(bits, 25, 85, 21, 0x26D9E3, 21);
  for (int i = 0; i < 21; ++i) bits[86 + i] = (r >> (20 - i)) & 1;

  if (!longFormat) return 112;  // bits 107-112 quedan a 0 (no protegidos)

  // PDF-2 arbitrario (bits 107-132) y BCH-2 (133-144)
  for (int i = 107; i <= 132; ++i) bits[i] = (i * 7) & 1;
  r = bchRemainder(bits, 107, 132, 12, 0x1539, 12);
  for (int i = 0; i < 12; ++i) bits[133 + i] = (r >> (11 - i)) & 1;
  return 144;
}

// --------------------------------------------------- generacion de flancos
// Bifase-L: transicion en el centro de cada bit; transicion en la frontera
// solo cuando el bit siguiente es igual al actual.
struct EdgeOpts {
  int jitterUs = 0;       // jitter pseudoaleatorio +/- en cada flanco
  bool doubleEdges = false;  // cada impulso produce dos flancos a 150 us
  uint32_t startUs = 50000;
};

static uint32_t prngState = 12345;
static int prng(int span) {  // [-span, +span]
  prngState = prngState * 1103515245u + 12345u;
  return (int)((prngState >> 16) % (2 * span + 1)) - span;
}

static std::vector<uint32_t> makeEdges(const uint8_t* bits, int nBits,
                                       const EdgeOpts& o) {
  std::vector<uint32_t> e;
  const uint32_t BIT = 2500, HALF = 1250;
  for (int i = 1; i <= nBits; ++i) {
    uint32_t t0 = o.startUs + (uint32_t)(i - 1) * BIT;
    uint32_t mid = t0 + HALF + (o.jitterUs ? prng(o.jitterUs) : 0);
    e.push_back(mid);
    if (o.doubleEdges) e.push_back(mid + 150);
    if (i < nBits && bits[i + 1] == bits[i]) {
      uint32_t edge = t0 + BIT + (o.jitterUs ? prng(o.jitterUs) : 0);
      e.push_back(edge);
      if (o.doubleEdges) e.push_back(edge + 150);
    }
  }
  return e;
}

static bool feed(Decoder& d, const std::vector<uint32_t>& edges) {
  bool got = false;
  for (uint32_t t : edges)
    if (d.onEdge(t)) got = true;
  d.onIdle();
  return got;
}

// -------------------------------------------------------------------- tests
void setUp() {}
void tearDown() {}

static void test_long_frame_clean() {
  uint8_t bits[145];
  int n = buildFrame(bits, true, false);
  Decoder d;
  TEST_ASSERT_TRUE(feed(d, makeEdges(bits, n, {})));
  const auto& f = d.frame();
  TEST_ASSERT_TRUE(f.longFormat);
  TEST_ASSERT_FALSE(f.selfTest);
  TEST_ASSERT_TRUE(f.bch1Ok);
  TEST_ASSERT_TRUE(f.bch2Ok);
  TEST_ASSERT_EQUAL_STRING(HEX_ID, f.hexId);
}

static void test_short_frame() {
  uint8_t bits[145];
  int n = buildFrame(bits, false, false);
  Decoder d;
  TEST_ASSERT_TRUE(feed(d, makeEdges(bits, n, {})));
  const auto& f = d.frame();
  TEST_ASSERT_FALSE(f.longFormat);
  TEST_ASSERT_EQUAL_INT(112, f.numBits);
  TEST_ASSERT_TRUE(f.bch1Ok);
  TEST_ASSERT_EQUAL_STRING(HEX_ID, f.hexId);
}

static void test_self_test_frame() {
  uint8_t bits[145];
  int n = buildFrame(bits, true, true);
  Decoder d;
  TEST_ASSERT_TRUE(feed(d, makeEdges(bits, n, {})));
  TEST_ASSERT_TRUE(d.frame().selfTest);
  TEST_ASSERT_TRUE(d.frame().bch1Ok);
  TEST_ASSERT_EQUAL_STRING(HEX_ID, d.frame().hexId);
}

static void test_jitter_and_double_edges() {
  uint8_t bits[145];
  int n = buildFrame(bits, true, false);
  EdgeOpts o;
  o.jitterUs = 150;
  o.doubleEdges = true;
  Decoder d;
  TEST_ASSERT_TRUE(feed(d, makeEdges(bits, n, o)));
  TEST_ASSERT_TRUE(d.frame().bch1Ok);
  TEST_ASSERT_EQUAL_STRING(HEX_ID, d.frame().hexId);
}

static void test_noise_prefix() {
  uint8_t bits[145];
  int n = buildFrame(bits, true, false);
  auto edges = makeEdges(bits, n, {});
  // ruido pseudoaleatorio antes de la rafaga (flancos entre 0 y 40 ms)
  std::vector<uint32_t> all;
  uint32_t t = 0;
  prngState = 777;
  while (t < 40000) {
    t += 300 + (prng(1000) + 1000);
    all.push_back(t);
  }
  all.insert(all.end(), edges.begin(), edges.end());
  Decoder d;
  TEST_ASSERT_TRUE(feed(d, all));
  TEST_ASSERT_TRUE(d.frame().bch1Ok);
  TEST_ASSERT_EQUAL_STRING(HEX_ID, d.frame().hexId);
}

static void test_corrupted_bch_detected() {
  uint8_t bits[145];
  int n = buildFrame(bits, true, false);
  bits[40] ^= 1;  // corromper un bit del ID
  Decoder d;
  TEST_ASSERT_TRUE(feed(d, makeEdges(bits, n, {})));
  TEST_ASSERT_FALSE(d.frame().bch1Ok);
}

static void test_two_consecutive_frames() {
  uint8_t bits[145];
  int n = buildFrame(bits, true, false);
  Decoder d;
  TEST_ASSERT_TRUE(feed(d, makeEdges(bits, n, {})));
  EdgeOpts o;
  o.startUs = 2000000;
  TEST_ASSERT_TRUE(feed(d, makeEdges(bits, n, o)));
  TEST_ASSERT_EQUAL_STRING(HEX_ID, d.frame().hexId);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_long_frame_clean);
  RUN_TEST(test_short_frame);
  RUN_TEST(test_self_test_frame);
  RUN_TEST(test_jitter_and_double_edges);
  RUN_TEST(test_noise_prefix);
  RUN_TEST(test_corrupted_bch_detected);
  RUN_TEST(test_two_consecutive_frames);
  return UNITY_END();
}
