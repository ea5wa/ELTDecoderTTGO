// Configuracion de hardware y parametros - LilyGo T-Beam v1.2 (433 MHz)
#pragma once

// I2C compartido: PMU AXP2101 (0x34) + OLED SSD1306 (0x3C)
#define PIN_I2C_SDA    21
#define PIN_I2C_SCL    22

// Boton de usuario (activo a nivel bajo, con pull-up externo)
#define PIN_BUTTON     38

// SX1278 (version 433 MHz del T-Beam)
#define PIN_LORA_SCK    5
#define PIN_LORA_MISO  19
#define PIN_LORA_MOSI  27
#define PIN_LORA_CS    18
#define PIN_LORA_RST   23
#define PIN_LORA_DIO0  26
#define PIN_LORA_DIO1  33
#define PIN_LORA_DIO2  32   // salida DATA del discriminador en modo continuo

// Canales COSPAS-SARSAT en uso (MHz). El boton conmuta entre ellos.
static const float CHANNELS[] = {406.025f, 406.028f, 406.031f, 406.037f, 406.040f};
static const int NUM_CHANNELS = sizeof(CHANNELS) / sizeof(CHANNELS[0]);

// Ancho de banda de recepcion FSK (kHz). 25 kHz tolera el error de cristal
// del SX1278 (sin TCXO puede haber varios kHz de offset a 406 MHz). Si se
// calibra el offset se puede bajar a 12.5 para ganar sensibilidad.
#define RXBW_KHZ                25.0f

// Correccion de frecuencia del cristal (Hz). Calibrar con una portadora
// conocida y ajustar por puerto serie con el comando "o <Hz>".
#define DEFAULT_FREQ_OFFSET_HZ  0

// Squelch: la captura de flancos solo se activa cuando el RSSI supera este
// umbral (la baliza emite 160 ms de portadora antes de los datos, de sobra
// para abrir la captura). Ajustable por serie con "s <dBm>".
#define DEFAULT_SQUELCH_DBM     -100.0f

// Tiempo (ms) con RSSI por debajo del umbral antes de cerrar la captura.
#define CAPTURE_HANG_MS         300

// Correccion automatica de frecuencia: al abrir el squelch (la baliza emite
// ~160 ms de portadora pura antes de los datos) se mide el error de
// frecuencia (FEI) y se re-sintoniza al vuelo. Absorbe el error del cristal
// propio y el de la baliza dentro del ancho de banda del filtro.
#define FEI_ENABLED             1
// Correccion minima (Hz) para molestarse en re-sintonizar.
#define FEI_MIN_TRIM_HZ         400
// Sentido de la correccion. FEI > 0 = senal por encima del LO -> subir
// sintonia (+1). Si en el banco la correccion empeora, poner -1.
#define FEI_SIGN                (+1)
