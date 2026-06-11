# ELT Decoder TTGO

Decodificador de balizas de emergencia **COSPAS-SARSAT 406 MHz** (ELT, PLB,
EPIRB) para **LilyGo T-Beam v1.2**. Recibe las ráfagas en 406,025 MHz (y
canales vecinos), decodifica la trama, verifica los códigos BCH y muestra en
la pantalla OLED el **identificador de baliza de 15 caracteres hexadecimales**
(bits 26–85 del mensaje), junto al código de país (MID) y el estado de la
verificación.

> ⚠️ **Solo recepción.** Este firmware no transmite nunca. Recuerda que una
> baliza real solo debe activarse en una emergencia; para pruebas usa el modo
> *self-test* de la baliza (el firmware lo detecta y lo marca como `TEST`) o
> un generador de señal sobre carga ficticia.

## Hardware necesario

| Componente | Detalle |
|---|---|
| LilyGo T-Beam **v1.2** | **Imprescindible la versión 433 MHz (SX1278)**. Las versiones 868/915 MHz tienen la etapa RF adaptada a esa banda y son prácticamente sordas en 406 MHz. PMU AXP2101. |
| OLED SSD1306 0,96" I2C | En el conector OLED del T-Beam (SDA=21, SCL=22, dir. 0x3C) |
| Antena | La antena de 433 MHz funciona aceptablemente en 406 MHz; ideal: monopolo cortado a ~17,6 cm |

## Cómo funciona

Las balizas 406 MHz transmiten ráfagas de ~520 ms cada 50 s: 160 ms de
portadora pura y después un mensaje de 112/144 bits a **400 bps en PSK
±1,1 rad con codificación bifase-L** (Manchester).

El SX1278 no demodula PSK, pero sí se puede usar como **discriminador FM**:

1. Se configura en modo **FSK continuo** a 406,025 MHz con el sincronizador
   de bits desactivado. La salida cruda del demodulador aparece en **DIO2**.
2. Cada transición de fase de la señal PSK es un impulso de frecuencia
   instantánea, que el discriminador convierte en un flanco en DIO2.
3. Una ISR del ESP32 marca el tiempo de cada flanco. En bifase-L los
   intervalos entre transiciones solo pueden ser de **1,25 ms** (medio bit) o
   **2,5 ms** (bit completo): un intervalo completo = el bit cambia; dos
   medios = el bit se repite. Así se reconstruye el flujo de bits sin
   necesitar la polaridad de la señal.
4. Se busca el sincronismo de trama (`000101111` normal, `011010000`
   autotest, en polaridad normal e invertida), se recogen los bits 25–144,
   se verifican **BCH-1** (g₁ = 1001101101100111100011) y **BCH-2**
   (g₂ = 1010100111001) y se extrae el ID de 15 hex.

Para no procesar ruido continuamente hay un **squelch por RSSI**: la captura
de flancos solo se activa cuando el RSSI supera el umbral (los 160 ms de
portadora dan tiempo de sobra a engancharse).

## Compilar y grabar

Con [PlatformIO](https://platformio.org/):

```bash
pio run -t upload          # compilar y grabar
pio device monitor         # consola serie a 115200
pio test -e native         # tests unitarios del decodificador (en el PC)
```

## Uso

- **Pantalla**: frecuencia y RSSI arriba; al recibir una trama, el ID de
  15 hex en grande, MID, formato (LARGA/CORTA), `TEST` si es autotest,
  resultado BCH, contador y antigüedad de la última recepción.
- **Botón de usuario** (IO38, el central): pulsación larga (2 s) → abre el
  **menú** en pantalla (`Apagar` / `Borrar stats` / `Cambiar frecuencia` /
  `Salir`). La acción larga se dispara al cumplirse los 2 s, sin necesidad
  de soltar el botón. Dentro del menú: corta = mover el cursor, larga =
  ejecutar, 8 s sin tocar = se cierra solo. `Cambiar frecuencia` abre la lista de
  canales (406,025 / 406,028 / 406,031 / 406,037 / 406,040 MHz; `*` marca el
  activo): corta para moverse, larga para sintonizar.
- **Apagado/encendido**: `Apagar` corta toda la alimentación a través del
  AXP2101; para **encender** se pulsa el botón **PWR** de la placa (el
  conectado a la PMU). Si no se detecta PMU, en su lugar se entra en sueño
  profundo y se despierta con el propio botón IO38.
- **Consola serie** (115200): vuelca cada trama completa en hexadecimal y
  acepta comandos:
  - `f 406.028` — cambiar de canal
  - `s -105` — umbral de squelch en dBm
  - `o 1500` — corrección del cristal en Hz (ver calibración)
  - `d [0-2]` — diagnóstico de ráfaga: `1` resumen + histograma de
    intervalos, `2` además volcado crudo de intervalos
  - `?` — estado

## Corrección automática de frecuencia (FEI)

Al abrirse el squelch, el firmware aprovecha los ~160 ms de portadora pura
que la baliza emite antes de los datos para leer el registro de error de
frecuencia (FEI) del SX1278 y **re-sintonizarse al vuelo**. Esto absorbe el
error del cristal propio y el de la baliza, y permite reducir `RXBW_KHZ`
para ganar sensibilidad. La corrección se aplica solo durante la ráfaga; al
terminar se vuelve a la frecuencia base del canal. Configuración en
[src/config.h](src/config.h) (`FEI_ENABLED`, `FEI_MIN_TRIM_HZ`, `FEI_SIGN` —
si en el banco la corrección empeora la recepción, invertir el signo).

## Diagnóstico de ráfagas (validación en banco)

El comportamiento del discriminador FM del SX1278 ante una señal PSK es la
principal incógnita del diseño. El comando `d 1` activa, al final de cada
ráfaga, un resumen con el número de flancos, el FEI medido y un **histograma
de los intervalos entre flancos**:

- Una señal decodificable se ve **bimodal**: picos en ~1,25 ms (medio bit) y
  ~2,5 ms (bit completo).
- Un continuo de intervalos cortos (<0,85 ms) delata *chatter* del data
  slicer (el discriminador reposa sobre el umbral entre transiciones). Si
  aparece, probar: offset deliberado de sintonía, otro `RXBW_KHZ`, o abrir
  un issue para el decodificador por ajuste a rejilla.
- `d 2` vuelca además los intervalos crudos (µs, separados por comas) para
  analizarlos en el PC.

## Calibración (importante)

El SX1278 del T-Beam usa cristal sin TCXO: a 406 MHz el error puede ser de
varios kHz. El ancho de banda por defecto (25 kHz) lo tolera, pero la
decodificación mejora si se corrige:

1. Inyecta o radia una portadora de frecuencia conocida cerca de 406 MHz.
2. Ajusta `o <Hz>` por puerto serie hasta maximizar el RSSI con squelch justo.
3. Fija el valor en `DEFAULT_FREQ_OFFSET_HZ` ([src/config.h](src/config.h)) y,
   si quieres más sensibilidad, baja `RXBW_KHZ` a 12.5.

## Limitaciones

- La demodulación PSK vía discriminador FM es menos sensible que un
  demodulador coherente (SDR). Funciona bien con señal fuerte (baliza en
  self-test a metros o pocos km); no es un receptor de búsqueda de largo
  alcance.
- Los códigos BCH solo se usan para detectar errores, no para corregirlos.
  Una trama con `BCH1 ERR` puede tener el ID corrupto.
- El campo de posición codificada (PDF-2, protocolos de localización) no se
  decodifica todavía; el mensaje completo se vuelca en hex por serie.

## Estructura del proyecto

```
platformio.ini
src/
  main.cpp          firmware: radio, squelch, OLED, botón, consola serie
  config.h          pines del T-Beam v1.2 y parámetros ajustables
lib/decoder406/
  decoder406.h/.cpp decodificador puro (bifase-L, sync, BCH, ID 15-hex)
test/test_decoder/
  test_main.cpp     tests con tramas sintéticas (pio test -e native)
```
