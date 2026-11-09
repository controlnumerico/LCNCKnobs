/*
  KEYPAD ESP-NOW -> PUENTE -> LinuxCNC  (ESP32-S3)
  Arduino IDE

  Modulo de entradas: teclado matricial 4x4. Manda al puente el estado de
  las 16 teclas por ESP-NOW, con el mismo formato que las perillas:

    Modulo -> puente:  KN|KEYS|K<hhhh>   estado de las teclas, 16 bits en hex
                                         (bit 0 = tecla 00 ... bit 15 = tecla 15)
                       Se manda al instante cuando cambia una tecla y se
                       repite cada 1 s (sirve tambien como latido).

                       KN|KEYS|T<n>      ActiveTool (u32 en decimal); llega a
                       LinuxCNC como knobs.io.in-0. Se manda al cambiar y cada 1 s.

    Puente -> modulo:  KN|KEYS|O<a>,<b>  dos valores u32 en decimal (salidas).
                       LinuxCNC los manda al cambiar y cada 1 s.
                       Se reciben en la funcion recibirSalidas(a, b).

  Numeracion: tecla = fila * 4 + columna   (fila y columna desde 0)
    Fila 1:  00 01 02 03
    Fila 2:  04 05 06 07
    Fila 3:  08 09 10 11
    Fila 4:  12 13 14 15

  Conexion (los 8 pines del teclado, filas primero, de izquierda a derecha):
    Filas    1..4  -> GPIO 4, 5, 6, 7        (salidas)
    Columnas 1..4  -> GPIO 15, 16, 17, 18    (entradas con pull-up interno)
  Pantalla OLED I2C (SSD1306, 128x64):
    VCC -> 3.3V   GND -> GND   SDA -> GPIO 8   SCL -> GPIO 9
  Libreria: Adafruit SSD1306 (y Adafruit GFX).

  Salida 0-10 V (tarjeta PW1VA01, convierte PWM a voltaje):
    GPIO 10 -> PWM (signal+)     GND del ESP32 -> GND de la señal PWM
    Vin de la tarjeta -> fuente de 12 a 30 V (no del ESP32)
    GND de la fuente debe unirse al GND del ESP32
    La salida es el valor A: 0 .. A_MAX  ->  0 .. 100 % de PWM

  Si tu teclado resulta al reves (filas y columnas cambiadas), no pasa nada
  grave: solo cambia la numeracion. Se arregla con INVERTIR_EJES.

  Antes de cargarlo:
    - Copiar la MAC del puente en BRIDGE_MAC (la misma que usan tus perillas).
    - Mismo CANAL que el puente y las perillas.
    - Tools > USB CDC On Boot > Enabled si quieres ver mensajes de prueba
      por el monitor serial.

  Sin diodos, si se presionan 3 o mas teclas a la vez puede haber teclas
  fantasma. Con una tecla a la vez no hay problema.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---- Configuracion ---------------------------------------------------------
// MAC del puente (la misma BRIDGE_MAC de tus perillas). Verificala: el puente
// la imprime por serial al arrancar.
const uint8_t BRIDGE_MAC[6] = {0xE0, 0x72, 0xA1, 0xF3, 0xD4, 0x54};
const uint8_t CANAL = 1;                    // mismo canal en puente y perillas

const uint8_t PIN_FILAS[4] = {4, 5, 6, 7};
const uint8_t PIN_COLS[4]  = {15, 16, 17, 18};

const bool     INVERTIR_EJES = false;       // true si filas y columnas estan cambiadas
const uint32_t DEBOUNCE_MS   = 20;          // antirrebote
const uint32_t SCAN_MS       = 2;           // cada cuanto se barre el teclado
const uint32_t LATIDO_MS     = 1000;        // reenvio del estado aunque no cambie
const bool     DEBUG_SERIAL  = true;        // imprime las teclas por el monitor serial

// Si dejan de llegar las salidas de LinuxCNC (PC apagada, puente caido...),
// se llama a recibirSalidas(0, 0) para dejar todo en un estado seguro.
const bool     SALIDAS_A_CERO_SIN_DATOS = true;
const uint32_t TIMEOUT_SALIDAS_MS       = 3000;

// ---- Salida PWM (valor A -> 0-10 V con la tarjeta PW1VA01) ----------------
const uint8_t  PIN_PWM   = 10;              // señal PWM hacia la tarjeta
const uint32_t PWM_FREC  = 1000;            // Hz (revisa el rango que acepta tu tarjeta)
const uint8_t  PWM_BITS  = 10;              // resolucion: 0..1023
const uint32_t A_MAX     = 1000;            // A = 0 -> 0 V,  A = 1000 -> 10 V

// ---- Pantalla OLED ---------------------------------------------------------
const uint8_t  OLED_ANCHO = 128;
const uint8_t  OLED_ALTO  = 64;
const uint8_t  OLED_ADDR  = 0x3C;           // si no funciona, prueba 0x3D
const uint8_t  PIN_SDA    = 8;
const uint8_t  PIN_SCL    = 9;

Adafruit_SSD1306 display(OLED_ANCHO, OLED_ALTO, &Wire, -1);
bool oledOk = false;                        // false si no se encontro la pantalla

// ---- Estado ----------------------------------------------------------------
uint16_t estable    = 0;    // estado confirmado (ya sin rebotes)
uint16_t candidato  = 0;    // ultima lectura cruda
uint32_t tCandidato = 0;    // desde cuando se mantiene esa lectura
uint32_t tScan      = 0;
uint32_t tEnvio     = 0;

// Valores recibidos desde LinuxCNC (los escribe el callback ESP-NOW, que corre
// en la tarea WiFi; el loop los lee con la seccion critica de abajo)
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t rxA = 0, rxB = 0;
volatile bool     rxNuevo = false;
volatile uint32_t tRx = 0;
bool salidasVivas = false;   // true mientras sigan llegando datos

// Variable que se manda a LinuxCNC como entrada u32 (knobs.io.in-0)
uint32_t ActiveTool = 0;
uint32_t tEnvioTool = 0;

// Envia "KN|KEYS|T<ActiveTool>" al puente
void enviarActiveTool() {
  char buf[32];
  int n = snprintf(buf, sizeof(buf), "KN|KEYS|T%lu", (unsigned long)ActiveTool);
  esp_now_send(BRIDGE_MAC, (const uint8_t *)buf, n);
  tEnvioTool = millis();
}

// ===========================================================================
//  AQUI RECIBES LOS 2 VALORES u32 DE LinuxCNC
//  a = knobs.io.out-0     b = knobs.io.out-1
//  Se llama al cambiar el valor y cada ~1 s (aunque no cambie).
// ===========================================================================
void mostrarSalidas(uint32_t a, uint32_t b) {
  if (!oledOk) return;
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 8);
  display.print("A =   ");
  display.print((unsigned long)a);
  display.setCursor(0, 40);
  display.print("B =   ");
  display.print((unsigned long)b);
  display.display();
}

// Convierte A (0..A_MAX) a ciclo de trabajo PWM (0..100 %)
void aplicarPWM(uint32_t a) {
  const uint32_t maxDuty = (1UL << PWM_BITS) - 1;
  if (a > A_MAX) a = A_MAX;
  uint32_t duty = (uint32_t)(((uint64_t)a * maxDuty) / A_MAX);
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcWrite(PIN_PWM, duty);
#else
  ledcWrite(0, duty);                       // canal 0
#endif
}

void recibirSalidas(uint32_t a, uint32_t b) {
  // Solo redibuja la pantalla si cambio algun valor (evita parpadeo y
  // no gasta tiempo del loop en cada latido de 1 s)
  static bool primera = true;
  static uint32_t ultA = 0, ultB = 0;
  if (primera || a != ultA || b != ultB) {
    primera = false;
    ultA = a;
    ultB = b;
    mostrarSalidas(a, b);
  }
  aplicarPWM(a);               // A -> salida PWM -> 0-10 V
  if (DEBUG_SERIAL) {
    Serial.printf("Salidas: a=%lu  b=%lu\n", (unsigned long)a, (unsigned long)b);
  }
  ActiveTool = a + b;          // copia a + b y la manda a LinuxCNC (knobs.io.in-0)
  enviarActiveTool();
}

// Paquete del puente: "KN|KEYS|O<a>,<b>". Corre en la tarea WiFi: aqui solo
// se guardan los datos; recibirSalidas() se llama desde loop().
void procesarPaquete(const uint8_t *data, int len) {
  char buf[40];
  if (len <= 9 || len >= (int)sizeof(buf)) return;
  memcpy(buf, data, len);
  buf[len] = 0;

  if (strncmp(buf, "KN|KEYS|O", 9) != 0) return;   // otro rol u otro mensaje
  char *fin;
  uint32_t a = strtoul(buf + 9, &fin, 10);
  if (*fin != ',') return;
  uint32_t b = strtoul(fin + 1, &fin, 10);

  portENTER_CRITICAL(&mux);
  rxA = a;
  rxB = b;
  tRx = millis();
  rxNuevo = true;
  portEXIT_CRITICAL(&mux);
}

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  procesarPaquete(data, len);
}
#else
void onRecv(const uint8_t *mac, const uint8_t *data, int len) {
  procesarPaquete(data, len);
}
#endif

// Barre el teclado: pone una fila en LOW a la vez y lee las columnas.
// Las filas inactivas quedan en alta impedancia (INPUT) para que dos filas
// nunca queden en corto a traves de teclas presionadas.
uint16_t escanear() {
  uint16_t mascara = 0;
  for (uint8_t f = 0; f < 4; f++) {
    pinMode(PIN_FILAS[f], OUTPUT);
    digitalWrite(PIN_FILAS[f], LOW);
    delayMicroseconds(10);                  // deja estabilizar la linea
    for (uint8_t c = 0; c < 4; c++) {
      if (digitalRead(PIN_COLS[c]) == LOW) {
        uint8_t tecla = INVERTIR_EJES ? (c * 4 + f) : (f * 4 + c);
        mascara |= (uint16_t)1 << tecla;
      }
    }
    pinMode(PIN_FILAS[f], INPUT);
  }
  return mascara;
}

// Envia "KN|KEYS|K<hhhh>" al puente
void enviarEstado() {
  char buf[24];
  int n = snprintf(buf, sizeof(buf), "KN|KEYS|K%04X", estable);
  esp_now_send(BRIDGE_MAC, (const uint8_t *)buf, n);
  tEnvio = millis();
}

void setup() {
  if (DEBUG_SERIAL) Serial.begin(115200);

  for (uint8_t f = 0; f < 4; f++) pinMode(PIN_FILAS[f], INPUT);
  for (uint8_t c = 0; c < 4; c++) pinMode(PIN_COLS[c], INPUT_PULLUP);

  // Pantalla OLED (si no responde, el teclado sigue funcionando igual)
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  oledOk = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (DEBUG_SERIAL && !oledOk) Serial.println("OLED no encontrada");
  mostrarSalidas(0, 0);

  // Salida PWM (empieza en 0 V)
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcAttach(PIN_PWM, PWM_FREC, PWM_BITS);
#else
  ledcSetup(0, PWM_FREC, PWM_BITS);
  ledcAttachPin(PIN_PWM, 0);
#endif
  aplicarPWM(0);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(CANAL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    if (DEBUG_SERIAL) Serial.println("ERROR ESP-NOW");
    while (true) delay(1000);
  }

  esp_now_register_recv_cb(onRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BRIDGE_MAC, 6);
  peer.channel = CANAL;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  if (DEBUG_SERIAL) {
    Serial.print("KEYPAD listo. MAC de este modulo: ");
    Serial.println(WiFi.macAddress());
  }
  enviarEstado();
  enviarActiveTool();
}

void loop() {
  uint32_t ahora = millis();

  // Barrido del teclado con antirrebote
  if (ahora - tScan >= SCAN_MS) {
    tScan = ahora;
    uint16_t lectura = escanear();

    if (lectura != candidato) {
      candidato = lectura;
      tCandidato = ahora;                   // cambio: reinicia el conteo
    } else if (candidato != estable && ahora - tCandidato >= DEBOUNCE_MS) {
      estable = candidato;                  // lectura estable: se confirma
      enviarEstado();                       // aviso inmediato al puente
      if (DEBUG_SERIAL) {
        Serial.print("Teclas: 0x");
        Serial.println(estable, HEX);
      }
    }
  }

  // Salidas recibidas de LinuxCNC -> funcion del usuario
  if (rxNuevo) {
    uint32_t a, b;
    portENTER_CRITICAL(&mux);
    a = rxA;
    b = rxB;
    rxNuevo = false;
    portEXIT_CRITICAL(&mux);
    salidasVivas = true;
    recibirSalidas(a, b);
  } else if (SALIDAS_A_CERO_SIN_DATOS && salidasVivas &&
             ahora - tRx > TIMEOUT_SALIDAS_MS) {
    salidasVivas = false;
    recibirSalidas(0, 0);      // sin datos de LinuxCNC: estado seguro
  }

  // Latido: se repite el estado para que LinuxCNC sepa que el modulo esta vivo
  // y para recuperarse de un paquete perdido
  if (ahora - tEnvio >= LATIDO_MS) {
    enviarEstado();
  }
  if (ahora - tEnvioTool >= LATIDO_MS) {
    enviarActiveTool();
  }
}
