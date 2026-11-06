/*
  PUENTE ESP-NOW <-> LinuxCNC  (ESP32-S3)
  Arduino IDE

  Se conecta por USB a la PC con LinuxCNC y reenvia mensajes entre las
  perillas (ESP-NOW) y el puerto serial.

  Perilla -> PC (por serial, una linea por mensaje):
    FEED E5     pulsos del encoder
    FEED R      boton presionado
    FEED H      latido

  PC -> perillas (por serial):
    FEED V150   valor actual en % (el puente lo difunde por ESP-NOW)
    MAC         pide imprimir la MAC del puente

  Notas para la placa ESP32-S3:
    - Si usas el puerto USB nativo: Tools > USB CDC On Boot > Enabled.
    - Si usas el puerto UART (CH340/CP210x), dejalo como esta.
    - No imprimir nada mas por Serial: la PC interpreta cada linea.

  LED RGB de la placa:
    - ROJO  = LinuxCNC no conectado (la PC no manda lineas validas)
    - VERDE = LinuxCNC conectado (knobs_serial.py manda el valor cada 1 s)
    - Se considera desconectado si pasan TIMEOUT_PC_MS sin lineas de la PC.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

const uint8_t CANAL = 1;   // mismo canal que las perillas
const uint8_t BCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// LED RGB integrado (WS2812). Si no se ilumina, probar con 38 en vez de 48
// (algunas versiones de la placa DevKitC-1 usan GPIO38).
#ifdef RGB_BUILTIN
  #define PIN_RGB RGB_BUILTIN
#else
  #define PIN_RGB 48
#endif
const uint8_t BRILLO = 20;                 // 0-255 (255 es muy brillante)
const uint32_t TIMEOUT_PC_MS = 3000;       // sin lineas de la PC => desconectado

// Cola: callback ESP-NOW (tarea WiFi) -> loop
static const int COLA_N = 32;
static const int LINEA_MAX = 40;
static char cola[COLA_N][LINEA_MAX];
static volatile int colaIn = 0;
static volatile int colaOut = 0;

bool pcActiva = false;          // true mientras la PC siga mandando lineas
bool pcVista = false;           // true si la PC mando al menos una linea
uint32_t ultimoPc = 0;          // millis() de la ultima linea valida de la PC

void ponerRGB(bool conectado) {
  if (conectado) neopixelWrite(PIN_RGB, 0, BRILLO, 0);   // verde
  else           neopixelWrite(PIN_RGB, BRILLO, 0, 0);   // rojo
}

void actualizarEstadoPc() {
  bool activa = pcVista && (millis() - ultimoPc < TIMEOUT_PC_MS);
  if (activa != pcActiva) {
    pcActiva = activa;
    ponerRGB(pcActiva);
  }
}
uint32_t ultimoMac = 0;

void imprimirMac() {
  uint8_t m[6];
  WiFi.macAddress(m);
  Serial.printf(
    "MAC %02X:%02X:%02X:%02X:%02X:%02X  ->  {0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X}\n",
    m[0], m[1], m[2], m[3], m[4], m[5],
    m[0], m[1], m[2], m[3], m[4], m[5]
  );
}

// Paquete de una perilla: "KN|<ROL>|<msg>"  ->  linea "<ROL> <msg>"
void procesarPaquete(const uint8_t *data, int len) {
  char buf[LINEA_MAX];
  if (len <= 3 || len >= LINEA_MAX) return;
  memcpy(buf, data, len);
  buf[len] = 0;

  if (strncmp(buf, "KN|", 3) != 0) return;
  char *rol = buf + 3;
  char *sep = strchr(rol, '|');
  if (!sep) return;
  *sep = ' ';

  int next = (colaIn + 1) % COLA_N;
  if (next == colaOut) return;          // cola llena: se descarta
  strcpy(cola[colaIn], rol);
  colaIn = next;
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

// Linea de la PC: "<ROL> <msg>"  ->  difusion "KN|<ROL>|<msg>"
void procesarLineaPC(char *lin) {
  if (strcmp(lin, "MAC") == 0) {
    imprimirMac();
    return;
  }

  char *sp = strchr(lin, ' ');
  if (!sp) return;
  *sp = 0;

  char out[LINEA_MAX];
  int n = snprintf(out, sizeof(out), "KN|%s|%s", lin, sp + 1);
  if (n > 0 && n < (int)sizeof(out)) {
    esp_now_send(BCAST, (const uint8_t *)out, n);
    pcVista = true;
    ultimoPc = millis();
  }
}

void leerSerial() {
  static char lin[LINEA_MAX];
  static int n = 0;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      lin[n] = 0;
      if (n > 0) procesarLineaPC(lin);
      n = 0;
    } else if (c != '\r' && n < LINEA_MAX - 1) {
      lin[n++] = c;
    }
  }
}

void setup() {
  ponerRGB(false);                 // rojo hasta que LinuxCNC se conecte
  Serial.begin(115200);
  delay(1000);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(CANAL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR ESP-NOW");
    while (true) delay(1000);
  }

  esp_now_register_recv_cb(onRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BCAST, 6);
  peer.channel = CANAL;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  imprimirMac();
}

void loop() {
  // Mensajes de las perillas hacia la PC
  while (colaOut != colaIn) {
    Serial.println(cola[colaOut]);
    colaOut = (colaOut + 1) % COLA_N;
  }

  // Mensajes de la PC hacia las perillas
  leerSerial();

  // LED verde/rojo segun haya o no datos de LinuxCNC
  actualizarEstadoPc();

  // Mientras la PC no se conecte, repetir la MAC cada 5 s
  if (!pcActiva && millis() - ultimoMac > 5000) {
    ultimoMac = millis();
    imprimirMac();
  }
}
