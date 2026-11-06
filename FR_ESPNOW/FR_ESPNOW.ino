/*
  ProgressBar circular para ESP32-C3 + pantalla GC9A01
  Arduino IDE
  Librerias:
    - Arduino_GFX_Library
    - lvgl 8.x
    - Encoder
    - Bounce2

  Pines conservados del programa original:
    Backlight: GPIO 8
    SPI DC:    GPIO 4
    SPI CS:    GPIO 10
    SPI SCK:   GPIO 1
    SPI MOSI:  GPIO 0
    Encoder A: GPIO 7
    Encoder B: GPIO 6
    Encoder SW: GPIO 9

  Nota: Configura LVGL con LV_COLOR_DEPTH = 16.

  VERSION ESP-NOW: el valor real vive en LinuxCNC. Un ESP32-S3 conectado
  por USB a la PC hace de puente.
    Perilla -> puente:  KN|<ROL>|E<n> (pulsos) | R (boton) | H (latido)
    Puente -> perilla:  KN|<ROL>|V<n> (valor actual en %)
*/

#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <lvgl.h>
#include <Encoder.h>
#include <Bounce2.h>
#include "esp_heap_caps.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// --------------------------------------------------
// ESP-NOW / LINUXCNC  (EDITAR)
// --------------------------------------------------

// MAC del puente ESP32-S3 (la imprime por serial al arrancar)

const uint8_t BRIDGE_MAC[6] = {0xE0, 0x72, 0xA1, 0xF3, 0xD4, 0x54};




const uint8_t CANAL = 1;                   // mismo canal en puente y perillas

// --------------------------------------------------
// TIPO DE PERILLA  (EDITAR: una sola linea distinta por cada ESP32-C3)
// --------------------------------------------------
#define PERILLA_FEED     1
#define PERILLA_SPINDLE  2
#define PERILLA_RAPID    3
#define PERILLA_VMAX     4
#define PERILLA_JOG      5
#define TIPO_PERILLA     PERILLA_FEED       // <-- PERILLA_FEED, SPINDLE, RAPID, VMAX o JOG

#if TIPO_PERILLA == PERILLA_SPINDLE
  const char *ROL = "SPINDLE";
  static const int32_t FEED_MAX = 200;     // tope del arco y del valor (%)
  static const int32_t FEED_NORMAL = 100;  // el LED parpadea si es distinto
  static const int PASO_CLIC = 5;          // % por clic del encoder
  static const char *TITULO = "SPINDLE\nSPEED";
  #define FORMATO "%d%%"
  #define COLOR_PRINCIPAL 0xFF8C00         // naranja
#elif TIPO_PERILLA == PERILLA_RAPID
  const char *ROL = "RAPID";
  static const int32_t FEED_MAX = 100;     // rapid override: 0 a 100 %
  static const int32_t FEED_NORMAL = 100;  // el LED parpadea si es distinto
  static const int PASO_CLIC = 5;          // % por clic del encoder
  static const char *TITULO = "RAPID\nSPEED";
  #define FORMATO "%d%%"
  #define COLOR_PRINCIPAL 0x00CCFF         // azul
#elif TIPO_PERILLA == PERILLA_VMAX
  const char *ROL = "VMAX";
  static const int32_t FEED_MAX = 3200;    // max-velocity: 0 a 3200 (unidades de maquina por minuto)
  static const int32_t FEED_NORMAL = 3200; // el LED parpadea si es distinto (velocidad por debajo del maximo)
  static const int PASO_CLIC = 160;        // unidades/min por clic del encoder
  static const char *TITULO = "MAX\nVEL";
  #define COLOR_PRINCIPAL 0xFF8C00         // naranja
  #define FORMATO "%d"                     // sin signo %
#elif TIPO_PERILLA == PERILLA_JOG
  const char *ROL = "JOG";
  static const int32_t FEED_MAX = 3200;    // jog speed: 0 a 3200 (unidades de maquina por minuto)
  static const int32_t FEED_NORMAL = 800;  // = INICIO_ABS["JOG"] en knobs_serial.py (el LED parpadea si es distinto)
  static const int PASO_CLIC = 160;        // unidades/min por clic del encoder
  static const char *TITULO = "JOG\nSPEED";
  #define COLOR_PRINCIPAL 0xFF8C00         // naranja
  #define FORMATO "%d"                     // sin signo %
#else
  const char *ROL = "FEED";
  static const int32_t FEED_MAX = 300;
  static const int32_t FEED_NORMAL = 100;
  static const int PASO_CLIC = 5;
  static const char *TITULO = "FEED\nRATE";
  #define FORMATO "%d%%"
  #define COLOR_PRINCIPAL 0xFF8C00         // naranja
#endif

// --------------------------------------------------
// PANTALLA GC9A01
// --------------------------------------------------

#define GFX_BL 8

Arduino_DataBus *bus = new Arduino_ESP32SPI(
  4,                  // DC
  10,                 // CS
  1,                  // SCK
  0,                  // MOSI
  GFX_NOT_DEFINED     // MISO
);

Arduino_GFX *gfx = new Arduino_GC9A01(
  bus,
  GFX_NOT_DEFINED,    // RST
  0,                  // Rotacion
  true                // IPS
);

static const uint32_t screenWidth = 240;
static const uint32_t screenHeight = 240;

// --------------------------------------------------
// ENCODER Y PULSADOR
// --------------------------------------------------

#define ROTARY_ENCODER_A_PIN       7
#define ROTARY_ENCODER_B_PIN       6
#define ROTARY_ENCODER_BUTTON_PIN  9

Encoder myEnc(
  ROTARY_ENCODER_A_PIN,
  ROTARY_ENCODER_B_PIN
);

Bounce2::Button button = Bounce2::Button();

// Rango y valor normal: ver "TIPO DE PERILLA" arriba

int32_t porcentaje = 0;   // ultimo valor recibido de LinuxCNC
int32_t last_encoder_value = 0;

// Datos recibidos por ESP-NOW (los escribe el callback, los lee el loop)
volatile bool nuevoValor = false;
volatile int32_t valorRx = 0;
volatile uint32_t ultimoRxMs = 0;

bool conectado = false;
uint32_t ultimoHb = 0;
uint32_t resetMs = 0;      // momento en que se presiono el boton
const uint32_t TIMEOUT_MS = 3000;   // sin datos de LinuxCNC => "--"

// --------------------------------------------------
// LVGL
// --------------------------------------------------

static lv_disp_draw_buf_t draw_buf;
static lv_disp_drv_t disp_drv;

static lv_color_t *disp_draw_buf1 = nullptr;
static lv_color_t *disp_draw_buf2 = nullptr;

static lv_obj_t *arc = nullptr;
static lv_obj_t *value_label = nullptr;
static lv_obj_t *title_label = nullptr;
static lv_obj_t *led_rojo = nullptr; // Objeto LED de LVGL

static int32_t animated_value = 0;

// Parpadeo del LED
static lv_timer_t *led_timer = nullptr;
static bool led_activo = false;              // true cuando el feed rate es distinto de 100
static const uint32_t LED_BLINK_MS = 200;    // periodo de parpadeo (ms)

// --------------------------------------------------
// ENVIO DE PIXELES A LA PANTALLA
// --------------------------------------------------

void my_disp_flush(
  lv_disp_drv_t *disp,
  const lv_area_t *area,
  lv_color_t *color_p
) {
  uint32_t w = area->x2 - area->x1 + 1;
  uint32_t h = area->y2 - area->y1 + 1;

#if (LV_COLOR_16_SWAP != 0)
  gfx->draw16bitBeRGBBitmap(
    area->x1,
    area->y1,
    (uint16_t *)&color_p->full,
    w,
    h
  );
#else
  gfx->draw16bitRGBBitmap(
    area->x1,
    area->y1,
    (uint16_t *)&color_p->full,
    w,
    h
  );
#endif

  lv_disp_flush_ready(disp);
}

// --------------------------------------------------
// ANIMACION DE LA BARRA
// --------------------------------------------------

static void arc_anim_exec(void *var, int32_t value) {
  (void)var;

  lv_arc_set_value(arc, value);
  animated_value = value;

  lv_label_set_text_fmt(
    value_label,
    FORMATO,
    (int)value
  );

  // El LED parpadea mientras el feed rate sea distinto de 100
  led_activo = (value != FEED_NORMAL);

  if (!led_activo && led_rojo != nullptr) {
    lv_led_off(led_rojo);
  }
}

// Callback del timer: alterna el LED si esta activo
static void led_blink_cb(lv_timer_t *timer) {
  (void)timer;

  if (led_rojo == nullptr) return;

  if (led_activo) {
    lv_led_toggle(led_rojo);
  } else {
    lv_led_off(led_rojo);
  }
}

void animarProgressBar(int32_t nuevo_valor, uint32_t duracion_ms) {
  // Evita acumular animaciones si el encoder gira rapidamente.
  lv_anim_del(&animated_value, arc_anim_exec);

  lv_anim_t anim;
  lv_anim_init(&anim);

  lv_anim_set_var(&anim, &animated_value);
  lv_anim_set_exec_cb(&anim, arc_anim_exec);
  lv_anim_set_values(&anim, animated_value, nuevo_valor);
  lv_anim_set_time(&anim, duracion_ms);
  lv_anim_set_path_cb(&anim, lv_anim_path_ease_out);

  lv_anim_start(&anim);
}

// --------------------------------------------------
// CREAR INTERFAZ GRAFICA
// --------------------------------------------------

void crearInterfaz() {
  lv_obj_t *scr = lv_scr_act();

  // Fondo
  lv_obj_set_style_bg_color(
    scr,
    lv_color_hex(0x080C12),
    LV_PART_MAIN
  );

  lv_obj_set_style_bg_opa(
    scr,
    LV_OPA_COVER,
    LV_PART_MAIN
  );

  // Arco circular
  arc = lv_arc_create(scr);

  lv_obj_set_size(arc, 240, 240);
  //lv_obj_set_size(arc, 190, 190);
  lv_obj_center(arc);

  lv_arc_set_rotation(arc, 135);
  lv_arc_set_bg_angles(arc, 0, 270);
  lv_arc_set_range(arc, 0, FEED_MAX);
  lv_arc_set_value(arc, 0);
  lv_arc_set_mode(arc, LV_ARC_MODE_NORMAL);

  // Ocultar el pequeño indicador/knob del arco
  lv_obj_remove_style(arc, NULL, LV_PART_KNOB);

  // Grosor del arco
  lv_obj_set_style_arc_width(
    arc, 30, LV_PART_MAIN
  );

  lv_obj_set_style_arc_width(
    arc, 30, LV_PART_INDICATOR
  );

  // Color de fondo del arco
  lv_obj_set_style_arc_color(
    arc,
    lv_color_hex(0x26313D),
    LV_PART_MAIN
  );

  // Color del progreso
  lv_obj_set_style_arc_color(
    arc,
    lv_color_hex(COLOR_PRINCIPAL),
    LV_PART_INDICATOR
  );

  // Sin sombra
  lv_obj_set_style_shadow_width(
    arc, 0, LV_PART_MAIN
  );

  // Porcentaje central
  value_label = lv_label_create(scr);
  lv_label_set_text(value_label, "--");

  lv_obj_set_style_text_font(value_label, &lv_font_montserrat_48, LV_PART_MAIN);  // <-- nueva

  lv_obj_set_style_text_color(
    value_label,
    lv_color_hex(COLOR_PRINCIPAL),
    LV_PART_MAIN
  );

  lv_obj_center(value_label);

  // CREACIÓN DEL LED ROJO (Arriba del porcentaje)
  led_rojo = lv_led_create(scr);
  lv_obj_set_size(led_rojo, 16, 16); // Tamaño del LED redondo (16x16 píxeles)
  
  // Color del LED (Rojo). En LVGL el LED usa su propio color,
  // por eso hay que usar lv_led_set_color() y no el estilo bg_color.
  lv_led_set_color(led_rojo, lv_color_hex(0xFF0000));
  
  // Alinearlo justo arriba del label de porcentaje (desplazado -45px en el eje Y hacia arriba)
  lv_obj_align(led_rojo, LV_ALIGN_CENTER, 0, -45); 
  
  // Estado inicial: Apagado
  lv_led_off(led_rojo);

  // Timer que hace parpadear el LED
  led_timer = lv_timer_create(led_blink_cb, LED_BLINK_MS, NULL);

  // Texto inferior en dos filas: FEED / RATE
  title_label = lv_label_create(scr);
  lv_label_set_text(title_label, TITULO);

  // Letra más grande. Montserrat 24 está disponible en LVGL
  // cuando se habilitan las fuentes estándar.
  lv_obj_set_style_text_font(
    title_label,
    &lv_font_montserrat_24,
    LV_PART_MAIN
  );

  lv_obj_set_style_text_color(
    title_label,
    lv_color_hex(COLOR_PRINCIPAL),
    LV_PART_MAIN
  );

  // Centrar las dos filas debajo del porcentaje.
  lv_obj_set_style_text_align(
    title_label,
    LV_TEXT_ALIGN_CENTER,
    LV_PART_MAIN
  );

  lv_obj_align(
    title_label,
    LV_ALIGN_CENTER,
    0,
    65
  );
}

// --------------------------------------------------
// ESP-NOW Y COMUNICACION CON LINUXCNC
// --------------------------------------------------

// Sin conexion: arco en cero, "--" y LED apagado
void mostrarDesconectado() {
  lv_anim_del(&animated_value, arc_anim_exec);
  animated_value = 0;
  lv_arc_set_value(arc, 0);
  lv_label_set_text(value_label, "--");
  led_activo = false;
  lv_led_off(led_rojo);
}

// Envia "KN|<ROL>|<msg>" al puente (con confirmacion de recepcion)
void enviarAlPuente(const char *msg) {
  char buf[40];
  int n = snprintf(buf, sizeof(buf), "KN|%s|%s", ROL, msg);
  esp_now_send(BRIDGE_MAC, (const uint8_t *)buf, n);
}

// Paquete recibido: "KN|<ROL>|V<n>". Se ejecuta en la tarea WiFi,
// por eso aqui NO se toca LVGL: solo se guardan los datos.
void procesarPaquete(const uint8_t *data, int len) {
  char buf[40];
  if (len <= 3 || len >= (int)sizeof(buf)) return;
  memcpy(buf, data, len);
  buf[len] = 0;

  if (strncmp(buf, "KN|", 3) != 0) return;
  char *rol = buf + 3;
  char *sep = strchr(rol, '|');
  if (!sep) return;
  *sep = 0;
  if (strcmp(rol, ROL) != 0) return;   // mensaje de otra perilla

  char *msg = sep + 1;
  if (msg[0] == 'V') {
    valorRx = atoi(msg + 1);
    ultimoRxMs = millis();
    nuevoValor = true;
  }
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

void iniciarEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(CANAL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error al iniciar ESP-NOW");
    while (true) delay(1000);
  }

  esp_now_register_recv_cb(onRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BRIDGE_MAC, 6);
  peer.channel = CANAL;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  Serial.println("ESP-NOW listo");
}

void atenderEspNow() {
  // Latido cada 1 s: LinuxCNC sabe que la perilla esta viva
  if (millis() - ultimoHb > 1000) {
    ultimoHb = millis();
    enviarAlPuente("H");
  }

  if (nuevoValor) {
    nuevoValor = false;

    int32_t v = valorRx;
    if (v < 0) v = 0;
    if (v > FEED_MAX) v = FEED_MAX;

    // LinuxCNC repite el valor cada segundo: solo animar si cambio
    if (!conectado || v != porcentaje) {
      // Tras presionar el boton, regreso lento; en lo demas, rapido
      bool porReset = (millis() - resetMs) < 500;
      animarProgressBar(v, porReset ? 1000 : 180);

      Serial.print("Valor: ");
      Serial.println(v);
    }

    porcentaje = v;
    conectado = true;
  }

  if (conectado && (millis() - ultimoRxMs > TIMEOUT_MS)) {
    conectado = false;
    mostrarDesconectado();
    Serial.println("Sin senal de LinuxCNC");
  }
}

// --------------------------------------------------
// LECTURA DEL ENCODER Y DEL PULSADOR
// --------------------------------------------------

void leerEncoder() {
  button.update();

  // Pulsar el knob pide a LinuxCNC regresar a 100
  if (button.pressed()) {
    resetMs = millis();
    enviarAlPuente("R");
    Serial.println("Reset enviado");
  }

  // Se conserva la escala de lectura del programa original.
  int32_t current_encoder_value = myEnc.read() / 2;
  int32_t encoder_diff =
    current_encoder_value - last_encoder_value;

  if (encoder_diff != 0) {
    last_encoder_value = current_encoder_value;

    // 5 unidades por clic, como antes. LinuxCNC aplica los limites.
    char m[16];
    snprintf(m, sizeof(m), "E%d", (int)(encoder_diff * PASO_CLIC));
    enviarAlPuente(m);
  }
}

// --------------------------------------------------
// SETUP
// --------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println();
  Serial.println("Iniciando ESP32-C3...");

  // Backlight: se conserva el nivel usado en tu programa.
  pinMode(GFX_BL, OUTPUT);
  digitalWrite(GFX_BL, LOW);

  // Inicializar pantalla
  if (!gfx->begin(80000000)) {
    Serial.println("Error al inicializar GC9A01");
    while (true) delay(1000);
  }

  Serial.println("Pantalla GC9A01 inicializada");

  // ESP-NOW
  iniciarEspNow();

  // Inicializar LVGL
  lv_init();

  // Configurar pulsador como en el programa original.
  button.attach(ROTARY_ENCODER_BUTTON_PIN, INPUT);
  button.interval(5);
  button.setPressedState(LOW);

  // Inicializar encoder
  myEnc.write(0);
  last_encoder_value = 0;

  // Reservar buffers de dibujo (1/8 de pantalla cada uno).
  const uint32_t bufferSize =
    (screenWidth * screenHeight) / 8;

  disp_draw_buf1 = (lv_color_t *)heap_caps_malloc(
    sizeof(lv_color_t) * bufferSize,
    MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT
  );

  disp_draw_buf2 = (lv_color_t *)heap_caps_malloc(
    sizeof(lv_color_t) * bufferSize,
    MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT
  );

  if (disp_draw_buf1 == nullptr || disp_draw_buf2 == nullptr) {
    Serial.println("Error: no hay memoria para los buffers LVGL");
    while (true) delay(1000);
  }

  lv_disp_draw_buf_init(
    &draw_buf,
    disp_draw_buf1,
    disp_draw_buf2,
    bufferSize
  );

  // Registrar driver de pantalla
  lv_disp_drv_init(&disp_drv);

  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;

  lv_disp_drv_register(&disp_drv);

  Serial.println("Driver LVGL inicializado");

  // Crear interfaz
  crearInterfaz();

  Serial.println("ProgressBar lista");
}

// --------------------------------------------------
// LOOP
// --------------------------------------------------

void loop() {
  // Leer encoder y pulsador
  leerEncoder();
  atenderEspNow();

  // LVGL 8: tick con el tiempo real transcurrido (ESP-NOW puede
  // alargar algunas vueltas del loop) y tareas graficas.
  static uint32_t t_prev = millis();
  uint32_t t_now = millis();
  lv_tick_inc(t_now - t_prev);
  t_prev = t_now;

  lv_timer_handler();

  delay(5);
}
