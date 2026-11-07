#!/usr/bin/env python3
"""
knobs_serial.py - Puente serial (ESP32-S3 + ESP-NOW) <-> LinuxCNC (HAL)

Lineas por el puerto serial (una por mensaje):
  puente -> PC:  <ROL> E<n> | <ROL> R | <ROL> H     (pulsos, boton, latido)
  PC -> puente:  <ROL> V<n>                         (valor actual: en %, o unidades/min en VMAX)

Pines HAL por perilla (rol FEED => knobs.feed.*):
  .counts     s32   out  contador relativo de pulsos
  .speed      float out  (solo perillas absolutas, ej. JOG) valor en unidades/s
  .reset      bit   out  pulso de reset
  .value      float in   valor actual (1.0 = 100%)
  .conectado  bit   out  la perilla esta enviando latidos

Teclado matricial 4x4 (modulo KEYPAD_ESPNOW, rol KEYS):
  modulo -> PC:  KEYS K<hhhh>   estado de las 16 teclas en hexadecimal
  Pines HAL:
    knobs.io.key-00 ... knobs.io.key-15   bit out  1 mientras la tecla esta presionada
    knobs.io.conectado                    bit out  el modulo esta enviando
  Si el modulo deja de enviar, las teclas se sueltan solas (no quedan pegadas).
  Entrada u32 desde el modulo (ActiveTool):
    knobs.io.in-0                         u32 out  valor de ActiveTool
  modulo -> PC:  KEYS T<n>      ActiveTool en decimal
  Salidas hacia el modulo (2 valores u32 desde LinuxCNC):
    knobs.io.out-0, knobs.io.out-1        u32 in   se mandan al modulo
  PC -> modulo:  KEYS O<a>,<b>   al cambiar y cada REENVIO segundos

Reconexion USB:
  Si el puente se desconecta (cable, ruido, suspension), el script cierra el
  puerto, busca el puente otra vez y reabre solo. Al reconectar reenvia de
  inmediato el valor de todas las perillas.

Requisitos:
  sudo apt install python3-serial
  sudo usermod -aG dialout $USER      (cerrar sesion y volver a entrar)
"""
import glob
import hal
import os
import serial
import sys
import time

# --- Puerto serial ---------------------------------------------------------
# Si PUERTO_SERIAL tiene un valor (por ejemplo una regla udev como
# "/dev/puente_knobs" o una ruta de /dev/serial/by-id/), se usa esa ruta.
# Si queda vacio, se busca automaticamente un dispositivo Espressif en
# /dev/serial/by-id/ cada vez que se intenta conectar, asi no importa si el
# sistema lo renombra de ttyACM0 a ttyACM1 al reconectar.
PUERTO_SERIAL = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5C85029247-if00"
PUERTO_RESPALDO = "/dev/ttyACM0"   # solo si no se encuentra nada por by-id
PATRON_BY_ID = "/dev/serial/by-id/*Espressif*"   # solo se usa si PUERTO_SERIAL esta vacio

BAUD = 115200
ROLES = ["FEED", "SPINDLE", "RAPID", "VMAX", "JOG"]

# Factor para convertir el .value de HAL al numero que muestra la perilla.
# Por defecto 100 (1.0 = 100 %). VMAX: halui.max-velocity.value esta en
# unidades de maquina por SEGUNDO y la perilla muestra por MINUTO => x60
FACTOR_VALOR = {"VMAX": 60.0, "JOG": 60.0}

# Perillas "absolutas": halui no tiene pin de cuentas para esto (jog-speed es
# un float directo), asi que el valor vive AQUI y sale por el pin .speed
# (unidades de maquina por SEGUNDO). El boton regresa al valor INICIO_ABS.
ABSOLUTAS = {"JOG"}
INICIO_ABS = {"JOG": 800}      # unidades/min al arrancar y al presionar el boton
LIMITE_ABS = (0, 3200)         # unidades/min

TIMEOUT_PERILLA = 3.0   # s sin latidos => .conectado = 0
REENVIO = 1.0           # s: se repite el valor aunque no cambie
ESPERA_REINTENTO = 1.0  # s entre intentos de reconexion

# Errores que indican que el puerto se cayo o se desconecto
ERRORES_PUERTO = (serial.SerialException, OSError, ValueError)

h = hal.component("knobs")


class Perilla:
    def __init__(self, rol):
        self.rol = rol
        base = rol.lower()
        self.counts = h.newpin(base + ".counts", hal.HAL_S32, hal.HAL_OUT)
        self.reset = h.newpin(base + ".reset", hal.HAL_BIT, hal.HAL_OUT)
        self.value = h.newpin(base + ".value", hal.HAL_FLOAT, hal.HAL_IN)
        self.conectado = h.newpin(base + ".conectado", hal.HAL_BIT, hal.HAL_OUT)
        self.factor = FACTOR_VALOR.get(rol, 100.0)
        self.absoluta = rol in ABSOLUTAS
        if self.absoluta:
            self.abs = INICIO_ABS[rol]
            self.speed = h.newpin(base + ".speed", hal.HAL_FLOAT, hal.HAL_OUT)
            self.speed.set(self.abs / 60.0)
        self.n = 0
        self.reset_hasta = 0.0
        self.visto = 0.0
        self.ultimo = None
        self.t_envio = 0.0


class Teclado:
    """Modulo de entradas con teclado matricial 4x4 (rol KEYS)."""
    ROL = "KEYS"
    N = 16

    def __init__(self):
        self.teclas = [h.newpin("io.key-%02d" % i, hal.HAL_BIT, hal.HAL_OUT)
                       for i in range(self.N)]
        self.conectado = h.newpin("io.conectado", hal.HAL_BIT, hal.HAL_OUT)
        self.out = [h.newpin("io.out-%d" % i, hal.HAL_U32, hal.HAL_IN)
                    for i in range(2)]
        self.tool = h.newpin("io.in-0", hal.HAL_U32, hal.HAL_OUT)   # ActiveTool
        self.mascara = 0
        self.visto = 0.0
        self.ultimo = None
        self.t_envio = 0.0

    def poner(self, mascara):
        self.mascara = mascara
        for i in range(self.N):
            self.teclas[i].set(bool((mascara >> i) & 1))


perillas = {rol: Perilla(rol) for rol in ROLES}
teclado = Teclado()
h.ready()


def log(texto):
    print("[knobs] " + texto)
    sys.stdout.flush()


def buscar_puerto():
    """Devuelve la ruta del puerto del puente, o None si no esta conectado."""
    if PUERTO_SERIAL:
        return PUERTO_SERIAL if os.path.exists(PUERTO_SERIAL) else None
    encontrados = sorted(glob.glob(PATRON_BY_ID))
    if encontrados:
        return encontrados[0]
    if os.path.exists(PUERTO_RESPALDO):
        return PUERTO_RESPALDO
    return None


def abrir():
    puerto = buscar_puerto()
    if puerto is None:
        raise serial.SerialException("puente no encontrado")
    s = serial.Serial()
    s.port = puerto
    s.baudrate = BAUD
    s.timeout = 0.02
    s.write_timeout = 1.0     # evita que una escritura se quede colgada
    s.dtr = False
    s.rts = False
    s.open()
    log("conectado a " + puerto)
    return s


def cerrar(s):
    """Cierra el puerto sin que un error al cerrar tire el componente."""
    try:
        s.close()
    except Exception:
        pass


def desconectado():
    """Marca todas las perillas como desconectadas y fuerza reenvio al volver."""
    for p in perillas.values():
        p.visto = 0.0
        p.ultimo = None
        p.conectado.set(0)
    teclado.visto = 0.0
    teclado.ultimo = None
    teclado.conectado.set(0)
    teclado.poner(0)


def procesar_teclado(partes):
    teclado.visto = time.time()
    msg = partes[1]
    if msg[0] == "K":
        try:
            teclado.poner(int(msg[1:], 16) & 0xFFFF)
        except ValueError:
            return
    elif msg[0] == "T":
        try:
            teclado.tool.set(int(msg[1:]) & 0xFFFFFFFF)
        except ValueError:
            return
    # "H" (latido) solo actualiza teclado.visto


def procesar(linea):
    partes = linea.split()
    if len(partes) < 2:
        return
    if partes[0].upper() == Teclado.ROL:
        procesar_teclado(partes)
        return
    p = perillas.get(partes[0].upper())
    if p is None:
        return
    p.visto = time.time()
    msg = partes[1]
    if msg[0] == "E":
        try:
            d = int(msg[1:])
        except ValueError:
            return
        if p.absoluta:
            p.abs = max(LIMITE_ABS[0], min(LIMITE_ABS[1], p.abs + d))
            p.speed.set(p.abs / 60.0)
        else:
            p.n += d
            p.counts.set(p.n)
    elif msg[0] == "R":
        if p.absoluta:
            p.abs = INICIO_ABS[p.rol]
            p.speed.set(p.abs / 60.0)
        else:
            p.reset.set(1)
            p.reset_hasta = time.time() + 0.1


ser = None
buf = b""

try:
    while True:
        ahora = time.time()

        # Terminar el pulso de reset y actualizar estado de conexion
        for p in perillas.values():
            if p.reset.get() and ahora > p.reset_hasta:
                p.reset.set(0)
            p.conectado.set(ahora - p.visto < TIMEOUT_PERILLA)
        teclado_vivo = ahora - teclado.visto < TIMEOUT_PERILLA
        teclado.conectado.set(teclado_vivo)
        if not teclado_vivo and teclado.mascara:
            teclado.poner(0)    # el modulo callo: soltar todas las teclas

        # (Re)abrir el puerto serial
        if ser is None:
            try:
                ser = abrir()
                buf = b""
                # Forzar reenvio inmediato del valor de cada perilla
                teclado.ultimo = None
                for p in perillas.values():
                    p.ultimo = None
            except ERRORES_PUERTO:
                time.sleep(ESPERA_REINTENTO)
                continue

        # Leer lineas del puente
        try:
            datos = ser.read(ser.in_waiting or 1)
        except ERRORES_PUERTO:
            log("puente desconectado, reintentando...")
            cerrar(ser)
            ser = None
            desconectado()
            continue

        buf += datos
        # Proteccion: si llega basura sin saltos de linea, no crecer sin limite
        if len(buf) > 4096:
            buf = b""
        while b"\n" in buf:
            linea, buf = buf.split(b"\n", 1)
            procesar(linea.decode("ascii", "ignore"))

        # Mandar el valor actual: al cambiar y cada REENVIO segundos
        try:
            for p in perillas.values():
                v = int(round(p.value.get() * p.factor))
                if v != p.ultimo or ahora - p.t_envio > REENVIO:
                    ser.write(("%s V%d\n" % (p.rol, v)).encode())
                    p.ultimo = v
                    p.t_envio = ahora
            # Salidas u32 hacia el modulo KEYPAD: "KEYS O<a>,<b>"
            vs = tuple(int(o.get()) & 0xFFFFFFFF for o in teclado.out)
            if vs != teclado.ultimo or ahora - teclado.t_envio > REENVIO:
                ser.write(("KEYS O%d,%d\n" % vs).encode())
                teclado.ultimo = vs
                teclado.t_envio = ahora
        except ERRORES_PUERTO:
            log("error al escribir, reintentando...")
            cerrar(ser)
            ser = None
            desconectado()
except KeyboardInterrupt:
    pass
finally:
    if ser is not None:
        cerrar(ser)
    raise SystemExit
