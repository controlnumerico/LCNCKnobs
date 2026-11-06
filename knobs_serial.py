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

Requisitos:
  sudo apt install python3-serial
  sudo usermod -aG dialout $USER      (cerrar sesion y volver a entrar)
"""
import hal
import serial
import time

# Mejor usar la ruta fija: ls /dev/serial/by-id/
PUERTO_SERIAL = "/dev/ttyACM0"
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


perillas = {rol: Perilla(rol) for rol in ROLES}
h.ready()


def abrir():
    s = serial.Serial()
    s.port = PUERTO_SERIAL
    s.baudrate = BAUD
    s.timeout = 0.02
    s.dtr = False
    s.rts = False
    s.open()
    return s


def procesar(linea):
    partes = linea.split()
    if len(partes) < 2:
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

        # (Re)abrir el puerto serial
        if ser is None:
            try:
                ser = abrir()
                buf = b""
            except (serial.SerialException, OSError):
                time.sleep(1.0)
                continue

        # Leer lineas del puente
        try:
            datos = ser.read(ser.in_waiting or 1)
        except (serial.SerialException, OSError):
            ser.close()
            ser = None
            continue

        buf += datos
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
        except (serial.SerialException, OSError):
            ser.close()
            ser = None
except KeyboardInterrupt:
    raise SystemExit
