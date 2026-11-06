1.- Use Arduino IDE with ESP32 Support

2.- Use lvgl by Kisvegabor 8.4.0

3.- Select "ESP32C3 Dev Module"

4.- Tools -- Partition Scheme -- Huge

5.- Run PUENTE_ESPNOW.ino and get MAC Address using serial console,   Change BRIDGE_MAC[6] on each Knob file PUENTE is a "ESP32S3 Dev Module"

6.- Copy files:  knobs_espnow.hal and knobs_serial.py  to linuxcnc config directory

7.- Add to Ini file:  HALFILE = knobs_espnow.hal

8.- Check Path in knobs_espnow.hal--  loadusr -Wn knobs python3 /home/cnc/linuxcnc/configs/sim.axis/knobs_serial.py

9.- 

1. Averigua por qué se desconecta
Abre una terminal, ejecuta sudo dmesg -w y espera a que se caiga. Si ves “USB disconnect”, suele ser cable, alimentación o ruido. Si ves “suspend” o “reset”, es manejo de energía.

2. Desactiva el autosuspend de USB
Ubuntu apaga puertos USB para ahorrar energía, y eso puede desconectar un dispositivo serial. Para probar de inmediato:

echo -1 | sudo tee /sys/module/usbcore/parameters/autosuspend

Para dejarlo permanente, edita /etc/default/grub, agrega usbcore.autosuspend=-1 a GRUB_CMDLINE_LINUX_DEFAULT y ejecuta sudo update-grub. Después reinicia.

3. Evita que ModemManager y brltty se adueñen del puerto
Pueden tomar el puerto serial del ESP32 y causar desconexiones o bloqueos:

sudo systemctl disable --now ModemManager
sudo apt remove brltty

4. Nombre fijo para el puerto
A veces el puerto cambia de ttyACM0 a ttyACM1 al reconectar. Crea una regla udev, por ejemplo /etc/udev/rules.d/99-puente.rules:

SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", SYMLINK+="puente_knobs"

303a es el identificador de Espressif; confírmalo con lsusb. Con eso tendrás /dev/puente_knobs, que siempre apunta al puente, y lo usas en knobs_serial.py. Tu usuario debe estar en el grupo dialout: sudo usermod -aG dialout $USER.



