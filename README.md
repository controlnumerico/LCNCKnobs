1.- Use Arduino IDE with ESP32 Support

2.- Use lvgl by Kisvegabor 8.4.0

3.- Select "ESP32C3 Dev Module"

4.- Tools -- Partition Scheme -- Huge

5.- Run PUENTE_ESPNOW.ino and get MAC Address using serial console,   Change BRIDGE_MAC[6] on each Knob file PUENTE is a "ESP32S3 Dev Module"

6.- Copy files:  knobs_espnow.hal and knobs_serial.py  to linuxcnc config directory

7.- Add to Ini file:  HALFILE = knobs_espnow.hal

8.- Check Path in knobs_espnow.hal--  loadusr -Wn knobs python3 /home/cnc/linuxcnc/configs/sim.axis/knobs_serial.py

9.- 

