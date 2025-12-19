# Tempy

Wireless thermostat system based on M5Stack ESP devices and Home Assistant

GLPv3

See Wiki for details :-)



Create symbolic links for secrets.h in tempy_atom_relays\secrets.h and tempy_coreink_sensor_displays\secrets.h

From the Tempy directory:

Arduino IDE requires hard links in Windows 11 (run cmd as Administrator):

<code>mklink /H tempy_atom_relays\secrets.h secrets.h
      mklink /H tempy_coreink_sensor_displays\secrets.h secrets.h
</code>

Linux:

<code>ln -s secrets.h tempy_atom_relays/secrets.h
      ln -s secrets.h tempy_sensor_displays/secrets.h
</code>
