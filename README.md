# Tempy

Wireless thermostat system based on M5Stack ESP devices and Home Assistant

GLPv3

See Wiki for details :-)



Create symlinks for secrets.h in tempy_atom_relays\secrets.h and tempy_coreink_sensor_displays\secrets.h

From the Tempy directory:

Windows 11:

<code>mklink tempy_atom_relays\secrets.h secrets.h
      mklink tempy_coreink_sensor_displays\secrets.h secrets.h
</code>

Linux:

<code>ln -s secrets.h tempy_atom_relays/secrets.h
      ln -s secrets.h tempy_sensor_displays/secrets.h
</code>
