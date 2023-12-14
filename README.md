# M3B Demo Board Examples

This repository provides various code examples based on arduino for the M3B Demo Board with the Swissphone mioty module m.YON.

- Tunnel mode & modul firmware upgrade
- Network Tester & GPS Tracker
- mioty sensor

The Network Tester & GPS Tracker require the Swissphone m.RADIO hardware, which expands the M3B Board with a display and GPS module. (The sensor example can also use the display to show the sensor values.)
For the GPS Tracker, the GPS switch on the extender must be enabled. For the other examples it is advised to disable it to avoid unnecessary energy use by the GPS module.

For more information on the examples see the README in the respective exmaple folder.

For mode information about the M3B Demo Board see [M3B Product page](https://shop.lze-innovation.de/products/mioty-magnolinq-makerboard-m3b).

All examples use the same blueprint for the payloads, see [Blueprint](m3b_demo_blueprint.txt).
