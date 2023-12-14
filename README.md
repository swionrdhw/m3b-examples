# M3B Demo Board Examples

This repository provides various code examples based on arduino for the M3B Demo Board with the Swissphone mioty module m.YON.

- Tunnel mode & modul firmware upgrade
- Tester Taster & CW mode
- GNSS Tracker
- mioty sensor
- CW mode

The Tester Taster and GNSS Tracker require the Swissphone M3B board extension with display and GNSS module. (The sensor example can also use the display to show the sensor values.)
For the GNSS example, the GNSS switch on the extender must b enabled. For the other examples it is advised to disable it to avoid unnecessary energy use by the GNSS module.

For more information on the examples see the README in the respective exmaple folder.

For mode information about the M3B Demo Board see [M3B Product page](https://shop.lze-innovation.de/products/mioty-magnolinq-makerboard-m3b).

All examples use the same blueprint for the payloads, see [Blueprint](m3b_demo_blueprint.txt).
