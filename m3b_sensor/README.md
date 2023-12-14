# M3B Demo Board: Sensor example

## Introduction

This is an arduino based code example for the M3B Demo Board to periodically read the on board sensors and send the data over mioty.

This example can be used with a standalone M3B Demo Board or with the Swissphone m.RADIO. This allows to also display the measured sensor data on the device and use a button to change the sending period.

## Getting Started

1. Install Arduino IDE (tested with v2.1.1)
2. Install [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html)
3. Add [stm32duino](https://github.com/stm32duino) to Board Managers (v2.6.0)
4. Add  to library manager:
     - [RingBuffer](https://github.com/Locoduino/RingBuffer) (v1.0.4)
     - [Adafruit_SH110X](https://github.com/adafruit/Adafruit_SH110X) (v2.1.8) (only if display is used)
     - [sht31](https://github.com/RobTillaart/SHT31) (v0.3.8)
     - [ms5637](https://github.com/sparkfun/SparkFun_MS5637_Arduino_Library) (v1.0.2)
     - [adxl362](https://www.arduino.cc/reference/en/libraries/adxl362/) (v1.5.0)
     - [at_client](https://www.github.com/mioty-iot/mioty_at_client_c)

## Flashing STM32 from Arduino IDE

1. Connect Device via UART (e.g. FTDI cable)
2. Under Arduino IDE -> Tools:
    - Select Board:             Generic STM32L0 Series
    - Select Board Part Number: Generic L072RBTx
    - Select USART Support:     Enabled (**generic** 'Serial')
    - Select Upload Method:     Stm32CubeProgrammer (Serial)
    - Make sure to select the correct port
3. Enable Bootloader mode on the m3b (i.e. shift the bootloader switch & press the uController reset button)
4. Upload sketch
5. Release the boot switch
6. Press reset button to start your sketch

## Usage

This application reads the following sensor values on the M3B Board periodically every second: humidity, temperature, pressure, acceleration.
The measured data are sent in a mioty uplink every minute. The blue LED indicates if it is currently transmitting.
If a display is present, the measured data are also displayed on it.
This example also includes a tunnel mode to forward commands over the serial interface to the mioty module. This can be used to change settings on the module, e.g. EUI, attachment etc.

## Blueprint

The uplink uses the payload format 192, see [Blueprint](../m3b_demo_blueprint.txt).
