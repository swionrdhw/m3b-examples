# M3B Demo Board: Network Tester & GPS Tracker

## Introduction

This is an arduino based code example for the M3B Demo Board to test the mioty signal coverage by sending uplinks to a base station that responds with a downlink containing the signal quality of the uplink.

This example is intended for the Swissphone m.RADIO, an M3B extension with display and GPS module.

It contains three different application modes:
- Coverage test mode manual: sends mioty uplinks on button press and displays signal quality.
- Coverage test mode periodic : periodically sends mioty uplinks and manually on button press. Displays signal quality on manually triggered uplinks.
- GPS tracker: periodically sends position.

## Getting Started

1. Install Arduino IDE (tested with v2.1.1)
2. Install [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html)
3. Set STM32duino path to board managers:
    - ArduinoIDE > File > Preferences > Setting > Additional board manager URLs > https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json
4. Add [stm32duino](https://github.com/stm32duino) to Board Managers (v2.6.0)
5. Add  to library manager:
     - [Adafruit_SH110X](https://github.com/adafruit/Adafruit_SH110X) (v2.1.8)
     - [Adafruit_GPS](https://github.com/adafruit/Adafruit_GPS) (v1.7.2)
     - [TimeLib](https://playground.arduino.cc/Code/Time/) (v1.6.1)
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

On startup one of the three application modes can be selected by pressing the corresponding button.
Depending on the mode, different settings can be changed by pressing the buttons:
- Uplink message period
- Transmission power
- mioty uplink mode {Standard, Low latency}

The manual uplink can be triggered with the side button.

Make sure the GPS power switch is ON when using the GPS tracker mode. It can be OFF when the other modes are used to save energy.

This example requires m.YON FW version 1.2.0 or newer to work correctly.

The mioty module must be attached before starting this example. This can be done with the tunnel mode in the serial tunnel example or the sensor example. Refer to these examples for more information.

The GPS Tracker does not give any feedback about the success of a transmission. It is advised to test the functionality of the mioty module in the coverage mode first.

## Blueprint

The manual uplink has the MPF = 0 and sends an increasing message number and the data from the acceleration sensor to determine the device orientation.
The downlink sends the RSSI, SNR, eqSNR and timestamp of the preceding uplink.
See [Blueprint](../m3b_demo_blueprint.txt).

The periodic uplink has the MPF = 194 and also sends an increasing message number. It is a separate message number from the one used in the manually triggered uplinks.

The GPS tracker payload uses the MPF = 193 and sends the information if a valid fix is available and the number of sattelites used. If a fix is available it also sends the latest position, accuracy and speed.

## Backend

The funcionality to send back the downlink on the manual uplink and to store the sent uplinks for later evaluation must be implemented in the backend. This [Node Red flow](../node_red_flow.json) creates the downlink containing the needed data to display the signal quality.

TODO: Refer to CenterBox?
