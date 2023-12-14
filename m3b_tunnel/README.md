# M3B Demo Board: Tunnel to m.YON example

## Introduction

This is an arduino based code example for the M3B Demo Board to tunnel AT-commands from the serial interface directly to the m.YON module.
It also allows for firmware updates of the m.YON with a serial tunnel to the bootloader.

## Getting Started

1. Install Arduino IDE (tested with v2.1.1)
2. Install [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html)
3. Set STM32duino path to board managers:
    - ArduinoIDE > File > Preferences > Setting > Additional board manager URLs > https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json
4. Add [stm32duino](https://github.com/stm32duino) to Board Managers (v2.6.0)
5. Add [RingBuffer](https://github.com/Locoduino/RingBuffer) (v1.0.4) to library manager

## Flashing STM32 from Arduino IDE

1. Connect Device via UART (e.g. FTDI cable)
2. Under Arduino IDE -> Tools:
    - Select Board:             Generic STM32L0 Series
    - Select Board Part Number: Generic L072RBTx
    - Select USART Support:     Enabled (**no generic** 'Serial')
    - Select Upload Method:     Stm32CubeProgrammer (Serial)
    - Make sure to select the correct port
3. Enable Bootloader mode on the m3b (i.e. shift the bootloader switch & press the uController reset button)
4. Upload sketch
5. Release the boot switch
6. Press reset button to start your sketch

## Tunnel Mode

This demo application simply tunnels AT-commands from the serial interface to the m.YON mioty module.
Refer to the m.YON reference manual for the full list of AT-commands.
Commands can be sent from a PC when it is connected to the M3B Demo board (e.g. with an FTDI cable). On the PC they can be sent directly in the Arduino IDE serial monitor or any other serial console (e.g. [Docklight](https://docklight.de/downloads/) or [Teraterm](https://ttssh2.osdn.jp/)).

Serial settings:
- 115200 baud
- 8 data bit
- no parity
- 1 stop bit.

The appended Docklight file contains examples of some usually used commands.

## Bootloader Mode

If the character `!` `(0x21)` is sent, the bootloader mode is started. The m.YON module reboots to the bootloader and can now receive firmware update files via XMODEM protocol, e.g. with Teraterm. Refer to the m.YON reference manual for more information.
The red LED indicates that the bootloader is active.
After a finished FW update, the board must be reset with the reset-button to exit the bootloader and return to tunnel mode.

## Restrictions

Because the TX_INH, TX_ACT and RX_ACT pin of the m.YON are not connected on the M3B Demo Board, the TX inhibit, TX active and RX active functionality can not be used. The same applies for the shutdown command: **The Shutdown command should not be used as there is no possible way to wake up the module** (other than disconnecting the power source).
