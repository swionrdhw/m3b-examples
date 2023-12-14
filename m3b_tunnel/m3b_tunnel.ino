/*
  Example Code for the m3b Demo Board to tunnel PC serial to m.YON.
  PC Serial: 115200 Baud

  Use character '!' (0x21) to start the bootloader.
  The red LED indicates that the bootloader is active.
  Then upload new FW with XMODEM protocol.
  Press reset button to exit bootloader after upload is finished.
*/

#include "HardwareSerial.h"
#include "RingBuf.h"

#define SAFEBOOT_PIN PB7
#define LED_RED PC8
#define LED_RED_EXTENDER PC1
#define LED_STATUS PC13


HardwareSerial SerialMioty(PC11, PC10);
HardwareSerial SerialM3B(PA10, PA9);


/* 
  Use a ring buffer to buffer the data sent from SerialM3B to SerialMioty.
  The higher baudrate of SerialM3B would else cause the Tx buffer of
  SerialMioty to overflow on large commands.
*/
static RingBuf<uint8_t, 1024> cmd_buffer;
static bool bootloader_active = false;


void setup() {
  /* setup pins */
  pinMode(SAFEBOOT_PIN, OUTPUT);
  pinMode(LED_STATUS, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_RED_EXTENDER, OUTPUT);
  digitalWrite(SAFEBOOT_PIN, 1);
  digitalWrite(LED_RED, 1);
  digitalWrite(LED_RED_EXTENDER, 1);

  /* setup Serials */
  SerialMioty.begin(9600);
  SerialM3B.begin(115200);

  /* send empty command, if bootloader is active this exits it */
  SerialMioty.print("\r");
  delay(20);
  /* discard possible bytes in Rx buffer */
  while (SerialMioty.available()) {
    SerialMioty.read();
  }

  /* indicate startup */
  digitalWrite(LED_STATUS, 0);
  SerialM3B.println("");
  SerialM3B.println("M3B Serial Tunnel Demo started");
}

void loop() {
  if (SerialM3B.available()) {
    if (bootloader_active) {
      SerialMioty.write(SerialM3B.read());
    } else {
      uint8_t new_byte = SerialM3B.read();
      if (new_byte == '!') {
        SerialM3B.println("Starting Bootloader, press Reset Button when finished");
        digitalWrite(SAFEBOOT_PIN, 0);
        cmd_buffer.clear();
        SerialMioty.print("AT-RST\r");
        SerialMioty.end();
        SerialMioty.begin(115200);
        bootloader_active = true;
        digitalWrite(LED_RED, 0);
        digitalWrite(LED_RED_EXTENDER, 0);
      } else {
        cmd_buffer.push(new_byte);
      }
    }
  }
  if (!cmd_buffer.isEmpty() && SerialMioty.availableForWrite()) {
    uint8_t byte;
    cmd_buffer.pop(byte);
    SerialMioty.write(byte);
  }
  if (SerialMioty.available()) {
    SerialM3B.write(SerialMioty.read());
  }
}
