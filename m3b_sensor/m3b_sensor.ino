/*
  Example Code for the m3b Demo Board with m.YON mioty module.
  
  This demo application periodically reads the on board sensors
  and sends a mioty uplink.
*/

#include "HardwareSerial.h"
#include "Wire.h"
#include <Adafruit_SH110X.h>
#include "miotyAtClient.h"
#include "RingBuf.h"
#include <SHT31.h>
#include <SparkFun_MS5637_Arduino_Library.h>
#include <ADXL362.h>


#define LED_RED PC8
#define LED_GREEN PC7
#define LED_BLUE PC6
#define LED_STATUS PC13

#define LED_RED_EXTENDER PC1
#define LED_GREEN_EXTENDER PC2
#define LED_BLUE_EXTENDER PC3

#define BUTTON_C PC9

#define MIOTY_IRQ_PIN PB6

#define UPDATE_PERIOD 2000  // 2s

#define BUTTON_DEBOUNCE_TIME 150  // ms

#define ARRAY_LENGTH(a) (sizeof(a) / sizeof(a[0]))


static void display_mioty_logo_f(void);
static void update_display_f(void);
static void send_uplink_f(void);
static void button_c_cb_f(void);
static bool check_debounce_f(void);


/* 
  Use a ring buffer to buffer the data sent from SerialM3B to SerialMioty.
  The higher baudrate of SerialM3B would else cause the Tx buffer of
  SerialMioty to overflow on large commands.
*/
static RingBuf<uint8_t, 1024> cmd_buffer;

static HardwareSerial SerialMioty(PC11, PC10);
static HardwareSerial SerialM3B(PA10, PA9);

static TwoWire Wire2(PB9, PB8);
static Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire2);

static SHT31 sht31;
static MS5637 ms5637;
static ADXL362 adxl362;

static bool display_present_b;
static const uint8_t ul_period_setting_a[]{ 15, 30, 60, 90, 120, 180 };
static uint8_t ul_period_choice = 2;
static uint32_t last_button_press;  // for button debounce

static uint32_t last_tx_time = 0xFFFF;
static uint32_t last_update = 0xFFFF;

static float humidity;
static float temperature;
static float pressure;
static int16_t acc_x, acc_y, acc_z;


void setup() {
  /* setup pins */
  pinMode(LED_STATUS, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_RED_EXTENDER, OUTPUT);
  pinMode(LED_GREEN_EXTENDER, OUTPUT);
  pinMode(LED_BLUE_EXTENDER, OUTPUT);
  pinMode(MIOTY_IRQ_PIN, INPUT_PULLUP);
  digitalWrite(LED_RED, 1);
  digitalWrite(LED_GREEN, 1);
  digitalWrite(LED_BLUE, 1);
  digitalWrite(LED_RED_EXTENDER, 1);
  digitalWrite(LED_GREEN_EXTENDER, 1);
  digitalWrite(LED_BLUE_EXTENDER, 1);

  /* setup Serials */
  SerialMioty.begin(9600);
  SerialM3B.begin(115200);

  uint32_t reset_time = millis();
  const char reset_cmd[] = "AT-RST\r";
  SerialMioty.write((uint8_t*)reset_cmd, 7);
  /* wait for IRQ pin to go low and then high again, indicating the module is ready, wait at most 1s */
  while ((digitalRead(MIOTY_IRQ_PIN) == 1) && ((millis() - reset_time) < 1000))
    ;
  while ((digitalRead(MIOTY_IRQ_PIN) == 0) && ((millis() - reset_time) < 1000))
    ;

  Wire2.begin();
  display_present_b = display.begin(0x3C, true);

  if (display_present_b) {
    display.setRotation(1);
    display.setTextColor(SH110X_WHITE);
    display_mioty_logo_f();
    display.setTextSize(1);
    pinMode(BUTTON_C, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BUTTON_C), button_c_cb_f, FALLING);
  }

  sht31.begin(0x44, &Wire2);
  ms5637.begin(Wire2);

  adxl362.begin(PA8);
  adxl362.beginMeasure();


  digitalWrite(LED_STATUS, 0);
  SerialM3B.println("");
  SerialM3B.println("M3B Sensor Demo started");
}

void loop() {
  uint32_t uptime = millis();
  if ((uptime - last_update) > UPDATE_PERIOD) {
    sht31.read();
    humidity = sht31.getHumidity();
    temperature = ms5637.getTemperature();
    pressure = ms5637.getPressure();
    int16_t temp;
    adxl362.readXYZTData(acc_x, acc_y, acc_z, temp);
    if (display_present_b) {
      update_display_f();
    }
    last_update = uptime;
  }
  if ((uptime - last_tx_time) > (ul_period_setting_a[ul_period_choice] * 1000)) {
    digitalWrite(LED_BLUE, 0);
    digitalWrite(LED_BLUE_EXTENDER, 0);
    send_uplink_f();
    digitalWrite(LED_BLUE, 1);
    digitalWrite(LED_BLUE_EXTENDER, 1);
    last_tx_time = uptime;
  }
  if (SerialM3B.available()) {
    cmd_buffer.push(SerialM3B.read());
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

static void display_mioty_logo_f(void) {
  display.clearDisplay();
  display.fillTriangle(9, 50, 24, 50, 9, 35, SH110X_WHITE);
  display.fillTriangle(24, 20, 24, 50, 9, 35, SH110X_WHITE);
  display.fillTriangle(24, 20, 24, 50, 39, 35, SH110X_WHITE);
  display.fillTriangle(24, 20, 39, 20, 39, 35, SH110X_WHITE);
  display.setTextSize(2);
  display.setCursor(45, 30);
  display.print("mioty");
  display.display();
}

static void update_display_f(void) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Temperature: ");
  display.print(temperature, 1);
  display.println(" C");
  display.print("Pressure: ");
  display.print(pressure, 1);
  display.println("hPa");
  display.print("Humidity: ");
  display.print(humidity, 1);
  display.println("%");
  display.print("X: ");
  display.println(acc_x);
  display.print("Y: ");
  display.println(acc_y);
  display.print("Z: ");
  display.println(acc_z);
  display.print("\nC: UL period: ");
  display.print(ul_period_setting_a[ul_period_choice]);
  display.println("s");
  display.display();
}

static void send_uplink_f(void) {
  uint8_t msg[14] = { 0xC0 };
  uint16_t temp_int = (temperature + 273.1f) * 10;
  uint16_t pres_int = pressure * 10;
  uint32_t pcnt = 0;
  msg[1] = (temp_int) >> 8;
  msg[2] = temp_int;
  msg[3] = (uint8_t)humidity;
  msg[4] = pres_int >> 8;
  msg[5] = pres_int;
  msg[6] = acc_x >> 8;
  msg[7] = acc_x;
  msg[8] = acc_y >> 8;
  msg[9] = acc_y;
  msg[10] = acc_z >> 8;
  msg[11] = acc_z;
  msg[12] = 0;  // Battery voltage, not measured on HW index A
  msg[13] = 0;
  miotyAtClient_sendMessageUniMPF(msg, sizeof(msg), &pcnt);
}

static void button_c_cb_f(void) {
  if (!check_debounce_f()) {
    return;
  }
  ul_period_choice = (ul_period_choice + 1) % ARRAY_LENGTH(ul_period_setting_a);
  update_display_f();
}

static bool check_debounce_f(void) {
  if ((millis() - last_button_press) > BUTTON_DEBOUNCE_TIME) {
    last_button_press = millis();
    return true;
  }
  return false;
}

void miotyAtClientWrite(uint8_t* msg, uint16_t len) {
  SerialMioty.write(msg, len);
  SerialM3B.println("");
  SerialM3B.write(msg, len);
  SerialM3B.println("");
}

bool miotyAtClientRead(uint8_t* buf, uint8_t* p_len) {
  int i = 0;
  while (SerialMioty.available() > 0) {
    buf[i++] = SerialMioty.read();
  }
  SerialM3B.write(buf, i);
  *p_len = i;
  return true;
}
