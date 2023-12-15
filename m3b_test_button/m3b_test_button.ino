/*
  Example Code for the m3b Demo Board with m.YON mioty module and
  Adafruit SH110X OLED display.
  
  This demo application  allows for mioty signal coverage testing.
  It includes three modes of operation:
  1. Manual coverage test: Manually trigger mioty uplinks and receive
                           downlinks with the signal quality.
  2. Periodic covarage test: Device sends periodic uplinks and can also
                             send uplinks manually whose result is displayed.
  3. GPS Tracker: Device sends the GPS position periodically.
*/

#include "HardwareSerial.h"
#include "SoftwareSerial.h"
#include "Wire.h"
#include <Adafruit_SH110X.h>
#include <Adafruit_GPS.h>
#include "miotyAtClient.h"
#include <math.h>
#include <TimeLib.h>
#include <ADXL362.h>


#define BUTTON_A PB4
#define BUTTON_B PC0
#define BUTTON_C PC9

#define LED_RED PC8
#define LED_GREEN PC7
#define LED_BLUE PC6
#define LED_STATUS PC13

#define BUTTON_EXTENDER PB5

#define LED_RED_EXTENDER PC1
#define LED_GREEN_EXTENDER PC2
#define LED_BLUE_EXTENDER PC3

#define MIOTY_IRQ_PIN PB6

#define DISPLAY_UPDATE_PERIOD 1000  // 1s

#define BUTTON_DEBOUNCE_TIME 150  // ms

#define ARRAY_LENGTH(a) (sizeof(a) / sizeof(a[0]))

typedef enum {
  MODE_COVERAGE_MANUAL,
  MODE_COVERAGE_PERIODIC,
  MODE_GPS_TRACKER,
  MODE_NONE,
} appl_mode_t;

typedef struct {
  uint8_t second;
  uint8_t minute;
  uint8_t hour;
  uint8_t day;
  uint8_t month;
  uint16_t year;
} gps_time_t;

typedef struct {
  gps_time_t time;
  int32_t longitude;
  int32_t latitude;
  uint8_t accuracy;
  uint8_t speed;
  uint8_t sat_num;
  bool valid_b;
} position_t;

static void display_mioty_logo_f(void);
static void display_mode_selection_f(void);
static void update_display_f(void);
static void print_coverage_display_f(void);
static void print_gps_display_f(void);
static void update_gps_f(void);
static void send_uplink_manual_f(void);
static void send_uplink_periodic_f(void);
static void send_uplink_gps_f(void);
static void button_a_cb_f(void);
static void button_b_cb_f(void);
static void button_c_cb_f(void);
static void button_ext_cb_f(void);
static bool check_debounce_f(void);
static void blink_f(uint32_t led1, uint32_t led2, uint8_t n, uint16_t period);


static SoftwareSerial SerialMioty_S(PC11, PC10);
static HardwareSerial SerialMioty_H(PC11, PC10);
static Stream* SerialMioty;
static HardwareSerial SerialM3B(PA10, PA9);
static HardwareSerial SerialGPS(PA1, PA0);

static TwoWire Wire2(PB9, PB8);
static Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire2);

static ADXL362 adxl362;

static Adafruit_GPS GPS(&SerialGPS);

static appl_mode_t appl_mode = MODE_NONE;
static uint32_t last_tx_time = UINT32_MAX >> 1;  // send after startup
static uint32_t last_display_update = 1;
static uint32_t last_button_press;  // for button debounce

static const uint8_t power_settings_a[] = { 100, 60, 38, 21, 12 };
static const int8_t power_dbm_a[] = { 18, 14, 10, 5, 0 };
static const uint8_t ul_period_setting_a[]{ 15, 30, 60, 90 };

static bool transmission_active_b = false;
static bool is_ll_mode_b = false;
static uint8_t power_choice = 0;
static uint8_t ul_period_choice = 1;
static uint16_t periodic_msg_counter = 0;
static uint16_t manual_msg_counter = 0;
static bool send_uplink_b = false;
static bool ul_ack_b = false;
static float last_rssi = NAN;
static float last_snr = NAN;
static float last_eqsnr = NAN;
static time_t last_downlink_time = 0;
static position_t gps_pos;

void setup() {
  /* setup pins */
  pinMode(LED_STATUS, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_RED_EXTENDER, OUTPUT);
  pinMode(LED_GREEN_EXTENDER, OUTPUT);
  pinMode(LED_BLUE_EXTENDER, OUTPUT);
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);
  pinMode(BUTTON_EXTENDER, INPUT_PULLUP);
  pinMode(MIOTY_IRQ_PIN, INPUT_PULLUP);
  digitalWrite(LED_STATUS, 1);
  digitalWrite(LED_RED, 1);
  digitalWrite(LED_GREEN, 1);
  digitalWrite(LED_BLUE, 1);
  digitalWrite(LED_RED_EXTENDER, 1);
  digitalWrite(LED_GREEN_EXTENDER, 1);
  digitalWrite(LED_BLUE_EXTENDER, 1);

  Wire2.begin();
  display.begin(0x3C, true);
  display.setRotation(1);
  display.setTextColor(SH110X_WHITE);

  display_mioty_logo_f();
  delay(2000);

  display.setTextSize(1);

  /* select appl_mode */
  display_mode_selection_f();
  while (appl_mode == MODE_NONE) {
    if (digitalRead(BUTTON_A) == LOW || digitalRead(BUTTON_EXTENDER) == LOW) {
      appl_mode = MODE_COVERAGE_MANUAL;
    }
    if (digitalRead(BUTTON_B) == LOW) {
      appl_mode = MODE_COVERAGE_PERIODIC;
    }
    if (digitalRead(BUTTON_C) == LOW) {
      appl_mode = MODE_GPS_TRACKER;
    }
    delay(50);
  }

  /* setup Serials */
  if (appl_mode == MODE_GPS_TRACKER) {
    SerialMioty = &SerialMioty_S;
    SerialMioty_S.begin(9600);
  } else {
    SerialMioty = &SerialMioty_H;
    SerialMioty_H.begin(9600);
  }
  SerialM3B.begin(115200);

  /* reset the mioty module */
  uint32_t reset_time = millis();
  const char reset_cmd[] = "AT-RST\r";
  SerialMioty->write((uint8_t*)reset_cmd, 7);
  /* wait for IRQ pin to go low and then high again, indicating the module is ready, wait at most 1s */
  while ((digitalRead(MIOTY_IRQ_PIN) == 1) && ((millis() - reset_time) < 1000))
    ;
  while ((digitalRead(MIOTY_IRQ_PIN) == 0) && ((millis() - reset_time) < 1000))
    ;

  uint32_t mioty_mode;
  miotyAtClient_uplinkMode(&mioty_mode, false);
  is_ll_mode_b = (mioty_mode == 2);

  if (appl_mode == MODE_GPS_TRACKER) {
    display.clearDisplay();
    display.setCursor(24, 32);
    display.print("Starting GPS");
    display.display();
    GPS.begin(9600);
    delay(5000);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  }
  else
  {
    /* init acceleration sensor */
    adxl362.begin(PA8);
    adxl362.beginMeasure();
  }

  attachInterrupt(digitalPinToInterrupt(BUTTON_A), button_a_cb_f, FALLING);
  attachInterrupt(digitalPinToInterrupt(BUTTON_B), button_b_cb_f, FALLING);
  attachInterrupt(digitalPinToInterrupt(BUTTON_C), button_c_cb_f, FALLING);
  attachInterrupt(digitalPinToInterrupt(BUTTON_EXTENDER), button_ext_cb_f, FALLING);

  digitalWrite(LED_STATUS, 0);
  SerialM3B.println("");
  SerialM3B.println("M3B Tester Button Demo started");
}

void loop() {
  update_gps_f();
  uint32_t uptime = millis();
  if ((uptime - last_display_update) > DISPLAY_UPDATE_PERIOD) {
    update_display_f();
    last_display_update = uptime;
  }
  if (send_uplink_b) {
    send_uplink_manual_f();
    send_uplink_b = false;
  }
  if ((appl_mode == MODE_COVERAGE_PERIODIC) && ((uptime - last_tx_time) > (ul_period_setting_a[ul_period_choice] * 1000))) {
    send_uplink_periodic_f();
    last_tx_time = uptime;
  }
  if ((appl_mode == MODE_GPS_TRACKER) && ((uptime - last_tx_time) > (ul_period_setting_a[ul_period_choice] * 1000))) {
    send_uplink_gps_f();
    last_tx_time = uptime;
  }
  if ((appl_mode == MODE_GPS_TRACKER) && ((uptime - last_tx_time) > 4000) && transmission_active_b) {
    digitalWrite(LED_BLUE, 1);
    digitalWrite(LED_BLUE_EXTENDER, 1);
    transmission_active_b = false;
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

static void display_mode_selection_f(void) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Select Mode:");
  display.println("\nA: Coverage manual");
  display.println("\nB: Coverage periodic");
  display.println("\nC: GPS tracker");
  display.display();
}

static void update_display_f(void) {
  if (appl_mode == MODE_GPS_TRACKER) {
    print_gps_display_f();
  } else {
    print_coverage_display_f();
  }
  display.display();
}

static void print_coverage_display_f(void) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Coverage Test: ");
  display.println((appl_mode == MODE_COVERAGE_MANUAL) ? "manual" : "per.");
  display.print("ACK RX: ");
  display.print(ul_ack_b ? "yes (" : "no  (");
  display.print(manual_msg_counter);
  display.println(")");
  display.print("RSSI:    ");
  if (isnan(last_rssi)) {
    display.println(" -");
  } else {
    display.print(last_rssi, 1);
    display.println(" dbm");
  }
  display.print("SNR:      ");
  if (isnan(last_snr)) {
    display.println("-");
  } else {
    display.print(last_snr, 1);
    display.println(" db");
  }
  display.print("eqSNR:    ");
  if (isnan(last_eqsnr)) {
    display.println("-");
  } else {
    display.print(last_eqsnr, 1);
    display.println(" db");
  }
  if (appl_mode == MODE_COVERAGE_MANUAL) {
    if (last_downlink_time) {
      char line[22 + 1];
      snprintf(line, sizeof(line), "last ACK: %02u:%02u:%02u", hour(last_downlink_time),
               minute(last_downlink_time),
               second(last_downlink_time));
      display.println(line);
    } else {
      display.println("");
    }
  }
  if (appl_mode == MODE_COVERAGE_PERIODIC) {
    display.print("A: UL period: ");
    display.print(ul_period_setting_a[ul_period_choice]);
    display.println("s");
  }
  display.print("B: Mode: ");
  display.println(is_ll_mode_b ? "Low latency" : "Standard");
  display.print("C: Power ");
  display.print(power_dbm_a[power_choice]);
  display.println(" dbm");
}

static void print_gps_display_f(void) {
  display.clearDisplay();
  display.setCursor(0, 0);
  char line[26];
  snprintf(line, sizeof(line), "%02u:%02u:%02u %02u.%02u.%02u",
           gps_pos.time.hour, gps_pos.time.minute, gps_pos.time.second,
           gps_pos.time.day, gps_pos.time.month, gps_pos.time.year);
  display.println(line);
  display.print("Satellites: ");
  display.println(gps_pos.sat_num);
  if (gps_pos.valid_b) {
    display.print(((float)gps_pos.longitude) / 10000000, 7);
    display.print("/");
    display.println(((float)gps_pos.latitude) / 10000000, 7);
    display.print("Accuracy: ");
    display.println(gps_pos.accuracy);
    display.print("Speed: ");
    display.print(gps_pos.speed);
    display.println(" km/h");
  } else {
    display.println("  --/--\n");
  }
  display.print("Msg nr: ");
  display.println(periodic_msg_counter);
  display.print("A: UL period: ");
  display.print(ul_period_setting_a[ul_period_choice]);
  display.println("s");
  display.display();
}

static void update_gps_f(void) {
  GPS.read();
  if (GPS.newNMEAreceived()) {
    if (GPS.parse(GPS.lastNMEA())) {
      gps_pos.time.second = GPS.seconds;
      gps_pos.time.minute = GPS.minute;
      gps_pos.time.hour = GPS.hour;
      gps_pos.time.day = GPS.day;
      gps_pos.time.month = GPS.month;
      gps_pos.time.year = GPS.year;
      gps_pos.sat_num = GPS.satellites;
      if (GPS.fix) {
        gps_pos.latitude = GPS.latitude_fixed;
        gps_pos.longitude = GPS.longitude_fixed;
        gps_pos.accuracy = (uint8_t)GPS.HDOP;
        gps_pos.speed = (uint8_t)(GPS.speed * 1.852f);  // convert knots to km/h
        gps_pos.valid_b = true;
      } else {
        gps_pos.valid_b = false;
      }
    }
  }
}

static void send_uplink_manual_f(void) {
  digitalWrite(LED_BLUE, 0);
  digitalWrite(LED_BLUE_EXTENDER, 0);
  int16_t temp, acc_x, acc_y, acc_z;
  adxl362.readXYZTData(acc_x, acc_y, acc_z, temp);
  transmission_active_b = true;
  manual_msg_counter++;
  uint8_t ul[10];
  ul[0] = (manual_msg_counter >> 8) & 0xFF;
  ul[1] = manual_msg_counter & 0xFF;
  ul[2] = acc_x >> 8;
  ul[3] = acc_x;
  ul[4] = acc_y >> 8;
  ul[5] = acc_y;
  ul[6] = acc_z >> 8;
  ul[7] = acc_z;
  ul[8] = 0;  // Battery voltage, not measured on HW index A
  ul[9] = 0;
  uint8_t dl[100];
  dl[6] = 0;
  uint8_t dl_size = sizeof(dl);
  uint32_t pcnt;
  SerialM3B.println("Sending uplink...");
  /* BMPF does not work correctly */
  miotyAtClient_returnCode r = miotyAtClient_sendMessageBidi(ul, sizeof(ul), dl, &dl_size, &pcnt);
  if (r == MIOTYATCLIENT_RETURN_CODE_OK) {
    ul_ack_b = true;
    /* check if dl data was set, use MSB of timstamp, it should never be 0 */
    if (dl[6] != 0) {
      int16_t rssi = (dl[0] << 8) + dl[1];
      last_rssi = ((float)rssi) / 10;
      int16_t snr = (dl[2] << 8) + dl[3];
      last_snr = ((float)snr) / 10;
      int16_t eq_snr = (dl[4] << 8) + dl[5];
      last_eqsnr = ((float)eq_snr) / 10;
      last_downlink_time = (dl[6] << 24) + (dl[7] << 16) + (dl[8] << 8) + dl[9];
    }
  } else {
    ul_ack_b = false;
    last_rssi = NAN;
    last_snr = NAN;
    last_eqsnr = NAN;
  }
  digitalWrite(LED_BLUE, 1);
  digitalWrite(LED_BLUE_EXTENDER, 1);
  transmission_active_b = false;
  if (ul_ack_b) {
    blink_f(LED_GREEN, LED_GREEN_EXTENDER, 3, 200);
  } else {
    blink_f(LED_RED, LED_RED_EXTENDER, 3, 200);
  }
}

static void send_uplink_periodic_f(void) {
  digitalWrite(LED_BLUE, 0);
  digitalWrite(LED_BLUE_EXTENDER, 0);
  transmission_active_b = true;
  periodic_msg_counter++;
  uint32_t pcnt;
  uint8_t msg[5] = { 194 };
  msg[1] = (periodic_msg_counter >> 8) & 0xFF;
  msg[2] = periodic_msg_counter & 0xFF;
  msg[3] = 0;  // Battery voltage, not measured on HW index A
  msg[4] = 0;
  SerialM3B.println("Sending uplink...");
  miotyAtClient_sendMessageUniMPF(msg, sizeof(msg), &pcnt);
  digitalWrite(LED_BLUE, 1);
  digitalWrite(LED_BLUE_EXTENDER, 1);
  transmission_active_b = false;
}

static void send_uplink_gps_f(void) {
  digitalWrite(LED_BLUE, 0);
  digitalWrite(LED_BLUE_EXTENDER, 0);
  transmission_active_b = true;
  periodic_msg_counter++;
  uint8_t msg[16] = { 193, 0 };
  uint8_t length = 6;
  msg[1] = 0;  // Battery voltage, not measured on HW index A
  msg[2] = 0;
  msg[3] = gps_pos.valid_b << 7;
  msg[3] += gps_pos.sat_num;
  msg[4] = (periodic_msg_counter >> 8) & 0xFF;
  msg[5] = periodic_msg_counter & 0xFF;
  if (gps_pos.valid_b) {
    msg[6] = (gps_pos.longitude >> 24) & 0xFF;
    msg[7] = (gps_pos.longitude >> 16) & 0xFF;
    msg[8] = (gps_pos.longitude >> 8) & 0xFF;
    msg[9] = (gps_pos.longitude >> 0) & 0xFF;
    msg[10] = (gps_pos.latitude >> 24) & 0xFF;
    msg[11] = (gps_pos.latitude >> 16) & 0xFF;
    msg[12] = (gps_pos.latitude >> 8) & 0xFF;
    msg[13] = (gps_pos.latitude >> 0) & 0xFF;
    msg[14] = gps_pos.accuracy;
    msg[15] = gps_pos.speed;
    length = 16;
  }
  SerialM3B.println("Sending uplink...");
  /* 
    the atClient does not work reliably with SoftwareSerial, sometimes it
    misses return values causing it to block indefinitely.
    -> create AT command ourself as we do not need the returned values in GPS mode.
    This has the disadvantage that we will not detect errors.
  */
  char cmd[42];
  uint8_t cmd_len = snprintf(cmd, sizeof(cmd), "AT-UMPF=%ux", length);
  for (int i = 0; i < length; i++) {
    cmd_len += snprintf(&cmd[cmd_len], 3, "%02X", msg[i]);
  }
  strncpy(&cmd[cmd_len], ".\r", 3);
  cmd_len += 2;
  SerialM3B.write(cmd, cmd_len);
  SerialM3B.println("");
  SerialMioty->write(cmd, cmd_len);
}

static void button_a_cb_f(void) {
  if (!check_debounce_f()) {
    return;
  }
  if ((appl_mode == MODE_COVERAGE_PERIODIC) || (appl_mode == MODE_GPS_TRACKER)) {
    ul_period_choice = (ul_period_choice + 1) % ARRAY_LENGTH(ul_period_setting_a);
    update_display_f();
  }
}

static void button_b_cb_f(void) {
  if (!check_debounce_f()) {
    return;
  }
  if (appl_mode != MODE_GPS_TRACKER) {
    if (transmission_active_b) {
      return;
    }
    is_ll_mode_b = !is_ll_mode_b;
    uint32_t mode = is_ll_mode_b ? 2 : 0;
    miotyAtClient_uplinkMode(&mode, true);
    update_display_f();
  }
}

static void button_c_cb_f(void) {
  if (!check_debounce_f()) {
    return;
  }
  if (transmission_active_b) {
    return;
  }
  if (appl_mode != MODE_GPS_TRACKER) {
    power_choice = (power_choice + 1) % ARRAY_LENGTH(power_settings_a);
    uint32_t power = power_settings_a[power_choice];
    miotyAtClient_getOrSetTransmitPower(&power, true);
    update_display_f();
  }
}

static void button_ext_cb_f(void) {
  if (!check_debounce_f()) {
    return;
  }
  if (appl_mode != MODE_GPS_TRACKER) {
    /* trigger uplink */
    send_uplink_b = true;
  }
}

static bool check_debounce_f(void) {
  if ((millis() - last_button_press) > BUTTON_DEBOUNCE_TIME) {
    last_button_press = millis();
    return true;
  }
  return false;
}

static void blink_f(uint32_t led1, uint32_t led2, uint8_t n, uint16_t period) {
  for (; n; n--) {
    digitalWrite(led1, 1);
    digitalWrite(led2, 1);
    delay(period >> 1);
    digitalWrite(led1, 0);
    digitalWrite(led2, 0);
    delay(period >> 1);
  }
  digitalWrite(led1, 1);
  digitalWrite(led2, 1);
}

void miotyAtClientWrite(uint8_t* msg, uint16_t len) {
  SerialMioty->write(msg, len);
  SerialM3B.println("");
  SerialM3B.write(msg, len);
  SerialM3B.println("");
}

bool miotyAtClientRead(uint8_t* buf, uint8_t* p_len) {
  int i = 0;
  while (SerialMioty->available() > 0) {
    buf[i++] = SerialMioty->read();
  }
  SerialM3B.write(buf, i);
  *p_len = i;
  return true;
}
