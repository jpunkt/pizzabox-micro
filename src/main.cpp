#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <SimpleSerialProtocol.h>
#include "Commands.h"
#include "Motor.h"
// #include "Serial_Comm.h"

// Vertical motor top
#define VERT_UP_PWM   3
#define VERT_UP_AIN2  4
#define VERT_UP_AIN1  5

// Vertical motor bottom
#define VERT_DOWN_PWM   9
#define VERT_DOWN_AIN2  7
#define VERT_DOWN_AIN1  8

// Horizontal motor left
#define HORZ_LEFT_PWM  24
#define HORZ_LEFT_AIN2 25
#define HORZ_LEFT_AIN1 26

// Horizontal motor right
#define HORZ_RIGHT_PWM  29
#define HORZ_RIGHT_AIN2 27
#define HORZ_RIGHT_AIN1 28

// Vertical sensors
#define VERT_END_OUTER  40
#define VERT_END_INNER  39
#define VERT_CNT_OUTER  38
#define VERT_CNT_INNER  37

// Horizontal sensors
#define HORZ_END_OUTER  33
#define HORZ_END_INNER  34
#define HORZ_CNT_INNER  35
#define HORZ_CNT_OUTER  36

// Lights
#define LED_FRONT 41
#define LED_COUNT_FRONT 26

#define LED_BACK  14
#define LED_COUNT_BACK 72

void on_serial_error(uint8_t errorNum);

volatile int32_t hor_pos;
volatile int32_t vert_pos;

int32_t hor_aim;
int32_t vert_aim;

bool up;

Motor vert_up(VERT_UP_PWM, VERT_UP_AIN1, VERT_UP_AIN2);
Motor vert_down(VERT_DOWN_PWM, VERT_DOWN_AIN1, VERT_DOWN_AIN2);
Motor horz_left(HORZ_LEFT_PWM, HORZ_LEFT_AIN1, HORZ_LEFT_AIN2);
Motor horz_right(HORZ_RIGHT_PWM, HORZ_RIGHT_AIN1, HORZ_RIGHT_AIN2);

Adafruit_NeoPixel led_front(LED_COUNT_FRONT, LED_FRONT, NEO_GBRW + NEO_KHZ800);
Adafruit_NeoPixel led_back(LED_COUNT_BACK, LED_BACK, NEO_GBRW + NEO_KHZ800);

bool back;
bool led_on;
int led_n;
u_int8_t brightness;
u_int8_t color;

bool serial_connected;

// inintialize hardware constants
const long BAUDRATE = 115200; // speed of serial connection
const long CHARACTER_TIMEOUT = 500; // wait max 500 ms between single chars to be received

// Create instance. Pass Serial instance. Define command-id-range within Simple Serial Protocol is listening (here: a - z)
SimpleSerialProtocol ssp(Serial1, BAUDRATE, CHARACTER_TIMEOUT, on_serial_error, 0, 'Z'); // ASCII: 'a' - 'z' (26 byes of RAM is reserved)

int32_t count(int pinA, int pinB) {
  if (digitalRead(pinA)) return digitalRead(pinB) ? -1 : 1;
  else return digitalRead(pinB) ? 1 : -1;
}

void hor_count() {
  hor_pos += count(HORZ_CNT_INNER, HORZ_CNT_OUTER);
}

void vert_count() {
  vert_pos += count(VERT_CNT_INNER, VERT_CNT_OUTER);
}

/**
 * @brief Generic motor control (full speed). Call every 10us for good results.
 * 
 */
void mot_control(Motor &mot1, Motor &mot2, int32_t pos, int32_t aim) {
  if (pos < aim) {
    mot1.run(255, false);
    mot2.run(127, false);
  } else if (vert_pos > vert_aim) {
    mot2.run(255, true);
    mot1.run(127, true);
  } else {
    mot1.stop(false);
    mot2.stop(false);
    // vert_aim = (vert_aim == 50) ? 0 : 50;
  }
}

uint8_t color_value(uint8_t col_from, uint8_t col_to, uint32_t cur_time, uint32_t duration) {
  float_t perc = (float) cur_time / (float) duration;
  float_t col = (float) (col_to - col_from) * perc;
  return (uint8_t) (col + 0.5);
}

void led_fade(Adafruit_NeoPixel &led, int8_t to_R, int8_t to_G, int8_t to_B, int8_t to_W, uint32_t time_ms) {
  uint32_t startcol = led.getPixelColor(0);
  Serial.printf("col = %i \n", startcol);
  uint8_t from_W = (startcol & 0xff000000) >> 24;
  uint8_t from_R = (startcol & 0x00ff0000) >> 16;
  uint8_t from_G = (startcol & 0x0000ff00) >> 8;
  uint8_t from_B = (startcol & 0x000000ff);

  Serial.printf("r = %i, g = %i, b = %i, w = %i \n", from_R, from_G, from_B, from_W);

  uint32_t start_time = millis();
  uint32_t end_time = start_time + time_ms;

  while (millis() < end_time) {
    u_int32_t cur_time = millis() - start_time;
    uint32_t color = led.Color(color_value(from_R, to_R, cur_time, time_ms),
                               color_value(from_G, to_G, cur_time, time_ms),
                               color_value(from_B, to_B, cur_time, time_ms),
                               color_value(from_W, to_W, cur_time, time_ms));
    led.fill(color);
    led.show();
    // Serial.printf("t = %i, c = %i \n", cur_time, color);
  }
}

void on_serial_error(uint8_t errno) {
  Serial.printf("SSP error %i \n", errno);
  ssp.writeCommand(ERROR);
  ssp.writeInt8(errno);
  ssp.writeEot();
}

void serial_received() {
  ssp.writeCommand(RECEIVED);
  ssp.writeEot();
}

void serial_hello() {
  ssp.readEot();

  if (!serial_connected) {
      ssp.writeCommand(HELLO);
      serial_connected = true;
      Serial.println("Connection established.");
  } 
  else {
    ssp.writeCommand(ALREADY_CONNECTED);
    Serial.println("Handshake complete.");
  }

  ssp.writeEot();
}

void serial_backlight() {
  uint8_t r = ssp.readUnsignedInt8();
  uint8_t g = ssp.readUnsignedInt8();
  uint8_t b = ssp.readUnsignedInt8();
  uint8_t w = ssp.readUnsignedInt8();
  uint32_t t = ssp.readUnsignedInt32();
  ssp.readEot();
  Serial.printf("Received BACKLIGHT (%i, %i, %i, %i, %i) \n", r, g, b, w, t);
  led_fade(led_back, r, g, b, w, t);
  serial_received();
}

void serial_frontlight() {
  ssp.readEot();
  Serial.println("Received FRONTLIGHT");
  serial_received();
}

void serial_motor_v() {
  ssp.readEot();
  Serial.println("Received MOTOR_V");
  serial_received();
}

void serial_motor_h() {
  ssp.readEot();
  Serial.println("Received MOTOR_H");
  serial_received();
}

void serial_record() {
  ssp.readEot();
  Serial.println("Received RECORD");
  serial_received();
}

void serial_rewind() {
  ssp.readEot();
  Serial.println("Received REWIND");
  serial_received();
}

void serial_userinteract() {
  ssp.readEot();
  Serial.println("Received USER_INTERACT");
  serial_received();
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);

  hor_pos = 0;
  vert_pos = 0;

  hor_aim = 0;
  vert_aim = -20;

  vert_up.setup();
  vert_down.setup();
  vert_up.stop(true);
  vert_down.stop(true);

  horz_left.setup();
  horz_right.setup();
  horz_left.stop(true);
  horz_right.stop(true);

  pinMode(HORZ_CNT_INNER, INPUT);
  pinMode(HORZ_CNT_OUTER, INPUT);
  pinMode(VERT_CNT_INNER, INPUT);
  pinMode(VERT_CNT_OUTER, INPUT);

  digitalWrite(HORZ_CNT_INNER, LOW);
  digitalWrite(HORZ_CNT_OUTER, LOW);
  digitalWrite(VERT_CNT_INNER, LOW);
  digitalWrite(VERT_CNT_OUTER, LOW);
  
  attachInterrupt(digitalPinToInterrupt(HORZ_CNT_INNER), hor_count, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(HORZ_CNT_OUTER), hor_count, CHANGE);
  attachInterrupt(digitalPinToInterrupt(VERT_CNT_INNER), vert_count, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(VERT_CNT_OUTER), vert_count, CHANGE);

  back = true;
  led_on = true;
  
  brightness = 0;
  color = 0;

  led_back.begin();
  led_back.show();

  led_front.begin();
  led_front.show();

  serial_connected = false;

  ssp.init();
  ssp.registerCommand(HELLO, serial_hello);
  ssp.registerCommand(ALREADY_CONNECTED, serial_hello);
  ssp.registerCommand(BACKLIGHT, serial_backlight);
  ssp.registerCommand(FRONTLIGHT, serial_frontlight);
  ssp.registerCommand(MOTOR_H, serial_motor_h);
  ssp.registerCommand(MOTOR_V, serial_motor_v);
  ssp.registerCommand(RECORD, serial_record);
  ssp.registerCommand(REWIND, serial_rewind);
  ssp.registerCommand(USER_INTERACT, serial_userinteract);
}

void loop() {
  ssp.loop();
}