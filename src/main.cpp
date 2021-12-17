#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "Motor.h"

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

/*
Generic motor control (full speed). Call every 10us for good results.
*/
void mot_control(Motor mot1, Motor mot2, int32_t pos, int32_t aim) {
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

void setup() {
  Serial.begin(115200);

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
}

void loop() {
  // led_front.setPixelColor(led_n, 0, 0, 0, led_on ? 255 : 0);
  uint32_t c = led_back.Color((color == 0 || color == 1 || color == 2 || color == 9) ? brightness : 0,
                               (color == 2 || color == 3 || color == 4 || color == 8) ? brightness : 0,
                               (color == 4 || color == 5 || color == 6 || color == 9) ? brightness : 0,
                               (color == 6 || color == 7 || color == 0 || color == 8) ? brightness : 0);
  if (back) {
    led_back.fill(c);
    led_back.show();
  } else {
    led_front.fill(c);
    led_front.show();
  }
  delay(10);

  if (led_on && (brightness < 255)) {
    brightness++;
  } else if (!led_on && (brightness > 0)) {
    brightness--;
  } else {
    if (!led_on) {
      if (color < 9) color++;
      else {
        color = 0;
        back = !back;
      }
    }
    led_on = !led_on;
  }

  if (brightness % 8 == 0) {
    Serial.printf("hor_pos %i | \t vert_pos %i \n", hor_pos, vert_pos);
  }
}