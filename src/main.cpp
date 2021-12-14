#include <Arduino.h>
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

volatile int32_t hor_pos;
volatile int32_t vert_pos;

Motor vert_up(VERT_UP_PWM, VERT_UP_AIN1, VERT_UP_AIN2);
Motor vert_down(VERT_DOWN_PWM, VERT_DOWN_AIN1, VERT_DOWN_AIN2);
Motor horz_left(HORZ_LEFT_PWM, HORZ_LEFT_AIN1, HORZ_LEFT_AIN2);
Motor horz_right(HORZ_RIGHT_PWM, HORZ_RIGHT_AIN1, HORZ_RIGHT_AIN2);

int32_t count(int pinA, int pinB) {
  if (digitalRead(pinA)) return digitalRead(pinB) ? 1 : -1;
  else return digitalRead(pinB) ? -1 : 1;
}

void hor_count() {
  hor_pos += count(HORZ_CNT_INNER, HORZ_CNT_OUTER);
}

void vert_count() {
  vert_pos += count(VERT_CNT_INNER, VERT_CNT_OUTER);
}

void setup() {
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);

  hor_pos = 0;
  vert_pos = 0;

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
}

void loop() {
  // Serial.printf("hor_count1: %i | \t hor_count2: %i | \t vert_count1: %i | \t vert_count2: %i \n", digitalRead(HORZ_CNT_INNER), digitalRead(HORZ_CNT_OUTER), digitalRead(VERT_CNT_INNER), digitalRead(VERT_CNT_OUTER));

  Serial.printf("hor_count %i | \t vert_count %i \n", hor_pos, vert_pos);

  delay(500);
}