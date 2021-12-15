#include "Motor.h"

Motor::Motor(uint8_t pwm, uint8_t in1, uint8_t in2)
{
  pwm_pin = pwm;
  in1_pin = in1;
  in2_pin = in2;
}

void Motor::setup()
{
  pinMode(pwm_pin, OUTPUT);
  analogWrite(pwm_pin, 0);
  pinMode(in1_pin, OUTPUT);
  digitalWrite(in1_pin, LOW);
  pinMode(in2_pin, OUTPUT);
  digitalWrite(in2_pin, LOW);
}

void Motor::run(uint8_t speed, bool forward)
{
  if (speed == 0) {
    return stop(false);
  }

  digitalWrite(in1_pin, forward ? HIGH : LOW);
  digitalWrite(in2_pin, forward ? LOW : HIGH);
  analogWrite(pwm_pin, speed);
}

void Motor::stop(bool freewheel)
{
  analogWrite(pwm_pin, 0);

  digitalWrite(in1_pin, freewheel ? LOW : HIGH);
  digitalWrite(in2_pin, freewheel ? LOW : HIGH);  
}