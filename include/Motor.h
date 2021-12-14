#include <Arduino.h>

class Motor
{
private:
  uint8_t pwm_pin;
  uint8_t in1_pin;
  uint8_t in2_pin;

public:
  Motor(uint8_t pwm_pin, uint8_t in1_pin, uint8_t in2_pin);
  void setup();
  void run(uint8_t speed, bool forward);
  void stop(bool freewheel);
};
