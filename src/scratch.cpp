#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// Lights
#define LED_FRONT 41
#define LED_COUNT_FRONT 26

#define LED_BACK  14
#define LED_COUNT_BACK 72

Adafruit_NeoPixel led_front(LED_COUNT_FRONT, LED_FRONT, NEO_GBRW + NEO_KHZ800);
Adafruit_NeoPixel led_back(LED_COUNT_BACK, LED_BACK, NEO_GBRW + NEO_KHZ800);

bool back;
bool led_on;
int led_n;
u_int8_t brightness;
u_int8_t color;

void led_loop() {
  // LED test loop, for reference..

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
}