#include <Arduino.h>
#include <Bounce2.h>
#include <Adafruit_NeoPixel.h>
#include <SimpleSerialProtocol.h>
#include <StateMachine.h>
#include "Config.h"
#include "Commands.h"
#include "States.h"
#include "Motor.h"

// Statemachine setup
StateMachine sm = StateMachine();

State* S00 = sm.addState(&state_post);
State* S10 = sm.addState(&state_zero);
State* S20 = sm.addState(&state_init_callbacks);
State* S30 = sm.addState(&state_wait_serial);
State* S40 = sm.addState(&state_serial_com);
State* SER = sm.addState(&state_error);

/*-------- Variables --------*/
// Convenience array of all UI LEDs
const int UI_LED_PINS[] = {LED_BUILTIN, BTN_LED_BLUE, BTN_LED_RED, BTN_LED_YELLOW, BTN_LED_GREEN};
#define N_LEDS (sizeof(UI_LED_PINS) / sizeof(UI_LED_PINS[0]))

// Heartbeat blinker timer
elapsedMillis blink_time;
bool blink_status;             // boolean to hold led status (needed to let more than one led blink)

// Statemachine booleans
bool handshake_complete;

// Position counters
volatile int16_t hor_pos;
volatile int16_t vert_pos;

// Last change on position counters in millis
elapsedMillis hor_lastchange;
elapsedMillis vert_lastchange;

/*-------- Objects --------*/
// Motors
Motor vert_up(VERT_UP_PWM, VERT_UP_AIN1, VERT_UP_AIN2);
Motor vert_down(VERT_DOWN_PWM, VERT_DOWN_AIN1, VERT_DOWN_AIN2);
Motor horz_left(HORZ_LEFT_PWM, HORZ_LEFT_AIN1, HORZ_LEFT_AIN2);
Motor horz_right(HORZ_RIGHT_PWM, HORZ_RIGHT_AIN1, HORZ_RIGHT_AIN2);

// LEDs
Adafruit_NeoPixel led_front(LED_COUNT_FRONT, LED_FRONT, NEO_GBRW + NEO_KHZ800);
Adafruit_NeoPixel led_back(LED_COUNT_BACK, LED_BACK, NEO_GBRW + NEO_KHZ800);

// Buttons
Bounce2::Button btn_blue   = Bounce2::Button();
Bounce2::Button btn_red    = Bounce2::Button();
Bounce2::Button btn_yellow = Bounce2::Button();
Bounce2::Button btn_green  = Bounce2::Button();

/*-------- Serial Communication --------*/
// Error handler
void serial_on_error(uint8_t errorNum);

// serial handshake was performed
bool serial_connected;

// inintialize hardware constants
const long BAUDRATE = 115200; // speed of serial connection
const long CHARACTER_TIMEOUT = 500; // wait max 500 ms between single chars to be received

// Create instance. Pass Serial instance. Define command-id-range within Simple Serial Protocol is listening (here: a - z)
SimpleSerialProtocol ssp(Serial1, BAUDRATE, CHARACTER_TIMEOUT, serial_on_error, 0, 'Z'); // ASCII: 'a' - 'z' (26 byes of RAM is reserved)

/**
 * @brief Generic encoder logic for callbacks
 * 
 * @param pinA 
 * @param pinB 
 * @return int32_t 
 */
int32_t count(int pinA, int pinB) {
  if (digitalRead(pinA)) return digitalRead(pinB) ? -1 : 1;
  else return digitalRead(pinB) ? 1 : -1;
}

/**
 * @brief Callback for horizontal counting
 * 
 */
void hor_count() {
  if (!digitalRead(HORZ_END_OUTER)) {
    hor_pos -= count(HORZ_CNT_INNER, HORZ_CNT_OUTER);
  } else {
    hor_pos = 0;
  }
  hor_lastchange = 0;
}

/**
 * @brief Callback for vertical counting
 * 
 */
void vert_count() {
  if (!digitalRead(VERT_END_INNER)) {
    vert_pos -= count(VERT_CNT_INNER, VERT_CNT_OUTER);
  } else {
    vert_pos = 0;
  }
  vert_lastchange = 0;
}

/**
 * @brief Blink the internal LED with defined on- and off- times. Call in loop to blink.
 * 
 * @param on_interval  time LED stays on in millis 
 * @param off_interval time LED is off in millis
 */
template <size_t N>
void blink(int (&led_pin)[N], uint32_t on_interval, uint32_t off_interval) {
  if (blink_status) {
    if (blink_time >= on_interval) {
      blink_status = false;
      for (const int &led : led_pin)
        digitalWrite(led, LOW);
      blink_time = blink_time - on_interval;
    }
  } else {
    if (blink_time >= off_interval) {
      blink_status = true;
      for (const int &led : led_pin) 
        digitalWrite(led, HIGH);
      blink_time = blink_time - off_interval;
    }
  }
}

/**
 * @brief Generic scroll zeroing code
 * 
 * zero_pin   | end_pin   | Condition -> Action
 * -----------+-----------+---------------------------
 * 0          | 0         | scroll between start & end -> rewind
 * 0          | 1         | scroll at end -> rewind
 * 1          | 0         | scroll zeroed -> do nothing
 * 1          | 1         | scroll at start end-stop -> forward until end-stop free
 * 
 * @param mot1       Motor in positive direction
 * @param mot2       Motor in negative direction
 * @param zero_pin   Sensor pin where LOW enables count
 * @param end_pin    Sensor pin attached to emergency stop (end-stop)
 */
void zero_motor(Motor &mot1, Motor &mot2, int zero_pin, int end_pin) {
  // elapsedMillis time = 0;
  if (!digitalRead(zero_pin)) {
    // rewind
    while (!digitalRead(zero_pin)) {
      mot2.run(255, false);
      mot1.run(127, false);
    }
    mot1.stop(false);
    delay(200);
    mot2.stop(false);
    delay(100);
    mot1.stop(true);
    mot2.stop(true);
  } else if (digitalRead(end_pin)) {
    // move forward
    while(digitalRead(zero_pin)) {
      mot1.run(127, true);
      mot2.run(55, true);
    }
    mot2.stop(false);
    delay(200);
    mot1.stop(false);
    delay(100);
    mot1.stop(true);
    mot2.stop(true);
  }
}

/**
 * @brief Serial communication error handler
 * 
 * @param errno 
 */
void serial_on_error(uint8_t errno) {
  Serial.printf("SSP error %i \n", errno);
  ssp.writeCommand(ERROR);
  ssp.writeInt8(errno);
  ssp.writeEot();
  sm.transitionTo(SER);
}

/**
 * @brief Send RECEIVED+EOT bytes over Serial
 * 
 */
void serial_received() {
  ssp.writeCommand(RECEIVED);
  ssp.writeEot();
}

void serial_received(uint8_t response) {
  ssp.writeCommand(RECEIVED);
  ssp.writeUnsignedInt8(response);
  ssp.writeEot();
}

/**
 * @brief Helper function to calculate transition between two colors
 * 
 * @param col_from  8bit color value start
 * @param col_to    8bit color value end 
 * @param perc      percentage in float
 * @return uint8_t  8bit color value at percentage of transition
 */
uint8_t color_value(uint8_t col_from, uint8_t col_to, float_t perc) {
  float_t col;
  if (col_from < col_to) {
    col = col_from + (float) (col_to - col_from) * perc;
  } else {
    col = col_from - (float) (col_from - col_to) * perc;
  }
  return (uint8_t) (col + 0.5);
}

/**
 * @brief Generic fade LEDs from one color to another
 * 
 * @param led 
 */
void serial_led_fade(Adafruit_NeoPixel &led) {
  uint8_t to_R = ssp.readUnsignedInt8();
  uint8_t to_G = ssp.readUnsignedInt8();
  uint8_t to_B = ssp.readUnsignedInt8();
  uint8_t to_W = ssp.readUnsignedInt8();
  uint32_t time_ms = ssp.readUnsignedInt32();
  ssp.readEot();
  Serial.printf("Received BACKLIGHT (%i, %i, %i, %i, %i) \n", to_R, to_G, to_B, to_W, time_ms);

  uint32_t startcol = led.getPixelColor(0);
  Serial.printf("col = %i \n", startcol);
  uint8_t from_W = (startcol & 0xff000000) >> 24;
  uint8_t from_R = (startcol & 0x00ff0000) >> 16;
  uint8_t from_G = (startcol & 0x0000ff00) >> 8;
  uint8_t from_B = (startcol & 0x000000ff);

  Serial.printf("r = %i, g = %i, b = %i, w = %i \n", from_R, from_G, from_B, from_W);

  elapsedMillis t = 0;

  while (t < time_ms) {
    float_t perc = (float) t / (float) time_ms;
    uint32_t color = led.Color(color_value(from_R, to_R, perc),
                               color_value(from_G, to_G, perc),
                               color_value(from_B, to_B, perc),
                               color_value(from_W, to_W, perc));
    led.fill(color);
    led.show();
  }

  led.fill(led.Color(to_R, to_G, to_B, to_W));
  led.show();

  serial_received();
}

/**
 * @brief Generic motor control (full speed). Call every 10us for good results.
 * 
 */
bool mot_control(Motor &mot1, Motor &mot2, volatile int16_t &pos, int16_t &aim) {
  if (pos < aim) {
    mot1.run(255, true);
    mot2.run(127, true);
    return false;
  } else if (pos > aim) {
    mot2.run(255, false);
    mot1.run(127, false);
    return false;
  } else {
    mot1.stop(false);
    mot2.stop(false);
    return true;
  }
}

/**
 * @brief Generic check endstop and stop motor
 * 
 * @param mot1 
 * @param mot2 
 * @param outer_pin 
 */
bool mot_stop(Motor &mot1, Motor &mot2, int outer_pin) {
  if (digitalRead(outer_pin)) {
    mot1.stop(true);
    mot2.stop(true);
    Serial.printf("Motor stopped. \n");
    return true;
  }
  return false;
}

/**
 * @brief Generic serial command handler to drive scroll to position
 * 
 * @param mot1  Motor in positive direction
 * @param mot2  Motor in negative direction
 * @param pos   position variable
 */
void serial_motor(Motor &mot1, Motor &mot2, volatile int16_t &pos, int end_pin) {
  int16_t inc = ssp.readInt16();
  ssp.readEot();

  int16_t aim = pos + inc;
  int16_t c = 0;
  while (!mot_control(mot1, mot2, pos, aim)) {
    if (c < ENDSTOP_OVERRIDE) {
      c++;
    } else {
      if (mot_stop(mot1, mot2, end_pin)) {
        break;
      }
    }
    delayMicroseconds(10);
  }
}

/**
 * @brief Serial command handler for handshake (responds to HELLO and ALREADY_CONNECTED)
 * 
 */
void serial_hello() {
  ssp.readEot();

  if (!serial_connected) {
      ssp.writeCommand(HELLO);
      serial_connected = true;
      Serial.println("Connection established.");
  } 
  else {
    ssp.writeCommand(ALREADY_CONNECTED);
    handshake_complete = true;
    Serial.println("Handshake complete.");
  }

  ssp.writeEot();
}

/**
 * @brief Serial command handler for BACKLIGHT
 * 
 */
void serial_backlight() {
  Serial.println("Received BACKLIGHT");
  serial_led_fade(led_back);
}

/**
 * @brief Serial command handler for FRONTLIGHT
 * 
 */
void serial_frontlight() {
  Serial.println("Received FRONTLIGHT");
  serial_led_fade(led_front);
}

/**
 * @brief Serial command handler for MOTOR_V
 * 
 */
void serial_motor_v() {
  Serial.println("Received MOTOR_V");
  serial_motor(vert_up, vert_down, vert_pos, VERT_END_OUTER);
  serial_received();
}

/**
 * @brief Serial command handler for MOTOR_H
 * 
 */
void serial_motor_h() {
  Serial.println("Received MOTOR_H");
  serial_motor(horz_left, horz_right, hor_pos, HORZ_END_INNER);
  serial_received();
}

/**
 * @brief Serial command handler for RECORD
 * 
 */
void serial_record() {
  uint32_t timeout = ssp.readUnsignedInt32();
  ssp.readEot();
  Serial.printf("Received RECORD, timeout %d\n", timeout);

  elapsedMillis time = 0;

  digitalWrite(BTN_LED_RED, HIGH);
  blink_status = true;
  int led[] = { BTN_LED_RED };

  while (time < timeout) {
    btn_red.update();

    if (btn_red.isPressed())
      break;
    
    // Blink LED for the last 5 seconds
    if ((timeout - time) < REC_COUNTDOWN_MS) {
      blink(led, UI_ON_MS, UI_OFF_MS);
    }
  }
  digitalWrite(BTN_LED_RED, LOW);
  
  serial_received();
}

/**
 * @brief Serial command handler for REWIND
 * 
 */
void serial_rewind() {
  ssp.readEot();
  Serial.println("Received REWIND");
  zero_motor(vert_up, vert_down, VERT_END_INNER, VERT_END_OUTER);
  zero_motor(horz_left, horz_right, HORZ_END_OUTER, HORZ_END_INNER);
  serial_received();
}

/**
 * @brief Serial command handler for USER_INTERACT
 * 
 */
void serial_userinteract() {
  char bt = ssp.readByte();
  uint32_t timeout = ssp.readUnsignedInt32();
  ssp.readEot();
  Serial.println("Received USER_INTERACT");
  
  int enabled_btns[] = {
    (bt & 0x1),         // blue button
    (bt & 0x2) >> 1,    // red button
    (bt & 0x4) >> 2,    // yellow button
    (bt & 0x8) >> 3     // green button
  };

  // u_int8_t blue = (bt & 0x1);
  // u_int8_t red = (bt & 0x2) >> 1;
  // u_int8_t yellow = (bt & 0x4) >> 2;
  // u_int8_t green = (bt & 0x8) >> 3;

  Serial.printf("Blink byte: B=%d, R=%d, Y=%d, G=%d; Timeout=%d\n", enabled_btns[0], enabled_btns[1], enabled_btns[2], enabled_btns[3], timeout);
  
  const u_int8_t n_leds = enabled_btns[0] + enabled_btns[1] + enabled_btns[2] + enabled_btns[3];
  int leds[] = {
    enabled_btns[0] ? BTN_LED_BLUE : NC_PIN,
    enabled_btns[1] ? BTN_LED_RED : NC_PIN,
    enabled_btns[2] ? BTN_LED_YELLOW : NC_PIN,
    enabled_btns[3] ? BTN_LED_GREEN : NC_PIN
  };
  
  for (const int &led : leds) {
    digitalWrite(led, LOW);
  }
  blink_status = false;
  
  uint8_t btn_pressed = 0;
  elapsedMillis t = 0;
  while ((timeout == 0) || (t < timeout))
  {
    blink(leds, UI_ON_MS, UI_OFF_MS);

    if (enabled_btns[0]) btn_blue.update();
    if (enabled_btns[1]) btn_red.update();
    if (enabled_btns[2]) btn_yellow.update();
    if (enabled_btns[3]) btn_green.update();

    btn_pressed = (enabled_btns[0] && btn_blue.pressed() ? 0x1 : 0) |
                  (enabled_btns[1] && btn_red.pressed() ? 0x2 : 0) |
                  (enabled_btns[2] && btn_yellow.pressed() ? 0x4 : 0) |
                  (enabled_btns[3] && btn_green.pressed() ? 0x8 : 0);
    
    if (btn_pressed > 0) break;
  }

  for (const int &led : leds) {
    digitalWrite(led, LOW);
  }
  blink_status = false;
  
  serial_received(btn_pressed);
}

/**
 * @brief Serial command handler for debugging scroll positions. Logs to USB Serial
 * 
 */
void serial_debug_pos() {
  ssp.readEot();
  u_int32_t hlt = hor_lastchange;
  u_int32_t vlt = vert_lastchange;
  Serial.printf("Scroll positions (last change): H=%d (%d), V=%d (%d) \n", hor_pos, hlt, vert_pos, vlt);
  serial_received();
}

/**
 * @brief Serial command handler for debugging sensors. Logs to USB Serial
 * 
 */
void serial_debug_sens() {
  ssp.readEot();
  Serial.printf("VERT_END_OUTER = %d \nVERT_END_INNER = %d \nVERT_CNT_OUTER = %d \nVERT_CNT_INNER = %d \nHORZ_END_OUTER = %d \nHORZ_END_INNER = %d \nHORZ_CNT_INNER = %d \nHORZ_CNT_OUTER = %d \n",
        digitalRead(VERT_END_OUTER),
        digitalRead(VERT_END_INNER),
        digitalRead(VERT_CNT_OUTER),
        digitalRead(VERT_CNT_INNER),
        digitalRead(HORZ_END_OUTER),
        digitalRead(HORZ_END_INNER),
        digitalRead(HORZ_CNT_INNER),
        digitalRead(HORZ_CNT_OUTER));
  serial_received();
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  // initialize LEDs
  for (const int &led : UI_LED_PINS) {
    pinMode(led, OUTPUT);
    digitalWrite(led, LOW);
  }

  btn_blue.attach(BTN_BLUE, INPUT_PULLUP);
  btn_red.attach(BTN_RED, INPUT_PULLUP);
  btn_yellow.attach(BTN_YELLOW, INPUT_PULLUP);
  btn_green.attach(BTN_GREEN, INPUT_PULLUP);

  btn_blue.interval(5);
  btn_red.interval(5);
  btn_yellow.interval(5);
  btn_green.interval(5);

  btn_blue.setPressedState(LOW);
  btn_red.setPressedState(LOW);
  btn_yellow.setPressedState(LOW);
  btn_green.setPressedState(LOW);

  hor_pos = 0;
  vert_pos = 0;
  hor_lastchange = 0;
  vert_lastchange = 0;

  blink_time = 0;
  blink_status = false;

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
  pinMode(HORZ_END_INNER, INPUT);
  pinMode(HORZ_END_OUTER, INPUT);
  pinMode(VERT_END_INNER, INPUT);
  pinMode(VERT_END_OUTER, INPUT);

  digitalWrite(HORZ_CNT_INNER, LOW);
  digitalWrite(HORZ_CNT_OUTER, LOW);
  digitalWrite(VERT_CNT_INNER, LOW);
  digitalWrite(VERT_CNT_OUTER, LOW);
  digitalWrite(HORZ_END_INNER, LOW);
  digitalWrite(HORZ_END_OUTER, LOW);
  digitalWrite(VERT_END_INNER, LOW);
  digitalWrite(VERT_END_OUTER, LOW);

  led_back.begin();
  led_back.show();

  led_front.begin();
  led_front.show();

  ssp.init();
  ssp.registerCommand(HELLO, serial_hello);
  ssp.registerCommand(ALREADY_CONNECTED, serial_hello);
  ssp.registerCommand(BACKLIGHT, serial_backlight);
  ssp.registerCommand(FRONTLIGHT, serial_frontlight);
  ssp.registerCommand(MOTOR_H, serial_motor_h);
  ssp.registerCommand(MOTOR_V, serial_motor_v);
  ssp.registerCommand(RECORD, serial_record);
  ssp.registerCommand(REWIND, serial_rewind);
  ssp.registerCommand(DEBUG_SCROLL, serial_debug_pos);
  ssp.registerCommand(DEBUG_SENSORS, serial_debug_sens);
  ssp.registerCommand(USER_INTERACT, serial_userinteract);

  S00->addTransition(transition_post_zero, S10);
  S10->addTransition(transition_zero_init, S20);
  S20->addTransition(transition_init_wait, S30);
  S30->addTransition(transition_wait_sercom, S40);
}

void loop() {
  // Just run the state machine
  sm.run();
}


void state_post() {
  if (sm.executeOnce) {
    for (const int &led : UI_LED_PINS) {
      digitalWrite(led, HIGH);
      delay(POST_LED_ON_MS);
      digitalWrite(led, LOW);
    }
  }
  Serial.println("State POST.");
}


void state_zero() {
  Serial.println("State Zeroing.");

  zero_motor(vert_up, vert_down, VERT_END_INNER, VERT_END_OUTER);
  zero_motor(horz_left, horz_right, HORZ_END_OUTER, HORZ_END_INNER);
}


void state_init_callbacks() {
  Serial.println("State Initialize Callbacks.");
  attachInterrupt(digitalPinToInterrupt(HORZ_CNT_INNER), hor_count, CHANGE);
  attachInterrupt(digitalPinToInterrupt(VERT_CNT_INNER), vert_count, CHANGE);
}


void state_wait_serial() {
  if (sm.executeOnce) {
    serial_connected = false;
    handshake_complete = false;

    Serial.println("State Waiting for Serial Handshake.");
    digitalWrite(LED_BUILTIN, LOW);
    blink_status = false;      
  }

  int led[] = {LED_BUILTIN};
  blink(led, WAIT_ON_MS, WAIT_OFF_MS);
  ssp.loop();
}


void state_serial_com() {
  if (sm.executeOnce) {
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("State Serial Communication.");
  }

  ssp.loop();
}


void state_error() {
  if (sm.executeOnce) {
    Serial.println("State Error.");
    blink_status = digitalRead(LED_BUILTIN);  
  }

  int led[] = { LED_BUILTIN };
  blink(led, ERROR_ON_MS, ERROR_OFF_MS);
}


bool transition_post_zero() {
  return true;
}


bool transition_zero_init() {
  return true;
}


bool transition_init_wait() {
  return true;
}


bool transition_wait_sercom() {
  return handshake_complete;
}