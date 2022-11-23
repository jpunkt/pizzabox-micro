#include <Arduino.h>
#include <Bounce2.h>
#include <Adafruit_NeoPixel.h>
#include <SimpleSerialProtocol.h>
#include <StateMachine.h>
#include "Config.h"
#include "Commands.h"
#include "States.h"
#include "Motor.h"
#include "ColorHelpers.h"


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
const int UI_LED_PINS[] = {LED_BUILTIN, BTN_LED_BLUE, BTN_LED_RED, BTN_LED_GREEN, BTN_LED_YELLOW, MON_LED01, MON_LED02};
#define N_LEDS (sizeof(UI_LED_PINS) / sizeof(UI_LED_PINS[0]))

// Statemachine booleans
bool handshake_complete;

// Expect HELO1 to go LOW for reset
bool reset_expected;

// Last change on position counters in millis
elapsedMillis pos_lastchange[N_SCROLLS] = {};

enum Scroll_Idx{
  TARGET,
  SPEED
};

// Preset scroll values
int16_t scroll_targets[N_SCROLLS][2] = {0};

enum Light_Values_Idx {
  C,
  T
};

// Preset light values
// light_values[n][0] is color as uint32_t
// light_values[n][1] is fade time
uint32_t light_values[N_LIGHTS][2] = {
  {0, 0},
  {0, 0}
};

/*-------- Objects --------*/
// Scrolls
Motor scrolls[N_SCROLLS][2] = {
  {
    Motor(HORZ_LEFT_PWM, HORZ_LEFT_AIN1, HORZ_LEFT_AIN2),
    Motor(HORZ_RIGHT_PWM, HORZ_RIGHT_AIN1, HORZ_RIGHT_AIN2),  
  },
  {
    Motor(VERT_UP_PWM, VERT_UP_AIN1, VERT_UP_AIN2),
    Motor(VERT_DOWN_PWM, VERT_DOWN_AIN1, VERT_DOWN_AIN2)
  }
};

enum Scroll_Pin_Idx {
  END_OUTER,
  END_INNER,
  COUNT_OUTER,
  COUNT_INNER,
};

uint8_t scroll_pins[N_SCROLLS][4] = {
  {HORZ_END_OUTER, HORZ_END_INNER, HORZ_CNT_INNER, HORZ_CNT_OUTER},
  {VERT_END_INNER, VERT_END_OUTER, VERT_CNT_OUTER, VERT_CNT_INNER}
};

// LEDs
Adafruit_NeoPixel lights[N_LIGHTS] = {
  Adafruit_NeoPixel(LED_COUNT_BACK, LED_BACK, NEO_GBRW + NEO_KHZ800),
  Adafruit_NeoPixel(LED_COUNT_FRONT, LED_FRONT, NEO_GBRW + NEO_KHZ800)
};

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

// serial command ABORT was sent
volatile bool serial_aborted;

// inintialize hardware constants
const long BAUDRATE = 115200; // speed of serial connection
const long CHARACTER_TIMEOUT = 500; // wait max 500 ms between single chars to be received

// Create instance. Pass Serial instance. Define command-id-range within Simple Serial Protocol is listening (here: a - z)
SimpleSerialProtocol ssp(Serial1, BAUDRATE, CHARACTER_TIMEOUT, serial_on_error, 0, 100);

// Heartbeat blinker timer
elapsedMillis blink_time;
bool blink_status;             // boolean to hold led status (needed to let more than one led blink)

// Battery monitor timer
uint16_t batt_level;
elapsedMillis mon_time;

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
 * @brief Measure battery voltage and turn on appropriate LEDs
 * 
 */
void measure_batt() {
  if (mon_time > MON_INTERVAL) {
    batt_level = (batt_level + analogRead(PIN_BATT)) / 2;
    // Serial.printf("Battery level %d\n", batt_level);
    if (batt_level > BATT_HIGH) {
      digitalWrite(MON_LED02, HIGH);
      digitalWrite(MON_LED01, LOW);
    } else if ((batt_level > BATT_ERR) && (batt_level < BATT_LOW)) {
      digitalWrite(MON_LED01, HIGH);
      digitalWrite(MON_LED02, LOW);
    } else if (batt_level <= BATT_ERR) {
      digitalWrite(MON_LED01, !digitalRead(MON_LED01));
      digitalWrite(MON_LED02, !digitalRead(MON_LED01));
    } else {
      digitalWrite(MON_LED01, HIGH);
      digitalWrite(MON_LED02, HIGH);
    }
    mon_time = 0;
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
 * @param count_pin  Sensor pin for image stops
 */
void zero_motor(Motor &mot1, Motor &mot2, int zero_pin, int end_pin, int count_pin) {
  elapsedMillis time = 0;
  if (!digitalRead(zero_pin)) {
    // rewind
    while (!digitalRead(zero_pin) && time < MOTOR_TIMEOUT) {
      if (digitalRead(count_pin)) {
        time = 0;
      }
      mot2.run(255, false);
      mot1.run(127, false);
    }
    if (time >= MOTOR_TIMEOUT) {
      Serial.printf("Motor timeout reached\n");
    }
    mot1.stop(false);
    delay(200);
    mot2.stop(false);
    delay(100);
    mot1.stop(true);
    mot2.stop(true);
  } else if (digitalRead(end_pin)) {
    // move forward
    while(digitalRead(zero_pin) && time < MOTOR_TIMEOUT) {
      if (digitalRead(count_pin)) {
        time = 0;
      }
      mot1.run(200, true);
      mot2.run(100, true);
    }
    if (time >= MOTOR_TIMEOUT) {
      Serial.printf("Motor timeout reached\n");
    }
    mot2.stop(false);
    delay(200);
    mot1.stop(false);
    delay(100);
    mot1.stop(true);
    mot2.stop(true);
  }
}

void zero_scrolls() {
  for (int i=0; i < N_SCROLLS; i++) {
    zero_motor(scrolls[i][0], scrolls[i][1], scroll_pins[i][END_OUTER], scroll_pins[i][END_INNER], scroll_pins[i][COUNT_INNER]);
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
 * @brief Send RECEIVED + EOT bytes over Serial
 * 
 */
void serial_received() {
  ssp.writeCommand(RECEIVED);
  ssp.writeEot();
}

/**
 * @brief Send RECEIVED + <response byte> + EOT bytes over Serial
 * 
 */
void serial_received(uint8_t response) {
  ssp.writeCommand(RECEIVED);
  ssp.writeUnsignedInt8(response);
  ssp.writeEot();
}

/**
 * @brief Generic stop motors
 * 
 * @param mot1 
 * @param mot2 
 */
void stop_scroll(Motor &mot1, Motor &mot2) {
  mot1.stop(true);
  mot2.stop(true);
  Serial.printf("Motor stopped. \n");
}

/**
 * @brief Run motors in appropriate direction until picture stop is reached
 * 
 * outer_state | inner_state | outer_stop | inner_stop | direction | startup | MOTORS 
 *         1/0 |         1/0 |        1/0 |        1/0 |       1/0 |       1 | run fullspeed in direction
 *           1 |           0 |          0 |          0 |         1 |       0 | run fullspeed forwards
 *           0 |           1 |          0 |          0 |         1 |       0 | run halfspeed forwards
 *           0 |           1 |          0 |          0 |         0 |       0 | run fullspeed backwards
 *           1 |           0 |          0 |          0 |         0 |       0 | run halfspeed backwards
 *           1 |           1 |          0 |          0 |       1/0 |       0 | stop
 * 
 * @param mot1 
 * @param mot2 
 * @param pins      Array of sensor pins for the scroll 
 * @param direction 
 * @param speed 
 * @param startup   True if elapsed time is shorter than OVERRIDE_ENDSTOPS
 * @return true  when motors are still running
 * @return false if motors stopped
 */
bool mot_advance(Motor &mot1, Motor &mot2, 
                 uint8_t pins[],
                 bool direction, int16_t speed, bool startup) {
  uint8_t speed1 = (speed * 64) - 1;
  uint8_t speed2 = (speed * 24) - 1;

  bool outer_state = digitalRead(pins[COUNT_OUTER]);
  bool inner_state = digitalRead(pins[COUNT_INNER]);
  bool outer_stop = digitalRead(pins[END_OUTER]);
  bool inner_stop = digitalRead(pins[END_INNER]);

  if (!startup) {
    if ((direction && inner_stop) || (!direction && outer_stop)) {
      stop_scroll(mot1, mot2);
      return false;
    }
    if (outer_state && inner_state) {
      stop_scroll(mot1, mot2);
      return false;
    }
  } else {
    if ((direction && inner_state) || (!direction && outer_state)) {
      if (direction) {
        mot1.run(speed1, direction);
        mot2.stop(true);
      } else {
        mot2.run(speed1, direction);
        mot1.stop(true);
      }
    } else {
      if (direction) {
        mot1.run(speed1, direction);
        mot2.run(speed2, direction);
      } else {
        mot2.run(speed1, direction);
        mot1.run(speed2, direction);
      }
    }
  }
  return true;
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
 * @brief Serial command handler for SET_LIGHT
 * 
 */
void serial_set_light() {
  uint8_t n = ssp.readUnsignedInt8();

  light_values[n][0] = ssp.readUnsignedInt32();  // set color
  light_values[n][1] = ssp.readUnsignedInt32();  // set fade time

  ssp.readEot();

  Serial.printf("Set light[%d] to [%d, %d]\n", n, 
                light_values[n][0], 
                light_values[n][1]);

  serial_received();
}

/**
 * @brief Serial command handler for SET_SCROLL
 * 
 */
void serial_set_scroll() {
  uint8_t n = ssp.readUnsignedInt8();

  scroll_targets[n][TARGET] = ssp.readInt16();
  scroll_targets[n][SPEED] = ssp.readUnsignedInt8();

  ssp.readEot();

  Serial.printf("Set scroll[%d] to target %d at speed %d \n", n, scroll_targets[n][TARGET], scroll_targets[n][SPEED]);

  serial_received();
}

/**
 * @brief Serial command handler for DO_IT
 * 
 */
void serial_do_it() {
  ssp.readEot();

  Serial.printf("Received DO_IT \n");

  bool lights_fading = false;   // True if lights must be changed

  uint32_t from_colors[N_LIGHTS] = {};
  for (uint8_t i=0; i < N_LIGHTS; i++) {
    from_colors[i] = lights[i].getPixelColor(0);
    lights_fading = lights_fading || (from_colors[i] != light_values[i][C]);
  }

  bool scrolls_moving = false;   // True if motors must be moved

  for (uint8_t i=0; i < N_SCROLLS; i++) {
    scrolls_moving = scrolls_moving || (scroll_targets[i][TARGET] != 0);
    pos_lastchange[i] = 0;
  }

  elapsedMillis t = 0;

  while (!serial_aborted && (lights_fading || scrolls_moving)) {
    ssp.loop();

    /***** Fade Lights *****/
    bool fade = false;
    for (uint8_t i=0; i < N_LIGHTS; i++) {
      if (t < light_values[i][T]) {
        float_t perc = (float) t / (float) light_values[i][T];
        lights[i].fill(color_value(from_colors[i], light_values[i][C], perc));
        lights[i].show();
        fade = fade || true;
      } else {
        fade = fade || false;
      }
    }

    /***** Move Motors *****/
    bool move = false;
    for (uint8_t i=0; i < N_SCROLLS; i++) {
      if (scroll_targets[i][TARGET] != 0) {
        bool direction = scroll_targets[i][TARGET] > 0;
      
        if (mot_advance(scrolls[i][0], scrolls[i][1], scroll_pins[i], direction, scroll_targets[i][SPEED], (pos_lastchange[i] < ENDSTOP_OVERRIDE))) {
          move = move || true;
        } else {
          scroll_targets[i][TARGET] -= direction ? 1 : -1;
          move = move || scroll_targets[i][TARGET] != 0;
          pos_lastchange[i] = 0;
        }
      }
    }

    lights_fading = fade;
    scrolls_moving = move;
  }

  if (serial_aborted) {
    Serial.println("Movement aborted");
    for (uint8_t i=0; i < N_SCROLLS; i++) {
      scrolls[i][0].stop(true);
      scrolls[i][1].stop(true);
    }
    for (uint8_t i=0; i < N_LIGHTS; i++) {
      lights[i].fill(0);
      lights[i].show();
    }
    serial_aborted = false;
    return;
  }
  
  for (int i=0; i < N_LIGHTS; i++) {
    lights[i].fill(light_values[i][C]);
    lights[i].show();
    light_values[i][T] = 0;
  }

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

  while (!serial_aborted && (time < timeout)) {
    btn_red.update();
    ssp.loop();

    if (btn_red.pressed())
      break;
    
    // Blink LED for the last 5 seconds
    if ((timeout - time) < REC_COUNTDOWN_MS) {
      blink(led, UI_ON_MS, UI_OFF_MS);
    }
  }
  digitalWrite(BTN_LED_RED, LOW);

  if (serial_aborted) {
    serial_aborted = false;
    return;
  }
  
  serial_received();
}

/**
 * @brief Serial command handler for REWIND
 * 
 */
void serial_rewind() {
  ssp.readEot();
  Serial.println("Received REWIND");

  zero_scrolls();
  
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

  Serial.printf("Blink byte: B=%d, R=%d, Y=%d, G=%d; Timeout=%d\n", enabled_btns[0], enabled_btns[1], enabled_btns[2], enabled_btns[3], timeout);
  
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
  while (!serial_aborted && ((timeout == 0) || (t < timeout)))
  {
    blink(leds, UI_ON_MS, UI_OFF_MS);
    ssp.loop();

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

  if (serial_aborted) {
    serial_aborted = false;
    return;
  }
  
  serial_received(btn_pressed);
}

void serial_abort() {
  ssp.readEot();
  Serial.println("Received ABORT.");

  serial_aborted = true;

  serial_received();
}

void serial_reset() {
  ssp.readEot();
  Serial.println("Received RESET.");

  reset_expected = true;

  serial_received();
}

/**
 * @brief Serial command handler for debugging scroll positions. Logs to USB Serial
 * 
 */
void serial_debug_pos() {
  ssp.readEot();
  u_int32_t hlt = pos_lastchange[HORIZONTAL];
  u_int32_t vlt = pos_lastchange[VERTICAL];
  Serial.printf("Scroll targets / last change: H: %d / %d, V: %d / %d \n", scroll_targets[HORIZONTAL][TARGET], hlt, scroll_targets[VERTICAL][TARGET], vlt);
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

  pinMode(PIN_HELO1, INPUT);
  pinMode(PIN_HELO2, OUTPUT);

  digitalWrite(PIN_HELO2, LOW);

  // Battery monitor
  batt_level = (BATT_HIGH + BATT_LOW) / 2;

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
  
  blink_time = 0;
  blink_status = false;

  // Initialize motors and scroll positions
  for (int i=0; i < N_SCROLLS; i++) {
    for (int j=0; j < 2; j++) {
      scrolls[i][j].setup();
      scrolls[i][j].stop(true);
    }
    pos_lastchange[i] = 0;
  }

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

  for (int i=0; i < N_LIGHTS; i++) {
    lights[i].begin();
    lights[i].show();
  }

  ssp.init();
  ssp.registerCommand(HELLO, serial_hello);
  ssp.registerCommand(ALREADY_CONNECTED, serial_hello);
  ssp.registerCommand(ABORT, serial_abort);
  ssp.registerCommand(SET_LIGHT, serial_set_light);
  ssp.registerCommand(SET_MOVEMENT, serial_set_scroll);
  ssp.registerCommand(DO_IT, serial_do_it);
  ssp.registerCommand(RECORD, serial_record);
  ssp.registerCommand(REWIND, serial_rewind);
  ssp.registerCommand(DEBUG_SCROLL, serial_debug_pos);
  ssp.registerCommand(DEBUG_SENSORS, serial_debug_sens);
  ssp.registerCommand(USER_INTERACT, serial_userinteract);
  ssp.registerCommand(RESET, serial_reset);

  S00->addTransition(transition_post_zero, S10);
  S10->addTransition(transition_zero_init, S20);
  S20->addTransition(transition_init_wait, S30);
  S30->addTransition(transition_wait_sercom, S40);

  serial_aborted = false;
}

void loop() {
  // Just run the state machine
  sm.run();
}


void check_helo() {
  if (handshake_complete && (digitalRead(PIN_HELO1) == LOW)) {
    if (reset_expected) {
      // Expected PIN_HELO1 to go low. Transition to state wait for serial
      sm.transitionTo(S30);
      reset_expected = false;
      Serial.println("Raspi sent reset. Transition to wait for serial");
    } else {
      // Unexpected connection interrupt. Transition to state Error
      sm.transitionTo(SER);
      Serial.println("Lost HELO1. Transition to error state.");
    }

    for (int i=0; i < N_LIGHTS; i++) {
      light_values[i][C] = 0;
      light_values[i][T] = 0;
    }

    for (int i=0; i < N_SCROLLS; i++) {
      scroll_targets[i][TARGET] = 0;
      scroll_targets[i][SPEED] = 0;
    }
  }
}


void state_post() {
  if (sm.executeOnce) {
    Serial.println("State POST.");
    for (const int &led : UI_LED_PINS) {
      digitalWrite(led, HIGH);
      delay(POST_LED_ON_MS);
      digitalWrite(led, LOW);
    }
  }
}


void state_zero() {
  if (sm.executeOnce) {
    Serial.println("State Zeroing.");

    if (btn_red.isPressed()) {
      zero_scrolls();
    }
  }
}


void state_init_callbacks() {
  if (sm.executeOnce) {
    Serial.println("State Initialize Callbacks.");

    // attachInterrupt(digitalPinToInterrupt(scroll_pins[HORIZONTAL][COUNT_INNER]), hor_count, CHANGE);
    // attachInterrupt(digitalPinToInterrupt(scroll_pins[VERTICAL][COUNT_INNER]), vert_count, CHANGE);
    
    digitalWrite(LED_BUILTIN, LOW);
    blink_status = false; 
  }

  int led[] = {LED_BUILTIN};
  blink(led, WAIT_ON_MS, WAIT_OFF_MS);
  measure_batt();
}


void state_wait_serial() {
  if (sm.executeOnce) {
    Serial.println("State Waiting for Serial Handshake.");
    Serial.printf("Battery level: %d\n", batt_level);
    serial_connected = false;
    handshake_complete = false;

    digitalWrite(LED_BUILTIN, LOW);
    blink_status = false;      
  }

  int led[] = {LED_BUILTIN};
  blink(led, WAIT_ON_MS, WAIT_OFF_MS);
  measure_batt();
  ssp.loop();
}


void state_serial_com() {
  if (sm.executeOnce) {
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(MON_LED01, LOW);
    digitalWrite(MON_LED02, LOW);
    Serial.println("State Serial Communication.");
  }

  ssp.loop();
  check_helo();
}


void state_error() {
  if (sm.executeOnce) {
    Serial.println("State Error.");
    blink_status = digitalRead(LED_BUILTIN);
    digitalWrite(PIN_HELO2, LOW);
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
  if (digitalRead(PIN_HELO1)) {
    digitalWrite(PIN_HELO2, HIGH);
    return true;
  }
  return false;
}


bool transition_wait_sercom() {
  return handshake_complete;
}