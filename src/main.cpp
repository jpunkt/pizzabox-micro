#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <SimpleSerialProtocol.h>
#include <StateMachine.h>
#include "Commands.h"
#include "Motor.h"
// #include "Serial_Comm.h"

/*-------- Pin definitions --------*/

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

/*-------- State Definitions --------*/

// States - implementations below loop()
void state_post();
void state_zero();
void state_init_callbacks();
void state_wait_serial();
void state_serial_com();
void state_error();

// Transitions - implementations below loop()
bool transition_post_zero();
bool transition_zero_init();
bool transition_init_wait();
bool transition_wait_sercom();

// Statemachine setup
StateMachine sm = StateMachine();

State* S00 = sm.addState(&state_post);
State* S10 = sm.addState(&state_zero);
State* S20 = sm.addState(&state_init_callbacks);
State* S30 = sm.addState(&state_wait_serial);
State* S40 = sm.addState(&state_serial_com);
State* SER = sm.addState(&state_error);

// Heartbeat blink interval constants
#define WAIT_ON_MS 200      // Blink when waiting for Serial
#define WAIT_OFF_MS 1800

#define ERROR_ON_MS 1000    // Blink when in error state
#define ERROR_OFF_MS 500

/*-------- Variables --------*/
// Heartbeat blinker timer
elapsedMillis blink_time;

// Statemachine booleans
bool handshake_complete;

// Position counters
volatile int16_t hor_pos;
volatile int16_t vert_pos;

/*-------- Objects --------*/
// Motors
Motor vert_up(VERT_UP_PWM, VERT_UP_AIN1, VERT_UP_AIN2);
Motor vert_down(VERT_DOWN_PWM, VERT_DOWN_AIN1, VERT_DOWN_AIN2);
Motor horz_left(HORZ_LEFT_PWM, HORZ_LEFT_AIN1, HORZ_LEFT_AIN2);
Motor horz_right(HORZ_RIGHT_PWM, HORZ_RIGHT_AIN1, HORZ_RIGHT_AIN2);

// LEDs
Adafruit_NeoPixel led_front(LED_COUNT_FRONT, LED_FRONT, NEO_GBRW + NEO_KHZ800);
Adafruit_NeoPixel led_back(LED_COUNT_BACK, LED_BACK, NEO_GBRW + NEO_KHZ800);

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
  // TODO enable: if (!digitalRead(HORZ_END_OUTER))
  hor_pos -= count(HORZ_CNT_INNER, HORZ_CNT_OUTER);
}

/**
 * @brief Callback for vertical counting
 * 
 */
void vert_count() {
  // TODO enable: if (!digitalRead(VERT_END_OUTER))
  vert_pos -= count(VERT_CNT_INNER, VERT_CNT_OUTER);
}

/**
 * @brief Blink the internal LED with defined on- and off- times. Call in loop to blink.
 * 
 * @param on_interval  time LED stays on in millis 
 * @param off_interval time LED is off in millis
 */
void blink_builtin(uint32_t on_interval, uint32_t off_interval) {
  if (digitalRead(LED_BUILTIN)) {
    if (blink_time >= on_interval) {
      digitalWrite(LED_BUILTIN, LOW);
      blink_time = blink_time - on_interval;
      // led_on = false;
      // Serial.println("Turn LED off");
    }
  } else {
    if (blink_time >= off_interval) {
      digitalWrite(LED_BUILTIN, HIGH);
      blink_time = blink_time - off_interval;
      // led_on = true;
      // Serial.println("Turn LED on");
    }
  }
}

/**
 * @brief Generic scroll zeroing code
 * 
 * @param mot1       Motor in positive direction
 * @param mot2       Motor in negative direction
 * @param zero_pin   Sensor pin where LOW enables count
 * @param end_pin    Sensor pin attached to emergency stop (end-stop)
 */
void zero_motor(Motor &mot1, Motor &mot2, int zero_pin, int end_pin) {
  bool is_zero = digitalRead(end_pin) & !digitalRead(zero_pin);
  uint32_t end_time = millis() + 10000;

  while (!is_zero & (millis() < end_time))
  {
    mot2.run(255, false);
    mot1.run(127, false);
    is_zero = digitalRead(zero_pin);
  }

  delayMicroseconds(20);

  end_time = millis() + 200;

  while (digitalRead(zero_pin) & (millis() < end_time))
  {
    mot1.run(200, true);
    mot2.stop(true);
  }
  
  mot1.stop(false);
  mot2.stop(false);
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

  uint32_t start_time = millis();
  uint32_t end_time = start_time + time_ms;

  while (millis() < end_time) {
    float_t perc = (float) (millis() - start_time) / (float) time_ms;
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
 * @brief Generic serial command handler to drive scroll to position
 * 
 * @param mot1  Motor in positive direction
 * @param mot2  Motor in negative direction
 * @param pos   position variable
 */
void serial_motor(Motor &mot1, Motor &mot2, volatile int16_t &pos) {
  int16_t inc = ssp.readInt16();
  ssp.readEot();

  int16_t aim = pos + inc;
  while (!mot_control(mot1, mot2, pos, aim)) {
    Serial.printf("aim = %i, pos = %i \n", aim, pos);
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
  serial_motor(vert_up, vert_down, vert_pos);
  serial_received();
}

/**
 * @brief Serial command handler for MOTOR_H
 * 
 */
void serial_motor_h() {
  Serial.println("Received MOTOR_V");
  serial_motor(horz_left, horz_right, hor_pos);
  serial_received();
}

/**
 * @brief Serial command handler for RECORD
 * 
 */
void serial_record() {
  ssp.readEot();
  Serial.println("Received RECORD");
  serial_received();
}

/**
 * @brief Serial command handler for REWIND
 * 
 */
void serial_rewind() {
  ssp.readEot();
  Serial.println("Received REWIND");
  serial_received();
}

/**
 * @brief Serial command handler for USER_INTERACT
 * 
 */
void serial_userinteract() {
  ssp.readEot();
  Serial.println("Received USER_INTERACT");
  serial_received();
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  hor_pos = 0;
  vert_pos = 0;

  blink_time = 0;

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
  ssp.registerCommand(USER_INTERACT, serial_userinteract);

  S00->addTransition(transition_post_zero, S10);
  S10->addTransition(transition_zero_init, S20);
  S20->addTransition(transition_init_wait, S30);
  S30->addTransition(transition_wait_sercom, S40);
}

void loop() {
  // Just run the state machine
  sm.run();
  // blink_builtin(WAIT_ON_MS, WAIT_OFF_MS);
}

/**
 * @brief State Power-On-Self-Test
 * 
 */
void state_post() {
  if (sm.executeOnce) {
    digitalWrite(LED_BUILTIN, HIGH);
  }
  Serial.println("State POST.");
}

/**
 * @brief State Zeroing motors
 * 
 */
void state_zero() {
  Serial.println("State Zeroing.");

  // zero_motor(vert_up, vert_down, VERT_END_INNER, VERT_END_OUTER);
  // zero_motor(horz_left, horz_right, HORZ_END_OUTER, HORZ_END_INNER);  // TODO check this
}

/**
 * @brief State Initialize callbacks (for counting)
 * 
 */
void state_init_callbacks() {
  Serial.println("State Initialize Callbacks.");
  attachInterrupt(digitalPinToInterrupt(HORZ_CNT_INNER), hor_count, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(HORZ_CNT_OUTER), hor_count, CHANGE);
  attachInterrupt(digitalPinToInterrupt(VERT_CNT_INNER), vert_count, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(VERT_CNT_OUTER), vert_count, CHANGE);
}

/**
 * @brief State wait for serial handshake
 * 
 */
void state_wait_serial() {
  if (sm.executeOnce) {
    serial_connected = false;
    handshake_complete = false;

    Serial.println("State Waiting for Serial Handshake.");
    digitalWrite(LED_BUILTIN, LOW);
  }

  blink_builtin(WAIT_ON_MS, WAIT_OFF_MS);
  ssp.loop();
}

/**
 * @brief State accept serial communications
 * 
 */
void state_serial_com() {
  if (sm.executeOnce) {
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("State Serial Communication.");
  }

  ssp.loop();
}

/**
 * @brief State an error occurred
 * 
 */
void state_error() {
  if (sm.executeOnce) {
    Serial.println("State Error.");
  }

  blink_builtin(ERROR_ON_MS, ERROR_OFF_MS);
}

/**
 * @brief Transition POST to zeroing. Always true.
 * 
 * @return true 
 */
bool transition_post_zero() {
  return true;
}

/**
 * @brief Transition zeroing to callback initialisation. Always true.
 * 
 * @return true 
 */
bool transition_zero_init() {
  return true;
}

/**
 * @brief Transition callback initialisation to wait for serial handshake. Always true.
 * 
 * @return true 
 */
bool transition_init_wait() {
  return true;
}

/**
 * @brief Transition serial handshake to serial communication. True when handshake complete.
 * 
 * @return true 
 * @return false 
 */
bool transition_wait_sercom() {
  // digitalWrite(LED_BUILTIN, LOW);
  return handshake_complete;
}