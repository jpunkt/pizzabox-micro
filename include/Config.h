#ifndef CONFIG_H
#define CONFIG_H
/*-------- Pin definitions --------*/

// Vertical motor top
#define VERT_UP_PWM      3
#define VERT_UP_AIN2     4
#define VERT_UP_AIN1     5

// Vertical motor bottom
#define VERT_DOWN_PWM    9
#define VERT_DOWN_AIN2   7
#define VERT_DOWN_AIN1   8

// Horizontal motor left
#define HORZ_LEFT_PWM   24
#define HORZ_LEFT_AIN2  25
#define HORZ_LEFT_AIN1  26

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
#define HORZ_CNT_INNER  36
#define HORZ_CNT_OUTER  35

// Lights
#define LED_FRONT       41
#define LED_COUNT_FRONT 26       // Number of LEDs in frontlight

#define LED_BACK        14
#define LED_COUNT_BACK  120      // Number of LEDs in backlight

// Buttons
#define BTN_LED_BLUE    23       // 21
#define BTN_BLUE        22       // 20
#define BTN_LED_RED     21       // 17
#define BTN_RED         20       // 16
#define BTN_LED_YELLOW  17       // 23
#define BTN_YELLOW      16       // 22
#define BTN_LED_GREEN   19       // 19
#define BTN_GREEN       18       // 18

// Handshake pins
#define PIN_HELO1       10
#define PIN_HELO2        2

// Battery Monitor
#define PIN_BATT        15       // Battery monitor (voltage divider) pin
#define MON_LED01       31       // Red notification LED
#define MON_LED02       32       // Green notification LED

const int32_t MON_INTERVAL = 500;  // interval for monitoring battery voltage (millis)
const int16_t BATT_HIGH = 900;     // voltage above this level is battery full
const int16_t BATT_LOW  = 820;     // voltage below this level is battery low
const int16_t BATT_ERR  = 700;     // voltage below this level is battery error

// NC (Blink-Sink)
#define NC_PIN          11

/*-------- Constants --------*/

const int ENDSTOP_OVERRIDE = 250;   // time to ignore endstop when motor starts (millis)
const int MOTOR_TIMEOUT = 5000;     // Timeout for zeroing in millis. Motor stops if no image stop was triggered in this interval

// Blink interval constants
#define WAIT_ON_MS        200       // Blink when waiting for Serial
#define WAIT_OFF_MS      1800

#define ERROR_ON_MS      1000       // Blink when in error state
#define ERROR_OFF_MS      500

#define POST_LED_ON_MS    500       // How long to turn on each LED at POST

#define REC_COUNTDOWN_MS 5000       // In Recording mode, blink LED for the last X milliseconds

#define UI_ON_MS          500       // General user interaction blink interval
#define UI_OFF_MS         500

#endif