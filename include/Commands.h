#ifndef COMMANDS_H
#define COMMANDS_H

enum Lights {
  BACKLIGHT = 0,
  FRONTLIGHT = 1
};

const int N_LIGHTS = 2;     // Set this to the number of lights. Used for iterating over array of lights


enum Scrolls {
  HORIZONTAL = 0,
  VERTICAL = 1
};

const int N_SCROLLS = 2;    // Set this to the number of scrolls. Used for iterating over array of scrolls.


enum Command {
  HELLO = 0,
  ALREADY_CONNECTED = 1,
  ERROR = 2,
  RECEIVED = 3,

  SET_MOVEMENT = 'M',
  SET_LIGHT = 'L',

  DO_IT = 'D',

  USER_INTERACT = 'U',
  RECORD = 'C',

  REWIND = 'R',

  DEBUG_SCROLL = 'S',
  DEBUG_SENSORS = 'Z',

  EOT = '\n'
};

typedef enum Command Command;
typedef enum Scrolls Scrolls;
typedef enum Lights Lights;

#endif