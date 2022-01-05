enum Command {
  HELLO = 0,
  ALREADY_CONNECTED = 1,
  ERROR = 2,
  RECEIVED = 3,

  MOTOR_H = 'H',
  MOTOR_V = 'V',

  BACKLIGHT = 'B',
  FRONTLIGHT = 'F',

  USER_INTERACT = 'U',

  RESP_BLUE = 'X',
  RESP_RED = 'O',
  RESP_YELLOW = 'Y',
  RESP_GREEN = 'N',
  
  RECORD = 'C',
  REWIND = 'R',
  EOT = '\n'
};

typedef enum Command Command;