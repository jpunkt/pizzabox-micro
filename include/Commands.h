enum Command {
  HELLO = 0,
  ALREADY_CONNECTED = 1,
  ERROR = 2,
  RECEIVED = 3,

  MOTOR_H = 'H',
  MOTOR_V = 'M',

  BACKLIGHT = 'B',
  FRONTLIGHT = 'F',

  USER_INTERACT = 'U',
  
  RECORD = 'C',
  REWIND = 'R',
  EOT = '\n'
};

typedef enum Command Command;