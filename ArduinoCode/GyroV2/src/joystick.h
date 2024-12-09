#ifndef _JOYSTICK_H
#define _JOYSTICK_H
#include "debug.h"
typedef struct
{
  double x;
  double y;
  double shiftx;
  double shifty;
  double rotation;
} JoystickPosition;

extern JoystickPosition read_joystick_position(bool debug);
extern bool parseJoystickData(String data, double &js1, double &js2, bool debug); // for testing
#endif