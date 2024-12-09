#ifndef PIO_UNIT_TESTING
#include <Arduino.h>
#endif
#include "joystick.h"
#include "debug.h"
#include "debug.h"

// Setpoints
// Keep last position for errors...
double joystickX = 0.0;
double joystickY = 0.0;
const double DEADZONE = 0.1;

// Because Arduino abs is int only
double fabs(double x)
{
  return x < 0 ? -x : x;
}

#ifndef PIO_UNIT_TESTING

// bool parseJoystickData(std::string &data, double &js1, double &js2);

JoystickPosition read_joystick_position(bool debug)
{
  JoystickPosition position;
  String joystickData = Serial.readStringUntil('\n');
  if (parseJoystickData(joystickData, joystickX, joystickY, debug))
  {
    debug_printf("Joystick X: %.2f, Joystick Y: %.2f\n", joystickX, joystickY);
  }
  else
  {
    debug_printf("Error parsing joystick data.\n");
  }

  position.x = joystickX;
  position.y = joystickY;
  return position;
}
#endif

bool parseJoystickData(String data, double &js1, double &js2, bool debug)
{
  data.trim();

  int commaIndex = data.indexOf(',');
  if (commaIndex == -1)
  {
    js1 = joystickX;
    js2 = joystickY;
    return false; // No comma found
  }

  // Extract substrings for the two numbers
  String js1Str = data.substring(0, commaIndex);
  String js2Str = data.substring(commaIndex + 1);
  if (debug)
  {
    debug_printf("js1Str: %s, js2Str: %s\n", js1Str.c_str(), js2Str.c_str());
  }
  // Check if both parts are non-empty
  if (js1Str.length() == 0 || js2Str.length() == 0)
  {
    js1 = joystickX;
    js2 = joystickY;
    return false;
  }

  // Convert substrings to floats
  js1 = js1Str.toDouble();
  js2 = js2Str.toDouble();

  // and round anything < 0.1 to zero to remove jitter
  if (fabs(js1) < DEADZONE)
  {
    if (debug)
      debug_printf("js1 %f was in DEADZONE < %f\n", abs(js1), DEADZONE);
    js1 = 0.0;
  }
  if (fabs(js2) < DEADZONE)
  {
    if (debug)
      debug_printf("js2 %f was in DEADZONE < %f\n", abs(js2), DEADZONE);
    js2 = 0.0;
  }

  return true;
}
