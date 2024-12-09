#include <stdio.h>
#include "src/joystick.h"
#include "src/brain.h"
extern float gx, gy, gz;

void createSensorFile()
{
  JoystickPosition joystick;
  joystick.x = 0;
  joystick.y = 0;

  SensorData sensorData;
  sensorData.ignoreSensorData = true;
  sensorData.accX = 0;
  sensorData.accY = 0;
  sensorData.accZ = 0;
  sensorData.gyrX = 0;
  sensorData.gyrY = 0;
  sensorData.gyrZ = 0;
  Movement m;
  printf("gx, gy, speedX, speedY, speedZ\n");
  for (gx = -25; gx < 25; gx += 2)
  {
    for (gy = -25; gy < 25; gy += 2)
    {
      m = brain_update(sensorData, joystick, true);
      printf("%.2f, %.2f, %.2f, %.2f, %.2f\n", gx, gy, m.speedX, m.speedY, m.speedZ);
    }
  }
}

int main()
{
  createSensorFile();
  return 0;
}