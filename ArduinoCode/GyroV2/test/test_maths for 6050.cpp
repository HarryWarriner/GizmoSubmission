#if 0
#include <stdio.h>
#include <string>
#include <unity.h>
#include "brain.h"
// #include "mpu6050.h"
#include "mpu_bno.h"
#include "motors.h"
#include "joystick.h"
#include "debug.h"
void test_omni_drive();
extern float gx, gy, gz;
void setUp(void)
{
}

void tearDown(void)
{
}

void show_position(Movement movement)
{
  printf("MOVEMENT:: GXYZ: %.2f, %.2f, %.2f :: Speed XYZ: %.2f, %.2f, %.2f :: %lu, %lu, %lu\n",
         gx, gy, gz,
         movement.speedX, movement.speedY, movement.speedZ,
         speed_to_step_interval(movement.speedX),
         speed_to_step_interval(movement.speedY),
         speed_to_step_interval(movement.speedZ));
}

void test_joystick_parser()
{
  double js1, js2;
  TEST_ASSERT_TRUE_MESSAGE(parseJoystickData("0.1,0.2", js1, js2, true), "Failed to parse 0.1,0.2");
  TEST_ASSERT_EQUAL_FLOAT_MESSAGE(0.1, js1, "Failed to parse 0.1");
  TEST_ASSERT_EQUAL_FLOAT_MESSAGE(0.2, js2, "Failed to parse 0.2");

  TEST_ASSERT_TRUE_MESSAGE(parseJoystickData("0.05,0.2", js1, js2, true), "Failed to parse 0.05,0.2");
  TEST_ASSERT_EQUAL_FLOAT_MESSAGE(0.0, js1, "Failed to parse 0.0");
  TEST_ASSERT_EQUAL_FLOAT_MESSAGE(0.2, js2, "Failed to parse 0.2");
}

void test_incremental_accelerometer()
{
  printf("starting test_incremental_accelerometer\n");
  brain_init();
  JoystickPosition joystick;
  joystick.x = 0;
  joystick.y = 0;

  SensorData sensorData;
  sensorData.accX = 0;
  sensorData.accY = 0;
  sensorData.accZ = 0;
  sensorData.gyrX = 0;
  sensorData.gyrY = 0;
  sensorData.gyrZ = 0;
  Movement m;
  // 5 degrees/second/second acceleration along x axis
  sensorData.accX = 5;
  // sensorData.gyrX = 5;
  for (int i = 0; i < FREQ; i++)
  {
    m = brain_update(sensorData, joystick, true);
    show_position(m);
  }
}

void test_manual_change()
{
  printf("starting manual change\n");
  brain_init();
  JoystickPosition joystick;
  joystick.x = 0;
  joystick.y = 0;

  SensorData sensorData;
  sensorData.accX = 0;
  sensorData.accY = 0;
  sensorData.accZ = 0;
  sensorData.gyrX = 0;
  sensorData.gyrY = 0;
  sensorData.gyrZ = 0;
  Movement m;
  for (int i = 0; i < FREQ; i++)
  {
    m = brain_update(sensorData, joystick, true);
    show_position(m);
  }
}

void test_incremental_joystick()
{
  printf("Starting test_incremental_joystick\n");
  brain_init();
  JoystickPosition joystick;
  joystick.x = 0;
  joystick.y = 0;

  SensorData sensorData;
  sensorData.accX = 0;
  sensorData.accY = 0;
  sensorData.accZ = 0;
  sensorData.gyrX = 0;
  sensorData.gyrY = 0;
  sensorData.gyrZ = 0;
  Movement m;
  // 5 degrees/second/second acceleration along x axis
  m = brain_update(sensorData, joystick, true);
  show_position(m);
  printf("starting test...\n");
  //

  joystick.x = 50;
  // sensorData.gyrX = 5;
  for (int i = 0; i < FREQ; i++)
  {
    m = brain_update(sensorData, joystick, true);
    show_position(m);
  }
  joystick.x = -50;
  // sensorData.gyrX = 5;
  for (int i = 0; i < FREQ; i++)
  {
    m = brain_update(sensorData, joystick, true);
    show_position(m);
  }
  joystick.x = 0;
  joystick.y = -50;
  // sensorData.gyrX = 5;
  for (int i = 0; i < FREQ; i++)
  {
    m = brain_update(sensorData, joystick, true);
    show_position(m);
  }
  joystick.y = 50;
  // sensorData.gyrX = 5;
  for (int i = 0; i < FREQ; i++)
  {
    m = brain_update(sensorData, joystick, true);
    show_position(m);
  }
}

void test_ranges()
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
  for (gx = -50; gx < 50; gx += 10)
  {
    for (gy = -50; gy < 50; gy += 10)
    {
      m = brain_update(sensorData, joystick, true);
      printf("gx: %.2f, gy: %.2f, gz: %.2f, speedX: %.2f, speedY: %.2f, speedZ: %.2f\n", gx, gy, gz, m.speedX, m.speedY, m.speedZ);
      //

      double accX = 0.86602540 * m.speedX - 0.86602540 * m.speedY + 0 * m.speedZ;
      double accY = -0.5 * m.speedX - 0.5 * m.speedY + 1 * m.speedZ;
      double rot = 1 * m.speedX + 1 * m.speedY + 1 * m.speedZ;
      printf("accX: %.2f, accY: %.2f, rot: %.2f\n", accX, accY, rot);
      TEST_ASSERT_FLOAT_WITHIN(0.0005, 0, rot); //, "Rotation should be zero");
    }
  }
}

void test_sensorfusion()

{
  SensorData sensorData;
  sensorData.ignoreSensorData = true;
  sensorData.accX = 0;

  sensorData.accY = 0;
  sensorData.accZ = 0;
  sensorData.gyrX = 1; // move one degree/second in X axis
  sensorData.gyrY = 0;
  sensorData.gyrZ = 0;
  brain_init();
  for (int i = 0; i < FREQ; i++)
  {
    brain_imu_update(sensorData, true);
  }
  printf("gx: %.2f, gy: %.2f, gz: %.2f\n", gx, gy, gz);
  TEST_ASSERT_FLOAT_WITHIN(0.01, 1.0, gx); // after one second, GX should be 1
  TEST_ASSERT_FLOAT_WITHIN(0.01, 0.0, gy); // after one second, GY should be 1
  TEST_ASSERT_FLOAT_WITHIN(0.01, 0.0, gz); // after one second, GZ should be 1
  //
  // Movement vector to get us back is (X,Y)
  //
  // TODO:: HERE
}

int main()
{
  UNITY_BEGIN();
  // RUN_TEST(test_incremental_accelerometer);
  // RUN_TEST(test_incremental_joystick);
  // RUN_TEST(test_joystick_parser);
  // RUN_TEST(test_ranges);
  // RUN_TEST(test_manual_change);
  RUN_TEST(test_sensorfusion);
  // RUN_TEST(test_omni_drive);
  UNITY_END();
  return 0;
}
#endif